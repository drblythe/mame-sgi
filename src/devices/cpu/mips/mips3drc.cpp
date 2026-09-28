// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mips3drc.cpp

    Universal machine language-based MIPS III/IV emulator.

****************************************************************************

    Future improvements/changes:

    * Add DRC option to flush PC before calling memory handlers

    * Constant tracking? (hasn't bought us much in the past)

    * Customized mapped/unmapped memory handlers
        - create 3 sets of handlers: cached, uncached, general
        - default to general
        - in general case, if cached use RECALL to point to cached code
        - (same for uncached)
        - in cached/uncached case, fall back to general case

***************************************************************************/

// Marker for bug fixes to this shared/generic MAME core file (not SGI-
// specific, not an "optional accuracy" patch - fixes real crashes with no
// legitimate reason to disable). Self-defined here rather than via a
// per-subtarget build flag: this file compiles as part of the shared
// "optional" project (liboptional.a), not the SGI driver's own project,
// so a defines{} block in sgi.lua never reaches it. Kept as a distinct
// marker (not folded into a plain comment) for future discoverability/
// diffing if more invasive fixes to the shared DRC frontend
// (src/devices/cpu/drcfe.ipp, affecting every DRC-capable core) follow.
// Three-way comparison (2026-07-23) concluded: fix confirmed to eliminate
// the original crash cleanly (crash reproduces exactly when reverted,
// disappears when re-enabled). Re-enabled.
#define MAME_DRC_BUGFIXES

// Own sub-macro (2026-09-11) for the null-seqlast fallback's reporting
// only - the fallback itself stays unconditional under MAME_DRC_BUGFIXES
// above regardless of this. See the fallback's own comment in
// code_compile_block() and diag_null_seqlast_count's comment in mips3.h.
#define MAME_DRC_BUGFIXES_NULL_SEQLAST_DIAG

// TEMPORARY diagnostic marker (2026-07-23), separate from MAME_DRC_BUGFIXES
// above - self-defined here for the same reason: this file is its own
// translation unit, so a #define in mips3.cpp (where this marker's other
// half lives) doesn't reach here. Explains why every "compiling seqhead"
// diagnostic attempt in code_compile_block() below produced zero hits all
// night despite the loop it was trying to observe definitely executing -
// the guarded code was never compiled in at all, in any version tried.
// Remove alongside the rest of the MAME_DRC_DIAG instrumentation once the
// investigation this supports concludes.
//
// Disabled (2026-07-26): several of the probes gated on this (mutex_unlock,
// the mutex write-tap callback, mutex_wake, sv_wait*, biodone, xlog*) fire
// unconditionally on every occurrence of ordinary, frequent kernel events
// and were left over from earlier, now-closed investigations - confirmed
// directly as the reason a `-str 30` test run took over two real minutes
// to reach 30 emulated seconds (19000+ mutex_unlock events, three
// synchronous fprintf+fflush lines apiece, in well under that). Re-enable
// only the specific probe(s) actually needed if a future investigation
// requires one of these again, rather than the whole macro at once.
// #define MAME_DRC_DIAG

// EXPERIMENTAL guest-kernel workaround (2026-07-26): NOT a MAME/DRC bug fix -
// this patches around a genuine IRIX kernel defect (confirmed against real
// IRIX 6.5.7m source, irix/kern/os/ksync/mutex.c: sv_timedwait's
// `unlock_func = !mp ? NOLOCK : ospl ? SPINLOCK : MUTEX` treats a legitimately
// -zero splhi() return value as "release via mutex_unlock", when it's really
// just "interrupts were already fully enabled" for a plain spinlock caller).
// spinlock_init() unconditionally writes the literal constant 2 into every
// lock_t at init; that value is never read again by real splhi/splx-based
// lock/unlock on this uniprocessor kernel, but mutex_unlock's tagged-word
// convention happens to read "2" as "has waiters, no owner" and crashes on a
// nonexistent `next` pointer if this ambiguity is ever hit. Overriding the
// value spinlock_init actually writes, from 2 to 0, defangs that misread
// kernel-wide (mutex_unlock's own flag check sees 0, takes its harmless
// fast path) without touching the ambiguous dispatch logic itself or any
// call site's argument-passing. See wiki/assets.md's root-cause writeup.
#define MAME_IRIX_SPINLOCK_INIT_WORKAROUND

// TEMPORARY WORKAROUND (2026-07-27), NOT a real fix - see the comment at its
// actual use site (static_generate_memory_mode_checks's MODE_SUPER case) for
// the full mechanism. Widens what the supervisor-mode memory accessor
// tolerates so a real, unresolved DRC bug (a shared C function reached under
// the wrong cached-compiled-mode instance) stops fatally crashing boot,
// purely to unblock reaching the rmap_del investigation. Masks the real
// KERNEL FAULT bug rather than fixing it.
//
// UPDATE (same night, later): the root mechanism behind the *value* this
// masks (Status/EPC/$k0 all becoming elocore_exl_3's own address) was found
// and is fully understood - see wiki/assets.md's "KERNEL FAULT root
// mechanism found" writeup. It's a real eret-vs-pending-interrupt race
// (architecturally possible on real MIPS hardware too, not a DRC-only
// artifact), not an emulator bug to fix - so this stays a deliberate,
// documented mask rather than being resolved into a real fix.
#define MAME_IRIX_KERNEL_FAULT_WORKAROUND

// TEMPORARY WORKAROUND (2026-07-27, later the same night), NOT a real fix -
// masks the downstream symptom of the eret-vs-pending-interrupt race
// documented at MAME_IRIX_KERNEL_FAULT_WORKAROUND above and in wiki/assets.md.
// When elocore_exl_3's own mtc0 gets reached via its "direct-entry" compiled
// instance (no fresh andi/ori setup - i.e. resumed into directly via eret
// rather than fallthrough), $k0 can still hold a stale exception_exit-left
// value instead of the freshly-intended Status value. See the use site
// (generate_set_cop0_reg's COP0_Status case) for the actual substitution
// logic. Deliberately conservative: only triggers when the value about to be
// stored is obviously implausible as real Status (top byte 0x88, matching
// kernel .text - never a real Status bit pattern), and simply no-ops the
// store (keeps the prior, presumably-still-sane Status) rather than
// attempting to reconstruct "the value that should have been written".
// A real fix would need to prevent the race itself (or make elocore_exl_3's
// direct-entry path independent of $k0's incidental prior value), not
// silently drop a write - remove once that's pursued, if ever.
#define MAME_IRIX_ERET_RACE_WORKAROUND

// TEMPORARY diagnostic (2026-08-14): logs every COP0 exception delivery,
// ERET execution, and COP0_Status store while total_cycles() falls inside
// the eret-race onset window bisected via MAME_SGI_EARLY_SAVE_DIAG
// (mips3.cpp) - see task #66 / journal.md's 2026-08-14 entries for the
// bisection itself: a -state save/reload taken at 2,749,877 cycles comes
// back healthy, one taken at 2,829,794 cycles comes back permanently
// stuck at SR.EXL=1/Cause=0x00000034 (the console shows this window sits
// inside PROM's "Running power-on diagnostics..." phase, well before any
// keyboard-wait prompt or disk activity). Bounds below are set with margin
// on both sides of that ~80,000-cycle transition to capture context before
// and after it, not just inside it. Unlike the disabled
// MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES family below, this is NOT filtered
// by PC or by "looks like a kernel .text address" - that family's whole
// filtering approach assumed IRIX kernel code (elocore_exl_N) and kernel
// .text addresses, neither of which necessarily applies here since this
// window is still PROM firmware, not IRIX kernel code.
// REVISED AGAIN (2026-08-14, same day): a full clean boot with the window
// wide open [0, 5,000,000] produced ZERO hits (no exception, no eret, no
// COP0_Status store) - the FIRST such event of any kind, in a completely
// vanilla continuous boot, doesn't happen until total_cycles() ~206,488,664
// (confirmed via an even wider sanity pass, [0, 2,000,000,000] - see
// journal.md 2026-08-14). So neither the pre-save window nor immediately-
// post-restore can be where an exception/eret/status-store first causes
// divergence - there simply isn't one there to diverge on. Narrowed to
// bracket that ~206M onset point instead: reload from BOTH the confirmed-
// healthy (2,749,877-cycle, sta/indy_4610/healthy_endpoint.sta) and
// confirmed-stuck (2,829,794-cycle, stuck_endpoint.sta) save points and
// compare their event sequences starting right where real COP0 activity
// actually begins, rather than at either save point itself.
//
// SAFETY: this WILL runaway into an enormous log if left on for a long/
// unbounded run - a 452MB log in under 2 minutes was produced once already
// when accidentally left on with HI=2,000,000,000 during an unrelated
// detour. Keep LO/HI tight and exit the process as soon as the window
// closes; never leave this defined for a run without deliberately re-
// checking these bounds first.
// RESTORED (2026-08-17) - task #69/#71 concluded (real root cause was
// MIPS3DRC_STRICT_COP1 never being enabled, see drc-bugfixes.md), so
// this is back to its normal always-on state after being temporarily
// disabled for that investigation's stock-vs-modified A/B test.
#define MAME_SGI_ERET_RACE_WINDOW_DIAG
#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG
// WIDENED (2026-08-14, same day): the 206M-207.5M window only shows the
// tail-end repeating Trap loop, which is a SYMPTOM (EXL already stuck by
// the time these fire - MIPS architecture leaves EPC untouched on any
// exception taken while EXL=1, so none of those events are what actually
// set EPC/EXL in the first place). The real onset must be somewhere in the
// still-unexamined gap between the bisected save-point window
// (2,700,000-2,900,000, confirmed zero hits) and this tail region -
// widened to cover that whole gap on a RELOAD run specifically (a
// continuous boot already confirmed clean in this same range at the
// narrower [0,5,000,000] and [0,2,000,000,000] checks).
#define MAME_SGI_ERET_RACE_WINDOW_LO 2900000
#define MAME_SGI_ERET_RACE_WINDOW_HI 3800000
#endif

// DISABLED (2026-07-28): the andi_ori/mtcz_read/fault_store_mode probe
// family below was deliberately left ungated (not behind MAME_DRC_DIAG)
// during the eret-race investigation, on the assumption their narrow PC
// range (elocore_exl_3's own vicinity) made them cheap - that assumption
// didn't hold once boot got far enough to reach it during heavy, normal
// interrupt/kernel activity: 186,000+ synchronous fprintf+fflush calls
// in under a minute of real time on one run, the same performance
// problem already documented in wiki/console.md's "-str interacts badly
// with verbose diagnostics" section. Their investigation concluded
// (see wiki/assets.md's full eret-race writeup) - re-enable only if
// that specific mechanism needs re-examining again.
// #define MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES

// TEMPORARY diagnostic (2026-07-30): re-enabled separately from the disabled
// bucket above - cfunc_rmap_del_panic_probe was swept up in that same
// MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES disable for being ungated, but unlike
// its bucket-mates it only fires at two exact JAL call sites that are
// themselves the real IRIX panic call for "PANIC: rmap_del: pde 0x%x not in
// pfdat 0x%x" (rmap_delmap_nolock, irix/kern/os/rmap.c) - i.e. immediately
// before a kernel panic that halts the machine anyway. Not a hot path by
// construction; safe to leave on unconditionally. rmap_del's own root cause
// was never determined in this investigation (see wiki/assets.md) - this
// captures pfdp/pdep/caller at the actual panic site the next time it's hit.
// RESTORED (2026-08-17) - task #69/#71 concluded, see
// MAME_SGI_ERET_RACE_WINDOW_DIAG's own restore comment above.
#define MAME_SGI_RMAP_DEL_PANIC_DIAG

// TEMPORARY diagnostic (2026-08-01): validating a theory about USER-mode
// (real IRIX application) code recompilation during multi-process
// operation - see mips3.h's comment on m_diag_usermode_recompile_counts
// for the full mechanism/hypothesis. Counts recompile events per
// (mode, pc), USER mode only, dumped at clean exit from mips3.cpp's
// device_stop(). RESTORED (2026-08-17) - task #69/#71 concluded, see
// MAME_SGI_ERET_RACE_WINDOW_DIAG's own restore comment above - at the
// time judged "cheap... safe to leave on for a normal test run" on an
// amortized-cost basis (one hash-map insert per compile).
// DISABLED, THEN RE-ENABLED WITH A FIX (2026-09-17, task #97 QoS
// follow-up) - that "cheap" judgment used the wrong success metric for
// what this investigation actually needs. Amortized-O(1) insert cost is
// real in aggregate, but says nothing about the periodic O(n) rehash
// spikes that same unbounded growth causes - exactly the tail-latency
// question this diagnostic's own caller (execute_run()'s MAME_SGI_
// STALL_DURATION_DIAG) exists to catch, not something perf_ab_stats.sh's
// aggregate instructions/cycles view would ever surface. Caught directly
// via a gdb conditional breakpoint on mmap(~2.7MB) during a live boot:
// m_diag_usermode_recompile_counts (an unordered_map<uint64_t,uint32_t>,
// never cleared, growing for the whole session) rehashing to 351,061
// buckets was one of two real causes found for MAME_SGI_COMPILE_LENGTH_
// CORRELATION_DIAG's ~11-13ms "long compile" outliers - see wiki/
// performance.md's "Root cause found" subsection, describe_code()/
// histogram/gdb-backtrace follow-up, for the other (m_checkints_addrs,
// mips3.h - unconditional production code, not diagnostic-gated, spun
// off as its own task since it needs a real fix, not just a toggle).
// Re-enabled after adding a one-time reserve() at the insert site (see
// its own comment below) that converts the periodic rehash stalls into
// one predictable upfront allocation - verified via boot test that
// MAME_SGI_STALL_DURATION_DIAG's over_10ms_threshold stays at 0 with
// this diagnostic active again.
#define MAME_DRC_USERMODE_RECOMPILE_DIAG

// TEMPORARY diagnostic (2026-09-17, task #100 scoping) - see
// m_diag_checkints_all_compiled_pcs's comment in mips3.h. Answers whether
// m_checkints_addrs's per-PC sticky-set design is even the right shape:
// if most compiled PCs end up armed anyway, tracking membership precisely
// isn't buying much over just being closer to unconditional. Cheap
// (reserved up front, same fix already applied to m_checkints_addrs
// itself and to m_diag_usermode_recompile_counts this session) - safe to
// leave on for a normal test run while this scoping question is open.
#define MAME_SGI_CHECKINTS_ARMED_FRACTION_DIAG

// Originally a temporary diagnostic (2026-09-17, task #101 Phase 0) -
// see diag_entry_reentry_count's comment in mips3.h. Real wall-clock
// frequency of static_generate_entry_point()'s per-re-entry interrupt
// check and static_generate_out_of_cycles()'s exit, measured directly
// rather than assumed. Kept permanently active (cheap - pure in-memory
// counters, no per-event I/O below its 5ms outlier threshold) as the
// standing canary for MAME_SGI_DISABLE_CHECKINTS_HEURISTIC_EXPERIMENT
// below: any recurrence of a large re-entry gap at a call-stitched-
// across-blocks loop shape (the pattern the heuristic exists to catch)
// would show up here first.
#define MAME_SGI_TASK101_REENTRY_FREQ_DIAG

// See the full rationale (test evidence, caveats) at the use site
// (code_compile_block()'s checkints arming block) and in this project's
// own task #101 tracking. Confirmed live: disabling the entire dynamic
// checkints heuristic apparatus produced zero measurable interrupt-
// latency/drift regression against indirect_loop.s (the heuristic's own
// hardest case - an indirect-jump loop with no calls, so it gets none
// of the front-end's own call-boundary sequence-ending protection
// either), and an ordinary full boot produced no hang and nothing
// beyond the already-explained, unrelated perform_dma() host stall.
// One open gap: the original mrupdatef interrupt-starvation livelock
// this mechanism was built to fix could not be directly re-triggered -
// its code executes normally in a current boot, but the genuine multi-
// reader lock contention that caused the original 10-real-second stall
// doesn't occur under tested conditions, so this is "not yet observed
// to recur," not "proven safe for that specific historical case."
// Opt-in (kept toggleable, not made unconditional), currently enabled
// by deliberate choice while broader, less-directed guest testing
// (real applications, general system activity) continues -
// MAME_SGI_TASK101_REENTRY_FREQ_DIAG above is the passive canary
// watching for a recurrence.
#define MAME_SGI_DISABLE_CHECKINTS_HEURISTIC_EXPERIMENT

// TEMPORARY FIX, DISABLED BY DEFAULT (2026-08-01, refined 2026-08-02) -
// see the full comment at its use site, generate_update_mode()
// (mips3drc.cpp), and wiki/performance.md's "USER-mode cross-process
// cache thrashing" section. Widens the DRC dispatch key to include
// ASID, fixing cross-process cache collisions for private/writable code
// while leaving kernel code's ASID-immunity intact.
//
// Numeric toggle, not a plain #define, specifically so all three
// development states stay one line away during this still-in-progress
// investigation (per direct request 2026-08-02 - "make it easy to flip
// back... including all the way back to the non-ASID-aware version"):
//   (undefined)  original behavior, no ASID awareness at all
//   1            first-pass fix: ASID folded into EVERY non-kernel
//                dispatch unconditionally. Correct, but a real measured
//                regression - fragments genuinely shared code (the
//                interpreter, common libc routines) that the old
//                checksum-only design already handled correctly and
//                cheaply. 9.3x more distinct cache keys, 6.8x more
//                compile events, ~25x more full-cache flushes on a
//                clean before/after boot-to-login comparison. Kept only
//                as a reference/comparison point, not the intended
//                end state.
//   2            refined fix: ASID folded in only for pages confirmed
//                writable via a live VTLB check (m_core->mode used
//                unchanged for read-only/shared pages). Validated
//                against real IRIX kernel source ($IRIXSRC's
//                exec.c/execmap(): "non-writable mappings... set up as
//                MAP_SHARED", i.e. IRIX's own kernel already uses
//                exactly this signal to decide what's safe to share) -
//                see wiki/performance.md for the full source citation.
// RESTORED (2026-08-06/07): the clean-baseline check (undefined) confirmed
// ~103.5s to /.oldtmp, matching the original ~92.7s baseline - conclusively
// showing the regression tracked back to this mechanism, not the two-page-
// vs-one-page distinction (single-page A/B was ALSO slow, 415s). Real
// cause found: page_shareable()'s FETCH_ALLOWED requirement was applied to
// BOTH pages in dispatch_page_writable()/generate_select_dispatch_key(),
// not just the delay-slot page that actually needs it - see
// own_page_needs_asid()'s comment in mips3.h. Fixed there; this stays 2.
// RESTORED to 2 (2026-08-17) after the task #69 ASID-vs-symptom A/B test:
// busy-loop reproduced identically with this disabled, confirming it's
// not the cause (consistent with the earlier -nodrc result) - back to
// the normal, validated-correctness-fix state.
//
// Task #84 bisection (2026-09-02): tested undefined - 18/100 failures,
// within this project's own established run-to-run noise band, same as
// every other flag tested so far. Restored to 2 (the normal,
// validated-correctness-fix state) - see wiki/console.md task #84
// section for the result.
// Task #84 forced-trigger retest (2026-09-02): undefined and tested
// against the new 100%-reliable native-gdb repro trigger (not just the
// earlier ~30% natural rate) - 30/30 failures, no change. Restored to 2.
// Task #84 forced-trigger retest (2026-09-02): undefined and tested
// against the new 100%-reliable native-gdb repro trigger (not just the
// earlier ~30% natural rate) - 30/30 failures, no change. Restored to 2.
// Task #84 natural-rate retest (2026-09-03): undefined again and tested
// against the NATURAL (non-forced) rate this time - 30 iterations,
// 3/30 failures (10%, one consecutive burst at i=19-21) - lower than
// the established ~29-36% baseline with this fix enabled, and the same
// bursty/clustered failure shape either way. Disabling this fix does
// not eliminate the bug and maximal recompile churn does not increase
// its rate - rules out both "this fix's mere presence causes the bug"
// and "more compile churn causes more failures" as explanations.
// Restored to 2.
// Task #84 natural-rate retest w/ passive polling (2026-09-03, later
// same day): undefined again to capture live mode/dispatch_mode/r19
// telemetry (via /proc/pid/mem polling, no gdb) around the residual
// failures - the as/sh/expr/gcc cross-binary-collision hypothesis was
// ruled out first (none of those binaries' .text reaches anywhere near
// 0x1007de00-0x1007e100), so this rerun is to gather direct evidence on
// what the residual mechanism actually is, not just its rate.
// Restored to 2 (2026-09-03, Opus-review followup) - the fix-off
// natural-rate test turned out confounded (the review found disabling it
// makes the shared key LESS ASID-scoped, not more private, contradicting
// the original framing) - back to the default/baseline config for the
// generated-code-diff experiment.
#define MAME_SGI_DISPATCH_MODE_ASID_FIX 2

// TEMPORARY diagnostic (2026-07-28): trace generate_checksum_block()'s
// per-execution self-check for the elocore_exl_N hot-PC family, to tell
// apart genuine self-modifying/patched guest code from a stale/wrong
// physical-address resolution on MAME's own side - part of the redundant-
// recompile investigation (one PC recompiled 108 times against only 13
// real cache flushes; generate_checksum_block's per-execution mismatch
// check, not the checkints/mode machinery, turned out to be the actual
// trigger for code_compile_block's "override" recompile path).
// DISABLED (2026-08-06): investigation concluded, root cause identified
// and fixed (see Fix 0, journal.md). Left as a toggle, not deleted -
// re-enable if generate_checksum_block's mismatch behavior needs
// re-investigating.
// #define MAME_DRC_CHECKSUM_MISMATCH_DIAG

// TEMPORARY diagnostic (2026-09-03, task #84): same question as
// MAME_DRC_CHECKSUM_MISMATCH_DIAG above (genuine self-modification vs.
// a stale physical-address resolution baked into the compiled block),
// retargeted at the "predicate's answer is empty" flake's own hot PCs
// (parse_answer/cpp_get_token's guest addresses, 0x1007de40-0x1007dffc)
// instead of the earlier elocore_exl_N kernel addresses. The original
// diagnostic's struct/context-pool/callback implementation was already
// removed from this file by the time this was written (only its
// call-site comments survived) - reimplemented fresh below
// generate_checksum_block's own declaration rather than resurrecting
// the old one, since the old context pool's 0x0fac5ffc-specific second
// half isn't needed here.
#define MAME_SGI_CHECKSUM_MISMATCH_DIAG_TASK84

// TEMPORARY diagnostic (2026-09-03, task #84): checksum-mismatch is
// ruled out (5/5 forced reproductions, zero hits from the diagnostic
// above) - this instead tracks, per watched PC, how many times
// code_compile_block() has been called for it this process's whole
// lifetime, plus the raw (pre-override) mode argument and the live
// dispatch_page_writable(pc) result at each call. Goal: confirm/refute
// whether the in-run duplicate-compile pattern observed via an external
// host-gdb watcher (df44/decc/de40 each compiling twice within one
// forced-trigger run, same mode both times) correlates with
// writable_now flipping for the same pc between calls - which would
// point at VTLB-slot contention (a different process's mapping
// temporarily occupying cc1's own code page's VTLB slot) rather than
// anything about cc1's code itself.
// Reverted 2026-09-09: found still active by accident (unconditional
// stdout spam on every compile, discovered while chasing an unrelated
// boot failure) - stale debris from a much earlier sub-investigation,
// never touched by anything this session, but a real, previously-
// unnoticed confound (real per-compile overhead) sitting underneath
// every experiment run today. Should have been caught by a full
// `grep '^#define MAME_SGI_TASK84'` sweep earlier - only specific known
// macros were checked, not an exhaustive pass.
// #define MAME_SGI_RECOMPILE_TRACKING_TASK84

// TEMPORARY diagnostic (2026-09-04, task #84 Opus-review followup #2):
// watches two specific registers at two specific PCs on every execution
// through cc1's parse_answer() loop - $v0/r2 (fast-mapped to a host
// register, per the Opus review's finding that this project's earlier
// "no register caching exists" claim was wrong) right after
// cpp_get_token() returns at 0x1007df50, and $s3/r19 (acount, confirmed
// NOT fast-mapped, always lives in m_core->r[19]) at the 0x1007dffc
// beql check. save_fast_iregs() is called before each probe specifically
// because $v0 IS fast-mapped - reading m_core->r[2] without first
// forcing a sync would return stale data, not the live value.
// Re-enabled (2026-09-04) - cleared of causing the segfault bug: the
// cc1-on-/dev/null test reproduced the predicate bug 60/60 with ZERO
// segfaults on a binary that had this diagnostic (and the tlb_mismatch
// one) fully disabled, proving neither is responsible. Re-running this
// against that new, 100%-reliable natural repro instead of the old
// bigtest.c loop.
// #define MAME_SGI_TASK84_V0_S3_DIAG

// TEMPORARY diagnostic (2026-09-04, task #84 isolation test): the full
// diagnostic above (save_fast_iregs()+UML_CALLC at both probe points)
// turned out to completely and reproducibly SUPPRESS the bug (80/80
// clean cc1-on-/dev/null runs, vs 60/60 failures with neither active) -
// a major result implicating stale fast-mapped-register state ($v0/r2,
// the only actually-fast-mapped register of the two touched) across the
// jalr call boundary into cpp_get_token(). This isolates WHICH part of
// the "fix" matters: save_fast_iregs() alone, with the UML_CALLC/logging
// entirely removed. If the bug stays suppressed, it's genuinely about
// the register sync itself; if it comes back, the earlier result was an
// incidental side effect of the extra UML_CALLC instructions on DRC
// codegen/scheduling, not the register value.
// #define MAME_SGI_TASK84_SAVEFASTIREGS_ONLY

// TEMPORARY diagnostic (2026-09-04, task #84 control test): the sync-only
// variant above ALSO completely suppressed the bug (60/60 clean), but
// save_fast_iregs() only writes the live host register to memory - it
// doesn't reload it, so nothing about the register's own content changes.
// This tests whether ANY extra instruction at these two points suppresses
// the bug (a codegen-structure/sequence-boundary effect, unrelated to
// $v0 specifically) by substituting a single, register-inert store of a
// hardcoded constant to an unused scratch location - same instruction
// count and shape as the sync-only variant, but touching no register
// content at all, not even reading I4. If the bug stays suppressed, the
// mechanism is generic (any extra instruction here dodges it); if it
// comes back, that confirms the register sync itself is what matters.
// Disabled (2026-09-04) - superseded by MAME_SGI_TASK84_POISON_R2 below,
// per the second Opus review's Test 1 (the more direct confirmation:
// prove m_core->r[2]'s memory image is actually consumed by a reload,
// rather than just that removing the whole fast-register mechanism
// removes the bug - see DISABLE_FAST_REGISTERS's own 2026-09-04 comment
// in mips3com.h, confirmed 260/260 clean with fast registers off entirely).
// #define MAME_SGI_TASK84_INERT_ONLY

// TEMPORARY diagnostic (2026-09-04, task #84, second Opus review Test 1 -
// the "poison store" test). Fast registers are back ENABLED for this test
// (DISABLE_FAST_REGISTERS reverted to (0) in mips3com.h) - this writes an
// obviously-wrong value into m_core->r[2] at 0x1007df50, the exact point
// where the suppressing fix (MAME_SGI_TASK84_SAVEFASTIREGS_ONLY) writes
// the CORRECT live value instead. The review's reasoning: the only thing
// in the entire codebase that ever reads m_core->r[2] back is
// load_fast_iregs(), called on every re-entry through static_generate_
// entry_point (mips3drc.cpp:4898) - which happens on every
// EXECUTE_MISSING_CODE recompile, not once per timeslice as this
// session's own earlier analysis wrongly assumed. If that's right, some
// later reload should clobber live r15/$v0 with this garbage and cause a
// loud, immediate crash (wild pointer deref, SIGSEGV) rather than the
// subtle "predicate's answer is empty" text - a decisive, unambiguous
// result either way, not another narrowing step.
// Disabled (2026-09-04) - CONFIRMED: 46/60 (76.7%) predicate failures vs
// ~10% baseline, 0 crashes/segfaults. Clean dose-response result (see
// journal.md 2026-09-04 18:44) - superseded by MAME_SGI_TASK84_WINDOW_TRACE
// below, which goes looking for the actual missing/mismatched sync rather
// than further demonstrating the mechanism is real.
// #define MAME_SGI_TASK84_POISON_R2

// TEMPORARY diagnostic (2026-09-04, task #84): two-sided trace to catch
// the actual defect in the act, not just demonstrate it exists (per the
// user's own sharp question - the poison/inert/disable-fast-regs tests
// above all *demonstrate* the mechanism, none of them *locate* it).
// diag_task84_armed is toggled by two plain, register-inert stores
// (confirmed inert by MAME_SGI_TASK84_INERT_ONLY's own 60/60 FAIL result -
// this must NOT use save_fast_iregs() at either probe point, since that's
// now known to suppress the bug outright) at generate_sequence_instruction's
// two existing probe PCs: armed=1 at 0x1007df50 (right after cpp_get_token()
// returns), armed=0 at 0x1007dffc (the loop-exit check) - confining
// everything below to the exact narrow window between them, regardless of
// which handler or which process's activity is actually responsible.
// Every real save_fast_iregs() call site that leaves generated code
// (static_generate_nocode_handler, static_generate_out_of_cycles,
// static_generate_tlb_mismatch, static_generate_exception) gets a
// gated CALLC right after its existing save_fast_iregs() call, firing
// only while armed, logging (site, pc, r[2]). static_generate_entry_point's
// load_fast_iregs() (the sole reload site, mips3drc.cpp:4921) gets a
// matching gated CALLC right after it, logging (pc, r[2]) - if this ever
// fires while still armed, that's a re-entry happening inside the exact
// window with no matching disarm, which is the direct, structural
// evidence needed: either its r[2] matches the most recent [SAVE] line
// (round-trip is lossless, rule this window out) or it doesn't (found the
// actual gap) or there's no preceding [SAVE] at all in this window
// (found a genuinely uninstrumented exit path).
// #define MAME_SGI_TASK84_WINDOW_TRACE

// TEMPORARY diagnostic (2026-09-04, task #84): the decisive save-vs-reload
// audit. Resolves a hard contradiction the investigation reached: the
// A/B/C/D experiments jointly imply m_core->r[2]'s MEMORY image goes stale
// and is reloaded into r15 (writing the correct live value there = 0/140
// failures; writing a WRONG value there = still failing; writing the same
// shape of instruction to a DIFFERENT address = still failing - so both the
// location and the value are load-bearing), yet a full structural audit
// says every path that leaves generated code calls save_fast_iregs() first
// (4 UML_EXIT sites in this file, all covered; and at the backend level
// drcbex64's only other exits are op_exit's m_exit, m_nocode which jumps
// back INTO generated code rather than out, and m_endofblock which is
// [[noreturn]] std::abort()). Both cannot be true.
//
// Rather than patch each of the 9 save_fast_iregs() call sites (and risk
// missing one - which is the very failure mode being hunted), the shadow
// copy is emitted INSIDE save_fast_iregs() itself, so coverage is complete
// by construction and stays complete if a call site is ever added. The
// entry point then compares live r[2] against that shadow immediately
// before load_fast_iregs() and calls out only on a mismatch, so the log
// stays empty unless the bug is real - important given a previous version
// of this diagnostic produced a 367MB log in 20 iterations on a disk at
// 98% full. The C side additionally caps itself at 200 logged lines.
//
// Outcome is decisive either way: a mismatch catches the stale reload in
// the act (with the site tag identifying which save preceded it), while a
// clean run falsifies the deduction above and forces the search elsewhere.
// #define MAME_SGI_TASK84_SAVE_RELOAD_AUDIT

// TEMPORARY diagnostic (2026-09-05, task #84): does cpp_get_token's page
// actually get MAME_SGI_DISPATCH_MODE_ASID_FIX's shared, ASID-independent
// dispatch key, or the full ASID-scoped one? See the use site
// (code_compile_block(), ~line 1573) for the full rationale and the
// widened-scope note (the exact-entry-only gate never fired despite
// MAME_SGI_TASK84_CPPGETTOKEN_COMPILE_TIMESTAMP proving fresh compiles
// happen - 0x10076008 surfaces as an interior HASH marker of some other
// block, never as its own code_compile_block() request).
// ANSWERED (2026-09-05, same day): confirmed live - every compile request
// touching cpp_get_token's page returns dispatch_page_writable()==false
// (WRITE_ALLOWED clear), and all 7 observed hits in one run carried the
// identical mode value (759) rather than varying per-ASID - the shared,
// ASID-independent key genuinely is what's in effect here, not just the
// fix's design intent. See journal.md's matching 2026-09-05 entry.
// RE-ENABLED and EXTENDED (2026-09-05, same day) - causal-connection
// follow-up: also logs the raw, unresolved m_core->dispatch_mode (ASID
// composite) per compile of a given pc, to check whether it actually
// varies across successive compiles of the same shared entry - see the
// use site's own extended comment for the full rationale.
// ANSWERED AGAIN (2026-09-05, same day): raw_asid/raw_dispatch_mode were
// identical (128/515) across all 8 hits in a 30-iteration cc1_null_loop.sh
// run - IRIX reuses the same ASID slot for every sequential invocation of
// the same binary in a tight loop with nothing else competing for
// process-table slots, so these recompiles happen with zero ASID
// transition. This result, combined with the project's own established
// rates (gcc -c bigtest.c: 10-36%; standalone cc1 -quiet /dev/null: 92.2%
// at n=500 - narrowing AWAY from process/load complexity INCREASED the
// hit rate, the opposite of what "more ASID transitions/load is the
// trigger" would predict), significantly weakens the whole ASID-sharing/
// process-count causal direction rather than confirming it. See
// journal.md's matching 2026-09-05 entry for the full reasoning.
// RE-ENABLED again (2026-09-05, same day) - genuine-concurrency follow-up:
// prior test only ever had ONE process (cc1) active at a time, sequentially
// re-executed - explaining the zero ASID variation found. This run adds
// real concurrent background load (mxcsrtest/fcvtrepro/fpload/fputest2,
// recovered into test/ this session - see wiki/README.md's $TEST entry)
// running alongside cc1_null_loop.sh, to test whether genuine multi-process
// concurrency (as opposed to sequential reuse of one binary) produces
// different ASID/recompile behavior.
// Reverted 2026-09-09: same accidental-discovery, same-comment class as
// MAME_SGI_RECOMPILE_TRACKING_TASK84's own revert note above - stale,
// unconditional stdout-spamming debris from a much earlier ASID
// sub-investigation, never touched this session, a real confound
// sitting underneath every experiment run today.
// #define MAME_SGI_TASK84_CPPGETTOKEN_ASID_DIAG

// Task #84 (2026-09-05): direct experimental control over whether
// cpp_get_token's own compiled sequence (seqhead 0x10076008) gets
// interrupt-check preambles woven in at its interior branch targets.
// Mutually exclusive - define at most one at a time (neither defined:
// natural behavior, driven by the sticky m_checkints_addrs mechanism -
// see code_compile_block()'s own comment for the full explanation).
// Use site: code_compile_block(), right before the natural
// m_checkints_addrs/self-loop/backward-branch heuristics.
// #define MAME_SGI_TASK84_FORCE_CHECKINTS_ON
// REVERTED 2026-09-07 (third Opus review §5/§7.5): this was left live from
// an earlier controlled A/B and was silently in effect for several recent
// measurements including the 2026-09-07 50-sample ASLR-correlation batch
// (32% failure rate, vs the normal ~14-25% baseline) - a more parsimonious
// candidate explanation for that deviation than anything about the
// checkpoint used. Commented back out so the widened [ENTRY_PC_WIDENED]
// probe (see cfunc_task84_entry_pc_diag) is measured against natural
// checkints behavior, not a forced-off build.
// #define MAME_SGI_TASK84_FORCE_CHECKINTS_OFF

// Task #84 (2026-09-05): logs every genuine interrupt delivery through
// static_generate_entry_point()'s unconditional check when the resume PC
// falls inside cpp_get_token's window - the one interrupt-delivery
// mechanism not yet ruled out after the checkints experiments above. See
// cfunc_task84_entry_irq_diag's own comment (near static_generate_entry_point)
// for the full rationale.
// #define MAME_SGI_TASK84_ENTRY_IRQ_DIAG

// Task #84 (2026-09-05): user-proposed structural check - maintain running
// counts of save_fast_iregs()/load_fast_iregs() call executions and look
// for an imbalance. Counters live in m_core (diag_task84_save_count/
// diag_task84_load_count, mips3.h) so a host-side gdb attach can read them
// directly (see tools/read_saveload_counts.sh) - no guest-visible
// footprint, same technique as tools/gdb_flush_drc_cache.sh. A raw
// imbalance doesn't necessarily mean a bug on its own (a save followed by
// a hashjmp directly into a compiled exception handler, bypassing
// static_generate_entry_point's one load site, is architecturally valid -
// see this macro's own use-site comments for the full reasoning) - this
// is built as an observational tool to correlate against pass/fail, not a
// direct bug detector.
// #define MAME_SGI_TASK84_SAVELOAD_BALANCE_DIAG

// Task #84 (2026-09-05): precise, guest-instruction-order bracket around
// ONE cpp_get_token() invocation's save/load activity - see
// cfunc_task84_saveload_bracket_entry/_exit's own comment
// (generate_sequence_instruction's use site) for the full rationale. This
// replaces the wall-clock/gdb-polling approach
// (tools/flush_saveload_sample.sh), which proved too noisy to trust -
// deltas varied by orders of magnitude cycle to cycle with no relation to
// the actual guest command, because real time kept passing (and the guest
// kept running) during each external gdb attach/read. Requires
// MAME_SGI_TASK84_SAVELOAD_BALANCE_DIAG also defined (needs the counters
// this brackets).
// #define MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG

// Task #84 (2026-09-06): direct seqhead-boundary/checkints diagnostic for
// the stress_v0_split increment - logs every seqhead compiled within
// stress_v0_split's own address window (0x10001000-0x10002000, shared
// with the -drc_log_native address filter already repurposed for this
// investigation), recording seqhead->pc, seqlast->pc (so the actual
// compiled extent of each sequence is visible directly, not reconstructed
// by hand from raw disassembly - that reconstruction attempt for this
// exact test produced ambiguous results, which is why this exists),
// is_branch_target(), and the resulting compiler.checkints/checksoftints
// decision. Answers directly whether the deliberate COMPILE_MAX_SEQUENCE-
// cap split (see test/stress_v0_split.s's own header) actually lands
// where hand-counted, and whether the segment containing $v0's read
// really does come up checkints=false as predicted.
// #define MAME_SGI_TASK84_SEQHEAD_CHECKINTS_DIAG

// TEMPORARY diagnostic (2026-09-07, task #84, third Opus review's §7.3 -
// "the single test that can settle the fast-register hypothesis in either
// direction"): a continuous $v0 shadow-divergence detector. Nothing in this
// investigation has ever directly asserted that r15 (when fast-mapped) still
// equals the last value the GUEST itself wrote to $v0 - every prior probe
// either checked a static plausibility threshold (the 2026-09-07
// [ENTRY_PC_WIDENED] work, which turned out to be mostly noise outside one
// specific PC) or sampled only at rare DRC-exit moments (missing the vast
// majority of executions). This checks EVERY guest instruction boundary.
//
// Mechanism, in generate_sequence_instruction(): BEFORE each instruction's
// own codegen, compare the live R64(2) (I4 if fast-mapped, memory
// otherwise - build-agnostic) against m_core->diag_v0_shadow; if they
// differ, log the divergence (desc->pc is where it was NOTICED, i.e. the
// first instruction boundary after whatever changed it). AFTER each
// instruction's own codegen, unconditionally save the (possibly
// just-changed) live value back into the shadow. Because the shadow is
// updated after literally every instruction - unconditionally, whether or
// not that specific instruction wrote to $v0 - the shadow can only ever
// go stale between two ADJACENT instruction boundaries, and the only way
// live and shadow can differ at the next "before" check is if something
// changed $v0 in between WITHOUT going through this same instrumented
// per-instruction path: a DRC exit+reentry with a bad reload, an
// interrupt/exception handler with an unaccounted-for save/restore gap,
// or any other out-of-band host-side mechanism this investigation has
// been trying to find.
//
// DELIBERATELY GLOBAL, NOT PC-RANGE-GATED, unlike every other task84
// probe in this file. This is a real, disclosed departure from this
// project's usual "gate to the narrow window under test" convention, and
// it is required for correctness here, not just thoroughness: the
// 2026-09-07 static disassembly work found cpp_get_token calls out to two
// helper functions at 0x100755f8/0x10075bf8 - BELOW any PC window this
// investigation has used - so a PC-gated version of this detector would
// see $v0 legitimately change during those calls (real writes, by
// functions outside the gate) with no matching "after" save to account
// for them, producing guaranteed false positives at every re-entry into
// the gated window. Global coverage (the shadow updated by EVERY guest
// instruction everywhere, kernel included) is what makes "a divergence
// can only mean an out-of-band change" a sound claim at all - a scoped
// version would not have this property. Cost is real (~3 host
// instructions per guest instruction, system-wide) but this is a
// temporary, revert-before-shipping diagnostic build like every other
// macro in this section, not a permanent change.
//
// Deliberately did NOT adopt the review's suggested speed-gate
// (`((op>>21)&31)==2 || ((op>>16)&31)==2`, i.e. only instrument
// instructions whose OWN encoding references $v0 as rs/rt) - checked it
// against the MIPS instruction encoding and it has a real gap: it never
// checks the rd field (bits 11-15), so an R-type instruction that writes
// $v0 as its DESTINATION while neither rs nor rt is $v0 (e.g. `or
// $v0,$a1,$zero`, rd=2/rs=throwaway/rt=$zero) would be silently skipped,
// reintroducing exactly the false-negative gap this detector exists to
// close. Full per-instruction coverage avoids relitigating that.
// Reverted 2026-09-08 for a clean baseline ahead of the predicate-stress
// reduced-test-case experiment: this diagnostic is deliberately global
// (not PC-gated, see the comment above), adding real unconditional
// per-instruction overhead system-wide, not just conditional logging -
// too big a confound to leave active for a fresh, independent experiment
// aimed at characterizing the bug's true behavior. Re-enable if resuming
// the shadow-divergence line of investigation specifically.
// #define MAME_SGI_TASK84_V0_SHADOW_DIAG

// TEMPORARY diagnostic (2026-09-09, task #84) - direction 2 of the two
// the user picked after cpp_get_token's AND parse_answer's own compiled
// bytes were proven identical between passing and failing instances
// (wiki/task84.md, 2026-09-09 "direct answer" entry): cpp_get_token is
// itself reached via a real GOT-indirect call (`lw $t9,-0x7998($gp)` /
// `jalr $t9` at 0x1007df44/0x1007df48, confirmed via fresh disassembly -
// not a plain direct `jal`). $t9/r25 is NOT fast-mapped on this build
// (only $v0/r15 is), so its value at the jalr is whatever guest MEMORY
// (the GOT slot) held at the moment of the preceding `lw` - a genuinely
// different class of state from every register-continuity mechanism
// already exhaustively checked for $v0. This logs ONLY a mismatch
// against the one legitimate value this call site should ever produce
// (0x10076008, independently confirmed via live gdb AND a static GOT
// read earlier in this investigation) - cheap, self-capped, PC-gated to
// this exact call site, same shape as every other task84 mismatch-only
// probe.
// #define MAME_SGI_TASK84_T9_GOT_DIAG

// TEMPORARY diagnostic (2026-09-09, task #84) - a real EXECUTION-order
// call trace, not the compile-order proxy used earlier today
// (COMPILE_ORDER_DIAG logs when a PC is first JIT-compiled, once per
// code region; this logs every actual JAL/JALR execution, which is what
// the user asked for directly: "how much code executes before the
// failure... do we have an execution trace of function calls"). Fires
// for any branch-and-link (linkreg != 0, i.e. JAL or JALR, not a plain
// branch) whose target lands in cc1's own user-space text
// (0x10000000-0x7fffffff, excluding KSEG0 kernel noise the same way
// V0_SHADOW_DIAG's own filter did). Runtime cost is real (one UML_CALLC
// per call executed) - scoped to a single, short, deliberately-run
// cc1-on-/dev/null invocation for orientation, not left on across a
// pass/fail-correlated batch the way every other probe here is.
// #define MAME_SGI_TASK84_CALL_TRACE_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - find the actual call
// site that decides between the ordinary-processing path and the
// cpp_error path found via wiki/task84.md's 2026-09-10 OK-vs-PREDICATE
// call-trace diff (divergence at ~line 160085, OK calling cpp_lookup vs
// PREDICATE calling cpp_error at the same relative position). Static
// disassembly can't resolve this: cc1 calls cpp_error (like
// cpp_get_token) via `jalr $t9` through a GOT slot, not a direct `jal`,
// so there's no fixed call-site address to grep for in the binary - a
// whole-binary scan for `jal 0x10086ab0` (cpp_error's entry) found zero
// hits, confirming this. Hooks the DYNAMIC-target call path specifically
// (mirroring CALL_TRACE_DIAG's own `else` branch), checks the runtime
// target against cpp_error's known address, and logs `desc->pc` itself
// - the call/jalr instruction's own compile-time-known address - which
// directly IS the call site, no $ra capture needed.
// #define MAME_SGI_TASK84_CPPERROR_CALLSITE_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - static_generate_tlb_mismatch()
// is a completely separate shared handler from static_generate_exception()
// (the one MAME_SGI_TASK84_DRC_EVENT_LOG's log_drc_exception() already
// hooks) - found live that task84_drc_events.log therefore has ZERO
// visibility into tlb_mismatch/compiler_page_fault() activity, even
// though today's ASM-diff work found the compiled code at 0x1007DFFC
// unconditionally invokes exactly this handler in a PREDICATE run. Logs
// every entry into this handler (PID-tagged, via the same log_drc_event()
// used elsewhere) so its actual firing pattern for a real PREDICATE
// invocation can finally be observed directly instead of inferred from
// static disassembly alone.
// #define MAME_SGI_TASK84_TLB_MISMATCH_ENTRY_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - trace $v1 (the token
// type byte just fetched via cpp_get_token, read at 0x1007de90) and $s1
// (a value saved from this function's own $a2 argument at entry) back
// to their origin. Hooked at the first of the three `beq` checks
// (0x1007de98) that decide whether to fall through to cpp_error - see
// wiki/task84.md 2026-09-10 for the full disassembly this targets.
// Fires on EVERY execution of this check, not just cpp_error-triggering
// ones, so an OK run's sequence of v1/s1 values at this exact call site
// can be diffed directly against a PREDICATE run's - if cpp_get_token's
// own compiled code and call sequence are both already proven identical
// (closed earlier this investigation), a difference here would localize
// the actual divergence to cpp_get_token's INPUT/lexer state at this
// specific call, not its code.
// #define MAME_SGI_TASK84_CPPERROR_COND_TRACE_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - direct empirical test of
// the source-text-buffer-corruption hypothesis raised by re-reading real
// GCC 3.3 source (gcc/cpplib.c, cached at this session's scratchpad -
// see wiki/task84.md 2026-09-10): parse_answer() is the SAME shared code
// for both #assert (registration) and #if #predicate() (testing), and
// the flush/recompile experiment (2026-09-08 entry) already proved the
// outcome is tied to the HOST-side compiled instance, not per-process
// guest data - meaning if source TEXT content is actually the culprit,
// it must come from some OTHER function's compiled bytes (not
// parse_answer/cpp_get_token, both already proven byte-identical)
// writing different (but compile-instance-deterministic) text into the
// "<built-in>" buffer. Tests this directly by dumping the actual raw
// source bytes cc1 is reading at the loop's own repeated cpp_get_token
// call site (0x1007df44), across every iteration, both outcomes -
// real corrupted/malformed text here would settle the hypothesis
// immediately; identical, well-formed text would rule it out and point
// back toward the tokenizer/register mechanism instead.
//
// cpp_reader->buffer is offset 0 (gcc/cpphash.h's struct cpp_reader,
// first field); cpp_buffer->cur (current read position) is ALSO offset
// 0 (struct cpp_buffer, first field) - two trivial zero-offset
// dereferences from $s0 (this function's own saved pfile pointer),
// deliberately chosen to minimize arithmetic-error risk after this
// session's earlier string-offset mistake.
// #define MAME_SGI_TASK84_SOURCE_TEXT_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - the decisive follow-up
// proposed back on 2026-09-10 (this file's own "traced state before the
// first (loop) cpp_get_token call" work, and wiki/task84.md's matching
// entry) and never built until now: read $s3 a SECOND time, right at the
// actual `beql $s3,$zero,0x1007e020` compare itself (0x1007DFFC), not
// just at the loop's call site (SOURCE_TEXT_DIAG's own $s3 capture,
// 0x1007df44/48) - a much earlier session found $s3 progressing 0 then 1
// at the call site even in a confirmed-failing run, directly contradicting
// cc1's own "answer is empty" report, and flagged this as a real,
// unresolved anomaly rather than trusted. User pushed to revisit this
// with fresh eyes ("does this bring us back to the $s3 issue") after
// every other lead (fast registers, tlb_mismatch/compiler_page_fault,
// compiled-code identity, $v0 corruption) was independently closed this
// session. Deliberately narrow and PC-gated - same low-risk shape as
// every other point-diagnostic in this file, not a system-wide change
// (ruled out MAME's own -debug/gdbstub machinery for this exact reason:
// enabling it changes codegen, via an extra UML_DEBUG call, on EVERY
// compiled instruction system-wide, not just at a breakpoint - a real,
// unavoidable perturbation of the very thing being measured, per the
// user's own direct pushback).
// #define MAME_SGI_TASK84_BEQL_S3_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - the user's delay-slot/
// page-crossing hypothesis, tested directly. static_generate_tlb_mismatch()
// is a SHARED trampoline (emitted once, at device-start) reached from
// every UML_EXH/UML_EXHc in the whole compiled program. Its "the saved PC
// may be set 1 instruction back with the low bit set to indicate a delay
// slot" comment matters here: it does TWO separate UML_RECOVER(MAPVAR_PC)
// reads of the SAME exception - the first (ADD 3; AND ~3, rounds UP) is
// used for the vtlb page lookup and can legitimately resolve to the delay
// slot's OWN address; the second (AND ~3 only, rounds DOWN) is what
// actually gets stored to m_core->pc and HASHJMP-redispatched to. For our
// beql at 0x1007DFFC, whose delay slot at 0x1007E000 sits on a fresh page
// (crosses a page boundary right there), a TLB miss firing while
// generate_delay_slot_and_branch() validates that page (reachable ONLY on
// the taken path, since the not-taken path never calls it at all - already
// confirmed by MAME_SGI_TASK84_BEQL_ANNUL_DIAG showing 0/0 branch_target/
// appended for the delay slot's own descriptor in both OK and PREDICATE
// compiles) would redispatch back to rounded-down PC 0x1007DFFC - i.e.
// force the CPU to RE-EXECUTE THE BEQL ITSELF FROM SCRATCH. If anything
// about that specific retry path re-evaluates the comparison against
// stale/wrong state, this is exactly the kind of narrow, page-crossing-
// specific mechanism that would explain a bug isolated to one beql rather
// than a general BEQL/annulment defect. Logs $s3 at the moment of
// redispatch AND the first (rounded-up, delay-slot-address-if-applicable)
// recovered PC, gated to fire only when the redispatch target is exactly
// our beql's own PC - cheap (one CMP) on every OTHER tlb_mismatch firing
// system-wide, since this is a shared handler.
// #define MAME_SGI_TASK84_TLBMISS_REDISPATCH_DIAG

// TEMPORARY diagnostic (2026-09-09, task #84) - the direct follow-up to
// CALL_TRACE_DIAG's own finding: cpp_get_token is called exactly 17
// times in every single trace (18 captured, 16 OK/2 PREDICATE), with
// zero exceptions - the call COUNT never varies, so whatever decides
// pass/fail must be in what one of those 17 calls RETURNS (the token
// pointer in $v0, specifically its type byte at offset 6, the same
// field every real caller reads via `lbu $v0,6($v0)`). This hooks
// cpp_get_token's own three real return points (0x10076134, 0x10076230,
// 0x100762a0 - found via fresh disassembly, not assumed) rather than
// duplicating logic at each of its several call sites, and tags every
// hit with a running per-process sequence number so an OK trace and a
// PREDICATE trace can be aligned by call INDEX (1st call, 2nd call, ...)
// to find exactly which one first differs.
// #define MAME_SGI_TASK84_TOKEN_RESULT_DIAG

// TEMPORARY diagnostic (2026-09-10, task #84) - PID-filter calibration.
// User's challenge: the kernel itself knows exactly which process is
// running at any moment, and this project has real leaked IRIX 6.5.7m
// kernel source ($IRIXSRC) plus the guest's own installed headers - so
// use those instead of the $sp/ASID register proxies from the previous
// round. Traced the real chain: PDAADDR (a fixed kernel VA, remapped to
// a different physical page on every context switch - immu.h) -> the
// `private` pda_t at that address -> `p_curuthread` (confirmed at
// offset 0x14 in pda_s from the GUEST'S OWN /usr/include/sys/pda.h,
// extracted via chd_extract_file.py with no boot needed - two leading
// pointer fields after the 16-byte `db` union) -> a uthread_s pointer,
// whose `ut_proc` field points at the owning proc_t, whose `p_pid` is
// the field we actually want.
//
// Both remaining hops (ut_proc's offset in uthread_s, p_pid's offset in
// proc_t) are NOT safely hand-derivable: uthread_s embeds a whole
// kthread_t before ut_proc, and proc_t's real struct body
// (pproc_private.h in $IRIXSRC) starts with a large, version-sensitive
// proc_proxy_t - and that header is kernel-build-private, not part of
// the guest's own installed /usr/include (confirmed: the guest's
// proc.h only forward-declares `struct proc`). Trusting a hand-counted
// offset here risks silently reading a plausible-looking WRONG pid.
//
// So: calibrate empirically instead of computing. This scans a bounded
// window of DWORDs from the (trusted) uthread_ptr for anything that
// looks like a plausible kernel pointer, then a bounded window from
// EACH such candidate for anything in a plausible pid_t range, logging
// every (outer_offset, inner_offset, value) hit. Cross-referencing
// against the harness's own captured real cc1 pid (see the harness
// script's `cc1pid=` addition) for the SAME invocation, offline, finds
// the one (outer,inner) pair that is genuinely stable - real struct
// offsets, not guessed ones. Deliberately capped low (s_seq) - this is
// a one-time calibration, not a permanent diagnostic; once the offsets
// are known, replace this with a direct two-read filter.
// #define MAME_SGI_TASK84_PID_CALIBRATION_DIAG

// Standing counter (2026-09-10, task #84) - measured how often
// generate_likely_branch_skip_fixup()'s compile-time condition
// (compiler_page_fault() on a BEQL-class delay slot) actually fires in
// realistic use, to inform the Fix A vs. Fix B (general drcfe.ipp fix)
// discussion - see wiki/task84.md's "cost estimate" entry. Confirmed
// firing at 20-35% of explicit-flush cc1 cycles (matching the historical
// pre-fix PREDICATE rate, as expected) with zero added recompile cost
// over the healthy baseline. Its job is done; disabled now that task #84
// is closed out (2026-09-10) - re-enable if the Fix A/B question ever
// needs revisiting for a different call site.
// #define MAME_SGI_TASK84_FIXUP_DIAG

// TEMPORARY diagnostic (2026-09-03, task #84 Opus-review followup):
// re-run of the review's recommendation #4 - watch tlb_mismatch arrivals
// specifically, not the TLBLOAD/TLBSTORE_FILL exception handlers this
// investigation originally watched (which the review found miss 96.2%
// of the real traffic, per journal.md 2026-08-23's own measurement at
// the compiler_page_fault() site below). Filtered at the two UML_EXH/
// UML_EXHc call sites in generate_sequence_instruction() (compile-time
// desc->pc range check, zero runtime cost outside the critical window)
// rather than via a live gdb watcher - an unconditional gdb breakpoint
// on this handler was tried first and cost a real, measured ~34
// real-minute stall for a single trivial guest command (system-wide
// tlb_mismatch volume is far higher than the exception handlers tried
// earlier), confirming this needs to be a compiled-in, PC-filtered
// probe like the existing 0x0fac5ffc diagnostics, not a runtime gdb
// condition.
// Disabled (2026-09-04) - same causation test as MAME_SGI_TASK84_V0_S3_DIAG
// above. This one is purely observational (no save_fast_iregs(), no
// state mutation) so it's a lower-risk suspect, but disabling for the
// cleanest possible baseline while testing whether ANY of today's
// diagnostics could be responsible for the newly-found segfault.
// #define MAME_SGI_TASK84_TLB_MISMATCH_DIAG

// CANDIDATE FIX under test (2026-09-05, task #84): static_generate_exception()
// (below, ~line 5445) calls save_fast_iregs() before handing off to the
// guest exception vector - added 2026-07-30 specifically because $v0 is
// fast-mapped to a host register (I4) and would otherwise never get
// flushed to m_core->r[] before the jump. CORRECTED 2026-09-05: this
// used to say "$v0/$v1/$a0 ... (I4/I5/I6)", copied from mips3.cpp's
// intent when the backend has >=7 direct integer registers (true on
// Windows - int_register_map there lists 7: Bx/Si/Di/R12-R15). On this
// project's actual Linux build, drcbex64.cpp's non-Windows
// int_register_map has only 5 entries (Bx/R12-R15), so
// beinfo.direct_iregs=5 clears only the ">4" gate - $v0/I4 is the ONLY
// fast-mapped GPR here; $v1/$a0 stay ordinary memory operands. Verified
// directly (mips3.cpp:786-807, drcbex64.cpp:397-404,1754-1755), not
// assumed - see journal.md's matching 2026-09-05 entry. load_fast_iregs() has
// exactly one call site in this file (static_generate_entry_point, the
// one-time C++-scheduler-to-DRC entry per timeslice - verified via a
// correctly-scoped grep across mips3drc.cpp AND the headers, not repeating
// the mistake of an earlier session's file-scoped grep). ERET's own
// handler (generate_special(), case 0x18 below) hashjmps directly back
// into guest code at [EPC]/[ErrorPC] without ever going through that
// entry point - so nothing reloads the fast-mapped registers on the way
// back from an interrupt/exception, even though the interrupted code's
// real value is sitting correctly saved in m_core->r[] the whole time.
// Since $v0 is a genuinely hot general-purpose register, whatever
// interrupt-handler code ran in between will very plausibly have written
// its own values into I4 for its own purposes - meaning the resumed code
// sees the handler's leftover value, not its own. This directly matches
// task #84's established facts (DRC-only; $v0/r15-specific; suppressed by
// DISABLE_FAST_REGISTERS=1) and today's finding that failure correlates
// with compiled instances carrying MORE interior interrupt-check
// insertion points. Testing whether adding the missing reload here
// changes the failure rate - see wiki/task84.md for the full methodology
// (confounds, sample size, what each outcome would mean) before reading
// too much into a bare rate change.
//
// REVERTED (2026-09-05, same day): caused an immediate PANIC: KERNEL
// FAULT (PC=0xff00) on the very first state-restore resume tested - not
// a rare/statistical regression, reproduced instantly. Root cause of why
// this hypothesis was wrong, not just mis-timed: real MIPS exception
// handlers restore the interrupted context's own registers themselves,
// via ordinary guest instructions in their epilogue (e.g. `lw $v0,
// off($sp)`) BEFORE executing eret - and since any guest write to $v0
// already goes straight into the fast-mapped host register (I4) through
// the completely normal compiled-code path, that restore already works
// correctly with zero special handling needed. This fix's load_fast_iregs()
// call ran AFTER that guest-level restore and OVERWROTE the value the
// handler had just correctly written with whatever stale content was
// sitting in m_core->r[2] from the last flush point - actively
// corrupting a register the guest had just fixed, not restoring
// anything. The entry-side save_fast_iregs() fix (2026-07-30, in
// static_generate_exception above) is not asymmetric in the way this fix
// assumed - it exists so m_core->r[] is correct for other consumers
// reading it directly while control is in the exception vector, not
// because anything needs to be pushed back INTO the fast registers on
// the way out. Kept here, commented out, as a record of a specific,
// tested, and refuted hypothesis - see wiki/task84.md for the full
// writeup and where the investigation goes next.
// #define MAME_SGI_TASK84_ERET_FASTREG_RELOAD_FIX

// TEMPORARY diagnostic (2026-09-01): task #84's gcc <built-in>
// "predicate's answer is empty" flake. Built to cross-check m_prptr()'s
// raw pointer against m_program->read_dword() for the same physical
// address at compile time, for user-mode blocks. DISABLED (2026-09-01,
// same day) after an opus-model review of this diagnostic's own code
// found the user-mode gate below is unsatisfiable as written: both
// insertion sites in code_compile_block() run AFTER that function's own
// `if (!dispatch_page_writable(pc)) mode = m_core->mode;` reassignment
// (~line 1245), which always fires for cc1's read-only .text - so the
// bit-decode (copied from MAME_DRC_USERMODE_RECOMPILE_DIAG, which is
// valid only where IT sits, before that reassignment) never passes
// there. The "zero events across 160 iterations" result this produced
// was reported as a real negative finding and used to argue DRC's
// cache-pointer path was clean; that conclusion has been retracted -
// see wiki/console.md's 2026-09-01 update for the correction and the
// (unrelated, more fundamental) finding that the missing content lives
// in cc1's .rodata as data, never fetched as an instruction, so DRC's
// instruction-fetch checksum/m_prptr machinery was never going to be
// the right place to look regardless of the gate bug. Left as a toggle,
// not deleted, in case the gate is fixed and reused later (would need
// the mode captured before the ~1245 reassignment, and the comparison
// target changed to seqhead->opptr rather than a second live read).
// #define MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG

// TEMPORARY diagnostic (2026-09-02): task #84's gcc <built-in>
// "predicate's answer is empty" flake. Follows a live discussion that
// reframed the leading theory: cc1's .rodata page is confirmed (via a
// separate mips3com.cpp diagnostic - see wiki/console.md) to be mapped
// FETCH_ALLOWED, sharing one executable segment with .text (common for
// static non-PIE binaries) - so from DRC's TLB-permission point of view
// it is indistinguishable from real code, even though PC never
// legitimately lands there. This project's own prior measurement work
// (tasks #39-42: sequence-length distribution, termination-reason
// categorization, window-edge vs branch-target vs max-sequence-cap
// termination) already established describe_code()'s sequence-window
// walk as a genuinely nontrivial area. If a compiled sequence's window
// ever walks past the true end of reachable code into the following
// .rodata bytes - misinterpreting string/table data as opcodes - a
// misdecoded "instruction" there could itself look like a branch and
// send the describe-ahead walk arbitrarily further into the data
// region (not just a small fixed overrun), which would explain
// reaching content ~110KB past the .text/.rodata boundary without
// requiring every offset in between to be independently corrupted.
// This would never touch the .rodata bytes' own storage (matching the
// TLB-race diagnostic's clean result) and would be DRC-only by
// construction (the interpreter fetches one instruction at a time from
// wherever PC genuinely is - it has no describe-ahead window to
// overrun). Checks, at every code_compile_block() call, whether the
// compiled sequence's endpoint (codelast) ever reaches into cc1's
// .rodata VA range (established via static analysis of the extracted
// binary - see wiki/console.md's task #84 section) - if this fires at
// all, and especially if it clusters around the same drc_cache-flush-
// heavy windows the actual gcc failures already cluster around, that
// would directly support the mechanism proposed for why a
// deterministic decode bug produces a load-correlated, intermittent
// symptom: which PC first triggers EXECUTE_MISSING_CODE (and therefore
// which describe_code() call happens at all) depends on cache eviction
// pressure, not just on the fixed bytes at a fixed address.
// RESULT (2026-09-02): ran against a matched-load compile loop, 100
// iterations, 47 real failures across several bursts - zero overrun
// events. DRC never once decoded an instruction from within .rodata
// during any of these failures. Rules this theory out too. See wiki/
// console.md's task #84 section for the pivot this motivated: from
// "is the memory content wrong" (now tested and ruled out every way
// this investigation could find) to "is the ADDRESS being read wrong" -
// cc1 itself computing a corrupted offset/index and landing on a
// different, still-validly-mapped .rodata location than intended.
// #define MAME_SGI_RODATA_SEQUENCE_OVERRUN_DIAG

// TEMPORARY diagnostic (2026-09-02, continued): task #84's gcc
// <built-in> "predicate's answer is empty" flake. Every prior
// diagnostic tested whether memory CONTENT at a known-correct address
// was ever wrong - all came back clean. This tests the complementary
// possibility: that cc1's own compiled code computes a wrong ADDRESS
// and reads a different (but still validly-mapped, so no fault)
// location within .rodata than the one that actually holds the
// predicate string. Hooks static_generate_memory_accessor() - the
// single shared UML routine ALL DRC-compiled guest reads/writes of
// every size funnel through (I0 holds the guest virtual address on
// entry) - to capture, for any READ whose target virtual address falls
// within the specific failing page (0x10343000-0x10343fff, containing
// the known string at 0x10343970), both the address actually read and
// the value the CPU's own compiled load instruction actually saw.
// Deliberately does NOT rely on knowing which guest PC performs the
// lookup (find_string_xref() found no direct literal reference - see
// the console.md entry on that dead end - meaning it's reached through
// an indexed/computed path, not a fixed instruction address this
// diagnostic could target directly). If, during a failure, this shows
// reads from an address OTHER than 0x10343970 within this page, that
// directly confirms address-computation corruption. If it only ever
// shows 0x10343970 with a correct value even during a failure, that
// would mean the corruption manifests even later - after the load
// itself completes correctly - narrowing things further still.
// FOUND AND FIXED A REAL BUG in this diagnostic's first version: the
// post-read check originally lived only in static_generate_memory_
// accessor() itself, right after static_generate_memory_rw() - but
// static_generate_fastram_accessor() (called just before that, for the
// common case of a plain resident RAM page like this one) issues its
// own UML_RET on the read path, returning before memory_accessor's own
// tail is ever reached. Confirmed live: this build crashed with
// "drcbe_x64::offset_from_rbp: delta out of range" the moment generated
// code first touched the scratch fields - caused by a DIFFERENT bug
// (see below), but investigating it is what surfaced the fastram-vs-
// memory_rw split and led to adding a second copy of the check directly
// inside static_generate_fastram_accessor's own read branch, right
// before ITS UML_RET - the actual path ordinary RAM reads take.
// SEPARATE BUG, fixed first: the scratch fields (diag_rodata_addr/
// value/hit) originally lived as plain file-scope C++ statics, not
// inside internal_mips3_state - the x64 DRC backend addresses mem()
// operands RBP-relative to that struct, and a free-standing global
// elsewhere in the binary's BSS landed out of the encodable
// displacement range. Moved into mips3.h's internal_mips3_state
// (diag_rodata_addr/value/hit fields) to fix.
// RESULT (2026-09-02, with both bugs fixed): ran the corrected build
// against a full matched-load run, 100 iterations, 21 real failures
// across 4 separate bursts (i=11-14, 36-41, 61-62, 70-78) - zero
// diagnostic events, confirmed via direct grep against logs/
// load_addr3_stdout.log after the run, not just the live stream.
// cc1's own compiled code never reads this specific VA range via any
// plain integer LW/LB/LH/LD-class instruction, not even during a
// confirmed failure where the exact error text was printed. Leading
// hypothesis for why (not yet tested): IRIX/SGI libc's optimized
// strlen/memcpy/puts-family routines are known to use FPU LOAD/STORE
// DOUBLE (LDC1/SDC1) for fast 8-byte-at-a-time copies - those go
// through a completely separate UML code-generation path in this file
// (COP1 load/store, not the integer static_generate_memory_accessor()
// this diagnostic hooks), so a read via that path would be invisible
// here. See wiki/console.md's task #84 section for the full writeup.
// #define MAME_SGI_RODATA_LOAD_ADDR_DIAG

// DISABLED (2026-07-29): the COMPILETIME(elocore_exl_N)/RAWWORDS fprintf
// pair below (generate_set_cop0_reg's COP0_Status case) was, like the
// probe family above, never gated - it fires unconditionally at compile
// time on every instruction touched within the 0x8801b600-0x8801b700
// range, including an 11-word memory dump per hit. Compile-time only
// (not per-execution), so bounded by recompile frequency rather than
// guest instruction count, but still a real, avoidable cost found while
// diffing this file against the pre-tonight snapshot in
// mame-mame0288-investigative-backup/.
// #define MAME_SGI_COMPILETIME_ELOCORE_DIAG

// TEMPORARY diagnostic (2026-07-31): investigating a recurring, reproducible
// full-DRC-speed boot hang at the same console point ("SETI@home config
// error"). PIT/IOC2 timer wiring already cleared as the cause (LOG_IRQS
// showed zero timer0_int/timer1_int activity during a confirmed stall).
// Live gdbstub samples during hangs kept landing in intr()/ecommon_allsaved
// and, critically, in mutex_lock/ktimer_switch - traced to real IRIX kernel
// source (irix/kern/os/ksync/mutex.c) rather than assumed: mutex_wait()'s
// `retry:` loop (called from mutex_lock on CAS failure) has a genuine
// adaptive spin-then-block path - resource_test_owner_pri() returning SPIN
// makes it DELAY(3) and retry via a direct backward branch. Disassembled the
// real compiled loop from the extracted unix_kernel ELF (mutex_wait has no
// exported symbol - traced via mutex_lock's own jal target) to get exact
// PCs: the SPIN-path DELAY(3) call is `jal us_delay` at 0x880eaca4, and the
// loop's own back-edge is the unconditional `b 0x880eac70` at 0x880eacac.
// That back-edge is a direct, provably-backward branch within one compiled
// sequence - the same shape the existing checkints generalization
// (MAME_DRC_BUGFIXES, targetpc < curdesc->pc) already covers - so this is
// checking whether that arming is actually reaching this exact loop live,
// not assuming it from the static shape alone. Two counters, dumped every 5
// real wall-clock seconds like the PIT_RATE_DIAG pattern: how often the SPIN
// path is actually taken (0x880eaca4), and how often intr()'s own dispatch
// loop is (re-)entered (0x88008a78) as a real-interrupt-delivery cross-
// check. Self-defined here (not via sgi.lua's defines{}) for the same
// scoping reason as MAME_DRC_BUGFIXES above - this file is part of the
// shared "optional" project, not the SGI driver's own.
// DISABLED (2026-07-31): job done for now - SPIN retries confirmed zero
// across four separate full-speed boots (one with a dramatic intr() storm,
// one unusually slow with no storm, two ordinary), so the mutex_wait
// adaptive-spin hypothesis is well negative-tested without a hang sample.
// The intr()-entry counter's own UML_CALLC hook fires on every real guest
// interrupt (seen up to ~500,000/5s during that storm) - real, non-trivial
// overhead disabled here specifically to keep the upcoming PIT-fix A/B
// timing comparison clean. Re-enable if a live hang needs catching again.
// #define MAME_SGI_MUTEXWAIT_SPIN_DIAG

// TEMPORARY diagnostic (2026-07-31): direct confirmation for the leading
// theory behind the recurring boot hang - irix/kern/io/pckm.c's global
// `pckm_mutex` (real kernel symbol, guest address 0x8832c6b8, confirmed via
// the extracted unix_kernel ELF's symbol table) is taken both by
// pckm_open() and by pckm_intr() - the real keyboard/mouse hardware
// interrupt handler itself. A kmutex acquired from interrupt context,
// contending with the same lock a normal thread (plausibly tfxd's
// device-open path, S99teleffect being the last rc2.d script and the
// trigger for every hang seen so far) can hold, is a textbook narrow
// self-deadlock hazard on this uniprocessor kernel. Already have strong
// retroactive evidence: an early hang sample today (PC=0x8802dbe0,
// a0=0x8832c6b8, gp=0x88332bf0) landed exactly on `jal mutex_lock` with
// a0 already loaded as &pckm_mutex, confirmed by disassembling that
// call site directly - this diagnostic is for a clean, deliberate
// confirmation on the next hang rather than resting on that alone.
// Two probes: a call counter at pckm's own mutex_lock call site
// (0x8802dbe0, the exact PC from the retroactive match), and a periodic
// (piggybacked on intr() entry, 0x88008a78, which keeps firing even
// during the hang per every sample captured today) raw read of
// pckm_mutex's own m_bits field (mutex_t's only field, sys/sema.h) -
// 0 means unlocked, nonzero means held, letting the diagnostic show
// directly whether it goes held-and-never-released right as a hang
// starts.
// DISABLED (2026-07-31): investigation resolved, real cause was
// ioc2_device's input_merger_device wiring, not pckm_mutex contention
// itself - see wiki/assets.md. Left in place, gated, in case future work
// needs it again.
// #define MAME_SGI_PCKM_MUTEX_DIAG

#include "emu.h"
#include "mips3.h"

#include "mips3com.h"
#include "mips3fe.h"
#include "mips3dsm.h"
#include "ps2vu.h"

#include "cpu/drcuml.h"
#include "cpu/drcumlsh.h"

#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <set>
#include <unordered_map>
#include <utility>

// TEMPORARY debug toggle (2026-08-02): compiles this file's HOST C++
// (the DRC compiler machinery itself - code_compile_block(),
// generate_sequence_instruction(), etc.) without optimization, purely to
// make host-gdb sessions reliable. Does NOT affect the JIT-generated
// guest native code this file emits at runtime - that still runs at
// full speed regardless, since it's produced by drcbex64.cpp's own
// codegen, unrelated to how this file's own source got compiled. Real,
// confirmed need: with -O3, a host-gdb session hit -O3-inlined STL
// frames ("this=<optimized out>", locals unresolvable) trying to break
// on a single line deep inside code_compile_block, even with full DWARF
// info present (see wiki/debugging.md) - a genuinely different failure
// mode than the earlier "no debug info at all" gap, needing a different
// fix. Undefined by default - only enable for a deliberate host-gdb
// investigation session, not as a standing default (compile-time DRC
// machinery runs measurably slower unoptimized, most noticeable during
// boot's heavy initial compile burst).
// #define MAME_SGI_HOST_DEBUG_NOOPT
#ifdef MAME_SGI_HOST_DEBUG_NOOPT
#pragma GCC optimize("O0")
#endif

// TEMPORARY diagnostic toggle (2026-08-06, re-enabled 2026-08-07 for the
// per-dispatch-runtime-check-cost sizing measurement) - jmpdest (JR/JALR)
// dispatch-key same-page hit rate, see diag_jmpdest_dispatch_total's
// comment in mips3.h. Off by default otherwise: real per-dispatch cost on
// the hottest path in the system (65M+ occurrences measured in a partial
// boot alone) - only worth paying for a deliberate re-measurement, not as
// a standing default.
// TEMPORARILY DISABLED (2026-08-17) - see the note above
// MAME_SGI_ERET_RACE_WINDOW_DIAG's own disable.
// #define MAME_SGI_JMPDEST_SAMEPAGE_DIAG

// TEMPORARY diagnostic toggle (2026-08-07) - validate_tlb() execution
// count, see diag_validate_tlb_total's comment in mips3.h. Paired with
// the toggle above for the same measurement; same off-by-default
// reasoning (real per-occurrence cost on a hot path).
// TEMPORARILY DISABLED (2026-08-17) - see the note above
// MAME_SGI_ERET_RACE_WINDOW_DIAG's own disable.
// #define MAME_SGI_VALIDATE_TLB_FREQ_DIAG

// TEMPORARY diagnostic toggle (2026-08-22) - resuming the 0x0fac5ffc
// validate_tlb() anomaly investigation (wiki/performance.md's
// "Investigated extensively... still not resolved" section). Runs an
// independent, unconditional C-side re-check on every dispatch through
// this one PC's validate_tlb site - see diag_0fac5ffc_compile_gen's
// comment in mips3.h and cfunc_0fac5ffc_anomaly_diag below. Scoped to a
// single, compile-time-known PC (not a general hot-path cost), so left
// enabled by default while this investigation is active.
// DISABLED (2026-08-22, later same session) - its question is fully
// answered (see wiki/performance.md), and its output would just add
// noise alongside MAME_SGI_VALIDATE_TLB_MISMATCH_CLASS_DIAG below,
// which generalizes the same measurement to every address. Re-enable
// if the 0x0fac5ffc-specific generation-staleness check is needed
// again.
// #define MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG

// TEMPORARY diagnostic (2026-09-16, task #97 QoS follow-up) - see
// diag_last_compile_instruction_count's comment in mips3.h. Self-defined
// here too (not just mips3.cpp) since this file compiles as part of the
// shared "optional" project, not the mame_sgi project sgi.lua's defines{}
// block scopes to - the two translation units don't share preprocessor
// state (see build-verify skill's own note on this exact gotcha). Must be
// defined identically in both files or one side's struct writes go to a
// field the other side never reads.
// #define MAME_SGI_COMPILE_LENGTH_CORRELATION_DIAG

// Fix (2026-09-17, task #97 QoS follow-up) - same cross-TU gotcha as
// above, needed here too now: code_compile_block()'s own abort_compilation
// catch block (below) times its internal code_flush_cache() call using
// this file's own MAME_SGI_STALL_DURATION_DIAG/MAME_SGI_STALL_DIAG_
// THRESHOLD_NS, mirroring mips3.cpp's two call sites exactly (same
// threshold value, same counter fields in internal_mips3_state) so this
// third, previously-unattributed flush mechanism reports through the same
// diag_flush_stall_* bucket instead of hiding inside diag_compile_stall_*/
// diag_compile_corr_*. See diag_last_compile_flush_ns's comment in
// mips3.h for the full mechanism this fixes.
// #define MAME_SGI_STALL_DURATION_DIAG
#define MAME_SGI_STALL_DIAG_THRESHOLD_NS 10'000'000ULL

// THE FIX (2026-08-22, task #8; reworked 2026-08-23) -
// static_generate_tlb_mismatch()'s "skip" branch, reached when the live
// TLB entry for this PC's page is mapped again by the time the handler
// runs, unconditionally EXITs to EXECUTE_MISSING_CODE. That throws away
// dispatch and pays a full describe_code() plus codegen pass even when a
// valid compiled block for this exact PC already exists in the hash
// table. This makes that branch redispatch through the hash table
// instead, with an anti-loop guard and an icount charge; see the
// handler itself for the full reasoning and the two brakes.
//
// The original 2026-08-22 form of this fix instead compared the live TLB
// entry against the value the dispatching instance had baked into its
// own validate_tlb CMP, on the theory that a transient unmap (IRIX
// doesn't use the hardware TLB Global bit for shared-library text, so
// mips3com_asid_changed() explicitly zeroes this process's own entry on
// every context switch away even though the physical page never changes)
// would have been refilled to the same value by the time the handler
// ran. That compare is unsatisfiable by construction and measured a
// literal 0% hit rate over several full boots - the conditional dispatch
// fires only when live != baked, and nothing executes between it and the
// handler's re-read of the same entry. Measurement also showed the
// validate_tlb fast path is only 3.8% of arrivals at that branch anyway;
// the compiler_page_fault() stub site is 96.2%.
//
// Toggle-numbered (not a plain #define) per this project's established
// convention for changes in this exact code region, which has already
// livelocked twice via two different structurally-unrelated approaches
// (see wiki/performance.md's full writeup) - keeps every state one line
// away without needing to rediscover this comment block to revert.
//
// Task #84 bisection (2026-09-02): tested undefined - 16/100 failures,
// within this project's own established run-to-run noise band, same as
// every other flag tested in this bisection. Restored to 1 (the normal
// state) - see wiki/console.md task #84 section for the result.
// Task #84 forced-trigger retest (2026-09-02): undefined and tested
// against the new 100%-reliable native-gdb repro trigger (not just the
// earlier ~30% natural rate) - 30/30 failures, no change. Restored to 1.
#define MAME_SGI_TLB_MISMATCH_REFRESH_FIX 1


// TEMPORARY diagnostic toggle (2026-08-22) - measuring what fraction of
// ALL validate_tlb() guard mismatches, system-wide (not just
// 0x0fac5ffc), fit the "transiently unmapped, live_value==0" pattern
// the proposed targeted refresh-not-recompile fix would catch, vs. a
// genuinely different (nonzero) live value that fix couldn't help with
// and would still need a real recompile. Direct answer to "what
// percentage of the recompiles would solution 2 catch" before
// committing to implementing it. Unconditional per-dispatch C call
// across every address using the shared-key fast path - real overhead,
// only meant to run for one measurement boot, not left on.
// DISABLED (2026-08-23) - task #8 concluded and this measurement
// question fully answered (96.2%/3.8%/0.0% call-site split, see
// wiki/performance.md's "tlb_mismatch handler redispatch" section);
// was still costing ~199M unconditional C calls/boot. Re-enable only
// if that split needs re-measuring after some other change to the
// TLB-mismatch paths.
// #define MAME_SGI_VALIDATE_TLB_MISMATCH_CLASS_DIAG

// TEMPORARY diagnostic (2026-08-23) - keyboard-drop investigation
// (Bug B, see wiki/console.md): msnat.cpp's MAME_SGI_KBD_SCAN_RATE_DIAG
// found a single ~354ms real-time gap where the embedded keyboard-
// controller's own matrix scan didn't run at all - this correlates that
// against DRC compile activity to check whether a recompile burst is
// the actual cause of the freeze. Counts compiles per real second and
// reports periodically, same wall-clock HH:MM:SS.mmm timestamp format
// as the keyboard-side diagnostics so the two logs can be lined up by
// eye directly. DISABLED (2026-08-23) - answered its question (a
// drc_cache flush landed in the same one-second window as the worst
// scan gap - see wiki/console.md's Bug B writeup); task #75
// reopened-investigation paused.
// #define MAME_SGI_DRC_COMPILE_RATE_DIAG

#ifdef MAME_SGI_DRC_COMPILE_RATE_DIAG
namespace {
std::string drc_compile_rate_timestamp()
{
	using namespace std::chrono;
	auto const now = system_clock::now();
	auto const t = system_clock::to_time_t(now);
	auto const ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
	std::tm tm_buf;
	localtime_r(&t, &tm_buf);
	char buf[16];
	std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, int(ms.count()));
	return buf;
}
}
#endif

// TEMPORARY diagnostic (2026-08-23) - FPU-corruption investigation (see
// wiki/console.md's "%f corruption under host load" section): a real,
// reproducible ~ASCII-10-offset digit corruption in guest printf("%f")
// output, seen on both the serial console and the GUI winterm (ruling
// out a serial-transmission-specific cause), correlated by the user to
// heavy I/O-interrupt/syscall activity (`ls -R /usr`). Task #69/#71
// already found and fixed ONE bug in the exact same general mechanism -
// IRIX's lazy FPU context-restore convention (kernel clears SR.CU1 on a
// context switch, expects a COP1-unusable trap on the next FP
// instruction to trigger the real restore) was silently unenforced,
// MIPS3DRC_STRICT_COP1 (mips3drc.cpp:7523) fixed it for that specific
// case and is confirmed still active. This diagnostic logs every
// EXCEPTION_BADCOP delivery specifically for cop==1 (COP1, not COP0 -
// see static_generate_exception's own GETEXP of the exception
// parameter) with a wall-clock timestamp, same format as the keyboard-
// side diagnostics, to check whether the STILL-unexplained corruption
// correlates with trap-firing frequency/timing under load - i.e.
// whether this is a subtler, load-dependent edge case in the same
// general mechanism rather than a regression of the already-fixed one.
// Was per-event-print, off-by-default; now always counts (cheap) and
// reports once as a clean-exit summary in mips3_device::device_stop()
// instead (2026-09-11) - see diag_cop1_trap_count's comment in mips3.h.
#define MAME_SGI_COP1_TRAP_DIAG

#ifdef MAME_SGI_COP1_TRAP_DIAG
void mips3_device::func_cop1_trap_diag()
{
	// The generated code already gated this call on cop==1 (see the call
	// site's own comment) - just record the event; func_*/cfunc_* split
	// per this file's usual convention since m_core is protected and
	// cfunc_* (below) is a plain C trampoline for UML_CALLCc.
	m_core->diag_cop1_trap_count++;
}

static void cfunc_cop1_trap_diag(void *param)
{
	((mips3_device *)param)->func_cop1_trap_diag();
}
#endif

// TEMPORARY diagnostic toggle (2026-08-07) - compiled-sequence length
// distribution, see diag_seq_count's comment in mips3.h. Cheap (counted
// once per compile, not per dispatch).
// TEMPORARILY DISABLED (2026-08-17) - see the note above
// MAME_SGI_ERET_RACE_WINDOW_DIAG's own disable.
// #define MAME_SGI_SEQUENCE_LENGTH_DIAG

// TEMPORARY diagnostic toggle (2026-08-08) - per-call-site inline-cache
// hit-rate probe at the jmpdest HASHJMP site, see
// m_diag_callsite_ic_state's comment in mips3.h. NOT cheap like the
// counters above - this is a real UML_CALLC (full C++ call + hash-map
// lookup) on every jmpdest dispatch (~115M/boot partial-boot measurement,
// 2026-08-07), specifically to answer one question before committing to
// the self-patching/inline-caching redesign (journal.md 2026-08-08): does
// a 1-slot per-call-site cache actually hit, or does JR $ra (polymorphic
// return, ~26% of indirect_jump terminations) thrash it. Wall time under
// this diagnostic is expected to be significantly distorted (same
// methodology caveat as -debug) - only the hit/miss ratio is meaningful,
// not absolute timing. Off by default.
// TEMPORARILY DISABLED (2026-08-17) - see the note above
// MAME_SGI_ERET_RACE_WINDOW_DIAG's own disable.
// #define MAME_SGI_CALLSITE_IC_HITRATE_DIAG

// PROTOTYPE toggle (2026-08-08) - the guard-based per-call-site inline
// cache at the jmpdest site, see the class-scope comment near
// m_diag_callsite_ic_state in mips3.h for the full design and history
// (two earlier redesigns - a per-call-site heap allocation, then a
// UML_CALLH + runtime byte-patching scheme found genuinely unsafe via a
// real guest-level crash - both preceded this one). Current design uses
// a new backend primitive, OP_DIRECTJMP (uml.h/drcbex64.cpp), to jump
// directly through a per-slot memory address with no table walk and no
// byte patching - the rsp-reset invariant that made byte-patching unsafe
// is handled correctly inside the new op itself, mirroring op_hashjmp's
// own preamble. See journal.md 2026-08-08.
// #define MAME_SGI_CALLSITE_IC_PROTOTYPE

// TEMPORARY diagnostic (2026-08-17, task #69/#71) - three independent
// binary-level techniques (a conditioned host watchpoint, a manual
// disassembly-text search, and an exhaustive byte-level immediate
// scan via mips3_device::debug_scan_for_immediate - see mips3.h) all
// agree that the compiled block covering wshApp::mainLoop contains no
// instruction anywhere touching $f20's memory slot, despite LWC1's own
// codegen (see its case below) unconditionally emitting
// UML_MOV(block, FPR32(RTREG), I0) with no gating found anywhere in
// generate_delay_slot_and_branch/generate_sequence_instruction/
// generate_opcode's call chain. This probe checks the other side of
// that contradiction directly: is generate_opcode ever actually
// *reached* for this exact delay-slot desc during compilation at all,
// and does it correctly dispatch to the LWC1 case and reach the
// UML_MOV call - rather than continuing to infer this from source
// reading alone. Fires at compile time (not runtime), gated to one
// fixed guest PC, cheap - see the four checkpoints this gates in
// generate_sequence_instruction/generate_opcode/the LWC1 case itself.
// RE-ENABLED (2026-08-17): confirmed (A/B, probe on vs off) that this
// probe's real fopen/fclose I/O during compile events is the trigger
// for a desktop black-to-white text/icon-color regression - almost
// certainly via timing perturbation exposing a pre-existing latent
// race (see MAME_SGI_COLOR_SRC_DIAG in newport.cpp), not direct state
// corruption (this probe is host-side only, wraps but doesn't alter
// the real UML emission - verified). Re-enabled together with that
// new diagnostic specifically so continuing $f20 work can passively
// catch and confirm the color-race mechanism as a side effect, without
// a dedicated repro session.
// DISABLED (2026-08-17) - task #69/#71 concluded. The store this probe
// was built to check turned out to be correctly emitted all along (the
// investigation's byte-level scans were reading transient DRC cache
// memory well after compile-time, which churns constantly - not a real
// codegen omission); the real root cause was MIPS3DRC_STRICT_COP1 never
// being enabled (see drc-bugfixes.md). Commented out, not deleted.
// #define MAME_SGI_LWC1_DELAYSLOT_PROBE
#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
#define MAME_SGI_LWC1_DELAYSLOT_PROBE_PC 0x10024d2c

// Writes to its own dedicated file rather than stderr - a real, confirmed
// gotcha found live 2026-08-17: this project's launch scripts redirect
// mamesgi's stdout (very high volume from -verbose) and stderr to the
// SAME file via `2>&1`. Buffered stdout writes and this probe's
// unbuffered-but-still-independent stderr writes share one underlying
// fd/file-offset without coordinated locking - a first attempt logging
// via fprintf(stderr,...)+fflush(stderr) produced a real, reproducible
// capture once (confirmed via decoded MIPS instruction bytes) that was
// then completely gone (zero byte-level occurrences, verified via a
// raw Python byte search, not just grep) by the next check, moments
// later, with the file's mtime/size having moved on - a silent
// overwrite race, not a grep artifact or a dead-inode issue (the
// process's own fd was independently confirmed still live and
// correctly named). A dedicated file with its own independent fd
// sidesteps this entirely.
static void lwc1_probe_log(const char *fmt, ...)
{
	FILE *f = fopen("/home/blythe/mame-sgi/lwc1_probe.log", "a");
	if (!f)
		return;
	va_list args;
	va_start(args, fmt);
	vfprintf(f, fmt, args);
	va_end(args);
	fclose(f);
}

// Set when this exact gated UML_MOV is emitted into the block currently
// being built by the frontend; checked (and cleared) by drcbex64.cpp's
// generate() when that same block is closed out, to log the block's real
// native address/size - avoids reconstructing a (mode,pc) cache key and
// guessing at get_codeptr(), the technique that turned out to be
// resolving to the nocode_point miss-fallback stub for this whole
// investigation (see MAME_SGI_LWC1_BLOCK_ADDR_PROBE in drcbex64.cpp).
// Plain extern global rather than a shared header - mips3drc.cpp and
// drcbex64.cpp both link into the same "optional" static library, so
// file-scope extern linkage resolves fine without a new header.
bool g_mame_sgi_lwc1_target_block = false;
#endif

// TEMPORARY diagnostic (2026-08-24) - MXCSR-desync investigation
// (fcvt_r/%f leading-digit corruption, wiki/performance.md and
// journal.md). Scheduler-level checkpoints (before/after exec->run(),
// before/after every named timer callback) found zero hits despite
// confirmed ongoing corruption during the same window - too coarse
// relative to the actual vulnerable window. This checks at every single
// guest instruction boundary instead: compares the real, live host MXCSR
// rounding-control bits (read via the repurposed GETFMOD in drcbex64.cpp,
// see MAME_SGI_GETFMOD_REAL_MXCSR_DIAG there) against what CCR1(31)'s
// guest RM field predicts. The inline compare (getfmod+shr+and+and+load+
// cmp+jmpc) runs unconditionally per instruction but only escalates to a
// CALLC on an actual mismatch, keeping the steady-state cost to a handful
// of x86 instructions rather than a full C call - real overhead on a very
// hot path, only meant to run for one diagnostic boot, not left on.
// DISABLED (2026-08-24, same day) - this per-guest-instruction check adds
// enough overhead (unconditional stmxcsr+shr+and+and+load+cmp+jmpc on
// every single instruction) to measurably perturb host-thread timing:
// with it active, mxcsrtest ran 34M iterations with bad=0, where the same
// setup previously found corruption within a couple thousand iterations
// at real speed. The probe was suppressing the very race it was built to
// catch. Superseded by the scheduler-level record-and-compare check in
// schedule.cpp (MAME_SGI_SCHEDULER_MXCSR_DIAG), which checks far less
// often and shouldn't have the same effect. Kept here, not deleted, in
// case per-instruction granularity is needed again once the scheduler
// check either finds the culprit or narrows things down further.
// #define MAME_SGI_MXCSR_DESYNC_DIAG
#ifdef MAME_SGI_MXCSR_DESYNC_DIAG
// MIPS FCSR RM field (CCR1(31) bits 1:0) -> expected x86 MXCSR RC field
// (bits 14:13), derived from drcbex64.cpp's sse_control[] table (indexed
// by uml::float_rounding_mode: TRUNC=0/RC=3, ROUND=1/RC=0, CEIL=2/RC=2,
// FLOOR=3/RC=1) composed with mips3.cpp's fpmode_source[] table (indexed
// by RM: 0->ROUND_ROUND, 1->ROUND_TRUNC, 2->ROUND_CEIL, 3->ROUND_FLOOR).
static const uint8_t s_mxcsr_desync_expected_rc[4] = { 0, 3, 2, 1 };
#endif

// TEMPORARY diagnostic (2026-08-24, same day) - narrower replacement for
// both of the above. The scheduler-level check (schedule.cpp) and the
// execute_run() DRC-loop check (mips3.cpp) both compare the real MXCSR
// against its own last-recorded value, which can only ever detect an
// UNEXPECTED write - real corruption was confirmed occurring in the same
// window both checks ran clean in, which is exactly what you'd see if the
// actual bug is a MISSING write: op_setfmod's cache (m_state.fmod)
// believing a real ldmxcsr isn't needed when it actually is, in which
// case nothing writes to MXCSR at all and it simply sits on its prior
// (now-wrong) value forever after - invisible to a "did it change" check,
// only visible to a check against independent ground truth (what the
// guest's own CTC1 write just asked for). Checks immediately after every
// UML_SETFMOD this CTC1 case emits - far rarer than every instruction
// (guest code doesn't change rounding mode continuously), so shouldn't
// perturb timing the way the per-instruction version did, while directly
// targeting the suspected skip-optimization failure mode. See wiki/
// performance.md and journal.md for the full investigation.
// DISABLED (2026-09-01) - investigation concluded. Root cause was the DRC
// entry point never resyncing m_state.fmod (see MAME_SGI_FMOD_ENTRY_RESYNC_FIX
// in drcbex64.cpp, the actual permanent fix), not this CTC1 site - this
// check's own "zero mismatches" result was correct but pointed at the wrong
// SETFMOD call site (this one can never stale-match; the entry-point one
// can). See wiki/drc-bugfixes.md section D.
// #define MAME_SGI_CTC1_MXCSR_CHECK_DIAG
#ifdef MAME_SGI_CTC1_MXCSR_CHECK_DIAG
// Indexed by fmod (uml::float_rounding_mode: TRUNC=0,ROUND=1,CEIL=2,
// FLOOR=3 - what I0 holds at the UML_SETFMOD call site below), giving the
// real x86 MXCSR RC bits (14:13) that value should produce - read
// directly off drcbex64.cpp's own sse_control[] table (0x7f80/0x1f80/
// 0x5f80/0x3f80 -> RC 3/0/2/1).
static const uint8_t s_ctc1_mxcsr_expected_rc[4] = { 3, 0, 2, 1 };
#endif

// TEMPORARY diagnostic (2026-08-24, same day) - the CTC1-scoped check above
// found zero mismatches despite confirmed ongoing corruption: every guest
// rounding-mode change is verified to take effect on the real MXCSR
// exactly when requested. So the corruption must be happening strictly
// between a verified-correct CTC1 and the specific FP arithmetic that
// depends on it - IRIX's _mixed_dtoa() magic-truncation trick is just
// `d = dm * 0.1; d = d + magic; d = d - magic;`, a multiply immediately
// followed by an add and a subtract. Checks real MXCSR against guest
// ground truth immediately before ADD.S/D, SUB.S/D, and MUL.S/D - the
// three ops that trick actually uses - narrowing the window from "the
// whole guest instruction stream" to "just FP arithmetic ops", which
// should still be far less frequent than every single instruction. See
// wiki/performance.md and journal.md for the full investigation.
// DISABLED (2026-09-01) - investigation concluded, real root cause found
// and fixed elsewhere (MAME_SGI_FMOD_ENTRY_RESYNC_FIX, drcbex64.cpp). This
// check also turned out to Heisenbug on its own repro workload (mxcsrtest's
// inner loop is almost entirely the three ops it checks) - see
// wiki/drc-bugfixes.md section D for the full writeup.
// #define MAME_SGI_FPOP_MXCSR_CHECK_DIAG
#ifdef MAME_SGI_FPOP_MXCSR_CHECK_DIAG
// Indexed by guest RM (CCR1(31)&3), giving the expected real x86 MXCSR RC
// bits - same mapping as s_mxcsr_desync_expected_rc above, duplicated
// under this macro's own name so the two diagnostics stay independently
// toggleable.
static const uint8_t s_fpop_mxcsr_expected_rc[4] = { 0, 3, 2, 1 };
#endif


/* Use with STRICT_VERIFY to print debug info to console for extra validation checks */
/* Set to 1 to activate and use MIPS3DRC_STRICT_VERIFY in the drc options */
#define DEBUG_STRICT_VERIFY 0


/***************************************************************************
    MACROS
***************************************************************************/

#define IS_FR0      ((m_core->mode & 1) == 0)
#define IS_FR1      ((m_core->mode & 1) != 0)

#define R32(reg)                m_regmaplo[reg]
#define LO32                    R32(REG_LO)
#define HI32                    R32(REG_HI)
#define CPR032(reg)             uml::mem(LOPTR(&m_core->cpr[0][reg]))
#define CCR032(reg)             uml::mem(LOPTR(&m_core->ccr[0][reg]))
#define FPR32(reg)              uml::mem((IS_FR0) ? &((float *)&m_core->cpr[1][reg & 0x1E])[BYTE_XOR_LE(reg & 1)] : (float *)&m_core->cpr[1][reg])
#define CCR132(reg)             uml::mem(LOPTR(&m_core->ccr[1][reg]))
#define CPR232(reg)             uml::mem(LOPTR(&m_core->cpr[2][reg]))
#define CCR232(reg)             uml::mem(LOPTR(&m_core->ccr[2][reg]))

#define R64(reg)                m_regmap[reg]
#define LO64                    R64(REG_LO)
#define HI64                    R64(REG_HI)
#define CPR064(reg)             uml::mem(&m_core->cpr[0][reg])
#define CCR064(reg)             uml::mem(&m_core->ccr[0][reg])
#define FPR64(reg)              uml::mem((IS_FR0) ? (double *)&m_core->cpr[1][reg & 0x1E] : (double *)&m_core->cpr[1][reg])
#define CCR164(reg)             uml::mem(&m_core->ccr[1][reg])
#define CPR264(reg)             uml::mem(&m_core->cpr[2][reg])
#define CCR264(reg)             uml::mem(&m_core->ccr[2][reg])

#define FCCSHIFT(which)         fcc_shift[(m_flavor < MIPS3_TYPE_MIPS_IV) ? 0 : ((which) & 7)]
#define FCCMASK(which)          (uint32_t(1 << FCCSHIFT(which)))



/***************************************************************************
    FUNCTION PROTOTYPES
***************************************************************************/

static void cfunc_printf_exception(void *param);
static void cfunc_get_cycles(void *param);
static void cfunc_printf_probe(void *param);
static void cfunc_debug_break(void *param);
#ifdef MAME_SGI_CALLSITE_IC_PROTOTYPE
static void cfunc_ic_learn_and_arm(void *param);
#endif


/***************************************************************************
    PRIVATE GLOBAL VARIABLES
***************************************************************************/

/* bit indexes for various FCCs */
static const uint8_t fcc_shift[8] = { 23, 25, 26, 27, 28, 29, 30, 31 };


/***************************************************************************
    INLINE FUNCTIONS
***************************************************************************/

/*-------------------------------------------------
    alloc_handle - allocate a handle if not
    already allocated
-------------------------------------------------*/

static inline void alloc_handle(drcuml_state &drcuml, uml::code_handle *&handleptr, const char *name)
{
	if (!handleptr)
		handleptr = drcuml.handle_alloc(name);
}


/*-------------------------------------------------
    load_fast_iregs - load any fast integer
    registers
-------------------------------------------------*/

inline void mips3_device::load_fast_iregs(drcuml_block &block)
{
	int regnum;

#ifdef MAME_SGI_TASK84_SAVELOAD_BALANCE_DIAG
	UML_ADD(block, mem(&m_core->diag_task84_load_count), mem(&m_core->diag_task84_load_count), 1);
#endif
	for (regnum = 0; regnum < std::size(m_regmap); regnum++)
		if (m_regmap[regnum].is_int_register())
			UML_DMOV(block, ireg(m_regmap[regnum].ireg() - REG_I0), mem(&m_core->r[regnum]));
}


/*-------------------------------------------------
    save_fast_iregs - save any fast integer
    registers
-------------------------------------------------*/

inline void mips3_device::save_fast_iregs(drcuml_block &block)
{
	int regnum;

#ifdef MAME_SGI_TASK84_SAVELOAD_BALANCE_DIAG
	UML_ADD(block, mem(&m_core->diag_task84_save_count), mem(&m_core->diag_task84_save_count), 1);
#endif
	for (regnum = 0; regnum < std::size(m_regmap); regnum++)
		if (m_regmap[regnum].is_int_register())
			UML_DMOV(block, mem(&m_core->r[regnum]), ireg(m_regmap[regnum].ireg() - REG_I0));

#ifdef MAME_SGI_TASK84_SAVE_RELOAD_AUDIT
	// Shadow whatever we just flushed. Deliberately a memory->memory copy of
	// the SAVED value rather than a second read of I4, so this records
	// exactly what landed in r[2] even if the loop above ever changes.
	// drcbex64's op_mov routes a mem->mem move through rax (volatile
	// scratch) and touches no UML register, so this is safe at every one of
	// the 9 call sites - including static_generate_exception's, where I0 is
	// still live (it holds the exception vector address for the HASHJMP that
	// immediately follows).
	UML_DMOV(block, mem(&m_core->diag_task84_last_saved_r2), mem(&m_core->r[2]));
#endif
}



/***************************************************************************
    CORE CALLBACKS
***************************************************************************/

/*-------------------------------------------------
    mips3drc_set_options - configure DRC options
-------------------------------------------------*/

void mips3_device::mips3drc_set_options(uint32_t options)
{
	if (!allow_drc()) return;
	m_drcoptions = options;
}

/*-------------------------------------------------
    mips3drc_get_options - return current DRC options
-------------------------------------------------*/

uint32_t mips3_device::mips3drc_get_options()
{
	return m_drcoptions;
}

/*-------------------------------------------------
    mips3drc_clears_fastram - clears fastram
    region starting at index select_start
-------------------------------------------------*/
void mips3_device::clear_fastram(uint32_t select_start)
{
	m_fastram_select=select_start;
	// Set cache to dirty so that re-mapping occurs
	m_drc_cache_dirty = true;
}

/*-------------------------------------------------
    mips3drc_add_fastram - add a new fastram
    region
-------------------------------------------------*/

void mips3_device::add_fastram(offs_t start, offs_t end, uint8_t readonly, void *base)
{
	if (m_fastram_select < std::size(m_fastram))
	{
		m_fastram[m_fastram_select].start = start;
		m_fastram[m_fastram_select].end = end;
		m_fastram[m_fastram_select].readonly = readonly;
		m_fastram[m_fastram_select].base = base;
		m_fastram[m_fastram_select].offset_base8 = (uint8_t*)base - start;
		m_fastram[m_fastram_select].offset_base16 = (uint16_t*)((uint8_t*)base - start);
		m_fastram[m_fastram_select].offset_base32 = (uint32_t*)((uint8_t*)base - start);
		m_fastram_select++;
		// Set cache to dirty so that re-mapping occurs
		m_drc_cache_dirty = true;
	}
}


/*-------------------------------------------------
    mips3drc_add_hotspot - add a new hotspot
-------------------------------------------------*/

void mips3_device::mips3drc_add_hotspot(offs_t pc, uint32_t opcode, uint32_t cycles)
{
	if (!allow_drc()) return;
	if (m_hotspot_select < std::size(m_hotspot))
	{
		m_hotspot[m_hotspot_select].pc = pc;
		m_hotspot[m_hotspot_select].opcode = opcode;
		m_hotspot[m_hotspot_select].cycles = cycles;
		m_hotspot_select++;
	}
}



/***************************************************************************
    CACHE MANAGEMENT
***************************************************************************/

/*-------------------------------------------------
    code_flush_cache - flush the cache and
    regenerate static code
-------------------------------------------------*/

void mips3_device::code_flush_cache()
{
	int mode;

	/* empty the transient cache contents */
	m_drcuml->reset();

	try
	{
		{
			drcuml_block &block(m_drcuml->begin_block(1024));
			int label = 1;

			/* generate the entry point and out-of-cycles handlers */
			static_generate_entry_point(block, label);
			static_generate_nocode_handler(block, label);
			static_generate_out_of_cycles(block, label);
			static_generate_tlb_mismatch(block, label);

			/* append exception handlers for various types */
			static_generate_exception(block, label, EXCEPTION_INTERRUPT,     true,  "exception_interrupt");
			static_generate_exception(block, label, EXCEPTION_INTERRUPT,     false, "exception_interrupt_norecover");
			static_generate_exception(block, label, EXCEPTION_TLBMOD,        true,  "exception_tlbmod");
			static_generate_exception(block, label, EXCEPTION_TLBLOAD,       true,  "exception_tlbload");
			static_generate_exception(block, label, EXCEPTION_TLBSTORE,      true,  "exception_tlbstore");
			static_generate_exception(block, label, EXCEPTION_TLBLOAD_FILL,  true,  "exception_tlbload_fill");
			static_generate_exception(block, label, EXCEPTION_TLBSTORE_FILL, true,  "exception_tlbstore_fill");
			static_generate_exception(block, label, EXCEPTION_ADDRLOAD,      true,  "exception_addrload");
			static_generate_exception(block, label, EXCEPTION_ADDRSTORE,     true,  "exception_addrstore");
			static_generate_exception(block, label, EXCEPTION_SYSCALL,       true,  "exception_syscall");
			static_generate_exception(block, label, EXCEPTION_BREAK,         true,  "exception_break");
			static_generate_exception(block, label, EXCEPTION_INVALIDOP,     true,  "exception_invalidop");
			static_generate_exception(block, label, EXCEPTION_BADCOP,        true,  "exception_badcop");
			static_generate_exception(block, label, EXCEPTION_OVERFLOW,      true,  "exception_overflow");
			static_generate_exception(block, label, EXCEPTION_TRAP,          true,  "exception_trap");
			static_generate_exception(block, label, EXCEPTION_FPE,           true,  "exception_fpe");

			block.end();
		}

#ifdef MAME_SGI_CALLSITE_IC_PROTOTYPE
		// PROTOTYPE (2026-08-08): reset every per-call-site inline-cache
		// slot's guard expectation to the sentinel - covers both every
		// later flush AND the very first, pre-flush setup, since this
		// whole function runs unconditionally once before any dynamic
		// compile (execute_run(), mips3.cpp). No handles to (re)arm with
		// the OP_DIRECTJMP design - see the class-scope comment near
		// m_diag_callsite_ic_state (mips3.h) for the full design/history.
		// ic_site_target doesn't strictly need resetting (func_ic_learn_
		// and_arm() only ever reads/writes it together with committing a
		// non-sentinel expected_pc, so a stale value can never be
		// observed - see that function's own comment) but zeroing it
		// anyway costs nothing and makes a bug in that invariant fail
		// loudly (a jump through nullptr) instead of silently.
		for (int i = 0; i < MAME_SGI_IC_SITE_COUNT; i++)
		{
			m_core->ic_site_expected_mode[i] = 0xFFFFFFFFu;
			m_core->ic_site_expected_pc[i] = 0xFFFFFFFFu;
			m_core->ic_site_target[i] = 0;
		}
#endif

		/* add subroutines for memory accesses */
		for (mode = 0; mode < 3; mode++)
		{
			drcuml_block &block(m_drcuml->begin_block(1024));
			int label = 1;
			static_generate_memory_accessor(block, label, mode, 1, false, false, "read8",       m_read8[mode]);
			static_generate_memory_accessor(block, label, mode, 1, true,  false, "write8",      m_write8[mode]);
			static_generate_memory_accessor(block, label, mode, 2, false, false, "read16",      m_read16[mode]);
			static_generate_memory_accessor(block, label, mode, 2, true,  false, "write16",     m_write16[mode]);
			static_generate_memory_accessor(block, label, mode, 4, false, false, "read32",      m_read32[mode]);
			static_generate_memory_accessor(block, label, mode, 4, false, true,  "read32mask",  m_read32mask[mode]);
			static_generate_memory_accessor(block, label, mode, 4, true,  false, "write32",     m_write32[mode]);
			static_generate_memory_accessor(block, label, mode, 4, true,  true,  "write32mask", m_write32mask[mode]);
			static_generate_memory_accessor(block, label, mode, 8, false, false, "read64",      m_read64[mode]);
			static_generate_memory_accessor(block, label, mode, 8, false, true,  "read64mask",  m_read64mask[mode]);
			static_generate_memory_accessor(block, label, mode, 8, true,  false, "write64",     m_write64[mode]);
			static_generate_memory_accessor(block, label, mode, 8, true,  true,  "write64mask", m_write64mask[mode]);
			block.end();
		}
	}
	catch (drcuml_block::abort_compilation &)
	{
		fatalerror("Unrecoverable error generating static code\n");
	}
}


/*-------------------------------------------------
    code_compile_block - compile a block of the
    given mode at the specified pc
-------------------------------------------------*/

void mips3_device::code_compile_block(uint32_t mode, offs_t pc)
{
	compiler_state compiler = { 0 };
	const opcode_desc *seqhead, *seqlast;
	const opcode_desc *codelast;
	const opcode_desc *desclist;
	bool override = false;

#ifdef MAME_SGI_TASK84_DRC_EVENT_LOG
	log_drc_event("COMPILE", pc);
#endif

	// Standing feature (2026-08-07, not a temporary diagnostic): report
	// drc_cache health (flush count, permanent-cache utilization) the
	// moment a new flush happens, not only when the process exits
	// cleanly. A plain kill -9 - this project's usual way of ending a
	// test run - skips drc_cache's own destructor entirely, so a real
	// cache-thrashing regression can run for an entire session without
	// ever becoming visible; this is exactly what happened investigating
	// the ASID_FIX==2 regression (journal.md 2026-08-07: 997 flushes by
	// /.oldtmp vs 7 for the same milestone with the mechanism disabled -
	// found only via a special clean-exit run, not from any routine
	// boot test's log). Checking flush_count() here is cheap (one
	// comparison per compile) and self-rate-limits: a healthy run flushes
	// rarely, so this stays near-silent; a thrashing one reports every
	// single flush as it happens.
	{
		static uint32_t s_last_flush_count = 0;
		uint32_t const current_flush_count = m_drcuml->cache().flush_count();
		if (current_flush_count != s_last_flush_count)
		{
			m_drcuml->cache().print_statistics();
			s_last_flush_count = current_flush_count;
		}
	}

#ifdef MAME_SGI_DRC_COMPILE_RATE_DIAG
	{
		using namespace std::chrono;
		static uint64_t s_compiles_this_period = 0;
		static uint64_t s_total_compiles = 0;
		static double s_next_report_ms = 0.0;
		double const now_ms = duration<double, std::milli>(system_clock::now().time_since_epoch()).count();
		s_compiles_this_period++;
		s_total_compiles++;
		if (s_next_report_ms == 0.0)
			s_next_report_ms = now_ms + 1000.0;
		if (now_ms >= s_next_report_ms)
		{
			osd_printf_info("MAME_SGI_DRC_COMPILE_RATE_DIAG: %s compiles_this_second=%llu total_compiles=%llu\n",
				drc_compile_rate_timestamp().c_str(),
				(unsigned long long)s_compiles_this_period, (unsigned long long)s_total_compiles);
			s_compiles_this_period = 0;
			s_next_report_ms = now_ms + 1000.0;
		}
	}
#endif

// Standing counter report for MAME_SGI_TLB_MISMATCH_REFRESH_FIX (task #8,
// diag_tlb_mismatch_skip_total/refresh_hits, see their comment in mips3.h)
// used to live here as a self-rate-limited live print every time the
// totals changed meaningfully. Moved to a single clean-exit summary in
// mips3_device::device_stop() instead (2026-09-11), matching the same
// hygiene convention applied to MAME_SGI_COP1_TRAP_DIAG the same day -
// the fix itself (the counters' increments, in static_generate_tlb_
// mismatch()) is untouched, only this reporting was live/periodic.

#ifdef MAME_DRC_USERMODE_RECOMPILE_DIAG
	// TEMPORARY diagnostic (2026-08-01, UPDATED same day): `mode` here is
	// now m_core->dispatch_mode (the ASID-aware composite, see
	// internal_mips3_state's comment on that field and mips3.cpp's
	// execute_run() call site) - NOT the plain privilege+FR encoding this
	// check originally assumed. Under the composite: bit0=FR, bit1=is_user
	// (only meaningful when non-kernel), bits2+ = asid-component (0 means
	// kernel sentinel). A genuine USER-mode composite is non-kernel
	// (bits2+ nonzero) AND has the is_user bit set - `(mode>>1)==MODE_USER`
	// (the original check, written for the old plain encoding) can never
	// be true under the new one, which silently produced zero recorded
	// entries on the first post-fix test run rather than a visible error -
	// caught by the diagnostic dump being unexpectedly empty, not by a
	// build/runtime failure.
	if (((mode >> 2) != 0) && ((mode & 0x02) != 0))
	{
		// Fix (2026-09-17, task #97 QoS follow-up) - see this macro's own
		// enable comment above for the full story: this map's incremental
		// rehashing (libstdc++ growing its bucket array, observed reaching
		// 351,061 buckets mid-boot via a live gdb backtrace) was the exact
		// mechanism causing this diagnostic's real ~11-13ms tail-latency
		// cost, despite genuinely being cheap on an amortized-per-insert
		// basis. Reserving a large bucket count on first use converts that
		// into one upfront, predictable allocation instead of several
		// scattered, unpredictable rehash stalls through the boot - same
		// total work, but paid once instead of as periodic surprises.
		// 500,000 is comfortably above the observed 351,061-bucket rehash
		// point, with headroom for a heavier session; if a given boot still
		// exceeds it, this just falls back to ordinary (rarer) rehashing,
		// same as before this fix - reserve() is a mitigation of frequency/
		// predictability, not a hard cap.
		if (m_diag_usermode_recompile_counts.empty())
			m_diag_usermode_recompile_counts.reserve(500'000);
		m_diag_usermode_recompile_counts[(uint64_t(mode) << 32) | uint32_t(pc)]++;
	}
	if ((mode >> 2) == 0)
		m_diag_kernel_dispatch_mode_values.insert(mode);
#endif

#ifdef MAME_SGI_TASK84_CPPGETTOKEN_ASID_DIAG
	// TEMPORARY diagnostic (2026-09-05, task #84): does cpp_get_token's
	// page actually get MAME_SGI_DISPATCH_MODE_ASID_FIX's shared,
	// ASID-independent dispatch key in practice, or the full ASID-scoped
	// one? dispatch_page_writable(pc) (mips3.h:1265) is the single
	// choke point that decides this - false means shared key (this
	// page's interior direct-entry hash registrations are then visible
	// to every process/context that ever dispatches through them, not
	// private per-caller). Read-only: dispatch_page_writable() is an
	// existing, already-safe accessor called on every compile anyway;
	// this only adds a log line, no behavior change.
	// WIDENED (2026-09-05, same day): the exact-entry gate (pc==0x10076008)
	// never fired despite MAME_SGI_TASK84_CPPGETTOKEN_COMPILE_TIMESTAMP
	// (drcbex64.cpp) proving a fresh compile containing a HASH marker for
	// 0x10076008 genuinely happens - that diagnostic scans for the marker
	// ANYWHERE inside a generated block's instruction list, not this
	// function's own top-level pc argument, so 0x10076008 apparently
	// surfaces as an INTERIOR point of some other block, not as its own
	// code_compile_block() request. Widened to log any compile request
	// anywhere on cpp_get_token's page (0x10076000-0x10076fff) to find out
	// what pc actually IS being requested near there.
	// CONFIRMED (2026-09-05): dispatch_page_writable() is false for every
	// hit, and the incoming `mode` parameter (already the resolved
	// registration key, per static_dispatch_key()'s own logic -
	// mips3.h:1278-1281 - which picks m_core->mode over m_core->
	// dispatch_mode exactly when the page isn't writable) is the identical
	// constant 759 across all hits, as expected for a shared key.
	// EXTENDED (2026-09-05, same day) - causal-connection follow-up: the
	// user pointed out this page is plausibly, legitimately shared across
	// MULTIPLE SEQUENTIAL cc1 PROCESSES (cc1_null_loop.sh invokes cc1 ten
	// times; any real Unix kernel's page cache maps the same on-disk
	// .text pages to the same physical frame across every exec of the
	// same binary - ordinary demand-paging, not a bug), not "no legitimate
	// sharing" as first assumed. `mode` alone can't show this - it's
	// already resolved to the shared, ASID-independent value regardless
	// of which process triggered the compile. Logging m_core->
	// dispatch_mode too (the RAW, unresolved ASID-scoped composite -
	// mips3.h:494, bit0=FR, bit1=is_user, bits2+=asid-component) alongside
	// a per-pc recompile counter: if dispatch_mode's asid-component varies
	// across successive compiles of the SAME pc while the registration key
	// stays flat, that's direct proof multiple distinct process contexts
	// really are sharing this one compiled entry - the concrete,
	// checkable version of "is the sharing causally connected to anything"
	// rather than just a true-but-possibly-inert structural fact.
	if ((pc & ~offs_t(0xfff)) == 0x10076000)
	{
		static std::unordered_map<offs_t, int> s_compile_count;
		static std::unordered_map<offs_t, uint32_t> s_last_dispatch_mode;
		int const count = ++s_compile_count[pc];
		uint32_t const raw_dispatch_mode = m_core->dispatch_mode;
		uint32_t const prev_dispatch_mode = s_last_dispatch_mode.count(pc) ? s_last_dispatch_mode[pc] : raw_dispatch_mode;
		bool const asid_changed = (count > 1) && (raw_dispatch_mode != prev_dispatch_mode);
		s_last_dispatch_mode[pc] = raw_dispatch_mode;
		fprintf(stderr, "[TASK84_ASID_DIAG] pc=0x%08x compile#=%d mode=%u dispatch_page_writable=%d raw_dispatch_mode=%u raw_asid=%u raw_is_user=%d asid_changed_since_last_compile=%d\n",
			pc, count, mode, dispatch_page_writable(pc), raw_dispatch_mode,
			raw_dispatch_mode >> 2, (raw_dispatch_mode & 0x02) != 0, asid_changed);
		fflush(stderr);
	}
#endif

#ifdef MAME_DRC_CHECKSUM_MISMATCH_DIAG
	// TEMPORARY diagnostic (2026-07-28): see mips3.h's comment on the field
	// itself. `pc` here is the literal EXECUTE_MISSING_CODE address for
	// this specific call, before describe_code() walks anything.
	m_diag_compile_entry_pc = pc;
#endif

#if defined(MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG) || defined(MAME_SGI_VALIDATE_TLB_MISMATCH_CLASS_DIAG)
	// See diag_0fac5ffc_compile_gen's comment in mips3.h. `pc` here is
	// the compile entry address, matching how SEQ_TRACE above already
	// identifies this same address at this same scope. Also read by the
	// causation check in cfunc_validate_tlb_mismatch_class_diag, hence
	// gated on either diagnostic, not just the original one.
	if (pc == 0x0fac5ffc)
		m_core->diag_0fac5ffc_compile_gen++;
#endif

#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
	// MAME_SGI_DISPATCH_MODE_ASID_FIX value 2: the caller (execute_run(),
	// mips3.cpp) always passes m_core->dispatch_mode (the ASID-scoped
	// composite) as `mode` regardless of variant - override it to the
	// ASID-independent shared key (m_core->mode) whenever pc's own page is
	// currently non-writable, since content on a read-only page can't
	// differ per process.
	//
	// FIX 1 (2026-08-06) was tried here twice - first checking every
	// instruction in the compiled desclist, then narrowed to just this
	// pc's page plus its delay slot's page - both attempts caused a real
	// early-boot livelock (console frozen, CPU pegged, zero forward
	// progress). Root cause, found by reading static_dispatch_key()/
	// live_dispatch_key() (mips3.h): those two functions - used by every
	// OTHER compiled block's HASHJMP to decide what key to look a target
	// pc up under - only ever call this same single-page
	// dispatch_page_writable(pc) check, with no knowledge of any delay
	// slot. Fix 1 changed the key code_compile_block() *registers* a
	// block under (this site) without also changing the key every
	// HASHJMP *looks it up* under (those two sites) - so on any
	// delay-slot branch where the two checks disagree, the block is
	// registered under one key and never found under the other,
	// producing an infinite EXECUTE_MISSING_CODE/recompile loop that
	// never progresses. A correct fix needs static_dispatch_key()/
	// live_dispatch_key() updated in lockstep with this site, not just
	// this site alone - reverted back to the original single-page check
	// pending that. See journal.md 2026-08-06 for the two failed
	// attempts and this diagnosis.
#ifdef MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG
	// TEMPORARY diagnostic (2026-08-22, continued) - the validate_tlb
	// mismatch mechanism found and fixed above only accounts for ~0.24%
	// of this pc's total recompile volume (149 of 62018 in one boot -
	// see wiki/performance.md). This checks whether the dominant driver
	// is a lookup-vs-registration key mismatch: does a hash entry exist
	// under the RAW incoming mode (what execute_run() passed in), vs.
	// under the OVERRIDDEN/shared key (what this function actually
	// registers under, a few lines below)? If raw lookups miss while the
	// overridden key is already populated, something is dispatching
	// EXECUTE_MISSING_CODE via a different key than registration uses.
	if (pc == 0x0fac5ffc)
	{
		bool const raw_hash_exists = m_drcuml->hash_exists(mode, pc);
		bool const writable_now_probe = dispatch_page_writable(pc);
		uint32_t const overridden_mode_probe = writable_now_probe ? mode : m_core->mode;
		bool const overridden_hash_exists = m_drcuml->hash_exists(overridden_mode_probe, pc);
		static uint64_t s_entry_total = 0;
		static uint64_t s_raw_hit = 0;
		static uint64_t s_key_mismatch = 0;
		static uint64_t s_both_miss = 0;
		++s_entry_total;
		if (raw_hash_exists)
			++s_raw_hit;
		else if (overridden_hash_exists)
			++s_key_mismatch;
		else
			++s_both_miss;
		if ((s_entry_total <= 20) || ((s_entry_total % 10000) == 0))
		{
			fprintf(stderr,
				"[0FAC5FFC_ENTRY_DIAG] #%llu raw_mode=%u raw_hash_exists=%d overridden_mode=%u overridden_hash_exists=%d "
				"writable_now=%d totals(raw_hit=%llu key_mismatch=%llu both_miss=%llu)\n",
				(unsigned long long)s_entry_total, mode, raw_hash_exists, overridden_mode_probe, overridden_hash_exists,
				writable_now_probe, (unsigned long long)s_raw_hit, (unsigned long long)s_key_mismatch,
				(unsigned long long)s_both_miss);
			fflush(stderr);
		}
	}
#endif
	bool const writable_now = dispatch_page_writable(pc);
#ifdef MAME_SGI_RECOMPILE_TRACKING_TASK84
	if (pc >= 0x1007de00 && pc <= 0x1007e100)
	{
		static std::unordered_map<uint32_t, uint32_t> s_recompile_counts;
		uint32_t const count = ++s_recompile_counts[pc];
		fprintf(stderr,
			"[RECOMPILE_TRACKING_TASK84] pc=%08x raw_mode=%u writable_now=%d final_mode=%u count=%u%s\n",
			pc, mode, writable_now, writable_now ? mode : m_core->mode, count,
			(count > 1) ? " DUPLICATE" : "");
		fflush(stderr);
	}
#endif
	if (!writable_now)
		mode = m_core->mode;

	// Cross-key aliasing - ATTEMPTED AND REVERTED (2026-08-22, same
	// session as the entry-key diagnostic above). Reasoning at the time:
	// execute_run()'s EXECUTE_MISSING_CODE handler (mips3.cpp) always
	// calls this function with the RAW, ASID-scoped m_core->dispatch_mode,
	// regardless of what key the dispatch that actually missed was using;
	// since registration always collapses to the shared key above, the
	// raw key is never registered, so any entry via a path that doesn't
	// go through generate_select_dispatch_key()'s own collapsed-key
	// HASHJMP forces a full, wasted recompile even though a valid compile
	// already exists under the shared key (measured: one raw mode value
	// alone reached 140,774 wasted recompiles of pc=0x0fac5ffc in one
	// boot). The fix built here aliased the raw key to the shared key's
	// existing code pointer (drcuml_state::set_codeptr(), added this
	// session specifically for this - a hash-table pointer write only,
	// no codegen) instead of recompiling, then returned - reasoned to be
	// safer than the 2026-08-02 attempt below because it actually
	// registers something under the key it was asked to compile for,
	// rather than bailing with nothing registered.
	//
	// REGRESSED IMMEDIATELY, different failure signature than expected:
	// not the classic frozen-console livelock, but a diag_alias_hits
	// counter that reached 1.2+ BILLION in under 90 seconds (a tight,
	// cheap, unbounded loop - each iteration just two hash lookups and a
	// return, so it doesn't peg the same way a real recompile would, but
	// is equally non-terminating), while console_live.log stalled
	// immediately after "All Rights Reserved" - early kernel bring-up,
	// before any user-mode process runs at all. Confirmed via the
	// 0x0fac5ffc-specific entry diagnostic (still active above) that it
	// logged ZERO hits during this entire run - this is a DIFFERENT
	// address than the one investigated, encountered during kernel init,
	// not the user-mode/shared-library case this fix was designed
	// around. This falsifies the "aliasing avoids the 2026-08-02 failure
	// mode because it registers something" reasoning above: the raw key
	// clearly WAS getting re-registered (alias_hits climbing in lockstep
	// with compile_total, both incrementing together every call) and
	// execute() STILL immediately re-entered EXECUTE_MISSING_CODE for
	// what must be the same (or a tiny rotating set of) raw key(s) again
	// and again. The actual mechanism making *m_entry's dispatch fail to
	// find a freshly-registered raw-key entry - whether that's a
	// set_codeptr()-on-an-unpopulated-mode precondition problem
	// (drc_hash_table::populate_mode(), drcbeut.h - never called here),
	// some kernel-init-specific mode/ASID churn invalidating the alias
	// before it's ever read, or something else entirely - is still
	// unknown. A third attempt at this class of fix needs that mechanism
	// actually understood first, not just a structurally-different
	// reason to expect it might work this time - see wiki/performance.md
	// for the full writeup. Left as dead code, not deleted, per this
	// project's convention - the drcuml_state::set_codeptr() API added
	// for this remains in place (harmless on its own, unused unless
	// called) since it may still be useful groundwork for a future,
	// better-understood fix.
#if 0
	if (mode != raw_mode && m_drcuml->hash_exists(mode, pc))
	{
		drccodeptr const existing = m_drcuml->get_codeptr(mode, pc);
		if (existing != nullptr && m_drcuml->set_codeptr(raw_mode, pc, existing))
			return;
	}
#endif
#ifdef MAME_SGI_JMPDEST_SAMEPAGE_DIAG
	// Report diag_jmpdest_dispatch_total's same-page hit rate periodically -
	// see the macro's own comment above. code_compile_block() runs far less
	// often than the JIT'd dispatch code being measured, so checking here
	// (host C++, once per compile) is cheap - the counters themselves are
	// incremented directly by JIT'd code at the jmpdest call site, not here.
	{
		static uint32_t s_last_reported = 0;
		uint32_t const total = m_core->diag_jmpdest_dispatch_total;
		if (total > 0 && (s_last_reported == 0 || total >= s_last_reported + 50000))
		{
			uint32_t const crosspage = m_core->diag_jmpdest_dispatch_crosspage;
			fprintf(stderr, "[JMPDEST_SAMEPAGE] total=%u cross_page=%u (%.4f%%)\n",
				total, crosspage, 100.0 * double(crosspage) / double(total));
			fflush(stderr);
			s_last_reported = total;
		}
	}
#endif
#ifdef MAME_SGI_VALIDATE_TLB_FREQ_DIAG
	// Same pattern as the jmpdest report above, see diag_validate_tlb_total's
	// comment in mips3.h.
	{
		static uint32_t s_last_reported_tlb = 0;
		uint32_t const total = m_core->diag_validate_tlb_total;
		if (total > 0 && (s_last_reported_tlb == 0 || total >= s_last_reported_tlb + 50000))
		{
			fprintf(stderr, "[VALIDATE_TLB_FREQ] total=%u\n", total);
			fflush(stderr);
			s_last_reported_tlb = total;
		}
	}
#endif
#endif

	// REVERTED (2026-08-02, same evening): first attempt at this fix
	// unconditionally skipped recompilation whenever hash_exists(mode,pc)
	// was true at entry, reasoning that static_generate_tlb_mismatch()'s
	// fall-through EXECUTE_MISSING_CODE (see that function, tag 10 in the
	// dispatch-site diagnostic) meant "TLB just needed a refill, the
	// compiled block is still fine, retry will succeed." Caused a real
	// boot livelock on the very next test run - console output froze
	// during early boot (long before any previous run's normal pace)
	// while the process sat at ~97% CPU, the exact signature of
	// execute()->EXECUTE_MISSING_CODE->[skip, no compile]->execute()...
	// spinning forever with zero forward progress. The assumption that
	// "hash_exists true at the entry pc ⇒ a bare retry will succeed" does
	// NOT hold in general - the existing in-loop logic further down this
	// function (the "assume out of sync, recompile" branch, matched by
	// `seqhead == desclist`) exists precisely because MAME's original
	// authors already knew hash_exists-true-at-entry can mean genuine
	// staleness, not just "TLB was stale" - this fix bypassed that
	// safeguard entirely instead of only replacing it for the specific
	// TLB-refill case it was reasoning about. Left as a comment, not
	// silently deleted, so the next attempt doesn't re-derive the same
	// wrong assumption from scratch - see wiki/performance.md and the
	// project journal for the fuller writeup. A real fix here would need
	// to distinguish the TLB-refill-only case from a genuine staleness
	// case before skipping, not treat hash_exists() alone as sufficient.
#if 0
	if (m_drcuml->hash_exists(mode, pc))
	{
		return;
	}
#endif

	auto profile = g_profiler.start(PROFILER_DRC_COMPILE);

	/* get a description of this sequence */
#ifdef MAME_SGI_STALL_DURATION_DIAG
	auto const describe_code_start = std::chrono::steady_clock::now();
#endif
	desclist = m_drcfe->describe_code(pc);
#ifdef MAME_SGI_STALL_DURATION_DIAG
	// See diag_last_describe_code_ns's comment in mips3.h.
	m_core->diag_last_describe_code_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - describe_code_start).count();
#endif
	/* get last instruction of the code (potentially used in generate_checksum) */
	codelast = m_drcfe->get_last();
#ifdef MAME_SGI_COMPILE_LENGTH_CORRELATION_DIAG
	// See diag_last_compile_instruction_count's comment in mips3.h. Total
	// opcode_desc nodes in this call's window - every instruction in every
	// sequence describe_code() just returned, read back by execute_run()
	// right after this function returns.
	{
		uint32_t count = 0;
		for (const opcode_desc *d = desclist; d != nullptr; d = d->next())
			count++;
		m_core->diag_last_compile_instruction_count = count;
	}
#endif
#ifdef MAME_SGI_RODATA_SEQUENCE_OVERRUN_DIAG
	// See this define's own comment above. Target range is cc1's
	// .rodata section (guest VA 0x10328000, size 0x73f902, established
	// via static analysis of the extracted binary - see wiki/
	// console.md's task #84 section) - generous enough to catch any
	// sequence window reaching into it, not just the exact failing
	// string's address. Checks every desc in the chain, not just
	// codelast, since a runaway walk-through-misdecoded-data could in
	// principle produce a non-monotonic pc sequence.
	{
		constexpr uint32_t rodata_start = 0x10328000u;
		constexpr uint32_t rodata_end = rodata_start + 0x800000u; // generous margin past the known 0x73f902 size
		for (const opcode_desc *d = desclist; d != nullptr; d = d->next())
		{
			if (d->pc >= rodata_start && d->pc < rodata_end)
			{
				static uint64_t s_overrun_count = 0;
				s_overrun_count++;
				fprintf(stderr, "[MAME_SGI_RODATA_SEQUENCE_OVERRUN_DIAG] #%llu entry_pc=0x%08x seqhead_pc=0x%08x overrun_pc=0x%08x overrun_physpc=0x%08x codelast_pc=0x%08x opptr=0x%08x mode=0x%x\n",
					(unsigned long long)s_overrun_count, pc, desclist->pc, d->pc, d->physpc,
					codelast ? codelast->pc : 0, d->opptr, mode);
				fflush(stderr);
				break;
			}
		}
	}
#endif
	if (m_drcuml->logging() || m_drcuml->logging_native())
		log_opcode_desc(desclist, 0);

	/* if we get an error back, flush the cache and try again */
	bool succeeded = false;
	while (!succeeded)
	{
		try
		{
			/* start the block */
			drcuml_block &block(m_drcuml->begin_block(4096));

			/* loop until we get through all instruction sequences */
			for (seqhead = desclist; seqhead != nullptr; seqhead = seqlast->next())
			{
				const opcode_desc *curdesc;
				uint32_t nextpc;

#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
				// TEMPORARY diagnostic (2026-08-02, continued) - see
				// journal 2026-08-02 15:20 entry. Traces the actual
				// composition of the multi-sequence batch for this
				// investigation's compile, to find how an unrelated
				// address (0x10002d64, ~5.5MB away, impossible under the
				// 128/512-byte describe_code() window) ended up walked
				// by this same code_compile_block() call. Rate-limited
				// (first 10 + every 20000th) for the ordinary case - a
				// full run showed 231K+ compiles, all identical
				// single-instruction sequences, which at unconditional
				// logging grew to 31MB just reaching login. Any
				// interesting seqhead (not exactly 0x0fac5ffc alone)
				// still needs to print unconditionally, handled below
				// once curdesc is known.
				static uint32_t s_seqtrace_seqhead_count = 0;
				bool const seqtrace_print_seqhead = (++s_seqtrace_seqhead_count <= 10)
						|| (s_seqtrace_seqhead_count % 20000) == 0;
				if (pc == 0x0fac5ffc && seqtrace_print_seqhead)
				{
					fprintf(stderr, "[SEQ_TRACE] #%u new seqhead=0x%08x is_branch_target=%d\n",
						s_seqtrace_seqhead_count, seqhead->pc, seqhead->is_branch_target());
					fflush(stderr);
				}
#endif

				/* add a code log entry */
				if (m_drcuml->logging())
					block.append_comment("-------------------------");                     // comment

				/* determine the last instruction in this sequence */
				for (seqlast = seqhead; seqlast != nullptr; seqlast = seqlast->next())
					if (seqlast->end_sequence())
						break;
				assert(seqlast != nullptr);
#ifdef MAME_DRC_BUGFIXES
				// Bug fix: the scan above can walk off the end of the list without
				// finding an end_sequence()-flagged description (observed with
				// branch-likely instructions landing as the last in-range
				// instruction of a block - build_sequence() in drcfe.ipp reclaims
				// such a description rather than appending it, discarding its
				// end_sequence flag). The assert above is compiled out in release
				// builds, so without this fallback seqlast stays null and both the
				// compile loop below and this loop's own increment dereference it,
				// causing a real, reproducible SIGSEGV in code_compile_block. Fall
				// back to treating seqhead as a single-instruction sequence rather
				// than crash.
				if (seqlast == nullptr)
				{
					// The fallback itself (seqlast = seqhead, below) is
					// permanent and unconditional - never gate it behind
					// its own diagnostic macro, it's the actual crash
					// prevention. Only the reporting is optional/gated,
					// under its own sub-macro (not bare MAME_DRC_BUGFIXES)
					// per this project's own hygiene convention of never
					// leaving a diagnostic's output ungated inside a
					// permanent fix's own block - see drc-diagnostics.md's
					// close-out checklist, item 1.
					//
					// Was: unconditional per-event fprintf with a running
					// count, "remove once the frequency/pattern is
					// understood" (2026-07-23). It now is - three-way
					// comparison the same week confirmed the fallback is
					// genuinely rare (zero fires across an 11+ minute
					// live run, see wiki/assets.md) - so reworked
					// 2026-09-11 to a silent counter, reported once as a
					// clean-exit summary in device_stop() instead of
					// per-event, matching MAME_SGI_COP1_TRAP_DIAG/
					// MAME_SGI_TLB_MISMATCH_REFRESH_FIX's same-day rework.
#ifdef MAME_DRC_BUGFIXES_NULL_SEQLAST_DIAG
					m_core->diag_null_seqlast_count++;
					m_core->diag_null_seqlast_last_pc = seqhead->pc;
#endif
					seqlast = seqhead;
				}
#endif

#ifdef MAME_SGI_SEQUENCE_LENGTH_DIAG
				// TEMPORARY diagnostic (2026-08-07) - see diag_seq_count's
				// comment in mips3.h. Counts guest instructions from
				// seqhead to seqlast inclusive - the actual size of the
				// dispatch fragment this compile is about to generate.
				{
					uint32_t len = 1;
					for (const opcode_desc *walk = seqhead; walk != seqlast; walk = walk->next())
						len++;
					++m_core->diag_seq_count;
					m_core->diag_seq_instr_total += len;
					if (len == 1)
						++m_core->diag_seq_len1_count;
					if (len > m_core->diag_seq_len_max)
						m_core->diag_seq_len_max = len;

					// Categorize why THIS sequence ended - see diag_seqend_*'s
					// comment in mips3.h. Exhaustive over every
					// set_end_sequence() call site in mips3fe.cpp: every
					// true end_sequence() is flagged as either an
					// unconditional branch (J/JAL/JR/JALR/branch-idiom),
					// will_cause_exception() (SYSCALL/BREAK), or
					// can_change_modes() (MTC0-Status/ERET) - anything
					// that falls through to none of those only happens via
					// the null-seqlast fallback above (build_sequence()
					// ran off the end of the window with no terminator
					// found at all), a genuinely distinct, meaningful
					// category (pure window/COMPILE_MAX_SEQUENCE
					// exhaustion, unrelated to any branch).
					if (seqlast->is_unconditional_branch() && seqlast->targetpc == BRANCH_TARGET_DYNAMIC)
						++m_core->diag_seqend_indirect_jump;
					else if (seqlast->is_unconditional_branch())
						++m_core->diag_seqend_static_jump;
					else if (seqlast->will_cause_exception())
						++m_core->diag_seqend_exception;
					else if (seqlast->can_change_modes())
						++m_core->diag_seqend_mode_change;
					else
						++m_core->diag_seqend_window_limit;

					static uint32_t s_last_reported = 0;
					if (m_core->diag_seq_count >= s_last_reported + 50000)
					{
						fprintf(stderr, "[SEQ_LENGTH] count=%u total_instr=%llu avg=%.3f len1=%u (%.2f%%) max=%u | "
							"end: indirect=%u static=%u except=%u modechg=%u window=%u\n",
							m_core->diag_seq_count, (unsigned long long)m_core->diag_seq_instr_total,
							double(m_core->diag_seq_instr_total) / double(m_core->diag_seq_count),
							m_core->diag_seq_len1_count,
							100.0 * double(m_core->diag_seq_len1_count) / double(m_core->diag_seq_count),
							m_core->diag_seq_len_max,
							m_core->diag_seqend_indirect_jump, m_core->diag_seqend_static_jump,
							m_core->diag_seqend_exception, m_core->diag_seqend_mode_change,
							m_core->diag_seqend_window_limit);
						fflush(stderr);
						s_last_reported = m_core->diag_seq_count;
					}
				}
#endif

				// Bug fix: interrupt-check code is only ever woven into a
				// compiled sequence's generated code opportunistically -
				// generate_set_cop0_reg's Status/Cause/Compare cases set
				// compiler.checkints/checksoftints = true, consumed once by
				// the next generate_update_cycles() call. A sequence that
				// never writes those COP0 registers (an ordinary tight
				// LL/AND/BNE spinlock-wait loop, for example) is compiled
				// with *zero* interrupt-check instructions anywhere in it -
				// confirmed by reading every exit path (in-sequence, the
				// shared out_of_cycles handler, and execute_run()'s own
				// dispatch loop, none of which call check_irqs() or
				// equivalent under DRC). Once such a sequence is compiled
				// and hashed, it can be re-entered and re-executed
				// indefinitely from cache without ever regaining a check,
				// regardless of how many real hardware interrupts arrive
				// while it runs - confirmed live: Status shows interrupts
				// fully enabled and unmasked the whole time, Cause never
				// shows anything pending, and the guest never leaves the
				// loop. -nodrc can't have this problem by construction
				// (the interpreter's check_irqs() runs unconditionally on
				// every single instruction dispatch).
				//
				// SCOPE: deliberately narrow, not a blanket per-sequence
				// force. First attempt forced this for every freshly-
				// compiled sequence unconditionally and produced a new,
				// 100% reproducible `PANIC: TLBMISS: KERNEL FAULT` very
				// early in boot (confirmed via an identical-disk retry -
				// same PC, same EP, same bad address, same cause - a clean
				// "structural placement bug" signature, unlike the original
				// bug's non-determinism). A guard against seqhead being a
				// delay-slot continuation did NOT fix it, so whatever the
				// real mechanism is, it isn't (only) about delay slots -
				// left unresolved rather than guessed at further. Given a
				// blanket per-sequence force touches every sequence
				// compiled anywhere in the kernel, not just the target
				// spinlock shape, the safer and sufficient fix is to scope
				// it to exactly the shape that actually needs it: a
				// sequence whose own terminating branch targets its own
				// start (seqlast->targetpc == seqhead->pc) - a genuine
				// tight, self-contained loop that can iterate indefinitely
				// without ever exiting the sequence on its own, which is
				// exactly the precondition for this bug (a sequence that
				// exits normally will reach a different sequence's own
				// checkints opportunities soon regardless). This leaves
				// every non-looping sequence in the kernel completely
				// unaffected, matching the pre-fix behavior exactly, so
				// whatever caused the TLBMISS regression (evidently some
				// unrelated sequence elsewhere, since it wasn't fixed by
				// excluding delay slots specifically) should not be
				// reachable through this narrower path.
				//
				// CORRECTED (2026-07-23): `seqlast->targetpc` is the wrong
				// field to check - confirmed live via a compile-time dump
				// of every sequence compiled for the known stuck loop
				// (seqhead=0x8800fcc4): `seqlast` was never actually the
				// loop's own backward BNE. Either the frontend grouped the
				// loop together with a lot of *subsequent* code into one
				// larger sequence (seqlast->pc far beyond the loop, e.g.
				// 0x8800fce0), so the backward branch never ends the
				// sequence at all - or the null-seqlast fallback above
				// fired for this exact loop, redirecting seqlast to
				// seqhead itself (the LL instruction, which isn't a branch
				// and so has no real target - hence the observed
				// `seqlast->targetpc=0xffffffff`, the BRANCH_TARGET_DYNAMIC
				// sentinel, in every single case). The real self-loop
				// signature has to be found by scanning every instruction
				// in the sequence for one whose *own* target is the
				// sequence's start, not by trusting seqlast to be that
				// instruction.
				// CLEANER ALTERNATIVE (2026-07-28), promoted from the
				// first-pass per-call scans below: consult the persistent
				// record before redoing any of describe_code()'s
				// from-scratch analysis. If this exact seqhead->pc has
				// ever needed checkints in any prior compile, under any
				// window, it needs it now too - the code at this address
				// hasn't changed, only which neighboring PC happened to
				// trigger this particular compile has.
#ifdef MAME_SGI_TASK84_SEQHEAD_CHECKINTS_DIAG
				// captured here, BEFORE the FORCE_CHECKINTS override's own
				// goto below (which would otherwise jump past a declaration
				// placed any later in this block - a real "jump bypasses
				// variable initialization" compile error, not just a style
				// choice) and before the sticky lookup itself, so the
				// diagnostic can tell "checkints=true because
				// m_checkints_addrs already had this exact PC from some
				// earlier compile" (which, being keyed purely by virtual PC
				// and never cleared by code_flush_cache(), can be inherited
				// from a DIFFERENT program's earlier, unrelated code that
				// happened to load at the same address) apart from
				// "checkints=true because this specific compile's own scan
				// found a real reason" - the two look identical in the final
				// checkints value alone.
				bool const task84_seqhead_was_sticky = m_checkints_addrs.count(seqhead->pc) != 0;
#endif
#if defined(MAME_SGI_TASK84_FORCE_CHECKINTS_ON) || defined(MAME_SGI_TASK84_FORCE_CHECKINTS_OFF)
				// Task #84 (2026-09-05): direct experimental override, scoped
				// to cpp_get_token's own seqhead only (0x10076008) - NOT a
				// blanket force, which already caused a 100%-reproducible
				// early-boot TLBMISS regression once (see this file's own
				// 2026-07-23 comment above). Bypasses BOTH the sticky
				// m_checkints_addrs lookup/insert AND the natural
				// self-loop/backward-branch/branch-target heuristics below
				// entirely for this one seqhead, so the resulting value is
				// under our direct control across every recompile,
				// independent of this process's execution history (unlike
				// the natural mechanism, confirmed sticky-forever once
				// triggered - see wiki/task84.md's 2026-09-05 "23 consecutive
				// LONG cycles" entry). Exists to get real causal A/B data on
				// whether this specific interrupt-check-preamble mechanism
				// affects cpp_get_token's pass/fail rate, instead of
				// continuing to infer it from whatever this process's
				// history happens to have already latched onto.
				// Widened from the exact entry PC (0x10076008) to the whole
				// function's address window: code_compile_block() processes
				// multiple internal seqheads per call (confirmed live - an
				// entry-only version of this override left interrupt_norecover
				// calls still present, traced to the interior points
				// 0x10076064/0x100760B8/0x100760C8/0x100760E4 each getting
				// their own seqhead-level arming pass within the same overall
				// compile), so only forcing the entry point left the interior
				// ones on the natural/sticky path untouched.
				if (seqhead->pc >= 0x10076000 && seqhead->pc <= 0x10076400)
				{
#if defined(MAME_SGI_TASK84_FORCE_CHECKINTS_ON)
					compiler.checkints = true;
					compiler.checksoftints = true;
#else
					compiler.checkints = false;
					compiler.checksoftints = false;
#endif
					goto task84_checkints_forced;
				}
#endif
#ifdef MAME_SGI_CHECKINTS_ARMED_FRACTION_DIAG
				// See m_diag_checkints_all_compiled_pcs's comment in mips3.h -
				// unconditional, tracks every seqhead this call site sees
				// regardless of checkints outcome, for later comparison
				// against m_checkints_addrs.size() at clean exit.
				if (m_diag_checkints_all_compiled_pcs.empty())
					m_diag_checkints_all_compiled_pcs.reserve(1'000'000);
				m_diag_checkints_all_compiled_pcs.insert(seqhead->pc);
#endif
// Disables the self-loop/backward-branch/branch-target heuristic below
// in its entirety (everything that feeds the sticky m_checkints_addrs
// set, including the 2026-07-27 is_branch_target() defensive fallback
// nested inside it) - the four deterministic MTC0-Status/Cause/Compare-
// write and ERET arming sites elsewhere in this file
// (generate_set_cop0_reg(), ERET handling) are untouched and stay fully
// active, as does static_generate_entry_point()'s own unconditional
// per-re-entry check (stock MAME code, not gated by this macro at all).
// Originally built (task #101, 2026-09-17) to test whether the standing
// entry-point recheck plus deterministic COP0-write arming is already
// sufficient on its own, without this heuristic's extra in-sequence
// checks. Tested and left enabled by deliberate choice - see this
// macro's own top-of-file comment for the test evidence and the one
// still-open caveat (the original mrupdatef livelock couldn't be
// directly re-triggered to confirm this covers that specific case too).
#ifndef MAME_SGI_DISABLE_CHECKINTS_HEURISTIC_EXPERIMENT
				if (m_checkints_addrs.count(seqhead->pc))
				{
					compiler.checkints = true;
					compiler.checksoftints = true;
				}
				for (curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
				{
					if (curdesc->targetpc == seqhead->pc)
					{
						compiler.checkints = true;
						compiler.checksoftints = true;
						break;
					}
				}
#ifdef MAME_DRC_BUGFIXES
				// Bug fix (2026-07-26): the check above only catches a loop
				// that's entirely self-contained within one compiled
				// sequence - it requires a branch's target to match *this*
				// sequence's own seqhead exactly. A loop stitched together
				// across multiple separately-compiled sequences via real
				// function calls (confirmed live: mrupdatef's retry loop -
				// jal to splhi/us_delay/min between each condition check,
				// closing via branches like `b 0x880e9a38` that target an
				// *earlier* sequence's seqhead, not this one's) has no
				// single instruction matching that exact-seqhead shape, so
				// it fell through the existing fix uncaught - confirmed via
				// a live interrupt-rate measurement (zero interrupts over
				// ten real seconds while stuck in exactly this loop) and a
				// compile-time dump (checkints=0, checksoftints=0 at all
				// three of the loop's real back-edge branches).
				//
				// Generalized: any branch whose target is provably
				// *backward* (targetpc < the branch's own pc, excluding the
				// BRANCH_TARGET_DYNAMIC sentinel for indirect/return
				// branches) is a loop back-edge by the standard control-
				// flow definition, regardless of which sequence originally
				// compiled the target. Deliberately not "force every
				// sequence unconditionally" - that exact blanket approach
				// already caused a 100%-reproducible early-boot TLBMISS
				// regression once (see the comment above) - this stays
				// scoped to sequences that actually contain a backward
				// branch, a meaningful subset, not all of them.
				if (!compiler.checkints)
				{
					for (curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
					{
						if (curdesc->targetpc != BRANCH_TARGET_DYNAMIC && curdesc->targetpc < curdesc->pc)
						{
							compiler.checkints = true;
							compiler.checksoftints = true;
							break;
						}
					}
				}
				// FIRST-PASS fix (2026-07-27), flagged for review of possible
				// alternate approaches once confirmed working - not
				// necessarily the final form. Both checks above only see
				// backward branches that are actually *part of* this
				// compiled sequence. A degenerate sequence (confirmed live:
				// seqhead==seqlast, just the bare LL of the known 0x8800fcc4
				// spinlock, no surrounding AND/BNE) can't see a backward
				// branch that lives in a *different* compiled instance of
				// the same loop - structurally the same "shared instruction,
				// independently reachable via hashjmp, safe instance isn't
				// necessarily the one that executes" shape as tonight's
				// elocore_exl_3 investigation, just on the interrupt-arming
				// side instead of a COP0 write. Conservative fix: if this
				// sequence's own seqhead is independently a hashjmp target
				// (is_branch_target()), we can't statically rule out it
				// being part of a loop reached that way, so arm checkints
				// defensively even with no backward branch visible locally.
				if (!compiler.checkints && seqhead->is_branch_target())
				{
					compiler.checkints = true;
					compiler.checksoftints = true;
				}
#endif
#endif // MAME_SGI_DISABLE_CHECKINTS_HEURISTIC_EXPERIMENT
				// CLEANER ALTERNATIVE (2026-07-28): record the outcome so
				// future compiles of this same address, under whatever
				// window happens to trigger them, consult this instead of
				// re-deriving it from scratch and sometimes missing it.
				//
				// Fix (2026-09-17, task #100 scoping step 1) - reserve a
				// large bucket count on first insert, same mitigation
				// already proven on m_diag_usermode_recompile_counts this
				// session: converts periodic incremental-rehash stalls
				// (confirmed via live gdb backtrace: this exact insert call
				// rehashing to 351,061 buckets mid-boot) into one
				// predictable upfront allocation. 1,000,000 is headroom
				// above that observed mid-boot count, not a proven bound
				// for arbitrarily long real sessions - see wiki/tasks.md
				// task #100 for why a fixed reserve is a stopgap, not a
				// full fix, and what would be needed for one.
				if (compiler.checkints)
				{
					if (m_checkints_addrs.empty())
						m_checkints_addrs.reserve(1'000'000);
					m_checkints_addrs.insert(seqhead->pc);
				}

#if defined(MAME_SGI_TASK84_FORCE_CHECKINTS_ON) || defined(MAME_SGI_TASK84_FORCE_CHECKINTS_OFF)
				task84_checkints_forced:;
#endif

#ifdef MAME_SGI_TASK84_SEQHEAD_CHECKINTS_DIAG
				// Retargeted 2026-09-09 from 0x10001000-0x10002000 (stress_v0_loop,
				// a since-closed synthetic-assembly investigation) to
				// cpp_get_token's own function window, matching the
				// FORCE_CHECKINTS override's window a few lines above - this
				// session's flush/recompile experiment (wiki/task84.md,
				// 2026-09-08) established that whatever decides pass/fail is
				// settled once, at the moment this PC region gets freshly
				// compiled (e.g. right after a code_flush_cache()), and is then
				// 100% sticky for every execution thereafter. This checks
				// whether checkints/is_branch_target/was_sticky - the one
				// piece of compile-time-decided, non-machine-code state
				// already known to exist and already instrumented - actually
				// varies between a "good" and a "bad" fresh compile of this
				// region, or is a fixed constant either way.
				if (seqhead->pc >= 0x10076000 && seqhead->pc <= 0x10076400)
				{
					static FILE *s_seqhead_diag_log = nullptr;
					if (s_seqhead_diag_log == nullptr)
						s_seqhead_diag_log = fopen("task84_seqhead_checkints.log", "a");
					if (s_seqhead_diag_log != nullptr)
					{
						fprintf(s_seqhead_diag_log,
							"seqhead=0x%08x seqlast=0x%08x is_branch_target=%d checkints=%d checksoftints=%d was_sticky=%d\n",
							seqhead->pc, seqlast->pc, seqhead->is_branch_target() ? 1 : 0,
							compiler.checkints ? 1 : 0, compiler.checksoftints ? 1 : 0,
							task84_seqhead_was_sticky ? 1 : 0);
						fflush(s_seqhead_diag_log);
					}
				}
#endif

				// DISABLED (2026-07-29): like the COMPILETIME(elocore_exl_N)
				// print above, this was never gated - found while
				// investigating whether repeated recompiles of the known
				// 0x8800fcc4 LL/AND/BNE spinlock sequence were a genuine
				// compile-thrashing bug (as suspected) or just normal
				// first-time compilation; the unconditional fprintf+fflush
				// itself was adding real cost on top of whatever the
                                // underlying recompile frequency turns out to be.
#ifdef MAME_SGI_COMPILETIME_ELOCORE_DIAG
				for (curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
				{
					if (curdesc->pc == 0x8800fcc4 || curdesc->pc == 0x8800fcc8)
					{
						fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME(spinloop_checkints) seqhead=0x%08x seqlast=0x%08x checkints=%d checksoftints=%d\n",
							seqhead->pc, seqlast->pc, compiler.checkints, compiler.checksoftints);
						fflush(stderr);
						break;
					}
				}
#endif

#ifdef MAME_SGI_MUTEXWAIT_SPIN_DIAG
				for (curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
				{
					if (curdesc->pc == 0x880eacac || curdesc->pc == 0x880eaca4)
					{
						osd_printf_info("MAME_SGI_MUTEXWAIT_SPIN_DIAG: COMPILETIME(mutex_wait_spin) seqhead=0x%08x seqlast=0x%08x curdesc_pc=0x%08x checkints=%d checksoftints=%d\n",
							seqhead->pc, seqlast->pc, curdesc->pc, compiler.checkints, compiler.checksoftints);
						break;
					}
				}
#endif

				/* if we don't have a hash for this mode/pc, or if we are overriding all, add one */
				if (override || !m_drcuml->hash_exists(mode, seqhead->pc))
				{
					UML_HASH(block, mode, seqhead->pc);                                     // hash    mode,pc

#ifdef MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG
					// UPDATED 2026-09-01 (same day): originally only the
					// override branch below was instrumented, on the
					// assumption a stale-m_prptr bug would show up as a
					// checksum-mismatch-triggered recompile. Live-tested
					// against a real, reproduced 14-failure burst (i=10-23,
					// task #84's bigtest.c compile loop) with this exact
					// build - ZERO override events fired despite 4 real
					// drc_cache flushes happening during the same window.
					// That's a genuine negative result: whatever's wrong
					// isn't going through the checksum-mismatch/override
					// path at all, which means the FIRST (only) compile of
					// the affected pc is the more likely place a stale
					// m_prptr read would matter - if generate_checksum_
					// block() bakes in a wrong host pointer on the very
					// first compile, and that pointer's own re-reads keep
					// agreeing with themselves forever after, `override`
					// would never trip for that pc at all. Extended to
					// cover this branch (seqhead==desclist, either from a
					// genuine first-ever compile OR override already set
					// by an earlier desc in this same call) so both
					// possibilities are covered, not just recompiles.
					if (seqhead == desclist && ((mode >> 2) != 0) && ((mode & 0x02) != 0))
					{
						static uint64_t s_usermode_compile_count = 0;
						static uint64_t s_usermode_compile_mismatch_count = 0;
						s_usermode_compile_count++;

						// Mirrors exactly what generate_checksum_block() is
						// about to bake into this fresh compile's own
						// per-execution checksum re-read (base/low_bits
						// computation, loose-verify single-word case) -
						// this is the same raw host pointer that block will
						// keep re-reading from on every future execution.
						const void *raw = m_prptr(seqhead->physpc);
						uint32_t const low_bits = (seqhead->physpc & (m_data_bits == 64 ? 4 : 0)) ^ m_dword_xor;
						uint32_t raw_word = 0xffffffff;
						if (raw != nullptr)
							memcpy(&raw_word, reinterpret_cast<const uint8_t *>(raw) + low_bits, sizeof(raw_word));
						uint32_t const space_word = m_program->read_dword(seqhead->physpc & ~3);
						bool const mismatch = (raw != nullptr && raw_word != space_word);

						if (mismatch)
						{
							s_usermode_compile_mismatch_count++;
							fprintf(stderr, "[MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG] STALE_PRPTR_MISMATCH #%llu pc=0x%08x physpc=0x%08x mode=0x%x raw_ptr_word=0x%08x space_word=0x%08x override=%d\n",
								(unsigned long long)s_usermode_compile_mismatch_count, seqhead->pc, seqhead->physpc, mode, raw_word, space_word, (int)override);
							fflush(stderr);
						}
						else if ((s_usermode_compile_count % 500) == 1)
						{
							fprintf(stderr, "[MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG] heartbeat usermode_compiles=%llu mismatches=%llu latest pc=0x%08x physpc=0x%08x override=%d\n",
								(unsigned long long)s_usermode_compile_count, (unsigned long long)s_usermode_compile_mismatch_count, seqhead->pc, seqhead->physpc, (int)override);
							fflush(stderr);
						}
					}
#endif
				}
				/* if we already have a hash, and this is the first sequence, assume that we */
				/* are recompiling due to being out of sync and allow future overrides */
				else if (seqhead == desclist)
				{
					override = true;
					UML_HASH(block, mode, seqhead->pc);                                     // hash    mode,pc

#ifdef MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG
					// See this define's own comment above. Only genuine
					// user-mode composites (same bit decode as
					// MAME_DRC_USERMODE_RECOMPILE_DIAG) - kernel-mode
					// override-recompiles are a much noisier, unrelated
					// population for this investigation.
					if (((mode >> 2) != 0) && ((mode & 0x02) != 0))
					{
						static uint64_t s_usermode_override_count = 0;
						static uint64_t s_usermode_override_mismatch_count = 0;
						s_usermode_override_count++;

						// Mirrors exactly what generate_checksum_block() is
						// about to bake into this fresh compile's own
						// per-execution checksum re-read (base/low_bits
						// computation, loose-verify single-word case) -
						// this is the same raw host pointer that block will
						// keep re-reading from on every future execution.
						const void *raw = m_prptr(seqhead->physpc);
						uint32_t const low_bits = (seqhead->physpc & (m_data_bits == 64 ? 4 : 0)) ^ m_dword_xor;
						uint32_t raw_word = 0xffffffff;
						if (raw != nullptr)
							memcpy(&raw_word, reinterpret_cast<const uint8_t *>(raw) + low_bits, sizeof(raw_word));
						uint32_t const space_word = m_program->read_dword(seqhead->physpc & ~3);
						bool const mismatch = (raw != nullptr && raw_word != space_word);

						if (mismatch)
						{
							s_usermode_override_mismatch_count++;
							fprintf(stderr, "[MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG] STALE_PRPTR_MISMATCH(override-branch) #%llu pc=0x%08x physpc=0x%08x mode=0x%x raw_ptr_word=0x%08x space_word=0x%08x\n",
								(unsigned long long)s_usermode_override_mismatch_count, seqhead->pc, seqhead->physpc, mode, raw_word, space_word);
							fflush(stderr);
						}
						else if ((s_usermode_override_count % 200) == 1)
						{
							fprintf(stderr, "[MAME_DRC_USERMODE_OVERRIDE_STALE_CHECK_DIAG] heartbeat(override-branch) usermode_overrides=%llu mismatches=%llu latest pc=0x%08x physpc=0x%08x\n",
								(unsigned long long)s_usermode_override_count, (unsigned long long)s_usermode_override_mismatch_count, seqhead->pc, seqhead->physpc);
							fflush(stderr);
						}
					}
#endif
				}

				/* otherwise, redispatch to that fixed PC and skip the rest of the processing */
				else
				{
					UML_LABEL(block, seqhead->pc | 0x80000000);                             // label   seqhead->pc | 0x80000000
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
					UML_HASHJMP(block, static_dispatch_key(seqhead->pc), seqhead->pc, *m_nocode);   // hashjmp <mode>,seqhead->pc,nocode
#else
					UML_HASHJMP(block, m_core->dispatch_mode, seqhead->pc, *m_nocode);               // hashjmp <mode>,seqhead->pc,nocode
#endif
					continue;
				}

				/* validate this code block if we're not pointing into ROM */
				if (m_program->get_write_ptr(seqhead->physpc) != nullptr)
					generate_checksum_block(block, compiler, seqhead, seqlast, codelast);

				/* label this instruction, if it may be jumped to locally */
				if (seqhead->is_branch_target())
				{
					UML_LABEL(block, seqhead->pc | 0x80000000);                             // label   seqhead->pc | 0x80000000
				}

				/* iterate over instructions in the sequence and compile them */
				for (curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
				{
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
					// TEMPORARY diagnostic (2026-08-02, continued) - see
					// the matching seqhead print above. Unconditional for
					// anything actually interesting (a page fault, or any
					// address other than 0x0fac5ffc itself showing up in
					// this chain) - only the boring, expected single-
					// instruction case is rate-limited.
					if (pc == 0x0fac5ffc)
					{
						bool const interesting = curdesc->compiler_page_fault() || curdesc->pc != 0x0fac5ffc;
						if (interesting || seqtrace_print_seqhead)
						{
							fprintf(stderr, "[SEQ_TRACE]   curdesc=0x%08x page_fault=%d length=%u end_seq=%d branch_target=%d%s\n",
								curdesc->pc, curdesc->compiler_page_fault(), curdesc->length,
								curdesc->end_sequence(), curdesc->is_branch_target(),
								interesting ? " <-- INTERESTING" : "");
							fflush(stderr);
						}
					}
#endif
					generate_sequence_instruction(block, compiler, curdesc);
				}

				/* if we need to return to the start, do it */
				if (seqlast->return_to_start())
					nextpc = pc;

				/* otherwise we just go to the next instruction */
				else
					nextpc = seqlast->pc + (seqlast->skipslots + 1) * 4;

				/* count off cycles and go there */
				generate_update_cycles(block, compiler, nextpc, true);          // <subtract cycles>

				/* if the last instruction can change modes, use a variable mode; otherwise, assume the same mode */
				if (seqlast->can_change_modes())
				{
#ifdef MAME_SGI_SEQUENCE_LENGTH_DIAG
					++m_core->diag_seqexit_mode_change_hashjmp;
#endif
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
					UML_HASHJMP(block, live_dispatch_key(nextpc), nextpc, *m_nocode);              // hashjmp <mode>,nextpc,nocode
#else
					UML_HASHJMP(block, mem(&m_core->dispatch_mode), nextpc, *m_nocode);              // hashjmp <mode>,nextpc,nocode
#endif
				}
				else if (seqlast->next() == nullptr || seqlast->next()->pc != nextpc)
				{
#ifdef MAME_SGI_SEQUENCE_LENGTH_DIAG
					++m_core->diag_seqexit_static_hashjmp;
					// TEMPORARY diagnostic (2026-08-08) - see
					// diag_seqexit_hashjmp_target_in_batch's comment in
					// mips3.h. O(n) walk, compile-time only (once per
					// sequence compiled, not per dispatch) - cheap in
					// aggregate even though not cheap per-occurrence.
					{
						bool found_in_batch = false;
						for (const opcode_desc *walk = desclist; walk != nullptr; walk = walk->next())
						{
							if (walk->pc == nextpc)
							{
								found_in_batch = true;
								break;
							}
						}
						if (found_in_batch)
							++m_core->diag_seqexit_hashjmp_target_in_batch;
						else
							++m_core->diag_seqexit_hashjmp_target_outside_batch;
					}
#endif
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
					UML_HASHJMP(block, static_dispatch_key(nextpc), nextpc, *m_nocode);                    // hashjmp <mode>,nextpc,nocode
#else
					UML_HASHJMP(block, m_core->dispatch_mode, nextpc, *m_nocode);                    // hashjmp <mode>,nextpc,nocode
#endif
				}
#ifdef MAME_SGI_SEQUENCE_LENGTH_DIAG
				else
				{
					// TEMPORARY diagnostic (2026-08-08) - the free tier:
					// neither branch above fired, so no dispatch code is
					// emitted at all - the next sequence's compiled code
					// simply follows in program order in the same
					// drcuml_block. See diag_seqexit_free_fallthrough's
					// comment above.
					++m_core->diag_seqexit_free_fallthrough;
				}
				{
					static uint32_t s_last_reported = 0;
					uint32_t const total = m_core->diag_seqexit_free_fallthrough
						+ m_core->diag_seqexit_mode_change_hashjmp + m_core->diag_seqexit_static_hashjmp;
					if (total >= s_last_reported + 50000)
					{
						fprintf(stderr, "[SEQEXIT] free=%u mode_change_hashjmp=%u static_hashjmp=%u (in_batch=%u outside_batch=%u)\n",
							m_core->diag_seqexit_free_fallthrough, m_core->diag_seqexit_mode_change_hashjmp,
							m_core->diag_seqexit_static_hashjmp, m_core->diag_seqexit_hashjmp_target_in_batch,
							m_core->diag_seqexit_hashjmp_target_outside_batch);
						fflush(stderr);
						s_last_reported = total;
					}
				}
#endif
			}

			/* end the sequence */
			block.end();
			succeeded = true;
		}
		catch (drcuml_block::abort_compilation &)
		{
#ifdef MAME_SGI_STALL_DURATION_DIAG
			auto const internal_flush_start = std::chrono::steady_clock::now();
#endif
			code_flush_cache();
#ifdef MAME_SGI_STALL_DURATION_DIAG
			{
				uint64_t const ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - internal_flush_start).count();
				m_core->diag_flush_stall_total_ns += ns;
				m_core->diag_flush_stall_event_count++;
				if (ns > m_core->diag_flush_stall_max_ns)
					m_core->diag_flush_stall_max_ns = ns;
				if (ns > MAME_SGI_STALL_DIAG_THRESHOLD_NS)
					m_core->diag_flush_stall_over_threshold_count++;
				// See diag_last_compile_flush_ns's comment in mips3.h -
				// execute_run() (mips3.cpp) reads this back and subtracts
				// it from the enclosing code_compile_block() call's own
				// elapsed time, so this cost is attributed here (flush)
				// and not double-counted as compile time too.
				m_core->diag_last_compile_flush_ns += ns;
			}
#endif
		}
	}
}



/***************************************************************************
    C FUNCTION CALLBACKS
***************************************************************************/


static void cfunc_mips3com_update_cycle_counting(void *param)
{
	((mips3_device *)param)->mips3com_update_cycle_counting();
}

static void cfunc_mips3com_asid_changed(void *param)
{
	((mips3_device *)param)->mips3com_asid_changed();
}

static void cfunc_mips3com_tlbr(void *param)
{
	((mips3_device *)param)->mips3com_tlbr();
}

static void cfunc_mips3com_tlbwi(void *param)
{
	((mips3_device *)param)->mips3com_tlbwi();
}

static void cfunc_mips3com_tlbwr(void *param)
{
	((mips3_device *)param)->mips3com_tlbwr();
}

static void cfunc_mips3com_tlbp(void *param)
{
	((mips3_device *)param)->mips3com_tlbp();
}

#ifdef MAME_SGI_RODATA_LOAD_ADDR_DIAG
// See MAME_SGI_RODATA_LOAD_ADDR_DIAG's own comment (near the top of this
// file) for what this targets. Scratch fields live in internal_mips3_
// state (mips3.h's diag_rodata_addr/value/hit), not file-scope statics -
// the x64 DRC backend addresses mem() operands RBP-relative to that
// struct, and a free-standing global elsewhere in the binary landed out
// of the encodable displacement range, crashing live with
// "drcbe_x64::offset_from_rbp: delta out of range" the first time
// generated code touched it. func_rodata_load_addr_diag() is a member
// function (needs m_core access); cfunc_rodata_load_addr_diag() is the
// free-function trampoline UML_CALLC actually takes, matching this
// file's established pattern (see cfunc_mxcsr_desync_diag/func_mxcsr_
// desync_diag for the same split).
void mips3_device::func_rodata_load_addr_diag()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_SGI_RODATA_LOAD_ADDR_DIAG] #%llu addr=0x%08x value=0x%08x\n",
		(unsigned long long)s_count, m_core->diag_rodata_addr, m_core->diag_rodata_value);
	fflush(stderr);
}

static void cfunc_rodata_load_addr_diag(void *param)
{
	((mips3_device *)param)->func_rodata_load_addr_diag();
}
#endif

/*-------------------------------------------------
    cfunc_get_cycles - compute the total number
    of cycles executed so far
-------------------------------------------------*/

void mips3_device::func_get_cycles()
{
	m_core->numcycles = total_cycles();
}

static void cfunc_get_cycles(void *param)
{
	((mips3_device *)param)->func_get_cycles();
}


/*-------------------------------------------------
    cfunc_callsite_ic_probe - TEMPORARY diagnostic
    (2026-08-08): see m_diag_callsite_ic_state's
    comment in mips3.h. Called on every jmpdest
    dispatch when MAME_SGI_CALLSITE_IC_HITRATE_DIAG
    is on - looks up this call site's (desc->pc) last
    resolved {mode,pc}, compares against the live
    values, updates hit/miss counts, and stores the
    live values as the new "last seen" pair.
-------------------------------------------------*/

void mips3_device::func_callsite_ic_probe()
{
	callsite_ic_entry &entry = m_diag_callsite_ic_state[m_core->diag_ic_probe_callsite];
	if (entry.seen && entry.last_mode == m_core->diag_ic_probe_mode && entry.last_pc == m_core->jmpdest)
		entry.hits++;
	else
		entry.misses++;
	entry.seen = true;
	entry.last_mode = m_core->diag_ic_probe_mode;
	entry.last_pc = m_core->jmpdest;
}

#ifdef MAME_SGI_CALLSITE_IC_HITRATE_DIAG
static void cfunc_callsite_ic_probe(void *param)
{
	((mips3_device *)param)->func_callsite_ic_probe();
}
#endif


#ifdef MAME_SGI_CALLSITE_IC_PROTOTYPE
/*-------------------------------------------------
    cfunc_ic_learn_and_arm - PROTOTYPE (2026-08-08):
    see the class-scope comment near m_diag_callsite_
    ic_state (mips3.h) for the full design/history.
    Runs on the guard-miss path, the first time a
    given slot is touched this generation (expected_pc
    still the sentinel) - never again until the next
    flush resets it.

    Commits expected_mode/expected_pc and ic_site_
    target TOGETHER, and only once hash_exists()/
    get_codeptr() have actually succeeded - if the
    target isn't compiled yet, the slot is left at the
    sentinel entirely (retried on the next dispatch of
    this slot, whatever call site that turns out to be)
    rather than "established but unresolved". This is
    the invariant that makes OP_DIRECTJMP safe to reach
    unconditionally on a guard hit: expected_pc !=
    sentinel is only ever true when ic_site_target also
    holds a valid, resolved codeptr - there's no
    intermediate state where a hit could jump through a
    null/stale slot.
-------------------------------------------------*/

void mips3_device::func_ic_learn_and_arm()
{
	uint32_t const idx = m_core->ic_slot_index;
	if (m_core->ic_site_expected_pc[idx] != 0xFFFFFFFFu)
		return;   // already established this generation - nothing to do

	uint32_t const mode = m_core->ic_live_mode;
	uint32_t const pc = m_core->jmpdest;

	if (!m_drcuml->hash_exists(mode, pc))
		return;   // not compiled yet - stays at the sentinel, retried on the next dispatch of this slot

	drccodeptr const target = m_drcuml->get_codeptr(mode, pc);
	m_core->ic_site_target[idx] = (uint64_t)(uintptr_t)target;
	m_core->ic_site_expected_mode[idx] = mode;
	m_core->ic_site_expected_pc[idx] = pc;
}

static void cfunc_ic_learn_and_arm(void *param)
{
	((mips3_device *)param)->func_ic_learn_and_arm();
}
#endif


/*-------------------------------------------------
    cfunc_lockstate_probe - TEMPORARY diagnostic
    (2026-07-24): logs interrupt-masking state at a
    handful of specific guest instruction addresses -
    the mutex_wake dequeue clear-pair (sites 1/2), the
    mutex enqueue empty-case set-pair (sites 3/4), the
    fast-path uncontended-acquire store (site 5), and
    the enqueue insert-at-head-of-existing-queue store
    (site 6) - to test whether interrupts are disabled
    at the exact moments these related fields (a
    wait-queue "next" pointer and a "has waiters" flag
    bit) actually get written, as opposed to only at the
    crash site that later observes an inconsistency
    between them, and to build a per-object write
    history broad enough to plausibly catch the actual
    crashing instance rather than an unrelated one.
    Trivial static wrappers (one per site) rather than
    encoding both `this` and a site tag into the single
    void* UML_CALLC provides.
-------------------------------------------------*/

void mips3_device::func_lockstate_probe(int site)
{
	const uint32_t status = (uint32_t)m_core->cpr[0][COP0_Status];
	const uint32_t cause = (uint32_t)m_core->cpr[0][COP0_Cause];
	// Object address (2026-07-24): sites 1/2 are inside mutex_wake, which
	// keeps the mutex pointer in $s0 (r16) throughout; sites 3/4/6 are the
	// separate enqueue helper, which uses $a0 (r4) directly; site 5 is the
	// mutex_lock fast-path, which keeps the mutex pointer in $s1 (r17).
	// Logged so separate runs' site hits can be correlated with a specific
	// crashing mutex instance rather than assumed to be the same object.
	uint32_t obj;
	if (site <= 2) obj = (uint32_t)m_core->r[16];
	else if (site == 5) obj = (uint32_t)m_core->r[17];
	else obj = (uint32_t)m_core->r[4];
	fprintf(stderr, "[MAME_DRC_DIAG] LOCKSTATE site=%d obj=0x%08x status=0x%08x (IE=%u EXL=%u ERL=%u IM=0x%02x) cause=0x%08x (IP=0x%02x)\n",
		site, obj, status, status & 1, (status >> 1) & 1, (status >> 2) & 1, (status >> 8) & 0xff,
		cause, (cause >> 8) & 0xff);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_1(void *param) { ((mips3_device *)param)->func_lockstate_probe(1); }
#endif
#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_2(void *param) { ((mips3_device *)param)->func_lockstate_probe(2); }
#endif
#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_3(void *param) { ((mips3_device *)param)->func_lockstate_probe(3); }
#endif
#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_4(void *param) { ((mips3_device *)param)->func_lockstate_probe(4); }
#endif
#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_5(void *param) { ((mips3_device *)param)->func_lockstate_probe(5); }
#endif

/*-------------------------------------------------
    cfunc_install_mutex_tap - TEMPORARY (2026-07-24):
    fires at 0x880eaee0 (the enqueue empty-case "set
    next" store - see site 3 above), where $a0 is the
    mutex object receiving its first-ever waiter. A
    static, pre-guessed write-tap address never worked
    (the object's physical address varies run to run),
    but this site tells us the real address at the
    earliest moment it becomes interesting - install a
    real write-tap for it right here, dynamically, and
    never remove it, so whichever object eventually
    crashes already has its complete write history
    captured from its first waiter onward. Tracks
    already-tapped physical addresses to avoid
    re-installing if the same object empties and refills
    more than once in a run.
-------------------------------------------------*/

static std::set<uint32_t> s_mutex_tap_installed;

void mips3_device::func_install_mutex_tap()
{
	const uint32_t obj = (uint32_t)m_core->r[4];
	const uint32_t phys = obj & 0x1fffffff;
	const uint32_t aligned = phys & ~7u;
	if (s_mutex_tap_installed.count(aligned))
		return;
	s_mutex_tap_installed.insert(aligned);
	fprintf(stderr, "[MAME_DRC_DIAG] MUTEXTAP installing for obj=0x%08x phys=0x%08x (aligned=0x%08x)\n", obj, phys, aligned);
	fflush(stderr);
	m_program->install_write_tap(aligned, aligned + 7, "mutex_watch",
		[this, obj](offs_t offset, u64 &data, u64 mem_mask)
		{
			fprintf(stderr, "[MAME_DRC_DIAG] MUTEXTAP write obj=0x%08x phys=0x%08x data=0x%016llx mask=0x%016llx pc=0x%08x\n",
				obj, (uint32_t)offset, (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
			fflush(stderr);
		});
}

#ifdef MAME_DRC_DIAG
static void cfunc_install_mutex_tap(void *param) { ((mips3_device *)param)->func_install_mutex_tap(); }
#endif

/*-------------------------------------------------
    cfunc_mutex_unlock_entry_probe - TEMPORARY
    (2026-07-24): fires at mutex_unlock's own literal
    first instruction (0x880ea53c). At this exact point
    $ra (r31) still holds the ORIGINAL caller's return
    address live in the register - mutex_unlock hasn't
    made any calls of its own yet to overwrite it, so
    no stack-save/sign-extension complications like the
    mutex_wake case needed. Logs (mutex pointer, caller)
    for every call, to empirically narrow mutex_unlock's
    1267 static call sites down to whichever ones are
    actually exercised - especially for the crashing
    object - rather than tracing all of them blindly.
-------------------------------------------------*/

void mips3_device::func_mutex_unlock_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t obj = (uint32_t)m_core->r[4];
	const uint32_t caller = (uint32_t)m_core->r[31];
	fprintf(stderr, "[MAME_DRC_DIAG] DRC mutex_unlock #%llu obj=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, obj, caller);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_mutex_unlock_entry_probe(void *param) { ((mips3_device *)param)->func_mutex_unlock_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_sv_wait_sig_unlock_probe - TEMPORARY
    (2026-07-24): fires at sv_wait_sig's own call to
    mutex_unlock (0x880eb7f0, the `jal 0x880ea53c` whose
    return address 0x880eb7f8 was already confirmed as
    the crashing call's caller). $s2/r18 is the register
    moved into $a0 in the delay slot (the mutex pointer
    argument); $s0/r16 and $a3/r7 are logged alongside
    since static tracing back through sv_wait_sig's own
    heavy register reuse couldn't pin down with certainty
    which of $s2's several candidate definitions is live
    at this specific call site.
-------------------------------------------------*/

static bool s_in_sv_wait_release_window = false;

void mips3_device::func_sv_wait_sig_unlock_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t s2 = (uint32_t)m_core->r[18];
	const uint32_t s0 = (uint32_t)m_core->r[16];
	const uint32_t a3 = (uint32_t)m_core->r[7];
	fprintf(stderr, "[MAME_DRC_DIAG] DRC sv_wait_sig unlock #%llu s2=0x%08x s0=0x%08x a3=0x%08x\n",
		(unsigned long long)s_count, s2, s0, a3);
	fflush(stderr);
	// TEMPORARY (2026-07-25): this is the log's lock being released
	// right before sv_wait_sig actually blocks/context-switches - the
	// window between here and xlog_state_get_iclog_space re-entering
	// its own splhi-protected loop (after waking back up) is where the
	// thread genuinely doesn't hold the lock. Bracketed the same way as
	// the splhi/splx window, to check the same interrupt-timing question
	// one level further out, since bracketing sv_wait_sig's own internal
	// reacquire instruction couldn't be pinned down with certainty by
	// static reading alone (this function has many distinct wake paths).
	s_in_sv_wait_release_window = true;
}

#ifdef MAME_DRC_DIAG
static void cfunc_sv_wait_sig_unlock_probe(void *param) { ((mips3_device *)param)->func_sv_wait_sig_unlock_probe(); }
#endif

/*-------------------------------------------------
    cfunc_sv_wait_sig_worker_entry_probe - TEMPORARY
    (2026-07-24): fires at the real entry of the unnamed
    static worker function that starts at 0x880eb588
    (a separate function from sv_wait_sig itself, sharing
    its nm symbol range only because it has no symbol of
    its own - confirmed by its own `addiu $sp,$sp,-0x30`
    prologue and sv_wait_sig's own `jr $ra` return just
    before it at 0x880eb520). Hooked at 0x880eb598 (the
    first instruction after the initial stack-pointer
    adjustment/register saves), where $a0-$a3 still hold
    the pristine incoming arguments. $a0 is the object the
    0x880eb5f0 queue-insert block operates on; $a3 becomes
    $s2, the pointer later passed to mutex_unlock. Logging
    both together settles whether they ever alias the same
    object - static reading alone couldn't tell. Also logs
    $ra: the worker has 8 static callers (sv_mrlock_wait[_sig],
    sv_sema_wait[_sig], sv_bitlock_timedwait[_sig], sv_timedwait[_sig])
    passing $a2 as either a fixed literal or a passthrough of
    their own caller's argument - tracing which literal case
    matches a2=3 hit ambiguity in the passthrough wrappers, so
    logging the actual caller directly is more reliable than
    finishing that static trace by hand.
-------------------------------------------------*/

void mips3_device::func_sv_wait_sig_worker_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC sv_wait_sig_worker #%llu a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[5],
		(uint32_t)m_core->r[6], (uint32_t)m_core->r[7], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_sv_wait_sig_worker_entry_probe(void *param) { ((mips3_device *)param)->func_sv_wait_sig_worker_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_sv_timedwait_entry_probe - TEMPORARY
    (2026-07-24): the crashing sv_wait_sig_worker call
    traced back one level to sv_timedwait (0x880ec28c),
    which passes its own $a2 argument straight through to
    the worker unchanged (dynamic, not a fixed literal) -
    so the a2=3/kseg2-a0 anomaly must originate from
    whoever calls sv_timedwait this way. Hooked at its
    literal first instruction (0x880ec28c, a `lw` that
    doesn't touch $a0-$a3), logging the pristine incoming
    arguments plus $ra to identify the actual caller among
    sv_timedwait's 5 static call sites (two of which fall
    in unnamed static functions with no symbol of their
    own, per this kernel's sparse symbol table).
-------------------------------------------------*/

void mips3_device::func_sv_timedwait_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC sv_timedwait #%llu a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[5],
		(uint32_t)m_core->r[6], (uint32_t)m_core->r[7], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_sv_timedwait_entry_probe(void *param) { ((mips3_device *)param)->func_sv_timedwait_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_sv_wait_entry_probe - TEMPORARY (2026-07-24):
    sv_timedwait's own caller for the crashing chain is
    always sv_wait (0x880eb3dc), a trivial passthrough
    wrapper (confirmed by reading its body - just two
    `move reg,$0` no-ops before its own prologue) with 70
    static call sites of its own, too many to trace by
    hand. Hooked at its `addiu $sp,$sp,-0x10` prologue
    instruction (0x880eb3e4) - $ra and $a0-$a3 are all
    still pristine at this point, since nothing before it
    in the function touches them.
-------------------------------------------------*/

void mips3_device::func_sv_wait_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC sv_wait #%llu a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[5],
		(uint32_t)m_core->r[6], (uint32_t)m_core->r[7], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_sv_wait_entry_probe(void *param) { ((mips3_device *)param)->func_sv_wait_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_state_clean_log_entry_probe - TEMPORARY
    (2026-07-24): the crashing chain traced back to
    sv_wait's caller resolving to xlog_state_clean_log
    (0x881e301c, XFS's log-cleanup function) via one of
    its 7 internal sv_wait call sites. That function turns
    out to itself be a thin wrapper (`addiu $sp,$sp,-0x30`
    prologue, only ~0x160 bytes) around a third unnamed
    static worker at 0x881e357c, whose own first instruction
    spills its fresh $a2 straight to sp+8 - the exact stack
    slot the crashing sv_wait call reloads its mutex
    argument from. That means the mutex isn't computed
    anywhere in this call chain - it's passed straight
    through unchanged. Hooked at the outer wrapper's literal
    first instruction (also `addiu`, still MIPS3DRC_DIAG
    opcode 0x09) to capture its own pristine arguments and
    real caller, one level closer to the true origin.
-------------------------------------------------*/

void mips3_device::func_xlog_state_clean_log_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC xlog_state_clean_log #%llu a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[5],
		(uint32_t)m_core->r[6], (uint32_t)m_core->r[7], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_state_clean_log_entry_probe(void *param) { ((mips3_device *)param)->func_xlog_state_clean_log_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_biodone_dispatch_probe - TEMPORARY (2026-07-25):
    biodone() (0x880b6350) dispatches a completed buf's
    completion callback indirectly - `lw $a3,0x80($a0)`
    then `jalr $a3` at 0x880b63a4, with the buf pointer
    held live in $s0 (r16) since biodone's own prologue.
    Static searching (immediate-construction scan, raw data-
    word scan, broad offset-0x80-store scan across the XFS
    log code) could not find where a buf's own 0x80/0x8c
    fields (b_iodone callback / private context) get
    WRITTEN in the first place - likely another unnamed
    static helper with no symbol of its own, the same gap
    that's shown up repeatedly tonight. Logging every
    dispatch here, and specifically flagging when the target
    equals xlog_iodone (0x881e2148), then installing a
    dynamic write-tap on that buf's own fields (the same
    "learn the real address live, then tap it going forward"
    technique used earlier for the mutex object) to catch the
    assignment on this buf's *next* reuse cycle, since static
    tracing of the original assignment came up empty.
-------------------------------------------------*/

static std::set<uint32_t> s_biodone_tap_installed;

void mips3_device::func_biodone_dispatch_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t buf = (uint32_t)m_core->r[16];
	const uint32_t target = (uint32_t)m_core->r[7];
	const bool is_xlog_iodone = (target == 0x881e2148);
	fprintf(stderr, "[MAME_DRC_DIAG] DRC biodone_dispatch #%llu buf=0x%08x target=0x%08x%s\n",
		(unsigned long long)s_count, buf, target, is_xlog_iodone ? " <== XLOG_IODONE" : "");
	fflush(stderr);
	if (is_xlog_iodone)
	{
		const uint32_t phys = buf & 0x1fffffff;
		const uint32_t aligned = phys & ~7u;
		if (!s_biodone_tap_installed.count(aligned))
		{
			s_biodone_tap_installed.insert(aligned);
			fprintf(stderr, "[MAME_DRC_DIAG] BIODONE_TAP installing for buf=0x%08x phys=0x%08x (range 0x%08x-0x%08x)\n",
				buf, phys, aligned, aligned + 0xff);
			fflush(stderr);
			m_program->install_write_tap(aligned, aligned + 0xff, "biodone_buf_watch",
				[this, buf](offs_t offset, u64 &data, u64 mem_mask)
				{
					fprintf(stderr, "[MAME_DRC_DIAG] BIODONE_TAP write buf=0x%08x phys=0x%08x data=0x%016llx mask=0x%016llx pc=0x%08x\n",
						buf, (uint32_t)offset, (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
					fflush(stderr);
				});
		}
	}
}

#ifdef MAME_DRC_DIAG
static void cfunc_biodone_dispatch_probe(void *param) { ((mips3_device *)param)->func_biodone_dispatch_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_iodone_entry_probe - TEMPORARY (2026-07-25):
    the biodone() dispatch probe just disproved the working
    assumption that biodone's `jalr` is how xlog_iodone gets
    invoked - across a full run reproducing the crash, every
    one of 66 biodone dispatches targeted a different address
    (inside xlog_bwrite's own range), never xlog_iodone. XFS
    log I/O completion must route through some other,
    not-yet-found dispatch mechanism. Rather than keep
    guessing at intermediate dispatchers, hook xlog_iodone's
    own literal entry directly (0x881e2148, `li $at,1` =
    `addiu $at,$0,1`) - unlike xlog_state_clean_log, this
    function's symbol boundaries are clean and confirmed, so
    $a0/$ra here should reliably show the real caller.
-------------------------------------------------*/

void mips3_device::func_xlog_iodone_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC xlog_iodone #%llu buf=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_iodone_entry_probe(void *param) { ((mips3_device *)param)->func_xlog_iodone_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_complete_entry_probe - TEMPORARY
    (2026-07-25): xlog_iodone's own literal entry
    (0x881e2148) turned out to be dead code for this crash's
    call chain - never once compiled or hit despite the chain
    clearly executing nearby. The compile log showed the real
    function actually entered is 0x881e2b28 (confirmed real
    prologue, `addiu $sp,$sp,-0xd0`), with real static callers
    this time: xfs_log_write and xfs_log_unmount_write - both
    direct, synchronous callers, not indirect/interrupt
    dispatch. Hooked at its own literal first instruction
    (`lw $v1,0x138($a0)`, LW opcode) to see, for the crash's
    specific invocation, which of those two real callers (or
    the third, still-unresolved one) it actually came from.

    UPDATE (2026-07-25): the crash's own mutex_wake call
    reloads its mutex pointer for sv_wait from a stack-spilled
    slot (`ld $a2,8($sp)`) deep inside the callee this function
    calls (0x881e357c) - that stack slot gets reused for many
    unrelated purposes across that huge function's body, too
    tangled to trace by hand reliably. Extended to log all four
    incoming arguments here (only $a0 was captured before) to
    check directly whether the crash's mutex value is simply
    this function's own argument, threaded straight through
    unchanged - the same passthrough pattern confirmed at every
    other hop in this chain tonight (sv_wait_sig_worker,
    sv_timedwait, sv_wait).
-------------------------------------------------*/

void mips3_device::func_xlog_complete_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t a0 = (uint32_t)m_core->r[4];
	const uint32_t a1 = (uint32_t)m_core->r[5];
	const uint32_t a2 = (uint32_t)m_core->r[6];
	const uint32_t a3 = (uint32_t)m_core->r[7];
	fprintf(stderr, "[MAME_DRC_DIAG] DRC xlog_complete #%llu a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, a0, a1, a2, a3, (uint32_t)m_core->r[31]);
	// Neither a0 nor a3 (the two stable-across-calls candidates) equals
	// the crash's known mutex value (0x883d6728) directly, so it isn't a
	// simple passthrough - it must be a field read from one of them (or
	// something they point to). Scan nearby memory for a word matching
	// that known value rather than guess field offsets from a possibly-
	// mismatched reference source.
	if (a0 >= 0x80000000 && a0 < 0xa0000000)
	{
		for (int32_t off = -0x40; off <= 0x400; off += 4)
		{
			const uint32_t addr = (a0 + (uint32_t)off) & 0x1fffffff;
			if (m_program->read_dword(addr) == 0x883d6728)
				fprintf(stderr, "[MAME_DRC_DIAG] XLOG_COMPLETE_SCAN match: a0+%+d = 0x883d6728\n", off);
		}
	}
	if (a3 >= 0x80000000 && a3 < 0xa0000000)
	{
		for (int32_t off = -0x40; off <= 0x400; off += 4)
		{
			const uint32_t addr = (a3 + (uint32_t)off) & 0x1fffffff;
			if (m_program->read_dword(addr) == 0x883d6728)
				fprintf(stderr, "[MAME_DRC_DIAG] XLOG_COMPLETE_SCAN match: a3+%+d = 0x883d6728\n", off);
		}
	}
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_complete_entry_probe(void *param) { ((mips3_device *)param)->func_xlog_complete_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_inner_worker_stack_tap - TEMPORARY
    (2026-07-25): the memory-scan diagnostic ruled out a
    simple one-level field read for the crash's mutex value -
    it isn't sitting within 0x400 bytes of either candidate
    parent structure. The actual `ld $a2,8($sp)` at the real
    sv_wait call site (0x881e4410) reloads from a stack slot
    reused constantly across this huge function
    (0x881e357c-ish, its own `addiu $sp,$sp,-0x60` prologue),
    too tangled to trace by hand. Hooked at that literal first
    instruction to catch $sp for THIS invocation, then installs
    a genuine write-tap covering the whole local frame
    (sp to sp+0x60) - $sp is a kseg3/TLB-mapped address here
    (confirmed via the mutex_wake stack-window work earlier
    tonight), so real vtlb translation is required, not the
    kseg0 `&0x1fffffff` mask used elsewhere. No dedup - this
    function is called only a few dozen times per run (unlike
    the 4000+ mutex_unlock calls), so re-tapping each
    invocation's frame is cheap and each invocation's writes
    are what we actually want to see.
-------------------------------------------------*/

static uint32_t s_xlog_inner_entry_sp = 0;
static uint32_t s_xlog_inner_entry_phys = 0;

void mips3_device::func_xlog_inner_worker_stack_tap()
{
	const uint32_t sp = (uint32_t)m_core->r[29];
	const uint32_t tlbval = vtlb_table()[sp >> 12];
	if (!(tlbval & READ_ALLOWED))
	{
		fprintf(stderr, "[MAME_DRC_DIAG] XLOG_INNER_ENTRY sp=0x%08x NOT READ_ALLOWED, skipping tap\n", sp);
		fflush(stderr);
		return;
	}
	const uint32_t phys_base = (tlbval & ~0xfff) | (sp & 0xfff);
	s_xlog_inner_entry_sp = sp;
	s_xlog_inner_entry_phys = phys_base;
	fprintf(stderr, "[MAME_DRC_DIAG] XLOG_INNER_ENTRY sp=0x%08x phys=0x%08x installing stack tap\n", sp, phys_base);
	fflush(stderr);
	m_program->install_write_tap(phys_base, phys_base + 0x5f, "xlog_inner_stack",
		[this, sp, phys_base](offs_t offset, u64 &data, u64 mem_mask)
		{
			fprintf(stderr, "[MAME_DRC_DIAG] XLOG_INNER_STACK write sp=0x%08x off=%+d data=0x%016llx mask=0x%016llx pc=0x%08x\n",
				sp, (int32_t)((uint32_t)offset - phys_base), (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
			fflush(stderr);
		});
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_inner_worker_stack_tap(void *param) { ((mips3_device *)param)->func_xlog_inner_worker_stack_tap(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_tlb_recheck_probe - TEMPORARY (2026-07-25):
    the stack-slot write tap (installed both statically in
    device_start() and dynamically on execute_run()'s first
    call) shows zero writes despite a non-zero, plausible
    value sitting there - genuinely contradicting RAM's
    zero-initialization guarantee if the tap is watching the
    right physical address the whole time. One live
    possibility not yet ruled out: this is a kseg3/TLB-mapped
    address, and the physical translation was only captured
    ONCE, at this function's entry - if the TLB entry for
    this virtual page gets evicted and refilled with a
    DIFFERENT physical page before the actual store happens
    (a few instructions later, at 0x881e440c's delay slot),
    the tap would be watching a stale, wrong address, and
    "zero writes" would be a translation artifact, not a real
    finding. Hooked right before the crash's own call site
    (0x881e43f4) to re-read the same virtual page's TLB
    translation and compare directly against what was
    captured at entry.
-------------------------------------------------*/

void mips3_device::func_xlog_tlb_recheck_probe()
{
	const uint32_t sp = (uint32_t)m_core->r[29];
	const uint32_t tlbval = vtlb_table()[sp >> 12];
	const uint32_t phys_base = (tlbval & READ_ALLOWED) ? ((tlbval & ~0xfff) | (sp & 0xfff)) : 0xffffffff;
	const bool matches = (sp == s_xlog_inner_entry_sp) && (phys_base == s_xlog_inner_entry_phys);
	fprintf(stderr, "[MAME_DRC_DIAG] XLOG_TLB_RECHECK sp=0x%08x phys_now=0x%08x entry_sp=0x%08x entry_phys=0x%08x match=%d\n",
		sp, phys_base, s_xlog_inner_entry_sp, s_xlog_inner_entry_phys, matches ? 1 : 0);
	if (phys_base != 0xffffffff)
	{
		// Read the correct slot (sp+8, using THIS correctly-derived
		// physical address, not the stale entry-time one) right now,
		// before the crash's own read a few instructions later, and
		// install a fresh dynamic tap on it for the rest of this
		// invocation - the earlier full-boot taps were almost certainly
		// watching the wrong address given the sp mismatch just found.
		const uint32_t slot_phys = phys_base + 8;
		// The crash's own read is a 64-bit `ld`, and per the earlier
		// saved_ra big-endian bug (2026-07-24), the sign-extension upper
		// half sits at the LOWER address for a real kseg0 pointer, with
		// the actual 32-bit value 4 bytes later. Read both halves this
		// time instead of assuming.
		const uint32_t upper_half = m_program->read_dword(slot_phys);
		const uint32_t lower_half = m_program->read_dword(slot_phys + 4);
		fprintf(stderr, "[MAME_DRC_DIAG] XLOG_TLB_RECHECK correct_slot_phys=0x%08x upper_half=0x%08x lower_half=0x%08x\n",
			slot_phys, upper_half, lower_half);
		fflush(stderr);
		m_program->install_write_tap(slot_phys, slot_phys + 7, "xlog_correct_stack_slot",
			[this, slot_phys](offs_t offset, u64 &data, u64 mem_mask)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] XLOG_CORRECT_SLOT_WRITE phys=0x%08x data=0x%016llx mask=0x%016llx pc=0x%08x\n",
					(uint32_t)offset, (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
				fflush(stderr);
			});
	}
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_tlb_recheck_probe(void *param) { ((mips3_device *)param)->func_xlog_tlb_recheck_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xfs_log_done_entry_probe - TEMPORARY
    (2026-07-25): resuming the caller-chain trace set aside
    for the tap-reliability detour. xfs_log_done (0x881e1680)
    has exactly 4 static jal callers (xfs_trans_reserve,
    xfs_trans_commit x2 call sites, xfs_trans_cancel), found
    via jal-scan but never dynamically confirmed which one is
    live for the crash's specific invocation. Hooked at its
    own literal first instruction (`addiu $sp,$sp,-0x30`) to
    log $a0 (mp, its own first arg) and $ra (real caller)
    directly.
-------------------------------------------------*/

void mips3_device::func_xfs_log_done_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC xfs_log_done #%llu a0=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xfs_log_done_entry_probe(void *param) { ((mips3_device *)param)->func_xfs_log_done_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_commit_record_entry_probe - TEMPORARY
    (2026-07-25): xfs_log_done calls fn@0x881e2628 (very
    likely xlog_commit_record, matching the real source's
    `xlog_commit_record(mp, ticket, iclog, &lsn)` call
    pattern almost exactly - $a3 is visibly set to the
    address of xfs_log_done's own local "lsn" stack
    variable right at this call). $a2 (the iclog** parameter)
    isn't set explicitly right before the call on any path
    visible in the static disassembly - it's whatever
    survived from earlier branches, some of which clobber it
    with a -1 sentinel for an unrelated purpose. Hooked at
    this function's own literal entry (`addiu $sp,$sp,-0x20`)
    to capture its real, live arguments directly instead of
    resolving the ambiguity by hand.
-------------------------------------------------*/

void mips3_device::func_xlog_commit_record_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] DRC xlog_commit_record #%llu a0=0x%08x a1=0x%08x a2=0x%08x a3=0x%08x caller=0x%08x\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[5],
		(uint32_t)m_core->r[6], (uint32_t)m_core->r[7], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_commit_record_entry_probe(void *param) { ((mips3_device *)param)->func_xlog_commit_record_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_write_call_capture_probe /
    cfunc_xlog_write_return_check_probe - TEMPORARY
    (2026-07-25): direct disassembly of xlog_commit_record
    (0x881e2628, via capstone since no MIPS objdump is
    available here) confirmed the N32 calling convention -
    physical registers $8-$11 are a4-a7 here, NOT the O32
    t0-t3 that a naive disassembler prints them as. $9 (a5)
    holds `commit_iclog` at the `jal 0x881e2b28` (xlog_write)
    call site (0x881e2654); $2 (v0) holds xlog_write's return
    code at the following `beqz $v0` (0x881e265c). Both
    error-check hops (this one and xfs_log_done's) were
    already confirmed to compile as faithful, correct checks
    of that return code - so the open question is no longer
    "is the check well-formed" but "does xlog_write actually
    return 0 (claimed success) on the crash's two stack-based
    calls despite never writing *commit_iclog". These two
    probes capture the address at the call, then read back
    what's actually sitting there right after the call
    returns, correlated with the real $v0 value - reading
    both 32-bit halves per the standing big-endian sd/lw
    methodology note, since it's not yet known which store
    width xlog_write uses for this assignment.
-------------------------------------------------*/

static uint32_t s_xlog_write_commit_iclog_vaddr = 0;
static std::set<uint32_t> s_xlog_write_commit_iclog_tapped_phys;

void mips3_device::func_xlog_write_call_capture_probe()
{
	const uint32_t vaddr = (uint32_t)m_core->r[9];
	s_xlog_write_commit_iclog_vaddr = vaddr;

	uint32_t phys = 0xffffffff;
	bool valid = false;
	if (vaddr >= 0x80000000 && vaddr < 0xc0000000)
	{
		phys = vaddr & 0x1fffffff;
		valid = true;
	}
	else
	{
		const uint32_t tlbval = vtlb_table()[vaddr >> 12];
		if (tlbval & READ_ALLOWED)
		{
			phys = (tlbval & ~0xfff) | (vaddr & 0xfff);
			valid = true;
		}
	}
	fprintf(stderr, "[MAME_DRC_DIAG] XLOG_WRITE_CALL vaddr=0x%08x phys=0x%08x valid=%d\n", vaddr, phys, valid ? 1 : 0);
	fflush(stderr);
	if (valid && s_xlog_write_commit_iclog_tapped_phys.insert(phys).second)
	{
		m_program->install_write_tap(phys, phys + 7, "xlog_commit_iclog_dynamic",
			[this, vaddr, phys](offs_t offset, u64 &data, u64 mem_mask)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] XLOG_COMMIT_ICLOG_WRITE vaddr=0x%08x phys_base=0x%08x offset=0x%08x data=0x%016llx mask=0x%016llx pc=0x%08x\n",
					vaddr, phys, (uint32_t)offset, (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
				fflush(stderr);
			});
	}
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_write_call_capture_probe(void *param) { ((mips3_device *)param)->func_xlog_write_call_capture_probe(); }
#endif

void mips3_device::func_xlog_write_return_check_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t v0 = (uint32_t)m_core->r[2];
	const uint32_t vaddr = s_xlog_write_commit_iclog_vaddr;
	uint32_t phys = 0xffffffff;
	bool valid = false;
	if (vaddr >= 0x80000000 && vaddr < 0xc0000000)
	{
		phys = vaddr & 0x1fffffff;
		valid = true;
	}
	else
	{
		const uint32_t tlbval = vtlb_table()[vaddr >> 12];
		if (tlbval & READ_ALLOWED)
		{
			phys = (tlbval & ~0xfff) | (vaddr & 0xfff);
			valid = true;
		}
	}
	if (!valid)
	{
		fprintf(stderr, "[MAME_DRC_DIAG] XLOG_WRITE_RETCHK #%llu v0=0x%08x iclog_vaddr=0x%08x TLB_MISS\n",
			(unsigned long long)s_count, v0, vaddr);
		fflush(stderr);
		return;
	}
	const uint32_t upper = m_program->read_dword(phys);
	const uint32_t lower = m_program->read_dword(phys + 4);
	fprintf(stderr, "[MAME_DRC_DIAG] XLOG_WRITE_RETCHK #%llu v0=0x%08x iclog_vaddr=0x%08x phys=0x%08x upper=0x%08x lower=0x%08x\n",
		(unsigned long long)s_count, v0, vaddr, phys, upper, lower);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_write_return_check_probe(void *param) { ((mips3_device *)param)->func_xlog_write_return_check_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xfs_trans_commit_iclog_consume_probe - TEMPORARY
    (2026-07-25): the write tap on xlog_write's target
    address showed a correct write even on the crash's two
    stack-based calls, then a flood of unrelated-looking
    writes to the same physical address right after - which
    could just as easily be a different process's kernel
    stack reusing the same virtual (and, post context-switch,
    a different physical) page as this one, since the tap's
    physical address was fixed at install time. That makes
    the tap's own "flood" inconclusive. The real test is
    narrower: what does xfs_trans_commit's own first read-back
    of this local (`lw $a1,0x80($sp)` at 0x881d8128, one call
    to fn@0x881d7d38 after xlog_done returns) actually see, for
    THIS frame, right now. Reads $sp fresh here (a long-lived,
    always-correctly-maintained register, unlike a just-returned
    call's return value) and re-derives the physical address at
    this exact moment rather than trusting the LW's own
    destination register or an address computed earlier.
-------------------------------------------------*/

void mips3_device::func_xfs_trans_commit_iclog_consume_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t sp = (uint32_t)m_core->r[29];
	const uint32_t vaddr = sp + 0x80;
	uint32_t phys = 0xffffffff;
	bool valid = false;
	if (vaddr >= 0x80000000 && vaddr < 0xc0000000)
	{
		phys = vaddr & 0x1fffffff;
		valid = true;
	}
	else
	{
		const uint32_t tlbval = vtlb_table()[vaddr >> 12];
		if (tlbval & READ_ALLOWED)
		{
			phys = (tlbval & ~0xfff) | (vaddr & 0xfff);
			valid = true;
		}
	}
	if (!valid)
	{
		fprintf(stderr, "[MAME_DRC_DIAG] XFS_TRANS_COMMIT_ICLOG_CONSUME #%llu sp=0x%08x vaddr=0x%08x TLB_MISS\n",
			(unsigned long long)s_count, sp, vaddr);
		fflush(stderr);
		return;
	}
	const uint32_t value = m_program->read_dword(phys);
	fprintf(stderr, "[MAME_DRC_DIAG] XFS_TRANS_COMMIT_ICLOG_CONSUME #%llu sp=0x%08x vaddr=0x%08x phys=0x%08x value=0x%08x\n",
		(unsigned long long)s_count, sp, vaddr, phys, value);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_xfs_trans_commit_iclog_consume_probe(void *param) { ((mips3_device *)param)->func_xfs_trans_commit_iclog_consume_probe(); }
#endif

/*-------------------------------------------------
    cfunc_xlog_write_entry_log_probe - TEMPORARY
    (2026-07-25): xlog_write's own first instruction
    (`lw $v1,0x138($a0)`) is `log = mp->m_log` - the exact
    persistent xlog_t pointer that flows down through
    xlog_state_get_iclog_space (confirmed via its 6-arg
    signature match at 0x881e2c48) into the wait-helper that
    computes `a0+0x28` as the mutex passed to sv_wait. The
    prior session's write tap on the crash's own known value
    (0x883d6728) never fired, but that address was found only
    by working backward from the crash - never cross-checked
    against a live, independently-derived `log` pointer.
    Dumps a live snapshot of log+0x00..+0x40 (real struct
    layout can't be pinned down from the source alone without
    the sema_t header, which isn't in the XFS-only source
    tree) and installs a dynamic write tap on log+0x28
    specifically, deduped per distinct log address, to test
    "nothing ever writes here" against a target verified by
    forward derivation instead of backward guessing.
-------------------------------------------------*/

static std::set<uint32_t> s_xlog_write_entry_log_tapped;

void mips3_device::func_xlog_write_entry_log_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t log_vaddr = (uint32_t)m_core->r[3];
	uint32_t phys = 0xffffffff;
	bool valid = false;
	if (log_vaddr >= 0x80000000 && log_vaddr < 0xc0000000)
	{
		phys = log_vaddr & 0x1fffffff;
		valid = true;
	}
	else
	{
		const uint32_t tlbval = vtlb_table()[log_vaddr >> 12];
		if (tlbval & READ_ALLOWED)
		{
			phys = (tlbval & ~0xfff) | (log_vaddr & 0xfff);
			valid = true;
		}
	}
	if (!valid)
	{
		fprintf(stderr, "[MAME_DRC_DIAG] XLOG_WRITE_ENTRY_LOG #%llu log=0x%08x TLB_MISS\n",
			(unsigned long long)s_count, log_vaddr);
		fflush(stderr);
		return;
	}
	fprintf(stderr, "[MAME_DRC_DIAG] XLOG_WRITE_ENTRY_LOG #%llu log=0x%08x phys=0x%08x snapshot:", (unsigned long long)s_count, log_vaddr, phys);
	for (uint32_t off = 0; off < 0x40; off += 4)
		fprintf(stderr, " +%02x=%08x", off, m_program->read_dword(phys + off));
	fprintf(stderr, "\n");
	fflush(stderr);
	if (s_xlog_write_entry_log_tapped.insert(phys).second)
	{
		const uint32_t tap_phys = phys + 0x28;
		m_program->install_write_tap(tap_phys, tap_phys + 7, "xlog_t_plus0x28_dynamic",
			[this, log_vaddr, tap_phys](offs_t offset, u64 &data, u64 mem_mask)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] XLOG_T_PLUS0X28_WRITE log=0x%08x tap_phys=0x%08x offset=0x%08x data=0x%016llx mask=0x%016llx pc=0x%08x\n",
					log_vaddr, tap_phys, (uint32_t)offset, (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
				fflush(stderr);
			});
	}
}

#ifdef MAME_DRC_DIAG
static void cfunc_xlog_write_entry_log_probe(void *param) { ((mips3_device *)param)->func_xlog_write_entry_log_probe(); }
#endif

/*-------------------------------------------------
    cfunc_mutex_wake_dequeue_probe - TEMPORARY
    (2026-07-25): the crash's own fault instruction,
    identified from the prior session's cached disassembly
    (`mutex_wake_full.asm`): `0x880eaf34: lw $a0,0x98($a2)`
    dereferences `next` (loaded one instruction earlier,
    `0x880eaf30: lw $a2,4($s0)`) with no null check, gated
    only on the mutex's own flags word having its "has
    waiters" bit set (`0x880eaf20-28`). Hooking the real
    crash PC itself is useless - the real memory read there
    (UML_CALLH read32) happens *before* our diagnostic hook
    and would fault/panic first. Hooked one instruction
    earlier instead, right after `next` loads but before it's
    dereferenced, so this fires (and survives) on every single
    dequeue attempt, crashing or not - not just the one that
    happens to panic. Captures the mutex address, the just-
    loaded `next` value, and a fresh read of the flags word,
    to see directly how often `next==0` while flags say
    "has waiters" actually occurs, rather than only inferring
    it after the fact from a panic.
-------------------------------------------------*/

void mips3_device::func_mutex_wake_dequeue_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t mutex_vaddr = (uint32_t)m_core->r[16];
	const uint32_t next_vaddr = (uint32_t)m_core->r[6];
	const uint32_t flags_word = m_program->read_dword(mutex_vaddr & 0x1fffffff);
	fprintf(stderr, "[MAME_DRC_DIAG] MUTEX_WAKE_DEQUEUE #%llu mutex=0x%08x flags_word=0x%08x next=0x%08x %s\n",
		(unsigned long long)s_count, mutex_vaddr, flags_word, next_vaddr,
		(next_vaddr == 0) ? "*** NEXT_IS_NULL ***" : "");
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_mutex_wake_dequeue_probe(void *param) { ((mips3_device *)param)->func_mutex_wake_dequeue_probe(); }
#endif

#ifdef MAME_IRIX_SPINLOCK_INIT_WORKAROUND
/*-------------------------------------------------
    cfunc_spinlock_init_workaround - see the
    MAME_IRIX_SPINLOCK_INIT_WORKAROUND definition at the top
    of this file for the full rationale. Hooked right after
    spinlock_init's own real store (`0x881b6540: sw $at,($a0)`,
    which unconditionally writes the literal 2) - overwrites
    the same word with 0 immediately after, for every lock_t
    in the kernel, every time spinlock_init runs.
-------------------------------------------------*/

void mips3_device::func_spinlock_init_workaround()
{
	const uint32_t lock_vaddr = (uint32_t)m_core->r[4];
	static uint64_t s_count = 0;
	s_count++;
	if (lock_vaddr >= 0x80000000 && lock_vaddr < 0xc0000000)
	{
		m_program->write_dword(lock_vaddr & 0x1fffffff, 0);
	}
	else
	{
		const uint32_t tlbval = vtlb_table()[lock_vaddr >> 12];
		if (tlbval & WRITE_ALLOWED)
		{
			const uint32_t phys = (tlbval & ~0xfff) | (lock_vaddr & 0xfff);
			m_program->write_dword(phys, 0);
		}
	}
	// DISABLED (2026-07-28): the workaround itself (the write_dword above)
	// stays active - it's real and needed - but this diagnostic print fired
	// on every single occurrence (64,000+ times in a 15-minute run, ~70/sec)
	// and was never gated, unlike the probes disabled elsewhere in this file
	// tonight. s_count is kept (harmless, cheap) in case a future session
	// wants the print back for a specific investigation.
	(void)s_count;
}

static void cfunc_spinlock_init_workaround(void *param) { ((mips3_device *)param)->func_spinlock_init_workaround(); }
#endif

/*-------------------------------------------------
    cfunc_us_delay_entry_probe - TEMPORARY (2026-07-26):
    the post-fix hang spins at `pc=0x8801b64c`, which real
    kernel symbols (via `nm`) identify as `delayloop`, called
    from `us_delay` (`0x88012200`). us_delay's own disassembly
    shows a `beqz` on `private.decinsperloop` (gp-relative
    load) taking a fallback path when that calibration value
    is zero - i.e. the delay-length calibration done by
    `delay_calibrate()` (real source: irix/kern/ml/delay.c)
    hadn't produced a usable value yet when this specific call
    happened. Hooked at us_delay's own entry to capture the
    real, source-level "us" (microseconds) argument and the
    caller ($ra), to tell whether this is a legitimately long
    (if unlucky under emulation) requested delay, or something
    computing an absurd value.
-------------------------------------------------*/

static uint32_t s_us_delay_calls_since_new_lock = 0xffffffff;

void mips3_device::func_us_delay_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	// Quieted (2026-07-26): this fires on every single us_delay call - the
	// mrupdatef retry loop alone produced 7+ million calls in 15 real
	// seconds when this logged unconditionally, which (same lesson as the
	// execute_run periodic diagnostic earlier) dominates wall-clock time
	// independently of real emulated progress and makes genuine stalls
	// indistinguishable from diagnostic-induced slowness. Log only every
	// 65536th call by default - EXCEPT for the first 32 calls right after
	// a brand-new mrlock is first seen (func_mrupdatef_entry_probe resets
	// the shared counter below to 0), logged unconditionally: the backoff
	// only takes ~4 iterations to hit its cap, so whatever causes Status
	// to transition from clean (confirmed IE=1,IM=0xff at mrupdatef's own
	// entry) to fully masked has to happen within the first handful of
	// iterations - far too early for the 1-in-65536 sampling to ever see.
	const bool early_window = s_us_delay_calls_since_new_lock < 32;
	if (s_us_delay_calls_since_new_lock != 0xffffffff)
		s_us_delay_calls_since_new_lock++;
	if (!early_window && (s_count & 0xffff) != 0)
		return;
	// Added (2026-07-26): direct check of whether interrupts are genuinely
	// masked at the hardware level (Status/Cause, same fields
	// func_lockstate_probe already reads elsewhere) during a stall where
	// INTERRUPT_TAKEN had stopped incrementing entirely after flowing for a
	// while - distinguishes "DRC still isn't checking for a real, pending,
	// unmasked interrupt" (a remaining codegen gap) from "the guest's own
	// splhi/splx nesting genuinely left IM/IE masked" (correct behavior,
	// a real logic issue elsewhere, not a DRC bug).
	const uint32_t status = (uint32_t)m_core->cpr[0][COP0_Status];
	const uint32_t cause = (uint32_t)m_core->cpr[0][COP0_Cause];
	fprintf(stderr, "[MAME_DRC_DIAG] US_DELAY_ENTRY #%llu us=%u (0x%08x) caller=0x%08x status=0x%08x (IE=%u EXL=%u ERL=%u IM=0x%02x) cause=0x%08x (IP=0x%02x)\n",
		(unsigned long long)s_count, (uint32_t)m_core->r[4], (uint32_t)m_core->r[4], (uint32_t)m_core->r[31],
		status, status & 1, (status >> 1) & 1, (status >> 2) & 1, (status >> 8) & 0xff,
		cause, (cause >> 8) & 0xff);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_us_delay_entry_probe(void *param) { ((mips3_device *)param)->func_us_delay_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_mrupdatef_entry_probe - TEMPORARY (2026-07-26):
    the caller behind the post-fix livelock's us_delay flood
    (0x880e9aa4) is inside `mrupdatef` (0x880e99f0), a
    multi-reader-lock upgrade primitive - a different lock
    type (`mrlock_t`) from the whole rest of this
    investigation's `lock_t`/`sv_wait`/`mutex_unlock` chain.
    Its own disassembly shows the wait condition is
    `*(mrlock) & 0x7fff == 0` (bits 0-14 clear); on success it
    claims the lock via `*(mrlock) |= 0x4000` directly, no
    mutex_lock/sv_wait involved. Hooked at entry to capture
    the real object address ($a0) and install a permanent
    write tap on it (deduped per address) - the same technique
    used all session - to answer directly, rather than assume,
    whether *anything* ever writes to this word during the
    spin (a real answer to whether the wait condition could
    ever become true) before considering a DRC-side interrupt
    or memory-visibility explanation.
-------------------------------------------------*/

static std::set<uint32_t> s_mrlock_tap_installed;

void mips3_device::func_mrupdatef_entry_probe()
{
	const uint32_t obj = (uint32_t)m_core->r[4];
	uint32_t phys = 0xffffffff;
	bool valid = false;
	if (obj >= 0x80000000 && obj < 0xc0000000)
	{
		phys = obj & 0x1fffffff;
		valid = true;
	}
	else
	{
		const uint32_t tlbval = vtlb_table()[obj >> 12];
		if (tlbval & READ_ALLOWED)
		{
			phys = (tlbval & ~0xfff) | (obj & 0xfff);
			valid = true;
		}
	}
	// Arms the early-window detailed logging in func_us_delay_entry_probe -
	// deliberately unconditional, on every mrupdatef entry, not just the
	// first-ever-seen-object case below. A prior version placed this reset
	// inside the dedup block and the stuck instance turned out to (at
	// least sometimes) be a *repeat* call on an address already in
	// s_mrlock_tap_installed, so the reset never armed for it and the
	// early-window logging silently never fired for the actual case that
	// mattered - caught by a full first attempt producing zero early-
	// window lines instead of assuming that meant "no contention".
	s_us_delay_calls_since_new_lock = 0;
	// Only log/tap-install once per distinct object - mrupdatef is called
	// far too often (millions of times during the livelock) to log every
	// entry without flooding the log the same way execute_run's periodic
	// diagnostic did earlier today.
	const uint32_t aligned = phys & ~7u;
	if (valid && s_mrlock_tap_installed.insert(aligned).second)
	{
		static uint64_t s_count = 0;
		s_count++;
		const uint32_t curval = m_program->read_dword(phys);
		// Added (2026-07-26): capture Status/Cause at the moment mrupdatef
		// is FIRST entered for this specific lock - not during its later
		// retry spin - to check directly whether interrupts were already
		// fully masked before mrupdatef was ever called (an upstream
		// caller's responsibility) versus becoming masked only later,
		// inside mrupdatef's own retry loop.
		const uint32_t status = (uint32_t)m_core->cpr[0][COP0_Status];
		const uint32_t cause = (uint32_t)m_core->cpr[0][COP0_Cause];
		fprintf(stderr, "[MAME_DRC_DIAG] MRUPDATEF_ENTRY #%llu obj=0x%08x phys=0x%08x aligned=0x%08x curval=0x%08x caller=0x%08x status=0x%08x (IE=%u IM=0x%02x) cause=0x%08x (IP=0x%02x)\n",
			(unsigned long long)s_count, obj, phys, aligned, curval, (uint32_t)m_core->r[31],
			status, status & 1, (status >> 8) & 0xff, cause, (cause >> 8) & 0xff);
		fflush(stderr);
		m_program->install_write_tap(aligned, aligned + 7, "mrlock_watch",
			[this, obj](offs_t offset, u64 &data, u64 mem_mask)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] MRLOCK_WRITE obj=0x%08x phys=0x%08x data=0x%016llx mask=0x%016llx pc=0x%08x\n",
					obj, (uint32_t)offset, (unsigned long long)data, (unsigned long long)mem_mask, m_core->pc);
				fflush(stderr);
			});
	}
}

#ifdef MAME_DRC_DIAG
static void cfunc_mrupdatef_entry_probe(void *param) { ((mips3_device *)param)->func_mrupdatef_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_tlbmiss_entry_probe - TEMPORARY (2026-07-26):
    a live run with the COP0_Status clobber fix active got
    substantially further than any prior run, then hit
    PANIC: KERNEL FAULT / PC: 0x881050e8, EXC code 20 (Write
    Address Error), Bad addr 0xffffcf30 - real nm symbols
    resolve 0x881050e8 to tlbmiss+4 (0x881050e4 is tlbmiss's
    own entry, "addiu $sp,$sp,-0x50"; the fault is its very
    first frame-save, "sd $a3,0x10($sp)"), called from
    elocore_exl_3 (the real TLB-miss exception vector,
    VEC_tlbmiss/locore_exl_3 at 0x8801b6bc). A fault this early
    in a fresh exception handler's own prologue means $sp
    itself is garbage entering the vector - not a TLB/paging
    problem with the address being stored to, an address-error
    on the store instruction itself. Hooked at tlbmiss's own
    entry to capture $sp/$a0-$a2/$ra/$k0/$k1 and EPC/BadVAddr/
    Cause/Status directly, rather than inferring further from
    static disassembly alone.
-------------------------------------------------*/

void mips3_device::func_tlbmiss_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	fprintf(stderr, "[MAME_DRC_DIAG] TLBMISS_ENTRY #%llu sp=0x%08x a0=0x%08x a1=0x%08x a2=0x%08x ra=0x%08x k0=0x%08x k1=0x%08x epc=0x%08x badvaddr=0x%08x cause=0x%08x status=0x%08x\n",
		(unsigned long long)s_count,
		(uint32_t)m_core->r[29], (uint32_t)m_core->r[4], (uint32_t)m_core->r[5], (uint32_t)m_core->r[6],
		(uint32_t)m_core->r[31], (uint32_t)m_core->r[26], (uint32_t)m_core->r[27],
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->cpr[0][COP0_BadVAddr],
		(uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_Status]);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_tlbmiss_entry_probe(void *param) { ((mips3_device *)param)->func_tlbmiss_entry_probe(); }
#endif

/*-------------------------------------------------
    cfunc_fault_store_mode_probe - TEMPORARY
    (2026-07-26): hooked directly at the SD
    instructions that fault with KERNEL FAULT/AdES
    (0x881050e8 in tlbmiss, 0x881053a0 in tlbmod) -
    every memory accessor in this file, including
    this SD case, selects its handler via a *compile-
    time* read of m_core->mode (m_write64[m_core->mode
    >> 1], not mem(&m_core->mode)), baked into the
    UML_CALLH target once and reused on every future
    execution of this compiled instruction regardless
    of the CPU's real mode at execution time. Two
    hypotheses already ruled out this session (a
    diagnostic-induced clobber; the mode-baking used by
    branch dispatch, which is provably safe there) - this
    checks the one remaining, structurally different use
    of the same bare-m_core->mode pattern directly: is
    m_core->mode actually kernel (0) live, right before
    the fault, or does it disagree with what the compiled
    accessor was baked for. Deliberately ungated (not
    behind MAME_DRC_DIAG) and narrowly targeted at two
    exact PCs, so it costs nothing elsewhere.
-------------------------------------------------*/

void mips3_device::func_fault_store_mode_probe(int site)
{
	// VPDA_KSTACK = offsetof(pda_t, p_kstackflag) + PDAADDR (real source:
	// irix/kern/ml/genassym.c) - real disassembly confirms this resolves
	// to the fixed KSEG2 address 0xffffa03f (lb $k0, -0x5fc1($zero)).
	// 0=user, 1=kernel, 2=interrupt-context, 3+=idle - longway
	// (fullfault.s) branches on this single byte to decide which base to
	// carve the new exception frame's $sp from. Reading it live here to
	// check directly whether it holds a plausible value at the moment of
	// this fault, rather than continuing to infer from $sp alone.
	const uint32_t kstack_vaddr = 0xffffa03fu;
	uint32_t kstack_phys = 0xffffffffu;
	bool kstack_valid = false;
	const uint32_t tlbval = vtlb_table()[kstack_vaddr >> 12];
	if (tlbval & READ_ALLOWED)
	{
		kstack_phys = (tlbval & ~0xfff) | (kstack_vaddr & 0xfff);
		kstack_valid = true;
	}
	const uint8_t kstackflag = kstack_valid ? m_program->read_byte(kstack_phys) : 0xff;
	fprintf(stderr, "[MAME_DRC_DIAG] FAULT_STORE_MODE site=%d live_mode=0x%08x (mode>>1=%u) sp=0x%08x status=0x%08x (KSU=%u EXL=%u ERL=%u) epc=0x%08x cause=0x%08x badvaddr=0x%08x s0=0x%08x k0=0x%08x k1=0x%08x ra=0x%08x kstackflag=%d (valid=%d phys=0x%08x)\n",
		site, m_core->mode, m_core->mode >> 1, (uint32_t)m_core->r[29],
		(uint32_t)m_core->cpr[0][COP0_Status], ((uint32_t)m_core->cpr[0][COP0_Status] >> 3) & 3,
		(uint32_t)m_core->cpr[0][COP0_Status] & 2, (uint32_t)m_core->cpr[0][COP0_Status] & 4,
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_BadVAddr],
		(uint32_t)m_core->r[16], (uint32_t)m_core->r[26], (uint32_t)m_core->r[27], (uint32_t)m_core->r[31],
		(int)kstackflag, kstack_valid ? 1 : 0, kstack_phys);
	fflush(stderr);
}

#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_fault_store_mode_probe_1(void *param) { ((mips3_device *)param)->func_fault_store_mode_probe(1); }
static void cfunc_fault_store_mode_probe_2(void *param) { ((mips3_device *)param)->func_fault_store_mode_probe(2); }
static void cfunc_fault_store_mode_probe_3(void *param) { ((mips3_device *)param)->func_fault_store_mode_probe(3); }
#endif

/*-------------------------------------------------
    func_interrupt_delivery_probe - TEMPORARY
    (2026-07-27): hooked in static_generate_exception's
    EXCEPTION_INTERRUPT case, right before the "set EXL
    in the SR" OR - m_core->cpr[0][COP0_Status] here is
    still the value from *before* this exception's own
    update, and m_core->diag_new_epc is I0 stashed at
    the point it holds the about-to-be-written EPC
    (before it gets reused for the SR_EXL OR two UML ops
    later). Gated at the call site to fire only when the
    pre-existing Status already looks address-shaped
    (top byte 0x88, matching the kernel .text region all
    the anomalous Status/EPC/$k0 samples this session
    have shared) - this checks whether the corruption is
    already present *entering* this delivery (implicating
    something upstream) or only appears *after* it
    (implicating this function's own Status update).
-------------------------------------------------*/

void mips3_device::func_interrupt_delivery_probe()
{
	const uint32_t pre_status = (uint32_t)m_core->cpr[0][COP0_Status];
	// Runtime filter (can't gate this in UML without extra branching): only
	// print when Status already looks address-shaped *entering* this
	// delivery - a real Status register never has a nonzero top byte, but
	// every anomalous sample this session shares "top byte 0x88" (matching
	// this kernel's .text region). Firing unconditionally would be too
	// noisy (every interrupt delivery, thousands per boot).
	if ((pre_status & 0xff000000) != 0x88000000)
		return;
	fprintf(stderr, "[MAME_DRC_DIAG] INTERRUPT_DELIVERY pre_status=0x%08x new_epc=0x%08x old_epc=0x%08x cause=0x%08x k0=0x%08x k1=0x%08x ra=0x%08x mode=0x%08x\n",
		pre_status, m_core->diag_new_epc, (uint32_t)m_core->cpr[0][COP0_EPC],
		(uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->r[26], (uint32_t)m_core->r[27], (uint32_t)m_core->r[31],
		m_core->mode);
	fflush(stderr);
}

// DISABLED (2026-07-28): same performance-tax pattern already caught and
// disabled for the andi_ori/mtcz/fault_store_mode probe family above (see
// that macro's own comment) - this eret-race investigation concluded
// (wiki/assets.md) but this probe, unlike its siblings, was never gated
// behind MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES and kept firing unconditionally
// on every matching instruction all night, including throughout the entire
// DRC-cache-thrashing investigation. Confirmed via the "DRC still isn't
// remotely faster than -nodrc" symptom - a JIT-to-C-function-call tax on
// this much guest code is a far more direct explanation than the recompile-
// count mechanism chased earlier.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_interrupt_delivery_probe(void *param) { ((mips3_device *)param)->func_interrupt_delivery_probe(); }
#endif

/*-------------------------------------------------
    func_status_store_probe - TEMPORARY (2026-07-27):
    hooked on *every* COP0_Status store in
    generate_set_cop0_reg (not gated to one PC - this
    kernel has multiple locore_exl_N vectors doing the
    same narrow-mask style update at different addresses,
    so a single hardcoded PC would miss most instances).
    Filters internally: the interrupt-delivery probe
    already showed Status is never address-shaped
    *entering* any exception delivery, and that delivery
    path's only Status write is a single-bit OR that can't
    manufacture an address value on its own - so if the
    guest's own about-to-be-stored value (I0, stashed as
    diag_new_status) is already address-shaped *before*
    this store even runs, the corruption's origin is
    upstream of this store entirely: in whatever computed
    RTREG's value (likely a real ALU-instruction or
    register-read bug), not in this store or the exception
    machinery.
-------------------------------------------------*/

void mips3_device::func_status_store_probe()
{
	if ((m_core->diag_new_status & 0xff000000) != 0x88000000)
		return;
	fprintf(stderr, "[MAME_DRC_DIAG] STATUS_STORE pc=0x%08x new_status=0x%08x old_status=0x%08x epc=0x%08x k0=0x%08x k1=0x%08x ra=0x%08x mode=0x%08x\n",
		m_core->diag_status_store_pc, m_core->diag_new_status, (uint32_t)m_core->cpr[0][COP0_Status],
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->r[26], (uint32_t)m_core->r[27], (uint32_t)m_core->r[31],
		m_core->mode);
	fflush(stderr);
}

// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's comment -
// same never-gated performance-tax pattern.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_status_store_probe(void *param) { ((mips3_device *)param)->func_status_store_probe(); }
#endif

/*-------------------------------------------------
    func_andi_ori_probe - TEMPORARY (2026-07-27):
    hooked right after the ANDI/ORI opcodes' own store,
    gated to the same PC vicinity as the elocore_exl_N
    Status-restore sequence - checks r[16]($s0, the
    source) and r[26]($k0, the destination) immediately
    after each of the two instructions that are supposed
    to compute (s0&0xff00)|1 into k0, to localize exactly
    where in this three-instruction sequence a bad value
    first appears (if $s0 itself is already bad, if the
    andi/ori compute wrong, or if $k0 is still correct
    here and only goes bad afterward, before the mtc0).
-------------------------------------------------*/

void mips3_device::func_andi_ori_probe(int site)
{
	// TEMPORARY (2026-07-27): one-time print of &m_core->r[26]'s real host
	// address, to set a hardware watchpoint on it (gdb `watch`) - traps on
	// ANY write to this exact memory location regardless of which
	// JIT-generated code performs it, sidestepping the need to audit
	// every opcode case that can target register 26.
	static bool printed_addr = false;
	if (!printed_addr)
	{
		printed_addr = true;
		fprintf(stderr, "[MAME_DRC_DIAG] R26_ADDR &m_core->r[26]=%p\n", (void *)&m_core->r[26]);
		fflush(stderr);
	}
	// TEMPORARY (2026-07-27): added raw Status alongside the derived mode -
	// generate_update_mode forces mode=0 (kernel) whenever SR_EXL or
	// SR_ERL is set, regardless of the real KSU bits, so "mode=0" alone
	// can't distinguish "genuinely kernel, no exception in progress" from
	// "any privilege level, mid-exception (EXL=1)" - exactly the
	// distinction needed to know whether a nested interrupt should even
	// be deliverable here at all.
	const uint32_t status = (uint32_t)m_core->cpr[0][COP0_Status];
	fprintf(stderr, "[MAME_DRC_DIAG] ANDI_ORI site=%d s0=0x%08x%08x k0=0x%08x%08x mode=0x%08x status=0x%08x (EXL=%u ERL=%u IE=%u)\n",
		site, (uint32_t)(m_core->r[16] >> 32), (uint32_t)m_core->r[16],
		(uint32_t)(m_core->r[26] >> 32), (uint32_t)m_core->r[26], m_core->mode,
		status, (status >> 1) & 1, (status >> 2) & 1, status & 1);
	fflush(stderr);
}

#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_andi_ori_probe_1(void *param) { ((mips3_device *)param)->func_andi_ori_probe(1); }
static void cfunc_andi_ori_probe_2(void *param) { ((mips3_device *)param)->func_andi_ori_probe(2); }
#endif

/*-------------------------------------------------
    func_mtcz_read_probe - TEMPORARY (2026-07-27):
    hooked immediately after MTCz's own UML_DSEXT reads
    R32(RTREG) - the earliest possible point to inspect
    r[26] before generate_set_cop0_reg does anything else
    with it, closing the last possible gap between the
    ANDI_ORI probes (right after ori) and this read.
-------------------------------------------------*/

void mips3_device::func_mtcz_read_probe(int compiletime_is_branch_target)
{
	fprintf(stderr, "[MAME_DRC_DIAG] MTCZ_READ compiletime_is_branch_target=%d k0=0x%08x%08x mode=0x%08x\n",
		compiletime_is_branch_target, (uint32_t)(m_core->r[26] >> 32), (uint32_t)m_core->r[26], m_core->mode);
	fflush(stderr);
}

#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_mtcz_read_probe_0(void *param) { ((mips3_device *)param)->func_mtcz_read_probe(0); }
static void cfunc_mtcz_read_probe_1(void *param) { ((mips3_device *)param)->func_mtcz_read_probe(1); }
#endif

/*-------------------------------------------------
    func_sd_value_probe - TEMPORARY (2026-07-27):
    hooked in the generic SD opcode case (every 64-bit
    store anywhere in the kernel), filtered to only print
    when the value being stored matches the known
    corrupting family (0x8801b6c8/0x8801b668/0x8801b638,
    elocore_exl_3's own address across different builds).
    Real IRIX source (exc_common.s's ecommon) saves the
    live COP0_EPC into each exception frame's EF_EPC field
    via exactly this kind of store (sreg s3,EF_EPC(k1)) -
    if this catches that specific store, it directly shows
    what real COP0_EPC/Cause looked like at the moment a
    nested exception's own entry recorded this address as
    its resume point, closing the "why does EF_EPC end up
    holding elocore_exl_3's own PC" question.
-------------------------------------------------*/

void mips3_device::func_sd_value_probe()
{
	if (m_core->diag_sd_i1_stash < 0x8801b600 || m_core->diag_sd_i1_stash > 0x8801b700)
		return;
	fprintf(stderr, "[MAME_DRC_DIAG] SD_VALUE store_pc=0x%08x addr=0x%08x value=0x%08x%08x live_epc=0x%08x live_cause=0x%08x live_status=0x%08x k1=0x%08x%08x\n",
		m_core->diag_sd_pc, m_core->diag_sd_i0_stash,
		(uint32_t)(m_core->diag_sd_i1_stash >> 32), (uint32_t)m_core->diag_sd_i1_stash,
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_Status],
		(uint32_t)(m_core->r[27] >> 32), (uint32_t)m_core->r[27]);
	fflush(stderr);
}

// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's comment -
// same never-gated performance-tax pattern. This one is the worst of the
// four: hooked in the generic SD opcode case, meaning it fired on every
// 64-bit store anywhere in the kernel, unconditionally, all night.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_sd_value_probe(void *param) { ((mips3_device *)param)->func_sd_value_probe(); }
#endif

/*-------------------------------------------------
    func_premature_interrupt_probe - TEMPORARY
    (2026-07-27): hooked in generate_update_cycles,
    right after its SR_EXL|SR_ERL guard has already
    passed (i.e. delivery is about to happen), only for
    the three call sites (COP0_Status/Count/Compare's
    own desc->pc-based calls) already implicated in the
    STATUS_STORE finding. Real MIPS architecture defines
    SR_EXL specifically so no new interrupt can ever be
    taken while it's set - and elocore_exl_3 (this
    session's whole focus) is itself exception-vector
    code, so EXL should already be set throughout its
    execution. This checks directly, at the exact instant
    delivery is accepted, whether that's actually true.
-------------------------------------------------*/

void mips3_device::func_premature_interrupt_probe()
{
	fprintf(stderr, "[MAME_DRC_DIAG] PREMATURE_INTERRUPT at_pc=0x%08x status=0x%08x (EXL=%u ERL=%u IE=%u) cause=0x%08x epc=0x%08x k0=0x%08x k1=0x%08x mode=0x%08x\n",
		m_core->diag_status_store_pc, (uint32_t)m_core->cpr[0][COP0_Status],
		((uint32_t)m_core->cpr[0][COP0_Status] >> 1) & 1, ((uint32_t)m_core->cpr[0][COP0_Status] >> 2) & 1,
		(uint32_t)m_core->cpr[0][COP0_Status] & 1, (uint32_t)m_core->cpr[0][COP0_Cause],
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->r[26], (uint32_t)m_core->r[27], m_core->mode);
	fflush(stderr);
}

/*-------------------------------------------------
    func_eret_window_exception_diag / _eret_diag /
    _status_diag - TEMPORARY (2026-08-14): see
    MAME_SGI_ERET_RACE_WINDOW_DIAG's own comment near the
    top of this file for the investigation this serves.
    Each self-disables permanently (via a static latch)
    once total_cycles() passes the window's high bound, so
    the per-call total_cycles() read isn't paid for the
    remainder of a long-running boot once the window of
    interest has closed - these fire on ERET/COP0_Status-
    store/exception-delivery, which, unlike the disabled
    ANDI_ORI/SD family above, are infrequent enough that
    the cost while INSIDE the window is a non-issue.
-------------------------------------------------*/

// MAME_SGI_ERET_RACE_WINDOW_DIAG_VERBOSE (2026-09-11) - off by default.
// Restores the original unconditional, capped live stderr trace for all
// three probes below (full register state per event) for anyone actively
// re-investigating this window again (e.g. the still-open "does IRIX 5.3
// show the same race" question) - without it, the three probes below only
// count, silently, reported once at clean exit. See diag_eret_window_
// exception_count's comment in mips3.h.
// #define MAME_SGI_ERET_RACE_WINDOW_DIAG_VERBOSE

#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG_VERBOSE
namespace {
	// Shared across all three diagnostics below - caps TOTAL combined output
	// regardless of how frequent events turn out to be inside the window,
	// independent of the cycle-based latch (which only protects against
	// staying armed too long, not against a single window turning out to
	// be much noisier than expected). See the runaway-log incident noted
	// in this macro's own comment above.
	int s_mame_sgi_eret_window_hit_cap = 50;
}
#endif

#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG
void mips3_device::func_eret_window_exception_diag()
{
	static bool s_window_passed = false;
	if (s_window_passed)
		return;
	uint64_t cyc = total_cycles();
	if (cyc > MAME_SGI_ERET_RACE_WINDOW_HI)
	{
		s_window_passed = true;
		return;
	}
	if (cyc < MAME_SGI_ERET_RACE_WINDOW_LO)
		return;
	m_core->diag_eret_window_exception_count++;
#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG_VERBOSE
	if (--s_mame_sgi_eret_window_hit_cap < 0)
	{
		s_window_passed = true;
		return;
	}
	fprintf(stderr, "[MAME_ERET_WINDOW] EXCEPTION cyc=%llu type=%u epc=0x%08x cause=0x%08x status=0x%08x (EXL=%u ERL=%u IE=%u) pc=0x%08x k0=0x%08x%08x k1=0x%08x%08x ra=0x%08x%08x\n",
		(unsigned long long)cyc, m_core->diag_eret_window_exc_type,
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_Status],
		((uint32_t)m_core->cpr[0][COP0_Status] >> 1) & 1, ((uint32_t)m_core->cpr[0][COP0_Status] >> 2) & 1,
		(uint32_t)m_core->cpr[0][COP0_Status] & 1, m_core->pc,
		(uint32_t)(m_core->r[26] >> 32), (uint32_t)m_core->r[26],
		(uint32_t)(m_core->r[27] >> 32), (uint32_t)m_core->r[27],
		(uint32_t)(m_core->r[31] >> 32), (uint32_t)m_core->r[31]);
	fflush(stderr);
#endif
}
static void cfunc_eret_window_exception_diag(void *param) { ((mips3_device *)param)->func_eret_window_exception_diag(); }

void mips3_device::func_eret_window_eret_diag()
{
	static bool s_window_passed = false;
	if (s_window_passed)
		return;
	uint64_t cyc = total_cycles();
	if (cyc > MAME_SGI_ERET_RACE_WINDOW_HI)
	{
		s_window_passed = true;
		return;
	}
	if (cyc < MAME_SGI_ERET_RACE_WINDOW_LO)
		return;
	m_core->diag_eret_window_eret_count++;
#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG_VERBOSE
	if (--s_mame_sgi_eret_window_hit_cap < 0)
	{
		s_window_passed = true;
		return;
	}
	fprintf(stderr, "[MAME_ERET_WINDOW] ERET cyc=%llu pc=0x%08x pre_status=0x%08x (EXL=%u ERL=%u IE=%u) epc=0x%08x errorpc=0x%08x cause=0x%08x\n",
		(unsigned long long)cyc, m_core->diag_eret_window_pc, (uint32_t)m_core->cpr[0][COP0_Status],
		((uint32_t)m_core->cpr[0][COP0_Status] >> 1) & 1, ((uint32_t)m_core->cpr[0][COP0_Status] >> 2) & 1,
		(uint32_t)m_core->cpr[0][COP0_Status] & 1,
		(uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->cpr[0][COP0_ErrorPC], (uint32_t)m_core->cpr[0][COP0_Cause]);
	fflush(stderr);
#endif
}
static void cfunc_eret_window_eret_diag(void *param) { ((mips3_device *)param)->func_eret_window_eret_diag(); }

void mips3_device::func_eret_window_status_diag()
{
	static bool s_window_passed = false;
	if (s_window_passed)
		return;
	uint64_t cyc = total_cycles();
	if (cyc > MAME_SGI_ERET_RACE_WINDOW_HI)
	{
		s_window_passed = true;
		return;
	}
	if (cyc < MAME_SGI_ERET_RACE_WINDOW_LO)
		return;
	m_core->diag_eret_window_status_count++;
#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG_VERBOSE
	if (--s_mame_sgi_eret_window_hit_cap < 0)
	{
		s_window_passed = true;
		return;
	}
	fprintf(stderr, "[MAME_ERET_WINDOW] STATUS_STORE cyc=%llu pc=0x%08x new_status=0x%08x (EXL=%u ERL=%u IE=%u) old_status=0x%08x\n",
		(unsigned long long)cyc, m_core->diag_status_store_pc, m_core->diag_new_status,
		(m_core->diag_new_status >> 1) & 1, (m_core->diag_new_status >> 2) & 1, m_core->diag_new_status & 1,
		(uint32_t)m_core->cpr[0][COP0_Status]);
	fflush(stderr);
#endif
}
static void cfunc_eret_window_status_diag(void *param) { ((mips3_device *)param)->func_eret_window_status_diag(); }
#endif

#ifdef MAME_SGI_MUTEXWAIT_SPIN_DIAG
namespace {
	uint64_t s_mame_sgi_mutexwait_spin_calls = 0;
	uint64_t s_mame_sgi_intr_entry_calls = 0;
	osd_ticks_t s_mame_sgi_mutexwait_next_dump = 0;

	void mame_sgi_mutexwait_dump_and_reset()
	{
		osd_ticks_t now = osd_ticks();
		if (s_mame_sgi_mutexwait_next_dump == 0)
			s_mame_sgi_mutexwait_next_dump = now + osd_ticks_per_second() * 5;
		if (now < s_mame_sgi_mutexwait_next_dump)
			return;
		osd_printf_info("MAME_SGI_MUTEXWAIT_SPIN_DIAG: mutex_wait SPIN retries=%llu, intr() entries=%llu (last 5s window)\n",
			(unsigned long long)s_mame_sgi_mutexwait_spin_calls, (unsigned long long)s_mame_sgi_intr_entry_calls);
		s_mame_sgi_mutexwait_spin_calls = 0;
		s_mame_sgi_intr_entry_calls = 0;
		s_mame_sgi_mutexwait_next_dump = now + osd_ticks_per_second() * 5;
	}
}

static void cfunc_mutexwait_spin_probe(void *param)
{
	s_mame_sgi_mutexwait_spin_calls++;
	mame_sgi_mutexwait_dump_and_reset();
}

static void cfunc_intr_entry_probe(void *param)
{
	s_mame_sgi_intr_entry_calls++;
	mame_sgi_mutexwait_dump_and_reset();
}
#endif

#ifdef MAME_SGI_PCKM_MUTEX_DIAG
namespace {
	uint64_t s_mame_sgi_pckm_lock_calls = 0;
	osd_ticks_t s_mame_sgi_pckm_next_dump = 0;
}

void mips3_device::func_pckm_mutex_lock_call_probe()
{
	// Silent counter only - pckm_intr fires at an extremely high rate (a
	// real finding on its own, a possible keyboard/mouse-controller
	// interrupt storm), so per-call osd_printf_info here both floods any
	// listener and adds real emulation overhead at that call volume. Only
	// the rare, actually-interesting case (mutex genuinely held at entry)
	// gets logged, unconditionally regardless of the periodic snapshot's
	// own rate limit - that's the one thing this diagnostic exists to
	// catch.
	s_mame_sgi_pckm_lock_calls++;
	const uint32_t m_bits = m_program->read_dword(0x8832c6b8 & 0x1fffffff);
	if (m_bits)
		osd_printf_info("MAME_SGI_PCKM_MUTEX_DIAG: *** HELD *** pckm mutex_lock call #%llu, pckm_mutex.m_bits=0x%08x\n",
			(unsigned long long)s_mame_sgi_pckm_lock_calls, m_bits);
}

void mips3_device::func_pckm_mutex_snapshot_probe()
{
	osd_ticks_t now = osd_ticks();
	if (s_mame_sgi_pckm_next_dump == 0)
		s_mame_sgi_pckm_next_dump = now + osd_ticks_per_second() * 2;
	if (now < s_mame_sgi_pckm_next_dump)
		return;
	const uint32_t m_bits = m_program->read_dword(0x8832c6b8 & 0x1fffffff);
	// Real wall-clock HH:MM:SS, same format as serial_driver.py --timestamps,
	// so this diagnostic's output can be lined up directly against the
	// timestamped console log to see what boot stage the guest is at.
	time_t wall = ::time(nullptr);
	struct tm wall_tm;
	localtime_r(&wall, &wall_tm);
	char timebuf[16];
	strftime(timebuf, sizeof(timebuf), "%H:%M:%S", &wall_tm);
	osd_printf_info("MAME_SGI_PCKM_MUTEX_DIAG: [%s] periodic snapshot pckm_mutex.m_bits=0x%08x (%s), lock calls so far=%llu\n",
		timebuf, m_bits, m_bits ? "HELD" : "free", (unsigned long long)s_mame_sgi_pckm_lock_calls);
	s_mame_sgi_pckm_next_dump = now + osd_ticks_per_second() * 2;
}

static void cfunc_pckm_mutex_lock_call_probe(void *param) { ((mips3_device *)param)->func_pckm_mutex_lock_call_probe(); }
static void cfunc_pckm_mutex_snapshot_probe(void *param) { ((mips3_device *)param)->func_pckm_mutex_snapshot_probe(); }
#endif

// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's comment -
// same never-gated performance-tax pattern, two call sites.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
static void cfunc_premature_interrupt_probe(void *param) { ((mips3_device *)param)->func_premature_interrupt_probe(); }
#endif

/*-------------------------------------------------
    cfunc_rmap_del_panic_probe - TEMPORARY (2026-07-26):
    both fixes (spinlock_init workaround, interrupt-check
    generalization) confirmed working together get boot to a
    third, already-known symptom from the very start of this
    investigation: PANIC: rmap_del: pde 0x%x not in pfdat 0x%x
    (rmap_delmap_nolock, real source: irix/kern/os/rmap.c) -
    a consistency check that a page-table-entry pointer being
    removed from a physical page's reverse-map is actually
    recorded there. Two real panic call sites compiled in
    (0x881882a8 for the long-rmap "scan exhausted" case,
    0x881882cc for the short-rmap "neither slot matches"
    case); $s3 (r19) is pfdp and the real, N32 t0 (r12, NOT
    O32's r8 - the register-naming trap from earlier tonight)
    is pdep at both, set once at function entry and never
    reassigned before either panic site, with no intervening
    calls on either path - $ra (r31) is still the genuine,
    unclobbered immediate caller of rmap_delmap_nolock itself.
-------------------------------------------------*/

void mips3_device::func_rmap_del_panic_probe(int site)
{
	fprintf(stderr, "[MAME_DRC_DIAG] RMAP_DEL_PANIC site=%d pfdp=0x%08x pdep=0x%08x caller=0x%08x\n",
		site, (uint32_t)m_core->r[19], (uint32_t)m_core->r[12], (uint32_t)m_core->r[31]);
	fflush(stderr);
}

// Re-enabled (2026-07-30) under its own dedicated MAME_SGI_RMAP_DEL_PANIC_DIAG
// macro - see that macro's own comment. Originally disabled here as part of
// a bucket of genuinely-expensive probes it didn't actually belong in (two
// exact panic-site PCs, not a whole opcode class or hot loop).
#ifdef MAME_SGI_RMAP_DEL_PANIC_DIAG
static void cfunc_rmap_del_panic_probe_1(void *param) { ((mips3_device *)param)->func_rmap_del_panic_probe(1); }
static void cfunc_rmap_del_panic_probe_2(void *param) { ((mips3_device *)param)->func_rmap_del_panic_probe(2); }
#endif

/*-------------------------------------------------
    cfunc_iclogspace_splhi_probe / cfunc_iclogspace_splx_probe
    / cfunc_interrupt_taken_probe - TEMPORARY (2026-07-25):
    `0x8801205c`/`0x88012140`, called throughout
    `xlog_state_get_iclog_space`'s loop, turned out on direct
    disassembly to be `mfc0`/`mtc0` COP0-Status read-modify-
    write routines (an `splhi()`/`splx()` pair), not a lock
    object's acquire/release - no `$a0` object argument at
    all. This makes the loop's own state-check a critical
    section protected by disabling interrupts, not a spinlock.
    Given this kernel's DRC already has multiple confirmed,
    fixed interrupt-check timing bugs (see the "four real DRC
    bugs" list elsewhere in this file's history), the natural
    next question is whether DRC ever delivers an interrupt
    *inside* one of these splhi/splx brackets, which real
    hardware would have kept masked - real reentrancy into a
    section the guest kernel believes is exclusive.

    splhi is called exactly once per loop iteration, at the
    loop's own top (`0x881e35ec`); splx is called from five
    distinct exit points within the same function
    (`0x881e35c8`, `0x881e3660`, `0x881e3690`, `0x881e36cc`,
    `0x881e3730`, found by grepping this function's full
    disassembly for both target addresses) - tracked with one
    shared boolean rather than trying to prove which exit
    pairs with which entry.

    `cfunc_interrupt_taken_probe` is hooked directly inside
    `static_generate_exception`'s own generated code, gated on
    `exception == EXCEPTION_INTERRUPT`, so it fires on every
    real interrupt delivery regardless of guest PC - reading
    the freshly-written EPC (`m_core->cpr[0][COP0_EPC]`, set
    just above this hook point in the generated sequence) and
    cross-checking it against the critical-section flag.
-------------------------------------------------*/

static bool s_in_iclogspace_critical = false;

void mips3_device::func_iclogspace_splhi_probe()
{
	s_in_iclogspace_critical = true;
	// Re-entering the protected loop implies the lock released before
	// blocking in sv_wait_sig has been reacquired by now (see
	// func_sv_wait_sig_unlock_probe's comment for why the exact
	// reacquire instruction wasn't bracketed directly).
	s_in_sv_wait_release_window = false;
}

#ifdef MAME_DRC_DIAG
static void cfunc_iclogspace_splhi_probe(void *param) { ((mips3_device *)param)->func_iclogspace_splhi_probe(); }
#endif

void mips3_device::func_iclogspace_splx_probe()
{
	s_in_iclogspace_critical = false;
}

#ifdef MAME_DRC_DIAG
static void cfunc_iclogspace_splx_probe(void *param) { ((mips3_device *)param)->func_iclogspace_splx_probe(); }
#endif

/*-------------------------------------------------
    cfunc_mutex_lock_enqueue_probe / cfunc_mutex_lock_enqueue_end_probe
    - TEMPORARY (2026-07-26): brackets `mutex_lock`'s own
    contended slow path (real function at `0x880eabb4`,
    found via `nm`'s real kernel symbols this time rather
    than guessed - `mutex_lock`, `mutex_unlock`,
    `mutex_check_queue_order` are all real, resolvable
    symbols in this unstripped kernel). The slow path raises
    the interrupt level once at entry (`splhi`, `0x880eabd0`)
    before ever checking whether the mutex is contended, then
    - only on the contended branch - calls straight into the
    real queue-insertion routine at `0x880eae40` (an unnamed
    static sibling of `mutex_check_queue_order` sharing its
    `nm` range - the same symbol-boundary trap this
    investigation has hit before) without an intervening
    `splx`. Bracketed from that enqueue call itself
    (`0x880eac04`) to the two `splx` sites found within this
    same function (`0x880eada4`, `0x880eadc8`) - an
    over-approximation rather than the exact reacquire
    instruction, same pragmatic tradeoff as the `sv_wait_sig`
    bracket, since this function has multiple `splhi`/`splx`
    pairs across different sub-phases and isolating the exact
    one protecting the enqueue write specifically would need
    more exhaustive tracing than this check warrants.
-------------------------------------------------*/

static bool s_in_mutex_lock_enqueue = false;

void mips3_device::func_mutex_lock_enqueue_probe()
{
	s_in_mutex_lock_enqueue = true;
}

#ifdef MAME_DRC_DIAG
static void cfunc_mutex_lock_enqueue_probe(void *param) { ((mips3_device *)param)->func_mutex_lock_enqueue_probe(); }
#endif

void mips3_device::func_mutex_lock_enqueue_end_probe()
{
	s_in_mutex_lock_enqueue = false;
}

#ifdef MAME_DRC_DIAG
static void cfunc_mutex_lock_enqueue_end_probe(void *param) { ((mips3_device *)param)->func_mutex_lock_enqueue_end_probe(); }
#endif

void mips3_device::func_interrupt_taken_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t epc = (uint32_t)m_core->cpr[0][14];
	fprintf(stderr, "[MAME_DRC_DIAG] INTERRUPT_TAKEN #%llu epc=0x%08x in_iclogspace_critical=%d in_sv_wait_release_window=%d in_mutex_lock_enqueue=%d\n",
		(unsigned long long)s_count, epc, s_in_iclogspace_critical ? 1 : 0, s_in_sv_wait_release_window ? 1 : 0,
		s_in_mutex_lock_enqueue ? 1 : 0);
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_interrupt_taken_probe(void *param) { ((mips3_device *)param)->func_interrupt_taken_probe(); }
#endif

/*-------------------------------------------------
    cfunc_lockstate_probe_7 - TEMPORARY (2026-07-24,
    CORRECTED): DRC-side mutex_wake real-work counter,
    fires at the unconditional post-dispatch entry
    (0x880eaf10), mirroring the interpreter's corrected
    probe exactly so the two are directly comparable.
    See mips3.cpp's INTERP mutex_wake probe comment for
    the full story of the original a1-gating mistake
    this replaces.
-------------------------------------------------*/

void mips3_device::func_mutex_wake_entry_probe()
{
	static uint64_t s_count = 0;
	s_count++;
	const uint32_t obj = (uint32_t)m_core->r[16];
	const uint32_t f0 = m_program->read_dword(obj & 0x1fffffff);
	const uint32_t f4 = m_program->read_dword((obj + 4) & 0x1fffffff);
	// Stack window dump (2026-07-24): a single-slot read at sp+0x10
	// (assumed to be the prologue's "sd $ra,0x10($sp)" save) showed
	// saved_ra=0xFFFFFFFF for BOTH a healthy call and the crashing one,
	// ruling out "corruption specific to the bad call" - dump a wider
	// window (sp-0x10 through sp+0x30) instead of trusting one guessed
	// offset, to sanity-check the translation mechanism itself (are
	// nearby slots sensible-looking, or is everything garbage) and to
	// find where the real saved $ra (if any) actually lives.
	const uint32_t sp = (uint32_t)m_core->r[29];
	fprintf(stderr, "[MAME_DRC_DIAG] DRC mutex_wake #%llu obj=0x%08x count=0x%016llx [0]=0x%08x [4]=0x%08x sp=0x%08x stack:",
		(unsigned long long)s_count, obj, (unsigned long long)m_core->r[17], f0, f4, sp);
	for (int32_t off = -0x10; off <= 0x30; off += 4)
	{
		const uint32_t slot = sp + (uint32_t)off;
		const uint32_t tlbval = vtlb_table()[slot >> 12];
		if (tlbval & READ_ALLOWED)
		{
			const uint32_t val = m_program->read_dword((tlbval & ~0xfff) | (slot & 0xfff));
			fprintf(stderr, " [%+d]=%08x", off, val);
		}
		else
		{
			fprintf(stderr, " [%+d]=UNMAPPED", off);
		}
	}
	fprintf(stderr, "\n");
	fflush(stderr);
}

#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_7(void *param) { ((mips3_device *)param)->func_mutex_wake_entry_probe(); }
#endif
#ifdef MAME_DRC_DIAG
static void cfunc_lockstate_probe_6(void *param) { ((mips3_device *)param)->func_lockstate_probe(6); }
#endif

/*-------------------------------------------------
    cfunc_printf_exception - log any exceptions that
    aren't interrupts
-------------------------------------------------*/

void mips3_device::func_printf_exception()
{
	const uint32_t epc = (uint32_t)m_core->cpr[0][COP0_EPC];
	printf("Exception: EPC=%08X Cause=%08X BadVAddr=%08X Jmp=%08X\n", epc, (uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_BadVAddr], m_core->pc);
	fflush(stdout);
	// TEMPORARY diagnostic (2026-07-24): 0x880eaf34 is the exact EPC of the
	// last exception logged before the KERNEL FAULT/SEGV panic, every time
	// it's been reproduced tonight, across four different fix combinations.
	// The earlier attempt to catch this by sampling m_core->pc once per
	// execute_run() call in mips3.cpp never fired - the fault happens mid
	// -block inside a compiled DRC sequence, so the C++ dispatcher's own
	// per-call PC sample never actually lands exactly on it. This callback,
	// by contrast, fires exactly when the exception is generated (wired
	// into generate_exception's own codegen), so it's the right place to
	// catch it. Dump the raw instruction word (kseg0, direct physical-
	// masked read, no TLB translation needed) and all GPRs to identify the
	// exact base register and its live value at the moment of fault.
	if (epc == 0x880eaf34)
	{
		const uint32_t insn = m_program->read_dword(epc & 0x1fffffff);
		fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC insn=0x%08x\n", insn);
		// Interrupt-masking state at the exact moment of fault (2026-07-24):
		// tests directly whether this critical section (no bitlock found
		// protecting it via static analysis) was actually running with
		// interrupts disabled the way its lack of explicit locking would
		// require for correctness, or whether they were left enabled.
		{
			const uint32_t status = (uint32_t)m_core->cpr[0][COP0_Status];
			const uint32_t cause = (uint32_t)m_core->cpr[0][COP0_Cause];
			fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC status=0x%08x (IE=%u EXL=%u ERL=%u IM=0x%02x) cause=0x%08x (IP=0x%02x)\n",
				status, status & 1, (status >> 1) & 1, (status >> 2) & 1, (status >> 8) & 0xff,
				cause, (cause >> 8) & 0xff);
		}
		for (int i = 0; i < 32; i += 4)
		{
			fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC r%02d=0x%016llx r%02d=0x%016llx r%02d=0x%016llx r%02d=0x%016llx\n",
				i, (unsigned long long)m_core->r[i], i+1, (unsigned long long)m_core->r[i+1],
				i+2, (unsigned long long)m_core->r[i+2], i+3, (unsigned long long)m_core->r[i+3]);
		}
		// Window dump (2026-07-24): the faulting lw $a0,0x98($a2) at
		// 0x880eaf34 is confirmed innocent - $a2 is null when it shouldn't
		// be. Dump the raw words for 96 instructions before the fault (a
		// generous window to find the function entry/prologue and whatever
		// last touched $a2) plus 32 instructions around the caller's return
		// address ($ra=r31, in case $a2 is a caller-supplied argument never
		// set locally), for offline hand-disassembly.
		fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC window (callee, %08x-%08x):\n", epc - 0x180, epc);
		for (uint32_t addr = epc - 0x180; addr <= epc; addr += 4)
			fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC   %08x: %08x\n", addr, m_program->read_dword(addr & 0x1fffffff));
		const uint32_t ra = (uint32_t)m_core->r[31];
		fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC window (caller around ra=%08x, %08x-%08x):\n", ra, ra - 0x80, ra + 0x10);
		for (uint32_t addr = ra - 0x80; addr <= ra + 0x10; addr += 4)
			fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC   %08x: %08x\n", addr, m_program->read_dword(addr & 0x1fffffff));
		// Struct dump (2026-07-24): a pre-installed write-tap on the crash
		// address from one run turned out to watch the wrong location on
		// the next run - the object $s0 points to is dynamically allocated
		// at a run-dependent physical address (confirmed: 0x883d6728 vs
		// 0x88ca7a10 across two runs from an identical fresh disk), so a
		// static tap can't be targeted in advance. Instead, dump the raw
		// contents of the object $s0 (r16) points to at the moment of
		// fault, to see whether the null field at +4 looks like an
        // isolated dropped/corrupted write (surrounding fields populated,
		// non-zero, sane-looking) or the whole object reads as freshly-
		// zeroed/uninitialized (consistent with a genuine allocation-order
		// kernel bug rather than DRC-side corruption).
		const uint32_t s0 = (uint32_t)m_core->r[16];
		fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC struct dump (s0=%08x, %08x-%08x):\n", s0, s0 - 0x20, s0 + 0xc0);
		for (uint32_t addr = s0 - 0x20; addr <= s0 + 0xc0; addr += 4)
			fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC   +%04x %08x: %08x\n", addr - s0, addr, m_program->read_dword(addr & 0x1fffffff));
		// Original-caller trace (2026-07-24): 1267 direct callers of
		// mutex_unlock exist in this kernel - too many to trace statically.
		// The live $ra at crash time (r31) only reflects mutex_wake's own
		// internal call to its count-resolve helper (0x8801205c), not its
		// original caller, since that call happens after the prologue's
		// own "sd $ra,0x10($sp)" save and never gets restored before the
		// crash. Read the ORIGINAL saved $ra directly off the stack
		// instead - $sp (r29) is a kseg3 address here (TLB-mapped, same
		// translation as the kseg2 lock-probe used earlier tonight), so a
		// real vtlb lookup is needed, not a simple kseg0-style mask.
		{
			const uint32_t sp = (uint32_t)m_core->r[29];
			const uint32_t slot = sp + 0x10;
			const uint32_t tlbval = vtlb_table()[slot >> 12];
			if (tlbval & READ_ALLOWED)
			{
				const uint32_t phys = (tlbval & ~0xfff) | (slot & 0xfff);
				const uint32_t saved_ra = m_program->read_dword(phys);
				fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC caller: sp=%08x saved_ra=%08x call_site=%08x\n",
					sp, saved_ra, saved_ra - 8);
			}
			else
			{
				fprintf(stderr, "[MAME_DRC_DIAG] FAULT_PC caller: sp=%08x slot=%08x NOT READ_ALLOWED\n", sp, slot);
			}
		}
		fflush(stderr);
	}
	func_printf_probe();
}

static void cfunc_printf_exception(void *param)
{
	((mips3_device *)param)->func_printf_exception();
}

/*-------------------------------------------------
    cfunc_printf_debug - generic printf for
    debugging
-------------------------------------------------*/

void mips3_device::func_printf_debug()
{
	printf(m_core->format, m_core->arg0, m_core->arg1);
}

static void cfunc_printf_debug(void *param)
{
	((mips3_device *)param)->func_printf_debug();
}

#ifdef MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG
/*-------------------------------------------------
    cfunc_0fac5ffc_anomaly_diag - resumed
    2026-08-22 investigation into the still-
    unexplained validate_tlb() mismatch storm at
    pc=0x0fac5ffc (see wiki/performance.md). Runs an
    independent, C-side re-verification of the exact
    same comparison the JIT'd CMP is about to make,
    on every single dispatch through that site - not
    gated on the JIT compare's own NE flag, so this
    never depends on flags surviving a call. Also
    checks whether the compiled instance currently
    executing was baked under an older "generation"
    than the latest compile of this same pc, which
    would mean a stale instance is somehow still
    reachable and running.
-------------------------------------------------*/

void mips3_device::func_0fac5ffc_anomaly_diag()
{
	uint32_t const baked_gen = m_core->arg0;
	uint32_t const baked_expected = m_core->arg1;
	uint32_t const live_gen = m_core->diag_0fac5ffc_compile_gen;
	const vtlb_entry *tlbtable = vtlb_table();
	uint32_t const live_value = tlbtable[0x0fac5ffc >> 12];

	static uint64_t s_total = 0;
	static uint64_t s_value_mismatch = 0;
	static uint64_t s_gen_stale = 0;
	++s_total;
	bool const value_mismatch = (live_value != baked_expected);
	bool const gen_stale = (baked_gen != live_gen);
	if (value_mismatch)
		++s_value_mismatch;
	if (gen_stale)
		++s_gen_stale;

	if (value_mismatch || gen_stale || (s_total <= 20) || ((s_total % 50000) == 0))
	{
		fprintf(stderr,
			"[0FAC5FFC_DIAG] #%llu baked_gen=%u live_gen=%u baked_expected=0x%08x live_value=0x%08x "
			"totals(value_mismatch=%llu gen_stale=%llu)%s%s\n",
			(unsigned long long)s_total, baked_gen, live_gen, baked_expected, live_value,
			(unsigned long long)s_value_mismatch, (unsigned long long)s_gen_stale,
			value_mismatch ? " <-- VALUE_MISMATCH" : "",
			gen_stale ? " <-- STALE_GENERATION" : "");
		fflush(stderr);
	}
}

static void cfunc_0fac5ffc_anomaly_diag(void *param)
{
	((mips3_device *)param)->func_0fac5ffc_anomaly_diag();
}
#endif

#ifdef MAME_SGI_VALIDATE_TLB_MISMATCH_CLASS_DIAG
/*-------------------------------------------------
    cfunc_validate_tlb_mismatch_class_diag
    (2026-08-22) - generalizes the 0x0fac5ffc
    measurement above to every address using the
    shared-key validate_tlb() fast path. Classifies
    each mismatch as "live_value==0" (transiently
    unmapped - what a targeted refresh-not-recompile
    fix would catch, without discarding still-valid
    compiled code) vs. any other nonzero-but-
    different value (a genuine remap - still needs a
    real recompile even with that fix in place).
-------------------------------------------------*/

void mips3_device::func_validate_tlb_mismatch_class_diag()
{
	uint32_t const pc = m_core->arg0;
	uint32_t const baked_expected = m_core->arg1;
	const vtlb_entry *tlbtable = vtlb_table();
	uint32_t const live_value = tlbtable[pc >> 12];

	static uint64_t s_total = 0;
	static uint64_t s_mismatch_total = 0;
	static uint64_t s_mismatch_zero = 0;
	static uint64_t s_mismatch_nonzero = 0;
	++s_total;
	bool const mismatch = (live_value != baked_expected);
	if (mismatch)
	{
		++s_mismatch_total;
		if (live_value == 0)
			++s_mismatch_zero;
		else
			++s_mismatch_nonzero;
	}

	// Causation check (2026-08-22) - does a zero-mismatch on this one
	// well-instrumented address actually correlate with a real
	// subsequent recompile (diag_0fac5ffc_compile_gen incrementing), or
	// does the guest's own TLB-refill-then-eret sequence typically
	// resolve this without ever reaching code_compile_block() again?
	// Answers this before writing the actual fix, given this code's
	// history of two livelocks from unverified assumptions about what
	// happens downstream of a guard failure.
	if ((pc == 0x0fac5ffc) && mismatch && (live_value == 0))
	{
		static uint32_t s_gen_at_last_zero_mismatch = 0xffffffff;
		static uint64_t s_zero_mismatch_events = 0;
		static uint64_t s_gen_advanced_between = 0;
		uint32_t const current_gen = m_core->diag_0fac5ffc_compile_gen;
		++s_zero_mismatch_events;
		if ((s_gen_at_last_zero_mismatch != 0xffffffff) && (current_gen != s_gen_at_last_zero_mismatch))
			++s_gen_advanced_between;
		if ((s_zero_mismatch_events <= 20) || ((s_zero_mismatch_events % 5000) == 0))
		{
			fprintf(stderr,
				"[ZERO_MISMATCH_CAUSATION] event=%llu compile_gen=%u (prev=%u) gen_advanced_since_last=%llu/%llu\n",
				(unsigned long long)s_zero_mismatch_events, current_gen, s_gen_at_last_zero_mismatch,
				(unsigned long long)s_gen_advanced_between, (unsigned long long)s_zero_mismatch_events);
			fflush(stderr);
		}
		s_gen_at_last_zero_mismatch = current_gen;
	}

	static uint64_t s_last_reported = 0;
	bool const interesting_nonzero_mismatch = mismatch && (live_value != 0) && (s_mismatch_nonzero <= 30);
	if ((s_total <= 20) || (s_total >= s_last_reported + 100000) || interesting_nonzero_mismatch)
	{
		fprintf(stderr,
			"[TLB_MISMATCH_CLASS] total=%llu mismatch_total=%llu (zero=%llu nonzero=%llu) "
			"pc=0x%08x baked=0x%08x live=0x%08x%s\n",
			(unsigned long long)s_total, (unsigned long long)s_mismatch_total,
			(unsigned long long)s_mismatch_zero, (unsigned long long)s_mismatch_nonzero,
			pc, baked_expected, live_value,
			interesting_nonzero_mismatch ? " <-- NONZERO_MISMATCH" : "");
		fflush(stderr);
		s_last_reported = s_total;
	}
}

static void cfunc_validate_tlb_mismatch_class_diag(void *param)
{
	((mips3_device *)param)->func_validate_tlb_mismatch_class_diag();
}

/*-------------------------------------------------
    cfunc_nocode_0fac5ffc_diag (2026-08-22) - see
    the call site's own comment in
    static_generate_nocode_handler. Fires whenever a
    HASHJMP anywhere misses specifically for
    pc=0x0fac5ffc, reporting the live dispatch-key
    state at that exact moment - answers whether
    generate_select_dispatch_key()'s computed key is
    actually disagreeing with registration for real,
    live executions, despite matching on static
    inspection everywhere it's called.
-------------------------------------------------*/

void mips3_device::func_nocode_0fac5ffc_diag()
{
	bool const writable_now = dispatch_page_writable(0x0fac5ffc);
	uint32_t const would_be_key = writable_now ? m_core->dispatch_mode : m_core->mode;
	const vtlb_entry *tlbtable = vtlb_table();
	uint32_t const live_value = tlbtable[0x0fac5ffc >> 12];

	static uint64_t s_total = 0;
	static uint32_t s_last_flush_count = 0xffffffff;
	uint32_t const current_flush_count = m_drcuml->cache().flush_count();
	bool const flush_since_last = (s_last_flush_count != 0xffffffff) && (current_flush_count != s_last_flush_count);
	++s_total;
	if ((s_total <= 30) || flush_since_last)
	{
		fprintf(stderr,
			"[NOCODE_0FAC5FFC] event=%llu mode=%u dispatch_mode=%u writable_now=%d would_be_key=%u "
			"hash_exists(would_be_key)=%d live_value=0x%08x flush_count=%u (prev=%u)%s\n",
			(unsigned long long)s_total, m_core->mode, m_core->dispatch_mode, writable_now, would_be_key,
			m_drcuml->hash_exists(would_be_key, 0x0fac5ffc), live_value, current_flush_count, s_last_flush_count,
			flush_since_last ? " <-- FLUSH_HAPPENED" : "");
		fflush(stderr);
	}
	s_last_flush_count = current_flush_count;
}

static void cfunc_nocode_0fac5ffc_diag(void *param)
{
	((mips3_device *)param)->func_nocode_0fac5ffc_diag();
}
#endif

#ifdef MAME_SGI_TASK84_TLB_MISMATCH_DIAG
/*-------------------------------------------------
    cfunc_task84_tlb_mismatch_diag (2026-09-03) -
    see MAME_SGI_TASK84_TLB_MISMATCH_DIAG's own
    comment above. Fires from two call sites in
    generate_sequence_instruction() (the
    compiler_page_fault() site, unconditional at
    that site's own runtime, and the validate_tlb()
    fast-path site, conditional on the live/baked
    vtlb entry mismatch that triggers UML_EXHc) -
    m_core->arg0/arg1 carry the compile-time-baked
    pc and site number, following the same pattern
    as func_0fac5ffc_anomaly_diag.
-------------------------------------------------*/

void mips3_device::func_task84_tlb_mismatch_diag()
{
	uint32_t const compile_pc = m_core->arg0;
	uint32_t const site = m_core->arg1;
	static uint64_t s_total = 0;
	++s_total;
	fprintf(stderr,
		"[TASK84_TLB_MISMATCH] #%llu site=%u compile_pc=%08x live_pc=%08x mode=%u dispatch_mode=%u r19=%lu\n",
		(unsigned long long)s_total, site, compile_pc, m_core->pc, m_core->mode, m_core->dispatch_mode, m_core->r[19]);
	fflush(stderr);
}

static void cfunc_task84_tlb_mismatch_diag(void *param)
{
	((mips3_device *)param)->func_task84_tlb_mismatch_diag();
}
#endif

#ifdef MAME_SGI_TASK84_V0_S3_DIAG
/*-------------------------------------------------
    cfunc_task84_v0_diag / cfunc_task84_s3_check_diag
    (2026-09-04) - see MAME_SGI_TASK84_V0_S3_DIAG's
    own comment above. Both callers force a
    save_fast_iregs() sync immediately beforehand,
    so m_core->r[] holds live values, not stale
    cached ones, at the moment of the read.
-------------------------------------------------*/

void mips3_device::func_task84_v0_diag()
{
	static uint64_t s_total = 0;
	++s_total;
	// Dedicated file, not stderr - a shared stdout/stderr fd (as used by
	// tools/boot_test.sh's launch redirection) mixes this unbuffered
	// output with MAME's own fully-buffered stdout in a way that
	// produced inconsistent, even shrinking, grep counts on the merged
	// log. This mirrors the already-proven MAME_SGI_LWC1_BLOCK_ADDR_PROBE
	// pattern (drcbex64.cpp) of a dedicated fopen/fclose per call.
	FILE *f = fopen("/home/blythe/mame-sgi/task84_v0_s3.log", "a");
	if (f)
	{
		fprintf(f, "[TASK84_V0] #%llu v0=%08x r19=%lu\n",
			(unsigned long long)s_total, (uint32_t)m_core->r[2], m_core->r[19]);
		fclose(f);
	}
}

static void cfunc_task84_v0_diag(void *param)
{
	((mips3_device *)param)->func_task84_v0_diag();
}

void mips3_device::func_task84_s3_check_diag()
{
	static uint64_t s_total = 0;
	++s_total;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_v0_s3.log", "a");
	if (!f)
		return;
	fprintf(f, "[TASK84_S3_CHECK] #%llu r19=%lu\n",
		(unsigned long long)s_total, m_core->r[19]);
	fclose(f);
}

static void cfunc_task84_s3_check_diag(void *param)
{
	((mips3_device *)param)->func_task84_s3_check_diag();
}
#endif

#ifdef MAME_SGI_TASK84_WINDOW_TRACE
/*-------------------------------------------------
    cfunc_task84_window_trace_save / _reload
    (2026-09-04) - see MAME_SGI_TASK84_WINDOW_TRACE's
    own comment above. Deliberately does NOT call
    save_fast_iregs() anywhere in its own probe
    points (0x1007df50/0x1007dffc just toggle
    diag_task84_armed via a plain inert store) -
    this diagnostic exists to observe the NATURAL
    bug, and any save_fast_iregs() call at those two
    PCs is now known to suppress it outright.

    REVISED (2026-09-04, same day): the first cut of this logged every
    single gated save/reload unconditionally - 20 loop iterations produced
    2.3M+ events and a 367MB log file (the "armed" window turned out to
    stay open far longer than intended: 0x1007df50 re-arms on EVERY token
    the inner loop processes, but 0x1007dffc - the disarm point - is only
    reached when $s3 finally hits zero, so the window spans most of a
    single higher-level call, not a tight ~20-instruction span as
    designed). Disk was at 98% full; the log was deleted immediately.
    Rewritten to log ONLY an actual divergence - a reload whose r[2]
    doesn't match the most recently observed save's r[2] - which is the
    only thing this diagnostic actually needs to catch. This should
    collapse output from millions of lines to (ideally) a small, directly
    actionable handful, while still covering every save/reload pair that
    occurs anywhere inside the armed window.
-------------------------------------------------*/

namespace {
uint32_t g_task84_last_save_r2 = 0;
uint32_t g_task84_last_save_site = 0;
uint32_t g_task84_last_save_pc = 0;
bool g_task84_have_baseline = false;
uint64_t g_task84_save_count = 0;
uint64_t g_task84_reload_count = 0;
}

void mips3_device::func_task84_window_trace_save()
{
	++g_task84_save_count;
	g_task84_last_save_r2 = (uint32_t)m_core->r[2];
	g_task84_last_save_site = m_core->diag_task84_trace_site;
	g_task84_last_save_pc = m_core->pc;
	g_task84_have_baseline = true;
	// cheap heartbeat so a totally-silent log doesn't get mistaken for
	// "the gating never fires" - not per-event, just a coarse liveness check
	if ((g_task84_save_count % 500000) == 1)
	{
		FILE *f = fopen("/home/blythe/mame-sgi/task84_window_trace.log", "a");
		if (f)
		{
			fprintf(f, "[HEARTBEAT] saves=%llu reloads=%llu last_save_site=%u last_save_pc=%08x last_save_r2=%08x\n",
				(unsigned long long)g_task84_save_count, (unsigned long long)g_task84_reload_count,
				g_task84_last_save_site, g_task84_last_save_pc, g_task84_last_save_r2);
			fclose(f);
		}
	}
}

static void cfunc_task84_window_trace_save(void *param)
{
	((mips3_device *)param)->func_task84_window_trace_save();
}

void mips3_device::func_task84_window_trace_reload()
{
	++g_task84_reload_count;
	uint32_t cur_r2 = (uint32_t)m_core->r[2];
	if (g_task84_have_baseline && cur_r2 != g_task84_last_save_r2)
	{
		static uint64_t s_anomaly_count = 0;
		++s_anomaly_count;
		FILE *f = fopen("/home/blythe/mame-sgi/task84_window_trace.log", "a");
		if (f)
		{
			fprintf(f, "[ANOMALY] #%llu reload_pc=%08x reload_r2=%08x last_save_site=%u last_save_pc=%08x last_save_r2=%08x saves_so_far=%llu reloads_so_far=%llu\n",
				(unsigned long long)s_anomaly_count, m_core->pc, cur_r2,
				g_task84_last_save_site, g_task84_last_save_pc, g_task84_last_save_r2,
				(unsigned long long)g_task84_save_count, (unsigned long long)g_task84_reload_count);
			fclose(f);
		}
		// don't re-flag the same persisted mismatch on every subsequent
		// reload - only flag NEW divergences from here on
		g_task84_last_save_r2 = cur_r2;
	}
}

static void cfunc_task84_window_trace_reload(void *param)
{
	((mips3_device *)param)->func_task84_window_trace_reload();
}

/*-------------------------------------------------
    cfunc_task84_entry_pc_diag (2026-09-04, widened
    2026-09-07 per the third Opus review's §7.1,
    "do this first") - the refinement per the user's
    own question about the armed-window trace's
    negative result: arming at 0x1007df50 is too late
    to see a reload that happens on the way INTO
    0x1007df50 itself (the block-entry
    load_fast_iregs() for a fresh compile of this
    exact PC runs before any of that block's own
    generated instructions, including our own
    arm-write, execute). This logs UNCONDITIONALLY
    (no armed-flag dependency) whenever a reentry's
    own target pc falls in the widened window
    (0x10076000-0x1007e100, covering cpp_get_token and
    parse_answer - see the widened gate at this
    function's UML_CMP call site), showing r[2] and,
    as of this widening, enough additional state to
    close the two biggest open caveats the review
    flagged against the original narrow version:
      - ASID/dispatch_mode: eliminates "a different
        guest process happened to have code at this
        VA" as an alternative explanation for any
        anomalous r[2] value.
      - diag_task84_trace_site (now written
        UNCONDITIONALLY by all 4 real UML_EXIT sites,
        not just inside the armed window - see
        static_generate_nocode_handler's comment):
        identifies WHICH exit produced this re-entry
        (1=nocode, 2=out_of_cycles, 3=tlb_mismatch,
        4=exception), turning "an anomaly happened
        somewhere" into "an anomaly happened at exit
        site N".
      - r[16]($s0)/r[18]($s2)/r[31]($ra): the guest
        registers most relevant to cpp_get_token's own
        control flow and its caller's use of the
        result, for cross-checking any anomalous r[2]
        against the surrounding register state rather
        than r[2] in isolation.
    A value r2 < 0x10000000 cannot be a valid pointer
    into cc1's data/text (both are mapped at/above
    0x10000000 in this USEG layout - see the
    "IMPOSSIBLE_PTR" tag below) and is flagged
    inline so the log is self-describing without
    needing to re-derive that threshold by hand.
-------------------------------------------------*/

void mips3_device::func_task84_entry_pc_diag()
{
	static uint64_t s_total = 0;
	++s_total;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_window_trace.log", "a");
	if (!f)
		return;
	const uint32_t r2 = (uint32_t)m_core->r[2];
	const uint32_t asid = (uint32_t)(m_core->cpr[0][COP0_EntryHi] & 0xff);
	fprintf(f, "[ENTRY_PC_WIDENED] #%llu pc=%08x r2=%08x%s asid=%02x dispatch_mode=%08x "
		"exit_site=%u last_saved_r2=%016llx s0=%08x s2=%08x ra=%08x\n",
		(unsigned long long)s_total, m_core->pc, r2,
		(r2 < 0x10000000) ? " IMPOSSIBLE_PTR" : "",
		asid, m_core->dispatch_mode, m_core->diag_task84_trace_site,
		(unsigned long long)m_core->diag_task84_last_saved_r2,
		(uint32_t)m_core->r[16], (uint32_t)m_core->r[18], (uint32_t)m_core->r[31]);
	fclose(f);
}

static void cfunc_task84_entry_pc_diag(void *param)
{
	((mips3_device *)param)->func_task84_entry_pc_diag();
}

#ifdef MAME_SGI_TASK84_V0_SHADOW_DIAG
/*-------------------------------------------------
    cfunc_v0_shadow_diag (2026-09-07, third Opus
    review's §7.3) - fires on every genuine
    divergence between the live $v0 and the shadow
    maintained across every guest instruction
    boundary (detection is intentionally global -
    see MAME_SGI_TASK84_V0_SHADOW_DIAG's own comment
    for why a scoped detector would be unsound), but
    only LOGS ones in cc1's own user-space text
    (0x10000000-0x7fffffff USEG, well below KSEG0's
    0x80000000). Added 2026-09-07 after the
    label-collision fix (see that same comment) still
    produces a wall of ordinary kernel-space $v0
    scratch-register churn (KSEG0 pc, ASID/dispatch_mode
    constant across many hits, values consistent with
    routine interrupt/dispatch bookkeeping, not
    corruption) - the underlying shadow mechanism stays
    fully global and correct (required for soundness),
    this filter only decides what's worth writing to a
    log a human will actually read. If cc1-space
    divergences ever turn out to also need excluding by
    ASID (another process happening to share this VA
    range), that's a one-line addition here, not a
    codegen change.
    Self-capping like the other task84 log functions -
    a real hit should be rare, and this project has
    been burned once already by an unbounded log
    (see MAME_SGI_TASK84_WINDOW_TRACE's own comment).
-------------------------------------------------*/

void mips3_device::func_v0_shadow_diag()
{
	if (m_core->diag_v0_divergence_pc < 0x10000000 || m_core->diag_v0_divergence_pc >= 0x80000000)
		return;
	static uint64_t s_hits = 0;
	++s_hits;
	if (s_hits > 500)
		return;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_window_trace.log", "a");
	if (!f)
		return;
	fprintf(f, "[V0_SHADOW_DIVERGENCE] #%llu pc=%08x shadow_was=%016llx now=%016llx asid=%02x dispatch_mode=%08x\n",
		(unsigned long long)s_hits, m_core->diag_v0_divergence_pc,
		(unsigned long long)m_core->diag_v0_shadow,
		(unsigned long long)m_core->diag_v0_divergence_current,
		(uint32_t)(m_core->cpr[0][COP0_EntryHi] & 0xff), m_core->dispatch_mode);
	fclose(f);
}

static void cfunc_v0_shadow_diag(void *param)
{
	((mips3_device *)param)->func_v0_shadow_diag();
}
#endif

#ifdef MAME_SGI_TASK84_T9_GOT_DIAG
/*-------------------------------------------------
    cfunc_t9_got_diag (2026-09-09) - see the macro's
    own comment. Fires only when [jmpdest] (the live
    value read from cpp_get_token's GOT slot,
    immediately before the jalr at 0x1007df48 uses it
    as the hashjmp target) is NOT 0x10076008 - the one
    value this call site should ever legitimately
    produce.
-------------------------------------------------*/

void mips3_device::func_t9_got_diag()
{
	static uint64_t s_hits = 0;
	++s_hits;
	if (s_hits > 500)
		return;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_t9_got.log", "a");
	if (!f)
		return;
	fprintf(f, "[T9_GOT_MISMATCH] #%llu jmpdest=%08x asid=%02x dispatch_mode=%08x\n",
		(unsigned long long)s_hits, m_core->jmpdest,
		(uint32_t)(m_core->cpr[0][COP0_EntryHi] & 0xff), m_core->dispatch_mode);
	fclose(f);
}

static void cfunc_t9_got_diag(void *param)
{
	((mips3_device *)param)->func_t9_got_diag();
}
#endif

#ifdef MAME_SGI_TASK84_CALL_TRACE_DIAG
/*-------------------------------------------------
    cfunc_call_trace_diag (2026-09-09) - see the
    macro's own comment. Logs every JAL/JALR call
    target in real execution order (not compile
    order), self-capped generously since this is a
    genuine per-execution runtime cost.
-------------------------------------------------*/

void mips3_device::func_call_trace_diag()
{
	// Kept open for the process lifetime rather than fopen/fclose per hit
	// (the pattern most other task84 diagnostics use, fine for their
	// <=500-hit caps but far too slow here - this fires per CALL
	// EXECUTED, not per rare mismatch/divergence, plausibly tens of
	// thousands of times even for one trivial cc1-on-/dev/null run).
	static uint64_t s_hits = 0;
	++s_hits;
	if (s_hits > 3000000)
		return;
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_call_trace.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%08x\n", m_core->diag_call_trace_target);
	// Flush far more often than the old 4096-call threshold (2026-09-10,
	// same root-cause class as MAME_SGI_TASK84_TOKEN_RESULT_DIAG's fix -
	// see wiki/task84.md 2026-09-10). This diagnostic fires ~163,000 times
	// per cc1 invocation, so 4096 never looked dangerous the way
	// token_result's ~17-64-per-run did - but every cross-cycle
	// comparison this investigation has drawn from call_trace's TOTAL
	// count (the ~163385 vs ~162930 "signature clusters", the PREDICATE-
	// vs-OK call-count-shorter-by-~450-900 claim) differs by an amount
	// SMALLER than the old 4096-line throttle window, meaning any of it
	// could have been entirely a boundary-flush artifact rather than a
	// real fact about cc1's execution. Flushing every 64 calls bounds the
	// worst-case cross-cycle misattribution to 63 lines - negligible
	// against the ~450+ differences those claims depend on. Every
	// total-call-count claim from before this fix needs re-verification;
	// do not treat old numbers as trustworthy until re-measured.
	if ((s_hits & 0x3f) == 0)
		fflush(s_f);
}

static void cfunc_call_trace_diag(void *param)
{
	((mips3_device *)param)->func_call_trace_diag();
}
#endif

#ifdef MAME_SGI_TASK84_CPPERROR_CALLSITE_DIAG
/*-------------------------------------------------
    func_cpperror_callsite_diag (2026-09-10) - see the
    macro's own comment. Logs the call site (desc->pc,
    the jalr instruction's own address) every time
    cpp_error is actually invoked, tagged with the real
    PID (read_current_pid(), validated 2026-09-10) so
    hits can be filtered to genuine cc1 invocations.
-------------------------------------------------*/

void mips3_device::func_cpperror_callsite_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 500)
		return;
	uint32_t const callsite_pc = m_core->diag_cpperror_callsite_pc;
	uint64_t const a2 = m_core->diag_cpperror_a2;
	uint64_t const s3 = m_core->diag_cpperror_s3;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_cpperror_callsite.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu callsite=%08x a2=%016llx s3=%016llx pid=%u\n",
		(unsigned long long)s_seq, callsite_pc, (unsigned long long)a2, (unsigned long long)s3, pid);
	fflush(s_f);
#ifdef MAME_SGI_TASK84_DRC_EVENT_LOG
	// Piggyback onto the unified, already-chronologically-ordered DRC
	// event log (2026-09-10) - this is the ONLY way to answer "before or
	// after the failure point" for any other event in that log (e.g.
	// EXC_SYSCALL): this file and task84_drc_events.log are separate,
	// independently-flushed streams with no shared clock, so a call site
	// hit recorded ONLY here can't be ordered against events recorded
	// only there. No new UML instrumentation - this reuses the same
	// already-firing, already-validated callsite hook.
	log_drc_event("CPPERROR_CALL", callsite_pc);
#endif
}

static void cfunc_cpperror_callsite_diag(void *param)
{
	((mips3_device *)param)->func_cpperror_callsite_diag();
}
#endif

#ifdef MAME_SGI_TASK84_CPPERROR_COND_TRACE_DIAG
/*-------------------------------------------------
    func_cpperror_cond_trace_diag (2026-09-10) - see
    the macro's own comment. Fires on every execution
    of the first `beq` at 0x1007de98, logging $v1 (token
    type byte) and $s1 so an OK run's sequence at this
    exact call site can be diffed directly against a
    PREDICATE run's. NOTE: $v0 itself is NOT captured -
    by this point in program order it's already been
    overwritten with the constant 0x16 (0x1007de94's
    addiu), not the original token pointer from the jalr
    eight instructions earlier.
-------------------------------------------------*/

void mips3_device::func_cpperror_cond_trace_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 5000)
		return;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_cpperror_cond_trace.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu v1=%02x s1=%016llx pid=%u\n",
		(unsigned long long)s_seq, (unsigned)(m_core->diag_cond_v1 & 0xff),
		(unsigned long long)m_core->diag_cond_s1, pid);
	fflush(s_f);
}

static void cfunc_cpperror_cond_trace_diag(void *param)
{
	((mips3_device *)param)->func_cpperror_cond_trace_diag();
}
#endif

#ifdef MAME_SGI_TASK84_SOURCE_TEXT_DIAG
/*-------------------------------------------------
    func_source_text_diag (2026-09-10) - see the
    macro's own comment. Dumps the actual raw source
    bytes at cpp_reader's current buffer position, at
    the loop's own repeated cpp_get_token call site,
    every iteration, both outcomes.
-------------------------------------------------*/

void mips3_device::func_source_text_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 2000)
		return;
	uint32_t const pfile = (uint32_t)(m_core->diag_srctext_s0 & 0xffffffff);
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_source_text.log", "a");
	if (!s_f)
		return;
	// pfile (cpp_reader*) is cc1's OWN userspace struct - KUSEG, i.e.
	// BELOW 0x80000000 (confirmed live: 0x104810d0, plausible cc1 heap) -
	// NOT a kernel pointer like read_current_pid()'s chain. Fixed after
	// an initial version wrongly reused that chain's >=0x80000000 check
	// here and silently skipped every real hit ("implausible, skipping"
	// for all 8/8 samples in the first test run) - caught by actually
	// looking at the output before trusting it, not after.
	if (pfile == 0 || pfile == 0xffffffff)
	{
		fprintf(s_f, "%llu pfile=%08x (implausible, skipping) pid=%u\n",
			(unsigned long long)s_seq, pfile, pid);
		fflush(s_f);
		return;
	}
	uint32_t const buffer_ptr = read_guest_dword_translated(pfile + 0);
	uint32_t const cur = (buffer_ptr != 0 && buffer_ptr != 0xffffffff)
		? read_guest_dword_translated(buffer_ptr + 0) : 0;
	char text[41];
	text[40] = '\0';
	if (cur != 0 && cur != 0xffffffff)
	{
		// Dump 8 bytes before and 32 after `cur` (the next byte cpp_get_token
		// is about to lex) for context - printable ASCII only, non-printable
		// shown as '.'. Uses the proven-correct translate()-first read
		// (read_guest_dword_translated's own pattern, byte-sized here) -
		// NOT the raw m_program->read_byte() this file's earlier
		// diagnostics used for $v0+6 token-type reads, which may only
		// have "worked" by address-range coincidence rather than genuine
		// translation (see this session's own KSEG2/3 translation-bug
		// finding, wiki/task84.md 2026-09-10) - not worth the same risk
		// here.
		for (int i = 0; i < 40; i++)
		{
			offs_t addr = (offs_t)(cur - 8 + i);
			address_space *tspace = nullptr;
			uint8_t b = 0;
			if (translate(AS_PROGRAM, device_memory_interface::TR_READ, addr, tspace) && tspace)
				b = tspace->read_byte(addr);
			text[i] = (b >= 0x20 && b < 0x7f) ? (char)b : '.';
		}
	}
	else
	{
		for (int i = 0; i < 40; i++)
			text[i] = '?';
	}
	fprintf(s_f, "%llu s3=%llu buffer_ptr=%08x cur=%08x text=[%s] pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)m_core->diag_srctext_s3,
		buffer_ptr, cur, text, pid);
	fflush(s_f);
}

static void cfunc_source_text_diag(void *param)
{
	((mips3_device *)param)->func_source_text_diag();
}
#endif

#ifdef MAME_SGI_TASK84_BEQL_S3_DIAG
/*-------------------------------------------------
    func_beql_s3_diag (2026-09-10) - see the macro's
    own comment. Fires on EVERY execution of the
    0x1007DFFC beql, both outcomes - the decisive
    read of $s3 right at the actual comparison,
    resolving whether the call-site's earlier "0 then
    1" progression (SOURCE_TEXT_DIAG's diag_srctext_s3)
    genuinely contradicts a real failure or was always
    consistent with one.
-------------------------------------------------*/

void mips3_device::func_beql_s3_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 500000)
		return;
	uint64_t const s3 = m_core->diag_beql_s3;
	uint64_t const v0 = m_core->diag_beql_v0;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_beql_s3.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu s3=%016llx v0=%016llx pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)s3, (unsigned long long)v0, pid);
	fflush(s_f);
}

static void cfunc_beql_s3_diag(void *param)
{
	((mips3_device *)param)->func_beql_s3_diag();
}

/*-------------------------------------------------
    func_breakcheck_s3_diag / func_s3_increment_diag
    (2026-09-10) - see MAME_SGI_TASK84_BEQL_S3_DIAG's
    own comment. Direct empirical count of real loop
    iterations: how many times the break-check actually
    finds close-paren (should be exactly once per
    predicate, at s3's own value at that moment), and
    how many times the increment fires (real iteration
    count), for a confirmed-failing invocation.
-------------------------------------------------*/

void mips3_device::func_breakcheck_s3_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 500000)
		return;
	uint64_t const v0 = m_core->diag_breakcheck_v0;
	uint64_t const s3 = m_core->diag_breakcheck_s3;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_breakcheck_s3.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu v0=%016llx s3=%016llx pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)v0, (unsigned long long)s3, pid);
	fflush(s_f);
}

static void cfunc_breakcheck_s3_diag(void *param)
{
	((mips3_device *)param)->func_breakcheck_s3_diag();
}

void mips3_device::func_s3_increment_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 500000)
		return;
	uint64_t const newval = m_core->diag_s3_increment_newval;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_s3_increment.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu newval=%016llx pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)newval, pid);
	fflush(s_f);
}

static void cfunc_s3_increment_diag(void *param)
{
	((mips3_device *)param)->func_s3_increment_diag();
}
#endif

#ifdef MAME_SGI_TASK84_TLBMISS_REDISPATCH_DIAG
/*-------------------------------------------------
    func_tlbmiss_redispatch_diag (2026-09-10) - see
    the macro's own comment inside
    static_generate_tlb_mismatch(). Fires only when
    that SHARED trampoline is about to redispatch back
    to 0x1007DFFC's own PC (i.e. the exception's
    recovered-and-rounded-down PC equals the beql
    itself) - the direct test of the user's delay-slot/
    page-crossing hypothesis: does a TLB miss for the
    delay slot's own page (0x1007E000, the first byte
    of a fresh page relative to the beql) ever fire
    mid-taken-branch, forcing a full re-execution of the
    beql from scratch, and if so what is $s3 at that
    exact moment (should still be legitimately 0 for a
    correct re-run to also be correct).
-------------------------------------------------*/

void mips3_device::func_tlbmiss_redispatch_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 500000)
		return;
	uint64_t const s3 = m_core->diag_tlbmiss_redispatch_s3;
	uint32_t const firstpc = m_core->diag_tlbmiss_redispatch_firstpc;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_tlbmiss_redispatch.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu s3=%016llx firstpc=%08x pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)s3, firstpc, pid);
	fflush(s_f);
}

static void cfunc_tlbmiss_redispatch_diag(void *param)
{
	((mips3_device *)param)->func_tlbmiss_redispatch_diag();
}

void mips3_device::func_tlbmiss_redispatch_e020_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 500000)
		return;
	uint64_t const s3 = m_core->diag_tlbmiss_redispatch_s3;
	uint32_t const firstpc = m_core->diag_tlbmiss_redispatch_firstpc;
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_tlbmiss_redispatch_e020.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu s3=%016llx firstpc=%08x pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)s3, firstpc, pid);
	fflush(s_f);
}

static void cfunc_tlbmiss_redispatch_e020_diag(void *param)
{
	((mips3_device *)param)->func_tlbmiss_redispatch_e020_diag();
}
#endif

#ifdef MAME_SGI_TASK84_TOKEN_RESULT_DIAG
/*-------------------------------------------------
    cfunc_token_result_diag (2026-09-09) - see the
    macro's own comment. Fires at each of
    cpp_get_token's three real return points; reads
    the token-type byte at [v0+6] the same way every
    real caller does (`lbu $v0,6($v0)`), via
    m_program->read_byte() - the same accessor this
    file already uses elsewhere for direct C++-side
    guest memory reads (e.g. the kstack flag read in
    generate_special()).
-------------------------------------------------*/

void mips3_device::func_token_result_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 3000000)
		return;
	uint64_t const v0 = m_core->diag_token_result_v0;
	uint8_t type_byte = 0xff;
	if (v0 != 0 && v0 < 0x100000000ULL)
		type_byte = m_program->read_byte((offs_t)(v0 + 6));
	// ASID/mode capture (2026-09-10) - testing the cross-process dispatch-
	// key-aliasing hypothesis: cpp_get_token's page is read-only, so per
	// MAME_SGI_DISPATCH_MODE_ASID_FIX==2 it always registers/dispatches
	// under the SHARED, ASID-independent key (m_core->mode), never the
	// private m_core->dispatch_mode. If some OTHER process/context also
	// reaches this exact virtual PC and hashes to the same shared key, its
	// dispatch lands on this same compiled block (and this same injected
	// hook) even though it isn't a real cpp_get_token return. Logging the
	// live ASID (COP0 EntryHi low byte, same accessor pattern as
	// V0_SHADOW_DIVERGENCE/T9_GOT_MISMATCH) and is_user lets us check
	// whether the 131-vs-17 excess hits cluster under a DIFFERENT asid
	// than cc1's own.
	uint32_t const asid = (uint32_t)(m_core->cpr[0][COP0_EntryHi] & 0xff);
	uint32_t const mode = m_core->mode;
	uint32_t const dispatch_mode = m_core->dispatch_mode;
	// $sp (r29) distinguishes "same cc1 process, ASID rotated mid-run" (sp
	// stable across ASID-labeled groups) from "genuinely different
	// processes sharing this compiled block" (sp would differ per group -
	// each process's stack lives at its own address). CONFIRMED same-
	// process (2026-09-10): sp takes only 3 values, identically
	// distributed, across every ASID-labeled group AND across both OK and
	// PREDICATE firing cycles - ASID is NOT process identity here.
	uint64_t const sp = m_core->r[29];
	// $ra (r31) is this JR's own dynamic jump target - the single most
	// direct signal available: a genuine return goes back to right after
	// one of the two known call sites (parse_answer's own call, or the
	// second caller at 0x1007b650). If instead $ra repeats the same value
	// across multiple consecutive hits with no intervening real call
	// (per CALL_TRACE_DIAG, which is NOT inflated the way this hook is -
	// see the macro's own comment), that's direct evidence of a
	// duplicate/self-referential dispatch landing on this PC again
	// without a fresh logical call, rather than 3x as many real calls.
	uint64_t const ra = m_core->r[31];
	// Real per-process PID, via the validated curprocp chain
	// (read_current_pid(), 2026-09-10) - the actual fix for the
	// question $sp/ASID could only answer indirectly: is this hit
	// really cc1's own, or does it belong to some other process
	// entirely? Cross-referencing this against the test harness's
	// own captured `cc1pid=$!` for the same invocation directly
	// answers the 17-vs-39 question instead of inferring it.
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_token_result.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu v0=%016llx type=%02x asid=%02x mode=%u dispatch_mode=%u sp=%016llx ra=%016llx pid=%u\n",
		(unsigned long long)s_seq, (unsigned long long)v0, (unsigned)type_byte,
		asid, mode, dispatch_mode, (unsigned long long)sp, (unsigned long long)ra, pid);
	// ALWAYS flush (2026-09-10, root-cause fix) - this hook only fires
	// ~17-64 times per cc1 invocation, far short of the 4096-call
	// threshold the old `(s_seq & 0xfff) == 0` throttle needed to ever
	// trigger. Writes sat in libc's stdio buffer and were flushed to the
	// visible file only whenever that buffer happened to fill on its own
	// (unrelated to any test cycle's boundary), which is what caused the
	// entire 17-vs-131/39/64/48 "discrepancy" this investigation chased
	// for two sessions - see wiki/task84.md 2026-09-10. A single write
	// here is cheap; don't reintroduce the throttle.
	fflush(s_f);
}

static void cfunc_token_result_diag(void *param)
{
	((mips3_device *)param)->func_token_result_diag();
}
#endif

/*-------------------------------------------------
    read_guest_dword_translated (2026-09-10) - REAL BUG
    FOUND while building the PID-filter calibration
    diagnostic: m_program->read_dword(vaddr) does NOT
    auto-translate through the TLB the way normal guest
    instruction execution does - confirmed live via the
    Lua console, comparing a raw m_program->read_dword()
    at PDAADDR (0xffffa000+off, always 0) against
    manager.machine.devices[":maincpu"].spaces["program"]
    :readv_u32() (a real, sensible KSEG0 pointer) for the
    exact same address. The Lua binding's own source
    (luaengine_mem.cpp's log_mem_read) shows why: it calls
    dev.translate(spacenum, TR_READ, address, tspace)
    BEFORE reading, translating logical->physical and
    possibly retargeting which address_space to read from.
    Shared here so every KSEG2/3-touching diagnostic uses
    the same correct pattern rather than each hand-rolling
    (and potentially getting wrong) their own - a bad
    translation here silently returns a plausible-looking
    wrong answer (0), not an error.
-------------------------------------------------*/

uint32_t mips3_device::read_guest_dword_translated(offs_t addr)
{
	address_space *tspace = nullptr;
	offs_t taddr = addr;
	if (!translate(AS_PROGRAM, device_memory_interface::TR_READ, taddr, tspace) || !tspace)
		return 0;
	return tspace->read_dword(taddr);
}

#ifdef MAME_SGI_TASK84_DRC_EVENT_LOG
/*-------------------------------------------------
    log_drc_event (2026-09-10) - see the macro's own
    comment in mips3.h. Called from execute_run()'s own
    dispatch loop (mips3.cpp, pure host C++, between JIT
    block executions) and code_compile_block() (this
    file, also pure host C++, outside the compiled-code
    path entirely) - deliberately zero UML/compiled-code
    footprint. `type` is a short static string
    ("COMPILE"/"MISSING_CODE"/"UNMAPPED"/"RESET_CACHE"/
    "OUT_OF_CYCLES"); `pc` is whatever guest PC is
    relevant to that event. Self-capped generously (this
    is much lower-frequency than per-guest-instruction
    diagnostics - expect low thousands per cc1 invocation
    at most, not hundreds of thousands).
-------------------------------------------------*/

void mips3_device::log_drc_event(const char *type, uint32_t pc)
{
	static uint64_t s_seq = 0;
	++s_seq;
	// OUT_OF_CYCLES fires continuously for as long as mamesgi runs at
	// all (once per MAME scheduling quantum, including idle time before
	// and between actual cc1 invocations) - NOT bounded by cc1 activity
	// the way this file's other diagnostics are, so this needs a much
	// higher cap than the per-hit-during-one-invocation ones (matching
	// CALL_TRACE_DIAG's own 3,000,000 precedent for a similarly
	// unbounded-by-cc1 source) rather than the lower caps used by
	// diagnostics that only fire during a single controlled invocation.
	// Raised 2026-09-10 for the scaled-up matched-pair batch (40+ cycles,
	// ~150-190K lines/cycle observed in the 8-cycle pilot - the original
	// 3,000,000 cap would have silently truncated the run partway through
	// with no error surfaced, the same silent-failure class this
	// investigation has been bitten by repeatedly).
	if (s_seq > 20000000)
		return;
	// Tagged with the real current PID (read_current_pid(), validated
	// 2026-09-10) so the log can be filtered to just one cc1 invocation's
	// own window after the fact, rather than trying to avoid logging
	// idle/other-process events at the source - a pure host-side read,
	// no guest-code-path risk, only adds real wall-clock cost (which
	// matters far less here than correctness).
	uint32_t const pid = read_current_pid();
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_drc_events.log", "a");
	if (!s_f)
		return;
	fprintf(s_f, "%llu %s pc=%08x mode=%u dispatch_mode=%u pid=%u\n",
		(unsigned long long)s_seq, type, pc, m_core->mode, m_core->dispatch_mode, pid);
	fflush(s_f);
}

/*-------------------------------------------------
    log_drc_exception (2026-09-10) - see
    MAME_SGI_TASK84_DRC_EVENT_LOG's comment in mips3.h.
    Called once per real exception OCCURRENCE via a
    single UML_CALLC placed at the very top of each
    exception type's SHARED handler trampoline
    (static_generate_exception(), generated once per
    type at setup time) - deliberately NOT inside the
    hot guest-code path under investigation (parse_answer/
    cpp_get_token's own compiled sequence), so this does
    not carry the same "could shift COMPILE_MAX_SEQUENCE
    boundaries in the code being studied" risk the user
    flagged for the earlier $s3 probe: exceptions are, by
    construction, rare, and their handler trampolines are
    separate compiled blocks from the hot path itself.
-------------------------------------------------*/

void mips3_device::log_drc_exception()
{
	log_drc_event(m_core->diag_exc_type == EXCEPTION_INTERRUPT ? "EXC_INTERRUPT" :
		m_core->diag_exc_type == EXCEPTION_TLBMOD ? "EXC_TLBMOD" :
		m_core->diag_exc_type == EXCEPTION_TLBLOAD ? "EXC_TLBLOAD" :
		m_core->diag_exc_type == EXCEPTION_TLBSTORE ? "EXC_TLBSTORE" :
		m_core->diag_exc_type == EXCEPTION_ADDRLOAD ? "EXC_ADDRLOAD" :
		m_core->diag_exc_type == EXCEPTION_ADDRSTORE ? "EXC_ADDRSTORE" :
		m_core->diag_exc_type == EXCEPTION_SYSCALL ? "EXC_SYSCALL" :
		m_core->diag_exc_type == EXCEPTION_BREAK ? "EXC_BREAK" :
		m_core->diag_exc_type == EXCEPTION_INVALIDOP ? "EXC_INVALIDOP" :
		m_core->diag_exc_type == EXCEPTION_BADCOP ? "EXC_BADCOP" :
		m_core->diag_exc_type == EXCEPTION_OVERFLOW ? "EXC_OVERFLOW" :
		m_core->diag_exc_type == EXCEPTION_TRAP ? "EXC_TRAP" :
		m_core->diag_exc_type == EXCEPTION_FPE ? "EXC_FPE" : "EXC_OTHER",
		m_core->pc);
}

static void cfunc_log_drc_exception(void *param)
{
	((mips3_device *)param)->log_drc_exception();
}

#ifdef MAME_SGI_TASK84_TLB_MISMATCH_ENTRY_DIAG
void mips3_device::log_tlb_mismatch_entry()
{
	log_drc_event("TLB_MISMATCH_ENTRY", m_core->diag_tlb_mismatch_pc);
}

static void cfunc_log_tlb_mismatch_entry(void *param)
{
	((mips3_device *)param)->log_tlb_mismatch_entry();
}
#endif
#endif

/*-------------------------------------------------
    read_current_pid (2026-09-10) - the validated
    PID-filter chain: PDAADDR (0xffffa000, fixed kernel
    VA, remapped per context switch - immu.h) ->
    pda_s.p_curuthread (offset 0x14, confirmed from the
    guest's own /usr/include/sys/pda.h) -> +4 (empirically
    calibrated: for this single-threaded-process case
    this reads back the SAME pointer, i.e. proc_t and
    uthread_s share a base address here - not yet checked
    for a genuinely multi-threaded sproc() process) -> +512
    is proc_t.p_pid. CROSS-VALIDATED live against two
    independent real cc1 invocations (captured via the test
    harness's own `cc1pid=$!`): pid 3661 and pid 3657 both
    read back correctly at this exact offset pair. See
    wiki/task84.md 2026-09-10 for the full derivation.
-------------------------------------------------*/

uint32_t mips3_device::read_current_pid()
{
	constexpr offs_t PDAADDR = 0xffffa000;
	constexpr offs_t P_CURUTHREAD_OFFSET = 0x14;
	constexpr offs_t UT_PROC_OFFSET = 4;
	constexpr offs_t P_PID_OFFSET = 512;

	uint32_t const uthread_ptr = read_guest_dword_translated(PDAADDR + P_CURUTHREAD_OFFSET);
	if (uthread_ptr < 0x80000000 || uthread_ptr == 0xffffffff)
		return 0;
	uint32_t const proc_ptr = read_guest_dword_translated(uthread_ptr + UT_PROC_OFFSET);
	if (proc_ptr < 0x80000000 || proc_ptr == 0xffffffff)
		return 0;
	return read_guest_dword_translated(proc_ptr + P_PID_OFFSET);
}

#ifdef MAME_SGI_TASK84_PID_CALIBRATION_DIAG
/*-------------------------------------------------
    func_pid_calibration_diag (2026-09-10) - see the
    macro's own comment. One-time empirical offset
    discovery for curprocp's uthread_s->ut_proc and
    proc_t->p_pid fields, using only the trusted hop
    (PDAADDR -> pda_s.p_curuthread, offset 0x14,
    confirmed from the guest's own pda.h) as a
    starting point. NOW CALIBRATED (see read_current_pid
    above) - this scan is kept only for re-deriving the
    offsets if a future kernel/config ever needs it again,
    not for routine use.
-------------------------------------------------*/

void mips3_device::func_pid_calibration_diag()
{
	static uint64_t s_seq = 0;
	++s_seq;
	if (s_seq > 800)
		return;
	static FILE *s_f = fopen("/home/blythe/mame-sgi/task84_pid_calib.log", "a");
	if (!s_f)
		return;

	constexpr offs_t PDAADDR = 0xffffa000;
	constexpr offs_t P_CURUTHREAD_OFFSET = 0x14;
	uint32_t const raw_tlb = vtlb_table()[PDAADDR >> 12];
	uint32_t const uthread_ptr = read_guest_dword_translated(PDAADDR + P_CURUTHREAD_OFFSET);
	fprintf(s_f, "seq=%llu uthread_ptr=%08x raw_tlb=%08x\n", (unsigned long long)s_seq, uthread_ptr, raw_tlb);
	if (uthread_ptr < 0x80000000 || uthread_ptr == 0xffffffff)
	{
		fflush(s_f);
		return;
	}

	for (uint32_t outer = 0; outer < 256; outer += 4)
	{
		uint32_t const proc_ptr = read_guest_dword_translated(uthread_ptr + outer);
		if (proc_ptr < 0x80000000 || proc_ptr == 0xffffffff)
			continue;
		for (uint32_t inner = 0; inner < 1024; inner += 4)
		{
			uint32_t const val = read_guest_dword_translated(proc_ptr + inner);
			if (val > 0 && val < 65536)
				fprintf(s_f, "  outer=%u proc_ptr=%08x inner=%u val=%u\n", outer, proc_ptr, inner, val);
		}
	}
	fflush(s_f);
}

static void cfunc_pid_calibration_diag(void *param)
{
	((mips3_device *)param)->func_pid_calibration_diag();
}
#endif

/*-------------------------------------------------
    cfunc_task84_t9_once_diag (2026-09-04) - fires
    at 0x1007df48 (the jalr itself), where $t9/r25
    holds whatever the preceding `lw $t9,-0x7998($gp)`
    at 0x1007df44 just loaded: cpp_get_token()'s real
    guest address. Logs exactly once (static bool) -
    it's a compile-time-constant call target that
    never varies, and this PC is hit thousands of
    times per run, so repeating would be pure noise.
    Needed because cc1 is stripped (no symbol table),
    so the callee's address can't be resolved
    statically from the binary alone.
-------------------------------------------------*/

void mips3_device::func_task84_t9_once_diag()
{
	static bool s_logged = false;
	if (s_logged)
		return;
	s_logged = true;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_window_trace.log", "a");
	if (!f)
		return;
	fprintf(f, "[T9_ONCE] cpp_get_token_addr=%08x gp=%08x ra=%08x\n",
		(uint32_t)m_core->r[25], (uint32_t)m_core->r[28], (uint32_t)m_core->r[31]);
	fclose(f);
}

static void cfunc_task84_t9_once_diag(void *param)
{
	((mips3_device *)param)->func_task84_t9_once_diag();
}
#endif

#ifdef MAME_SGI_TASK84_SAVE_RELOAD_AUDIT
/*-------------------------------------------------
    cfunc_task84_reload_mismatch (2026-09-04) -
    fires ONLY when m_core->r[2] differs, at DRC
    re-entry, from what save_fast_iregs() last
    flushed there. See the macro's own comment for
    why this is the decisive test. Self-capping:
    the whole point is that a real hit should be
    rare, and an unbounded log already cost 367MB
    in 20 iterations once this session.
-------------------------------------------------*/

void mips3_device::func_task84_reload_mismatch()
{
	static uint64_t s_hits = 0;
	++s_hits;
	if (s_hits > 200)
		return;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_window_trace.log", "a");
	if (!f)
		return;
	fprintf(f, "[RELOAD_MISMATCH] #%llu pc=%08x r2_now=%016llx last_saved=%016llx last_save_site=%u\n",
		(unsigned long long)s_hits, m_core->pc,
		(unsigned long long)m_core->r[2],
		(unsigned long long)m_core->diag_task84_last_saved_r2,
		m_core->diag_task84_trace_site);
	fclose(f);
}

static void cfunc_task84_reload_mismatch(void *param)
{
	((mips3_device *)param)->func_task84_reload_mismatch();
}
#endif

#ifdef MAME_SGI_MXCSR_DESYNC_DIAG
/*-------------------------------------------------
    cfunc_mxcsr_desync_diag (2026-08-24) - fires
    only on an actual mismatch between the real live
    host MXCSR and what the guest's own CCR1(31)
    rounding mode predicts (the inline compare in
    generate_sequence_instruction does the cheap
    per-instruction check; this only runs on the
    rare/nonexistent-if-the-desync-theory-is-wrong
    path). Confirms/pinpoints the exact guest PC
    where the desync first becomes observable, much
    finer-grained than the scheduler-level checkpoint
    that found zero hits.
-------------------------------------------------*/

void mips3_device::func_mxcsr_desync_diag()
{
	uint32_t const pc = m_core->diag_mxcsr_desync_pc;
	uint32_t const guest_rm = m_core->diag_mxcsr_desync_guest_rm;
	uint32_t const real_rc = m_core->diag_mxcsr_desync_real_rc;
	uint32_t const expected_rc = s_mxcsr_desync_expected_rc[guest_rm];

	static uint64_t s_total = 0;
	++s_total;
	if (s_total <= 200)
	{
		fprintf(stderr,
			"[MXCSR_DESYNC] event=%llu pc=0x%08x guest_rm=%u expected_rc=%u real_rc=%u\n",
			(unsigned long long)s_total, pc, guest_rm, expected_rc, real_rc);
		fflush(stderr);
		if (s_total == 200)
			fprintf(stderr, "[MXCSR_DESYNC] cap reached, further hits suppressed\n");
	}
}

static void cfunc_mxcsr_desync_diag(void *param)
{
	((mips3_device *)param)->func_mxcsr_desync_diag();
}
#endif

#ifdef MAME_SGI_CTC1_MXCSR_CHECK_DIAG
/*-------------------------------------------------
    cfunc_ctc1_mxcsr_check_diag (2026-08-24) - fires
    only when the real MXCSR immediately after a
    guest CTC1's own UML_SETFMOD doesn't match what
    that fmod value should have produced. Targets the
    suspected failure mode directly: op_setfmod's
    cache (m_state.fmod) believing no real ldmxcsr is
    needed when it actually is, leaving real MXCSR
    frozen on its prior value instead of updating -
    invisible to the record-and-compare checks in
    schedule.cpp/mips3.cpp's execute_run(), which can
    only see unexpected writes, not missing ones.
-------------------------------------------------*/

void mips3_device::func_ctc1_mxcsr_check_diag()
{
	uint32_t const pc = m_core->diag_ctc1_mxcsr_pc;
	uint32_t const fmod = m_core->diag_ctc1_mxcsr_fmod;
	uint32_t const real_rc = m_core->diag_ctc1_mxcsr_real_rc;
	uint32_t const expected_rc = s_ctc1_mxcsr_expected_rc[fmod];

	static uint64_t s_total = 0;
	++s_total;
	if (s_total <= 200)
	{
		fprintf(stderr,
			"[CTC1_MXCSR_CHECK] event=%llu pc=0x%08x fmod=%u expected_rc=%u real_rc=%u\n",
			(unsigned long long)s_total, pc, fmod, expected_rc, real_rc);
		fflush(stderr);
		if (s_total == 200)
			fprintf(stderr, "[CTC1_MXCSR_CHECK] cap reached, further hits suppressed\n");
	}
}

static void cfunc_ctc1_mxcsr_check_diag(void *param)
{
	((mips3_device *)param)->func_ctc1_mxcsr_check_diag();
}
#endif

#ifdef MAME_SGI_FPOP_MXCSR_CHECK_DIAG
/*-------------------------------------------------
    cfunc_fpop_mxcsr_check_diag (2026-08-24) - fires
    only when the real MXCSR doesn't match what the
    guest's own current FCR31 rounding mode calls for,
    checked immediately before an FP ADD/SUB/MUL -
    narrower than the CTC1-scoped check above (which
    found zero mismatches), targeting the specific
    window between a verified-correct CTC1 and the FP
    arithmetic that depends on it.
-------------------------------------------------*/

void mips3_device::func_fpop_mxcsr_check_diag()
{
	uint32_t const pc = m_core->diag_mxcsr_desync_pc;
	uint32_t const guest_rm = m_core->diag_mxcsr_desync_guest_rm;
	uint32_t const real_rc = m_core->diag_mxcsr_desync_real_rc;
	uint32_t const expected_rc = s_fpop_mxcsr_expected_rc[guest_rm];

	static uint64_t s_total = 0;
	++s_total;
	if (s_total <= 200)
	{
		fprintf(stderr,
			"[FPOP_MXCSR_CHECK] event=%llu pc=0x%08x guest_rm=%u expected_rc=%u real_rc=%u\n",
			(unsigned long long)s_total, pc, guest_rm, expected_rc, real_rc);
		fflush(stderr);
		if (s_total == 200)
			fprintf(stderr, "[FPOP_MXCSR_CHECK] cap reached, further hits suppressed\n");
	}
}

static void cfunc_fpop_mxcsr_check_diag(void *param)
{
	((mips3_device *)param)->func_fpop_mxcsr_check_diag();
}

void mips3_device::generate_fpop_mxcsr_precheck(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	uml::code_label const fpop_check_skip = compiler.labelnum++;
	UML_GETFMOD(block, I0);                                              // getfmod i0                  (real live MXCSR)
	UML_SHR(block, I0, I0, 13);                                          // shr     i0,i0,13
	UML_AND(block, I0, I0, 3);                                           // and     i0,i0,3              (real RC bits)
	UML_AND(block, I1, CCR132(31), 3);                                   // and     i1,ccr1[31],3        (guest RM)
	UML_LOAD(block, I2, &s_fpop_mxcsr_expected_rc[0], I1, SIZE_BYTE, SCALE_x1); // load i2,expected_rc,i1,byte
	UML_CMP(block, I0, I2);                                              // cmp     i0,i2
	UML_JMPc(block, COND_Z, fpop_check_skip);                            // jmp     skip,Z
	UML_MOV(block, mem(&m_core->diag_mxcsr_desync_pc), desc->pc);              // mov     [diag_pc],desc->pc
	UML_MOV(block, mem(&m_core->diag_mxcsr_desync_guest_rm), I1);              // mov     [diag_rm],i1
	UML_MOV(block, mem(&m_core->diag_mxcsr_desync_real_rc), I0);               // mov     [diag_real_rc],i0
	UML_CALLC(block, cfunc_fpop_mxcsr_check_diag, this);                      // callc   fpop_mxcsr_check_diag,mips3
	UML_LABEL(block, fpop_check_skip);                                   // skip:
}
#endif


/*-------------------------------------------------
    cfunc_printf_probe - print the current CPU
    state and return
-------------------------------------------------*/

void mips3_device::func_printf_probe()
{
	printf(" PC=%08X          r1=%08X%08X  r2=%08X%08X  r3=%08X%08X\n",
		m_core->pc,
		(uint32_t)(m_core->r[1] >> 32), (uint32_t)m_core->r[1],
		(uint32_t)(m_core->r[2] >> 32), (uint32_t)m_core->r[2],
		(uint32_t)(m_core->r[3] >> 32), (uint32_t)m_core->r[3]);
	printf(" r4=%08X%08X  r5=%08X%08X  r6=%08X%08X  r7=%08X%08X\n",
		(uint32_t)(m_core->r[4] >> 32), (uint32_t)m_core->r[4],
		(uint32_t)(m_core->r[5] >> 32), (uint32_t)m_core->r[5],
		(uint32_t)(m_core->r[6] >> 32), (uint32_t)m_core->r[6],
		(uint32_t)(m_core->r[7] >> 32), (uint32_t)m_core->r[7]);
	printf(" r8=%08X%08X  r9=%08X%08X r10=%08X%08X r11=%08X%08X\n",
		(uint32_t)(m_core->r[8] >> 32), (uint32_t)m_core->r[8],
		(uint32_t)(m_core->r[9] >> 32), (uint32_t)m_core->r[9],
		(uint32_t)(m_core->r[10] >> 32), (uint32_t)m_core->r[10],
		(uint32_t)(m_core->r[11] >> 32), (uint32_t)m_core->r[11]);
	printf("r12=%08X%08X r13=%08X%08X r14=%08X%08X r15=%08X%08X\n",
		(uint32_t)(m_core->r[12] >> 32), (uint32_t)m_core->r[12],
		(uint32_t)(m_core->r[13] >> 32), (uint32_t)m_core->r[13],
		(uint32_t)(m_core->r[14] >> 32), (uint32_t)m_core->r[14],
		(uint32_t)(m_core->r[15] >> 32), (uint32_t)m_core->r[15]);
	printf("r16=%08X%08X r17=%08X%08X r18=%08X%08X r19=%08X%08X\n",
		(uint32_t)(m_core->r[16] >> 32), (uint32_t)m_core->r[16],
		(uint32_t)(m_core->r[17] >> 32), (uint32_t)m_core->r[17],
		(uint32_t)(m_core->r[18] >> 32), (uint32_t)m_core->r[18],
		(uint32_t)(m_core->r[19] >> 32), (uint32_t)m_core->r[19]);
	printf("r20=%08X%08X r21=%08X%08X r22=%08X%08X r23=%08X%08X\n",
		(uint32_t)(m_core->r[20] >> 32), (uint32_t)m_core->r[20],
		(uint32_t)(m_core->r[21] >> 32), (uint32_t)m_core->r[21],
		(uint32_t)(m_core->r[22] >> 32), (uint32_t)m_core->r[22],
		(uint32_t)(m_core->r[23] >> 32), (uint32_t)m_core->r[23]);
	printf("r24=%08X%08X r25=%08X%08X r26=%08X%08X r27=%08X%08X\n",
		(uint32_t)(m_core->r[24] >> 32), (uint32_t)m_core->r[24],
		(uint32_t)(m_core->r[25] >> 32), (uint32_t)m_core->r[25],
		(uint32_t)(m_core->r[26] >> 32), (uint32_t)m_core->r[26],
		(uint32_t)(m_core->r[27] >> 32), (uint32_t)m_core->r[27]);
	printf("r28=%08X%08X r29=%08X%08X r30=%08X%08X r31=%08X%08X\n",
		(uint32_t)(m_core->r[28] >> 32), (uint32_t)m_core->r[28],
		(uint32_t)(m_core->r[29] >> 32), (uint32_t)m_core->r[29],
		(uint32_t)(m_core->r[30] >> 32), (uint32_t)m_core->r[30],
		(uint32_t)(m_core->r[31] >> 32), (uint32_t)m_core->r[31]);
	printf(" hi=%08X%08X  lo=%08X%08X\n",
		(uint32_t)(m_core->r[REG_HI] >> 32), (uint32_t)m_core->r[REG_HI],
		(uint32_t)(m_core->r[REG_LO] >> 32), (uint32_t)m_core->r[REG_LO]);
}

static void cfunc_printf_probe(void *param)
{
	((mips3_device *)param)->func_printf_probe();
}

/*-------------------------------------------------
    func_debug_break - debugger break
-------------------------------------------------*/

void mips3_device::func_debug_break()
{
	machine().debug_break();
}

static void cfunc_debug_break(void *param)
{
	((mips3_device *)param)->func_debug_break();
}

/*-------------------------------------------------
    cfunc_unimplemented - handler for
    unimplemented opcdes
-------------------------------------------------*/

void mips3_device::func_unimplemented()
{
	uint32_t opcode = m_core->arg0;
	fatalerror("PC=%08X: Unimplemented op %08X (%02X,%02X)\n", m_core->pc, opcode, opcode >> 26, opcode & 0x3f);
}

static void cfunc_unimplemented(void *param)
{
	((mips3_device *)param)->func_unimplemented();
}


/*-------------------------------------------------
    cfunc_task84_entry_irq_diag (2026-09-05) - fires
    only on the genuinely-taken path of
    static_generate_entry_point()'s unconditional
    interrupt check (i.e. only when an interrupt is
    actually about to be delivered there), right
    before the CALLH into the exception handler.
    This is the ONE interrupt-delivery mechanism left
    unfalsified after today's checkints experiments -
    see wiki/task84.md's 2026-09-05 "entry-point
    interrupt check" entry for why this specific site
    matters: it's the only place under DRC where a
    pending interrupt actually gets delivered, since
    execute_run()'s DRC path never calls check_irqs(),
    and today's controlled MAME_SGI_TASK84_FORCE_CHECKINTS_OFF
    test already ruled out the opportunistic in-sequence
    checks as the cause. Filtered to cpp_get_token's own
    address window (0x10076000-0x10076400) so the log
    stays small and directly relevant - this fires on
    every genuine interrupt delivery system-wide
    otherwise, which would mostly be noise unrelated to
    this investigation.
-------------------------------------------------*/

void mips3_device::func_task84_entry_irq_diag()
{
	uint32_t const pc = m_core->pc;
	if (pc < 0x10076000 || pc > 0x10076400)
		return;
	static uint64_t s_hit_count = 0;
	++s_hit_count;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_entry_irq_diag.log", "a");
	if (f)
	{
		using namespace std::chrono;
		auto const now = system_clock::now();
		auto const ms = duration_cast<milliseconds>(now.time_since_epoch()).count();
		time_t const secs = time_t(ms / 1000);
		struct tm tmval;
		localtime_r(&secs, &tmval);
		fprintf(f, "[%02d:%02d:%02d.%03d] #%llu interrupt delivered at resume pc=%08x Cause=%08x Status=%08x\n",
			tmval.tm_hour, tmval.tm_min, tmval.tm_sec, int(ms % 1000),
			(unsigned long long)s_hit_count, pc,
			(uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_Status]);
		fclose(f);
	}
}

#ifdef MAME_SGI_TASK84_ENTRY_IRQ_DIAG
static void cfunc_task84_entry_irq_diag(void *param)
{
	((mips3_device *)param)->func_task84_entry_irq_diag();
}
#endif

/*-------------------------------------------------
    cfunc_task84_saveload_bracket_entry/_exit
    (2026-09-05) - precisely brackets ONE
    cpp_get_token() invocation's save_fast_iregs()/
    load_fast_iregs() call-count activity, in guest-
    instruction order rather than wall-clock -
    correlated per cc1_null_loop.sh iteration against
    PASS/PREDICATE via tools/flush_saveload_bracket_sample.sh.
    See MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG's use
    site (generate_sequence_instruction) for the exact
    call/return PCs this brackets.
-------------------------------------------------*/

void mips3_device::func_task84_saveload_bracket_entry()
{
	m_core->diag_task84_bracket_save_entry = m_core->diag_task84_save_count;
	m_core->diag_task84_bracket_load_entry = m_core->diag_task84_load_count;
}

#ifdef MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG
static void cfunc_task84_saveload_bracket_entry(void *param)
{
	((mips3_device *)param)->func_task84_saveload_bracket_entry();
}
#endif

void mips3_device::func_task84_saveload_bracket_exit()
{
	uint64_t const save_delta = m_core->diag_task84_save_count - m_core->diag_task84_bracket_save_entry;
	uint64_t const load_delta = m_core->diag_task84_load_count - m_core->diag_task84_bracket_load_entry;
	static uint64_t s_call_count = 0;
	++s_call_count;
	FILE *f = fopen("/home/blythe/mame-sgi/task84_saveload_bracket.log", "a");
	if (f)
	{
		fprintf(f, "#%llu save_delta=%llu load_delta=%llu\n",
			(unsigned long long)s_call_count, (unsigned long long)save_delta, (unsigned long long)load_delta);
		fclose(f);
	}
}

#ifdef MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG
static void cfunc_task84_saveload_bracket_exit(void *param)
{
	((mips3_device *)param)->func_task84_saveload_bracket_exit();
}
#endif

#ifdef MAME_SGI_TASK101_REENTRY_FREQ_DIAG
// See diag_entry_reentry_count's comment in mips3.h. Called via UML_CALLC
// at the very top of static_generate_entry_point()'s generated code -
// runs on every single re-entry into the DRC dispatch loop from host C++
// (after a recompile, or a fresh scheduler call following EXECUTE_OUT_OF_
// CYCLES), regardless of whether an interrupt is actually pending. Pure
// in-memory counter/interval bookkeeping - no per-event I/O, so this
// stays cheap even at very high call frequency.
// Fires only when a gap exceeds MAME_SGI_TASK101_OUTLIER_THRESHOLD_NS -
// per-event, printing host wall-clock time (matches console_live.log's
// own timestamp convention, serial_driver.py --timestamps) so an
// outlier here can be lined up directly against a host-level monitoring
// log (e.g. /proc/stat steal time) captured over the same boot, same
// correlation technique already used for the compile-duration outliers
// earlier this session.
//
// Threshold history: originally 5ms for task #101 Phase 0's own
// characterization work, where seeing every outlier in detail was the
// point. That threshold is wrong now that this diagnostic has a second,
// permanent job as the checkints-disabled-by-default canary (see this
// macro's own top-of-file comment) - live-tested at 5ms during an
// ordinary boot and produced 4,445 print lines in ~2.5 minutes, which
// would flood every session's log indefinitely without helping spot a
// real regression (max_interval_ns/mean tracking below is unconditional
// regardless of this threshold, so nothing about the canary's actual
// detection capability depended on printing routine noise live).
// Raised to comfortably clear every already-characterized benign event
// this project has found (ordinary DRC/scheduling noise up to ~40ms,
// the known unrelated perform_dma() host stall at ~150ms) while still
// firing immediately and unmistakably on anything resembling the actual
// failure mode this canary watches for (the original mrupdatef
// livelock was a full 10 real seconds with zero progress).
#define MAME_SGI_TASK101_OUTLIER_THRESHOLD_NS 1'000'000'000ULL

void mips3_device::func_task101_entry_reentry_diag()
{
	using namespace std::chrono;
	uint64_t const now_ns = uint64_t(duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count());
	uint64_t const now_cycles = total_cycles();
	m_core->diag_entry_reentry_count++;
	if (m_core->diag_entry_last_timestamp_ns != 0)
	{
		uint64_t const interval_ns = now_ns - m_core->diag_entry_last_timestamp_ns;
		uint64_t const guest_cycles = now_cycles - m_core->diag_entry_last_total_cycles;
		m_core->diag_entry_interval_total_ns += interval_ns;
		if (interval_ns > m_core->diag_entry_interval_max_ns)
		{
			m_core->diag_entry_interval_max_ns = interval_ns;
			// See diag_entry_max_interval_guest_cycles's comment in
			// mips3.h - the whole point of tracking this alongside the
			// wall-clock interval.
			m_core->diag_entry_max_interval_guest_cycles = guest_cycles;
		}
		if (interval_ns < m_core->diag_entry_interval_min_ns)
			m_core->diag_entry_interval_min_ns = interval_ns;
		if (interval_ns > MAME_SGI_TASK101_OUTLIER_THRESHOLD_NS)
		{
			auto const now = system_clock::now();
			auto const t = system_clock::to_time_t(now);
			auto const us = duration_cast<microseconds>(now.time_since_epoch()) % 1'000'000;
			std::tm tm_buf;
			localtime_r(&t, &tm_buf);
			char tsbuf[24];
			std::snprintf(tsbuf, sizeof(tsbuf), "%02d:%02d:%02d.%06d", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, int(us.count()));
			osd_printf_info(
					"MAME_SGI_TASK101_REENTRY_FREQ_DIAG outlier: wallclock=%s interval_ns=%llu guest_cycles=%llu pc=%08x\n",
					tsbuf, (unsigned long long)interval_ns, (unsigned long long)guest_cycles, m_core->pc);
		}
	}
	m_core->diag_entry_last_timestamp_ns = now_ns;
	m_core->diag_entry_last_total_cycles = now_cycles;
}

static void cfunc_task101_entry_reentry_diag(void *param)
{
	((mips3_device *)param)->func_task101_entry_reentry_diag();
}

// Called via UML_CALLC at the top of static_generate_out_of_cycles() -
// runs every time this CPU exhausts its icount budget and exits back to
// host C++/the scheduler. No interval tracking needed here (this diag
// only cares about entry_point's own re-entry cadence) - just a count,
// to compare against diag_entry_reentry_count and see how tightly the
// two are coupled in practice.
void mips3_device::func_task101_out_of_cycles_diag()
{
	m_core->diag_out_of_cycles_count++;
}

static void cfunc_task101_out_of_cycles_diag(void *param)
{
	((mips3_device *)param)->func_task101_out_of_cycles_diag();
}
#endif

/***************************************************************************
    STATIC CODEGEN
***************************************************************************/

/*-------------------------------------------------
    static_generate_entry_point - generate a
    static entry point
-------------------------------------------------*/

void mips3_device::static_generate_entry_point(drcuml_block &block, int &label)
{
	uml::code_label const skip = label++;

	/* forward references */
	alloc_handle(*m_drcuml, m_exception_norecover[EXCEPTION_INTERRUPT], "interrupt_norecover");
	alloc_handle(*m_drcuml, m_nocode, "nocode");

	alloc_handle(*m_drcuml, m_entry, "entry");
	UML_HANDLE(block, *m_entry);                                                // handle  entry

#ifdef MAME_SGI_TASK101_REENTRY_FREQ_DIAG
	UML_CALLC(block, cfunc_task101_entry_reentry_diag, this);
#endif

	/* reset the FPU mode */
	UML_AND(block, I0, CCR132(31), 3);                                          // and     i0,ccr1[31],3
	UML_LOAD(block, I0, &m_fpmode[0], I0, SIZE_BYTE, SCALE_x1);                 // load    i0,fpmode,i0,byte
	UML_SETFMOD(block, I0);                                                     // setfmod i0

#ifdef MAME_SGI_TASK84_SAVE_RELOAD_AUDIT
	// BEFORE the reload: does memory still hold what the last save wrote? If
	// not, something changed r[2] behind the fast-register mechanism's back,
	// and load_fast_iregs() is about to push that value into r15 as the
	// guest's $v0. I0 is free here (the fmod setup above has finished with
	// it, and the interrupt check below reloads it from scratch).
	UML_DMOV(block, I0, mem(&m_core->r[2]));
	UML_DCMP(block, I0, mem(&m_core->diag_task84_last_saved_r2));
	UML_CALLCc(block, COND_NE, cfunc_task84_reload_mismatch, this);
#endif

	/* load fast integer registers */
	load_fast_iregs(block);
#ifdef MAME_SGI_TASK84_V0_SHADOW_DIAG
	// Found live 2026-09-07, after the label-collision fix: this is the
	// ONE active call site of load_fast_iregs() (the ERET-path reload at
	// generate_opcode's OP_ERET handling is gated behind
	// MAME_SGI_TASK84_ERET_FASTREG_RELOAD_FIX, currently disabled - see
	// that macro's own comment, "falsified by an immediate PANIC" per
	// wiki/task84.md 2026-09-05). Every EXECUTE_MISSING_CODE/out_of_cycles/
	// tlb_mismatch round-trip re-enters HERE and reloads $v0 from memory -
	// a legitimate, correct reload - but diag_v0_shadow is a plain memory
	// field never touched by load_fast_iregs()/save_fast_iregs()
	// themselves, only by this diagnostic's own before/after wrapping
	// inside generate_sequence_instruction(). Without this line, the
	// shadow-vs-live compare at the FIRST instruction after ANY re-entry
	// compares a freshly (correctly) reloaded value against whatever the
	// shadow happened to hold from a completely unrelated point in guest
	// execution - a guaranteed, meaningless "divergence" on every single
	// re-entry, which is an extremely common event (this was the real
	// cause of the storm of hits at 0x1007b650/0x1007c650 - both are
	// jalr-return sites, exactly the kind of PC that gets hashjmp'd back
	// into after a cache-miss round-trip - and very likely a meaningful
	// contributor to the 47.4% failure-rate spike measured with this gap
	// still open, via the per-hit CALLC+fopen/fprintf overhead landing
	// disproportionately on exactly the recompile-heavy code paths this
	// investigation already believes are timing-sensitive). Resyncing
	// here makes the shadow correctly reflect "whatever this specific
	// re-entry legitimately established", the same operation
	// generate_sequence_instruction's own "after" save performs for
	// intra-block adjacency - entry is just the other place continuity
	// needs to be established.
	UML_DMOV(block, mem(&m_core->diag_v0_shadow), R64(2));
#endif
#ifdef MAME_SGI_TASK84_WINDOW_TRACE
	{
		uml::code_label const trace_skip = label++;
		UML_CMP(block, mem(&m_core->diag_task84_armed), 0);
		UML_JMPc(block, COND_E, trace_skip);
		UML_CALLC(block, cfunc_task84_window_trace_reload, this);
		UML_LABEL(block, trace_skip);
	}
	{
		// Widened 2026-09-07 (third Opus review §7.1) from an exact match
		// on 0x1007df50 to the full cpp_get_token+parse_answer window, so
		// the probe isn't blind to anomalies at any other resume PC in
		// that range - unsigned range check since guest PCs here are
		// well below 0x80000000 (COND_B/COND_A are unsigned per uml.h).
		uml::code_label const trace_skip2 = label++;
		UML_CMP(block, mem(&m_core->pc), 0x10076000);
		UML_JMPc(block, COND_B, trace_skip2);
		UML_CMP(block, mem(&m_core->pc), 0x1007e100);
		UML_JMPc(block, COND_A, trace_skip2);
		UML_CALLC(block, cfunc_task84_entry_pc_diag, this);
		UML_LABEL(block, trace_skip2);
	}
#endif

	/* check for interrupts */
	UML_AND(block, I0, CPR032(COP0_Cause), CPR032(COP0_Status));                // and     i0,[Cause],[Status]
	UML_AND(block, I0, I0, 0xfc00);                                             // and     i0,i0,0xfc00,Z
	UML_JMPc(block, COND_Z, skip);                                              // jmp     skip,Z
	UML_TEST(block, CPR032(COP0_Status), SR_IE);                                // test    [Status],SR_IE
	UML_JMPc(block, COND_Z, skip);                                              // jmp     skip,Z
	UML_TEST(block, CPR032(COP0_Status), SR_EXL | SR_ERL);                      // test    [Status],SR_EXL | SR_ERL
	UML_JMPc(block, COND_NZ, skip);                                             // jmp     skip,NZ
#ifdef MAME_SGI_TASK84_ENTRY_IRQ_DIAG
	UML_CALLC(block, cfunc_task84_entry_irq_diag, this);
#endif
	UML_MOV(block, I0, mem(&m_core->pc));                                       // mov     i0,pc
	UML_MOV(block, I1, 0);                                                      // mov     i1,0
	UML_CALLH(block, *m_exception_norecover[EXCEPTION_INTERRUPT]);              // callh   exception_norecover
	UML_LABEL(block, skip);                                                     // skip:

	/* generate a hash jump via the current mode and PC */
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
	generate_select_dispatch_key(block, uml::mem(&m_core->pc), uml::I3, label);
	UML_HASHJMP(block, I3, mem(&m_core->pc), *m_nocode);        // hashjmp <mode>,<pc>,nocode
#else
	UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->pc), *m_nocode);        // hashjmp <mode>,<pc>,nocode
#endif
}


/*-------------------------------------------------
    static_generate_nocode_handler - generate an
    exception handler for "out of code"
-------------------------------------------------*/

// MAME_SGI_NOCODE_REFRESH_FIX (task #97, 2026-09-16): static_dispatch_key()
// bakes its resolved dispatch key as a compile-time constant at the jumping
// instruction's own compile time. If the target page's writability
// classification later changes (routine under ASID switches, since
// dispatch_page_writable() is a per-page WRITE_ALLOWED check), that baked
// key goes stale and the HASHJMP misses here even though valid code already
// exists under the CURRENT resolved key - the shared nocode handler had no
// way to tell "genuinely missing" apart from "stale key, code exists
// elsewhere", so it unconditionally paid a full recompile either way.
// Measured before this fix (MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG): 99.985%
// of arrivals at one hot PC's compile were exactly this stale-key case.
//
// Fix, directly modeled on the already-shipped, already-proven
// MAME_SGI_TLB_MISMATCH_REFRESH_FIX (same file, static_generate_tlb_
// mismatch() - see its own comment for the full reasoning this mirrors):
// before giving up, recompute the key LIVE (not the stale baked one) via
// generate_select_dispatch_key() and try one more HASHJMP. If a valid
// compile exists under the current key, this finds it and jumps away
// without ever reaching the recompile below. Same two brakes as the
// original: a one-shot per-PC retry guard (nocode_retry_pc) so a genuine
// double-miss falls through to a real compile instead of looping, and an
// icount charge with out_of_cycles bailout so no retry pattern can spin
// without the guest clock advancing.
#define MAME_SGI_NOCODE_REFRESH_FIX 1

void mips3_device::static_generate_nocode_handler(drcuml_block &block, int &label)
{
#ifdef MAME_SGI_NOCODE_REFRESH_FIX
	uml::code_label const no_retry = label++;
#endif

	/* generate a hash jump via the current mode and PC */
	alloc_handle(*m_drcuml, m_nocode, "nocode");
#ifdef MAME_SGI_NOCODE_REFRESH_FIX
	// Forward reference - the retry path charges a cycle and can bail out
	// to out_of_cycles, exactly as static_generate_tlb_mismatch()'s own
	// retry tail does.
	alloc_handle(*m_drcuml, m_out_of_cycles, "out_of_cycles");
#endif
	UML_HANDLE(block, *m_nocode);                                               // handle  nocode
	UML_GETEXP(block, I0);                                                      // getexp  i0
	UML_MOV(block, mem(&m_core->pc), I0);                                       // mov     [pc],i0
#ifdef MAME_SGI_VALIDATE_TLB_MISMATCH_CLASS_DIAG
	// Pinpointing which of the two EXECUTE_MISSING_CODE sites actually
	// fires for pc=0x0fac5ffc, given the guard-bounce case (live_value==0)
	// can't reach static_generate_tlb_mismatch's own skip/EXECUTE_MISSING_
	// CODE branch (established: FETCH_ALLOWED is clear for that case) -
	// meaning if compiles genuinely follow it (confirmed via the
	// causation check), a real HASHJMP miss must be landing here instead,
	// despite generate_select_dispatch_key()'s computed key looking
	// correct on static inspection everywhere it's called. this=I0 is
	// still the missed target PC at this point.
	UML_CMP(block, I0, 0x0fac5ffc);
	UML_CALLCc(block, COND_E, cfunc_nocode_0fac5ffc_diag, this);
#endif
	save_fast_iregs(block);
#ifdef MAME_SGI_TASK84_WINDOW_TRACE
	// diag_task84_trace_site is written UNCONDITIONALLY (2026-09-07, third
	// Opus review §7.1) - previously it was set only inside the
	// diag_task84_armed-gated block below, so it was stale/meaningless
	// outside the narrow armed window. func_task84_entry_pc_diag now reads
	// it on every sample (see its own updated comment), so it must reflect
	// the MOST RECENT save regardless of whether the armed-gated full
	// window-trace log also fires.
	UML_MOV(block, mem(&m_core->diag_task84_trace_site), 1);
	{
		uml::code_label const trace_skip = label++;
		UML_CMP(block, mem(&m_core->diag_task84_armed), 0);
		UML_JMPc(block, COND_E, trace_skip);
		UML_CALLC(block, cfunc_task84_window_trace_save, this);
		UML_LABEL(block, trace_skip);
	}
#endif

#ifdef MAME_SGI_NOCODE_REFRESH_FIX
	UML_ADD(block, mem(&m_core->diag_nocode_skip_total), mem(&m_core->diag_nocode_skip_total), 1);

	UML_CMP(block, I0, mem(&m_core->nocode_retry_pc));                          // cmp     i0,[nocode_retry_pc]
	UML_JMPc(block, COND_E, no_retry);                                          // jmp     no_retry,e
	UML_MOV(block, mem(&m_core->nocode_retry_pc), I0);                          // mov     [nocode_retry_pc],i0
	UML_ADD(block, mem(&m_core->diag_nocode_refresh_hits), mem(&m_core->diag_nocode_refresh_hits), 1);
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), 1);              // sub     [icount],[icount],1
	UML_EXHc(block, COND_S, *m_out_of_cycles, I0);                              // exh     out_of_cycles,i0,s
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
	generate_select_dispatch_key(block, uml::mem(&m_core->pc), uml::I3, label);
	UML_HASHJMP(block, I3, mem(&m_core->pc), *m_nocode);                        // hashjmp <mode>,[pc],nocode
#else
	UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->pc), *m_nocode); // hashjmp <mode>,[pc],nocode
#endif

	UML_LABEL(block, no_retry);                                                 // no_retry:
	// Disarm, so a later, genuinely unrelated miss at this same PC gets its
	// own retry rather than being permanently suppressed.
	UML_MOV(block, mem(&m_core->nocode_retry_pc), 0xffffffff);                  // mov     [nocode_retry_pc],~0
#endif
	UML_EXIT(block, EXECUTE_MISSING_CODE);                                      // exit    EXECUTE_MISSING_CODE
}


/*-------------------------------------------------
    static_generate_out_of_cycles - generate an
    out of cycles exception handler
-------------------------------------------------*/

void mips3_device::static_generate_out_of_cycles(drcuml_block &block, int &label)
{
	/* generate a hash jump via the current mode and PC */
	alloc_handle(*m_drcuml, m_out_of_cycles, "out_of_cycles");
	UML_HANDLE(block, *m_out_of_cycles);                                        // handle  out_of_cycles
#ifdef MAME_SGI_TASK101_REENTRY_FREQ_DIAG
	UML_CALLC(block, cfunc_task101_out_of_cycles_diag, this);
#endif
	UML_GETEXP(block, I0);                                                      // getexp  i0
	UML_MOV(block, mem(&m_core->pc), I0);                                       // mov     <pc>,i0
	save_fast_iregs(block);
#ifdef MAME_SGI_TASK84_WINDOW_TRACE
	// See static_generate_nocode_handler's matching comment: unconditional now.
	UML_MOV(block, mem(&m_core->diag_task84_trace_site), 2);
	{
		uml::code_label const trace_skip = label++;
		UML_CMP(block, mem(&m_core->diag_task84_armed), 0);
		UML_JMPc(block, COND_E, trace_skip);
		UML_CALLC(block, cfunc_task84_window_trace_save, this);
		UML_LABEL(block, trace_skip);
	}
#endif
	UML_EXIT(block, EXECUTE_OUT_OF_CYCLES);                                     // exit    EXECUTE_OUT_OF_CYCLES
}


/*-------------------------------------------------
    static_generate_tlb_mismatch - generate a
    TLB mismatch handler
-------------------------------------------------*/

void mips3_device::static_generate_tlb_mismatch(drcuml_block &block, int &label)
{
	uml::code_label const skip = label++;
#ifdef MAME_SGI_TLB_MISMATCH_REFRESH_FIX
	uml::code_label const no_retry = label++;
#endif

	/* forward references */
	alloc_handle(*m_drcuml, m_exception[EXCEPTION_TLBLOAD], "exception_tlbload");
	alloc_handle(*m_drcuml, m_exception[EXCEPTION_TLBLOAD_FILL], "exception_tlbload_fill");
#ifdef MAME_SGI_TLB_MISMATCH_REFRESH_FIX
	// The retry path charges a cycle and can bail out to out_of_cycles,
	// exactly as static_generate_exception()'s own tail does.
	alloc_handle(*m_drcuml, m_out_of_cycles, "out_of_cycles");
#endif

	/* generate a hash jump via the current mode and PC */
	alloc_handle(*m_drcuml, m_tlb_mismatch, "tlb_mismatch");
	UML_HANDLE(block, *m_tlb_mismatch);                                         // handle  tlb_mismatch
	UML_RECOVER(block, I0, MAPVAR_PC);                                          // recover i0,PC
	// Need the current PC address for the instruction to do the TLB lookup
	// so need to adjust I0 so it points to the true PC address in case this is a delay slot
	UML_ADD(block, I0, I0, 3);                                                  // add     i1,i0,3
	UML_AND(block, I0, I0, ~3);                                                 // and     i0,i0,~3
	UML_SHR(block, I1, I0, 12);                                                 // shr     i1,i0,12
	UML_LOAD(block, I1, (void *)vtlb_table(), I1, SIZE_DWORD, SCALE_x4);        // load    i1,[vtlb_table],i1,dword
#ifdef MAME_SGI_TASK84_TLB_MISMATCH_ENTRY_DIAG
	// TEMPORARY diagnostic (2026-09-10) - static_generate_tlb_mismatch()
	// is a SEPARATE shared handler from static_generate_exception() (the
	// one MAME_SGI_TASK84_DRC_EVENT_LOG's log_drc_exception() already
	// hooks) - a real gap found live: task84_drc_events.log has ZERO
	// visibility into tlb_mismatch activity, so a clean, PID-isolated
	// check of a real PREDICATE invocation showed no events at all for
	// PC 0x1007DFFC even though the compiled code there (compiler_page_
	// fault() case) unconditionally calls into this exact handler. Logs
	// into the SAME unified log via log_drc_event() so it can be ordered
	// against everything else already captured there. Same low-risk
	// shape as the exception-trampoline hook: fires only on real
	// tlb_mismatch invocations, a separate, comparatively rare code path
	// from parse_answer's own hot loop.
	UML_MOV(block, mem(&m_core->diag_tlb_mismatch_pc), I0);
	UML_CALLC(block, cfunc_log_tlb_mismatch_entry, this);
#endif
	if (PRINTF_MMU)
	{
		static const char text[] = "TLB mismatch @ %08X (ent=%08X)\n";
		UML_MOV(block, mem(&m_core->format), (uintptr_t)text);                  // mov     [format],text
		UML_MOV(block, mem(&m_core->arg0), I0);                                 // mov     [arg0],i0
		UML_MOV(block, mem(&m_core->arg1), I1);                                 // mov     [arg1],i1
		UML_CALLC(block, cfunc_printf_debug, this);                             // callc   printf_debug
	}
	UML_TEST(block, I1, FETCH_ALLOWED);                                         // test    i1,FETCH_ALLOWED
	UML_JMPc(block, COND_NZ, skip);                                             // jmp     skip,nz
	UML_TEST(block, I1, FLAG_FIXED);                                            // test    i1,FLAG_FIXED
	UML_EXHc(block, COND_NZ, *m_exception[EXCEPTION_TLBLOAD], I0);              // exh     exception[TLBLOAD],i0,nz
	UML_EXH(block, *m_exception[EXCEPTION_TLBLOAD_FILL], I0);                   // exh     exception[TLBLOAD_FILL],i0
	UML_LABEL(block, skip);                                                     // skip:
	save_fast_iregs(block);
#ifdef MAME_SGI_TASK84_WINDOW_TRACE
	// See static_generate_nocode_handler's matching comment: unconditional now.
	UML_MOV(block, mem(&m_core->diag_task84_trace_site), 3);
	{
		uml::code_label const trace_skip = label++;
		UML_CMP(block, mem(&m_core->diag_task84_armed), 0);
		UML_JMPc(block, COND_E, trace_skip);
		UML_CALLC(block, cfunc_task84_window_trace_save, this);
		UML_LABEL(block, trace_skip);
	}
#endif

	// the saved PC may be set 1 instruction back with the low bit set to indicate
	// a delay slot; in this path we want the previous instruction address, so recover it
	UML_RECOVER(block, I0, MAPVAR_PC);                                          // recover i0,PC
	UML_AND(block, I0, I0, ~3);                                                 // and     i0,i0,~3
	UML_MOV(block, mem(&m_core->pc), I0);                                       // mov     i0,[pc]

#ifdef MAME_SGI_TASK84_TLBMISS_REDISPATCH_DIAG
	// See the macro's own comment above. diag_tlb_mismatch_pc (set earlier
	// in this same function, before the vtlb lookup) already holds the
	// FIRST (rounded-up) recovery's value - reuse it here as "firstpc"
	// rather than recovering a third time.
	{
		uml::code_label const not_our_beql = label++;
		UML_CMP(block, mem(&m_core->pc), 0x1007dffc);
		UML_JMPc(block, COND_NE, not_our_beql);
		UML_DMOV(block, mem(&m_core->diag_tlbmiss_redispatch_s3), R64(19));
		UML_MOV(block, mem(&m_core->diag_tlbmiss_redispatch_firstpc), mem(&m_core->diag_tlb_mismatch_pc));
		UML_CALLC(block, cfunc_tlbmiss_redispatch_diag, this);
		UML_LABEL(block, not_our_beql);
	}
	// SECOND check (2026-09-10, same session): the OK/PREDICATE x64 diff
	// found label $10 (our beql's not-taken/annulled continuation, which
	// should redispatch to 0x1007E004) tail-merged by the backend with
	// the compiler_page_fault() stub tagged mapvar m0=$1007E020 (the
	// beql's TAKEN target) - because both are byte-identical unconditional
	// UML_EXH(tlb_mismatch,0) stubs once the whole 0x1007E0xx page is
	// compiler_page_fault'd at compile time. If real, a not-taken
	// execution redispatches to 0x1007E020 (the cpp_error path) instead
	// of 0x1007E004, regardless of $s3. Same cheap one-CMP gate.
	{
		uml::code_label const not_e020 = label++;
		UML_CMP(block, mem(&m_core->pc), 0x1007e020);
		UML_JMPc(block, COND_NE, not_e020);
		UML_DMOV(block, mem(&m_core->diag_tlbmiss_redispatch_s3), R64(19));
		UML_MOV(block, mem(&m_core->diag_tlbmiss_redispatch_firstpc), mem(&m_core->diag_tlb_mismatch_pc));
		UML_CALLC(block, cfunc_tlbmiss_redispatch_e020_diag, this);
		UML_LABEL(block, not_e020);
	}
#endif

#ifdef MAME_SGI_TLB_MISMATCH_REFRESH_FIX
	// THE FIX. Reaching here means the live TLB entry for this PC's page
	// has FETCH_ALLOWED set - the page IS mapped right now - yet the
	// compiled instance that dispatched to this handler could not proceed.
	// Stock behaviour is to unconditionally EXIT to EXECUTE_MISSING_CODE,
	// which runs a full describe_code() window walk plus (usually) a
	// codegen pass, even when a perfectly good compiled block for this
	// exact PC is already sitting in the hash table.
	//
	// Instead, redispatch through the ordinary hash table. This makes no
	// new assumption about whether any particular compiled instance is
	// still valid: HASHJMP re-enters through the normal dispatch path, so
	// whatever block it lands on runs its own validate_tlb guard exactly
	// as if the guest had branched here, and a hash miss falls through to
	// m_nocode, which re-raises EXECUTE_MISSING_CODE and compiles for
	// real. That is what separates this from the 2026-08-02 attempt that
	// livelocked (see the #if 0 block in code_compile_block()): that one
	// made code_compile_block() return WITHOUT compiling, so a hash miss
	// had no way to ever make progress. Here the miss path still compiles.
	//
	// Two independent brakes bound the retry:
	//
	//  1. tlb_mismatch_retry_pc - if the previous trip through this label
	//     already redispatched this same PC and we came straight back to
	//     it, the block registered at that PC must itself be the stub that
	//     sent us here (a compiler_page_fault() sequence head registers
	//     UML_HASH at its own PC), so retrying again cannot help. Take the
	//     recompile. This is the realistic self-loop and it is what the
	//     bare-early-return attempt hit.
	//  2. An icount charge plus the standard out_of_cycles exit, mirroring
	//     static_generate_exception()'s own tail. Even a pathological
	//     multi-PC retry cycle that brake 1 cannot see must therefore burn
	//     guest cycles and hand control back to the scheduler, so no retry
	//     pattern can spin without the machine advancing.
	//
	// Measured 2026-08-23 (full boot to login, 402,000 skip events): 96.2%
	// of arrivals here come from the compiler_page_fault() stub site, 3.8%
	// from validate_tlb()'s conditional fast path, 0% from the
	// compile-time-already-unmapped site (compiler_page_fault() is tested
	// first in generate_sequence_instruction() and always wins, so that
	// third site is unreachable in practice).
	UML_ADD(block, mem(&m_core->diag_tlb_mismatch_skip_total), mem(&m_core->diag_tlb_mismatch_skip_total), 1);

	UML_CMP(block, I0, mem(&m_core->tlb_mismatch_retry_pc));                    // cmp     i0,[tlb_mismatch_retry_pc]
	UML_JMPc(block, COND_E, no_retry);                                          // jmp     no_retry,e
	UML_MOV(block, mem(&m_core->tlb_mismatch_retry_pc), I0);                    // mov     [tlb_mismatch_retry_pc],i0
	UML_ADD(block, mem(&m_core->diag_tlb_mismatch_refresh_hits), mem(&m_core->diag_tlb_mismatch_refresh_hits), 1);
	// Reporting these two counters periodically happens from
	// code_compile_block() instead of here - this function only runs
	// ONCE, at device-start, to emit this shared handler's machine code;
	// a "static" local here would evaluate at codegen time, not per
	// runtime invocation, which is not useful for a runtime counter.
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), 1);              // sub     [icount],[icount],1
	UML_EXHc(block, COND_S, *m_out_of_cycles, I0);                              // exh     out_of_cycles,i0,s
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
	generate_select_dispatch_key(block, uml::mem(&m_core->pc), uml::I3, label);
	UML_HASHJMP(block, I3, mem(&m_core->pc), *m_nocode);                        // hashjmp <mode>,[pc],nocode
#else
	UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->pc), *m_nocode); // hashjmp <mode>,[pc],nocode
#endif

	UML_LABEL(block, no_retry);                                                 // no_retry:
	// Disarm, so a later, genuinely unrelated mismatch at this same PC
	// gets its own retry rather than being permanently suppressed.
	UML_MOV(block, mem(&m_core->tlb_mismatch_retry_pc), 0xffffffff);            // mov     [tlb_mismatch_retry_pc],~0
#endif
	UML_EXIT(block, EXECUTE_MISSING_CODE);                                      // exit    EXECUTE_MISSING_CODE
}


/*-------------------------------------------------
    static_generate_exception - generate a static
    exception handler
-------------------------------------------------*/

void mips3_device::static_generate_exception(drcuml_block &block, int &label, uint8_t exception, int recover, const char *name)
{
	uml::code_handle *&exception_handle = recover ? m_exception[exception] : m_exception_norecover[exception];
	uint32_t offset = 0x180;
	uml::code_label const next = label++;
	uml::code_label const skip = label++;

	/* translate our fake fill exceptions into real exceptions */
	if (exception == EXCEPTION_TLBLOAD_FILL || exception == EXCEPTION_TLBSTORE_FILL)
	{
		offset = 0x000;
		exception = (exception - EXCEPTION_TLBLOAD_FILL) + EXCEPTION_TLBLOAD;
	}

	/* add a global entry for this */
	alloc_handle(*m_drcuml, exception_handle, name);
	UML_HANDLE(block, *exception_handle);                                           // handle  name

#ifdef MAME_SGI_TASK84_DRC_EVENT_LOG
	// See MAME_SGI_TASK84_DRC_EVENT_LOG's comment in mips3.h and
	// log_drc_exception's own comment above. `exception` is compile-time
	// known here (already through the TLB-fill-type remap above), baked
	// in as an immediate - one CALLC per real exception occurrence, not
	// per guest instruction in the hot path this investigation cares
	// about keeping undisturbed.
	UML_MOV(block, mem(&m_core->diag_exc_type), exception);
	UML_CALLC(block, cfunc_log_drc_exception, this);
#endif

	/* exception parameter is expected to be the fault address in this case */
	if (exception == EXCEPTION_TLBLOAD || exception == EXCEPTION_TLBSTORE || exception == EXCEPTION_TLBMOD || exception == EXCEPTION_ADDRLOAD || exception == EXCEPTION_ADDRSTORE)
	{
		/* set BadVAddr to the fault address */
		UML_GETEXP(block, I0);                                                  // getexp  i0
		// FIX (2026-07-30): BadVAddr is a genuinely 64-bit COP0 register
		// (unlike Status/Cause/Count/Compare, which are architecturally
		// 32-bit-only). The interpreter's equivalent (generate_tlb_exception,
		// mips3.cpp) assigns the full 64-bit register from the fault
		// address, zero-extending it; a narrow 32-bit store here left the
		// upper 32 bits as whatever was previously in the register.
		// Zero-extend explicitly (GETEXP's own upper-bit contents aren't
		// otherwise guaranteed) before the widened store.
		UML_DAND(block, I0, I0, 0xffffffff);                                    // dand    i0,i0,0xffffffff
		UML_DMOV(block, CPR064(COP0_BadVAddr), I0);                             // dmov    [BadVAddr],i0
	}

	if (exception == EXCEPTION_TLBMOD || exception == EXCEPTION_TLBLOAD || exception == EXCEPTION_TLBSTORE)
	{
		/* set the upper bits of EntryHi and the lower bits of Context to the fault page */
		UML_ROLINS(block, CPR032(COP0_EntryHi), I0, 0, 0xffffe000); // rolins  [EntryHi],i0,0,0xffffe000
		UML_ROLINS(block, CPR032(COP0_Context), I0, 32-9, 0x7ffff0);    // rolins  [Context],i0,32-9,0x7ffff0
	}

	if (exception == EXCEPTION_FPE)
	{
		/* set the flag and cause */
		UML_GETEXP(block, I0);                                      // getexp  i0
		UML_ROLINS(block, CCR132(31), I0, FCR31_FLAGS, 0x0000007c); // rolins  [CCR31],i0,FCR31_FLAGS,0x0000007c
		UML_ROLINS(block, CCR132(31), I0, FCR31_CAUSE, 0x0003f000); // rolins  [CCR31],i0,FCR31_CAUSE,0x0003f000
	}

	/* set the EPC and Cause registers */
	if (recover)
	{
		UML_RECOVER(block, I0, MAPVAR_PC);                                      // recover i0,PC
		UML_RECOVER(block, I1, MAPVAR_CYCLES);                                  // recover i1,CYCLES
	}

	UML_AND(block, I2, CPR032(COP0_Cause), ~0x800000ff);                        // and     i2,[Cause],~0x800000ff
	UML_TEST(block, I0, 1);                                                     // test    i0,1
	UML_JMPc(block, COND_Z, next);                                              // jz      <next>
	UML_OR(block, I2, I2, 0x80000000);                                          // or      i2,i2,0x80000000
	UML_SUB(block, I0, I0, 1);                                                  // sub     i0,i0,1
	UML_LABEL(block, next);                                                     // <next>:
	UML_MOV(block, I3, offset);                                                 // mov     i3,offset
	// FIX (2026-07-30): EPC is a genuinely 64-bit COP0 register; the
	// interpreter assigns it from m_core->pc (a uint32_t), zero-extending
	// into the full 64-bit slot. Zero-extend I0 (a plain guest PC value via
	// RECOVER/arithmetic above) here, strictly BEFORE the flag-setting TEST
	// just below - an earlier attempt at this fix put the zero-extend
	// *between* TEST and the COND_Z move that consumes its flag, and
	// UML_DAND clobbering that flag as a side effect silently skipped the
	// EPC store on real guest execution (confirmed: caused an early-boot
	// KERNEL FAULT with PC=0x0, reverted, root-caused, now fixed by
	// reordering - see wiki/journal for the full incident). TEST must be
	// the last thing before the conditional move that depends on it.
	UML_DAND(block, I0, I0, 0xffffffff);                                        // dand    i0,i0,0xffffffff
	UML_TEST(block, CPR032(COP0_Status), SR_EXL);                               // test    [Status],SR_EXL
	UML_DMOVc(block, COND_Z, CPR064(COP0_EPC), I0);                             // dmov    [EPC],i0,Z
	UML_MOVc(block, COND_NZ, I3, 0x180);                                        // mov     i3,0x180,NZ
	UML_OR(block, CPR032(COP0_Cause), I2, exception << 2);                      // or      [Cause],i2,exception << 2

	/* for BADCOP exceptions, we use the exception parameter to know which COP */
	if (exception == EXCEPTION_BADCOP)
	{
		UML_GETEXP(block, I0);                                                  // getexp  i0
		UML_ROLINS(block, CPR032(COP0_Cause), I0, 28, 0x30000000);              // rolins  [Cause],i0,28,0x30000000

#ifdef MAME_SGI_COP1_TRAP_DIAG
		// I0 still holds the raw cop number here (ROLINS reads it as a
		// source, doesn't clobber it) - gate the C call in generated code
		// on cop==1 so the C function itself needs no arguments/instance
		// access at all, avoiding any mips3.h header change for a
		// one-shot investigation diagnostic.
		UML_CMP(block, I0, 1);                                                  // cmp     i0,1
		UML_CALLCc(block, COND_E, cfunc_cop1_trap_diag, this);                  // callc   cop1_trap_diag,e
#endif
	}

	// TEMPORARY probe (2026-07-27): stash the EPC candidate (I0, still valid
	// here - nothing since <next> has touched it) and check whether Status
	// already looks address-shaped *entering* this delivery, before this
	// function's own SR_EXL OR below touches it at all. See
	// func_interrupt_delivery_probe's own comment for the full rationale.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
	if (exception == EXCEPTION_INTERRUPT)
	{
		UML_MOV(block, mem(&m_core->diag_new_epc), I0);                        // mov     [diag_new_epc],i0
		UML_CALLC(block, cfunc_interrupt_delivery_probe, this);                // callc   cfunc_interrupt_delivery_probe
	}
#endif

	/* set EXL in the SR */
	UML_OR(block, I0, CPR032(COP0_Status), SR_EXL);                             // or      i0,[Status],SR_EXL
	UML_MOV(block, CPR032(COP0_Status), I0);                                    // mov     [Status],i0
	generate_update_mode(block);

#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG
	// `exception` is a per-call-site compile-time constant (this function is
	// invoked once per exception type during device_start-time codegen), so
	// this MOV bakes in an immediate - same idiom as `UML_MOV(block, I3,
	// offset)` a few lines above using the same kind of parameter.
	UML_MOV(block, mem(&m_core->diag_eret_window_exc_type), exception);         // mov     [diag_eret_window_exc_type],exception
	UML_CALLC(block, cfunc_eret_window_exception_diag, this);                   // callc   cfunc_eret_window_exception_diag
#endif

#ifdef MAME_DRC_DIAG
	if (exception == EXCEPTION_INTERRUPT)
		UML_CALLC(block, cfunc_interrupt_taken_probe, this);
#endif

	/* optionally print exceptions */
	if ((PRINTF_EXCEPTIONS && exception != EXCEPTION_INTERRUPT && exception != EXCEPTION_SYSCALL) ||
		(PRINTF_MMU && (exception == EXCEPTION_TLBLOAD || exception == EXCEPTION_TLBSTORE)))
	{
		UML_CALLC(block, cfunc_printf_exception, this);                         // callc   cfunc_printf_exception,nullptr
		//UML_CALLC(block, cfunc_debug_break, this);                            // callc   cfunc_debug_break,nullptr
	}

	/* choose our target PC */
	UML_ADD(block, I0, I3, 0xbfc00200);                                         // add     i0,i3,0xbfc00200
	UML_TEST(block, CPR032(COP0_Status), SR_BEV);                               // test    CPR032(COP0_Status),SR_BEV
	UML_JMPc(block, COND_NZ, skip);                                             // jnz     <skip>
	UML_ADD(block, I0, I3, 0x80000000);                                         // add     i0,i3,0x80000000,z
	UML_LABEL(block, skip);                                                     // <skip>:

	/* adjust cycles */
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), I1);             // sub icount,icount,cycles,S
	UML_EXHc(block, COND_S, *m_out_of_cycles, I0);                              // exh     out_of_cycles,i0

	// FIX (2026-07-30): every other static_generate_* handler that leaves
	// this function's own compiled context (static_generate_nocode_handler,
	// static_generate_out_of_cycles, static_generate_tlb_mismatch) calls
	// save_fast_iregs before its own exit point - this one, which hands off
	// to the guest exception vector via hashjmp, never did. $v0 is
	// fast-mapped to host register I4 (mips3.cpp:786-807; only I4's gate
	// clears on this project's actual Linux build - drcbex64.cpp's
	// non-Windows int_register_map has 5 entries, not the >=7 needed for
	// $v1/$a0 to also get mapped, corrected 2026-09-05 from an earlier,
	// wrong "$v0/$v1/$a0 -> I4/I5/I6" version of this comment) rather than
	// always living in m_core->r[]; without this, whatever that host
	// register held at the moment of the exception was never flushed to
	// memory before jumping into the vector, so any consumer that reads r[]
	// directly - a differently-compiled instance of the interrupted code
	// resumed later, or the exception vector itself - could see stale
	// values instead of what the interrupted code actually had live. See
	// MAME_IRIX_ERET_RACE_WORKAROUND's own comment for the specific guest
	// symptom (elocore_exl_3's $k0) this general gap plausibly explains.
	save_fast_iregs(block);
#ifdef MAME_SGI_TASK84_WINDOW_TRACE
	// See static_generate_nocode_handler's matching comment: unconditional now.
	UML_MOV(block, mem(&m_core->diag_task84_trace_site), 4);
	{
		uml::code_label const trace_skip = label++;
		UML_CMP(block, mem(&m_core->diag_task84_armed), 0);
		UML_JMPc(block, COND_E, trace_skip);
		UML_CALLC(block, cfunc_task84_window_trace_save, this);
		UML_LABEL(block, trace_skip);
	}
#endif

#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
	generate_select_dispatch_key(block, uml::I0, uml::I3, label);	UML_HASHJMP(block, I3, I0, *m_nocode);                      // hashjmp <mode>,i0,nocode
#else
	UML_HASHJMP(block, mem(&m_core->dispatch_mode), I0, *m_nocode);                      // hashjmp <mode>,i0,nocode
#endif
}


/*------------------------------------------------------------------
    static_generate_memory_mode_checks
------------------------------------------------------------------*/

void mips3_device::static_generate_memory_mode_checks(drcuml_block &block, uml::code_handle &exception_addrerr, int &label, int mode)
{
	/* user mode? generate address exception if top bit is set */
	if (mode == MODE_USER)
	{
		UML_TEST(block, I0, 0x80000000);                                            // test    i0,0x80000000
		UML_EXHc(block, COND_NZ, exception_addrerr, I0);                            // exh     addrerr,i0,nz
	}

	/* supervisor mode? generate address exception if not in user space or in $C0000000-DFFFFFFF */
	if (mode == MODE_SUPER)
	{
#ifdef MAME_IRIX_KERNEL_FAULT_WORKAROUND
		// TEMPORARY WORKAROUND (2026-07-27, widened further same day), NOT a
		// real fix. Root cause still open (see wiki/assets.md's "KERNEL
		// FAULT deep dive"): a shared C function (confirmed via real IRIX
		// source, tlbmiss()/tlbmod()) gets compiled once per real mode it's
		// ever reached under, and its own compiled instance's
		// memory-accessor selection (static_generate_memory_accessor,
		// m_read64/m_write64[mode>>1]) is permanently baked in - a
		// legitimate kernel-mode call can end up stuck running through an
		// already-cached supervisor-mode instance. The first cut of this
		// workaround only widened the accepted range from 0xC0000000-
		// 0xDFFFFFFF to 0xC0000000-0xFFFFFFFF, which fixed the originally-
		// found SD/write fault (a KSEG2 kernel stack address,
		// 0xffffc000-0xffffffff) but NOT a second, independently-confirmed
		// instance of the exact same mechanism on the LD/read side, hitting
		// a KSEG0 address instead (~0x8892b150, ep->ef_sr in the exception
		// frame) - a completely different segment the widened range never
		// covered either. Since mode-tracking is already known-unreliable
		// at this cache boundary (that's the entire reason this workaround
		// exists), guessing at further specific ranges one newly-discovered
		// fault at a time doesn't converge - so this drops the range check
		// for MODE_SUPER entirely, matching MODE_KERNEL's own already-
		// zero-check behavior. A real supervisor-mode program legitimately
		// referencing kseg0/kseg1/kseg2/kseg3 would now silently succeed
		// instead of correctly faulting (real MIPS supervisor mode may only
		// reach kuseg and ksseg) - remove once the real mode-selection bug
		// (why a shared kernel function's cached instance's compile-time
		// mode snapshot doesn't match the real privilege level it's later
		// invoked under) is found and fixed.
#else
		int addrok;
		UML_TEST(block, I0, 0x80000000);                                            // test    i0,0x80000000
		UML_JMPc(block, COND_Z, addrok = label++);                                  // jz      addrok
		UML_SHR(block, I3, I0, 29);                                                 // shr     i3,i0,29
		UML_CMP(block, I3, 6);                                                      // cmp     i3,6
		UML_EXHc(block, COND_NE, exception_addrerr, I0);                            // exh     addrerr,i0,ne
		UML_LABEL(block, addrok);                                                   // addrok:
#endif
	}
}


/*------------------------------------------------------------------
    static_generate_fastram_accessor
------------------------------------------------------------------*/

void mips3_device::static_generate_fastram_accessor(drcuml_block &block, int &label, int size, bool iswrite, bool ismasked)
{
	if (!debugger_enabled())
	{
		for (int ramnum = 0; ramnum < m_fastram_select; ramnum++)
		{
			if (!(iswrite && m_fastram[ramnum].readonly))
			{
				void *fastbase = (uint8_t *)m_fastram[ramnum].base - m_fastram[ramnum].start;
				uint32_t skip = label++;
				if (m_fastram[ramnum].end != 0xffffffff)
				{
					UML_CMP(block, I0, m_fastram[ramnum].end);                      // cmp     i0,end
					UML_JMPc(block, COND_A, skip);                                  // ja      skip
				}
				if (m_fastram[ramnum].start != 0x00000000)
				{
					UML_CMP(block, I0, m_fastram[ramnum].start);                    // cmp     i0,fastram_start
					UML_JMPc(block, COND_B, skip);                                  // jb      skip
				}
				if (!iswrite)
				{
					if (size == 1)
					{
						UML_XOR(block, I0, I0, m_bigendian ? BYTE4_XOR_BE(0) : BYTE4_XOR_LE(0));
																					// xor     i0,i0,bytexor
						UML_LOAD(block, I0, fastbase, I0, SIZE_BYTE, SCALE_x1);     // load    i0,fastbase,i0,byte
					}
					else if (size == 2)
					{
						UML_XOR(block, I0, I0, m_bigendian ? WORD_XOR_BE(0) : WORD_XOR_LE(0));
																					// xor     i0,i0,wordxor
						UML_LOAD(block, I0, fastbase, I0, SIZE_WORD, SCALE_x1);     // load    i0,fastbase,i0,word_x1
					}
					else if (size == 4)
					{
						UML_LOAD(block, I0, fastbase, I0, SIZE_DWORD, SCALE_x1);    // load    i0,fastbase,i0,dword_x1
					}
					else if (size == 8)
					{
						UML_DLOAD(block, I0, fastbase, I0, SIZE_QWORD, SCALE_x1);   // dload   i0,fastbase,i0,qword_x1
						UML_DROR(block, I0, I0, 32 * (m_bigendian ? BYTE_XOR_BE(0) : BYTE_XOR_LE(0)));
																					// dror    i0,i0,32*bytexor
					}
#ifdef MAME_SGI_RODATA_LOAD_ADDR_DIAG
					// See MAME_SGI_RODATA_LOAD_ADDR_DIAG's own comment
					// (near the top of this file). The post-read check in
					// static_generate_memory_accessor() never fires for
					// ordinary system RAM - this fastram path returns
					// (below) before reaching it. This is the actual path
					// real reads of cc1's .rodata take, confirmed live:
					// the memory_accessor-only version reported zero
					// events across a run that included several real
					// gcc failures, which is what motivated moving the
					// check here instead. I0 holds the freshly loaded
					// value at this exact point; the address was already
					// stashed into m_core->diag_rodata_addr/hit by this
					// same read's earlier pass through static_generate_
					// memory_accessor(), before it called here.
					{
						int rodata_diag_skip3 = 0;
						UML_CMP(block, mem(&m_core->diag_rodata_hit), 0);
						UML_JMPc(block, COND_E, rodata_diag_skip3 = label++);
						UML_MOV(block, mem(&m_core->diag_rodata_value), I0);
						UML_CALLC(block, cfunc_rodata_load_addr_diag, this);
						UML_LABEL(block, rodata_diag_skip3);
					}
#endif
					UML_RET(block);                                                 // ret
				}
				else
				{
					if (size == 1)
					{
						UML_XOR(block, I0, I0, m_bigendian ? BYTE4_XOR_BE(0) : BYTE4_XOR_LE(0));
																					// xor     i0,i0,bytexor
						UML_STORE(block, fastbase, I0, I1, SIZE_BYTE, SCALE_x1);    // store   fastbase,i0,i1,byte
					}
					else if (size == 2)
					{
						UML_XOR(block, I0, I0, m_bigendian ? WORD_XOR_BE(0) : WORD_XOR_LE(0));
																					// xor     i0,i0,wordxor
						UML_STORE(block, fastbase, I0, I1, SIZE_WORD, SCALE_x1);    // store   fastbase,i0,i1,word_x1
					}
					else if (size == 4)
					{
						if (ismasked)
						{
							UML_LOAD(block, I3, fastbase, I0, SIZE_DWORD, SCALE_x1);    // load    i3,fastbase,i0,dword_x1
							UML_ROLINS(block, I3, I1, 0, I2);                           // rolins  i3,i1,0,i2
							UML_STORE(block, fastbase, I0, I3, SIZE_DWORD, SCALE_x1);   // store   fastbase,i0,i3,dword_x1
						}
						else
							UML_STORE(block, fastbase, I0, I1, SIZE_DWORD, SCALE_x1);   // store   fastbase,i0,i1,dword_x1
					}
					else if (size == 8)
					{
						UML_DROR(block, I1, I1, 32 * (m_bigendian ? BYTE_XOR_BE(0) : BYTE_XOR_LE(0)));
																						// dror    i1,i1,32*bytexor
						if (ismasked)
						{
							UML_DROR(block, I2, I2, 32 * (m_bigendian ? BYTE_XOR_BE(0) : BYTE_XOR_LE(0)));
																						// dror    i2,i2,32*bytexor
							UML_DLOAD(block, I3, fastbase, I0, SIZE_QWORD, SCALE_x1);   // dload   i3,fastbase,i0,qword_x1
							UML_DROLINS(block, I3, I1, 0, I2);                          // drolins i3,i1,0,i2
							UML_DSTORE(block, fastbase, I0, I3, SIZE_QWORD, SCALE_x1);  // dstore  fastbase,i0,i3,qword_x1
						}
						else
							UML_DSTORE(block, fastbase, I0, I1, SIZE_QWORD, SCALE_x1);  // dstore  fastbase,i0,i1,qword_x1
					}
					UML_RET(block);                                                     // ret
				}

				UML_LABEL(block, skip);                                                 // skip:
			}
		}
	}
}


/*------------------------------------------------------------------
    static_generate_memory_rw
------------------------------------------------------------------*/

void mips3_device::static_generate_memory_rw(drcuml_block &block, int size, bool iswrite, bool ismasked)
{
	switch (size)
	{
		case 1:
			if (iswrite)
				UML_WRITE(block, I0, I1, SIZE_BYTE, SPACE_PROGRAM);                 // write   i0,i1,program_byte
			else
				UML_READ(block, I0, I0, SIZE_BYTE, SPACE_PROGRAM);                  // read    i0,i0,program_byte
			break;

		case 2:
			if (iswrite)
				UML_WRITE(block, I0, I1, SIZE_WORD, SPACE_PROGRAM);                 // write   i0,i1,program_word
			else
				UML_READ(block, I0, I0, SIZE_WORD, SPACE_PROGRAM);                  // read    i0,i0,program_word
			break;

		case 4:
			if (iswrite)
			{
				if (!ismasked)
					UML_WRITE(block, I0, I1, SIZE_DWORD, SPACE_PROGRAM);            // write   i0,i1,program_dword
				else
					UML_WRITEM(block, I0, I1, I2, SIZE_DWORD, SPACE_PROGRAM);       // writem  i0,i1,i2,program_dword
			}
			else
			{
				if (!ismasked)
					UML_READ(block, I0, I0, SIZE_DWORD, SPACE_PROGRAM);             // read    i0,i0,program_dword
				else
					UML_READM(block, I0, I0, I2, SIZE_DWORD, SPACE_PROGRAM);        // readm   i0,i0,i2,program_dword
			}
			break;

		case 8:
			if (iswrite)
			{
				if (!ismasked)
					UML_DWRITE(block, I0, I1, SIZE_QWORD, SPACE_PROGRAM);           // dwrite  i0,i1,program_qword
				else
					UML_DWRITEM(block, I0, I1, I2, SIZE_QWORD, SPACE_PROGRAM);      // dwritem i0,i1,i2,program_qword
			}
			else
			{
				if (!ismasked)
					UML_DREAD(block, I0, I0, SIZE_QWORD, SPACE_PROGRAM);            // dread   i0,i0,program_qword
				else
					UML_DREADM(block, I0, I0, I2, SIZE_QWORD, SPACE_PROGRAM);       // dreadm  i0,i0,i2,program_qword
			}
			break;
	}
	UML_RET(block);                                                                 // ret
}


/*------------------------------------------------------------------
    static_generate_memory_accessor
------------------------------------------------------------------*/

void mips3_device::static_generate_memory_accessor(drcuml_block &block, int &label, int mode, int size, bool iswrite, bool ismasked, const char *name, uml::code_handle *&handleptr)
{
	/* on entry, address is in I0; data for writes is in I1; mask for accesses is in I2 */
	/* on exit, read result is in I0 */
	/* routine trashes I0-I3 */
	uml::code_handle &exception_tlb = *m_exception[iswrite ? EXCEPTION_TLBSTORE : EXCEPTION_TLBLOAD];
	uml::code_handle &exception_tlbfill = *m_exception[iswrite ? EXCEPTION_TLBSTORE_FILL : EXCEPTION_TLBLOAD_FILL];
	uml::code_handle &exception_addrerr = *m_exception[iswrite ? EXCEPTION_ADDRSTORE : EXCEPTION_ADDRLOAD];
	int tlbmiss = 0;

	/* add a global entry for this */
	alloc_handle(*m_drcuml, handleptr, name);
	UML_HANDLE(block, *handleptr);                                                  // handle  handleptr

	static_generate_memory_mode_checks(block, exception_addrerr, label, mode);

#ifdef MAME_SGI_RODATA_LOAD_ADDR_DIAG
	// See this define's own comment above. I0 still holds the guest
	// VIRTUAL address here (the ROLINS below is what overwrites it with
	// the physical address) - this is the only point in this routine
	// where the virtual address is available to compare against the
	// known failing page's VA range.
	{
		int rodata_diag_skip = 0;
		UML_MOV(block, mem(&m_core->diag_rodata_hit), 0);
		if (!iswrite)
		{
			UML_CMP(block, I0, 0x10343000);
			UML_JMPc(block, COND_B, rodata_diag_skip = label++);
			UML_CMP(block, I0, 0x10344000);
			UML_JMPc(block, COND_AE, rodata_diag_skip);
			UML_MOV(block, mem(&m_core->diag_rodata_addr), I0);
			UML_MOV(block, mem(&m_core->diag_rodata_hit), 1);
			UML_LABEL(block, rodata_diag_skip);
		}
	}
#endif

	/* general case: assume paging and perform a translation */
	UML_SHR(block, I3, I0, 12);                                                     // shr     i3,i0,12
	UML_LOAD(block, I3, (void *)vtlb_table(), I3, SIZE_DWORD, SCALE_x4);            // load    i3,[vtlb_table],i3,dword
	UML_TEST(block, I3, iswrite ? WRITE_ALLOWED : READ_ALLOWED);                    // test    i3,iswrite ? WRITE_ALLOWED : READ_ALLOWED
	UML_JMPc(block, COND_Z, tlbmiss = label++);                                     // jmp     tlbmiss,z
	UML_ROLINS(block, I0, I3, 0, 0xfffff000);                                       // rolins  i0,i3,0,0xfffff000

	static_generate_fastram_accessor(block, label, size, iswrite, ismasked);
	static_generate_memory_rw(block, size, iswrite, ismasked);

#ifdef MAME_SGI_RODATA_LOAD_ADDR_DIAG
	// Post-read: I0 now holds the loaded value (per this function's own
	// "on exit, read result is in I0" contract). Fire only if the
	// pre-check above found this specific invocation's address in
	// range - the hit flag is reset unconditionally at the top of every
	// call, so this can't fire on stale data from an unrelated earlier
	// access.
	{
		int rodata_diag_skip2 = 0;
		UML_CMP(block, mem(&m_core->diag_rodata_hit), 0);
		UML_JMPc(block, COND_E, rodata_diag_skip2 = label++);
		UML_MOV(block, mem(&m_core->diag_rodata_value), I0);
		UML_CALLC(block, cfunc_rodata_load_addr_diag, this);
		UML_LABEL(block, rodata_diag_skip2);
	}
#endif

	UML_LABEL(block, tlbmiss);                                                      // tlbmiss:
	if (iswrite)
	{
		UML_TEST(block, I3, READ_ALLOWED);                                          // test    i3,READ_ALLOWED
		UML_EXHc(block, COND_NZ, *m_exception[EXCEPTION_TLBMOD], I0);               // exh     tlbmod,i0,nz
	}
	UML_TEST(block, I3, FLAG_FIXED);                                                // test    i3,FLAG_FIXED
	UML_EXHc(block, COND_NZ, exception_tlb, I0);                                    // exh     tlb,i0,nz
	UML_EXH(block, exception_tlbfill, I0);                                          // exh     tlbfill,i0
}

void r4650_device::static_generate_memory_accessor(drcuml_block &block, int &label, int mode, int size, bool iswrite, bool ismasked, const char *name, uml::code_handle *&handleptr)
{
	/* on entry, address is in I0; data for writes is in I1; mask for accesses is in I2 */
	/* on exit, read result is in I0 */
	/* routine trashes I0-I3 */
	uml::code_handle &exception_addrerr = *m_exception[iswrite ? EXCEPTION_ADDRSTORE : EXCEPTION_ADDRLOAD];

	/* add a global entry for this */
	alloc_handle(*m_drcuml, handleptr, name);
	UML_HANDLE(block, *handleptr);                                                  // handle  handleptr

	static_generate_memory_mode_checks(block, exception_addrerr, label, mode);

	if (mode == MODE_USER)
	{
		int addrok;
		UML_CMP(block, I0, CPR032(COP0_R4650_DBound));                              // cmp     i0,CPR0[DBound]
		UML_JMPc(block, COND_LE, addrok = label++);                                 // jle     addrok
		UML_EXHc(block, COND_G, exception_addrerr, I0);                             // exh     addrerr,i0,ne
		UML_LABEL(block, addrok);                                                   // addrok:
		UML_ADD(block, I0, I0, CPR032(COP0_R4650_DBase));                           // add     i0,i0,CPR0[DBase]
	}

	static_generate_fastram_accessor(block, label, size, iswrite, ismasked);
	static_generate_memory_rw(block, size, iswrite, ismasked);
}


/***************************************************************************
    CODE GENERATION
***************************************************************************/

/*-------------------------------------------------
    generate_update_mode - update the mode based
    on a new SR (in i0); trashes i1, i2
-------------------------------------------------*/

void mips3_device::generate_update_mode(drcuml_block &block)
{
	UML_ROLAND(block, I2, I0, 32-2, 0x06);                      // roland  i2,i0,32-2,0x06
	UML_TEST(block, I0, SR_EXL | SR_ERL);                                   // test    i0,SR_EXL | SR_ERL
	UML_MOVc(block, COND_NZ, I2, 0);                                        // mov     i2,0,nz
	UML_ROLINS(block, I2, I0, 32-26, 0x01);                     // rolins  i2,i0,32-26,0x01
	UML_MOV(block, mem(&m_core->mode), I2);                            // mov     [mode],i2

#ifdef MAME_SGI_DISPATCH_MODE_ASID_FIX
	// MAME_SGI_DISPATCH_MODE_ASID_FIX (2026-08-01, refined 2026-08-02):
	// compute the ASID-scoped composite from the mode just derived above
	// (still in i2) plus a live read of the current ASID (EntryHi's low
	// byte). Kernel gets a fixed sentinel (asid component forced to 0
	// below, regardless of the live ASID value) - that constancy is what
	// keeps kernel code's existing immunity to cross-process collision
	// intact; SUPER/USER get ((real_asid+1)<<2) | (is_user<<1) | FR_bit.
	// See the toggle comment at the top of this file for what value 1
	// vs 2 actually does with this value - this computation itself is
	// identical either way, it's only *used* differently: value 1 uses
	// it unconditionally for every non-kernel dispatch (the confirmed-
	// regressive first pass); value 2 uses it only for pages confirmed
	// writable via a live VTLB check, chosen per-PC at code_compile_block
	// (registration) and at each dispatch site (this function only
	// produces the value, doesn't decide when to use it - see
	// code_compile_block() and the UML_HASHJMP call sites).
	UML_AND(block, I1, CPR032(COP0_EntryHi), 0xff);             // i1 = live ASID
	UML_ADD(block, I1, I1, 1);                                  // i1 = asid+1
	UML_SHL(block, I1, I1, 2);                                  // i1 = (asid+1)<<2
	UML_ROLINS(block, I1, I2, 31, 0x02);                        // i1 bit1 = is_user (mode bit2)
	UML_TEST(block, I2, 0x06);                                  // Z iff mode's privilege bits are 0 (kernel)
	UML_MOVc(block, COND_Z, I1, 0);                             // kernel: zero the asid/is_user portion
	UML_ROLINS(block, I1, I2, 0, 0x01);                         // i1 bit0 = FR (mode bit0), always
	UML_MOV(block, mem(&m_core->dispatch_mode), I1);            // mov     [dispatch_mode],i1
#else
	// Fix disabled (see above) - dispatch_mode just mirrors mode, which
	// restores the exact pre-fix behavior everywhere dispatch_mode is
	// used (the 14 UML_HASH/UML_HASHJMP/hash_exists sites, and
	// execute_run()'s code_compile_block() call) without needing to
	// touch any of those sites individually.
	UML_MOV(block, mem(&m_core->dispatch_mode), I2);            // mov     [dispatch_mode],i2
#endif
}


#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
/*-------------------------------------------------
    generate_select_dispatch_key - MAME_SGI_
    DISPATCH_MODE_ASID_FIX value 2: live equivalent
    of dispatch_page_writable() for dispatch targets
    not known until runtime (JR/JALR, eret) - loads
    both the target's own page AND its delay slot's
    page (target_pc+4 - see dispatch_page_writable()'s
    comment in mips3.h for why this must mirror that
    function exactly, in lockstep, not just approximate
    it) and selects between m_core->mode (shared,
    granted only if BOTH pages are currently fetchable
    and non-writable) and m_core->dispatch_mode
    (ASID-scoped private key, the default) into `dest`.

    Same-page short-circuit (2026-08-06): pc and pc+4
    land in different pages only when pc is the last
    instruction slot in a page (low 12 bits == 0xffc) -
    true for at most 1 in 1024 dispatches. That test is
    pure arithmetic on bits already available, no memory
    access needed, so it's cheap and well-predicted to
    check first and skip the second, almost-always-
    redundant vtlb_table() load entirely in the common
    case. An earlier version always did both loads
    unconditionally, doubling the cost of this function's
    hottest call sites (JR/JALR's `jmpdest`, this file's
    own "main entry point into shared library code") for
    no benefit in the overwhelmingly common case -
    confirmed as a real, measured ~4.3x boot-time
    regression (see journal.md 2026-08-06) before this
    fix. `labelnum` is threaded in from the caller (either
    a static generator's own `label` counter or
    `compiler_state::labelnum` - both plain int refs, see
    this function's 5 call sites) since this is the one
    branching (not branch-free) codegen helper in this
    file's ASID_FIX machinery.

    Trashes I2 and dest itself (I2 is unused as target_pc
    or dest at every one of this function's call sites,
    confirmed by inspection, not assumed).
-------------------------------------------------*/

void mips3_device::generate_select_dispatch_key(drcuml_block &block, uml::parameter target_pc, uml::parameter dest, int &labelnum)
{
#ifndef MAME_SGI_DISPATCH_TWO_PAGE_CHECK
	// A/B fallback (2026-08-06, corrected 2026-08-07) - see the toggle's
	// own comment in mips3.h. Must match dispatch_page_writable()'s
	// #else branch exactly (lockstep registration/lookup contract) - that
	// branch is now the literal pre-2026-08-06 WRITE_ALLOWED-only test,
	// not the FETCH_ALLOWED-aware one (an earlier version of this
	// fallback used FETCH_ALLOWED here to match a since-superseded
	// version of dispatch_page_writable()'s fallback - correct in its
	// own right at the time, but journal.md 2026-08-07 00:14-00:52
	// measured that FETCH_ALLOWED-awareness, not page count, is what
	// costs ~4x boot time, so both sides moved back to WRITE_ALLOWED-only
	// together).
	UML_SHR(block, I3, target_pc, MIPS3_MIN_PAGE_SHIFT);                        // shr     i3,target_pc,12
	UML_LOAD(block, I3, (void *)vtlb_table(), I3, SIZE_DWORD, SCALE_x4);        // load    i3,[vtlb_table],i3,dword
	UML_TEST(block, I3, WRITE_ALLOWED);                                         // test    i3,WRITE_ALLOWED
	UML_MOV(block, dest, mem(&m_core->mode));                                   // mov     dest,[mode]           (shared key, default)
	UML_MOVc(block, COND_NZ, dest, mem(&m_core->dispatch_mode));                // mov     dest,[dispatch_mode],nz (private key, if page unsafe)
#else
	uml::code_label const cross_page = labelnum++;
	uml::code_label const combine = labelnum++;

	// REVERTED (2026-08-07): own page's indicator also needs FETCH_ALLOWED,
	// not just WRITE_ALLOWED - see page_shareable()'s comment in mips3.h.
	// A WRITE_ALLOWED-only own-page test reintroduced the exact failure
	// class this investigation is about, on the target's own page instead
	// of the delay page - reproduced a real livelock immediately.
	// own page: I2 = 0 iff shareable (FETCH_ALLOWED set, WRITE_ALLOWED clear)
	UML_SHR(block, I2, target_pc, MIPS3_MIN_PAGE_SHIFT);                        // shr     i2,target_pc,12
	UML_LOAD(block, I2, (void *)vtlb_table(), I2, SIZE_DWORD, SCALE_x4);        // load    i2,[vtlb_table],i2,dword
	UML_AND(block, I2, I2, FETCH_ALLOWED | WRITE_ALLOWED);                      // and     i2,i2,FETCH_ALLOWED|WRITE_ALLOWED
	UML_XOR(block, I2, I2, FETCH_ALLOWED);                                      // xor     i2,i2,FETCH_ALLOWED  (0 iff shareable)

	// cheap same-page test: no memory access, just the low bits already at
	// hand - only take the cross-page path when pc is the last slot in a page
	UML_AND(block, dest, target_pc, MIPS3_MIN_PAGE_MASK);                       // and     dest,target_pc,0xfff
	UML_CMP(block, dest, MIPS3_MIN_PAGE_MASK - 3);                              // cmp     dest,0xffc
	UML_JMPc(block, COND_E, cross_page);                                        // jmp     cross_page,E

	// common case: pc+4 is in the same page as pc - I2 already covers it,
	// nothing more to load
	UML_MOV(block, dest, I2);                                                   // mov     dest,i2
	UML_JMP(block, combine);                                                    // jmp     combine

	// rare case: delay-slot page is genuinely different, needs its own load
	UML_LABEL(block, cross_page);                                               // cross_page:
	UML_ADD(block, dest, target_pc, 4);                                         // add     dest,target_pc,4
	UML_SHR(block, dest, dest, MIPS3_MIN_PAGE_SHIFT);                           // shr     dest,dest,12
	UML_LOAD(block, dest, (void *)vtlb_table(), dest, SIZE_DWORD, SCALE_x4);    // load    dest,[vtlb_table],dest,dword
	UML_AND(block, dest, dest, FETCH_ALLOWED | WRITE_ALLOWED);                  // and     dest,dest,FETCH_ALLOWED|WRITE_ALLOWED
	UML_XOR(block, dest, dest, FETCH_ALLOWED);                                  // xor     dest,dest,FETCH_ALLOWED
	UML_OR(block, dest, dest, I2);                                              // or      dest,dest,i2

	// needs ASID key iff EITHER page's indicator is nonzero
	UML_LABEL(block, combine);                                                  // combine:
	UML_TEST(block, dest, dest);                                                // test    dest,dest
	UML_MOV(block, dest, mem(&m_core->mode));                                   // mov     dest,[mode]           (shared key, default)
	UML_MOVc(block, COND_NZ, dest, mem(&m_core->dispatch_mode));                // mov     dest,[dispatch_mode],nz (private key, if either page unsafe)
#endif
}
#endif


/*-------------------------------------------------
    generate_update_cycles - generate code to
    subtract cycles from the icount and generate
    an exception if out
-------------------------------------------------*/

void mips3_device::generate_update_cycles(drcuml_block &block, compiler_state &compiler, uml::parameter param, bool allow_exception, bool diag_tag, uint32_t diag_desc_pc)
{
	/* check software interrupts if pending */
	if (compiler.checksoftints)
	{
		uml::code_label skip;

		compiler.checksoftints = false;
		UML_AND(block, I0, CPR032(COP0_Cause), CPR032(COP0_Status));            // and     i0,[Cause],[Status]
		UML_AND(block, I0, I0, 0x0300);                                         // and     i0,i0,0x0300
		UML_JMPc(block, COND_Z, skip = compiler.labelnum++);                    // jmp     skip,Z
#ifdef MAME_DRC_BUGFIXES
		// Bug fix: this block only ever tested (Cause & Status & 0x0300),
		// unlike its sibling checkints block just below (and the
		// interpreter's own equivalent check in set_cop0_reg's COP0_Cause
		// case, mips3.cpp) which both additionally require SR_IE set and
		// neither SR_EXL nor SR_ERL set before actually delivering the
		// exception. Without these, a software interrupt could fire here
		// even while the kernel has interrupts explicitly disabled (e.g.
		// to protect a critical section) or while already inside another
		// exception - landing an interrupt somewhere the interpreter/real
		// hardware never would, exactly the kind of gap that could tear a
		// multi-field kernel update (as seen in the mutex_wake flags/next
		// -pointer inconsistency this session traced this bug to).
		UML_TEST(block, CPR032(COP0_Status), SR_IE);                            // test    [Status],SR_IE
		UML_JMPc(block, COND_Z, skip);                                          // jmp     skip,Z
		UML_TEST(block, CPR032(COP0_Status), SR_EXL | SR_ERL);                  // test    [Status],SR_EXL | SR_ERL
		UML_JMPc(block, COND_NZ, skip);                                         // jmp     skip,NZ
#endif
		// TEMPORARY probe (2026-07-27): delivery is about to happen (the
		// EXL/ERL guard above already passed) - I0/I1 haven't been set to
		// their delivery values yet, so a CALLC here is safe, the real
		// UML_MOV(I0,param)/UML_MOV(I1,cycles) right after freshly
		// overwrite whatever this probe's own call may have clobbered.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
		if (diag_tag)
		{
			UML_MOV(block, mem(&m_core->diag_status_store_pc), diag_desc_pc);  // mov     [diag_status_store_pc],diag_desc_pc
			UML_CALLC(block, cfunc_premature_interrupt_probe, this);           // callc   cfunc_premature_interrupt_probe
		}
#endif
		UML_MOV(block, I0, param);                                              // mov     i0,nextpc
		UML_MOV(block, I1, compiler.cycles);                                    // mov     i1,cycles
		UML_CALLH(block, *m_exception_norecover[EXCEPTION_INTERRUPT]);          // callh   interrupt_norecover
		UML_LABEL(block, skip);                                                 // skip:
	}

	/* check full interrupts if pending */
	if (compiler.checkints)
	{
		uml::code_label skip;

		compiler.checkints = false;
		UML_AND(block, I0, CPR032(COP0_Cause), CPR032(COP0_Status));            // and     i0,[Cause],[Status]
		UML_AND(block, I0, I0, 0xfc00);                                         // and     i0,i0,0xfc00
		UML_JMPc(block, COND_Z, skip = compiler.labelnum++);                    // jmp     skip,Z
		UML_TEST(block, CPR032(COP0_Status), SR_IE);                            // test    [Status],SR_IE
		UML_JMPc(block, COND_Z, skip);                                          // jmp     skip,Z
		UML_TEST(block, CPR032(COP0_Status), SR_EXL | SR_ERL);                  // test    [Status],SR_EXL | SR_ERL
		UML_JMPc(block, COND_NZ, skip);                                         // jmp     skip,NZ
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
		if (diag_tag)
		{
			UML_MOV(block, mem(&m_core->diag_status_store_pc), diag_desc_pc);  // mov     [diag_status_store_pc],diag_desc_pc
			UML_CALLC(block, cfunc_premature_interrupt_probe, this);           // callc   cfunc_premature_interrupt_probe
		}
#endif
		UML_MOV(block, I0, param);                                              // mov     i0,nextpc
		UML_MOV(block, I1, compiler.cycles);                                    // mov     i1,cycles
		UML_CALLH(block, *m_exception_norecover[EXCEPTION_INTERRUPT]);          // callh   interrupt_norecover
		UML_LABEL(block, skip);                                                 // skip:
	}

	/* account for cycles */
	if (compiler.cycles > 0)
	{
		UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), MAPVAR_CYCLES);    // sub     icount,icount,cycles
		UML_MAPVAR(block, MAPVAR_CYCLES, 0);                                        // mapvar  cycles,0
		if (allow_exception)
			UML_EXHc(block, COND_S, *m_out_of_cycles, param);
																					// exh     out_of_cycles,nextpc
	}
	compiler.cycles = 0;
}

#ifdef MAME_SGI_CHECKSUM_MISMATCH_DIAG_TASK84
// See MAME_SGI_CHECKSUM_MISMATCH_DIAG_TASK84's own comment above for
// why this exists. On a checksum mismatch at one of the watched PCs,
// re-reads the word through both the STALE pointer baked into the
// compiled block at compile time (`base`) and a FRESH m_prptr() lookup
// done right now - if the two pointers differ, or the fresh word
// matches `expected` while the stale one didn't, that's a stale
// physical-address resolution, not real self-modifying code.
struct checksum_mismatch_diag_ctx
{
	mips3_device *device;
	uint32_t pc;
	const void *base;
	uint32_t low_bits;
	uint32_t expected;
};

static checksum_mismatch_diag_ctx s_checksum_diag_pool[8];

void mips3_device::func_checksum_mismatch_diag(void *ctxptr)
{
	checksum_mismatch_diag_ctx &ctx = *(checksum_mismatch_diag_ctx *)ctxptr;
	uint32_t const stale_word = *(uint32_t const *)((uint8_t const *)ctx.base + ctx.low_bits);
	const void *fresh_base = m_prptr(ctx.pc);
	uint32_t const fresh_word = (fresh_base != nullptr)
		? *(uint32_t const *)((uint8_t const *)fresh_base + ctx.low_bits)
		: 0xffffffff;
	fprintf(stderr,
		"[CHECKSUM_MISMATCH_DIAG_TASK84] pc=%08x expected=%08x stale_base=%p stale_word=%08x "
		"fresh_base=%p fresh_word=%08x base_changed=%d fresh_matches_expected=%d\n",
		ctx.pc, ctx.expected, ctx.base, stale_word, fresh_base, fresh_word,
		(fresh_base != ctx.base), (fresh_word == ctx.expected));
	fflush(stderr);
}

static void cfunc_checksum_mismatch_diag(void *param)
{
	checksum_mismatch_diag_ctx *ctx = (checksum_mismatch_diag_ctx *)param;
	ctx->device->func_checksum_mismatch_diag(ctx);
}
#endif

/*-------------------------------------------------
    generate_checksum_block - generate code to
    validate a sequence of opcodes
-------------------------------------------------*/

void mips3_device::generate_checksum_block(drcuml_block &block, compiler_state &compiler, const opcode_desc *seqhead, const opcode_desc *seqlast, const opcode_desc *codelast)
{
	const opcode_desc *curdesc;
	if (m_drcuml->logging())
		block.append_comment("[Validation for %08X]", seqhead->pc);                // comment

	if (!(m_drcoptions & MIPS3DRC_STRICT_VERIFY) || seqhead->next() == nullptr)
	{
		// loose verify or single instruction: just compare and fail
		if (!seqhead->virtual_noop())
		{
			uint32_t sum = seqhead->opptr;
			const void *base = m_prptr(seqhead->physpc);
			uint32_t low_bits = (seqhead->physpc & (m_data_bits == 64 ? 4 : 0)) ^ m_dword_xor;
			UML_LOAD(block, I0, base, low_bits, SIZE_DWORD, SCALE_x1);         // load    i0,base,0,dword

#if defined(MAME_DRC_CHECKSUM_MISMATCH_DIAG) || defined(MAME_SGI_CHECKSUM_MISMATCH_DIAG_TASK84)
			// captured here, before the delay-slot branch below can
			// reassign the outer `base`/`low_bits` to point at a
			// different instruction entirely
			const void *diag_base = base;
			uint32_t diag_low_bits = low_bits;
#endif

			if (seqhead->delay.first() != nullptr
				&& !seqhead->delay.first()->virtual_noop()
				&& seqhead->physpc != seqhead->delay.first()->physpc)
			{
				uint32_t low_bits = (seqhead->delay.first()->physpc & (m_data_bits == 64 ? 4 : 0)) ^ m_dword_xor;
				base = m_prptr(seqhead->delay.first()->physpc);
				assert(base != nullptr);
				UML_LOAD(block, I1, base, low_bits, SIZE_DWORD, SCALE_x1);                 // load    i1,base,dword
				UML_ADD(block, I0, I0, I1);                     // add     i0,i0,i1

				sum += seqhead->delay.first()->opptr;
			}

			UML_CMP(block, I0, sum);                                    // cmp     i0,opptr
#ifdef MAME_SGI_CHECKSUM_MISMATCH_DIAG_TASK84
			if (seqhead->pc == 0x1007de40 || seqhead->pc == 0x1007de44 ||
				seqhead->pc == 0x1007decc || seqhead->pc == 0x1007df40 ||
				seqhead->pc == 0x1007df44 || seqhead->pc == 0x1007df48 ||
				seqhead->pc == 0x1007df50 || seqhead->pc == 0x1007dfd0 ||
				seqhead->pc == 0x1007dffc)
			{
				static int s_next_ctx = 0;
				checksum_mismatch_diag_ctx &ctx = s_checksum_diag_pool[s_next_ctx];
				s_next_ctx = (s_next_ctx + 1) % std::size(s_checksum_diag_pool);
				ctx.device = this;
				ctx.pc = seqhead->pc;
				ctx.base = diag_base;
				ctx.low_bits = diag_low_bits;
				ctx.expected = seqhead->opptr;
				UML_CALLCc(block, COND_NE, cfunc_checksum_mismatch_diag, &ctx);
			}
#endif
			UML_EXHc(block, COND_NE, *m_nocode, seqhead->epc());       // exne    nocode,seqhead->pc
		}
	}
	else
	{
		// full verification; sum up everything
#if 0
		for (curdesc = seqhead->next(); curdesc != seqlast->next(); curdesc = curdesc->next())
		{
			if (!curdesc->virtual_noop())
			{
				const void *base = m_prptr(seqhead->physpc);
				UML_LOAD(block, I0, base, m_dword_xor, SIZE_DWORD, SCALE_x1);     // load    i0,base,0,dword
				UML_CMP(block, I0, curdesc->opptr);                    // cmp     i0,opptr
				UML_EXHc(block, COND_NE, *m_nocode, epc(seqhead));   // exne    nocode,seqhead->pc
			}
		}
#else
		uint32_t sum = 0;
		const void *base = m_prptr(seqhead->physpc);
		const uint32_t data_bits_mask = (m_data_bits == 64 ? 4 : 0);
		const uint32_t last_physpc = codelast->physpc;
		uint32_t low_bits = (seqhead->physpc & data_bits_mask) ^ m_dword_xor;
		UML_LOAD(block, I0, base, low_bits, SIZE_DWORD, SCALE_x1);             // load    i0,base,0,dword
		sum += seqhead->opptr;
		if ((m_drcoptions & MIPS3DRC_EXTRA_INSTR_CHECK) && !codelast->virtual_noop() && last_physpc != seqhead->physpc)
		{
			base = m_prptr(last_physpc);
			assert(base != nullptr);
			low_bits = (last_physpc & data_bits_mask) ^ m_dword_xor;
			UML_LOAD(block, I1, base, low_bits, SIZE_DWORD, SCALE_x1);     // load    i1,base,dword
			UML_ADD(block, I0, I0, I1);                         // add     i0,i0,i1
			sum += codelast->opptr;
		}
		if (!(m_drcoptions & MIPS3DRC_EXTRA_INSTR_CHECK))
		{
			for (curdesc = seqhead->next(); curdesc != seqlast->next(); curdesc = curdesc->next())
			{
				if (!curdesc->virtual_noop())
				{
					base = m_prptr(curdesc->physpc);
					assert(base != nullptr);
					low_bits = (curdesc->physpc & data_bits_mask) ^ m_dword_xor;
					UML_LOAD(block, I1, base, low_bits, SIZE_DWORD, SCALE_x1);     // load    i1,base,dword
					UML_ADD(block, I0, I0, I1);                         // add     i0,i0,i1
					sum += curdesc->opptr;

					if (curdesc->delay.first() != nullptr
						&& !curdesc->delay.first()->virtual_noop()
						&& (curdesc == seqlast || (curdesc->next() != nullptr && curdesc->next()->physpc != curdesc->delay.first()->physpc)))
					{
						base = m_prptr(curdesc->delay.first()->physpc);
						assert(base != nullptr);
						low_bits = (curdesc->delay.first()->physpc & data_bits_mask) ^ m_dword_xor;
						UML_LOAD(block, I1, base, low_bits, SIZE_DWORD, SCALE_x1); // load    i1,base,dword
						UML_ADD(block, I0, I0, I1);                     // add     i0,i0,i1
						sum += curdesc->delay.first()->opptr;
					}
				}
			}
		}
		else
		{
			for (curdesc = seqhead->next(); curdesc != seqlast->next(); curdesc = curdesc->next())
			{
				if (!curdesc->virtual_noop())
				{
					// Skip the last if it was already included above
					if (curdesc->physpc != last_physpc)
					{
						base = m_prptr(curdesc->physpc);
						assert(base != nullptr);
						low_bits = (curdesc->physpc & data_bits_mask) ^ m_dword_xor;
						UML_LOAD(block, I1, base, low_bits, SIZE_DWORD, SCALE_x1);     // load    i1,base,dword
						UML_ADD(block, I0, I0, I1);                         // add     i0,i0,i1
						sum += curdesc->opptr;
					}

					if (curdesc->delay.first() != nullptr
						&& !curdesc->delay.first()->virtual_noop()
						&& (curdesc == seqlast || (curdesc->next() != nullptr && curdesc->next()->physpc != curdesc->delay.first()->physpc)))
					{
						base = m_prptr(curdesc->delay.first()->physpc);
						assert(base != nullptr);
						low_bits = (curdesc->delay.first()->physpc & data_bits_mask) ^ m_dword_xor;
						UML_LOAD(block, I1, base, low_bits, SIZE_DWORD, SCALE_x1); // load    i1,base,dword
						UML_ADD(block, I0, I0, I1);                     // add     i0,i0,i1
						sum += curdesc->delay.first()->opptr;
					}
				}
			}
		}
		UML_CMP(block, I0, sum);                                            // cmp     i0,sum
		UML_EXHc(block, COND_NE, *m_nocode, seqhead->epc());           // exne    nocode,seqhead->pc
		if (DEBUG_STRICT_VERIFY)
		{
			// This code will do additional checks on the last instruction and last delay slot and indicate if the check failed
			uml::code_label check_second = compiler.labelnum++;
			uml::code_label check_failed = compiler.labelnum++;
			uml::code_label check_passed = compiler.labelnum++;
			// Check the last instruction
			if (!codelast->virtual_noop() && (last_physpc != seqhead->physpc))
			{
				base = m_prptr(last_physpc);
				assert(base != nullptr);
				low_bits = (last_physpc & (m_data_bits == 64 ? 4 : 0)) ^ m_dword_xor;
				UML_LOAD(block, I0, base, low_bits, SIZE_DWORD, SCALE_x1);     // load    i1,base,dword
				sum = codelast->opptr;
				UML_CMP(block, I0, sum);                                            // cmp     i0,sum
				UML_JMPc(block, COND_E, check_second);
				static const char text[] = "Last instr validation fail seq: %08X end: %08x\n";
				UML_DMOV(block, mem(&m_core->format), (uintptr_t)text);          // mov     [format],text
				UML_MOV(block, mem(&m_core->arg0), seqhead->pc);
				UML_MOV(block, mem(&m_core->arg1), last_physpc);              // mov     [arg0],desc->pc
				UML_CALLC(block, cfunc_printf_debug, this);                            // callc   printf_debug
				//UML_CALLC(block, cfunc_debug_break, this);
				// Skip delay slot check
				UML_JMP(block, check_failed);
				// Check the last instruction delay slot
				UML_LABEL(block, check_second);
				if (codelast->delay.first() != nullptr && !codelast->delay.first()->virtual_noop()
					&& last_physpc != seqhead->physpc)
				{
					base = m_prptr(codelast->delay.first()->physpc);
					assert(base != nullptr);
					low_bits = (codelast->delay.first()->physpc & (m_data_bits == 64 ? 4 : 0)) ^ m_dword_xor;
					UML_LOAD(block, I0, base, low_bits, SIZE_DWORD, SCALE_x1); // load    i1,base,dword
					sum = codelast->delay.first()->opptr;
					UML_CMP(block, I0, sum);                                            // cmp     i0,sum
					UML_JMPc(block, COND_E, check_passed);
					static const char text[] = "Last delay slot validation fail seq: %08X end: %08x\n";
					UML_DMOV(block, mem(&m_core->format), (uintptr_t)text);          // mov     [format],text
					UML_MOV(block, mem(&m_core->arg0), seqhead->pc);
					UML_MOV(block, mem(&m_core->arg1), last_physpc);              // mov     [arg0],desc->pc
					UML_CALLC(block, cfunc_printf_debug, this);                            // callc   printf_debug
					//UML_CALLC(block, cfunc_debug_break, this);
					UML_JMP(block, check_failed);
				}
			}
			UML_JMP(block, check_passed);
			UML_LABEL(block, check_failed);
			UML_EXH(block, *m_nocode, seqhead->epc());           // exne    nocode,seqhead->pc
			UML_LABEL(block, check_passed);
		}
#endif
	}
}


/*-------------------------------------------------
    generate_sequence_instruction - generate code
    for a single instruction in a sequence
-------------------------------------------------*/

void mips3_device::generate_sequence_instruction(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
#ifdef MAME_SGI_TASK84_V0_SHADOW_DIAG
	// "Before" half of the shadow-divergence check - see the macro's own
	// comment (near its #define) for the full design. Runs first, before
	// any of this desc's own early-return paths (unmapped/page-fault/
	// tlb-mismatch), so it always sees whatever the shadow held coming
	// INTO this instruction boundary.
	//
	// DELIBERATELY label-free (UML_CALLCc, not a manual UML_JMPc/
	// UML_LABEL pair) - confirmed live 2026-09-07 that a manual
	// compiler.labelnum++ here corrupts control flow site-wide: this
	// function is called recursively for delay-slot instructions via
	// generate_delay_slot_and_branch's `compiler_temp` (mips3.h:1183-1191's
	// compiler_state, plain-copied, `labelnum` a plain value not a shared
	// counter) - a hazard this codebase already has documented, careful
	// bridging code for at exactly one other call site
	// (generate_select_dispatch_key via compiler_temp.labelnum, see the
	// comment above generate_delay_slot_and_branch's own use of it) but
	// which a naive compiler.labelnum++ here does NOT get for free. The
	// result was duplicate label numbers within the same compiled block
	// for effectively every branch-with-delay-slot instruction in the
	// entire guest (i.e. almost everywhere), observed live as both a
	// storm of spurious [V0_SHADOW_DIVERGENCE] hits (500-hit cap reached
	// in 5 seconds) AND cc1 dropping to a 100% failure rate (30/30) -
	// this was corrupting real control flow, not just noisy diagnostics.
	// UML_CALLCc needs no label at all, sidestepping the hazard entirely -
	// the same pattern already proven safe elsewhere in this file (e.g.
	// cfunc_task84_reload_mismatch's site, cfunc_task84_tlb_mismatch_diag's
	// site 2). Plain UML_MOV/UML_DMOV do not clobber the flags UML_DCMP
	// just set (also already-established elsewhere in this file), so
	// stashing the context unconditionally before the conditional call is
	// safe and cheap - only the CALLCc itself is actually conditional.
	UML_DCMP(block, R64(2), mem(&m_core->diag_v0_shadow));
	UML_MOV(block, mem(&m_core->diag_v0_divergence_pc), desc->pc);
	UML_DMOV(block, mem(&m_core->diag_v0_divergence_current), R64(2));
	UML_CALLCc(block, COND_NE, cfunc_v0_shadow_diag, this);
#endif
#ifdef MAME_SGI_TASK84_V0_S3_DIAG
	if (desc->pc == 0x1007df50)
	{
		save_fast_iregs(block);
		UML_CALLC(block, cfunc_task84_v0_diag, this);
	}
	else if (desc->pc == 0x1007dffc)
	{
		save_fast_iregs(block);
		UML_CALLC(block, cfunc_task84_s3_check_diag, this);
	}
#elif defined(MAME_SGI_TASK84_SAVEFASTIREGS_ONLY)
	if (desc->pc == 0x1007df50 || desc->pc == 0x1007dffc)
	{
		save_fast_iregs(block);
	}
#elif defined(MAME_SGI_TASK84_INERT_ONLY)
	if (desc->pc == 0x1007df50 || desc->pc == 0x1007dffc)
	{
		UML_DMOV(block, mem(&m_core->diag_task84_inert_scratch), 0x1234);
	}
#elif defined(MAME_SGI_TASK84_POISON_R2)
	if (desc->pc == 0x1007df50)
	{
		UML_DMOV(block, mem(&m_core->r[2]), 0xdeadbeef);
	}
#elif defined(MAME_SGI_TASK84_WINDOW_TRACE)
	// Disarm at 0x1007df68 (the `beq $v0,$s4,...` right after $v0 gets
	// consumed - straight-line code from 0x1007df50, no branches in
	// between, so always reached on the very next pass), NOT at
	// 0x1007dffc (the loop-exit check) - the original design armed at
	// 0x1007df50 but only disarmed at 0x1007dffc, which is reached rarely
	// (only when $s3 hits zero), leaving the window open across many
	// unrelated loop passes where r2 legitimately changes for ordinary
	// reasons having nothing to do with this bug. Confirmed the hard way:
	// 20 iterations produced 2.3M+ gated events and a 367MB log before
	// this was caught (see the log function's own comment).
	if (desc->pc == 0x1007df50)
	{
		UML_MOV(block, mem(&m_core->diag_task84_armed), 1);
	}
	else if (desc->pc == 0x1007df68)
	{
		UML_MOV(block, mem(&m_core->diag_task84_armed), 0);
	}
	// One-shot: log $t9 (r25) right before the jalr at 0x1007df48 compiles -
	// by codegen/runtime order this is exactly the value the preceding
	// `lw $t9,-0x7998($gp)` at 0x1007df44 just loaded, i.e. cpp_get_token()'s
	// real guest address. Only need one sample (it's a compile-time-constant
	// call target, never changes across invocations) - gated by a static
	// bool inside the log function itself to avoid repeating this on every
	// one of the thousands of loop passes.
	else if (desc->pc == 0x1007df48)
	{
		UML_CALLC(block, cfunc_task84_t9_once_diag, this);
	}
#endif
#ifdef MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG
	// Precisely brackets ONE cpp_get_token() invocation's save_fast_iregs()/
	// load_fast_iregs() call activity, in guest-instruction order (not
	// wall-clock) - see MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG's own comment
	// near MAME_SGI_TASK84_SAVELOAD_BALANCE_DIAG for why the wall-clock/gdb-
	// polling approach (tools/flush_saveload_sample.sh) was too noisy to
	// trust. 0x1007df48/0x1007df50 are parse_answer's own call/return sites
	// around cpp_get_token (jalr $t9 / the instruction right after its
	// delay slot) - the same precise, previously-proven bracket
	// MAME_SGI_TASK84_WINDOW_TRACE already used for a different purpose.
	if (desc->pc == 0x1007df48)
	{
		UML_CALLC(block, cfunc_task84_saveload_bracket_entry, this);
	}
	else if (desc->pc == 0x1007df50)
	{
		UML_CALLC(block, cfunc_task84_saveload_bracket_exit, this);
	}
#endif
#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
	if (desc->pc == MAME_SGI_LWC1_DELAYSLOT_PROBE_PC)
	{
		lwc1_probe_log("[LWC1_PROBE] generate_sequence_instruction ENTRY pc=0x%08x opptr=0x%08x "
			"in_delay_slot=%d virtual_noop=%d invalid_opcode=%d compiler_page_fault=%d "
			"compiler_unmapped=%d validate_tlb=%d\n",
			desc->pc, desc->opptr, desc->in_delay_slot(), desc->virtual_noop(), desc->invalid_opcode(),
			desc->compiler_page_fault(), desc->compiler_unmapped(), desc->validate_tlb());
	}
#endif
	/* add an entry for the log */
	if (m_drcuml->logging() && !desc->virtual_noop())
		log_add_disasm_comment(block, desc->pc, desc->opptr);

	/* set the PC map variable */
	const offs_t expc = desc->epc();
	UML_MAPVAR(block, MAPVAR_PC, expc);                                             // mapvar  PC,expc

	/* accumulate total cycles */
	compiler.cycles += desc->cycles;

	/* is this a hotspot? */
	for (int hotnum = 0; hotnum < MIPS3_MAX_HOTSPOTS; hotnum++)
		if (m_hotspot[hotnum].pc != 0 && desc->pc == m_hotspot[hotnum].pc && desc->opptr == m_hotspot[hotnum].opcode)
		{
			compiler.cycles += m_hotspot[hotnum].cycles;
			break;
		}

	/* update the icount map variable */
	UML_MAPVAR(block, MAPVAR_CYCLES, compiler.cycles);                             // mapvar  CYCLES,compiler.cycles

	/* if we want a probe, add it here */
	if (desc->pc == PROBE_ADDRESS)
	{
		UML_MOV(block, mem(&m_core->pc), desc->pc);                              // mov     [pc],desc->pc
		UML_CALLC(block, cfunc_printf_probe, this);                                // callc   cfunc_printf_probe,mips3
	}

	/* if we are debugging, call the debugger */
	if (debugger_enabled())
	{
		UML_MOV(block, mem(&m_core->pc), desc->pc);                              // mov     [pc],desc->pc
		save_fast_iregs(block);
		UML_DEBUG(block, desc->pc);                                         // debug   desc->pc
	}

	/* if we hit an unmapped address, fatal error */
	if (desc->compiler_unmapped())
	{
		UML_MOV(block, mem(&m_core->pc), desc->pc);                              // mov     [pc],desc->pc
		save_fast_iregs(block);
		UML_EXIT(block, EXECUTE_UNMAPPED_CODE);                             // exit    EXECUTE_UNMAPPED_CODE
	}

	/* if we hit a compiler page fault, it's just like a TLB mismatch */
	if (desc->compiler_page_fault())
	{
		if (PRINTF_MMU)
		{
			static const char text[] = "Compiler page fault @ %08X\n";
			UML_MOV(block, mem(&m_core->format), (uintptr_t)text);          // mov     [format],text
			UML_MOV(block, mem(&m_core->arg0), desc->pc);              // mov     [arg0],desc->pc
			UML_CALLC(block, cfunc_printf_debug, this);                            // callc   printf_debug
		}
		// Measured 2026-08-23: this stub is the origin of 96.2% of all
		// arrivals at the shared handler's skip branch - by far the
		// dominant driver of TLB-mismatch recompiles. See that handler
		// for what now happens there instead of a blind recompile.
#ifdef MAME_SGI_TASK84_TLB_MISMATCH_DIAG
		if (desc->pc >= 0x1007de00 && desc->pc <= 0x1007e100)
		{
			UML_MOV(block, mem(&m_core->arg0), desc->pc);
			UML_MOV(block, mem(&m_core->arg1), 1);           // site 1: compiler_page_fault
			UML_CALLC(block, cfunc_task84_tlb_mismatch_diag, this);
		}
#endif
		UML_EXH(block, *m_tlb_mismatch, 0);                      // exh     tlb_mismatch,0
		// Unconditional tlb exception, no point going further
		return;
	}

	/* validate our TLB entry at this PC; if we fail, we need to handle it */
	if (desc->validate_tlb() && (desc->pc < 0x80000000 || desc->pc >= 0xc0000000))
	{
		const vtlb_entry *tlbtable = vtlb_table();

		/* if we currently have a valid TLB read entry, we just verify */
		if (tlbtable[desc->pc >> 12] & FETCH_ALLOWED)
		{
			if (PRINTF_MMU)
			{
				static const char text[] = "Checking TLB at @ %08X\n";
				UML_MOV(block, mem(&m_core->format), (uintptr_t)text);      // mov     [format],text
				UML_MOV(block, mem(&m_core->arg0), desc->pc);          // mov     [arg0],desc->pc
				UML_CALLC(block, cfunc_printf_debug, this);                        // callc   printf_debug
			}
#ifdef MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG
			// See diag_0fac5ffc_compile_gen's comment in mips3.h.
			// Unconditional (not gated on the CMP's NE flag below) -
			// runs its own independent C-side re-verification every
			// single dispatch through this site, rather than depending
			// on the JIT'd CMP's flags surviving a call (they wouldn't -
			// a C function call is not flag-preserving in this ABI).
			if (desc->pc == 0x0fac5ffc)
			{
				UML_MOV(block, mem(&m_core->arg0), m_core->diag_0fac5ffc_compile_gen);  // this instance's own generation, baked
				UML_MOV(block, mem(&m_core->arg1), tlbtable[desc->pc >> 12]);           // compile-time-expected value, baked
				UML_CALLC(block, cfunc_0fac5ffc_anomaly_diag, this);
			}
#endif
#ifdef MAME_SGI_VALIDATE_TLB_MISMATCH_CLASS_DIAG
			// Unconditional for every pc, not just 0x0fac5ffc - see the
			// macro's own comment above.
			UML_MOV(block, mem(&m_core->arg0), desc->pc);
			UML_MOV(block, mem(&m_core->arg1), tlbtable[desc->pc >> 12]);
			UML_CALLC(block, cfunc_validate_tlb_mismatch_class_diag, this);
#endif
			// NOTE (2026-08-23): an earlier version of the refresh fix
			// stashed tlbtable[pc>>12] into m_core here so the shared
			// handler could compare it against the live entry. That
			// compare could never be true: the EXHc below fires only
			// when live != baked, and the handler re-reads that same
			// entry with nothing running in between, so the two can only
			// ever differ. It also cost a store on every execution of
			// this guard, not just on mismatches. Removed.
			UML_LOAD(block, I0, &tlbtable[desc->pc >> 12], 0, SIZE_DWORD, SCALE_x4);        // load i0,tlbtable[desc->pc >> 12],0,dword
			UML_CMP(block, I0, tlbtable[desc->pc >> 12]);                   // cmp     i0,*tlbentry
#ifdef MAME_SGI_TASK84_TLB_MISMATCH_DIAG
			if (desc->pc >= 0x1007de00 && desc->pc <= 0x1007e100)
			{
				UML_MOV(block, mem(&m_core->arg0), desc->pc);
				UML_MOV(block, mem(&m_core->arg1), 2);       // site 2: validate_tlb fast-path mismatch
				UML_CALLCc(block, COND_NE, cfunc_task84_tlb_mismatch_diag, this);
			}
#endif
			UML_EXHc(block, COND_NE, *m_tlb_mismatch, 0);            // exh     tlb_mismatch,0,NE
#ifdef MAME_SGI_VALIDATE_TLB_FREQ_DIAG
			// TEMPORARY diagnostic (2026-08-07) - see diag_validate_tlb_total's
			// comment in mips3.h. Counts real executions of this fast-path
			// check, not compile events - this is the per-dispatch-ish
			// runtime cost being sized, paired with the jmpdest counters.
			UML_ADD(block, mem(&m_core->diag_validate_tlb_total), mem(&m_core->diag_validate_tlb_total), 1);
#endif
		}

		/* otherwise, we generate an unconditional exception */
		else
		{
			if (PRINTF_MMU)
			{
				static const char text[] = "No valid TLB @ %08X\n";
				UML_MOV(block, mem(&m_core->format), (uintptr_t)text);      // mov     [format],text
				UML_MOV(block, mem(&m_core->arg0), desc->pc);          // mov     [arg0],desc->pc
				UML_CALLC(block, cfunc_printf_debug, this);                        // callc   printf_debug
			}
			// Measured 2026-08-23: unreachable in practice. A desc whose
			// fetch fails to translate gets compiler_page_fault() set by
			// the frontend, and that is tested earlier in this function
			// and returns, so this branch never actually dispatches.
			UML_EXH(block, *m_tlb_mismatch, 0);                  // exh     tlb_mismatch,0
			// Unconditional tlb exception, no point going further
			return;
		}
	}

#ifdef MAME_SGI_MXCSR_DESYNC_DIAG
	// See MAME_SGI_MXCSR_DESYNC_DIAG's own comment above (near
	// s_mxcsr_desync_expected_rc) for the full rationale. Cheap inline
	// compare on every instruction boundary; only escalates to a CALLC
	// when real_rc != expected_rc.
	{
		uml::code_label const mxcsr_desync_skip = compiler.labelnum++;
		UML_GETFMOD(block, I0);                                              // getfmod i0                  (real live MXCSR)
		UML_SHR(block, I0, I0, 13);                                          // shr     i0,i0,13
		UML_AND(block, I0, I0, 3);                                           // and     i0,i0,3              (real RC bits)
		UML_AND(block, I1, CCR132(31), 3);                                   // and     i1,ccr1[31],3        (guest RM)
		UML_LOAD(block, I2, &s_mxcsr_desync_expected_rc[0], I1, SIZE_BYTE, SCALE_x1); // load i2,expected_rc,i1,byte
		UML_CMP(block, I0, I2);                                              // cmp     i0,i2
		UML_JMPc(block, COND_Z, mxcsr_desync_skip);                          // jmp     skip,Z
		UML_MOV(block, mem(&m_core->diag_mxcsr_desync_pc), desc->pc);              // mov     [diag_pc],desc->pc
		UML_MOV(block, mem(&m_core->diag_mxcsr_desync_guest_rm), I1);              // mov     [diag_rm],i1
		UML_MOV(block, mem(&m_core->diag_mxcsr_desync_real_rc), I0);               // mov     [diag_real_rc],i0
		UML_CALLC(block, cfunc_mxcsr_desync_diag, this);                          // callc   mxcsr_desync_diag,mips3
		UML_LABEL(block, mxcsr_desync_skip);                                 // skip:
	}
#endif

#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
	if (desc->pc == MAME_SGI_LWC1_DELAYSLOT_PROBE_PC)
	{
		lwc1_probe_log("[LWC1_PROBE] generate_sequence_instruction reached dispatch pc=0x%08x "
			"(survived compiler_unmapped/compiler_page_fault/tlb-mismatch early returns)\n", desc->pc);
	}
#endif
	if (desc->invalid_opcode())
	{
		// if this is an invalid opcode, generate the exception now
		UML_EXH(block, *m_exception[EXCEPTION_INVALIDOP], 0);    // exh     invalidop,0
	}
	else if (!desc->virtual_noop())
	{
		// otherwise, unless this is a virtual no-op, it's a regular instruction
		// compile the instruction
		bool const opcode_result = generate_opcode(block, compiler, desc);
#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
		if (desc->pc == MAME_SGI_LWC1_DELAYSLOT_PROBE_PC)
		{
			lwc1_probe_log("[LWC1_PROBE] generate_opcode returned %d for pc=0x%08x\n", opcode_result, desc->pc);
		}
#endif
		if (!opcode_result)
		{
			UML_MOV(block, mem(&m_core->pc), desc->pc);                          // mov     [pc],desc->pc
			UML_MOV(block, mem(&m_core->arg0), desc->opptr);      // mov     [arg0],desc->opptr
			UML_CALLC(block, cfunc_unimplemented, this);                           // callc   cfunc_unimplemented
		}
	}
#ifdef MAME_SGI_TASK84_V0_SHADOW_DIAG
	// "After" half - unconditionally re-mirror the (possibly just-changed)
	// live value, so the next instruction's "before" check has an
	// up-to-date shadow. Not reached for the compiler_page_fault/
	// tlb-mismatch-invalid early `return;` paths above (rare, exceptional
	// control flow, not normal steady-state execution) - an accepted,
	// disclosed gap rather than a claimed one, see the macro's own comment.
	UML_DMOV(block, mem(&m_core->diag_v0_shadow), R64(2));
#endif
}


/*------------------------------------------------------------------
    generate_delay_slot_and_branch
------------------------------------------------------------------*/

void mips3_device::generate_delay_slot_and_branch(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, uint8_t linkreg)
{
	compiler_state compiler_temp(compiler);
	const uint32_t op = desc->opptr;

	/* fetch the target register if dynamic, in case it is modified by the delay slot */
	if (desc->targetpc == BRANCH_TARGET_DYNAMIC)
	{
		UML_MOV(block, mem(&m_core->jmpdest), R32(RSREG));                 // mov     [jmpdest],<rsreg>

#ifdef MAME_SGI_TASK84_T9_GOT_DIAG
		if (desc->pc == 0x1007df48)
		{
			UML_CMP(block, mem(&m_core->jmpdest), 0x10076008);
			UML_CALLCc(block, COND_NE, cfunc_t9_got_diag, this);
		}
#endif

#ifdef MAME_SGI_TASK84_TOKEN_RESULT_DIAG
		// See the macro's own comment near the top of this file.
		// WIDENED (2026-09-09, same day) from three exact hardcoded
		// addresses (found via disassembly, but which produced zero hits
		// across 7 straight runs before finally firing 131 times on an
		// 8th - a real, unexplained gap, not trusted) to any `jr $ra`
		// (RSREG==31, linkreg==0 i.e. not a call) inside cpp_get_token's
		// function range - more robust against a missed/misidentified
		// return path than three specific PCs. $v0/R64(2) here is
		// whatever the function's own body last set it to.
		if (linkreg == 0 && RSREG == 31 && desc->pc == 0x10076134)
		{
			UML_DMOV(block, mem(&m_core->diag_token_result_v0), R64(2));
			UML_CALLC(block, cfunc_token_result_diag, this);
		}
#endif

#ifdef MAME_SGI_TASK84_PID_CALIBRATION_DIAG
			if (linkreg == 0 && RSREG == 31 && desc->pc == 0x10076134)
			{
				UML_CALLC(block, cfunc_pid_calibration_diag, this);
			}
#endif
	}

	/* set the link if needed -- before the delay slot */
	if (linkreg != 0)
	{
		UML_DMOV(block, R64(linkreg), (int32_t)(desc->pc + 8));                   // dmov    <linkreg>,desc->pc + 8

#ifdef MAME_SGI_TASK84_CALL_TRACE_DIAG
		// See the macro's own comment near the top of this file. Range
		// check done as a single unsigned comparison ((target - LO) <
		// (HI - LO)) rather than two branches/labels - avoids the
		// compiler/compiler_temp labelnum hazard this file has been
		// bitten by before (see generate_sequence_instruction's own
		// v0-shadow-diag comment), and matches this file's established
		// "UML_CALLCc over manual labels" convention. I2 is free here -
		// nothing in this function has touched it yet at this point.
		if (desc->targetpc != BRANCH_TARGET_DYNAMIC)
		{
			if (desc->targetpc >= 0x10000000 && desc->targetpc < 0x80000000)
			{
				UML_MOV(block, mem(&m_core->diag_call_trace_target), desc->targetpc);
				UML_CALLC(block, cfunc_call_trace_diag, this);
			}
		}
		else
		{
			UML_MOV(block, mem(&m_core->diag_call_trace_target), mem(&m_core->jmpdest));
			UML_SUB(block, I2, mem(&m_core->jmpdest), 0x10000000);
			UML_CMP(block, I2, 0x70000000);
			UML_CALLCc(block, COND_B, cfunc_call_trace_diag, this);
		}
#endif

#ifdef MAME_SGI_TASK84_SOURCE_TEXT_DIAG
		// See the macro's own comment near the top of this file. This is
		// the loop's own repeated cpp_get_token call site inside
		// parse_answer - the `jalr $t9` itself at 0x1007df48 (NOT the
		// preceding `lw $t9,-0x7998($gp)` at 0x1007df44 - desc->pc for a
		// JALR/JR-class instruction is that instruction's own address,
		// matching the already-verified-correct pattern
		// CPPERROR_CALLSITE_DIAG uses for its own jalr at 0x1007dec4;
		// caught this addressing mistake before rebuilding, not after).
		// Fires once per token fetched, every iteration, both outcomes.
		// $s0 (register 16) is pfile; $s3 (register 19) is the loop's
		// own iteration/acount counter, captured here (BEFORE the call)
		// since it's callee-saved and unaffected by a well-behaved
		// cpp_get_token.
		if (desc->pc == 0x1007df48)
		{
			UML_DMOV(block, mem(&m_core->diag_srctext_s0), R64(16));
			UML_DMOV(block, mem(&m_core->diag_srctext_s3), R64(19));
			UML_CALLC(block, cfunc_source_text_diag, this);
		}
#endif

#ifdef MAME_SGI_TASK84_CPPERROR_CALLSITE_DIAG
		// See the macro's own comment near the top of this file. cpp_error
		// is called indirectly (jalr $t9 via GOT, same as cpp_get_token -
		// confirmed via a whole-binary scan finding zero direct `jal`
		// references to it), so this can only be caught on the
		// DYNAMIC-target path, by comparing the live jmpdest against
		// cpp_error's known address (0x10086ab0).
		if (desc->targetpc == BRANCH_TARGET_DYNAMIC)
		{
			UML_CMP(block, mem(&m_core->jmpdest), 0x10086ab0);
			UML_MOVc(block, COND_E, mem(&m_core->diag_cpperror_callsite_pc), desc->pc);
			// Direct empirical resolution (2026-09-10) of the $s3
			// contradiction: capture BOTH the actual message-pointer
			// argument ($a2, register 6 - distinguishes which of the
			// three cpp_error() call sites in real source this
			// particular firing corresponds to) AND $s3 itself, AT THE
			// REAL CALL, rather than continuing to infer either from
			// static disassembly. Unconditional capture (cheap, no
			// side effect) - only the CALLC below is gated to real hits.
			UML_DMOV(block, mem(&m_core->diag_cpperror_a2), R64(6));
			UML_DMOV(block, mem(&m_core->diag_cpperror_s3), R64(19));
			UML_CALLCc(block, COND_E, cfunc_cpperror_callsite_diag, this);
		}
#endif
	}

	/* compile the delay slot using temporary compiler state */
	assert(desc->delay.first() != nullptr);
	generate_sequence_instruction(block, compiler_temp, desc->delay.first());       // <next instruction>

	/* update the cycles and jump through the hash table to the target */
	if (desc->targetpc != BRANCH_TARGET_DYNAMIC)
	{
		generate_update_cycles(block, compiler_temp, desc->targetpc, true); // <subtract cycles>
		if (!(m_drcoptions & MIPS3DRC_DISABLE_INTRABLOCK) && desc->intrablock_branch())
		{
			UML_JMP(block, desc->targetpc | 0x80000000);                            // jmp     desc->targetpc | 0x80000000
		}
		else
		{
#ifdef MAME_DRC_BUGFIXES
			// Bug fix: this was a bare m_core->mode read, baking the mode
			// used for the hash *lookup itself* into a compile-time
			// snapshot - not just an optimization like the safe
			// same-sequence case elsewhere (generate_checksum_block's
			// "assume the same mode" path), because a jal/branch target
			// reached via this hashjmp (e.g. a shared C function like
			// tlbmiss(), called from multiple different real modes over
			// the life of the emulation) gets its OWN separately-compiled,
			// per-mode instance cached under whatever mode this exact call
			// site last baked in. Confirmed live as the root cause of a
			// real, reproducible PANIC: KERNEL FAULT: tlbmiss()'s own PC
			// was compiled once under MODE_SUPER (from some earlier,
			// legitimately non-kernel TLB miss) and once under
			// MODE_KERNEL; when a later, genuinely kernel-mode TLB miss's
			// call into tlbmiss() baked in MODE_SUPER at this call site,
			// every memory access inside that cached instance kept using
			// the supervisor-mode accessor (which - unlike kernel mode -
			// checks and rejects addresses outside 0xC0000000-0xDFFFFFFF),
			// incorrectly rejecting a legitimate KSEG2 kernel exception
			// stack address. Fixed by reading the real, current mode live
			// (mem(&m_core->mode)) so the hash lookup - and any fresh
			// compile it triggers - always matches the mode this call is
			// actually happening under, not a stale snapshot from whenever
			// this call site was first compiled.
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
			UML_HASHJMP(block, live_dispatch_key(desc->targetpc), desc->targetpc, *m_nocode);       // hashjmp <mode>,desc->targetpc,nocode
#else
			UML_HASHJMP(block, mem(&m_core->dispatch_mode), desc->targetpc, *m_nocode);       // hashjmp <mode>,desc->targetpc,nocode
#endif
#else
			UML_HASHJMP(block, m_core->dispatch_mode, desc->targetpc, *m_nocode);            // hashjmp <mode>,desc->targetpc,nocode
#endif
		}
	}
	else
	{
		generate_update_cycles(block, compiler_temp, uml::mem(&m_core->jmpdest), true); // <subtract cycles>
#ifdef MAME_DRC_BUGFIXES
		// Bug fix: same class as the branch case above - see its comment.
		// This is also the main JALR/JR entry point into shared library
		// code (indirect calls through a function pointer resolved by
		// the dynamic linker) - the one dispatch site where getting the
		// MAME_SGI_DISPATCH_MODE_ASID_FIX value-2 refinement right
		// matters most, since a target unknown until runtime needs a
		// genuine live writability check (generate_select_dispatch_key),
		// not something bakeable at compile time.
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
		{
			// bridge: two things to get right here, both found the hard way
			// (first attempt crashed with asmjit "LabelAlreadyBound" on the
			// very first jmpdest dispatch). (1) uml::code_label is u32-based,
			// but generate_select_dispatch_key takes a plain int& to also
			// serve its two static-generator call sites (static_generate_
			// entry_point/static_generate_exception), which thread a raw int
			// label counter, not a code_label - no single reference type
			// binds to both. (2) THIS specific call site is inside the
			// `compiler_temp` branch (compiler_temp is a *separate*
			// compiler_state, copy-constructed from `compiler` at this
			// function's entry and only merged back into `compiler.labelnum`
			// at its very end, line ~5013) - generate_update_cycles() just
			// above already advanced compiler_temp's own label counter, not
			// compiler's. Bridging from `compiler.labelnum` (stale here,
			// unrelated to what's actually been allocated in this branch)
			// handed out label numbers compiler_temp had already used
			// internally - a real collision, not a hypothetical one. Must
			// bridge from compiler_temp.labelnum instead, matching what
			// generate_update_cycles(block, compiler_temp, ...) above
			// already does.
			int labelnum_tmp = compiler_temp.labelnum;
			generate_select_dispatch_key(block, uml::mem(&m_core->jmpdest), uml::I3, labelnum_tmp);
			compiler_temp.labelnum = labelnum_tmp;
		}
#ifdef MAME_SGI_JMPDEST_SAMEPAGE_DIAG
		// TEMPORARY diagnostic (2026-08-06, disabled by default 2026-08-06) -
		// see diag_jmpdest_dispatch_total's comment in mips3.h. Duplicates
		// generate_select_dispatch_key()'s own cheap same-page test (2 ops,
		// no memory access) purely to count, rather than touching that
		// function's signature/logic for a measurement that only applies to
		// this one call site. Already used once to get a real number
		// (65,141,818 dispatches, 0.0158% cross-page, journal.md 2026-08-06)
		// - off by default since it's still per-dispatch cost on the
		// hottest path in the system; re-enable if that number needs
		// re-checking after a future change (e.g. the epoch/host-fault
		// redesign).
		UML_AND(block, I2, mem(&m_core->jmpdest), MIPS3_MIN_PAGE_MASK);         // and     i2,[jmpdest],0xfff
		UML_CMP(block, I2, MIPS3_MIN_PAGE_MASK - 3);                            // cmp     i2,0xffc
		UML_MOV(block, I2, 0);                                                  // mov     i2,0
		UML_MOVc(block, COND_E, I2, 1);                                         // mov     i2,1,E
		UML_ADD(block, mem(&m_core->diag_jmpdest_dispatch_crosspage), mem(&m_core->diag_jmpdest_dispatch_crosspage), I2);
		UML_ADD(block, mem(&m_core->diag_jmpdest_dispatch_total), mem(&m_core->diag_jmpdest_dispatch_total), 1);
#endif
#ifdef MAME_SGI_CALLSITE_IC_HITRATE_DIAG
		// TEMPORARY diagnostic (2026-08-08) - see cfunc_callsite_ic_probe's
		// comment above and m_diag_callsite_ic_state's comment in mips3.h.
		// desc->pc (this JR/JALR instruction's own guest address) is the
		// call-site identity - stable across re-executions of this same
		// compiled instance, which is what a real per-call-site inline
		// cache would key on. I3 already holds the live resolved dispatch
		// key (generate_select_dispatch_key() just above); jmpdest itself
		// already holds the live target pc, read directly in the probe.
		UML_MOV(block, mem(&m_core->diag_ic_probe_callsite), desc->pc);
		UML_MOV(block, mem(&m_core->diag_ic_probe_mode), I3);
		UML_CALLC(block, cfunc_callsite_ic_probe, this);
#endif
#ifdef MAME_SGI_CALLSITE_IC_PROTOTYPE
		// PROTOTYPE (2026-08-08) - see the class-scope comment near
		// m_diag_callsite_ic_state (mips3.h) for the full design/history.
		// ic_idx is a purely compile-time hash of this call site's own
		// desc->pc into the fixed slot table - different call sites CAN
		// collide on the same slot, which is safe (the guard always
		// re-validates live values before trusting the fast path; a
		// collision just means more guard misses for the colliding
		// sites, never a wrong jump). Guard reads the slot's current
		// expectation live via mem() - NOT baked as an immediate, since
		// func_ic_learn_and_arm() sets these at runtime, after this code
		// is already generated. On a hit, OP_DIRECTJMP jumps straight
		// through &m_core->ic_site_target[ic_idx] - a fixed, compile-
		// time-known host address (m_core never moves once allocated),
		// so unlike the earlier byte-patched design there's no
		// instruction-address-capture problem to solve at all here.
		//
		// On a MISS, additionally check (cheaply - one more CMP against
		// the sentinel, only reached on the already-taken slow path)
		// whether this slot has never been established; if so, call
		// cfunc_ic_learn_and_arm() to seed it from THIS dispatch's live
		// values. This only actually does anything the first time each
		// slot is touched per generation (at most MAME_SGI_IC_SITE_COUNT
		// times total, not per-dispatch) - every later use of an
		// established slot either guard-hits (fast path, untouched by
		// this) or guard-misses against a REAL (non-sentinel)
		// expectation and correctly does nothing further here.
		{
			uint32_t const ic_idx = (desc->pc >> 2) % MAME_SGI_IC_SITE_COUNT;
			uml::code_label const ic_slow = compiler_temp.labelnum++;
			uml::code_label const ic_established = compiler_temp.labelnum++;
			UML_CMP(block, I3, mem(&m_core->ic_site_expected_mode[ic_idx]));   // cmp     <mode>,[ic_site_expected_mode[idx]]
			UML_JMPc(block, COND_NE, ic_slow);                                  // jmp     ic_slow,NE
			UML_CMP(block, mem(&m_core->jmpdest), mem(&m_core->ic_site_expected_pc[ic_idx])); // cmp [jmpdest],[ic_site_expected_pc[idx]]
			UML_JMPc(block, COND_NE, ic_slow);                                  // jmp     ic_slow,NE
			UML_DIRECTJMP(block, &m_core->ic_site_target[ic_idx]);             // directjmp &ic_site_target[idx]
			UML_LABEL(block, ic_slow);                                          // ic_slow:
			UML_CMP(block, mem(&m_core->ic_site_expected_pc[ic_idx]), 0xFFFFFFFFU); // cmp [ic_site_expected_pc[idx]],SENTINEL
			UML_JMPc(block, COND_NE, ic_established);                          // jmp     ic_established,NE
			UML_MOV(block, mem(&m_core->ic_live_mode), I3);                     // mov     [ic_live_mode],<mode>
			UML_MOV(block, mem(&m_core->ic_slot_index), ic_idx);                // mov     [ic_slot_index],ic_idx
			UML_CALLC(block, cfunc_ic_learn_and_arm, this);                     // callc   ic_learn_and_arm
			UML_LABEL(block, ic_established);                                   // ic_established:
		}
#endif
		UML_HASHJMP(block, I3, mem(&m_core->jmpdest), *m_nocode);   // hashjmp <mode>,<rsreg>,nocode
#else
		UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->jmpdest), *m_nocode);   // hashjmp <mode>,<rsreg>,nocode
#endif
#else
		UML_HASHJMP(block, m_core->dispatch_mode, mem(&m_core->jmpdest), *m_nocode);         // hashjmp <mode>,<rsreg>,nocode
#endif
	}

	/* update the label */
	compiler.labelnum = compiler_temp.labelnum;

	/* reset the mapvar to the current cycles and account for skipped slots */
	compiler.cycles += desc->skipslots;
	UML_MAPVAR(block, MAPVAR_CYCLES, compiler.cycles);                             // mapvar  CYCLES,compiler.cycles
}


/*-------------------------------------------------
    generate_likely_branch_skip_fixup - task #84 fix
    (2026-09-10). BEQL/BNEL/BLEZL/BGTZL ("branch
    likely") annul their delay slot when not taken.
    The not-taken path (the caller's own `skip:` label,
    reached via UML_JMPc) does nothing further and just
    falls through into whatever generate_sequence_
    instruction()'s outer per-sequence loop naturally
    compiles next - trusting the frontend's live-
    descriptor chain to resume at desc->pc+8 (skipslots=1
    already tells that loop's own nextpc computation,
    mips3drc.cpp ~2876, to skip the delay slot's 4
    bytes). That trust breaks when the delay slot's own
    instruction hits compiler_page_fault() at compile
    time (mips3com.cpp's normal, frequent ASID-blanking
    vtlb invalidation - wiki/performance.md): drcfe.ipp's
    do_describe_code() raw walk BREAKS entirely on a page
    fault, so desc->pc+8 never gets its own descriptor in
    that compile at all, and whatever OTHER, unrelated
    address the frontend independently discovered next
    (observed live: this same branch's own TAKEN target,
    reached via a separate branch-target worklist entry)
    silently takes its place in the live sequence. This
    is the confirmed root cause of task #84 (wiki/
    task84.md, 2026-09-10 "ROOT CAUSE FOUND" entry) -
    live-instrumented, 100% correlation across 30
    explicit-flush cycles: the not-taken path redispatches
    to the branch's own taken target regardless of the
    real, correctly-evaluated branch outcome.

    Fix: when the frontend ALREADY knows (at compile
    time - this is information drcfe.ipp has already
    computed, not something guessed here) that the delay
    slot's own instruction hit compiler_page_fault(),
    don't trust fallthrough at all - emit the same kind
    of explicit redispatch the TAKEN path already uses
    (generate_delay_slot_and_branch, the non-dynamic-
    target case), straight to the real not-taken PC
    (desc->pc + 8). Gated on that flag so the normal,
    non-faulted case (the overwhelming majority of BEQL-
    class branches system-wide, per wiki/performance.md's
    own measurement that compiler_page_fault() is a
    small minority of all compile-time TLB activity)
    keeps its existing, free fallthrough with zero added
    cost - this only pays for the extra HASHJMP exactly
    when the fallthrough assumption is actually unsafe.

    desc->skipslots != 0 is the same condition mips3fe.cpp
    uses to distinguish "L" (likely) opcodes from their
    plain counterparts (`desc.skipslots = (opswitch &
    0x10) ? 1 : 0`) - checking it here means this function
    is a no-op for plain BEQ/BNE/BLEZ/BGTZ, whose delay
    slot is never excluded from the live sequence and
    must keep executing via ordinary fallthrough.
-------------------------------------------------*/

void mips3_device::generate_likely_branch_skip_fixup(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	if (desc->skipslots != 0 && desc->delay.first() != nullptr && desc->delay.first()->compiler_page_fault())
	{
#ifdef MAME_SGI_TASK84_FIXUP_DIAG
		// This is a plain C++ increment, not a UML runtime counter - the
		// condition itself is evaluated once per COMPILE, not per
		// execution, so a host-side static counter directly measures the
		// right thing. See the macro's own comment (near the other task
		// #84 diagnostic defines, above) for what this was used to
		// measure and the result.
		static uint32_t s_fixup_count = 0;
		++s_fixup_count;
		fprintf(stderr, "[TASK84_FIXUP_DIAG] generate_likely_branch_skip_fixup fired: count=%u pc=0x%08x\n",
			s_fixup_count, desc->pc);
		fflush(stderr);
#endif

		generate_update_cycles(block, compiler, desc->pc + 8, true);
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
		UML_HASHJMP(block, static_dispatch_key(desc->pc + 8), desc->pc + 8, *m_nocode);   // hashjmp <mode>,desc->pc+8,nocode
#else
		UML_HASHJMP(block, m_core->dispatch_mode, desc->pc + 8, *m_nocode);               // hashjmp <mode>,desc->pc+8,nocode
#endif
	}
}


/*-------------------------------------------------
    generate_opcode - generate code for a specific
    opcode
-------------------------------------------------*/

bool mips3_device::generate_opcode(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const int in_delay_slot = desc->in_delay_slot();
	const uint32_t op = desc->opptr;
	const uint8_t opswitch = op >> 26;
	uml::code_label skip;

#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
	if (desc->pc == MAME_SGI_LWC1_DELAYSLOT_PROBE_PC)
	{
		lwc1_probe_log("[LWC1_PROBE] generate_opcode ENTRY pc=0x%08x op=0x%08x opswitch=0x%02x "
			"in_delay_slot=%d mode=0x%08x dispatch_mode=0x%08x (expect opswitch=0x31 for LWC1)\n",
			desc->pc, op, opswitch, in_delay_slot, m_core->mode, m_core->dispatch_mode);
	}
#endif

#ifdef MAME_SGI_MUTEXWAIT_SPIN_DIAG
	if (desc->pc == 0x880eaca4)
		UML_CALLC(block, cfunc_mutexwait_spin_probe, this);                   // callc   cfunc_mutexwait_spin_probe
	else if (desc->pc == 0x88008a78)
		UML_CALLC(block, cfunc_intr_entry_probe, this);                       // callc   cfunc_intr_entry_probe
#endif

#ifdef MAME_SGI_PCKM_MUTEX_DIAG
	if (desc->pc == 0x8802dbe0)
		UML_CALLC(block, cfunc_pckm_mutex_lock_call_probe, this);             // callc   cfunc_pckm_mutex_lock_call_probe
	else if (desc->pc == 0x88008a78)
		UML_CALLC(block, cfunc_pckm_mutex_snapshot_probe, this);              // callc   cfunc_pckm_mutex_snapshot_probe
#endif

	switch (opswitch)
	{
		/* ----- sub-groups ----- */

		case 0x00:  /* SPECIAL - MIPS I */
			return generate_special(block, compiler, desc);

		case 0x01:  /* REGIMM - MIPS I */
			return generate_regimm(block, compiler, desc);

		case 0x1c:  /* IDT-specific */
			return generate_idt(block, compiler, desc);


		/* ----- jumps and branches ----- */

		case 0x02:  /* J - MIPS I */
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			return true;

		case 0x03:  /* JAL - MIPS I */
#ifdef MAME_DRC_DIAG
			if (desc->pc == 0x880eb7f0)
				UML_CALLC(block, cfunc_sv_wait_sig_unlock_probe, this);
			else if (desc->pc == 0x881e2654)
				UML_CALLC(block, cfunc_xlog_write_call_capture_probe, this);
			else if (desc->pc == 0x881e35ec)
				UML_CALLC(block, cfunc_iclogspace_splhi_probe, this);
			else if (desc->pc == 0x881e35c8 || desc->pc == 0x881e3660 || desc->pc == 0x881e3690 ||
			         desc->pc == 0x881e36cc || desc->pc == 0x881e3730)
				UML_CALLC(block, cfunc_iclogspace_splx_probe, this);
			else if (desc->pc == 0x880eac04)
				UML_CALLC(block, cfunc_mutex_lock_enqueue_probe, this);
			else if (desc->pc == 0x880eada4 || desc->pc == 0x880eadc8)
				UML_CALLC(block, cfunc_mutex_lock_enqueue_end_probe, this);
#endif
#ifdef MAME_SGI_RMAP_DEL_PANIC_DIAG
			if (desc->pc == 0x881882a8)
				UML_CALLC(block, cfunc_rmap_del_panic_probe_1, this);
			else if (desc->pc == 0x881882c8)
				UML_CALLC(block, cfunc_rmap_del_panic_probe_2, this);
#endif
			generate_delay_slot_and_branch(block, compiler, desc, 31);       // <next instruction + hashjmp>
			return true;

		case 0x04:  /* BEQ - MIPS I */
		case 0x14:  /* BEQL - MIPS II */
#ifdef MAME_DRC_DIAG
			if (desc->pc == 0x881e265c)
				UML_CALLC(block, cfunc_xlog_write_return_check_probe, this);
			// COMPILE-TIME check (2026-07-26): does the mrupdatef retry
			// loop's own back-edge (this "b"/beqz-to-self-ish branch at
			// 0x880e9ac0/0x880e9ab8, part of a multi-block loop formed by
			// jal calls to splhi/us_delay/min rather than one self-
			// contained sequence) actually have interrupt-check code armed?
			// The existing self-loop fix (see "four real DRC bugs" in
			// wiki/assets.md) only scans for an instruction whose own
			// targetpc equals the CURRENT sequence's start - a multi-block
			// loop like this one has no single instruction matching that
			// shape, so the expectation is this reads false, unlike a
			// bare register-only spin loop.
			if (desc->pc == 0x880e9ab8 || desc->pc == 0x880e9ac0)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME(mrupdatef) pc=0x%08x checkints=%d checksoftints=%d\n",
					desc->pc, compiler.checkints, compiler.checksoftints);
				fflush(stderr);
			}
#endif
#ifdef MAME_SGI_TASK84_CPPERROR_COND_TRACE_DIAG
			// See the macro's own comment near the top of this file.
			// RSREG here is $v1 (the token type byte just fetched);
			// $s1 is register 17 in standard MIPS GPR numbering.
			if (desc->pc == 0x1007de98)
			{
				UML_DMOV(block, mem(&m_core->diag_cond_v1), R64(RSREG));
				UML_DMOV(block, mem(&m_core->diag_cond_s1), R64(17));
				UML_CALLC(block, cfunc_cpperror_cond_trace_diag, this);
			}
#endif
#ifdef MAME_SGI_TASK84_BEQL_S3_DIAG
			// The actual "predicate's answer is empty" decision itself:
			// RSREG here is $s3 (register 19, parse_answer's own acount),
			// RTREG is $zero. Captured right before the real UML_DCMP
			// below - as close to the CPU's own comparison as this
			// codegen structure allows.
			if (desc->pc == 0x1007dffc)
			{
				UML_DMOV(block, mem(&m_core->diag_beql_s3), R64(RSREG));
				UML_DMOV(block, mem(&m_core->diag_beql_v0), R64(2));
				UML_CALLC(block, cfunc_beql_s3_diag, this);
			}
			// The loop's OWN break check ("if (token->type ==
			// CPP_CLOSE_PAREN) break;", real GCC source cpplib.c:1590) -
			// this BEQ (not BEQL) jumps directly into 0x1007DFFC when
			// $v0(RSREG, the just-fetched token's type)==$s4(0x17, the
			// CLOSE_PAREN constant). Capturing $s3 HERE, at the moment
			// the loop actually decides to exit, is the direct check for
			// whether the loop genuinely never iterated (real acount==0)
			// or whether something diverges between this decision point
			// and the beql immediately downstream of it.
			if (desc->pc == 0x1007df68)
			{
				UML_DMOV(block, mem(&m_core->diag_breakcheck_v0), R64(RSREG));
				UML_DMOV(block, mem(&m_core->diag_breakcheck_s3), R64(19));
				UML_CALLC(block, cfunc_breakcheck_s3_diag, this);
			}
#endif
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_JMPc(block, COND_NE, skip = compiler.labelnum++);                  // jmp     skip,NE
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			UML_LABEL(block, skip);                                             // skip:
			generate_likely_branch_skip_fixup(block, compiler, desc);          // TASK84 FIX (2026-09-10)
			return true;

		case 0x05:  /* BNE - MIPS I */
		case 0x15:  /* BNEL - MIPS II */
#ifdef MAME_DRC_DIAG
			if (desc->pc == 0x880e9a94)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME(mrupdatef) pc=0x%08x checkints=%d checksoftints=%d\n",
					desc->pc, compiler.checkints, compiler.checksoftints);
				fflush(stderr);
			}
#endif
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_JMPc(block, COND_E, skip = compiler.labelnum++);                       // jmp     skip,E
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			UML_LABEL(block, skip);                                             // skip:
			generate_likely_branch_skip_fixup(block, compiler, desc);          // TASK84 FIX (2026-09-10)
			return true;

		case 0x06:  /* BLEZ - MIPS I */
		case 0x16:  /* BLEZL - MIPS II */
			if (RSREG != 0)
			{
				UML_DCMP(block, R64(RSREG), 0);                             // dcmp    <rsreg>,0
				UML_JMPc(block, COND_G, skip = compiler.labelnum++);                   // jmp     skip,G
				generate_delay_slot_and_branch(block, compiler, desc, 0);    // <next instruction + hashjmp>
				UML_LABEL(block, skip);                                         // skip:
				generate_likely_branch_skip_fixup(block, compiler, desc);      // TASK84 FIX (2026-09-10)
			}
			else
				generate_delay_slot_and_branch(block, compiler, desc, 0);    // <next instruction + hashjmp>
			return true;

		case 0x07:  /* BGTZ - MIPS I */
		case 0x17:  /* BGTZL - MIPS II */
			UML_DCMP(block, R64(RSREG), 0);                                 // dcmp    <rsreg>,0
			UML_JMPc(block, COND_LE, skip = compiler.labelnum++);                  // jmp     skip,LE
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			UML_LABEL(block, skip);                                             // skip:
			generate_likely_branch_skip_fixup(block, compiler, desc);          // TASK84 FIX (2026-09-10)
			return true;


		/* ----- immediate arithmetic ----- */

		case 0x0f:  /* LUI - MIPS I */
			if (RTREG != 0)
				UML_DMOV(block, R64(RTREG), UIMMVAL << 16);                 // dmov    <rtreg>,UIMMVAL << 16
			return true;

		case 0x08:  /* ADDI - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			if (m_drcoptions & MIPS3DRC_CHECK_OVERFLOWS)
				UML_EXHc(block, COND_V, *m_exception[EXCEPTION_OVERFLOW], 0);
																					// exh    overflow,0
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);                       // dsext   <rtreg>,i0,dword
			return true;

		case 0x09:  /* ADDIU - MIPS I */
#ifdef MAME_DRC_DIAG
			if (desc->pc == 0x880eb3e4)
				UML_CALLC(block, cfunc_sv_wait_entry_probe, this);
			else if (desc->pc == 0x881e301c)
				UML_CALLC(block, cfunc_xlog_state_clean_log_entry_probe, this);
			else if (desc->pc == 0x881e357c)
				UML_CALLC(block, cfunc_xlog_inner_worker_stack_tap, this);
			else if (desc->pc == 0x881e2148)
				UML_CALLC(block, cfunc_xlog_iodone_entry_probe, this);
			else if (desc->pc == 0x881e1680)
				UML_CALLC(block, cfunc_xfs_log_done_entry_probe, this);
			else if (desc->pc == 0x881e2628)
				UML_CALLC(block, cfunc_xlog_commit_record_entry_probe, this);
			else if (desc->pc == 0x88012200)
				UML_CALLC(block, cfunc_us_delay_entry_probe, this);
			else if (desc->pc == 0x880e99f0)
				UML_CALLC(block, cfunc_mrupdatef_entry_probe, this);
			else if (desc->pc == 0x881050e4)
				UML_CALLC(block, cfunc_tlbmiss_entry_probe, this);
#endif
			if (RTREG != 0)
			{
				UML_ADD(block, I0, R32(RSREG), SIMMVAL);                    // add     i0,<rsreg>,SIMMVAL,V
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);                       // dsext   <rtreg>,i0,dword
			}
#ifdef MAME_SGI_TASK84_BEQL_S3_DIAG
			// The loop's own "acount++" (real GCC source cpplib.c:1584's
			// for-loop increment, in the delay slot of the loop's own
			// back-edge "b 0x1007df40" at 0x1007dfd0) - counts real
			// iterations directly, complementing the break-check and
			// beql captures above.
			if (desc->pc == 0x1007dfd4)
			{
				UML_DMOV(block, mem(&m_core->diag_s3_increment_newval), R64(RTREG));
				UML_CALLC(block, cfunc_s3_increment_diag, this);
			}
#endif
			return true;

		case 0x18:  /* DADDI - MIPS III */
			UML_DADD(block, I0, R64(RSREG), SIMMVAL);                       // dadd    i0,<rsreg>,SIMMVAL
			if (m_drcoptions & MIPS3DRC_CHECK_OVERFLOWS)
				UML_EXHc(block, COND_V, *m_exception[EXCEPTION_OVERFLOW], 0);
																					// exh    overflow,0
			if (RTREG != 0)
				UML_DMOV(block, R64(RTREG), I0);                                // dmov    <rtreg>,i0
			return true;

		case 0x19:  /* DADDIU - MIPS III */
			if (RTREG != 0)
				UML_DADD(block, R64(RTREG), R64(RSREG), SIMMVAL);               // dadd    <rtreg>,<rsreg>,SIMMVAL
			return true;

		case 0x0c:  /* ANDI - MIPS I */
			if (RTREG != 0)
				UML_DAND(block, R64(RTREG), R64(RSREG), UIMMVAL);               // dand    <rtreg>,<rsreg>,UIMMVAL
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			if (desc->pc >= 0x8801b600 && desc->pc < 0x8801b700)
				UML_CALLC(block, cfunc_andi_ori_probe_1, this);                 // callc   cfunc_andi_ori_probe_1 (TEMPORARY, 2026-07-27)
#endif
			return true;

		case 0x0d:  /* ORI - MIPS I */
			if (RTREG != 0)
				UML_DOR(block, R64(RTREG), R64(RSREG), UIMMVAL);                // dor     <rtreg>,<rsreg>,UIMMVAL
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			if (desc->pc >= 0x8801b600 && desc->pc < 0x8801b700)
				UML_CALLC(block, cfunc_andi_ori_probe_2, this);                 // callc   cfunc_andi_ori_probe_2 (TEMPORARY, 2026-07-27)
#endif
			return true;

		case 0x0e:  /* XORI - MIPS I */
			if (RTREG != 0)
				UML_DXOR(block, R64(RTREG), R64(RSREG), UIMMVAL);               // dxor    <rtreg>,<rsreg>,UIMMVAL
			return true;

		case 0x0a:  /* SLTI - MIPS I */
			if (RTREG != 0)
			{
				UML_DCMP(block, R64(RSREG), SIMMVAL);                           // dcmp    <rsreg>,SIMMVAL
				UML_DSETc(block, COND_L, R64(RTREG));                                   // dset    <rtreg>,l
			}
			return true;

		case 0x0b:  /* SLTIU - MIPS I */
			if (RTREG != 0)
			{
				UML_DCMP(block, R64(RSREG), SIMMVAL);                           // dcmp    <rsreg>,SIMMVAL
				UML_DSETc(block, COND_B, R64(RTREG));                                   // dset    <rtreg>,b
			}
			return true;


		/* ----- memory load operations ----- */

		case 0x20:  /* LB - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read8[m_core->mode >> 1]);  // callh   read8
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), I0, SIZE_BYTE);                        // dsext   <rtreg>,i0,byte
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x21:  /* LH - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read16[m_core->mode >> 1]); // callh   read16
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), I0, SIZE_WORD);                        // dsext   <rtreg>,i0,word
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x23:  /* LW - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read32[m_core->mode >> 1]); // callh   read32
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);                       // dsext   <rtreg>,i0
#ifdef MAME_DRC_DIAG
			// TEMPORARY (2026-07-24, CORRECTED): mutex_wake's real,
			// unconditional post-dispatch entry (0x880eaf10 is
			// `lw $a2,-0x5fe8($0)`) - reached by BOTH the a1-was-already-
			// real-count path and the a1==-1-resolved-to-a-real-count path
			// (see func_mutex_wake_entry_probe's comment for the full
			// story of why the original hook, on the literal function top
			// gated on a1 != -1, was measuring nothing useful).
			if (desc->pc == 0x880eaf10)
				UML_CALLC(block, cfunc_lockstate_probe_7, this);
			else if (desc->pc == 0x880ea53c)
				UML_CALLC(block, cfunc_mutex_unlock_entry_probe, this);
			else if (desc->pc == 0x880eb598)
				UML_CALLC(block, cfunc_sv_wait_sig_worker_entry_probe, this);
			else if (desc->pc == 0x881e2b28)
			{
				UML_CALLC(block, cfunc_xlog_complete_entry_probe, this);
				UML_CALLC(block, cfunc_xlog_write_entry_log_probe, this);
			}
			else if (desc->pc == 0x881e43f4)
				UML_CALLC(block, cfunc_xlog_tlb_recheck_probe, this);
			else if (desc->pc == 0x880ec28c)
				UML_CALLC(block, cfunc_sv_timedwait_entry_probe, this);
			else if (desc->pc == 0x881d8128)
				UML_CALLC(block, cfunc_xfs_trans_commit_iclog_consume_probe, this);
			else if (desc->pc == 0x880eaf30)
				UML_CALLC(block, cfunc_mutex_wake_dequeue_probe, this);
#endif
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x30:  /* LL - MIPS II */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
#ifdef MAME_DRC_BUGFIXES
			// Interpreter (mips3.cpp's LL case) only records LLAddr/llbit
			// after a successful read into a non-$zero register. This
			// unconditional version recorded a reservation before the read
			// even completed and regardless of the destination register,
			// diverging from the interpreter's LL/SC bookkeeping.
			UML_MOV(block, I1, I0);                                         // mov     i1,i0 (save address across the read)
			UML_CALLH(block, *m_read32[m_core->mode >> 1]);                 // callh   read32
			if (RTREG != 0)
			{
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);               // dsext   <rtreg>,i0
				UML_MOV(block, mem(&m_core->cpr[0][COP0_LLAddr]), I1);      // mov     [LLAddr],i1
				UML_MOV(block, mem(&m_core->llbit), 1);                     // mov     [llbit],1
			}
#else
			UML_MOV(block, mem(&m_core->cpr[0][COP0_LLAddr]), I0);          // mov     [LLAddr],i0
			UML_CALLH(block, *m_read32[m_core->mode >> 1]);                 // callh   read32
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);               // dsext   <rtreg>,i0
			UML_MOV(block, mem(&m_core->llbit), 1);                         // mov     [llbit],1
#endif
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			if LL_BREAK
				UML_CALLC(block, cfunc_debug_break, this);                  // callc   cfunc_debug_break,nullptr
			return true;

		case 0x24:  /* LBU - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read8[m_core->mode >> 1]);  // callh   read8
			if (RTREG != 0)
				UML_DAND(block, R64(RTREG), I0, 0xff);                  // dand    <rtreg>,i0,0xff
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x25:  /* LHU - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read16[m_core->mode >> 1]); // callh   read16
			if (RTREG != 0)
				UML_DAND(block, R64(RTREG), I0, 0xffff);                    // dand    <rtreg>,i0,0xffff
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x27:  /* LWU - MIPS III */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read32[m_core->mode >> 1]); // callh   read32
			if (RTREG != 0)
				UML_DAND(block, R64(RTREG), I0, 0xffffffff);                // dand    <rtreg>,i0,0xffffffff
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x37:  /* LD - MIPS III */
			// DISABLED (2026-07-29): never gated, same pattern as the other
			// COMPILETIME diagnostics found tonight.
#ifdef MAME_SGI_COMPILETIME_ELOCORE_DIAG
			if (desc->pc == 0x88105308 || desc->pc == 0x881053b4)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME(LD-accessor-select) pc=0x%08x compiletime_mode=0x%08x (mode>>1=%u)\n",
					desc->pc, m_core->mode, m_core->mode >> 1);
				fflush(stderr);
			}
#endif
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read64[m_core->mode >> 1]); // callh   read64
			if (RTREG != 0)
				UML_DMOV(block, R64(RTREG), I0);                                // dmov    <rtreg>,i0
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x34:  /* LLD - MIPS III */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
#ifdef MAME_DRC_BUGFIXES
			// Same LL/LLAddr/llbit divergence as case 0x30 (LL) above.
			UML_MOV(block, I1, I0);                                         // mov     i1,i0 (save address across the read)
			UML_CALLH(block, *m_read64[m_core->mode >> 1]);                 // callh   read64
			if (RTREG != 0)
			{
				UML_DMOV(block, R64(RTREG), I0);                            // dmov    <rtreg>,i0
				UML_MOV(block, mem(&m_core->cpr[0][COP0_LLAddr]), I1);      // mov     [LLAddr],i1
				UML_MOV(block, mem(&m_core->llbit), 1);                     // mov     [llbit],1
			}
#else
			UML_MOV(block, mem(&m_core->cpr[0][COP0_LLAddr]), I0);          // mov     [LLAddr],i0
			UML_CALLH(block, *m_read64[m_core->mode >> 1]);                 // callh   read64
			if (RTREG != 0)
				UML_DMOV(block, R64(RTREG), I0);                            // dmov    <rtreg>,i0
			UML_MOV(block, mem(&m_core->llbit), 1);                         // mov     [llbit],1
#endif
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			if LL_BREAK
				UML_CALLC(block, cfunc_debug_break, this);                  // callc   cfunc_debug_break,nullptr
			return true;

		case 0x22:  /* LWL - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I1, I0, 3);                              // shl     i1,i0,3
			UML_AND(block, I0, I0, ~3);                             // and     i0,i0,~3
			if (!m_bigendian)
				UML_XOR(block, I1, I1, 0x18);                       // xor     i1,i1,0x18
			UML_SHR(block, I2, ~0, I1);                             // shr     i2,~0,i1
			UML_CALLH(block, *m_read32mask[m_core->mode >> 1]);
																					// callh   read32mask
			if (RTREG != 0)
			{
				UML_SHL(block, I2, ~0, I1);                         // shl     i2,~0,i1
				UML_MOV(block, I3, R32(RTREG));                             // mov     i3,<rtreg>
				UML_ROLINS(block, I3, I0, I1, I2);              // rolins  i3,i0,i1,i2
				UML_DSEXT(block, R64(RTREG), I3, SIZE_DWORD);                       // dsext   <rtreg>,i3,dword
			}
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x26:  /* LWR - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I1, I0, 3);                              // shl     i1,i0,3
			UML_AND(block, I0, I0, ~3);                             // and     i0,i0,~3
			if (m_bigendian)
				UML_XOR(block, I1, I1, 0x18);                       // xor     i1,i1,0x18
			UML_SHL(block, I2, ~0, I1);                             // shl     i2,~0,i1
			UML_CALLH(block, *m_read32mask[m_core->mode >> 1]);
																					// callh   read32mask
			if (RTREG != 0)
			{
				UML_SHR(block, I2, ~0, I1);                         // shr     i2,~0,i1
				UML_SUB(block, I1, 32, I1);                         // sub     i1,32,i1
				UML_MOV(block, I3, R32(RTREG));                             // mov     i3,<rtreg>
				UML_ROLINS(block, I3, I0, I1, I2);              // rolins  i3,i0,i1,i2
				UML_DSEXT(block, R64(RTREG), I3, SIZE_DWORD);                       // dsext   <rtreg>,i3,dword
			}
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x1a:  /* LDL - MIPS III */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I1, I0, 3);                              // shl     i1,i0,3
			UML_AND(block, I0, I0, ~7);                             // and     i0,i0,~7
			if (!m_bigendian)
				UML_XOR(block, I1, I1, 0x38);                       // xor     i1,i1,0x38
			UML_DSHR(block, I2, (uint64_t)~0, I1);                        // dshr    i2,~0,i1
			UML_CALLH(block, *m_read64mask[m_core->mode >> 1]);
																					// callh   read64mask
			if (RTREG != 0)
			{
				UML_DSHL(block, I2, (uint64_t)~0, I1);                    // dshl    i2,~0,i1
				UML_DROLINS(block, R64(RTREG), I0, I1, I2);         // drolins <rtreg>,i0,i1,i2
			}
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x1b:  /* LDR - MIPS III */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);            // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I1, I0, 3);                          // shl     i1,i0,3
			UML_AND(block, I0, I0, ~7);                         // and     i0,i0,~7
			if (m_bigendian)
				UML_XOR(block, I1, I1, 0x38);                   // xor     i1,i1,0x38
			UML_DSHL(block, I2, (uint64_t)~0, I1);                // dshl    i2,~0,i1
			UML_CALLH(block, *m_read64mask[m_core->mode >> 1]); // callh   read64mask
			if (RTREG != 0)
			{
				UML_DSHR(block, I2, (uint64_t)~0, I1);            // dshr    i2,~0,i1
				UML_SUB(block, I1, 64, I1);                     // sub     i1,64,i1
				UML_DROLINS(block, R64(RTREG), I0, I1, I2);     // drolins <rtreg>,i0,i1,i2
			}
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x31:  /* LWC1 - MIPS I */
#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
			if (desc->pc == MAME_SGI_LWC1_DELAYSLOT_PROBE_PC)
			{
				lwc1_probe_log("[LWC1_PROBE] LWC1 case reached pc=0x%08x RTREG=%d RSREG=%d "
					"mode=0x%08x dispatch_mode=0x%08x about to emit UML_MOV(FPR32(%d), I0)\n",
					desc->pc, RTREG, RSREG, m_core->mode, m_core->dispatch_mode, RTREG);
			}
#endif
			check_cop1_access(block);
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read32[m_core->mode >> 1]); // callh   read32
			UML_MOV(block, FPR32(RTREG), I0);           // mov     <cpr1_rt>,i0
#ifdef MAME_SGI_LWC1_DELAYSLOT_PROBE
			if (desc->pc == MAME_SGI_LWC1_DELAYSLOT_PROBE_PC)
			{
				lwc1_probe_log("[LWC1_PROBE] LWC1 case: UML_MOV(FPR32(%d), I0) emitted, pc=0x%08x\n",
					RTREG, desc->pc);
				g_mame_sgi_lwc1_target_block = true;
			}
#endif
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x35:  /* LDC1 - MIPS III */
			check_cop1_access(block);
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read64[m_core->mode >> 1]); // callh   read64
			UML_DMOV(block, FPR64(RTREG), I0);              // dmov    <cpr1_rt>,i0
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x32:  /* LWC2 - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read32[m_core->mode >> 1]); // callh   read32
			UML_DAND(block, CPR264(RTREG), I0, 0xffffffff);             // dand    <cpr2_rt>,i0,0xffffffff
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x36:  /* LDC2 - MIPS II */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read64[m_core->mode >> 1]); // callh   read64
			UML_DMOV(block, CPR264(RTREG), I0);                             // dmov    <cpr2_rt>,i0
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;


		/* ----- memory store operations ----- */

		case 0x28:  /* SB - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_MOV(block, I1, R32(RTREG));                                 // mov     i1,<rtreg>
			UML_CALLH(block, *m_write8[m_core->mode >> 1]); // callh   write8
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x29:  /* SH - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_MOV(block, I1, R32(RTREG));                                 // mov     i1,<rtreg>
			UML_CALLH(block, *m_write16[m_core->mode >> 1]);    // callh   write16
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x2b:  /* SW - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_MOV(block, I1, R32(RTREG));                                 // mov     i1,<rtreg>
			UML_CALLH(block, *m_write32[m_core->mode >> 1]);    // callh   write32
#ifdef MAME_DRC_DIAG
			// TEMPORARY (2026-07-24): six specific SW sites in the mutex
			// lock/enqueue/dequeue code - see cfunc_lockstate_probe above.
			if (desc->pc == 0x880eaf80)
				UML_CALLC(block, cfunc_lockstate_probe_1, this);
			else if (desc->pc == 0x880eaf88)
				UML_CALLC(block, cfunc_lockstate_probe_2, this);
			else if (desc->pc == 0x880eaee0)
			{
				UML_CALLC(block, cfunc_lockstate_probe_3, this);
				UML_CALLC(block, cfunc_install_mutex_tap, this);
			}
			else if (desc->pc == 0x880eaeec)
				UML_CALLC(block, cfunc_lockstate_probe_4, this);
			else if (desc->pc == 0x880eadc4)
				UML_CALLC(block, cfunc_lockstate_probe_5, this);
			else if (desc->pc == 0x880eae74)
				UML_CALLC(block, cfunc_lockstate_probe_6, this);
			// COMPILE-TIME check (2026-07-24): does the sequence containing
			// these exact instructions actually have interrupt-check code
			// armed for them at all? compiler.checkints/checksoftints only
			// ever get armed by an earlier COP0 Status/Cause/Compare write
			// in the same compiled sequence, or by the self-loop scan - this
			// enqueue/dequeue code touches neither, so the expectation is
			// both read false here. This is host-side compile-time state,
			// printed directly (no UML/runtime call needed).
			if (desc->pc == 0x880eaf80 || desc->pc == 0x880eaf88 ||
				desc->pc == 0x880eaee0 || desc->pc == 0x880eaeec ||
				desc->pc == 0x880eadc4 || desc->pc == 0x880eae74)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME pc=0x%08x checkints=%d checksoftints=%d\n",
					desc->pc, compiler.checkints, compiler.checksoftints);
				fflush(stderr);
			}
#endif
#ifdef MAME_IRIX_SPINLOCK_INIT_WORKAROUND
			if (desc->pc == 0x881b6540)
				UML_CALLC(block, cfunc_spinlock_init_workaround, this);
#endif
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x38:  /* SC - MIPS II */
			UML_MOV(block, I1, R32(RTREG));                                 // mov     i1,<rtreg>
			// SC must set the full 64-bit rt to 0/1, not just the low 32
			// bits: AND/OR (building the tested/set value) are full
			// 64-bit ops, LL sign-extends the loaded word, and the retry
			// BEQ/BNE compare all 64 bits. A 32-bit-only write here left
			// stale upper bits in place whenever SC legitimately failed
			// (reservation invalidated - exactly when a retry is
			// required), making the 64-bit retry compare see a spurious
			// nonzero value and silently skip the retry. Matches the
			// interpreter's RTVAL64 assignment and SCD's own UML_DMOV.
			UML_DMOV(block, R64(RTREG), 0);                                 // dmov    <rtreg>, 0
			UML_CMP(block, mem(&m_core->llbit), 0);                         // cmp     [llbit],0
			UML_JMPc(block, COND_E, skip = compiler.labelnum++);            // je      skip
			if (RTREG == RSREG)
				UML_ADD(block, I0, I1, SIMMVAL);                            // add     i0,i1,SIMMVAL
			else
				UML_ADD(block, I0, R32(RSREG), SIMMVAL);                    // add     i0,<rsreg>,SIMMVAL
			UML_CMP(block, mem(&m_core->cpr[0][COP0_LLAddr]), I0);          // cmp     [LLADDR],RSREG + SIMMVAL
			UML_JMPc(block, COND_NE, skip);                                 // jne     skip
			UML_CALLH(block, *m_write32[m_core->mode >> 1]);                // callh   write32
			UML_DMOV(block, R64(RTREG), 1);                                 // dmov    <rtreg>, 1
			UML_LABEL(block, skip);                                         // skip:
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x3f:  /* SD - MIPS III */
			// DISABLED (2026-07-29): never gated, same pattern as the other
			// COMPILETIME diagnostics found tonight.
#ifdef MAME_SGI_COMPILETIME_ELOCORE_DIAG
			if (desc->pc == 0x881050e8 || desc->pc == 0x881053a0)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME(SD-accessor-select) pc=0x%08x compiletime_mode=0x%08x (mode>>1=%u)\n",
					desc->pc, m_core->mode, m_core->mode >> 1);
				fflush(stderr);
			}
#endif
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			if (desc->pc == 0x881050e8)
				UML_CALLC(block, cfunc_fault_store_mode_probe_1, this);
			else if (desc->pc == 0x881053a0)
				UML_CALLC(block, cfunc_fault_store_mode_probe_2, this);
#endif
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_DMOV(block, I1, R64(RTREG));                                // dmov    i1,<rtreg>
			// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's
			// comment - this was the worst offender of the four, firing on
			// every 64-bit store anywhere in the kernel, unconditionally,
			// all night.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			UML_MOV(block, mem(&m_core->diag_sd_i0_stash), I0);             // mov     [diag_sd_i0_stash],i0
			UML_DMOV(block, mem(&m_core->diag_sd_i1_stash), I1);            // dmov    [diag_sd_i1_stash],i1
			UML_MOV(block, mem(&m_core->diag_sd_pc), desc->pc);             // mov     [diag_sd_pc],desc->pc
			UML_CALLC(block, cfunc_sd_value_probe, this);                   // callc   cfunc_sd_value_probe
			UML_MOV(block, I0, mem(&m_core->diag_sd_i0_stash));             // mov     i0,[diag_sd_i0_stash]
			UML_DMOV(block, I1, mem(&m_core->diag_sd_i1_stash));            // dmov    i1,[diag_sd_i1_stash]
#endif
			UML_CALLH(block, *m_write64[m_core->mode >> 1]);    // callh   write64
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x3c:  /* SCD - MIPS III */
			UML_DMOV(block, I1, R64(RTREG));                                // dmov    i1,<rtreg>
			UML_DMOV(block, R64(RTREG), 0);                                 // dmov    <rtreg>,0
			UML_CMP(block, mem(&m_core->llbit), 0);                         // cmp     [llbit],0
			UML_JMPc(block, COND_E, skip = compiler.labelnum++);            // je      skip
			if (RTREG == RSREG)
				UML_ADD(block, I0, I1, SIMMVAL);                            // add     i0,i,SIMMVAL
			else
				UML_ADD(block, I0, R32(RSREG), SIMMVAL);                    // add     i0,<rsreg>,SIMMVAL
			UML_CMP(block, mem(&m_core->cpr[0][COP0_LLAddr]), I0);          // cmp     [LLADDR],RSREG + SIMMVAL
			UML_JMPc(block, COND_NE, skip);                                 // jne     skip
			UML_CALLH(block, *m_write64[m_core->mode >> 1]);                // callh   write64
			UML_DMOV(block, R64(RTREG), 1);                                 // dmov    <rtreg>,1
			UML_LABEL(block, skip);                                         // skip:
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x2a:  /* SWL - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I3, I0, 3);                              // shl     i3,i0,3
			UML_AND(block, I0, I0, ~3);                             // and     i0,i0,~3
			UML_MOV(block, I1, R32(RTREG));                                 // mov     i1,<rtreg>
			if (!m_bigendian)
				UML_XOR(block, I3, I3, 0x18);                       // xor     i3,i3,0x18
			UML_SHR(block, I2, ~0, I3);                             // shr     i2,~0,i3
			UML_SHR(block, I1, I1, I3);                             // shr     i1,i1,i3
			UML_CALLH(block, *m_write32mask[m_core->mode >> 1]);
																					// callh   write32mask
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x2e:  /* SWR - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I3, I0, 3);                              // shl     i3,i0,3
			UML_AND(block, I0, I0, ~3);                             // and     i0,i0,~3
			UML_MOV(block, I1, R32(RTREG));                                 // mov     i1,<rtreg>
			if (m_bigendian)
				UML_XOR(block, I3, I3, 0x18);                       // xor     i3,i3,0x18
			UML_SHL(block, I2, ~0, I3);                             // shl     i2,~0,i3
			UML_SHL(block, I1, I1, I3);                             // shl     i1,i1,i3
			UML_CALLH(block, *m_write32mask[m_core->mode >> 1]);
																					// callh   write32mask
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x2c:  /* SDL - MIPS III */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);            // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I3, I0, 3);                          // shl     i3,i0,3
			UML_AND(block, I0, I0, ~7);                         // and     i0,i0,~7
			UML_DMOV(block, I1, R64(RTREG));                    // dmov    i1,<rtreg>
			if (!m_bigendian)
				UML_XOR(block, I3, I3, 0x38);                   // xor     i3,i3,0x38
			UML_DSHR(block, I2, (uint64_t)~0, I3);                // dshr    i2,~0,i3
			UML_DSHR(block, I1, I1, I3);                        // dshr    i1,i1,i3
			UML_CALLH(block, *m_write64mask[m_core->mode >> 1]);// callh   write64mask

			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x2d:  /* SDR - MIPS III */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);            // add     i0,<rsreg>,SIMMVAL
			UML_SHL(block, I3, I0, 3);                          // shl     i3,i0,3
			UML_AND(block, I0, I0, ~7);                         // and     i0,i0,~7
			UML_DMOV(block, I1, R64(RTREG));                    // dmov    i1,<rtreg>
			if (m_bigendian)
				UML_XOR(block, I3, I3, 0x38);                   // xor     i3,i3,0x38
			UML_DSHL(block, I2, (uint64_t)~0, I3);                // dshl    i2,~0,i3
			UML_DSHL(block, I1, I1, I3);                        // dshl    i1,i1,i3
			UML_CALLH(block, *m_write64mask[m_core->mode >> 1]);// callh   write64mask

			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x39:  /* SWC1 - MIPS I */
			check_cop1_access(block);
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);            // add     i0,<rsreg>,SIMMVAL
			UML_MOV(block, I1, FPR32(RTREG));                   // mov     i1,<cpr1_rt>
			UML_CALLH(block, *m_write32[m_core->mode >> 1]);    // callh   write32
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x3d:  /* SDC1 - MIPS III */
			check_cop1_access(block);
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);            // add     i0,<rsreg>,SIMMVAL
			UML_DMOV(block, I1, FPR64(RTREG));                  // dmov    i1,<cpr1_rt>
			UML_CALLH(block, *m_write64[m_core->mode >> 1]);    // callh   write64
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x3a:  /* SWC2 - MIPS I */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_MOV(block, I1, CPR232(RTREG));                                  // mov     i1,<cpr2_rt>
			UML_CALLH(block, *m_write32[m_core->mode >> 1]);    // callh   write32
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x3e:  /* SDC2 - MIPS II */
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_DMOV(block, I1, CPR264(RTREG));                             // dmov    i1,<cpr2_rt>
			UML_CALLH(block, *m_write64[m_core->mode >> 1]);    // callh   write64
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;


		/* ----- effective no-ops ----- */

		case 0x2f:  /* CACHE - MIPS II */
		case 0x33:  /* PREF - MIPS IV */
			return true;


		/* ----- coprocessor instructions ----- */

		case 0x10:  /* COP0 - MIPS I */
			return generate_cop0(block, compiler, desc);

		case 0x11:  /* COP1 - MIPS I */
			return generate_cop1(block, compiler, desc);

		case 0x13:  /* COP1X - MIPS IV */
			return generate_cop1x(block, compiler, desc);

		case 0x12:  /* COP2 - MIPS I */
			UML_EXH(block, *m_exception[EXCEPTION_INVALIDOP], 0);// exh     invalidop,0
			return true;


		/* ----- unimplemented/illegal instructions ----- */

//      default:    /* ??? */       invalid_instruction(op);                                                break;
	}

	return false;
}


/*-------------------------------------------------
    generate_special - compile opcodes in the
    'SPECIAL' group
-------------------------------------------------*/

bool mips3_device::generate_special(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uint32_t op = desc->opptr;
	const uint8_t opswitch = op & 63;

	switch (opswitch)
	{
		/* ----- shift instructions ----- */

		case 0x00:  /* SLL - MIPS I */
			if (RDREG != 0)
			{
				UML_SHL(block, I0, R32(RTREG), SHIFT);                  // shl     i0,<rtreg>,<shift>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x02:  /* SRL - MIPS I */
			if (RDREG != 0)
			{
				UML_SHR(block, I0, R32(RTREG), SHIFT);                  // shr     i0,<rtreg>,<shift>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x03:  /* SRA - MIPS I */
			if (RDREG != 0)
			{
				UML_SAR(block, I0, R32(RTREG), SHIFT);                  // sar     i0,<rtreg>,<shift>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x04:  /* SLLV - MIPS I */
			if (RDREG != 0)
			{
				UML_SHL(block, I0, R32(RTREG), R32(RSREG));                 // shl     i0,<rtreg>,<rsreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x06:  /* SRLV - MIPS I */
			if (RDREG != 0)
			{
				UML_SHR(block, I0, R32(RTREG), R32(RSREG));                 // shr     i0,<rtreg>,<rsreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x07:  /* SRAV - MIPS I */
			if (RDREG != 0)
			{
				UML_SAR(block, I0, R32(RTREG), R32(RSREG));                 // sar     i0,<rtreg>,<rsreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x38:  /* DSLL - MIPS III */
			if (RDREG != 0)
				UML_DSHL(block, R64(RDREG), R64(RTREG), SHIFT);             // dshl    <rdreg>,<rtreg>,<shift>
			return true;

		case 0x3a:  /* DSRL - MIPS III */
			if (RDREG != 0)
				UML_DSHR(block, R64(RDREG), R64(RTREG), SHIFT);             // dshr    <rdreg>,<rtreg>,<shift>
			return true;

		case 0x3b:  /* DSRA - MIPS III */
			if (RDREG != 0)
				UML_DSAR(block, R64(RDREG), R64(RTREG), SHIFT);             // dsar    <rdreg>,<rtreg>,<shift>
			return true;

		case 0x3c:  /* DSLL32 - MIPS III */
			if (RDREG != 0)
				UML_DSHL(block, R64(RDREG), R64(RTREG), SHIFT + 32);            // dshl    <rdreg>,<rtreg>,<shift>+32
			return true;

		case 0x3e:  /* DSRL32 - MIPS III */
			if (RDREG != 0)
				UML_DSHR(block, R64(RDREG), R64(RTREG), SHIFT + 32);            // dshr    <rdreg>,<rtreg>,<shift>+32
			return true;

		case 0x3f:  /* DSRA32 - MIPS III */
			if (RDREG != 0)
				UML_DSAR(block, R64(RDREG), R64(RTREG), SHIFT + 32);            // dsar    <rdreg>,<rtreg>,<shift>+32
			return true;

		case 0x14:  /* DSLLV - MIPS III */
			if (RDREG != 0)
				UML_DSHL(block, R64(RDREG), R64(RTREG), R64(RSREG));                // dshl    <rdreg>,<rtreg>,<rsreg>
			return true;

		case 0x16:  /* DSRLV - MIPS III */
			if (RDREG != 0)
				UML_DSHR(block, R64(RDREG), R64(RTREG), R64(RSREG));                // dshr    <rdreg>,<rtreg>,<rsreg>
			return true;

		case 0x17:  /* DSRAV - MIPS III */
			if (RDREG != 0)
				UML_DSAR(block, R64(RDREG), R64(RTREG), R64(RSREG));                // dsar    <rdreg>,<rtreg>,<rsreg>
			return true;


		/* ----- basic arithmetic ----- */

		case 0x20:  /* ADD - MIPS I */
			if (m_drcoptions & MIPS3DRC_CHECK_OVERFLOWS)
			{
				UML_ADD(block, I0, R32(RSREG), R32(RTREG));                 // add     i0,<rsreg>,<rtreg>
				UML_EXHc(block, COND_V, *m_exception[EXCEPTION_OVERFLOW], 0);
																					// exh     overflow,0,V
				if (RDREG != 0)
					UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                   // dsext   <rdreg>,i0,dword
			}
			else if (RDREG != 0)
			{
				UML_ADD(block, I0, R32(RSREG), R32(RTREG));                 // add     i0,<rsreg>,<rtreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x21:  /* ADDU - MIPS I */
			if (RDREG != 0)
			{
				UML_ADD(block, I0, R32(RSREG), R32(RTREG));                 // add     i0,<rsreg>,<rtreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x2c:  /* DADD - MIPS III */
			if (m_drcoptions & MIPS3DRC_CHECK_OVERFLOWS)
			{
				UML_DADD(block, I0, R64(RSREG), R64(RTREG));                    // dadd    i0,<rsreg>,<rtreg>
				UML_EXHc(block, COND_V, *m_exception[EXCEPTION_OVERFLOW], 0);
																					// exh     overflow,0,V
				if (RDREG != 0)
					UML_DMOV(block, R64(RDREG), I0);                            // dmov    <rdreg>,i0
			}
			else if (RDREG != 0)
				UML_DADD(block, R64(RDREG), R64(RSREG), R64(RTREG));                // dadd    <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x2d:  /* DADDU - MIPS III */
			if (RDREG != 0)
				UML_DADD(block, R64(RDREG), R64(RSREG), R64(RTREG));                // dadd    <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x22:  /* SUB - MIPS I */
			if (m_drcoptions & MIPS3DRC_CHECK_OVERFLOWS)
			{
				UML_SUB(block, I0, R32(RSREG), R32(RTREG));                 // sub     i0,<rsreg>,<rtreg>
				UML_EXHc(block, COND_V, *m_exception[EXCEPTION_OVERFLOW], 0);
																					// exh     overflow,0,V
				if (RDREG != 0)
					UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                   // dsext   <rdreg>,i0,dword
			}
			else if (RDREG != 0)
			{
				UML_SUB(block, I0, R32(RSREG), R32(RTREG));                 // sub     i0,<rsreg>,<rtreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x23:  /* SUBU - MIPS I */
			if (RDREG != 0)
			{
				UML_SUB(block, I0, R32(RSREG), R32(RTREG));                 // sub     i0,<rsreg>,<rtreg>
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                       // dsext   <rdreg>,i0,dword
			}
			return true;

		case 0x2e:  /* DSUB - MIPS III */
			if (m_drcoptions & MIPS3DRC_CHECK_OVERFLOWS)
			{
				UML_DSUB(block, I0, R64(RSREG), R64(RTREG));                    // dsub    i0,<rsreg>,<rtreg>
				UML_EXHc(block, COND_V, *m_exception[EXCEPTION_OVERFLOW], 0);
																					// exh     overflow,0,V
				if (RDREG != 0)
					UML_DMOV(block, R64(RDREG), I0);                            // dmov    <rdreg>,i0
			}
			else if (RDREG != 0)
				UML_DSUB(block, R64(RDREG), R64(RSREG), R64(RTREG));                // dsub    <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x2f:  /* DSUBU - MIPS III */
			if (RDREG != 0)
				UML_DSUB(block, R64(RDREG), R64(RSREG), R64(RTREG));                // dsub    <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x18:  /* MULT - MIPS I */
			UML_MULS(block, I0, I1, R32(RSREG), R32(RTREG));                // muls    i0,i1,<rsreg>,<rtreg>
			UML_DSEXT(block, LO64, I0, SIZE_DWORD);                                 // dsext   lo,i0,dword
			UML_DSEXT(block, HI64, I1, SIZE_DWORD);                                 // dsext   hi,i1,dword
			return true;

		case 0x19:  /* MULTU - MIPS I */
			UML_MULU(block, I0, I1, R32(RSREG), R32(RTREG));                // mulu    i0,i1,<rsreg>,<rtreg>
			UML_DSEXT(block, LO64, I0, SIZE_DWORD);                                 // dsext   lo,i0,dword
			UML_DSEXT(block, HI64, I1, SIZE_DWORD);                                 // dsext   hi,i1,dword
			return true;

		case 0x1c:  /* DMULT - MIPS III */
			UML_DMULS(block, LO64, HI64, R64(RSREG), R64(RTREG));
			return true;

		case 0x1d:  /* DMULTU - MIPS III */
			UML_DMULU(block, LO64, HI64, R64(RSREG), R64(RTREG));
			return true;

		case 0x1a:  /* DIV - MIPS I */
			// FIX (2026-07-30): both UML backends make DIVS a no-op on a
			// zero divisor, leaving I0/I1 untouched - previously whatever
			// scratch value an earlier instruction happened to leave there,
			// then unconditionally sign-extended into LO/HI regardless. The
			// interpreter (mips3.cpp) guards with `if (RTVAL32)` and leaves
			// LO/HI unchanged on divide-by-zero. Pre-seed I0/I1 from the
			// current LO/HI so the skip case reproduces "unchanged" instead
			// of committing unrelated garbage. DDIV/DDIVU don't need this -
			// they divide straight into the real LO64/HI64.
			UML_MOV(block, I0, LO32);                                           // mov     i0,lo
			UML_MOV(block, I1, HI32);                                           // mov     i1,hi
			UML_DIVS(block, I0, I1, R32(RSREG), R32(RTREG));                    // divs    i0,i1,<rsreg>,<rtreg>
			UML_DSEXT(block, LO64, I0, SIZE_DWORD);                             // dsext   lo,i0,dword
			UML_DSEXT(block, HI64, I1, SIZE_DWORD);                             // dsext   hi,i1,dword
			return true;

		case 0x1b:  /* DIVU - MIPS I */
			// FIX (2026-07-30): same as DIV above.
			UML_MOV(block, I0, LO32);                                           // mov     i0,lo
			UML_MOV(block, I1, HI32);                                           // mov     i1,hi
			UML_DIVU(block, I0, I1, R32(RSREG), R32(RTREG));                    // divu    i0,i1,<rsreg>,<rtreg>
			UML_DSEXT(block, LO64, I0, SIZE_DWORD);                             // dsext   lo,i0,dword
			UML_DSEXT(block, HI64, I1, SIZE_DWORD);                             // dsext   hi,i1,dword
			return true;

		case 0x1e:  /* DDIV - MIPS III */
			UML_DDIVS(block, LO64, HI64, R64(RSREG), R64(RTREG));                   // ddivs    lo,hi,<rsreg>,<rtreg>
			return true;

		case 0x1f:  /* DDIVU - MIPS III */
			UML_DDIVU(block, LO64, HI64, R64(RSREG), R64(RTREG));                   // ddivu    lo,hi,<rsreg>,<rtreg>
			return true;


		/* ----- basic logical ops ----- */

		case 0x24:  /* AND - MIPS I */
			if (RDREG != 0)
				UML_DAND(block, R64(RDREG), R64(RSREG), R64(RTREG));                // dand     <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x25:  /* OR - MIPS I */
			if (RDREG != 0)
				UML_DOR(block, R64(RDREG), R64(RSREG), R64(RTREG));                 // dor      <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x26:  /* XOR - MIPS I */
			if (RDREG != 0)
				UML_DXOR(block, R64(RDREG), R64(RSREG), R64(RTREG));                // dxor     <rdreg>,<rsreg>,<rtreg>
			return true;

		case 0x27:  /* NOR - MIPS I */
			if (RDREG != 0)
			{
				UML_DOR(block, I0, R64(RSREG), R64(RTREG));                 // dor      i0,<rsreg>,<rtreg>
				UML_DXOR(block, R64(RDREG), I0, (uint64_t)~0);                // dxor     <rdreg>,i0,~0
			}
			return true;


		/* ----- basic comparisons ----- */

		case 0x2a:  /* SLT - MIPS I */
			if (RDREG != 0)
			{
				UML_DCMP(block, R64(RSREG), R64(RTREG));                            // dcmp    <rsreg>,<rtreg>
				UML_DSETc(block, COND_L, R64(RDREG));                                   // dset    <rdreg>,l
			}
			return true;

		case 0x2b:  /* SLTU - MIPS I */
			if (RDREG != 0)
			{
				UML_DCMP(block, R64(RSREG), R64(RTREG));                            // dcmp    <rsreg>,<rtreg>
				UML_DSETc(block, COND_B, R64(RDREG));                                   // dset    <rdreg>,b
			}
			return true;


		/* ----- conditional traps ----- */

		case 0x30:  /* TGE - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_EXHc(block, COND_GE, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,GE
			return true;

		case 0x31:  /* TGEU - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_EXHc(block, COND_AE, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,AE
			return true;

		case 0x32:  /* TLT - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_EXHc(block, COND_L, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,LT
			return true;

		case 0x33:  /* TLTU - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_EXHc(block, COND_B, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,B
			return true;

		case 0x34:  /* TEQ - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_EXHc(block, COND_E, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,E
			return true;

		case 0x36:  /* TNE - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_EXHc(block, COND_NE, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,NE
			return true;


		/* ----- conditional moves ----- */

		case 0x0a:  /* MOVZ - MIPS IV */
			if (RDREG != 0)
			{
				UML_DCMP(block, R64(RTREG), 0);                             // dcmp    <rtreg>,0
				UML_DMOVc(block, COND_Z, R64(RDREG), R64(RSREG));                       // dmov    <rdreg>,<rsreg>,Z
			}
			return true;

		case 0x0b:  /* MOVN - MIPS IV */
			if (RDREG != 0)
			{
				UML_DCMP(block, R64(RTREG), 0);                             // dcmp    <rtreg>,0
				UML_DMOVc(block, COND_NZ, R64(RDREG), R64(RSREG));                  // dmov    <rdreg>,<rsreg>,NZ
			}
			return true;

		case 0x01:  /* MOVF/MOVT - MIPS IV */
			if (RDREG != 0)
			{
				UML_TEST(block, CCR132(31), FCCMASK(op >> 18));             // test    ccr31,fcc_mask[x]
				UML_DMOVc(block, ((op >> 16) & 1) ? COND_NZ : COND_Z, R64(RDREG), R64(RSREG));
																					// dmov    <rdreg>,<rsreg>,NZ/Z
			}
			return true;


		/* ----- jumps and branches ----- */

		case 0x08:  /* JR - MIPS I */
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			return true;

		case 0x09:  /* JALR - MIPS I */
#ifdef MAME_DRC_DIAG
			if (desc->pc == 0x880b63a4)
				UML_CALLC(block, cfunc_biodone_dispatch_probe, this);
#endif
			generate_delay_slot_and_branch(block, compiler, desc, RDREG);    // <next instruction + hashjmp>
			return true;


		/* ----- system calls ----- */

		case 0x0c:  /* SYSCALL - MIPS I */
			UML_EXH(block, *m_exception[EXCEPTION_SYSCALL], 0);  // exh     syscall,0
			return true;

		case 0x0d:  /* BREAK - MIPS I */
			UML_EXH(block, *m_exception[EXCEPTION_BREAK], 0);    // exh     break,0
			return true;


		/* ----- effective no-ops ----- */

		case 0x0f:  /* SYNC - MIPS II */
			// NOTE: genuinely a no-op under this DRC's current architecture -
			// MIPS SYNC is a multiprocessor memory-ordering barrier, and this
			// core (like the interpreter's own SYNC handling) only ever
			// models a single CPU executing on a single host thread with
			// instructions compiled/emitted in strict program order, so
			// there is currently no reordering for a barrier to prevent.
			// Confirmed empirically, not just by this reasoning: a real host
			// memory fence was tried here in place of the no-op and made no
			// observable difference to a reproducible corruption bug this
			// was suspected of contributing to. Flagged here for whoever
			// eventually attempts true multiprocessor emulation (multiple
			// host threads, one per virtual CPU): at that point SYNC would
			// need a real implementation (e.g. a host memory fence, or
			// explicit coordination with the other virtual CPUs' threads).
			return true;


		/* ----- hi/lo register access ----- */

		case 0x10:  /* MFHI - MIPS I */
			if (RDREG != 0)
				UML_DMOV(block, R64(RDREG), HI64);                                  // dmov    <rdreg>,hi
			return true;

		case 0x11:  /* MTHI - MIPS I */
			UML_DMOV(block, HI64, R64(RSREG));                                      // dmov    hi,<rsreg>
			return true;

		case 0x12:  /* MFLO - MIPS I */
			if (RDREG != 0)
				UML_DMOV(block, R64(RDREG), LO64);                                  // dmov    <rdreg>,lo
			return true;

		case 0x13:  /* MTLO - MIPS I */
			UML_DMOV(block, LO64, R64(RSREG));                                      // dmov    lo,<rsreg>
			return true;
	}
	return false;
}


/*-------------------------------------------------
    generate_regimm - compile opcodes in the
    'REGIMM' group
-------------------------------------------------*/

bool mips3_device::generate_regimm(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uint32_t op = desc->opptr;
	const uint8_t opswitch = RTREG;
	uml::code_label skip;

	switch (opswitch)
	{
		case 0x00:  /* BLTZ */
		case 0x02:  /* BLTZL */
		case 0x10:  /* BLTZAL */
		case 0x12:  /* BLTZALL */
			if (opswitch & 0x10)
			{
				UML_DMOV(block, R64(31), (int32_t)(desc->pc + 8));
			}
			if (RSREG != 0)
			{
				UML_DCMP(block, R64(RSREG), 0);                             // dcmp    <rsreg>,0
				UML_JMPc(block, COND_GE, skip = compiler.labelnum++);              // jmp     skip,GE
				generate_delay_slot_and_branch(block, compiler, desc, 0);
																					// <next instruction + hashjmp>
				UML_LABEL(block, skip);                                         // skip:
			}
			return true;

		case 0x01:  /* BGEZ */
		case 0x03:  /* BGEZL */
		case 0x11:  /* BGEZAL */
		case 0x13:  /* BGEZALL */
			if (opswitch & 0x10)
			{
				UML_DMOV(block, R64(31), (int32_t)(desc->pc + 8));
			}

			if (RSREG != 0)
			{
				UML_DCMP(block, R64(RSREG), 0);                             // dcmp    <rsreg>,0
				UML_JMPc(block, COND_L, skip = compiler.labelnum++);       // jmp     skip,L
				generate_delay_slot_and_branch(block, compiler, desc, 0);   // <next instruction + hashjmp>
				UML_LABEL(block, skip);                                     // skip:
			}
			else
			{
				generate_delay_slot_and_branch(block, compiler, desc, 0);   // <next instruction + hashjmp>
			}
			return true;

		case 0x08:  /* TGEI */
			UML_DCMP(block, R64(RSREG), SIMMVAL);                               // dcmp    <rsreg>,SIMMVAL
			UML_EXHc(block, COND_GE, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,GE
			return true;

		case 0x09:  /* TGEIU */
			UML_DCMP(block, R64(RSREG), SIMMVAL);                               // dcmp    <rsreg>,SIMMVAL
			UML_EXHc(block, COND_AE, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,AE
			return true;

		case 0x0a:  /* TLTI */
			UML_DCMP(block, R64(RSREG), SIMMVAL);                               // dcmp    <rsreg>,SIMMVAL
			UML_EXHc(block, COND_L, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,L
			return true;

		case 0x0b:  /* TLTIU */
			UML_DCMP(block, R64(RSREG), SIMMVAL);                               // dcmp    <rsreg>,SIMMVAL
			UML_EXHc(block, COND_B, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,B
			return true;

		case 0x0c:  /* TEQI */
			UML_DCMP(block, R64(RSREG), SIMMVAL);                               // dcmp    <rsreg>,SIMMVAL
			UML_EXHc(block, COND_E, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,E
			return true;

		case 0x0e:  /* TNEI */
			UML_DCMP(block, R64(RSREG), SIMMVAL);                               // dcmp    <rsreg>,SIMMVAL
			UML_EXHc(block, COND_NE, *m_exception[EXCEPTION_TRAP], 0);// exh     trap,0,NE
			return true;
	}
	return false;
}


/*-------------------------------------------------
    generate_idt - compile opcodes in the IDT-
    specific group
-------------------------------------------------*/

bool mips3_device::generate_idt(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uint32_t op = desc->opptr;
	const uint8_t opswitch = op & 0x1f;

	/* only enabled on IDT processors */
	if (m_flavor != MIPS3_TYPE_R4650)
		return false;

	switch (opswitch)
	{
		case 0: /* MAD */
			if (RSREG != 0 && RTREG != 0)
			{
				UML_MULS(block, I0, I1, R32(RSREG), R32(RTREG));                // muls   i0,i1,rsreg,rtreg
				UML_ADD(block, I0, I0, LO32);                                   // add    i0,i0,lo
				UML_ADDC(block, I1, I1, HI32);                                  // addc   i1,i1,hi
				UML_DSEXT(block, LO64, I0, SIZE_DWORD);                         // dsext   lo,i0,dword
				UML_DSEXT(block, HI64, I1, SIZE_DWORD);                         // dsext   hi,i1,dword
			}
			return true;

		case 1: /* MADU */
			if (RSREG != 0 && RTREG != 0)
			{
				UML_MULU(block, I0, I1, R32(RSREG), R32(RTREG));                // mulu   i0,i1,rsreg,rtreg
				UML_ADD(block, I0, I0, LO32);                                   // add    i0,i0,lo
				UML_ADDC(block, I1, I1, HI32);                                  // addc   i1,i1,hi
				UML_DSEXT(block, LO64, I0, SIZE_DWORD);                         // dsext   lo,i0,dword
				UML_DSEXT(block, HI64, I1, SIZE_DWORD);                         // dsext   hi,i1,dword
			}
			return true;

		case 2: /* MUL */
			if (RDREG != 0)
			{
				UML_MULS(block, I0, I0, R32(RSREG), R32(RTREG));                // muls   i0,i0,rsreg,rtreg
				UML_DSEXT(block, R64(RDREG), I0, SIZE_DWORD);                   // dsext  rdreg,i0,dword
			}
			return true;
	}
	return false;
}


/*-------------------------------------------------
    generate_set_cop0_reg - generate code to
    handle special COP0 registers
-------------------------------------------------*/

bool mips3_device::generate_set_cop0_reg(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, uint8_t reg)
{
	const int in_delay_slot = desc->in_delay_slot();
	uml::code_label link;

	switch (reg)
	{
		case COP0_Cause:
			UML_ROLINS(block, CPR032(COP0_Cause), I0, 0, ~0xfc00);              // rolins  [Cause],i0,0,~0xfc00
			compiler.checksoftints = true;
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case COP0_Status:
			// TEMPORARY (2026-07-27): widened from one hardcoded PC
			// (0x8801b6c8) to a range - the same logical vector has now
			// been observed compiled at 0x8801b6c8, 0x8801b668, and
			// 0x8801b638 (an earlier session) across different builds, as
			// PCs shift with each fix. A fixed single address silently
			// misses whichever sibling instance is actually live in a
			// given build/run - confirmed directly tonight when a panic
			// hit 0x8801b668 while this probe only watched 0x8801b6c8.
#ifdef MAME_SGI_COMPILETIME_ELOCORE_DIAG
			if (desc->pc >= 0x8801b600 && desc->pc < 0x8801b700)
			{
				fprintf(stderr, "[MAME_DRC_DIAG] COMPILETIME(elocore_exl_N) pc=0x%08x compiletime_mode=0x%08x checkints=%d checksoftints=%d in_delay_slot=%d is_branch_target=%d genuine_miss=%d\n",
					desc->pc, m_core->mode, compiler.checkints, compiler.checksoftints, in_delay_slot, desc->is_branch_target(),
#ifdef MAME_DRC_CHECKSUM_MISMATCH_DIAG
					(desc->pc == m_diag_compile_entry_pc) ? 1 : 0
#else
					-1
#endif
					);
				// TEMPORARY (2026-07-27): dump raw instruction words around
				// this PC directly from guest memory at compile time (no
				// runtime probe needed - the code is already resident) to
				// re-verify what's actually here in the *current* build,
				// rather than trusting an earlier session's disassembly of
				// what may be a different address after PCs shifted.
				// Use m_program->read_dword() rather than a raw m_prptr byte
				// cast - this is plain host C++ (compile-time diagnostic,
				// not generated guest code), so the normal, already-correct
				// address-space read path is available and handles
				// endianness/alignment the same way every other legitimate
				// read in this file does, unlike m_prptr (meant to hand a
				// base pointer to UML_LOAD, which itself applies a
				// caller-supplied XOR adjustment - not a flat byte array to
				// index directly, as an earlier attempt here wrongly assumed).
				fprintf(stderr, "[MAME_DRC_DIAG] RAWWORDS pc=0x%08x:", desc->pc);
				for (int off = -20; off <= 20; off += 4)
					fprintf(stderr, " %08x", m_program->read_dword(desc->physpc + off));
				fprintf(stderr, "\n");
				fflush(stderr);
			}
#endif
#ifdef MAME_DRC_BUGFIXES
			// Bug fix: generate_update_cycles (below) uses I0/I1 as scratch
			// whenever compiler.checkints/checksoftints is already pending
			// entering this instruction (set by an earlier Status/Cause/
			// Compare write compiled into the same code_compile_block call -
			// compiler_state is declared once per call, outside the loop
			// over sequences, so this can carry across sequence/function
			// boundaries), computing Cause & OldStatus & mask into I0
			// unconditionally before branching. Calling it before this
			// case's own store meant that scratch write could silently
			// replace the guest's real MTC0 value with that unrelated AND
			// result - confirmed live: real kernel restores of a saved
			// Status (after a cache-flush or TLB critical section) landing
			// as exact 0x00000000 instead of the intended value. Fix:
			// finish every use of I0/I1 (the store itself, and the diff
			// this case computes against the old value) before calling
			// generate_update_cycles, so its scratch use has nothing left
			// to clobber.
			// TEMPORARY probe (2026-07-27): stash I0 (the value about to be
			// stored, unmodified since this instruction's own RTREG read)
			// and desc->pc into memory now (a plain MOV, doesn't touch I0) -
			// but the actual CALLC to inspect it is deferred to *after* I0's
			// last real use below (the diff computation), since UML_CALLC
			// can clobber scratch registers and I0 must survive intact
			// through the real store and generate_update_mode - the exact
			// clobber-ordering bug class this whole investigation started
			// from. See func_status_store_probe's comment for what this is
			// checking.
			UML_MOV(block, mem(&m_core->diag_new_status), I0);                 // mov     [diag_new_status],i0
			UML_MOV(block, mem(&m_core->diag_status_store_pc), desc->pc);      // mov     [diag_status_store_pc],desc->pc
#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG
			// Hooked right after the stash above and before the real store
			// below - COP0_Status in memory here is still the pre-store
			// value, so func_eret_window_status_diag can report "old" vs
			// "new" (diag_new_status) correctly. This reads diag_new_status
			// as the guest's originally-requested value even if
			// MAME_IRIX_ERET_RACE_WORKAROUND below ends up no-oping the
			// actual store - deliberately, so a suppressed store is still
			// visible in this window's log rather than silently missing.
			UML_CALLC(block, cfunc_eret_window_status_diag, this);             // callc   cfunc_eret_window_status_diag
#endif
			UML_MOV(block, I1, CPR032(COP0_Status));                            // mov     i1,[Status]
#ifdef MAME_IRIX_ERET_RACE_WORKAROUND
			// TEMPORARY WORKAROUND (2026-07-27): if the value about to be
			// stored looks implausible as real Status (top byte 0x88,
			// matching kernel .text - never a real Status bit pattern),
			// substitute the prior value instead of storing garbage - see
			// the macro's own comment near the top of this file for the
			// full mechanism this masks.
			UML_AND(block, I3, I0, 0xff000000);                                 // and     i3,i0,$ff000000
			UML_CMP(block, I3, 0x88000000);                                     // cmp     i3,$88000000
			UML_MOVc(block, COND_E, I0, I1);                                    // mov     i0,i1,e  (no-op the store)
#endif
			UML_MOV(block, CPR032(COP0_Status), I0);                            // mov     [Status],i0
			generate_update_mode(block);                                        // <update mode>
			UML_XOR(block, I0, I0, I1);                                         // xor     i0,i0,i1
			UML_TEST(block, I0, 0x8000);                                        // test    i0,0x8000
			UML_CALLCc(block, COND_NZ, cfunc_mips3com_update_cycle_counting, this);      // callc   mips3com_update_cycle_counting,mips.core,NZ
			// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's
			// comment - hooked on every COP0_Status store anywhere in the
			// kernel, unconditionally, all night.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			UML_CALLC(block, cfunc_status_store_probe, this);                  // callc   cfunc_status_store_probe
#endif
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
#else
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
			UML_MOV(block, I1, CPR032(COP0_Status));                            // mov     i1,[Status]
			UML_MOV(block, CPR032(COP0_Status), I0);                            // mov     [Status],i0
			generate_update_mode(block);                                        // <update mode>
			UML_XOR(block, I0, I0, I1);                                         // xor     i0,i0,i1
			UML_TEST(block, I0, 0x8000);                                        // test    i0,0x8000
			UML_CALLCc(block, COND_NZ, cfunc_mips3com_update_cycle_counting, this);      // callc   mips3com_update_cycle_counting,mips.core,NZ
#endif
			compiler.checkints = true;
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case COP0_Count:
			// don't count the cycle for this instruction yet; we need a non-zero cycle
			// count in case we are in a delay slot, otherwise the test for negative cycles
			// won't be generated (due to compiler.cycles == 0); see the loop during early
			// boot of gauntdl, @BFC01A24
			compiler.cycles--;
#ifdef MAME_DRC_BUGFIXES
			// Bug fix: same generate_update_cycles I0-clobber class as the
			// COP0_Status case above - move the store ahead of the call that
			// can trash I0 before it's consumed. cfunc_get_cycles below
			// already treats I0 as disposable scratch immediately after the
			// store for an unrelated purpose (count_zero_time bookkeeping),
			// so moving the store earlier changes nothing else here.
			UML_MOV(block, CPR032(COP0_Count), I0);                             // mov     [Count],i0
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
#else
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
			UML_MOV(block, CPR032(COP0_Count), I0);                             // mov     [Count],i0
#endif
			UML_CALLC(block, cfunc_get_cycles, this);                           // callc   cfunc_get_cycles,mips3
			UML_DAND(block, I0, I0, 0xffffffff);                                // and     i0,i0,0xffffffff
			UML_DADD(block, I0, I0, I0);                                        // dadd    i0,i0,i0
			UML_DSUB(block, mem(&m_core->count_zero_time), mem(&m_core->numcycles), I0);
																				// dsub    [count_zero_time],[m_numcycles],i0
			UML_CALLC(block, cfunc_mips3com_update_cycle_counting, this);       // callc   mips3com_update_cycle_counting,mips.core
			compiler.cycles++;
			return true;

		case COP0_Compare:
			UML_MOV(block, mem(&m_core->compare_armed), 1);                     // mov     [compare_armed],1
#ifdef MAME_DRC_BUGFIXES
			// Bug fix: same generate_update_cycles I0-clobber class as the
			// COP0_Status case above. Nothing between the store and the end
			// of this case reads I0, so this is a pure reordering.
			UML_MOV(block, CPR032(COP0_Compare), I0);                           // mov     [Compare],i0
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
#else
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
			UML_MOV(block, CPR032(COP0_Compare), I0);                           // mov     [Compare],i0
#endif
			UML_AND(block, CPR032(COP0_Cause), CPR032(COP0_Cause), ~0x8000);    // and     [Cause],[Cause],~0x8000
			UML_CALLC(block, cfunc_mips3com_update_cycle_counting, this);       // callc   mips3com_update_cycle_counting,mips.core
			return true;

		case COP0_PRId:
			return true;

		case COP0_Config:
			UML_ROLINS(block, CPR032(COP0_Config), I0, 0, 0x0007);              // rolins  [Config],i0,0,0x0007
			return true;

		case COP0_EntryHi:
			// FIX (2026-07-30): EntryHi has its own dedicated case that
			// bypasses default entirely. Both MTCz and DMTCz callers now
			// prepare I0 as a correctly-widened 64-bit value (MTCz
			// zero-extends, DMTCz uses the real 64-bit register - see each
			// caller), so this always stores the full 64 bits, matching
			// tlb_map_entry()'s (mips3com.cpp) use of the full VPN2/region-
			// select bits for a 64-bit kernel's XKSEG-style addresses. ASID
			// stays in bits 7:0 regardless of width, so the 0xff change-
			// detection mask is unaffected by the 64-bit compare.
			UML_DXOR(block, I1, I0, CPR064(reg));                               // dxor    i1,i0,cpr0[reg]
			UML_DMOV(block, CPR064(reg), I0);                                   // dmov    cpr0[reg],i0
			UML_DTEST(block, I1, 0xff);                                         // dtest   i1,0xff
			UML_JMPc(block, COND_Z, link = compiler.labelnum++);                // jmp     link,z
			UML_CALLC(block, cfunc_mips3com_asid_changed, this);                // callc   mips3com_asid_changed
			UML_LABEL(block, link);                                             // link:
			return true;

		default:
			// FIX (2026-07-30): both MTCz and DMTCz now prepare I0 as a
			// correctly-widened 64-bit value before reaching here (MTCz
			// zero-extends its 32-bit source, matching the interpreter's
			// set_cop0_reg(reg, RTVAL32) implicitly zero-extending into its
			// uint64_t parameter; DMTCz already used the real 64-bit
			// register) - always store the full 64 bits. Registers that are
			// architecturally 32-bit-only (Status/Cause/Count/Compare/
			// Config/...) have their own dedicated cases above and are
			// unaffected.
			UML_DMOV(block, CPR064(reg), I0);                                   // dmov    cpr0[reg],i0
			return true;
	}
}


/*-------------------------------------------------
    generate_get_cop0_reg - generate code to
    read special COP0 registers
-------------------------------------------------*/

bool mips3_device::generate_get_cop0_reg(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, uint8_t reg, bool is64)
{
	uml::code_label link1, link2;

	switch (reg)
	{
		case COP0_Count:
			// don't count the cycle for this instruction yet; we need a non-zero cycle
			// count in case we are in a delay slot, otherwise the test for negative cycles
			// won't be generated (due to compiler.cycles == 0); see the loop during early
			// boot of gauntdl, @BFC01A24
			compiler.cycles--;
			generate_update_cycles(block, compiler, desc->pc, false);           // <subtract cycles>
			UML_CALLC(block, cfunc_get_cycles, this);                           // callc   cfunc_get_cycles,mips3
			UML_DSUB(block, I0, mem(&m_core->numcycles), mem(&m_core->count_zero_time));
																				// dsub    i0,[numcycles],[count_zero_time]
			UML_DSHR(block, I0, I0, 1);                                         // dshr    i0,i0,1
			UML_DSEXT(block, I0, I0, SIZE_DWORD);                               // dsext   i0,i0,dword
			compiler.cycles++;
			return true;

		case COP0_Random:
			generate_update_cycles(block, compiler, desc->pc, false);           // <subtract cycles>
			UML_CALLC(block, cfunc_get_cycles, this);                           // callc   cfunc_get_cycles,mips3
			UML_DSUB(block, I0, mem(&m_core->numcycles), mem(&m_core->count_zero_time));
																				// dsub    i0,[numcycles],[count_zero_time]
			UML_AND(block, I1, CPR032(COP0_Wired), 0x3f);                       // and     i1,[Wired],0x3f
			UML_SUB(block, I2, 48, I1);                                         // sub     i2,48,i1
			UML_JMPc(block, COND_BE, link1 = compiler.labelnum++);              // jmp     link1,BE
			UML_DAND(block, I2, I2, 0xffffffff);                                // dand    i2,i2,0xffffffff
			UML_DDIVU(block, I0, I2, I0, I2);                                   // ddivu   i0,i2,i0,i2
			UML_ADD(block, I0, I2, I1);                                         // add     i0,i2,i1
			UML_DAND(block, I0, I0, 0x3f);                                      // dand    i0,i0,0x3f
			UML_JMP(block, link2 = compiler.labelnum++);                        // jmp     link2
			UML_LABEL(block, link1);                                            // link1:
			UML_DMOV(block, I0, 47);                                            // dmov    i0,47
			UML_LABEL(block, link2);                                            // link2:
			return true;

		default:
#ifdef MAME_DRC_BUGFIXES
			// Bug fix: see the matching note in generate_set_cop0_reg. DMFCz
			// (64-bit read) was routed through this same default case as
			// MFCz (32-bit), which only ever read the low 32 bits (CPR032)
			// and sign-extended - meaning DMFCz could never observe a
			// register's real upper 32 bits, even those a prior (also
			// buggy, now fixed) DMTCz had genuinely written. MFCz's own
			// caller already re-applies its own 32-bit sign-extend after
			// this call regardless, so leaving its path untouched is safe.
			if (is64)
			{
				UML_DMOV(block, I0, CPR064(reg));                               // dmov    i0,cpr0[reg]
				return true;
			}
#endif
			UML_DSEXT(block, I0, CPR032(reg), SIZE_DWORD);                      // dsext   i0,cpr0[reg],dword
			return true;
	}
}


/*-------------------------------------------------------------------------
    generate_badcop - raise a BADCOP exception
-------------------------------------------------------------------------*/

void mips3_device::generate_badcop(drcuml_block &block, const int cop)
{
	UML_TEST(block, CPR032(COP0_Status), SR_COP0 << cop);               // test    [Status], SR_COP0 << cop
	UML_EXHc(block, COND_Z, *m_exception[EXCEPTION_BADCOP], cop);       // exh     badcop,cop,Z
}

/*-------------------------------------------------------------------------
    check_cop0_access - raise a BADCOP exception if we're not in kernel mode
-------------------------------------------------------------------------*/

void mips3_device::check_cop0_access(drcuml_block &block)
{
	if ((m_core->mode >> 1) != MODE_KERNEL)
	{
		generate_badcop(block, 0);
	}
}

/*-------------------------------------------------
    generate_cop0 - compile COP0 opcodes
-------------------------------------------------*/

bool mips3_device::generate_cop0(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uint32_t op = desc->opptr;
	const uint8_t opswitch = RSREG;
	int skip;

	/* generate an exception if COP0 is disabled unless we are in kernel mode */
#ifdef MAME_DRC_BUGFIXES
	// Bug fix: this was a compile-time C++ "if", reading m_core->mode as a
	// plain build-time snapshot rather than the CPU's real mode at
	// execution time. Once an instruction using this snapshot is compiled
	// and cached, the decision of whether the SR_COP0 check even exists in
	// the generated code is baked in forever, regardless of what mode this
	// exact cached code is later (correctly or incorrectly) reached from.
	// Confirmed live as the root cause of a real, reproducible
	// PANIC: KERNEL FAULT: elocore_exl_3 (the real TLB-miss exception
	// vector)'s own mtc0 to Status clears bit 28 (CU0/SR_COP0) as a side
	// effect of its narrow (status & 0xff00)|1 computation. If this
	// compiled instance of that mtc0 ever picked up the stray SR_COP0
	// gate below (from a mode snapshot that looked non-kernel at compile
	// time, even though real MIPS hardware never needs this check at all
	// in kernel mode), the very next time this same cached code runs it
	// sees the CU0 bit it cleared on its own previous execution and
	// spuriously traps on its own vector entry - a self-inflicted, nested
	// BADCOP exception whose cascade (corrupted EPC/Status/Cause) is what
	// surfaces as the panic. Fixed by testing the real, current mode live
	// (mem(&m_core->mode), the same KSU mask generate_update_mode itself
	// uses) on every execution, skipping the check entirely for kernel
	// mode exactly as real hardware does - not a one-time compile-time
	// decision.
	{
		uml::code_label const cop0_ok = compiler.labelnum++;
		UML_TEST(block, mem(&m_core->mode), 0x06);                          // test    [mode],KSU_MASK
		UML_JMPc(block, COND_Z, cop0_ok);                                   // jz      cop0_ok (kernel - COP0 always usable)
		if (desc->pc == 0x8801205c)
		{
			// TEMPORARY probe (2026-07-27): 0x8801205c is splhi()-style
			// (real mfc0 Status / mask / mtc0, per wiki/assets.md's earlier
			// disassembly), reached from every interrupt-priority-level
			// change throughout the kernel. If this ever legitimately
			// traps here (live mode non-kernel, SR_COP0 clear) it means
			// Status/mode are genuinely wrong at this instant, not just a
			// stale-compiled-mode caching artifact - the gate above already
			// reads mode live via mem(&m_core->mode), so any trap past this
			// point can no longer be explained by the compile-time-baking
			// bug already fixed elsewhere in this file.
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			UML_CALLC(block, cfunc_fault_store_mode_probe_3, this);
#endif
		}
		UML_TEST(block, CPR032(COP0_Status), SR_COP0);                      // test    [Status],SR_COP0
		UML_EXHc(block, COND_Z, *m_exception[EXCEPTION_BADCOP], 0);         // exh     cop,0,Z
		UML_LABEL(block, cop0_ok);                                          // cop0_ok:
	}
#else
	if ((m_core->mode >> 1) != MODE_KERNEL)
	{
		UML_TEST(block, CPR032(COP0_Status), SR_COP0);                          // test    [Status],SR_COP0
		UML_EXHc(block, COND_Z, *m_exception[EXCEPTION_BADCOP], 0);             // exh     cop,0,Z
	}
#endif

	switch (opswitch)
	{
		case 0x00:  /* MFCz */
			if (RTREG != 0)
			{
				generate_get_cop0_reg(block, compiler, desc, RDREG);            // <get cop0 reg>
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);                   // dsext   <rtreg>,i0,dword
			}
			return true;

		case 0x01:  /* DMFCz */
			if (RTREG != 0)
			{
				generate_get_cop0_reg(block, compiler, desc, RDREG, true);      // <get cop0 reg>
				UML_DMOV(block, R64(RTREG), I0);                                // dmov    <rtreg>,i0
			}
			return true;

		case 0x02:  /* CFCz */
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), CCR032(RDREG), SIZE_DWORD);        // dsext   <rtreg>,ccr0[rdreg],dword
			return true;

		case 0x04:  /* MTCz */
			// FIX (2026-07-30): was UML_DSEXT (sign-extend). The interpreter's
			// MTCz calls set_cop0_reg(RDREG, RTVAL32) - RTVAL32 is a uint32_t,
			// implicitly zero-extended into the uint64_t parameter - so a
			// 32-bit mtc0 always zero-extends into the full 64-bit COP0
			// register. Sign-extending here fed a wrong-width value into the
			// now-widened COP0 register writes below (generate_set_cop0_reg's
			// EntryHi and default cases, and this switch is the only source
			// of I0 for a plain MTCz).
			UML_DAND(block, I0, R32(RTREG), 0xffffffff);                        // dand    i0,<rtreg>,0xffffffff
#ifdef MAME_SGI_ANDI_ORI_MTCZ_FAULT_PROBES
			if (desc->pc >= 0x8801b600 && desc->pc < 0x8801b700)
			{
				// TEMPORARY (2026-07-27): stash/restore I0 around the probe
				// CALLC - UML_CALLC can clobber scratch registers, and I0
				// must survive intact into generate_set_cop0_reg right
				// after (the exact clobber-ordering bug class this whole
				// investigation started from).
				UML_DMOV(block, mem(&m_core->diag_mtcz_i0_stash), I0);         // mov     [diag_mtcz_i0_stash],i0
				// Tag with desc->is_branch_target(), captured at THIS
				// specific compile (it varies compile-to-compile for the
				// identical literal PC - confirmed earlier tonight) so a
				// runtime hit can be tied back to which compiled instance
				// (fallthrough-from-andi/ori vs. direct-entry-only)
				// produced it, rather than inferring from log adjacency.
				if (desc->is_branch_target())
					UML_CALLC(block, cfunc_mtcz_read_probe_1, this);            // callc   cfunc_mtcz_read_probe_1
				else
					UML_CALLC(block, cfunc_mtcz_read_probe_0, this);            // callc   cfunc_mtcz_read_probe_0
				UML_DMOV(block, I0, mem(&m_core->diag_mtcz_i0_stash));         // mov     i0,[diag_mtcz_i0_stash]
			}
#endif
			generate_set_cop0_reg(block, compiler, desc, RDREG);                // <set cop0 reg>
			return true;

		case 0x05:  /* DMTCz */
			UML_DMOV(block, I0, R64(RTREG));                                    // dmov    i0,<rtreg>
			generate_set_cop0_reg(block, compiler, desc, RDREG);                // <set cop0 reg>
			return true;

		case 0x06:  /* CTCz */
			UML_DSEXT(block, CCR064(RDREG), R32(RTREG), SIZE_DWORD);            // dsext   ccr0[rdreg],<rtreg>,dword
			return true;

		case 0x10:
		case 0x11:
		case 0x12:
		case 0x13:
		case 0x14:
		case 0x15:
		case 0x16:
		case 0x17:
		case 0x18:
		case 0x19:
		case 0x1a:
		case 0x1b:
		case 0x1c:
		case 0x1d:
		case 0x1e:
		case 0x1f:  /* COP */
			switch (op & 0x01ffffff)
			{
				case 0x01:  /* TLBR */
					UML_CALLC(block, cfunc_mips3com_tlbr, this);                // callc   mips3com_tlbr,mips3
					return true;

				case 0x02:  /* TLBWI */
					UML_CALLC(block, cfunc_mips3com_tlbwi, this);               // callc   mips3com_tlbwi,mips3
					return true;

				case 0x06:  /* TLBWR */
					UML_CALLC(block, cfunc_mips3com_tlbwr, this);               // callc   mips3com_tlbwr,mips3
					return true;

				case 0x08:  /* TLBP */
					UML_CALLC(block, cfunc_mips3com_tlbp, this);                // callc   mips3com_tlbp,mips3
					return true;

				case 0x18:  /* ERET */
#ifdef MAME_SGI_ERET_RACE_WINDOW_DIAG
					// desc->pc is compile-time-known here (this UML is
					// generated once per compiled instance of this ERET
					// instruction), so a plain MOV bakes it in as an
					// immediate. Hooked before Status is touched below, so
					// the probe sees the genuine pre-ERET value.
					UML_MOV(block, mem(&m_core->diag_eret_window_pc), desc->pc);   // mov     [diag_eret_window_pc],desc->pc
					UML_CALLC(block, cfunc_eret_window_eret_diag, this);           // callc   cfunc_eret_window_eret_diag
#endif
					UML_MOV(block, mem(&m_core->llbit), 0);                     // mov     [llbit],0
					UML_MOV(block, I0, CPR032(COP0_Status));                    // mov     i0,[Status]
					UML_TEST(block, I0, SR_ERL);                                // test    i0,SR_ERL
					UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);       // jmp     skip,nz
					UML_AND(block, I0, I0, ~SR_EXL);                            // and     i0,i0,~SR_EXL
					UML_MOV(block, CPR032(COP0_Status), I0);                    // mov     [Status],i0
					generate_update_mode(block);
					compiler.checkints = true;
					generate_update_cycles(block, compiler, CPR032(COP0_EPC), true);// <subtract cycles>
#ifdef MAME_SGI_TASK84_ERET_FASTREG_RELOAD_FIX
					// see the macro's own comment near the top of this file -
					// this hashjmp resumes guest execution directly, bypassing
					// static_generate_entry_point (the only other place fast
					// registers get reloaded), so without this they'd carry
					// over whatever the interrupt/exception handler itself
					// last left in them.
					load_fast_iregs(block);
#endif
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
					{
						// bridge: see the matching comment at the jmpdest
						// call site (generate_delay_slot_and_branch) - same
						// code_label-vs-int mismatch, same reason.
						int labelnum_tmp = compiler.labelnum;
						generate_select_dispatch_key(block, CPR032(COP0_EPC), uml::I3, labelnum_tmp);
						compiler.labelnum = labelnum_tmp;
					}
					UML_HASHJMP(block, I3, CPR032(COP0_EPC), *m_nocode);
																				// hashjmp <mode>,[EPC],nocode
#else
					UML_HASHJMP(block, mem(&m_core->dispatch_mode), CPR032(COP0_EPC), *m_nocode);
																				// hashjmp <mode>,[EPC],nocode
#endif
					UML_LABEL(block, skip);                                     // skip:
					UML_AND(block, I0, I0, ~SR_ERL);                            // and     i0,i0,~SR_ERL
					UML_MOV(block, CPR032(COP0_Status), I0);                    // mov     [Status],i0
					generate_update_mode(block);
					compiler.checkints = true;
					generate_update_cycles(block, compiler, CPR032(COP0_ErrorPC), true); // <subtract cycles>
#ifdef MAME_SGI_TASK84_ERET_FASTREG_RELOAD_FIX
					// see the matching comment at the other exit path above
					load_fast_iregs(block);
#endif
#if defined(MAME_SGI_DISPATCH_MODE_ASID_FIX) && (MAME_SGI_DISPATCH_MODE_ASID_FIX == 2)
					{
						// bridge: see the matching comment at the jmpdest
						// call site (generate_delay_slot_and_branch) - same
						// code_label-vs-int mismatch, same reason.
						int labelnum_tmp = compiler.labelnum;
						generate_select_dispatch_key(block, CPR032(COP0_ErrorPC), uml::I3, labelnum_tmp);
						compiler.labelnum = labelnum_tmp;
					}
					UML_HASHJMP(block, I3, CPR032(COP0_ErrorPC), *m_nocode); // hashjmp <mode>,[EPC],nocode
#else
					UML_HASHJMP(block, mem(&m_core->dispatch_mode), CPR032(COP0_ErrorPC), *m_nocode); // hashjmp <mode>,[EPC],nocode
#endif
					return true;

				case 0x20:  /* WAIT */
					return true;
			}
			break;
	}

	return false;
}



/***************************************************************************
    COP1 RECOMPILATION
***************************************************************************/

/*-------------------------------------------------------------------------
    check_cop1_access - raise a BADCOP exception if COP1 is not enabled
-------------------------------------------------------------------------*/

void mips3_device::check_cop1_access(drcuml_block &block)
{
	if (m_drcoptions & MIPS3DRC_STRICT_COP1)
	{
		generate_badcop(block, 1);
	}
}

/*-------------------------------------------------------
    generate_cop1 - compile COP1 opcodes
---------------------------------------------------------*/

bool mips3_device::generate_cop1(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uint32_t op = desc->opptr;
	uml::code_label skip;
	uml::condition_t condition;

	check_cop1_access(block);

	switch (RSREG)
	{
		case 0x00:  /* MFC1 - MIPS I */
			if (RTREG != 0)
			{
				UML_DSEXT(block, R64(RTREG), FPR32(RDREG), SIZE_DWORD);     // dsext   <rtreg>,fpr[rdreg],dword
			}
			return true;

		case 0x01:  /* DMFC1 - MIPS III */
			if (RTREG != 0)
			{
				UML_DMOV(block, R64(RTREG), FPR64(RDREG));                  // dmov    <rtreg>,fpr[rdreg]
			}
			return true;

		case 0x02:  /* CFC1 - MIPS I */
			if (RTREG != 0)
				UML_DSEXT(block, R64(RTREG), CCR132(RDREG), SIZE_DWORD);        // dsext   <rtreg>,ccr132[rdreg],dword
			return true;

		case 0x04:  /* MTC1 - MIPS I */
			UML_MOV(block, FPR32(RDREG), R32(RTREG));                       // mov     fpr[rdreg],<rtreg>
			return true;

		case 0x05:  /* DMTC1 - MIPS III */
			UML_DMOV(block, FPR64(RDREG), R64(RTREG));                      // dmov    fpr[rdreg],<rtreg>
			return true;

		case 0x06:  /* CTC1 - MIPS I */
			if (RDREG != 31)
			{
				UML_DSEXT(block, CCR164(RDREG), R32(RTREG), SIZE_DWORD);        // dsext   ccr1[rdreg],<rtreg>,dword
			}
			else
			{
				UML_XOR(block, I0, CCR132(31), R32(RTREG));                     // xor     i0,ccr1[31],<rtreg>
				UML_DSEXT(block, CCR164(31), R32(RTREG), SIZE_DWORD);           // dsext   ccr1[31],<rtreg>,dword
				UML_TEST(block, I0, 3);                                         // test    i0,3
				UML_JMPc(block, COND_Z, skip = compiler.labelnum++);            // jmp     skip,Z
				UML_AND(block, I0, CCR132(31), 3);                              // and     i0,ccr1[31],3
				UML_LOAD(block, I0, &m_fpmode[0], I0, SIZE_BYTE, SCALE_x1);     // load   i0,fpmode,i0,byte
				UML_SETFMOD(block, I0);                                         // setfmod i0
#ifdef MAME_SGI_CTC1_MXCSR_CHECK_DIAG
				{
					uml::code_label const ctc1_check_skip = compiler.labelnum++;
					// I0 still holds the just-requested fmod value (SETFMOD's
					// only parameter, not its destination - unclobbered).
					UML_GETFMOD(block, I1);                                          // getfmod i1              (real live MXCSR)
					UML_SHR(block, I1, I1, 13);                                      // shr     i1,i1,13
					UML_AND(block, I1, I1, 3);                                       // and     i1,i1,3         (real RC bits)
					UML_LOAD(block, I2, &s_ctc1_mxcsr_expected_rc[0], I0, SIZE_BYTE, SCALE_x1); // load i2,expected_rc,i0,byte
					UML_CMP(block, I1, I2);                                          // cmp     i1,i2
					UML_JMPc(block, COND_Z, ctc1_check_skip);                        // jmp     skip,Z
					UML_MOV(block, mem(&m_core->diag_ctc1_mxcsr_pc), desc->pc);            // mov     [diag_pc],desc->pc
					UML_MOV(block, mem(&m_core->diag_ctc1_mxcsr_fmod), I0);                // mov     [diag_fmod],i0
					UML_MOV(block, mem(&m_core->diag_ctc1_mxcsr_real_rc), I1);             // mov     [diag_real_rc],i1
					UML_CALLC(block, cfunc_ctc1_mxcsr_check_diag, this);                   // callc   ctc1_mxcsr_check_diag,mips3
					UML_LABEL(block, ctc1_check_skip);                               // skip:
				}
#endif
				UML_LABEL(block, skip);                                         // skip:
			}
			return true;

		case 0x08:  /* BC */
			switch ((op >> 16) & 3)
			{
				case 0x00:  /* BCzF - MIPS I */
				case 0x02:  /* BCzFL - MIPS II */
					UML_TEST(block, CCR132(31), FCCMASK(op >> 18));             // test    ccr1[31],fccmask[which]
					UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);       // jmp     skip,NZ
					generate_delay_slot_and_branch(block, compiler, desc, 0);   // <next instruction + hashjmp>
					UML_LABEL(block, skip);                                     // skip:
					return true;

				case 0x01:  /* BCzT - MIPS I */
				case 0x03:  /* BCzTL - MIPS II */
					UML_TEST(block, CCR132(31), FCCMASK(op >> 18));             // test    ccr1[31],fccmask[which]
					UML_JMPc(block, COND_Z, skip = compiler.labelnum++);        // jmp     skip,Z
					generate_delay_slot_and_branch(block, compiler, desc, 0);   // <next instruction + hashjmp>
					UML_LABEL(block, skip);                                     // skip:
					return true;
			}
			break;

		default:
			switch (op & 0x3f)
			{
				case 0x00:
#ifdef MAME_SGI_FPOP_MXCSR_CHECK_DIAG
					generate_fpop_mxcsr_precheck(block, compiler, desc);
#endif
					if (IS_SINGLE(op))  /* ADD.S - MIPS I */
					{
						UML_FSADD(block, FPR32(FDREG), FPR32(FSREG), FPR32(FTREG));       // fsadd   <fdreg>,<fsreg>,<ftreg>
					}
					else                /* ADD.D - MIPS I */
					{
						UML_FDADD(block, FPR64(FDREG), FPR64(FSREG), FPR64(FTREG));       // fdadd   <fdreg>,<fsreg>,<ftreg>
					}
					return true;

				case 0x01:
#ifdef MAME_SGI_FPOP_MXCSR_CHECK_DIAG
					generate_fpop_mxcsr_precheck(block, compiler, desc);
#endif
					if (IS_SINGLE(op))  /* SUB.S - MIPS I */
					{
						UML_FSSUB(block, FPR32(FDREG), FPR32(FSREG), FPR32(FTREG));       // fssub   <fdreg>,<fsreg>,<ftreg>
					}
					else                /* SUB.D - MIPS I */
					{
						UML_FDSUB(block, FPR64(FDREG), FPR64(FSREG), FPR64(FTREG));       // fdsub   <fdreg>,<fsreg>,<ftreg>
					}
					return true;

				case 0x02:
#ifdef MAME_SGI_FPOP_MXCSR_CHECK_DIAG
					generate_fpop_mxcsr_precheck(block, compiler, desc);
#endif
					if (IS_SINGLE(op))  /* MUL.S - MIPS I */
					{
						UML_FSMUL(block, FPR32(FDREG), FPR32(FSREG), FPR32(FTREG));       // fsmul   <fdreg>,<fsreg>,<ftreg>
					}
					else                /* MUL.D - MIPS I */
					{
						UML_FDMUL(block, FPR64(FDREG), FPR64(FSREG), FPR64(FTREG));       // fdmul    <fdreg>,<fsreg>,<ftreg>
					}
					return true;

				case 0x03:
					if (IS_SINGLE(op))  /* DIV.S - MIPS I */
					{
						UML_TEST(block, FPR32(FTREG), 0xffffffff);                              // test    <ftreg>,-1
						UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);                   // jmp     skip,NZ
						UML_TEST(block, CCR132(31), 1 << (FCR31_ENABLE + FPE_DIV0));            // test    [FCR31], 1 << DIV0_EN
						UML_EXHc(block, COND_NZ, *m_exception[EXCEPTION_FPE], 1 << FPE_DIV0);   // exh     FPE,cop,FPE_DIV0
						UML_LABEL(block, skip);                                                 // skip:
						UML_FSDIV(block, FPR32(FDREG), FPR32(FSREG), FPR32(FTREG));             // fsdiv   <fdreg>,<fsreg>,<ftreg>
					}
					else                /* DIV.D - MIPS I */
					{
						UML_DTEST(block, FPR64(FTREG), 0xffffffff'ffffffffull);                 // dtest    <ftreg>,-1
						UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);                   // jmp     skip,NZ
						UML_TEST(block, CCR132(31), 1 << (FCR31_ENABLE + FPE_DIV0));            // test    [FCR31], 1 << DIV0_EN
						UML_EXHc(block, COND_NZ, *m_exception[EXCEPTION_FPE], 1 << FPE_DIV0);   // exh     FPE,cop,FPE_DIV0
						UML_LABEL(block, skip);                                                 // skip:
						UML_FDDIV(block, FPR64(FDREG), FPR64(FSREG), FPR64(FTREG));             // fddiv   <fdreg>,<fsreg>,<ftreg>
					}
					return true;

				case 0x04:
					if (IS_SINGLE(op))  /* SQRT.S - MIPS II */
					{
						UML_FSSQRT(block, FPR32(FDREG), FPR32(FSREG));                  // fssqrt  <fdreg>,<fsreg>
					}
					else                /* SQRT.D - MIPS II */
					{
						UML_FDSQRT(block, FPR64(FDREG), FPR64(FSREG));                  // fdsqrt  <fdreg>,<fsreg>
					}
					return true;

				case 0x05:
					if (IS_SINGLE(op))  /* ABS.S - MIPS I */
					{
						UML_FSABS(block, FPR32(FDREG), FPR32(FSREG));                   // fsabs   <fdreg>,<fsreg>
					}
					else                /* ABS.D - MIPS I */
					{
						UML_FDABS(block, FPR64(FDREG), FPR64(FSREG));                   // fdabs   <fdreg>,<fsreg>
					}
					return true;

				case 0x06:
					if (IS_SINGLE(op))  /* MOV.S - MIPS I */
					{
						UML_FSMOV(block, FPR32(FDREG), FPR32(FSREG));                   // fsmov   <fdreg>,<fsreg>
					}
					else                /* MOV.D - MIPS I */
					{
						UML_FDMOV(block, FPR64(FDREG), FPR64(FSREG));                   // fdmov   <fdreg>,<fsreg>
					}
					return true;

				case 0x07:
					if (IS_SINGLE(op))  /* NEG.S - MIPS I */
					{
						UML_FSNEG(block, FPR32(FDREG), FPR32(FSREG));         // fsneg   <fdreg>,<fsreg>
						UML_CMP(block, FPR32(FSREG), 0);                      // cmp     <fsreg>,0.0
						UML_MOVc(block, COND_E, FPR32(FDREG), 0x80000000);    // mov     <fdreg>,-0.0,e
					}
					else                /* NEG.D - MIPS I */
					{
						UML_FDNEG(block, FPR64(FDREG), FPR64(FSREG));                         // fdneg   <fdreg>,<fsreg>
						UML_DCMP(block, FPR64(FSREG), 0);                                     // cmp     <fsreg>,0.0
						UML_DMOVc(block, COND_E, FPR64(FDREG), 0x8000000000000000U);          // dmov    <fdreg>,-0.0,e
					}
					return true;

				case 0x08:
					if (IS_SINGLE(op))  /* ROUND.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_ROUND);// fstoint <fdreg>,<fsreg>,qword,round
					}
					else                /* ROUND.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_ROUND);// fdtoint <fdreg>,<fsreg>,qword,round
					}
					return true;

				case 0x09:
					if (IS_SINGLE(op))  /* TRUNC.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_TRUNC);// fstoint <fdreg>,<fsreg>,qword,trunc
					}
					else                /* TRUNC.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_TRUNC);// fdtoint <fdreg>,<fsreg>,qword,trunc
					}
					return true;

				case 0x0a:
					if (IS_SINGLE(op))  /* CEIL.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_CEIL);// fstoint <fdreg>,<fsreg>,qword,ceil
					}
					else                /* CEIL.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_CEIL);// fdtoint <fdreg>,<fsreg>,qword,ceil
					}
					return true;

				case 0x0b:
					if (IS_SINGLE(op))  /* FLOOR.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_FLOOR);// fstoint <fdreg>,<fsreg>,qword,floor
					}
					else                /* FLOOR.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_FLOOR);// fdtoint <fdreg>,<fsreg>,qword,floor
					}
					return true;

				case 0x0c:
					if (IS_SINGLE(op))  /* ROUND.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_ROUND);// fstoint <fdreg>,<fsreg>,dword,round
					}
					else                /* ROUND.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_ROUND);// fdtoint <fdreg>,<fsreg>,dword,round
					}
					return true;

				case 0x0d:
					if (IS_SINGLE(op))  /* TRUNC.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_TRUNC);// fstoint <fdreg>,<fsreg>,dword,trunc
					}
					else                /* TRUNC.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_TRUNC);// fdtoint <fdreg>,<fsreg>,dword,trunc
					}
					return true;

				case 0x0e:
					if (IS_SINGLE(op))  /* CEIL.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_CEIL);// fstoint <fdreg>,<fsreg>,dword,ceil
					}
					else                /* CEIL.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_CEIL);// fdtoint <fdreg>,<fsreg>,dword,ceil
					}
					return true;

				case 0x0f:
					if (IS_SINGLE(op))  /* FLOOR.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_FLOOR);// fstoint <fdreg>,<fsreg>,dword,floor
					}
					else                /* FLOOR.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_FLOOR);// fdtoint <fdreg>,<fsreg>,dword,floor
					}
					return true;

				case 0x11:
					condition = ((op >> 16) & 1) ? uml::COND_NZ : uml::COND_Z;
					UML_TEST(block, CCR132(31), FCCMASK(op >> 18));         // test    ccr31,fccmask[op]
					if (IS_SINGLE(op))  /* MOVT/F.S - MIPS IV */
					{
						UML_FSMOVc(block, condition, FPR32(FDREG), FPR32(FSREG));   // fsmov   <fdreg>,<fsreg>,condition
					}
					else                /* MOVT/F.D - MIPS IV */
					{
						UML_FDMOVc(block, condition, FPR64(FDREG), FPR64(FSREG));   // fdmov   <fdreg>,<fsreg>,condition
					}
					return true;

				case 0x12:
					UML_DCMP(block, R64(RTREG), 0);                             // dcmp    <rtreg>,0
					if (IS_SINGLE(op))  /* MOVZ.S - MIPS IV */
					{
						UML_FSMOVc(block, COND_Z, FPR32(FDREG), FPR32(FSREG));      // fsmov   <fdreg>,<fsreg>,Z
					}
					else                /* MOVZ.D - MIPS IV */
					{
						UML_FDMOVc(block, COND_Z, FPR64(FDREG), FPR64(FSREG));      // fdmov   <fdreg>,<fsreg>,Z
					}
					return true;

				case 0x13:
					UML_DCMP(block, R64(RTREG), 0);                                 // dcmp    <rtreg>,0
					if (IS_SINGLE(op))  /* MOVN.S - MIPS IV */
					{
						UML_FSMOVc(block, COND_NZ, FPR32(FDREG), FPR32(FSREG));         // fsmov   <fdreg>,<fsreg>,NZ
					}
					else                /* MOVN.D - MIPS IV */
					{
						UML_FDMOVc(block, COND_NZ, FPR64(FDREG), FPR64(FSREG));         // fdmov   <fdreg>,<fsreg>,NZ
					}
					return true;

				case 0x15:
					if (IS_SINGLE(op))  /* RECIP.S - MIPS IV */
					{
						UML_FSRECIP(block, FPR32(FDREG), FPR32(FSREG));                 // fsrecip <fdreg>,<fsreg>
					}
					else                /* RECIP.D - MIPS IV */
					{
						UML_FDRECIP(block, FPR64(FDREG), FPR64(FSREG));                 // fdrecip <fdreg>,<fsreg>
					}
					return true;

				case 0x16:
					if (IS_SINGLE(op))  /* RSQRT.S - MIPS IV */
					{
						UML_FSRSQRT(block, FPR32(FDREG), FPR32(FSREG));                 // fsrsqrt <fdreg>,<fsreg>
					}
					else                /* RSQRT.D - MIPS IV */
					{
						UML_FDRSQRT(block, FPR64(FDREG), FPR64(FSREG));                 // fdrsqrt <fdreg>,<fsreg>
					}
					return true;

				case 0x20:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.S.W - MIPS I */
						{
							UML_FSFRINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD); // fsfrint <fdreg>,<fsreg>,dword
						}
						else                /* CVT.S.L - MIPS I */
						{
							UML_FSFRINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_QWORD); // fsfrint <fdreg>,<fsreg>,qword
						}
					}
					else                    /* CVT.S.D - MIPS I */
					{
						UML_FSFRFLT(block, FPR32(FDREG), FPR64(FSREG), SIZE_QWORD);         // fsfrflt <fdreg>,<fsreg>,qword
					}
					return true;

				case 0x21:
					if (IS_INTEGRAL(op))
					{
						if (IS_SINGLE(op))  /* CVT.D.W - MIPS I */
						{
							UML_FDFRINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_DWORD); // fdfrint <fdreg>,<fsreg>,dword
						}
						else                /* CVT.D.L - MIPS I */
						{
							UML_FDFRINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD); // fdfrint <fdreg>,<fsreg>,qword
						}
					}
					else                    /* CVT.D.S - MIPS I */
					{
						UML_FDFRFLT(block, FPR64(FDREG), FPR32(FSREG), SIZE_DWORD);         // fdfrflt <fdreg>,<fsreg>,dword
					}
					return true;

				case 0x24:
					if (IS_SINGLE(op))  /* CVT.W.S - MIPS I */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_DEFAULT);// fstoint <fdreg>,<fsreg>,dword,default
					}
					else                /* CVT.W.D - MIPS I */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_DEFAULT);// fdtoint <fdreg>,<fsreg>,dword,default
					}
					return true;

				case 0x25:
					if (IS_SINGLE(op))  /* CVT.L.S - MIPS I */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_DEFAULT);// fstoint <fdreg>,<fsreg>,qword,default
					}
					else                /* CVT.L.D - MIPS I */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_DEFAULT);// fdtoint <fdreg>,<fsreg>,qword,default
					}
					return true;

				case 0x30:
				case 0x38:              /* C.F.S/D - MIPS I */
					UML_AND(block, CCR132(31), CCR132(31), ~FCCMASK(op >> 8));  // and     ccr31,ccr31,~fccmask[op]
					return true;

				case 0x31:
				case 0x39:
					if (IS_SINGLE(op))  /* C.UN.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.UN.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_U, I0);                                    // set     i0,u
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;

				case 0x32:
				case 0x3a:
					if (IS_SINGLE(op))  /* C.EQ.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.EQ.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_E, I0);                                    // set     i0,e
					UML_SETc(block, COND_NU, I1);                               // set     i1,nu
					UML_AND(block, I0, I0, I1);                     // and     i0,i0,i1
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;

				case 0x33:
				case 0x3b:
					if (IS_SINGLE(op))  /* C.UEQ.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.UEQ.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_U, I0);                                    // set     i0,u
					UML_SETc(block, COND_E, I1);                                    // set     i1,e
					UML_OR(block, I0, I0, I1);                      // or      i0,i0,i1
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;

				case 0x34:
				case 0x3c:
					if (IS_SINGLE(op))  /* C.OLT.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.OLT.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_B, I0);                                    // set     i0,b
					UML_SETc(block, COND_NU, I1);                               // set     i1,nu
					UML_AND(block, I0, I0, I1);                     // and     i0,i0,i1
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;

				case 0x35:
				case 0x3d:
					if (IS_SINGLE(op))  /* C.ULT.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.ULT.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_U, I0);                                    // set     i0,u
					UML_SETc(block, COND_B, I1);                                    // set     i1,b
					UML_OR(block, I0, I0, I1);                      // or      i0,i0,i1
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;

				case 0x36:
				case 0x3e:
					if (IS_SINGLE(op))  /* C.OLE.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.OLE.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_BE, I0);                               // set     i0,be
					UML_SETc(block, COND_NU, I1);                               // set     i1,nu
					UML_AND(block, I0, I0, I1);                     // and     i0,i0,i1
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;

				case 0x37:
				case 0x3f:
					if (IS_SINGLE(op))  /* C.ULE.S - MIPS I */
					{
						UML_FSCMP(block, FPR32(FSREG), FPR32(FTREG));               // fscmp   <fsreg>,<ftreg>
					}
					else                /* C.ULE.D - MIPS I */
					{
						UML_FDCMP(block, FPR64(FSREG), FPR64(FTREG));               // fdcmp   <fsreg>,<ftreg>
					}
					UML_SETc(block, COND_U, I0);                                    // set     i0,u
					UML_SETc(block, COND_BE, I1);                               // set     i1,be
					UML_OR(block, I0, I0, I1);                      // or      i0,i0,i1
					UML_ROLINS(block, CCR132(31), I0, FCCSHIFT(op >> 8), FCCMASK(op >> 8));
																					// rolins  ccr31,i0,fccshift,fcc
					return true;
			}
			break;
	}
	return false;
}

/***************************************************************************
    COP1X RECOMPILATION
***************************************************************************/

/*----------------------------------------------------------
    generate_cop1x - compile COP1X opcodes
----------------------------------------------------------*/

bool mips3_device::generate_cop1x(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const int in_delay_slot = desc->in_delay_slot();
	const uint32_t op = desc->opptr;

	check_cop1_access(block);

	switch (op & 0x3f)
	{
		case 0x00:      /* LWXC1 - MIPS IV */
			UML_ADD(block, I0, R32(RSREG), R32(RTREG));         // add     i0,<rsreg>,<rtreg>
			UML_CALLH(block, *m_read32[m_core->mode >> 1]);     // callh   read32
			UML_MOV(block, FPR32(FDREG), I0);                   // mov     <cpr1_fd>,i0
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x01:      /* LDXC1 - MIPS IV */
			UML_ADD(block, I0, R32(RSREG), R32(RTREG));         // add     i0,<rsreg>,<rtreg>
			UML_CALLH(block, *m_read64[m_core->mode >> 1]);     // callh   read64
			UML_DMOV(block, FPR64(FDREG), I0);                  // dmov    <cpr1_fd>,i0
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x08:      /* SWXC1 - MIPS IV */
			UML_ADD(block, I0, R32(RSREG), R32(RTREG));         // add     i0,<rsreg>,<rtreg>
			UML_MOV(block, I1, FPR32(FSREG));                   // mov     i1,<cpr1_fs>
			UML_CALLH(block, *m_write32[m_core->mode >> 1]);    // callh   write32
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x09:      /* SDXC1 - MIPS IV */
			UML_ADD(block, I0, R32(RSREG), R32(RTREG));         // add     i0,<rsreg>,<rtreg>
			UML_DMOV(block, I1, FPR64(FSREG));                  // dmov    i1,<cpr1_fs>
			UML_CALLH(block, *m_write64[m_core->mode >> 1]);    // callh   write64
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x0f:      /* PREFX */
			return true;

		case 0x20:      /* MADD.S - MIPS IV */
			UML_FSMUL(block, F0, FPR32(FSREG), FPR32(FTREG));           // fsmul   f0,<fsreg>,<ftreg>
			UML_FSADD(block, FPR32(FDREG), F0, FPR32(FRREG));           // fsadd   <fdreg>,f0,<frreg>
			return true;

		case 0x21:      /* MADD.D - MIPS IV */
			UML_FDMUL(block, F0, FPR64(FSREG), FPR64(FTREG));           // fdmul   f0,<fsreg>,<ftreg>
			UML_FDADD(block, FPR64(FDREG), F0, FPR64(FRREG));           // fdadd   <fdreg>,f0,<frreg>
			return true;

		case 0x28:      /* MSUB.S - MIPS IV */
			UML_FSMUL(block, F0, FPR32(FSREG), FPR32(FTREG));           // fsmul   f0,<fsreg>,<ftreg>
			UML_FSSUB(block, FPR32(FDREG), F0, FPR32(FRREG));           // fssub   <fdreg>,f0,<frreg>
			return true;

		case 0x29:      /* MSUB.D - MIPS IV */
			UML_FDMUL(block, F0, FPR64(FSREG), FPR64(FTREG));           // fdmul   f0,<fsreg>,<ftreg>
			UML_FDSUB(block, FPR64(FDREG), F0, FPR64(FRREG));           // fdadd   <fdreg>,f0,<frreg>
			return true;

		case 0x30:      /* NMADD.S - MIPS IV */
			UML_FSMUL(block, F0, FPR32(FSREG), FPR32(FTREG));           // fsmul   f0,<fsreg>,<ftreg>
			UML_FSADD(block, F0, F0, FPR32(FRREG));                     // fsadd   f0,f0,<frreg>
			UML_FSNEG(block, FPR32(FDREG), F0);                         // fsneg   <fdreg>,f0
			return true;

		case 0x31:      /* NMADD.D - MIPS IV */
			UML_FDMUL(block, F0, FPR64(FSREG), FPR64(FTREG));           // fdmul   f0,<fsreg>,<ftreg>
			UML_FDADD(block, F0, F0, FPR64(FRREG));                     // fdadd   f0,f0,<frreg>
			UML_FDNEG(block, FPR64(FDREG), F0);                         // fdneg   <fdreg>,f0
			return true;

		case 0x38:      /* NMSUB.S - MIPS IV */
			UML_FSMUL(block, F0, FPR32(FSREG), FPR32(FTREG));           // fsmul   f0,<fsreg>,<ftreg>
			UML_FSSUB(block, FPR32(FDREG), FPR32(FRREG), F0);           // fssub   <fdreg>,<frreg>,f0
			return true;

		case 0x39:      /* NMSUB.D - MIPS IV */
			UML_FDMUL(block, F0, FPR64(FSREG), FPR64(FTREG));           // fdmul   f0,<fsreg>,<ftreg>
			UML_FDSUB(block, FPR64(FDREG), FPR64(FRREG), F0);           // fdsub   <fdreg>,<frreg>,f0
			return true;

		default:
			fprintf(stderr, "cop1x %X\n", op);
			break;
	}
	return false;
}

/***************************************************************************
    CODE LOGGING HELPERS
***************************************************************************/

/*-------------------------------------------------
    log_add_disasm_comment - add a comment
    including disassembly of a MIPS instruction
-------------------------------------------------*/

void mips3_device::log_add_disasm_comment(drcuml_block &block, uint32_t pc, uint32_t op)
{
	if (m_drcuml->logging())
	{
		mips3_disassembler mips3d;
		std::ostringstream stream;
		mips3d.dasm_one(stream, pc, op);
		const std::string stream_string = stream.str();
		block.append_comment("%08X: %s", pc, stream_string.c_str());                                  // comment
	}
}


/*-------------------------------------------------
    log_desc_flags_to_string - generate a string
    representing the instruction description
    flags
-------------------------------------------------*/

const char *mips3_device::log_desc_flags_to_string(opcode_desc const &desc)
{
	static char tempbuf[30];
	char *dest = tempbuf;

	/* branches */
	if (desc.is_unconditional_branch())
		*dest++ = 'U';
	else if (desc.is_conditional_branch())
		*dest++ = 'C';
	else
		*dest++ = '.';

	/* intrablock branches */
	*dest++ = desc.intrablock_branch() ? 'i' : '.';

	/* branch targets */
	*dest++ = desc.is_branch_target() ? 'B' : '.';

	/* delay slots */
	*dest++ = desc.in_delay_slot() ? 'D' : '.';

	/* exceptions */
	if (desc.will_cause_exception())
		*dest++ = 'E';
	else if (desc.can_cause_exception())
		*dest++ = 'e';
	else
		*dest++ = '.';

	/* read/write */
	if (desc.reads_memory())
		*dest++ = 'R';
	else if (desc.writes_memory())
		*dest++ = 'W';
	else
		*dest++ = '.';

	/* TLB validation */
	*dest++ = desc.validate_tlb() ? 'V' : '.';

	/* TLB modification */
	*dest++ = desc.modifies_translation() ? 'T' : '.';

	/* redispatch */
	*dest++ = desc.redispatch() ? 'R' : '.';
	return tempbuf;
}


/*-------------------------------------------------
    log_register_list - log a list of GPR registers
-------------------------------------------------*/

void mips3_device::log_register_list(const char *string, const std::bitset<68> &reglist, const std::bitset<68> *regnostarlist)
{
	int count = 0;

	/* skip if nothing */
	if (reglist.none())
		return;

	m_drcuml->log_printf("[%s:", string);

	for (int regnum = 1; regnum < 32; regnum++)
	{
		if (reglist[opcode_desc::REG_FLAG_R0 + regnum])
		{
			m_drcuml->log_printf("%sr%d", (count++ == 0) ? "" : ",", regnum);
			if (regnostarlist && !(*regnostarlist)[opcode_desc::REG_FLAG_R0 + regnum])
				m_drcuml->log_printf("*");
		}
	}

	for (int regnum = 0; regnum < 32; regnum++)
	{
		if (reglist[opcode_desc::REG_FLAG_CPR10 + regnum])
		{
			m_drcuml->log_printf("%sfr%d", (count++ == 0) ? "" : ",", regnum);
			if (regnostarlist && !(*regnostarlist)[opcode_desc::REG_FLAG_CPR10 + regnum])
				m_drcuml->log_printf("*");
		}
	}

	if (reglist[opcode_desc::REG_FLAG_LO])
	{
		m_drcuml->log_printf("%slo", (count++ == 0) ? "" : ",");
		if (regnostarlist && !(*regnostarlist)[opcode_desc::REG_FLAG_LO])
			m_drcuml->log_printf("*");
	}
	if (reglist[opcode_desc::REG_FLAG_HI])
	{
		m_drcuml->log_printf("%shi", (count++ == 0) ? "" : ",");
		if (regnostarlist && !(*regnostarlist)[opcode_desc::REG_FLAG_HI])
			m_drcuml->log_printf("*");
	}
	if (reglist[opcode_desc::REG_FLAG_FCC])
	{
		m_drcuml->log_printf("%sfcc", (count++ == 0) ? "" : ",");
		if (regnostarlist && !(*regnostarlist)[opcode_desc::REG_FLAG_FCC])
			m_drcuml->log_printf("*");
	}

	m_drcuml->log_printf("] ");
}


/*-------------------------------------------------
    log_opcode_desc - log a list of descriptions
-------------------------------------------------*/

void mips3_device::log_opcode_desc(const opcode_desc *desclist, int indent)
{
	/* open the file, creating it if necessary */
	if (indent == 0)
		m_drcuml->log_printf("\nDescriptor list @ %08X\n", desclist->pc);

	/* output each descriptor */
	for ( ; desclist != nullptr; desclist = desclist->next())
	{
		std::ostringstream buffer;

		/* disassemle the current instruction and output it to the log */
		if (m_drcuml->logging() || m_drcuml->logging_native())
		{
			if (desclist->virtual_noop())
				buffer << "<virtual nop>";
			else
			{
				mips3_disassembler mips3d;
				mips3d.dasm_one(buffer, desclist->pc, desclist->opptr);
			}
		}
		else
			buffer << "???";

		const std::string buffer_string = buffer.str();
		m_drcuml->log_printf("%08X [%08X] t:%08X f:%s: %-30s", desclist->pc, desclist->physpc, desclist->targetpc, log_desc_flags_to_string(*desclist), buffer_string.c_str());

		/* output register states */
		log_register_list("use", desclist->regin, nullptr);
		log_register_list("mod", desclist->regout, &desclist->regreq);
		m_drcuml->log_printf("\n");

		/* if we have a delay slot, output it recursively */
		if (desclist->delay.first() != nullptr)
			log_opcode_desc(desclist->delay.first(), indent + 1);

		/* at the end of a sequence add a dividing line */
		if (desclist->end_sequence())
			m_drcuml->log_printf("-----\n");
	}
}
