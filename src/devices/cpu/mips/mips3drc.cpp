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

// MAME_SGI_COP1_CHECK_ONCE: emit the STRICT_COP1 usability test (Status
// CU1) once per sequence instead of before every FP instruction; see
// check_cop1_access(). 0 restores the per-instruction test.
#ifndef MAME_SGI_COP1_CHECK_ONCE
#define MAME_SGI_COP1_CHECK_ONCE 1
#endif

// Own sub-macro (2026-09-11) for the null-seqlast fallback's reporting
// only - the fallback itself stays unconditional under MAME_DRC_BUGFIXES
// above regardless of this. See the fallback's own comment in
// code_compile_block() and diag_null_seqlast_count's comment in mips3.h.
// #define MAME_DRC_BUGFIXES_NULL_SEQLAST_DIAG


// TEMPORARY diagnostic (2026-08-01): validating a theory about USER-mode
// (real IRIX application) code recompilation during multi-process
// operation - see mips3.h's comment on m_diag_usermode_recompile_counts
// for the full mechanism/hypothesis. Counts recompile events per
// (mode, pc), USER mode only, dumped at clean exit from mips3.cpp's
// device_stop(). RESTORED (2026-08-17) - stock-vs-modified A/B concluded - at the
// time judged "cheap... safe to leave on for a normal test run" on an
// amortized-cost basis (one hash-map insert per compile).
// DISABLED, THEN RE-ENABLED WITH A FIX (2026-09-17, QoS
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
// CORRELATION_DIAG's ~11-13ms "long compile" outliers - the other,
// found by describe_code()/histogram/gdb-backtrace follow-up, was
// m_checkints_addrs (a per-PC set of the since-removed in-sequence
// interrupt-check heuristic).
// Re-enabled after adding a one-time reserve() at the insert site (see
// its own comment below) that converts the periodic rehash stalls into
// one predictable upfront allocation - verified via boot test that
// MAME_SGI_STALL_DURATION_DIAG's over_10ms_threshold stays at 0 with
// this diagnostic active again.
// #define MAME_DRC_USERMODE_RECOMPILE_DIAG

// TEMPORARY diagnostic (2026-10-09, recompile study): classifies
// every USER-mode compile request by cause - first compile of its key, after
// a full cache flush, hash miss without a flush, rejected by the checksum
// guard (nocode exit with the block still registered), or rejected by the
// TLB guard (tlb_mismatch exit) - and whether the code content differs from
// that key's previous compile. Also records each distinct code variant per
// pc (hash + first 8 opcode words, for offline file attribution) and the
// physical pages seen. Costs only on the compile path, plus one store on the
// nocode/tlb_mismatch exits. Written as TSV at exit to
// $MAME_SGI_RECOMPILE_STUDY_OUT (default recompile_study.tsv). Also counts
// tlb_mismatch skip-path arrivals by source (validate check, stub embedded
// in another block, stub at its own block head). Optional per-event traces:
// $MAME_SGI_RECOMPILE_STUDY_PCS (comma-separated hex pcs; compiles, failed
// validate checks and handler arrivals near them, with the compile that
// produced each stub) and $MAME_SGI_RECOMPILE_STUDY_WINDOW ("lo-hi" range
// of compile serial numbers, every compile in it). Trace files are written
// next to the TSV with .trace/.validate/.handler suffixes.
// #define MAME_SGI_RECOMPILE_STUDY_DIAG

// TEMPORARY diagnostic (2026-10-10): when $MAME_SGI_SYSCALL_LOG
// names a file, log every SYSCALL exception there - total cycles, ASID,
// syscall number (v0) and a0-a3, plus the path string for calls whose
// first or second argument is a path - to find what a guest process sleeps
// in. Costs a C call per syscall when built in.
// #define MAME_SGI_SYSCALL_LOG_DIAG

// Diagnostic (2026-10-09): report the DRC hash table's footprint
// just before every cache flush and at exit - populated dispatch modes and
// L2 tables in use, split into shared keys (mode < 8) and per-ASID keys
// (MAME_SGI_DISPATCH_MODE_ASID_FIX composites), plus the L1/L2 tables
// allocated so far, which is the permanent drc_cache footprint the tables
// cost (each table is 256KB). Cheap: runs a handful of times per boot.
// #define MAME_SGI_HASH_TABLE_STATS_DIAG

// Repair blocks that embed a stale compiler_page_fault() stub.
// describe_code() walks backwards as well as forwards from the compile pc,
// so a block can embed stubs for instructions on a neighbouring page that
// was unmapped at compile time. Once that page is mapped, every pass
// through the block that reaches the stub (local branch, fall-through,
// delay slot, or the same sequence) exits to tlb_mismatch, which only
// recompiles the stub's own pc - the containing block keeps its stale stub
// and the next pass repeats the exit. With this defined, each stub records
// its containing block's entry (pc and hash key); when it fires on a page
// that is now mapped, code_compile_block() recompiles that container too,
// so later entries through the hash table reach fresh code.
#define MAME_SGI_STALE_STUB_FIX


// TEMPORARY FIX, DISABLED BY DEFAULT (2026-08-01, refined 2026-08-02) -
// see the full comment at its use site, generate_update_mode()
// (mips3drc.cpp). Addresses USER-mode cross-process cache thrashing.
// Widens the DRC dispatch key to include ASID, fixing cross-process
// cache collisions for private/writable code while leaving kernel
// code's ASID-immunity intact.
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
//                exactly this signal to decide what's safe to share).
// RESTORED (2026-08-06/07): the clean-baseline check (undefined) confirmed
// ~103.5s to /.oldtmp, matching the original ~92.7s baseline - conclusively
// showing the regression tracked back to this mechanism, not the two-page-
// vs-one-page distinction (single-page A/B was ALSO slow, 415s). Real
// cause found: page_shareable()'s FETCH_ALLOWED requirement was applied to
// BOTH pages in dispatch_page_writable()/generate_select_dispatch_key(),
// not just the delay-slot page that actually needs it - see
// own_page_needs_asid()'s comment in mips3.h. Fixed there; this stays 2.
// RESTORED to 2 (2026-08-17) after the ASID-vs-symptom A/B test:
// busy-loop reproduced identically with this disabled, confirming it's
// not the cause (consistent with the earlier -nodrc result) - back to
// the normal, validated-correctness-fix state.
//
// cc1 <built-in> flake bisection (2026-09-02): tested undefined - 18/100 failures,
// within this project's own established run-to-run noise band, same as
// every other flag tested so far. Restored to 2 (the normal,
// validated-correctness-fix state).
// Flake forced-trigger retest (2026-09-02): undefined and tested
// against the new 100%-reliable native-gdb repro trigger (not just the
// earlier ~30% natural rate) - 30/30 failures, no change. Restored to 2.
// Flake forced-trigger retest (2026-09-02): undefined and tested
// against the new 100%-reliable native-gdb repro trigger (not just the
// earlier ~30% natural rate) - 30/30 failures, no change. Restored to 2.
// Flake natural-rate retest (2026-09-03): undefined again and tested
// against the NATURAL (non-forced) rate this time - 30 iterations,
// 3/30 failures (10%, one consecutive burst at i=19-21) - lower than
// the established ~29-36% baseline with this fix enabled, and the same
// bursty/clustered failure shape either way. Disabling this fix does
// not eliminate the bug and maximal recompile churn does not increase
// its rate - rules out both "this fix's mere presence causes the bug"
// and "more compile churn causes more failures" as explanations.
// Restored to 2.
// Flake natural-rate retest w/ passive polling (2026-09-03, later
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


// Diagnostic (off): for each PC in a watched range, log how many times
// code_compile_block() has compiled it in this process, with the raw mode
// argument and dispatch_page_writable(pc) at each call, to see whether
// repeat compiles of one PC coincide with its page's writable state
// changing between calls (another process's mapping temporarily in the
// same VTLB slot). Writes a stderr line on every watched compile.
// #define MAME_SGI_RECOMPILE_TRACKING


#include "emu.h"
#include "mame_sgi_framecost.h"
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
#include <map>
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
// info present - a genuinely different failure
// mode than the earlier "no debug info at all" gap, needing a different
// fix. Undefined by default - only enable for a deliberate host-gdb
// investigation session, not as a standing default (compile-time DRC
// machinery runs measurably slower unoptimized, most noticeable during
// boot's heavy initial compile burst).
// #define MAME_SGI_HOST_DEBUG_NOOPT
#ifdef MAME_SGI_HOST_DEBUG_NOOPT
#pragma GCC optimize("O0")
#endif


// Fix (2026-09-17, QoS follow-up) - same cross-TU gotcha as
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

// THE FIX (2026-08-22; reworked 2026-08-23) -
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
// - keeps every state one line
// away without needing to rediscover this comment block to revert.
//
// cc1 <built-in> flake bisection (2026-09-02): tested undefined - 16/100 failures,
// within this project's own established run-to-run noise band, same as
// every other flag tested in this bisection. Restored to 1 (the normal
// state).
// Flake forced-trigger retest (2026-09-02): undefined and tested
// against the new 100%-reliable native-gdb repro trigger (not just the
// earlier ~30% natural rate) - 30/30 failures, no change. Restored to 1.
#define MAME_SGI_TLB_MISMATCH_REFRESH_FIX 1


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

	for (regnum = 0; regnum < std::size(m_regmap); regnum++)
		if (m_regmap[regnum].is_int_register())
			UML_DMOV(block, mem(&m_core->r[regnum]), ireg(m_regmap[regnum].ireg() - REG_I0));

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

void mips3_device::set_idle_pc(offs_t pc, uint32_t opcode)
{
	m_idle_pc = pc;
	m_idle_opcode = opcode;
	m_drc_cache_dirty = true;	// recompiled, with the idle PC, on the next execute_run
}

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

void mips3_device::report_hash_stats(char const *when)
{
#ifndef MAME_SGI_HASH_TABLE_STATS_DIAG
	(void)when;
	return;
#else
	drcbe_interface::hash_stats_t st;
	m_drcuml->hash_stats(st);
	uint32_t shared_modes = 0, asid_modes = 0, shared_l2 = 0, asid_l2 = 0, asid_l2_max = 0;
	uint32_t hist[5] = {};  // per-ASID mode L2 counts: 1, 2, 3-4, 5-8, >8
	for (auto const &[mode, l2] : st.modes)
	{
		if (mode < 8)
		{
			shared_modes++;
			shared_l2 += l2;
		}
		else
		{
			asid_modes++;
			asid_l2 += l2;
			asid_l2_max = std::max(asid_l2_max, l2);
			hist[l2 <= 1 ? 0 : l2 == 2 ? 1 : l2 <= 4 ? 2 : l2 <= 8 ? 3 : 4]++;
		}
	}
	osd_printf_info("MAME_SGI_HASH_TABLE_STATS (%s, flush_count=%u): shared modes=%u L2=%u | asid modes=%u L2=%u (max per mode %u; per-mode L2 1:%u 2:%u 3-4:%u 5-8:%u >8:%u) | allocated L1=%u L2=%u = %.1fMB\n",
		when, m_drcuml->cache().flush_count(), shared_modes, shared_l2, asid_modes, asid_l2, asid_l2_max,
		hist[0], hist[1], hist[2], hist[3], hist[4],
		unsigned(st.l1_allocated), unsigned(st.l2_allocated), double(st.table_bytes) / (1024.0 * 1024.0));

	// density of the L2 tables in use: entry points per 128KB slice, distinct
	// 4KB pages holding entries, and where the slices are
	uint64_t entries = 0, pages = 0;
	uint32_t dens[5] = {};  // entries per L2: 1-8, 9-64, 65-512, 513-4096, >4096
	std::map<std::pair<uint32_t, uint32_t>, std::array<uint32_t, 3>> region;  // (mode, pc>>24) -> L2s, entries, pages
	std::set<std::pair<uint32_t, uint32_t>> mb;  // distinct (mode, 1MB region)
	for (auto const &l : st.l2s)
	{
		entries += l[2];
		pages += l[3];
		dens[l[2] <= 8 ? 0 : l[2] <= 64 ? 1 : l[2] <= 512 ? 2 : l[2] <= 4096 ? 3 : 4]++;
		auto &r = region[{ l[0], l[1] >> 24 }];
		r[0]++; r[1] += l[2]; r[2] += l[3];
		mb.insert({ l[0], l[1] >> 20 });
	}
	if (st.l2s.empty())
		return;
	osd_printf_info("MAME_SGI_HASH_TABLE_STATS density: %u L2s, %llu entries (%.1f per L2 of 32768), %llu distinct 4KB pages; L2s by entries 1-8:%u 9-64:%u 65-512:%u 513-4096:%u >4096:%u; distinct 1MB regions %u\n",
		unsigned(st.l2s.size()), (unsigned long long)entries, double(entries) / st.l2s.size(), (unsigned long long)pages,
		dens[0], dens[1], dens[2], dens[3], dens[4], unsigned(mb.size()));
	osd_printf_info("MAME_SGI_HASH_TABLE_STATS slices needed by L2 size: 10b(4KB)=%llu 11b(8KB)=%llu 12b(16KB)=%llu 13b(32KB)=%llu 14b(64KB)=%llu 15b(128KB)=%llu\n",
		(unsigned long long)st.slices_by_l2bits[0], (unsigned long long)st.slices_by_l2bits[1], (unsigned long long)st.slices_by_l2bits[2],
		(unsigned long long)st.slices_by_l2bits[3], (unsigned long long)st.slices_by_l2bits[4], (unsigned long long)st.slices_by_l2bits[5]);
	for (auto const &[k, r] : region)
		osd_printf_info("MAME_SGI_HASH_TABLE_STATS region mode=%u pc=%02x000000: L2s=%u entries=%u pages=%u\n", k.first, k.second, r[0], r[1], r[2]);
#endif
}

void mips3_device::code_flush_cache()
{
	MAME_SGI_FRAME_COST(MSFC_FLUSH);
	int mode;

#ifdef MAME_SGI_HASH_TABLE_STATS_DIAG
	report_hash_stats("before flush");
#endif

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
			static_generate_exception(block, label, EXCEPTION_BUSDATA,       true,  "exception_busdata");
			static_generate_exception(block, label, EXCEPTION_SYSCALL,       true,  "exception_syscall");
			static_generate_exception(block, label, EXCEPTION_BREAK,         true,  "exception_break");
			static_generate_exception(block, label, EXCEPTION_INVALIDOP,     true,  "exception_invalidop");
			static_generate_exception(block, label, EXCEPTION_BADCOP,        true,  "exception_badcop");
			static_generate_exception(block, label, EXCEPTION_OVERFLOW,      true,  "exception_overflow");
			static_generate_exception(block, label, EXCEPTION_TRAP,          true,  "exception_trap");
			static_generate_exception(block, label, EXCEPTION_FPE,           true,  "exception_fpe");

			block.end();
		}


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

// pcs listed in $MAME_SGI_RECOMPILE_STUDY_PCS (comma-separated hex) get a
// per-event trace; `near` also accepts the pages either side of a listed pc
bool mips3_device::recompile_study_traced(uint32_t pc, bool near)
{
	static std::vector<uint32_t> s_pcs;
	static bool s_parsed = false;
	if (!s_parsed)
	{
		s_parsed = true;
		if (char const *env = getenv("MAME_SGI_RECOMPILE_STUDY_PCS"))
		{
			while (*env)
			{
				char *end = nullptr;
				unsigned long const v = strtoul(env, &end, 16);
				if (end == env)
					break;
				s_pcs.push_back(uint32_t(v));
				env = (*end == ',') ? end + 1 : end;
			}
		}
	}
	for (uint32_t const t : s_pcs)
	{
		if (t == pc)
			return true;
		if (near && ((pc >> 12) + 1 >= (t >> 12)) && ((pc >> 12) <= (t >> 12) + 1))
			return true;
	}
	return false;
}

void mips3_device::recompile_study_record(uint32_t key_mode, offs_t pc, const opcode_desc *desclist)
{
	enum { FIRST, FLUSH, MISS_NOFLUSH, CHECKSUM, TLB };
	uint32_t const origin = m_core->diag_recompile_origin;
	m_core->diag_recompile_origin = 0;
	uint32_t const flush = m_drcuml->cache().flush_count();
	bool const entry_exists = m_drcuml->hash_exists(key_mode, pc);

	// FNV-1a over every opcode in the window; first 8 words from pc onward
	uint64_t hash = 1469598103934665603ULL;
	uint32_t words[8] = {};
	unsigned nwords = 0;
	uint32_t physpc = 0;
	uint32_t stub_pc = 0;
	uint32_t last_desc_pc = 0;
	unsigned ndesc = 0;
	uint64_t own_hash = 1469598103934665603ULL;
	bool stub_at_pc = false;
	auto own = [&own_hash, pc](const opcode_desc *x)
	{
		if ((x->pc >> 12) != (pc >> 12))
			return;
		uint32_t const v = x->compiler_page_fault() ? 0xfffffffeU : uint32_t(x->opptr);
		for (int b = 0; b < 4; b++)
			own_hash = (own_hash ^ ((v >> (b * 8)) & 0xff)) * 1099511628211ULL;
	};
	for (const opcode_desc *d = desclist; d != nullptr; d = d->next())
	{
		last_desc_pc = d->pc;
		ndesc++;
		own(d);
		for (const opcode_desc *ds = d->delay.first(); ds != nullptr; ds = ds->next())
			own(ds);
		if (d->pc == pc && d->compiler_page_fault())
			stub_at_pc = true;
		uint32_t const op = d->opptr;
		if (d->compiler_page_fault() && stub_pc == 0)
			stub_pc = d->pc;
		for (const opcode_desc *ds = d->delay.first(); ds != nullptr; ds = ds->next())
			if (ds->compiler_page_fault() && stub_pc == 0)
				stub_pc = ds->pc;
		for (int b = 0; b < 4; b++)
			hash = (hash ^ ((op >> (b * 8)) & 0xff)) * 1099511628211ULL;
		if (d->pc == pc)
			physpc = d->physpc;
		if (nwords < 8 && d->pc == pc + nwords * 4)
			words[nwords++] = op;
	}

	m_recompile_study_serial++;
	m_recompile_study_cur_serial = m_recompile_study_serial;
	if (m_recompile_study_log.size() <= m_recompile_study_serial)
		m_recompile_study_log.resize(m_recompile_study_serial + 1);
	m_recompile_study_log[m_recompile_study_serial] = { uint32_t(pc), m_core->dispatch_mode, total_cycles() };
	recompile_study_key &k = m_recompile_study_keys[(uint64_t(key_mode) << 32) | pc];
	unsigned cls;
	if (k.count == 0)
		cls = FIRST;
	else if (origin == 2)
		cls = TLB;
	else if (entry_exists)
		cls = CHECKSUM;
	else if (flush != k.last_flush)
		cls = FLUSH;
	else
		cls = MISS_NOFLUSH;
	bool const changed = (k.count != 0) && (hash != k.last_hash);
	unsigned tlb_kind = 0, tlb_outcome = 0;
	if (cls == TLB)
	{
		tlb_kind = m_recompile_study_last_kind;
		tlb_outcome = k.last_was_stub ? 0 : !changed ? 1 : (own_hash == k.last_own_hash) ? 2 : 3;
		m_recompile_study_tlb[tlb_kind][tlb_outcome]++;
	}
	m_recompile_study_last_kind = 3;
	m_recompile_study_bucket = cls == FIRST ? (stub_at_pc ? 1 : 0) : cls == FLUSH ? 2 : cls == TLB ? (tlb_kind == 0 ? 3 : tlb_kind == 2 ? 4 : 5) : 5;
	k.count++;
	k.last_flush = flush;
	k.last_hash = hash;
	k.last_own_hash = own_hash;
	k.last_physpc = physpc;
	k.last_was_stub = stub_at_pc;

	m_recompile_study_totals[cls][changed]++;
	recompile_study_pc &p = m_recompile_study_pcs[pc];
	p.cls[cls][changed]++;
	if (cls == TLB)
		p.tlb[tlb_kind][tlb_outcome]++;
	if (key_mode == m_core->mode)
		p.shared_key++;
	else
		p.asid_key++;
	if (p.requesters.size() < 4096)
		p.requesters.insert(m_core->dispatch_mode);
	bool found = false;
	for (auto &v : p.variants)
		if (v.hash == hash) { v.count++; found = true; break; }
	if (!found && p.variants.size() < 16)
	{
		recompile_study_variant v{hash, 1, {}};
		std::copy(std::begin(words), std::end(words), v.words);
		p.variants.push_back(v);
	}
	if (std::find(p.physpcs.begin(), p.physpcs.end(), physpc) == p.physpcs.end() && p.physpcs.size() < 16)
		p.physpcs.push_back(physpc);
	if (stub_pc != 0)
	{
		if ((stub_pc >> 12) == (pc >> 12))
			p.stub_same_page++;
		else
			p.stub_next_page++;
	}

	// time-ordered trace: the first 300 events at pcs listed in MAME_SGI_RECOMPILE_STUDY_PCS,
	// plus every compile in the serial window from MAME_SGI_RECOMPILE_STUDY_WINDOW ("lo-hi")
	static uint32_t s_win_lo = 1, s_win_hi = 0;
	static bool s_win_parsed = false;
	if (!s_win_parsed)
	{
		s_win_parsed = true;
		if (char const *const w = getenv("MAME_SGI_RECOMPILE_STUDY_WINDOW"))
			sscanf(w, "%u-%u", &s_win_lo, &s_win_hi);
	}
	bool const in_window = m_recompile_study_serial >= s_win_lo && m_recompile_study_serial <= s_win_hi;
	if (in_window || recompile_study_traced(pc, false))
	{
		static unsigned s_traced = 0;
		static FILE *s_trace = nullptr;
		if (s_traced < 300 || in_window)
		{
			if (!s_trace)
			{
				char const *const env = getenv("MAME_SGI_RECOMPILE_STUDY_OUT");
				std::string const path = std::string(env ? env : "recompile_study.tsv") + ".trace";
				s_trace = fopen(path.c_str(), "w");
			}
			if (s_trace)
			{
				vtlb_entry const *const t = vtlb_table();
				fprintf(s_trace, "%llu\tserial=%u\tpc=%08x\treq=%u\tcls=%u\torigin=%u\tstub=%08x\tvtlb_pc=%08x\tvtlb_next=%08x\twin=%u\tdesc_first=%08x\tdesc_last=%08x\tndesc=%u\n",
					(unsigned long long)total_cycles(), m_recompile_study_serial, pc, m_core->dispatch_mode, cls, origin, stub_pc,
					t[pc >> 12], t[(pc >> 12) + 1], unsigned(nwords), desclist ? desclist->pc : 0, last_desc_pc, ndesc);
				fflush(s_trace);
				s_traced++;
			}
		}
	}
}

void mips3_device::recompile_study_validate_probe()
{
	// runs at a traced pc's validate_tlb check: arg0 = pc, arg1 = entry baked
	// at compile time, diag_study_serial = compile serial of the containing block
	static unsigned s_logged = 0;
	uint32_t const pc = m_core->arg0;
	uint32_t const baked = m_core->arg1;
	uint32_t const live = vtlb_table()[pc >> 12];
	if (live == baked || s_logged >= 400)
		return;
	char const *const env = getenv("MAME_SGI_RECOMPILE_STUDY_OUT");
	static FILE *s_f = nullptr;
	if (!s_f)
		s_f = fopen((std::string(env ? env : "recompile_study.tsv") + ".validate").c_str(), "w");
	if (s_f)
	{
		fprintf(s_f, "%llu\tVALIDATE_MISMATCH\tpc=%08x\tbaked=%08x\tlive=%08x\tblock_serial=%u\tcurrent_serial=%u\treq=%u\n",
			(unsigned long long)total_cycles(), pc, baked, live, m_core->diag_study_serial, m_recompile_study_serial, m_core->dispatch_mode);
		fflush(s_f);
		s_logged++;
	}
}

[[maybe_unused]] static void cfunc_recompile_study_validate_probe(void *param)
{
	((mips3_device *)param)->recompile_study_validate_probe();
}

void mips3_device::recompile_study_tlb_handler_probe()
{
	// tlb_mismatch handler, after the page-mapped (skip) test: m_core->pc = recovered pc
	static unsigned s_logged = 0;
	uint32_t const pc = m_core->pc;
	{
		// global attribution of every arrival at the page-mapped (skip) path
		uint32_t const site = m_core->arg0 <= 3 ? m_core->arg0 : 0;
		if (site == 1)
		{
			uint32_t const cs = m_core->diag_study_serial;
			bool const embedded = cs < m_recompile_study_log.size() && m_recompile_study_log[cs].pc != pc;
			m_recompile_study_handler[embedded ? 1 : 2]++;
			m_recompile_study_last_kind = embedded ? 3 : 2;
		}
		else if (site == 0)
		{
			m_recompile_study_handler[0]++;
			uint32_t const baked = m_core->diag_study_baked;
			uint32_t const live = vtlb_table()[pc >> 12];
			m_recompile_study_last_kind = ((baked ^ live) & ~0xfffU) ? 0 : 1;
			m_recompile_study_flag_xor[(baked ^ live) & 0xfff]++;
		}
		else
		{
			m_recompile_study_handler[3]++;
			m_recompile_study_last_kind = 3;
		}
	}
	bool const in_range = recompile_study_traced(pc, true);
	if (!in_range || s_logged >= 400)
	{
		m_core->arg0 = 0xffffffff;
		return;
	}
	char const *const env = getenv("MAME_SGI_RECOMPILE_STUDY_OUT");
	static FILE *s_f = nullptr;
	if (!s_f)
		s_f = fopen((std::string(env ? env : "recompile_study.tsv") + ".handler").c_str(), "w");
	if (s_f)
	{
		vtlb_entry const *const t = vtlb_table();
		fprintf(s_f, "%llu\tTLB_HANDLER\tpc=%08x\tlive_pc=%08x\tlive_next=%08x\tretry_pc=%08x\tserial=%u\tsite=%u\tsite_pc=%08x\n",
			(unsigned long long)total_cycles(), pc, t[pc >> 12], t[(pc >> 12) + 1], m_core->tlb_mismatch_retry_pc, m_recompile_study_serial,
			m_core->arg0 <= 3 ? m_core->arg0 : 0, m_core->arg0 <= 3 ? m_core->arg1 : 0);
		if (m_core->arg0 == 1)
		{
			uint32_t const cs = m_core->diag_study_serial;
			if (cs < m_recompile_study_log.size())
				fprintf(s_f, "\tstub_from_block\tserial=%u\tentry_pc=%08x\tcompiled_at=%llu\treq=%u\n", cs,
					m_recompile_study_log[cs].pc, (unsigned long long)m_recompile_study_log[cs].cycles, m_recompile_study_log[cs].req);
		}
		fflush(s_f);
		s_logged++;
	}
	m_core->arg0 = 0xffffffff;
}

[[maybe_unused]] static void cfunc_recompile_study_tlb_handler_probe(void *param)
{
	((mips3_device *)param)->recompile_study_tlb_handler_probe();
}

void mips3_device::syscall_log_diag()
{
	static FILE *s_f = nullptr;
	static bool s_init = false;
	if (!s_init)
	{
		s_init = true;
		if (char const *const p = getenv("MAME_SGI_SYSCALL_LOG"))
		{
			s_f = fopen(p, "w");
			if (s_f)
				setvbuf(s_f, nullptr, _IOFBF, 1 << 16);
		}
	}
	if (!s_f)
		return;
	uint32_t const num = uint32_t(m_core->r[2]);
	auto guest_string = [this] (uint64_t vaddr, char *out, size_t len)
	{
		size_t i = 0;
		for (; i + 1 < len; i++)
		{
			offs_t a = offs_t(vaddr + i);
			address_space *sp;
			if (!memory_translate(AS_PROGRAM, device_memory_interface::TR_READ, a, sp))
				break;
			char const c = char(sp->read_byte(a));
			if (!c)
				break;
			out[i] = (c >= 32 && c < 127) ? c : '.';
		}
		out[i] = 0;
	};
	char path[96] = "";
	switch (num)
	{
	case 1005: case 1010: case 1011: case 1015: case 1018: case 1033: case 1059: case 1116: // open unlink execv chmod stat access execve readlink
		guest_string(m_core->r[4], path, sizeof(path));
		break;
	case 1158: case 1159: // xstat lxstat: path in a1
		guest_string(m_core->r[5], path, sizeof(path));
		break;
	}
	// IRIX IP22 (32-bit kernel, 4KB pages): the PDA is at 0xffffa000 and
	// pda_s.p_curuthread is the word at offset 20 - unique per live thread,
	// which separates processes that end up sharing an ASID
	uint32_t uthread = 0;
	{
		offs_t a = 0xffffa014;
		address_space *sp;
		if (memory_translate(AS_PROGRAM, device_memory_interface::TR_READ, a, sp))
			uthread = sp->read_dword(a);
	}
	fprintf(s_f, "%llu %02x/%08x %u %llx %llx %llx %llx %s\n", (unsigned long long)total_cycles(),
		uint32_t(m_core->cpr[0][COP0_EntryHi] & 0xff), uthread, num,
		(unsigned long long)m_core->r[4], (unsigned long long)m_core->r[5],
		(unsigned long long)m_core->r[6], (unsigned long long)m_core->r[7], path);
}

[[maybe_unused]] static void cfunc_syscall_log_diag(void *param)
{
	((mips3_device *)param)->syscall_log_diag();
}

void mips3_device::recompile_study_dump()
{
	if (m_recompile_study_pcs.empty())
		return;
	char const *const env = getenv("MAME_SGI_RECOMPILE_STUDY_OUT");
	std::string const path = env ? env : "recompile_study.tsv";
	FILE *f = fopen(path.c_str(), "w");
	if (!f)
		return;
	static char const *const names[5] = { "first", "flush", "miss_noflush", "checksum", "tlb" };
	uint64_t total = 0;
	for (auto &c : m_recompile_study_totals)
		total += c[0] + c[1];
	fprintf(f, "# tlb_mismatch skip-path arrivals: validate(site2)=%llu stub_embedded_in_other_block=%llu stub_as_own_block_head=%llu site3=%llu\n",
		(unsigned long long)m_recompile_study_handler[0], (unsigned long long)m_recompile_study_handler[1],
		(unsigned long long)m_recompile_study_handler[2], (unsigned long long)m_recompile_study_handler[3]);
	fprintf(f, "# recompile study: %llu USER-mode compile requests, %u distinct pcs, %u distinct keys, flushes=%u\n",
		(unsigned long long)total, unsigned(m_recompile_study_pcs.size()), unsigned(m_recompile_study_keys.size()),
		m_drcuml->cache().flush_count());
	for (int c = 0; c < 5; c++)
		fprintf(f, "# class %-12s same_content=%llu changed_content=%llu\n", names[c],
			(unsigned long long)m_recompile_study_totals[c][0], (unsigned long long)m_recompile_study_totals[c][1]);
	static char const *const kinds[4] = { "validate_phys", "validate_flags", "stub_own_head", "stub_other" };
	fprintf(f, "# tlb recompiles by source x outcome: prev_was_stub window_same own_page_same own_page_changed\n");
	for (int k = 0; k < 4; k++)
		fprintf(f, "# tlb %-15s %llu %llu %llu %llu\n", kinds[k],
			(unsigned long long)m_recompile_study_tlb[k][0], (unsigned long long)m_recompile_study_tlb[k][1],
			(unsigned long long)m_recompile_study_tlb[k][2], (unsigned long long)m_recompile_study_tlb[k][3]);
	static char const *const buckets[7] = { "first", "first_stub", "flush", "tlb_validate_phys", "tlb_after_stub", "other_user", "kernel" };
	for (int b = 0; b < 7; b++)
		fprintf(f, "# host time %-18s n=%llu ms=%.1f us_each=%.1f\n", buckets[b], (unsigned long long)m_recompile_study_n[b],
			m_recompile_study_ns[b] / 1e6, m_recompile_study_n[b] ? m_recompile_study_ns[b] / 1e3 / m_recompile_study_n[b] : 0.0);
	for (auto const &[x, n] : m_recompile_study_flag_xor)
		fprintf(f, "# validate mismatch flag xor %03x: %llu\n", x, (unsigned long long)n);
	fprintf(f, "PC\tpc\ttotal\tshared_key\tasid_key\trequesters\tvariants\tphyspcs\tstub_same_page\tstub_next_page");
	for (int c = 0; c < 5; c++)
		fprintf(f, "\t%s_same\t%s_changed", names[c], names[c]);
	for (int k = 0; k < 4; k++)
		fprintf(f, "\t%s_stub\t%s_same\t%s_neigh\t%s_own", kinds[k], kinds[k], kinds[k], kinds[k]);
	fprintf(f, "\n");
	for (auto const &[pc, p] : m_recompile_study_pcs)
	{
		uint32_t const t = p.shared_key + p.asid_key;
		if (t < 2)
			continue;
		fprintf(f, "PC\t%08x\t%u\t%u\t%u\t%u\t%u\t%u\t%u\t%u", pc, t, p.shared_key, p.asid_key,
			unsigned(p.requesters.size()), unsigned(p.variants.size()), unsigned(p.physpcs.size()),
			p.stub_same_page, p.stub_next_page);
		for (int c = 0; c < 5; c++)
			fprintf(f, "\t%u\t%u", p.cls[c][0], p.cls[c][1]);
		for (int k = 0; k < 4; k++)
			for (int o = 0; o < 4; o++)
				fprintf(f, "\t%u", p.tlb[k][o]);
		fprintf(f, "\n");
		for (auto const &v : p.variants)
		{
			fprintf(f, "VAR\t%08x\t%016llx\t%u", pc, (unsigned long long)v.hash, v.count);
			for (uint32_t w : v.words)
				fprintf(f, "\t%08x", w);
			fprintf(f, "\n");
		}
		fprintf(f, "PHYS\t%08x", pc);
		for (uint32_t ph : p.physpcs)
			fprintf(f, "\t%08x", ph);
		fprintf(f, "\n");
	}
	fclose(f);
	osd_printf_info("MAME_SGI_RECOMPILE_STUDY_DIAG: %llu compile requests written to %s\n", (unsigned long long)total, path.c_str());
}

void mips3_device::code_compile_block(uint32_t mode, offs_t pc)
{
	MAME_SGI_FRAME_COST(MSFC_COMPILE);
	compiler_state compiler = { 0 };
	const opcode_desc *seqhead, *seqlast;
	const opcode_desc *codelast;
	const opcode_desc *desclist;
	bool override = false;

#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
	// host time of each top-level request, by the bucket the (last) record call chose
	struct study_timer
	{
		mips3_device &d;
		bool const top;
		std::chrono::steady_clock::time_point const t0;
		study_timer(mips3_device &dev) : d(dev), top(dev.m_recompile_study_depth++ == 0), t0(std::chrono::steady_clock::now())
		{
			if (top)
				d.m_recompile_study_bucket = 6;
		}
		~study_timer()
		{
			d.m_recompile_study_depth--;
			if (!top)
				return;
			uint64_t const ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - t0).count();
			d.m_recompile_study_ns[d.m_recompile_study_bucket] += ns;
			d.m_recompile_study_n[d.m_recompile_study_bucket]++;
		}
	} study_timer_scope(*this);
#endif


	// Standing feature (2026-08-07, not a temporary diagnostic): report
	// drc_cache health (flush count, permanent-cache utilization) the
	// moment a new flush happens, not only when the process exits
	// cleanly. A plain kill -9 - this project's usual way of ending a
	// test run - skips drc_cache's own destructor entirely, so a real
	// cache-thrashing regression can run for an entire session without
	// ever becoming visible; this is exactly what happened investigating
	// the ASID_FIX==2 regression (2026-08-07: 997 flushes by
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


// Standing counter report for MAME_SGI_TLB_MISMATCH_REFRESH_FIX (
// diag_tlb_mismatch_skip_total/refresh_hits, see their comment in mips3.h)
// used to live here as a self-rate-limited live print every time the
// totals changed meaningfully. Moved to a single clean-exit summary in
// mips3_device::device_stop() instead (2026-09-11), matching the same
// hygiene convention applied to other diagnostics the same day -
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
		// Fix (2026-09-17, QoS follow-up) - see this macro's own
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


	if (m_drc_asid_dispatch)
	{
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
		// pending that.
		bool const writable_now = dispatch_page_writable(pc);
#ifdef MAME_SGI_RECOMPILE_TRACKING
		if (pc >= 0x1007de00 && pc <= 0x1007e100)
		{
			static std::unordered_map<uint32_t, uint32_t> s_recompile_counts;
			uint32_t const count = ++s_recompile_counts[pc];
			fprintf(stderr,
				"[RECOMPILE_TRACKING] pc=%08x raw_mode=%u writable_now=%d final_mode=%u count=%u%s\n",
				pc, mode, writable_now, writable_now ? mode : m_core->mode, count,
				(count > 1) ? " DUPLICATE" : "");
			fflush(stderr);
		}
#endif
		if (!writable_now)
			mode = m_core->mode;

#ifdef MAME_SGI_STALE_STUB_FIX
		// See MAME_SGI_STALE_STUB_FIX: a stub in another block sent us here, and
		// its page is mapped now - recompile that container from its own entry
		// so it stops carrying the stale stub. Only when its hash key is the
		// shared key or the current process's own, so it is compiled against the
		// mappings it will run under.
		if (m_core->stub_site_pc == pc)
		{
			uint32_t const container_pc = m_core->stub_container_pc;
			uint32_t const container_mode = m_core->stub_container_mode;
			m_core->stub_site_pc = 0xffffffff;
			if (container_pc != pc && (container_mode == m_core->mode || container_mode == m_core->dispatch_mode) &&
					(vtlb_table()[container_pc >> 12] & FETCH_ALLOWED))
			{
				m_stale_stub_container_recompiles++;
				code_compile_block(container_mode, container_pc);
			}
		}
		m_compile_entry_pc = pc;
		m_compile_entry_mode = mode;
#endif

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
		// reason to expect it might work this time. Left as dead code, not deleted, per this
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
	}

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
	// wrong assumption from scratch. A real fix here would need
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
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
	if (((m_core->dispatch_mode >> 2) != 0) && ((m_core->dispatch_mode & 0x02) != 0))
		recompile_study_record(mode, pc, desclist);
	else
		m_core->diag_recompile_origin = 0;
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
					// permanent fix's own block.
					//
					// Was: unconditional per-event fprintf with a running
					// count, "remove once the frequency/pattern is
					// understood" (2026-07-23). It now is - three-way
					// comparison the same week confirmed the fallback is
					// genuinely rare (zero fires across an 11+ minute
					// live run) - so reworked
					// 2026-09-11 to a silent counter, reported once as a
					// clean-exit summary in device_stop() instead of
					// per-event, matching
					// MAME_SGI_TLB_MISMATCH_REFRESH_FIX's same-day rework.
#ifdef MAME_DRC_BUGFIXES_NULL_SEQLAST_DIAG
					m_core->diag_null_seqlast_count++;
					m_core->diag_null_seqlast_last_pc = seqhead->pc;
#endif
					seqlast = seqhead;
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


				/* if we don't have a hash for this mode/pc, or if we are overriding all, add one */
				if (override || !m_drcuml->hash_exists(mode, seqhead->pc))
				{
					UML_HASH(block, mode, seqhead->pc);                                     // hash    mode,pc

				}
				/* if we already have a hash, and this is the first sequence, assume that we */
				/* are recompiling due to being out of sync and allow future overrides */
				else if (seqhead == desclist)
				{
					override = true;
					UML_HASH(block, mode, seqhead->pc);                                     // hash    mode,pc

				}

				/* otherwise, redispatch to that fixed PC and skip the rest of the processing */
				else
				{
					UML_LABEL(block, seqhead->pc | 0x80000000);                             // label   seqhead->pc | 0x80000000
					if (m_drc_asid_dispatch)
					{
						UML_HASHJMP(block, static_dispatch_key(seqhead->pc), seqhead->pc, *m_nocode);   // hashjmp <mode>,seqhead->pc,nocode
					}
					else
					{
						UML_HASHJMP(block, m_core->dispatch_mode, seqhead->pc, *m_nocode);               // hashjmp <mode>,seqhead->pc,nocode
					}
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
				compiler.cop1_checked = false;
				for (curdesc = seqhead; curdesc != seqlast->next(); curdesc = curdesc->next())
				{
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
					if (m_drc_asid_dispatch)
					{
						UML_HASHJMP(block, live_dispatch_key(nextpc), nextpc, *m_nocode);              // hashjmp <mode>,nextpc,nocode
					}
					else
					{
						UML_HASHJMP(block, mem(&m_core->dispatch_mode), nextpc, *m_nocode);              // hashjmp <mode>,nextpc,nocode
					}
				}
				else if (seqlast->next() == nullptr || seqlast->next()->pc != nextpc)
				{
					if (m_drc_asid_dispatch)
					{
						UML_HASHJMP(block, static_dispatch_key(nextpc), nextpc, *m_nocode);                    // hashjmp <mode>,nextpc,nocode
					}
					else
					{
						UML_HASHJMP(block, m_core->dispatch_mode, nextpc, *m_nocode);                    // hashjmp <mode>,nextpc,nocode
					}
				}
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


// MAME_SGI_FP_CONVERT_FIX: see fp_convert_fixup()
static void cfunc_fp_convert_fixup(void *param)
{
	((mips3_device *)param)->fp_convert_fixup();
}

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
    cfunc_printf_exception - log any exceptions that
    aren't interrupts
-------------------------------------------------*/

void mips3_device::func_printf_exception()
{
	printf("Exception: EPC=%08X Cause=%08X BadVAddr=%08X Jmp=%08X\n", (uint32_t)m_core->cpr[0][COP0_EPC], (uint32_t)m_core->cpr[0][COP0_Cause], (uint32_t)m_core->cpr[0][COP0_BadVAddr], m_core->pc);
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


	/* reset the FPU mode */
	UML_AND(block, I0, CCR132(31), 3);                                          // and     i0,ccr1[31],3
	UML_LOAD(block, I0, &m_fpmode[0], I0, SIZE_BYTE, SCALE_x1);                 // load    i0,fpmode,i0,byte
	UML_SETFMOD(block, I0);                                                     // setfmod i0


	/* load fast integer registers */
	load_fast_iregs(block);

	/* check for interrupts */
	UML_AND(block, I0, CPR032(COP0_Cause), CPR032(COP0_Status));                // and     i0,[Cause],[Status]
	UML_AND(block, I0, I0, 0xfc00);                                             // and     i0,i0,0xfc00,Z
	UML_JMPc(block, COND_Z, skip);                                              // jmp     skip,Z
	UML_TEST(block, CPR032(COP0_Status), SR_IE);                                // test    [Status],SR_IE
	UML_JMPc(block, COND_Z, skip);                                              // jmp     skip,Z
	UML_TEST(block, CPR032(COP0_Status), SR_EXL | SR_ERL);                      // test    [Status],SR_EXL | SR_ERL
	UML_JMPc(block, COND_NZ, skip);                                             // jmp     skip,NZ
	UML_MOV(block, I0, mem(&m_core->pc));                                       // mov     i0,pc
	UML_MOV(block, I1, 0);                                                      // mov     i1,0
	UML_CALLH(block, *m_exception_norecover[EXCEPTION_INTERRUPT]);              // callh   exception_norecover
	UML_LABEL(block, skip);                                                     // skip:

	/* generate a hash jump via the current mode and PC */
	if (m_drc_asid_dispatch)
	{
		generate_select_dispatch_key(block, uml::mem(&m_core->pc), uml::I3, label);
		UML_HASHJMP(block, I3, mem(&m_core->pc), *m_nocode);        // hashjmp <mode>,<pc>,nocode
	}
	else
	{
		UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->pc), *m_nocode);        // hashjmp <mode>,<pc>,nocode
	}
}


/*-------------------------------------------------
    static_generate_nocode_handler - generate an
    exception handler for "out of code"
-------------------------------------------------*/

// MAME_SGI_NOCODE_REFRESH_FIX (2026-09-16): static_dispatch_key()
// bakes its resolved dispatch key as a compile-time constant at the jumping
// instruction's own compile time. If the target page's writability
// classification later changes (routine under ASID switches, since
// dispatch_page_writable() is a per-page WRITE_ALLOWED check), that baked
// key goes stale and the HASHJMP misses here even though valid code already
// exists under the CURRENT resolved key - the shared nocode handler had no
// way to tell "genuinely missing" apart from "stale key, code exists
// elsewhere", so it unconditionally paid a full recompile either way.
// Measured before this fix: 99.985%
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
	save_fast_iregs(block);

#ifdef MAME_SGI_NOCODE_REFRESH_FIX
	UML_ADD(block, mem(&m_core->diag_nocode_skip_total), mem(&m_core->diag_nocode_skip_total), 1);

	UML_CMP(block, I0, mem(&m_core->nocode_retry_pc));                          // cmp     i0,[nocode_retry_pc]
	UML_JMPc(block, COND_E, no_retry);                                          // jmp     no_retry,e
	UML_MOV(block, mem(&m_core->nocode_retry_pc), I0);                          // mov     [nocode_retry_pc],i0
	UML_ADD(block, mem(&m_core->diag_nocode_refresh_hits), mem(&m_core->diag_nocode_refresh_hits), 1);
	UML_SUB(block, mem(&m_core->icount), mem(&m_core->icount), 1);              // sub     [icount],[icount],1
	UML_EXHc(block, COND_S, *m_out_of_cycles, I0);                              // exh     out_of_cycles,i0,s
	if (m_drc_asid_dispatch)
	{
		generate_select_dispatch_key(block, uml::mem(&m_core->pc), uml::I3, label);
		UML_HASHJMP(block, I3, mem(&m_core->pc), *m_nocode);                        // hashjmp <mode>,[pc],nocode
	}
	else
	{
		UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->pc), *m_nocode); // hashjmp <mode>,[pc],nocode
	}

	UML_LABEL(block, no_retry);                                                 // no_retry:
	// Disarm, so a later, genuinely unrelated miss at this same PC gets its
	// own retry rather than being permanently suppressed.
	UML_MOV(block, mem(&m_core->nocode_retry_pc), 0xffffffff);                  // mov     [nocode_retry_pc],~0
#endif
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
	UML_MOV(block, mem(&m_core->diag_recompile_origin), 1);                     // mov     [diag_recompile_origin],1
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
	UML_GETEXP(block, I0);                                                      // getexp  i0
	UML_MOV(block, mem(&m_core->pc), I0);                                       // mov     <pc>,i0
	save_fast_iregs(block);
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

	// the saved PC may be set 1 instruction back with the low bit set to indicate
	// a delay slot; in this path we want the previous instruction address, so recover it
	UML_RECOVER(block, I0, MAPVAR_PC);                                          // recover i0,PC
	UML_AND(block, I0, I0, ~3);                                                 // and     i0,i0,~3
	UML_MOV(block, mem(&m_core->pc), I0);                                       // mov     i0,[pc]
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
	UML_CALLC(block, cfunc_recompile_study_tlb_handler_probe, this);
	UML_MOV(block, I0, mem(&m_core->pc));
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
	if (m_drc_asid_dispatch)
	{
		generate_select_dispatch_key(block, uml::mem(&m_core->pc), uml::I3, label);
		UML_HASHJMP(block, I3, mem(&m_core->pc), *m_nocode);                        // hashjmp <mode>,[pc],nocode
	}
	else
	{
		UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->pc), *m_nocode); // hashjmp <mode>,[pc],nocode
	}

	UML_LABEL(block, no_retry);                                                 // no_retry:
	// Disarm, so a later, genuinely unrelated mismatch at this same PC
	// gets its own retry rather than being permanently suppressed.
	UML_MOV(block, mem(&m_core->tlb_mismatch_retry_pc), 0xffffffff);            // mov     [tlb_mismatch_retry_pc],~0
#endif
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
	UML_MOV(block, mem(&m_core->diag_recompile_origin), 2);                     // mov     [diag_recompile_origin],2
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

#ifdef MAME_SGI_SYSCALL_LOG_DIAG
	if (exception == EXCEPTION_SYSCALL)
	{
		save_fast_iregs(block);
		UML_CALLC(block, cfunc_syscall_log_diag, this);
	}
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
	// reordering). TEST must be
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
	}

	// TEMPORARY probe (2026-07-27): stash the EPC candidate (I0, still valid
	// here - nothing since <next> has touched it) and check whether Status
	// already looks address-shaped *entering* this delivery, before this
	// function's own SR_EXL OR below touches it at all. See
	// func_interrupt_delivery_probe's own comment for the full rationale.

	/* set EXL in the SR */
	UML_OR(block, I0, CPR032(COP0_Status), SR_EXL);                             // or      i0,[Status],SR_EXL
	UML_MOV(block, CPR032(COP0_Status), I0);                                    // mov     [Status],i0
	generate_update_mode(block);


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
	// values instead of what the interrupted code actually had live.
	save_fast_iregs(block);

	if (m_drc_asid_dispatch)
	{
		generate_select_dispatch_key(block, uml::I0, uml::I3, label);	UML_HASHJMP(block, I3, I0, *m_nocode);                      // hashjmp <mode>,i0,nocode
	}
	else
	{
		UML_HASHJMP(block, mem(&m_core->dispatch_mode), I0, *m_nocode);                      // hashjmp <mode>,i0,nocode
	}
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
		int addrok;
		UML_TEST(block, I0, 0x80000000);                                            // test    i0,0x80000000
		UML_JMPc(block, COND_Z, addrok = label++);                                  // jz      addrok
		UML_SHR(block, I3, I0, 29);                                                 // shr     i3,i0,29
		UML_CMP(block, I3, 6);                                                      // cmp     i3,6
		UML_EXHc(block, COND_NE, exception_addrerr, I0);                            // exh     addrerr,i0,ne
		UML_LABEL(block, addrok);                                                   // addrok:
	}
}


/*------------------------------------------------------------------
    static_generate_fastram_accessor
------------------------------------------------------------------*/

void mips3_device::static_generate_fastram_accessor(drcuml_block &block, int &label, int size, bool iswrite, bool ismasked)
{
	// This function is shared MIPS3-core code, used by every
	// driver that calls add_fastram() (not just this one) - its
	// byte-lane-swizzle math (below) must match the CPU's ACTUAL program
	// address space width, not assume either 32-bit or 64-bit unilaterally.
	// A 32-bit-native mips3_device (the more common MIPS3 configuration)
	// needs the original scale (no swizzle at size==4, a half-swap DROR at
	// size==8); this SGI driver's genuinely 64-bit-native space needs the
	// opposite (see the per-size comments below; a live mismatch on
	// 2026-09-30 found this).
	bool const native64 = m_program->data_width() == 64;
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
					// A fast-path/slow-path comparison found this
					// fastram region is natively 64-bit (Width=3 - confirmed
					// via the read_dword() dispatch trace, 2026-09-30),
					// not 32-bit. Every branch below
					// needs the "...to a 64-bit space" XOR variant
					// (BYTE8_XOR_BE/WORD2_XOR_BE/DWORD_XOR_BE in
					// endianness.h) to pick the right byte lane within the
					// native 8-byte unit - the size==1/2 branches previously
					// used BYTE4_XOR_BE/WORD_XOR_BE (the "...to a 32-bit
					// space" variants, wrong scale for this buffer), and
					// size==4 had no swizzle at all. Getting size==1/2's
					// scale wrong is self-consistent for a same-size
					// write-then-read round trip (a fixed wrong permutation
					// still round-trips with itself), which is why it never
					// showed up as a correctness bug on its own - it only
					// disagrees with a DIFFERENT access size/path touching
					// the same bytes, exactly the shape of this investigation's
					// original fast/slow mismatch.
					if (size == 1)
					{
						UML_XOR(block, I0, I0, m_bigendian ? (native64 ? BYTE8_XOR_BE(0) : BYTE4_XOR_BE(0))
															: (native64 ? BYTE8_XOR_LE(0) : BYTE4_XOR_LE(0)));
																					// xor     i0,i0,bytexor
						UML_LOAD(block, I0, fastbase, I0, SIZE_BYTE, SCALE_x1);     // load    i0,fastbase,i0,byte
					}
					else if (size == 2)
					{
						UML_XOR(block, I0, I0, m_bigendian ? (native64 ? WORD2_XOR_BE(0) : WORD_XOR_BE(0))
															: (native64 ? WORD2_XOR_LE(0) : WORD_XOR_LE(0)));
																					// xor     i0,i0,wordxor
						UML_LOAD(block, I0, fastbase, I0, SIZE_WORD, SCALE_x1);     // load    i0,fastbase,i0,word_x1
					}
					else if (size == 4)
					{
						// A dword access exactly matches the native width of
						// a 32-bit-native space (no lane to select - the
						// original, untouched behavior below), but is only
						// HALF the native width of this SGI driver's
						// genuinely 64-bit-native one, needing the
						// DWORD_XOR_BE/LE lane swizzle added for it.
						if (native64)
							UML_XOR(block, I0, I0, m_bigendian ? DWORD_XOR_BE(0) : DWORD_XOR_LE(0));
						UML_LOAD(block, I0, fastbase, I0, SIZE_DWORD, SCALE_x1);    // load    i0,fastbase,i0,dword_x1
					}
					else if (size == 8)
					{
						// The same comparison (widened
						// pass, 2026-09-30): confirmed via a live mismatch
						// (addr=0x0832d8a8, size=8:
						// fast=0xbfbb000bffffffff slow=0xffffffffbfbb000b)
						// that for a 64-bit-native space, this DROR must NOT
						// run - rotating fast's halves back (undoing the
						// DROR by 32) gives exactly slow, i.e. the RAW host
						// load already equals the correct value with no
						// rotation at all. For a 32-bit-native space,
						// though, a qword access is TWICE the native width
						// - the original DROR-by-32 half-swap (combining
						// two native 32-bit lanes into one 64-bit value) is
						// still exactly what that case needs, so it's kept
						// here rather than removed outright.
						UML_DLOAD(block, I0, fastbase, I0, SIZE_QWORD, SCALE_x1);   // dload   i0,fastbase,i0,qword_x1
						if (!native64)
							UML_DROR(block, I0, I0, 32 * (m_bigendian ? BYTE_XOR_BE(0) : BYTE_XOR_LE(0)));
																					// dror    i0,i0,32*bytexor
					}
					UML_RET(block);                                                 // ret
				}
				else
				{
					// Same width-conditional correction as the
					// read side's own comments above: choose the "...to a
					// 32-bit space" or "...to a 64-bit space" swizzle
					// variant per size based on `native64`, matching this
					// specific mips3_device's actual address space width
					// rather than assuming either one for every caller of
					// this shared function.
					if (size == 1)
					{
						UML_XOR(block, I0, I0, m_bigendian ? (native64 ? BYTE8_XOR_BE(0) : BYTE4_XOR_BE(0))
															: (native64 ? BYTE8_XOR_LE(0) : BYTE4_XOR_LE(0)));
																					// xor     i0,i0,bytexor
						UML_STORE(block, fastbase, I0, I1, SIZE_BYTE, SCALE_x1);    // store   fastbase,i0,i1,byte
					}
					else if (size == 2)
					{
						UML_XOR(block, I0, I0, m_bigendian ? (native64 ? WORD2_XOR_BE(0) : WORD_XOR_BE(0))
															: (native64 ? WORD2_XOR_LE(0) : WORD_XOR_LE(0)));
																					// xor     i0,i0,wordxor
						UML_STORE(block, fastbase, I0, I1, SIZE_WORD, SCALE_x1);    // store   fastbase,i0,i1,word_x1
					}
					else if (size == 4)
					{
						// See the read side's own size==4 comment: only a
						// 64-bit-native space needs this swizzle (a dword
						// exactly matches a 32-bit-native space's width).
						if (native64)
							UML_XOR(block, I0, I0, m_bigendian ? DWORD_XOR_BE(0) : DWORD_XOR_LE(0));
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
						// See the read side's own size==8 comment: a
						// 64-bit-native space needs no rotation at all
						// (confirmed via the live mismatch); a 32-bit-
						// native space still needs the original half-swap
						// DROR on I1 (the value) and, for a masked write,
						// I2 (the mask) as well, to combine two native
						// 32-bit lanes into one 64-bit value correctly.
						if (!native64)
						{
							UML_DROR(block, I1, I1, 32 * (m_bigendian ? BYTE_XOR_BE(0) : BYTE_XOR_LE(0)));
																						// dror    i1,i1,32*bytexor
							if (ismasked)
								UML_DROR(block, I2, I2, 32 * (m_bigendian ? BYTE_XOR_BE(0) : BYTE_XOR_LE(0)));
																						// dror    i2,i2,32*bytexor
						}
						if (ismasked)
						{
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

void mips3_device::static_generate_memory_rw(drcuml_block &block, int &label, int size, bool iswrite, bool ismasked)
{
	// a read handler may call bus_error(): the read then takes a data bus
	// error exception instead of returning
	if (!iswrite)
		UML_MOV(block, mem(&m_core->bus_error), 0);                             // mov     [bus_error],0

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
	if (!iswrite)
	{
		uml::code_label const ok = label++;
		UML_TEST(block, mem(&m_core->bus_error), 1);                            // test    [bus_error],1
		UML_JMPc(block, COND_Z, ok);                                            // jz      ok
		UML_MOV(block, mem(&m_core->bus_error), 0);                             // mov     [bus_error],0
		UML_EXH(block, *m_exception[EXCEPTION_BUSDATA], 0);                     // exh     busdata,0
		UML_LABEL(block, ok);                                                   // ok:
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


	/* general case: assume paging and perform a translation */
	UML_SHR(block, I3, I0, 12);                                                     // shr     i3,i0,12
	UML_LOAD(block, I3, (void *)vtlb_table(), I3, SIZE_DWORD, SCALE_x4);            // load    i3,[vtlb_table],i3,dword
	UML_TEST(block, I3, iswrite ? WRITE_ALLOWED : READ_ALLOWED);                    // test    i3,iswrite ? WRITE_ALLOWED : READ_ALLOWED
	UML_JMPc(block, COND_Z, tlbmiss = label++);                                     // jmp     tlbmiss,z
	UML_ROLINS(block, I0, I3, 0, 0xfffff000);                                       // rolins  i0,i3,0,0xfffff000

	static_generate_fastram_accessor(block, label, size, iswrite, ismasked);
	static_generate_memory_rw(block, label, size, iswrite, ismasked);


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
	static_generate_memory_rw(block, label, size, iswrite, ismasked);
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

	if (m_drc_asid_dispatch)
	{
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
	}
	else
	{
		// Fix disabled (see above) - dispatch_mode just mirrors mode, which
		// restores the exact pre-fix behavior everywhere dispatch_mode is
		// used (the 14 UML_HASH/UML_HASHJMP/hash_exists sites, and
		// execute_run()'s code_compile_block() call) without needing to
		// touch any of those sites individually.
		UML_MOV(block, mem(&m_core->dispatch_mode), I2);            // mov     [dispatch_mode],i2
	}
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
    regression (2026-08-06) before this
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
	// own right at the time, but a 2026-08-07 measurement
	// showed that FETCH_ALLOWED-awareness, not page count, is what
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

	/* at the idle PC, give up the timeslice unless an interrupt is pending */
	if (m_idle_pc != 0 && desc->pc == m_idle_pc && desc->opptr == m_idle_opcode)
	{
		uml::code_label skip;
		UML_AND(block, I0, CPR032(COP0_Cause), CPR032(COP0_Status));      // and     i0,[Cause],[Status]
		UML_AND(block, I0, I0, 0xff00);                                   // and     i0,i0,0xff00
		UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);             // jmp     skip,NZ
		UML_MOV(block, mem(&m_core->icount), 0);                          // mov     [icount],0
		UML_LABEL(block, skip);                                           // skip:
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
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
		UML_MOV(block, mem(&m_core->arg0), 1);
		UML_MOV(block, mem(&m_core->arg1), desc->pc);
		UML_MOV(block, mem(&m_core->diag_study_serial), m_recompile_study_cur_serial);
#endif
#ifdef MAME_SGI_STALE_STUB_FIX
		UML_MOV(block, mem(&m_core->stub_container_pc), m_compile_entry_pc);    // mov     [stub_container_pc],entry pc of this block
		UML_MOV(block, mem(&m_core->stub_container_mode), m_compile_entry_mode); // mov     [stub_container_mode],hash key of this block
		UML_MOV(block, mem(&m_core->stub_site_pc), desc->epc() & ~3);           // mov     [stub_site_pc],pc tlb_mismatch will recover
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
			// NOTE (2026-08-23): an earlier version of the refresh fix
			// stashed tlbtable[pc>>12] into m_core here so the shared
			// handler could compare it against the live entry. That
			// compare could never be true: the EXHc below fires only
			// when live != baked, and the handler re-reads that same
			// entry with nothing running in between, so the two can only
			// ever differ. It also cost a store on every execution of
			// this guard, not just on mismatches. Removed.
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
			if (recompile_study_traced(desc->pc, true))
			{
				UML_MOV(block, mem(&m_core->arg0), desc->pc);
				UML_MOV(block, mem(&m_core->arg1), tlbtable[desc->pc >> 12]);
				UML_MOV(block, mem(&m_core->diag_study_serial), m_recompile_study_serial);
				UML_CALLC(block, cfunc_recompile_study_validate_probe, this);
			}
#endif
			UML_LOAD(block, I0, &tlbtable[desc->pc >> 12], 0, SIZE_DWORD, SCALE_x4);        // load i0,tlbtable[desc->pc >> 12],0,dword
			UML_CMP(block, I0, tlbtable[desc->pc >> 12]);                   // cmp     i0,*tlbentry
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
			UML_MOVc(block, COND_NE, mem(&m_core->diag_study_baked), tlbtable[desc->pc >> 12]); // flags preserved
#endif
			UML_EXHc(block, COND_NE, *m_tlb_mismatch, 0);            // exh     tlb_mismatch,0,NE
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
#ifdef MAME_SGI_RECOMPILE_STUDY_DIAG
			UML_MOV(block, mem(&m_core->arg0), 3);
			UML_MOV(block, mem(&m_core->arg1), desc->pc);
#endif
			UML_EXH(block, *m_tlb_mismatch, 0);                  // exh     tlb_mismatch,0
			// Unconditional tlb exception, no point going further
			return;
		}
	}


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
		// MAME_SGI_COP1_CHECK_ONCE: a COP0 instruction (MTC0 to Status, ERET)
		// or one compiled as a C call can change Status CU1
		if ((desc->opptr >> 26) == 0x10 || !opcode_result)
			compiler.cop1_checked = false;
		if (!opcode_result)
		{
			UML_MOV(block, mem(&m_core->pc), desc->pc);                          // mov     [pc],desc->pc
			UML_MOV(block, mem(&m_core->arg0), desc->opptr);      // mov     [arg0],desc->opptr
			UML_CALLC(block, cfunc_unimplemented, this);                           // callc   cfunc_unimplemented
		}
	}
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


	}

	/* set the link if needed -- before the delay slot */
	if (linkreg != 0)
	{
		UML_DMOV(block, R64(linkreg), (int32_t)(desc->pc + 8));                   // dmov    <linkreg>,desc->pc + 8


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
			if (m_drc_asid_dispatch)
			{
				UML_HASHJMP(block, live_dispatch_key(desc->targetpc), desc->targetpc, *m_nocode);       // hashjmp <mode>,desc->targetpc,nocode
			}
			else
			{
				UML_HASHJMP(block, mem(&m_core->dispatch_mode), desc->targetpc, *m_nocode);       // hashjmp <mode>,desc->targetpc,nocode
			}
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
		if (m_drc_asid_dispatch)
		{
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
			UML_HASHJMP(block, I3, mem(&m_core->jmpdest), *m_nocode);   // hashjmp <mode>,<rsreg>,nocode
		}
		else
		{
			UML_HASHJMP(block, mem(&m_core->dispatch_mode), mem(&m_core->jmpdest), *m_nocode);   // hashjmp <mode>,<rsreg>,nocode
		}
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
    generate_likely_branch_skip_fixup - cc1 flake fix
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
    vtlb invalidation): drcfe.ipp's
    do_describe_code() raw walk BREAKS entirely on a page
    fault, so desc->pc+8 never gets its own descriptor in
    that compile at all, and whatever OTHER, unrelated
    address the frontend independently discovered next
    (observed live: this same branch's own TAKEN target,
    reached via a separate branch-target worklist entry)
    silently takes its place in the live sequence. This
    is the confirmed root cause of the cc1 <built-in>
    "predicate's answer is empty" flake (2026-09-10) -
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
    class branches system-wide, per an earlier
    measurement that compiler_page_fault() is a
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

		generate_update_cycles(block, compiler, desc->pc + 8, true);
		if (m_drc_asid_dispatch)
		{
			UML_HASHJMP(block, static_dispatch_key(desc->pc + 8), desc->pc + 8, *m_nocode);   // hashjmp <mode>,desc->pc+8,nocode
		}
		else
		{
			UML_HASHJMP(block, m_core->dispatch_mode, desc->pc + 8, *m_nocode);               // hashjmp <mode>,desc->pc+8,nocode
		}
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
			generate_delay_slot_and_branch(block, compiler, desc, 31);       // <next instruction + hashjmp>
			return true;

		case 0x04:  /* BEQ - MIPS I */
		case 0x14:  /* BEQL - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_JMPc(block, COND_NE, skip = compiler.labelnum++);                  // jmp     skip,NE
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			UML_LABEL(block, skip);                                             // skip:
			generate_likely_branch_skip_fixup(block, compiler, desc);          // likely branch: not-taken fixup
			return true;

		case 0x05:  /* BNE - MIPS I */
		case 0x15:  /* BNEL - MIPS II */
			UML_DCMP(block, R64(RSREG), R64(RTREG));                                // dcmp    <rsreg>,<rtreg>
			UML_JMPc(block, COND_E, skip = compiler.labelnum++);                       // jmp     skip,E
			generate_delay_slot_and_branch(block, compiler, desc, 0);        // <next instruction + hashjmp>
			UML_LABEL(block, skip);                                             // skip:
			generate_likely_branch_skip_fixup(block, compiler, desc);          // likely branch: not-taken fixup
			return true;

		case 0x06:  /* BLEZ - MIPS I */
		case 0x16:  /* BLEZL - MIPS II */
			if (RSREG != 0)
			{
				UML_DCMP(block, R64(RSREG), 0);                             // dcmp    <rsreg>,0
				UML_JMPc(block, COND_G, skip = compiler.labelnum++);                   // jmp     skip,G
				generate_delay_slot_and_branch(block, compiler, desc, 0);    // <next instruction + hashjmp>
				UML_LABEL(block, skip);                                         // skip:
				generate_likely_branch_skip_fixup(block, compiler, desc);      // likely branch: not-taken fixup
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
			generate_likely_branch_skip_fixup(block, compiler, desc);          // likely branch: not-taken fixup
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
			if (RTREG != 0)
			{
				UML_ADD(block, I0, R32(RSREG), SIMMVAL);                    // add     i0,<rsreg>,SIMMVAL,V
				UML_DSEXT(block, R64(RTREG), I0, SIZE_DWORD);                       // dsext   <rtreg>,i0,dword
			}
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
			return true;

		case 0x0d:  /* ORI - MIPS I */
			if (RTREG != 0)
				UML_DOR(block, R64(RTREG), R64(RSREG), UIMMVAL);                // dor     <rtreg>,<rsreg>,UIMMVAL
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
			check_cop1_access(block, compiler);
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);        // add     i0,<rsreg>,SIMMVAL
			UML_CALLH(block, *m_read32[m_core->mode >> 1]); // callh   read32
			UML_MOV(block, FPR32(RTREG), I0);           // mov     <cpr1_rt>,i0
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x35:  /* LDC1 - MIPS III */
			check_cop1_access(block, compiler);
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
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);                        // add     i0,<rsreg>,SIMMVAL
			UML_DMOV(block, I1, R64(RTREG));                                // dmov    i1,<rtreg>
			// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's
			// comment - this was the worst offender of the four, firing on
			// every 64-bit store anywhere in the kernel, unconditionally,
			// all night.
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
			check_cop1_access(block, compiler);
			UML_ADD(block, I0, R32(RSREG), SIMMVAL);            // add     i0,<rsreg>,SIMMVAL
			UML_MOV(block, I1, FPR32(RTREG));                   // mov     i1,<cpr1_rt>
			UML_CALLH(block, *m_write32[m_core->mode >> 1]);    // callh   write32
			if (!in_delay_slot)
				generate_update_cycles(block, compiler, desc->pc + 4, true);
			return true;

		case 0x3d:  /* SDC1 - MIPS III */
			check_cop1_access(block, compiler);
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
#ifdef MAME_DRC_BUGFIXES
			// The interrupt check for this instruction must happen before its
			// store takes effect (with EPC = this instruction, it re-executes
			// on return). generate_update_cycles uses I0/I1 as scratch, so
			// preserve the value being stored across it.
			UML_DMOV(block, mem(&m_core->cop0_store_value), I0);               // dmov    [cop0_store_value],i0
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
			UML_DMOV(block, I0, mem(&m_core->cop0_store_value));               // dmov    i0,[cop0_store_value]
			// (An earlier version of this fix moved the store ahead of the
			// interrupt check instead, to stop the check's I0/I1 scratch use
			// clobbering the value being stored. That let the check see the
			// NEW Status - e.g. EXL just cleared and IE set - and deliver an
			// interrupt with EPC pointing back at this mtc0, which then
			// re-executed on return with whatever the handler had left in its
			// source register; IRIX's vec_tlbmiss elocore_exl_3 sequence uses
			// $k0, which exception_exit does not preserve, storing an address
			// into Status. Preserving I0 in memory avoids both problems.)
			// TEMPORARY probe (2026-07-27): stash I0 (the value about to be
			// stored, unmodified since this instruction's own RTREG read)
			// and desc->pc into memory now (a plain MOV, doesn't touch I0) -
			// but the actual CALLC to inspect it is deferred to *after* I0's
			// last real use below (the diff computation), since UML_CALLC
			// can clobber scratch registers and I0 must survive intact
			// through the real store and generate_update_mode - the exact
			// clobber-ordering bug class this whole investigation started
			// from. See func_status_store_probe's comment for what this is
			// checking. Only needed by that probe family.
			UML_MOV(block, I1, CPR032(COP0_Status));                            // mov     i1,[Status]
			UML_MOV(block, CPR032(COP0_Status), I0);                            // mov     [Status],i0
			generate_update_mode(block);                                        // <update mode>
			UML_XOR(block, I0, I0, I1);                                         // xor     i0,i0,i1
			UML_TEST(block, I0, 0x8000);                                        // test    i0,0x8000
			UML_CALLCc(block, COND_NZ, cfunc_mips3com_update_cycle_counting, this);      // callc   mips3com_update_cycle_counting,mips.core,NZ
			// DISABLED (2026-07-28): see cfunc_interrupt_delivery_probe's
			// comment - hooked on every COP0_Status store anywhere in the
			// kernel, unconditionally, all night.
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
			// The interrupt check for this instruction must happen before its
			// store takes effect (with EPC = this instruction, it re-executes
			// on return). generate_update_cycles uses I0/I1 as scratch, so
			// preserve the value being stored across it.
			UML_DMOV(block, mem(&m_core->cop0_store_value), I0);               // dmov    [cop0_store_value],i0
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
			UML_DMOV(block, I0, mem(&m_core->cop0_store_value));               // dmov    i0,[cop0_store_value]
			UML_MOV(block, CPR032(COP0_Count), I0);                             // mov     [Count],i0
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
			// The interrupt check for this instruction must happen before its
			// store takes effect (with EPC = this instruction, it re-executes
			// on return). generate_update_cycles uses I0/I1 as scratch, so
			// preserve the value being stored across it.
			UML_DMOV(block, mem(&m_core->cop0_store_value), I0);               // dmov    [cop0_store_value],i0
			generate_update_cycles(block, compiler, desc->pc, !in_delay_slot, true, desc->pc);  // <subtract cycles> (TEMPORARY diag_tag, 2026-07-27)
			UML_DMOV(block, I0, mem(&m_core->cop0_store_value));               // dmov    i0,[cop0_store_value]
			UML_MOV(block, CPR032(COP0_Compare), I0);                           // mov     [Compare],i0
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
			// (real mfc0 Status / mask / mtc0, per an earlier
			// disassembly), reached from every interrupt-priority-level
			// change throughout the kernel. If this ever legitimately
			// traps here (live mode non-kernel, SR_COP0 clear) it means
			// Status/mode are genuinely wrong at this instant, not just a
			// stale-compiled-mode caching artifact - the gate above already
			// reads mode live via mem(&m_core->mode), so any trap past this
			// point can no longer be explained by the compile-time-baking
			// bug already fixed elsewhere in this file.
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
					UML_MOV(block, mem(&m_core->llbit), 0);                     // mov     [llbit],0
					UML_MOV(block, I0, CPR032(COP0_Status));                    // mov     i0,[Status]
					UML_TEST(block, I0, SR_ERL);                                // test    i0,SR_ERL
					UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);       // jmp     skip,nz
					UML_AND(block, I0, I0, ~SR_EXL);                            // and     i0,i0,~SR_EXL
					UML_MOV(block, CPR032(COP0_Status), I0);                    // mov     [Status],i0
					generate_update_mode(block);
					compiler.checkints = true;
					generate_update_cycles(block, compiler, CPR032(COP0_EPC), true);// <subtract cycles>
					if (m_drc_asid_dispatch)
					{
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
					}
					else
					{
						UML_HASHJMP(block, mem(&m_core->dispatch_mode), CPR032(COP0_EPC), *m_nocode);
																					// hashjmp <mode>,[EPC],nocode
					}
					UML_LABEL(block, skip);                                     // skip:
					UML_AND(block, I0, I0, ~SR_ERL);                            // and     i0,i0,~SR_ERL
					UML_MOV(block, CPR032(COP0_Status), I0);                    // mov     [Status],i0
					generate_update_mode(block);
					compiler.checkints = true;
					generate_update_cycles(block, compiler, CPR032(COP0_ErrorPC), true); // <subtract cycles>
					if (m_drc_asid_dispatch)
					{
						{
							// bridge: see the matching comment at the jmpdest
							// call site (generate_delay_slot_and_branch) - same
							// code_label-vs-int mismatch, same reason.
							int labelnum_tmp = compiler.labelnum;
							generate_select_dispatch_key(block, CPR032(COP0_ErrorPC), uml::I3, labelnum_tmp);
							compiler.labelnum = labelnum_tmp;
						}
						UML_HASHJMP(block, I3, CPR032(COP0_ErrorPC), *m_nocode); // hashjmp <mode>,[EPC],nocode
					}
					else
					{
						UML_HASHJMP(block, mem(&m_core->dispatch_mode), CPR032(COP0_ErrorPC), *m_nocode); // hashjmp <mode>,[EPC],nocode
					}
					return true;

				case 0x20:  /* WAIT */
#ifdef MAME_SGI_WAIT_IDLE
				{
					// The guest re-enables interrupts with an MTC0 Status just
					// before WAIT, and that MTC0's interrupt check is only
					// emitted at the next cycle update, so test for an
					// already-pending interrupt here rather than idling past it.
					// The next cycle update then exits through out_of_cycles.
					uml::code_label skip;
					UML_AND(block, I0, CPR032(COP0_Cause), CPR032(COP0_Status)); // and     i0,[Cause],[Status]
					UML_AND(block, I0, I0, 0xff00);                             // and     i0,i0,0xff00
					UML_JMPc(block, COND_NZ, skip = compiler.labelnum++);       // jmp     skip,NZ
					UML_MOV(block, mem(&m_core->icount), 0);                    // mov     [icount],0
					UML_LABEL(block, skip);                                     // skip:
				}
#endif
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

// MAME_SGI_COP1_CHECK_ONCE: within a sequence the test need only precede
// the first FP instruction. Code is entered only at a sequence's head (its
// hash entry or label); every exit mid-sequence (an exception, an
// interrupt or running out of cycles) resumes through the dispatcher,
// which compiles a new sequence at the resume PC; and CU1 changes only by
// an exception or a COP0 instruction, after which the test is emitted
// again. IRIX switches the FPU lazily, so this test is what hands it to the
// current process.
void mips3_device::check_cop1_access(drcuml_block &block, compiler_state &compiler)
{
	if (m_drcoptions & MIPS3DRC_STRICT_COP1)
	{
#if MAME_SGI_COP1_CHECK_ONCE == 1
		if (compiler.cop1_checked)
			return;
		compiler.cop1_checked = true;
#endif
		generate_badcop(block, 1);
	}
}

/*-------------------------------------------------------
    generate_cop1 - compile COP1 opcodes
---------------------------------------------------------*/

/*-------------------------------------------------
    fp_convert_fixup - redo a floating point to
    integer conversion (opcode in arg0) whose host
    result was the x86 "integer indefinite" value
-------------------------------------------------*/

// MAME_SGI_FP_CONVERT_FIX: the host conversion returns 0x80000000 (or
// 0x8000000000000000) for a NaN or an out of range value, where MIPS gives
// 0x7fffffff (0x7fffffffffffffff) and sets the V cause and flag bits. That
// value is also the correct result for -2^31 (-2^63), so the generated code
// calls this only when the host returned it, and fp_convert() decides.
void mips3_device::fp_convert_fixup()
{
	const uint32_t op = m_core->arg0;
	const uint32_t funct = op & 0x3f;
	const int fd = (op >> 6) & 31;
	const bool fr0 = (m_core->mode & 1) == 0;
	const bool single = ((op >> 21) & 31) == 16;
	// the source as saved before the conversion, which may have overwritten it
	// (fd == fs)
	double src;
	if (single)
	{
		const uint32_t bits = (uint32_t)m_core->fp_convert_src;
		float f;
		memcpy(&f, &bits, sizeof(f));
		src = f;
	}
	else
	{
		const uint64_t bits = m_core->fp_convert_src;
		memcpy(&src, &bits, sizeof(src));
	}
	const int mode = funct >= 0x24 ? (m_core->ccr[1][31] & 3) : (funct & 3);
	const bool wide = funct == 0x25 || (funct >= 0x08 && funct <= 0x0b);
	const int64_t r = fp_convert(src, mode, wide);
	if (wide)
		*(uint64_t *)&m_core->cpr[1][fr0 ? (fd & 0x1e) : fd] = r;
	else if (fr0)
		((uint32_t *)&m_core->cpr[1][fd & 0x1e])[BYTE_XOR_LE(fd & 1)] = (uint32_t)r;
	else
		((uint32_t *)&m_core->cpr[1][fd])[BYTE_XOR_LE(0)] = (uint32_t)r;
}

void mips3_device::generate_fp_convert_save(drcuml_block &block, uint32_t op)
{
	if (IS_SINGLE(op))
		UML_MOV(block, mem(&m_core->fp_convert_src), FPR32(FSREG));            // mov     [fp_convert_src],<fsreg>
	else
		UML_DMOV(block, mem(&m_core->fp_convert_src), FPR64(FSREG));           // dmov    [fp_convert_src],<fsreg>
}

void mips3_device::generate_fp_convert_fixup(drcuml_block &block, compiler_state &compiler, uint32_t op, bool wide)
{
	const int skip = compiler.labelnum++;
	if (wide)
		UML_DCMP(block, FPR64(FDREG), 0x8000000000000000ULL);                 // dcmp    <fdreg>,0x8000000000000000
	else
		UML_CMP(block, FPR32(FDREG), 0x80000000);                             // cmp     <fdreg>,0x80000000
	UML_JMPc(block, COND_NE, skip);                                           // jmp     skip,ne
	UML_MOV(block, mem(&m_core->arg0), op);                                   // mov     [arg0],op
	UML_CALLC(block, cfunc_fp_convert_fixup, this);                           // callc   fp_convert_fixup
	UML_LABEL(block, skip);                                                   // skip:
}

bool mips3_device::generate_cop1(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc)
{
	const uint32_t op = desc->opptr;
	uml::code_label skip;
	uml::condition_t condition;

	check_cop1_access(block, compiler);

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
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* ROUND.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_ROUND);// fstoint <fdreg>,<fsreg>,qword,round
					}
					else                /* ROUND.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_ROUND);// fdtoint <fdreg>,<fsreg>,qword,round
					}
					generate_fp_convert_fixup(block, compiler, op, true);
					return true;

				case 0x09:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* TRUNC.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_TRUNC);// fstoint <fdreg>,<fsreg>,qword,trunc
					}
					else                /* TRUNC.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_TRUNC);// fdtoint <fdreg>,<fsreg>,qword,trunc
					}
					generate_fp_convert_fixup(block, compiler, op, true);
					return true;

				case 0x0a:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* CEIL.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_CEIL);// fstoint <fdreg>,<fsreg>,qword,ceil
					}
					else                /* CEIL.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_CEIL);// fdtoint <fdreg>,<fsreg>,qword,ceil
					}
					generate_fp_convert_fixup(block, compiler, op, true);
					return true;

				case 0x0b:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* FLOOR.L.S - MIPS III */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_FLOOR);// fstoint <fdreg>,<fsreg>,qword,floor
					}
					else                /* FLOOR.L.D - MIPS III */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_FLOOR);// fdtoint <fdreg>,<fsreg>,qword,floor
					}
					generate_fp_convert_fixup(block, compiler, op, true);
					return true;

				case 0x0c:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* ROUND.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_ROUND);// fstoint <fdreg>,<fsreg>,dword,round
					}
					else                /* ROUND.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_ROUND);// fdtoint <fdreg>,<fsreg>,dword,round
					}
					generate_fp_convert_fixup(block, compiler, op, false);
					return true;

				case 0x0d:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* TRUNC.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_TRUNC);// fstoint <fdreg>,<fsreg>,dword,trunc
					}
					else                /* TRUNC.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_TRUNC);// fdtoint <fdreg>,<fsreg>,dword,trunc
					}
					generate_fp_convert_fixup(block, compiler, op, false);
					return true;

				case 0x0e:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* CEIL.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_CEIL);// fstoint <fdreg>,<fsreg>,dword,ceil
					}
					else                /* CEIL.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_CEIL);// fdtoint <fdreg>,<fsreg>,dword,ceil
					}
					generate_fp_convert_fixup(block, compiler, op, false);
					return true;

				case 0x0f:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* FLOOR.W.S - MIPS II */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_FLOOR);// fstoint <fdreg>,<fsreg>,dword,floor
					}
					else                /* FLOOR.W.D - MIPS II */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_FLOOR);// fdtoint <fdreg>,<fsreg>,dword,floor
					}
					generate_fp_convert_fixup(block, compiler, op, false);
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
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* CVT.W.S - MIPS I */
					{
						UML_FSTOINT(block, FPR32(FDREG), FPR32(FSREG), SIZE_DWORD, ROUND_DEFAULT);// fstoint <fdreg>,<fsreg>,dword,default
					}
					else                /* CVT.W.D - MIPS I */
					{
						UML_FDTOINT(block, FPR32(FDREG), FPR64(FSREG), SIZE_DWORD, ROUND_DEFAULT);// fdtoint <fdreg>,<fsreg>,dword,default
					}
					generate_fp_convert_fixup(block, compiler, op, false);
					return true;

				case 0x25:
					generate_fp_convert_save(block, op);
					if (IS_SINGLE(op))  /* CVT.L.S - MIPS I */
					{
						UML_FSTOINT(block, FPR64(FDREG), FPR32(FSREG), SIZE_QWORD, ROUND_DEFAULT);// fstoint <fdreg>,<fsreg>,qword,default
					}
					else                /* CVT.L.D - MIPS I */
					{
						UML_FDTOINT(block, FPR64(FDREG), FPR64(FSREG), SIZE_QWORD, ROUND_DEFAULT);// fdtoint <fdreg>,<fsreg>,qword,default
					}
					generate_fp_convert_fixup(block, compiler, op, true);
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

	check_cop1_access(block, compiler);

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
