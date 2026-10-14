// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

mips3.h

Interface file for the universal machine language-based
MIPS III/IV emulator.

***************************************************************************/

#ifndef MAME_CPU_MIPS_MIPS3_H
#define MAME_CPU_MIPS_MIPS3_H

#pragma once

// TEMPORARY diagnostic (2026-09-10) - per user direction: stop
// chasing narrow, targeted hypotheses one at a time and instead build a
// complete, end-to-end trace of DRC-level events (fresh compiles AND
// re-entries into already-compiled code) across a whole cc1 invocation,
// for a matched OK/PREDICATE pair - looking for where a genuinely
// comparable pass diverges into a fail, without assuming in advance
// which mechanism it'll be. Deliberately does NOT add any UML
// instructions to compiled guest code (the exact risk the user flagged
// - this project's own prior instrumentation, including this session's
// own $s3 probe, can shift COMPILE_MAX_SEQUENCE boundaries and change
// behavior). Both event sources logged here are pure host-side C++,
// called BETWEEN JIT block executions (execute_run()'s own dispatch
// loop in mips3.cpp) or entirely outside the compiled-code path
// (code_compile_block() itself, mips3drc.cpp) - zero footprint on the
// actual generated code or its timing. Shared here (not file-local like
// most other task84 macros) because it gates code in both mips3.cpp
// (execute_run) and mips3drc.cpp (code_compile_block, log_drc_event).
// #define MAME_SGI_TASK84_DRC_EVENT_LOG

#include "ps2vu.h"

#include "cpu/drcuml.h"

#include <unordered_map>
#include <unordered_set>
#include <map>
#include <cstring>

#include "divtlb.h"

#include <bitset>

// TEMPORARY diagnostic (2026-08-17) - see debug_get_codeptr/
// debug_scan_for_immediate's own comments below. Self-defined here
// (not via sgi.lua's defines{} block) because this header compiles as
// part of the shared "optional" project (used by every MAME subtarget,
// not just sgi), which sgi.lua's defines{} block never reaches - see
// tools/build_profiled.sh-adjacent build notes / the project's own
// build-verify convention for this exact gotcha.
#define MIPS3_DEBUG_HELPERS

// PROTOTYPE (2026-08-08): fixed slot count for the per-call-site guard-
// based inline cache (see the class-scope comment, below, near
// m_diag_callsite_ic_state). Must be visible to internal_mips3_state,
// which embeds the near-cache-resident slot arrays - a plain #define,
// not a class member, since internal_mips3_state is declared before any
// class-scope constant would be.
#define MAME_SGI_IC_SITE_COUNT 256

// WAIT (R4600/R4700/R5000) idles until the next scheduled event instead of
// executing as a no-op: when no enabled-and-pending interrupt is visible in
// Cause & Status, the CPU gives up the rest of its timeslice. Emulated time
// (and so Count/Compare) advances exactly as if the CPU had spun, and the
// timeslice already ends at the next scheduled timer or input change, so
// interrupt latency is unchanged; only host time spent in the guest's idle
// loop is saved. Shared by the interpreter (mips3.cpp) and the recompiler
// (mips3drc.cpp).
#define MAME_SGI_WAIT_IDLE

DECLARE_DEVICE_TYPE(R4000BE, r4000be_device)
DECLARE_DEVICE_TYPE(R4000LE, r4000le_device)
DECLARE_DEVICE_TYPE(R4400BE, r4400be_device)
DECLARE_DEVICE_TYPE(R4400LE, r4400le_device)
// NEC VR4300 series is MIPS III with 32-bit address bus and slightly custom COP0/TLB
DECLARE_DEVICE_TYPE(VR4300BE, vr4300be_device)
DECLARE_DEVICE_TYPE(VR4300LE, vr4300le_device)
// VR4310 = VR4300 with different speed bin
DECLARE_DEVICE_TYPE(VR4310BE, vr4310be_device)
DECLARE_DEVICE_TYPE(VR4310LE, vr4310le_device)
DECLARE_DEVICE_TYPE(R4600BE, r4600be_device)
DECLARE_DEVICE_TYPE(R4600LE, r4600le_device)
DECLARE_DEVICE_TYPE(R4650BE, r4650be_device)
DECLARE_DEVICE_TYPE(R4650LE, r4650le_device)
DECLARE_DEVICE_TYPE(R4700BE, r4700be_device)
DECLARE_DEVICE_TYPE(R4700LE, r4700le_device)
DECLARE_DEVICE_TYPE(TX4925BE, tx4925be_device)
DECLARE_DEVICE_TYPE(TX4925LE, tx4925le_device)
DECLARE_DEVICE_TYPE(R5000BE, r5000be_device)
DECLARE_DEVICE_TYPE(R5000LE, r5000le_device)
DECLARE_DEVICE_TYPE(VR5500BE, vr5500be_device)
DECLARE_DEVICE_TYPE(VR5500LE, vr5500le_device)
DECLARE_DEVICE_TYPE(QED5271BE, qed5271be_device)
DECLARE_DEVICE_TYPE(QED5271LE, qed5271le_device)
DECLARE_DEVICE_TYPE(RM7000BE, rm7000be_device)
DECLARE_DEVICE_TYPE(RM7000LE, rm7000le_device)
DECLARE_DEVICE_TYPE(R5900LE, r5900le_device)
DECLARE_DEVICE_TYPE(R5900BE, r5900be_device)


/***************************************************************************
REGISTER ENUMERATION
***************************************************************************/

enum {
	MIPS3_PC = STATE_GENPC,
	MIPS3_R0 = 1,
	MIPS3_R1,
	MIPS3_R2,
	MIPS3_R3,
	MIPS3_R4,
	MIPS3_R5,
	MIPS3_R6,
	MIPS3_R7,
	MIPS3_R8,
	MIPS3_R9,
	MIPS3_R10,
	MIPS3_R11,
	MIPS3_R12,
	MIPS3_R13,
	MIPS3_R14,
	MIPS3_R15,
	MIPS3_R16,
	MIPS3_R17,
	MIPS3_R18,
	MIPS3_R19,
	MIPS3_R20,
	MIPS3_R21,
	MIPS3_R22,
	MIPS3_R23,
	MIPS3_R24,
	MIPS3_R25,
	MIPS3_R26,
	MIPS3_R27,
	MIPS3_R28,
	MIPS3_R29,
	MIPS3_R30,
	MIPS3_R31,
	MIPS3_HI,
	MIPS3_LO,
	MIPS3_FPR0,
	MIPS3_FPS0,
	MIPS3_FPD0,
	MIPS3_FPR1,
	MIPS3_FPS1,
	MIPS3_FPD1,
	MIPS3_FPR2,
	MIPS3_FPS2,
	MIPS3_FPD2,
	MIPS3_FPR3,
	MIPS3_FPS3,
	MIPS3_FPD3,
	MIPS3_FPR4,
	MIPS3_FPS4,
	MIPS3_FPD4,
	MIPS3_FPR5,
	MIPS3_FPS5,
	MIPS3_FPD5,
	MIPS3_FPR6,
	MIPS3_FPS6,
	MIPS3_FPD6,
	MIPS3_FPR7,
	MIPS3_FPS7,
	MIPS3_FPD7,
	MIPS3_FPR8,
	MIPS3_FPS8,
	MIPS3_FPD8,
	MIPS3_FPR9,
	MIPS3_FPS9,
	MIPS3_FPD9,
	MIPS3_FPR10,
	MIPS3_FPS10,
	MIPS3_FPD10,
	MIPS3_FPR11,
	MIPS3_FPS11,
	MIPS3_FPD11,
	MIPS3_FPR12,
	MIPS3_FPS12,
	MIPS3_FPD12,
	MIPS3_FPR13,
	MIPS3_FPS13,
	MIPS3_FPD13,
	MIPS3_FPR14,
	MIPS3_FPS14,
	MIPS3_FPD14,
	MIPS3_FPR15,
	MIPS3_FPS15,
	MIPS3_FPD15,
	MIPS3_FPR16,
	MIPS3_FPS16,
	MIPS3_FPD16,
	MIPS3_FPR17,
	MIPS3_FPS17,
	MIPS3_FPD17,
	MIPS3_FPR18,
	MIPS3_FPS18,
	MIPS3_FPD18,
	MIPS3_FPR19,
	MIPS3_FPS19,
	MIPS3_FPD19,
	MIPS3_FPR20,
	MIPS3_FPS20,
	MIPS3_FPD20,
	MIPS3_FPR21,
	MIPS3_FPS21,
	MIPS3_FPD21,
	MIPS3_FPR22,
	MIPS3_FPS22,
	MIPS3_FPD22,
	MIPS3_FPR23,
	MIPS3_FPS23,
	MIPS3_FPD23,
	MIPS3_FPR24,
	MIPS3_FPS24,
	MIPS3_FPD24,
	MIPS3_FPR25,
	MIPS3_FPS25,
	MIPS3_FPD25,
	MIPS3_FPR26,
	MIPS3_FPS26,
	MIPS3_FPD26,
	MIPS3_FPR27,
	MIPS3_FPS27,
	MIPS3_FPD27,
	MIPS3_FPR28,
	MIPS3_FPS28,
	MIPS3_FPD28,
	MIPS3_FPR29,
	MIPS3_FPS29,
	MIPS3_FPD29,
	MIPS3_FPR30,
	MIPS3_FPS30,
	MIPS3_FPD30,
	MIPS3_FPR31,
	MIPS3_FPS31,
	MIPS3_FPD31,
	MIPS3_CCR1_31,
	MIPS3_SR,
	MIPS3_EPC,
	MIPS3_CAUSE,
	MIPS3_COUNT,
	MIPS3_COMPARE,
	MIPS3_INDEX,
	MIPS3_RANDOM,
	MIPS3_ENTRYHI,
	MIPS3_ENTRYLO0,
	MIPS3_ENTRYLO1,
	MIPS3_PAGEMASK,
	MIPS3_WIRED,
	MIPS3_BADVADDR,
	MIPS3_LLADDR,
	MIPS3_R0H,
	MIPS3_R1H,
	MIPS3_R2H,
	MIPS3_R3H,
	MIPS3_R4H,
	MIPS3_R5H,
	MIPS3_R6H,
	MIPS3_R7H,
	MIPS3_R8H,
	MIPS3_R9H,
	MIPS3_R10H,
	MIPS3_R11H,
	MIPS3_R12H,
	MIPS3_R13H,
	MIPS3_R14H,
	MIPS3_R15H,
	MIPS3_R16H,
	MIPS3_R17H,
	MIPS3_R18H,
	MIPS3_R19H,
	MIPS3_R20H,
	MIPS3_R21H,
	MIPS3_R22H,
	MIPS3_R23H,
	MIPS3_R24H,
	MIPS3_R25H,
	MIPS3_R26H,
	MIPS3_R27H,
	MIPS3_R28H,
	MIPS3_R29H,
	MIPS3_R30H,
	MIPS3_R31H,
};

/* COP1 CCR register */
#define COP1_FCR31              (m_core->ccr[1][31])

/***************************************************************************
INTERRUPT CONSTANTS
***************************************************************************/

#define MIPS3_IRQ0      0       /* IRQ0 */
#define MIPS3_IRQ1      1       /* IRQ1 */
#define MIPS3_IRQ2      2       /* IRQ2 */
#define MIPS3_IRQ3      3       /* IRQ3 */
#define MIPS3_IRQ4      4       /* IRQ4 */
#define MIPS3_IRQ5      5       /* IRQ5 */



/***************************************************************************
STRUCTURES
***************************************************************************/

class mips3_device : public cpu_device, public device_vtlb_interface {
protected:
	class frontend;
	class opcode_desc;

	/* MIPS flavors */
	enum mips3_flavor {
		/* MIPS III variants */
		MIPS3_TYPE_MIPS_III,
		MIPS3_TYPE_R4000,
		MIPS3_TYPE_R4400,
		MIPS3_TYPE_VR4300,
		MIPS3_TYPE_R4600,
		MIPS3_TYPE_R4650,
		MIPS3_TYPE_R4700,
		MIPS3_TYPE_TX4925,

		/* MIPS IV variants */
		MIPS3_TYPE_MIPS_IV,
		MIPS3_TYPE_R5000,
		MIPS3_TYPE_VR5500,
		MIPS3_TYPE_QED5271,
		MIPS3_TYPE_R5900,
		MIPS3_TYPE_RM7000
	};

	static inline constexpr unsigned MIPS3_MAX_FASTRAM      = 3;
	static inline constexpr unsigned MIPS3_MAX_HOTSPOTS     = 16;

	static inline constexpr unsigned MIPS3_MAX_TLB_ENTRIES  = 48;

	// MIPS3 TLB entry
	struct mips3_tlb_entry {
		bool matches_asid(uint8_t asid) const noexcept;
		bool is_global() const noexcept;
		void log_half(int tlbindex, int which) const;

		uint64_t    page_mask;
		uint64_t    entry_hi;
		uint64_t    entry_lo[2];
	};

public:
	// construction/destruction
	mips3_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, mips3_flavor flavor, endianness_t endiannes, uint32_t data_bits);
	virtual ~mips3_device();

	void set_drc_cache_size(size_t bytes) { m_drc_cache.set_size(bytes); }
	void set_icache_size(size_t icache_size) { c_icache_size = icache_size; }
	// MAME_SGI_HASH32: use the DRC hash table with 32-bit entries (half the
	// size; see drcbex64.cpp). Off by default, so other drivers keep stock.
	void set_drc_hash32(bool enable) { m_drc_hash32 = enable; }
	void set_dcache_size(size_t dcache_size) { c_dcache_size = dcache_size; }
	void set_secondary_cache_line_size(uint8_t secondary_cache_line_size) { c_secondary_cache_line_size = secondary_cache_line_size; }
	void set_system_clock(uint32_t system_clock) { c_system_clock = system_clock; }

	TIMER_CALLBACK_MEMBER(compare_int_callback);

	void add_fastram(offs_t start, offs_t end, uint8_t readonly, void *base);
	void clear_fastram(uint32_t select_start);
	// Public wrapper for the otherwise-private code_flush_cache(), added
	// for sgi/mc.cpp's use. Only ever called from a genuine
	// scheduler-driven timer callback there (never synchronously from a
	// device register write reached by the guest's own currently-
	// executing JIT-compiled code), so this is safe by construction, not
	// by timing luck - see mc.cpp's check_fastram_settle() for why that
	// distinction mattered here.
	void invalidate_fastram_cache() { code_flush_cache(); }
	// Temporary: dumps the raw m_fastram[] table (every slot
	// up to m_fastram_select) to stderr - start/end/base exactly as
	// stored, not as re-derived from the caller's own registration
	// arithmetic, to rule out "the table holds something other than
	// what was asked for" as a hypothesis in its own right.
	void dump_fastram_table();
	void mips3drc_set_options(uint32_t options);
	uint32_t mips3drc_get_options();
	void mips3drc_add_hotspot(offs_t pc, uint32_t opcode, uint32_t cycles);

	// TEMPORARY diagnostic (2026-08-17) - host-gdb-callable helpers for
	// live investigation of a specific compiled DRC block, without
	// needing manual PIE/ASLR address math (drcuml_state::get_codeptr's
	// own wrapper gets inlined away in an optimized build, and computing
	// the real runtime address by hand from `nm`'s static address once
	// already caused a real SIGSEGV in this exact investigation,
	// 2026-08-16). noinline guarantees a real,
	// callable symbol regardless of optimization level. Not compiled in
	// by default (see MIPS3_DEBUG_HELPERS below) since this is purely a
	// debugging aid with no purpose in a normal build.
#ifdef MIPS3_DEBUG_HELPERS
	// `used` is required in addition to `noinline` - noinline only
	// blocks inlining at call sites, it does not stop the compiler from
	// dead-code-eliminating the entire function when nothing in the
	// codebase ever calls it (confirmed live: without `used`, these two
	// functions simply did not exist as symbols at all after a build,
	// `nm` found nothing).
	__attribute__((used, noinline)) void *debug_get_codeptr(uint32_t mode, offs_t pc)
	{
		return m_drcuml ? (void *)m_drcuml->get_codeptr(mode, pc) : nullptr;
	}

	// Bounded, read-only scan of native code starting at `start` for
	// any 4-byte little-endian occurrence of `needle` (e.g. a target
	// host address's low 32 bits, or a struct-relative displacement) -
	// answers "does this compiled block reference this location at
	// all" directly and reliably, instead of inferring it from grepping
	// manually-disassembled text (which this investigation already
	// tried and found inconclusive - not exhaustive enough to rule
	// anything in or out with confidence) or from -drc_log_uml/
	// -drc_log_native (which, even filtered, produced a genuine
	// disk-safety near-miss). Caller is
	// responsible for `len` being a sane bound - this does no
	// validation of the memory range itself, matching the trust level
	// already implied by using host gdb to call this in the first
	// place. Returns the byte offset of the first match, or -1.
	__attribute__((used, noinline)) int32_t debug_scan_for_immediate(const uint8_t *start, uint32_t len, uint32_t needle)
	{
		if (len < 4)
			return -1;
		for (uint32_t i = 0; i <= len - 4; i++)
		{
			uint32_t v;
			memcpy(&v, start + i, 4);
			if (v == needle)
				return (int32_t)i;
		}
		return -1;
	}
#endif
	void mame_sgi_postload_diag();
	void mame_sgi_presave_diag();

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_stop() override ATTR_COLD;

	// device_execute_interface overrides
	virtual uint32_t execute_min_cycles() const noexcept override { return 1; }
	virtual uint32_t execute_max_cycles() const noexcept override { return 40; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;
	virtual bool memory_translate(int spacenum, int intention, offs_t &address, address_space *&target_space) override;

	// device_state_interface overrides
	virtual void state_export(const device_state_entry &entry) override;
	virtual void state_import(const device_state_entry &entry) override;
	virtual void state_string_export(const device_state_entry &entry, std::string &str) const override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	void invalid_instruction(uint32_t op);

	virtual bool RBYTE(offs_t address, uint32_t *result);
	virtual bool RHALF(offs_t address, uint32_t *result);
	virtual bool RWORD(offs_t address, uint32_t *result, bool insn = false);
	virtual bool RWORD_MASKED(offs_t address, uint32_t *result, uint32_t mem_mask);
	virtual bool RDOUBLE(offs_t address, uint64_t *result);
	virtual bool RDOUBLE_MASKED(offs_t address, uint64_t *result, uint64_t mem_mask);
	virtual void WBYTE(offs_t address, uint8_t data);
	virtual void WHALF(offs_t address, uint16_t data);
	virtual void WWORD(offs_t address, uint32_t data);
	virtual void WWORD_MASKED(offs_t address, uint32_t data, uint32_t mem_mask);
	virtual void WDOUBLE(offs_t address, uint64_t data);
	virtual void WDOUBLE_MASKED(offs_t address, uint64_t data, uint64_t mem_mask);

	virtual void set_cop0_reg(int idx, uint64_t val);

	struct internal_mips3_state {
		/* core registers */
		uint32_t        pc;
		int             icount;
		uint64_t        r[35];
		uint32_t        sa;

		/* upper 64 bits of 128-bit GPRs (R5900 only) */
		uint64_t        rh[35];

		/* COP registers */
		uint64_t        cpr[3][32];
		uint64_t        ccr[3][32];
		uint32_t        llbit;
		float           acc;

		/* VU0 registers (R5900 only) */
		float           vfr[32][4]; // 0..3 = x..w
		uint32_t        vcr[32];
		float           vumem[0x1000];
		float*          vfmem;
		uint32_t*       vimem;
		float           vacc[4];
		float           p;

		uint32_t*       vr;
		float*          i;
		float*          q;

		uint32_t        mode;                       /* current global mode */

		// MAME_SGI_DISPATCH_MODE_ASID_FIX (2026-08-01): separate DRC
		// hash-dispatch key, distinct from `mode` above. `mode` alone
		// (privilege+FR, no ASID) is what the DRC hash table has always
		// keyed compiled blocks on, which works fine for KERNEL-mode code
		// (kseg0/1 is direct-mapped, genuinely ASID-independent) but
		// causes real cross-process thrashing for USER/SUPER code, which
		// is TLB/ASID-mapped: confirmed live, one hot USER-mode PC
		// recompiled 129,873 times in a single boot as competing
		// short-lived rc-script processes evicted each other's cached
		// code at shared conventional virtual addresses (74% of all
		// USER-mode compiles in that run were pure redundant work).
		// Encoding: kernel
		// gets a fixed sentinel (just the FR bit, asid-component=0) that
		// never varies with the live ASID - that constancy is what keeps
		// kernel code's immunity to this whole problem intact; SUPER/USER
		// get `((real_asid+1) << 2) | (is_user ? 2 : 0) | FR_bit`.
		// Computed in exactly one place, generate_update_mode()
		// (mips3drc.cpp), the same central function `mode` above is
		// itself recomputed from on every privilege transition - not
		// scattered across multiple update sites. Used only by the DRC
		// hash-table dispatch call sites (UML_HASH/UML_HASHJMP/
		// hash_exists) - every other consumer of `mode` (the 37 sites
		// using `mode >> 1` as a direct accessor-table index, IS_FR0/
		// IS_FR1, static_generate_memory_mode_checks, etc.) is completely
		// unaffected, since `mode` itself keeps its exact original
		// encoding and meaning.
		uint32_t        dispatch_mode;

		// TEMPORARY diagnostic (2026-08-06) - jmpdest (JR/JALR) dispatch-key
		// same-page hit rate for generate_select_dispatch_key()'s short-
		// circuit. IRIX's PIC ABI resolves essentially every cross-DSO call
		// through this exact indirect-JALR path (GOT + $t9), so this isn't
		// a rare corner case - measuring the real hit rate rather than
		// trusting the "function entries rarely land on the last page slot"
		// assumption. Incremented directly by JIT'd code at the jmpdest
		// call site only (mips3drc.cpp), not inside
		// generate_select_dispatch_key() itself, so the other 4 call sites
		// (EPC/ErrorPC/branch/entry-point) stay completely unaffected.
		uint32_t        diag_jmpdest_dispatch_total;
		uint32_t        diag_jmpdest_dispatch_crosspage;

		// TEMPORARY diagnostic (2026-08-07) - validate_tlb()'s inline
		// runtime check (mips3drc.cpp, the block-entry/page-boundary TLB
		// verify, unrelated to ASID_FIX - the same mechanism the
		// 0x0fac5ffc investigation traced) execution count, for sizing
		// how much total execution goes to per-dispatch-ish runtime
		// checks versus real guest work (paired with the jmpdest
		// counters above for the same measurement). Incremented directly
		// by JIT'd code at validate_tlb's fast-path site only (the
		// common case - TLB entry present at compile time); the rare
		// unconditional-exception path isn't counted, since that exits
		// immediately and never re-executes this check again for that
		// compile.
		uint32_t        diag_validate_tlb_total;

		// TEMPORARY diagnostic (2026-08-07) - real frequency split between
		// the two independent triggers that mutate vtlb_table() for live
		// (post-reset) entries, both funneling through tlb_map_entry()
		// (mips3com.cpp): bulk ASID-switch remaps (mips3com_asid_changed())
		// vs. single-entry TLBWI/TLBWR writes (tlb_write_common()). Needed
		// before sizing a global-vs-per-ASID epoch counter for the epoch/
		// host-fault redesign - if TLBWI/TLBWR fire far more often than
		// ASID switches, a naive global epoch bumped on every vtlb_table()
		// mutation would over-invalidate every compiled block on each
		// write, reproducing the same recompile-storm shape as the
		// FETCH_ALLOWED regression via a different trigger. Both are plain
		// host C++ counters (mips3com.cpp), not JIT'd - these call sites
		// aren't compiled UML code.
		uint32_t        diag_asid_changed_events;
		uint32_t        diag_tlbwi_tlbwr_events;

		// TEMPORARY diagnostic (2026-08-07) - dedup potential for
		// tlb_map_entry()'s real (non-degenerate) vtlb_load() writes:
		// compares vtlb_table()'s entry before/after each write to see
		// whether the mapping actually changed, or just re-established
		// an identical value. If most of the 5.19M TLBWI/TLBWR-triggered
		// mutations are no-ops, a value-compare before bumping any future
		// epoch counter could cut the effective invalidation rate by
		// orders of magnitude with far less new code than a full per-page
		// epoch redesign. Only covers the "real load" branch (a valid,
		// in-range mapping); the degenerate/unmap branches (ASID
		// mismatch, out-of-range VPN, explicit unmap) are counted
		// separately without a dedup claim, since their actual affected
		// table index isn't cheaply knowable from mips3com.cpp alone.
		uint32_t        diag_tlb_write_total;
		uint32_t        diag_tlb_write_nochange;
		uint32_t        diag_tlb_write_degenerate;

		// TEMPORARY diagnostic (2026-08-22) - resuming the 0x0fac5ffc
		// validate_tlb() anomaly investigation (investigated extensively,
		// still not resolved): every prior check confirmed both the live TLB entry and the
		// compile-time-baked immediate are individually correct and
		// stable, yet the JIT'd CMP still evaluates NE far more often
		// than it should. This field lets an independent, unconditional
		// C-side re-check run on every dispatch through that one PC's
		// validate_tlb site (see cfunc_0fac5ffc_anomaly_diag,
		// mips3drc.cpp) - bumped once per code_compile_block() call for
		// pc==0x0fac5ffc, read back both at compile time (baked as this
		// instance's own "generation") and live (this field's current
		// value) to test whether a stale/older compiled instance could
		// somehow still be reachable and executing after a newer compile
		// of the same dispatch key.
		uint32_t        diag_0fac5ffc_compile_gen;

		// TEMPORARY diagnostic (2026-09-02) - the intermittent gcc
		// <built-in> flake, MAME_SGI_RODATA_LOAD_ADDR_DIAG (mips3drc.cpp). Must live
		// here, not as plain file-scope statics - the x64 DRC backend
		// addresses mem() operands RBP-relative to this struct, and a
		// free-standing global elsewhere in the binary's BSS segment
		// landed out of the encodable displacement range, crashing with
		// "drcbe_x64::offset_from_rbp: delta out of range" the first time
		// generated code actually tried to touch it (confirmed live -
		// this is not a hypothetical). diag_rodata_hit is reset
		// unconditionally at the top of every static_generate_memory_
		// accessor() invocation and only set on this specific call's own
		// in-range check, so stale data from an unrelated earlier access
		// can't leak into the post-read callback.
		uint32_t        diag_rodata_addr;
		uint32_t        diag_rodata_value;
		uint32_t        diag_rodata_hit;

		// MAME_SGI_FASTRAM_COMPARE_DIAG (mips3drc.cpp) - same
		// RBP-relative-struct requirement as diag_rodata_* above, same
		// reason (a free-standing global crashed the x64 DRC backend the
		// same way). addr/size are stashed BEFORE static_generate_
		// fastram_accessor()'s own address-swizzle+load sequence runs
		// (see that function's own comment on why), so they hold the
		// real physical address and access width being served - value
		// holds what the fast path's own inline load just returned, for
		// func_fastram_compare_diag() to check against a fresh read of
		// the same address through the generic slow path.
		uint32_t        diag_fastram_addr;
		uint32_t        diag_fastram_size;
		uint64_t        diag_fastram_value;
		uint32_t        diag_fastram_iswrite;
		uint32_t        diag_fastram_masked;

		// TEMPORARY diagnostic (2026-09-16, QoS follow-up) - real
		// host wall-clock (not guest-cycle) duration of each
		// code_compile_block()/code_flush_cache() call from execute_run().
		// Motivation: this project's scheduler is purely cycle-count-driven
		// (device_scheduler::timeslice() advances every device's clock from
		// icount alone, never host time - confirmed 2026-09),
		// so a compile/flush event doesn't distort the GUEST's own notion
		// of elapsed time, but it does freeze the entire single emulation
		// thread in REAL time - already measured once, ad hoc, as a 373.9ms
		// freeze directly correlated with a full drc_cache flush (the
		// console "Bug B" stall) via two separate indirect diagnostics
		// (MAME_SGI_KBD_SCAN_RATE_DIAG + MAME_SGI_DRC_COMPILE_RATE_DIAG)
		// matched by timestamp after the fact - no direct instrument existed
		// for this until now. This is a genuinely different axis from
		// perf_ab_stats.sh's aggregate instructions/cycles comparison, which
		// can't distinguish "374ms in one lump" from "374ms spread across
		// many small pieces" - only a real wall-clock stall-duration measure
		// can see that difference, which is exactly what a segmented DRC
		// cache would need to demonstrate its value.
		// _over_threshold_count counts events exceeding
		// MAME_SGI_STALL_DIAG_THRESHOLD_NS (mips3.cpp) - the DoS-relevant
		// bucket a human would perceive as a stutter/dropped input, as
		// opposed to routine sub-millisecond compiles.
		uint64_t        diag_compile_stall_total_ns;
		uint64_t        diag_compile_stall_max_ns;
		uint32_t        diag_compile_stall_over_threshold_count;
		uint32_t        diag_compile_stall_event_count;
		uint64_t        diag_flush_stall_total_ns;
		uint64_t        diag_flush_stall_max_ns;
		uint32_t        diag_flush_stall_over_threshold_count;
		uint32_t        diag_flush_stall_event_count;

		// MAME_SGI_COMPILE_LENGTH_CORRELATION_DIAG (2026-09-16, QoS
		// follow-up) - is a long compile a function of how far describe_code()
		// had to traverse (window_limit/branch-driven sequence length), or
		// something else? code_compile_block() (mips3drc.cpp) counts the total
		// opcode_desc nodes in this call's desclist (every instruction in
		// every sequence describe_code() returned, not just one sequence -
		// a single call can batch several) into diag_last_compile_instruction_
		// count right after describe_code() returns; execute_run() (mips3.cpp)
		// reads it back alongside the same wall-clock duration already being
		// measured, accumulating the running sums needed for a Pearson
		// correlation coefficient (computed once, at clean exit, from these
		// sums - no per-event log needed for the aggregate answer). The two
		// outlier fields capture the single longest event's own PC/count/
		// duration directly, since with only ~2 events/boot over the 10ms
		// threshold, individually inspecting them is cheap and more concrete
		// than a coefficient alone.
		uint32_t        diag_last_compile_instruction_count;

		// Fix (2026-09-17, QoS follow-up, found while interpreting
		// stall_ab_stats.sh's pearson_r/outlier output): code_compile_block()
		// (mips3drc.cpp) has a third code_flush_cache() call site - the
		// abort_compilation catch block inside its own while(!succeeded)
		// retry loop, hit whenever the near/permanent cache runs out mid-
		// codegen - that isn't one of the two call sites execute_run()
		// (mips3.cpp) times as "flush". That call regenerates the ENTIRE
		// static handler set (entry point, nocode/out_of_cycles/tlb_mismatch,
		// ~14 exception handlers, memory accessors for 3 modes - ~50+ real
		// codegen blocks) every time, but its cost was previously silently
		// folded into whatever compile call happened to trigger it -
		// producing outliers like "instructions=1, ns=13ms" and diluting
		// diag_compile_corr_*'s correlation, while diag_flush_stall_* stayed
		// near-zero despite drc_cache's own flush_count() reaching ~10-11/
		// boot. code_compile_block() now times this call site itself (same
		// MAME_SGI_STALL_DURATION_DIAG chrono pattern as execute_run()'s two
		// sites) and accumulates the result here; execute_run() reads it
		// back right after code_compile_block() returns and subtracts it
		// from that call's own elapsed time before feeding diag_compile_
		// stall_*/diag_compile_corr_*, so "compile" reflects only describe_
		// code()+codegen cost and "flush" finally captures this dominant,
		// previously-invisible mechanism. Reset to 0 by execute_run() after
		// each read; accumulates (rather than overwrites) in case the retry
		// loop hits abort_compilation more than once in a single call.
		uint64_t        diag_last_compile_flush_ns;

		// TEMPORARY diagnostic (2026-09-17, QoS follow-up,
		// continuing the investigation once the flush-attribution fix
		// above ruled out the internal-flush mechanism as the cause of
		// the ~10-13ms "long compile" outliers still seen after that fix).
		// Narrows the still-open question this diagnostic's own original
		// comment posed ("is a long compile a function of how far
		// describe_code() had to traverse... or something else?") by
		// timing describe_code() (mips3drc.cpp's call into it) separately
		// from the rest of code_compile_block() - the front-end decode/
		// window-walk pass (drcfe.ipp's build_sequence(), called twice per
		// describe_code() - once forward, once backward from startpc - up
		// to m_max_sequence or a window/branch-target boundary) is a
		// plausible source of cost NOT reflected in diag_last_compile_
		// instruction_count, which only counts what survives into the
		// final compiled desclist, not how far the walk had to probe to
		// get there. Set unconditionally each call, right after describe_
		// code() returns (mirrors diag_last_compile_instruction_count's
		// own placement/lifetime - no separate reset needed, always
		// overwritten before use).
		uint64_t        diag_last_describe_code_ns;
		uint64_t        diag_compile_corr_n;
		double          diag_compile_corr_sum_x;   // instruction counts
		double          diag_compile_corr_sum_y;   // durations (ns)
		double          diag_compile_corr_sum_xy;
		double          diag_compile_corr_sum_x2;
		double          diag_compile_corr_sum_y2;
		uint64_t        diag_compile_longest_ns;
		uint32_t        diag_compile_longest_pc;
		uint32_t        diag_compile_longest_instruction_count;
		// describe_code()-share tracking (2026-09-17) alongside the above -
		// see diag_last_describe_code_ns's comment above for what this is
		// isolating. Running total lets execute_run() report describe_
		// code()'s mean share of total compile time at clean exit;
		// longest-event's own describe_ns is captured alongside its
		// existing pc/ns/instruction_count fields so a single outlier can
		// be read as "how much of this was describe_code()?" directly.
		uint64_t        diag_describe_code_total_ns;
		uint64_t        diag_compile_longest_describe_ns;

		// Duration histogram (2026-09-17, QoS follow-up) - added
		// after describe_code() was directly measured and ruled out as the
		// outlier cause (0.01-0.16% share of the ~11-14ms events), to
		// answer a prior question before drilling further: are these truly
		// 2 sharp, isolated outliers per boot, or the visible tail of a
		// smoother distribution the single "longest event" field can't
		// show? Log-scale buckets, boundaries in nanoseconds: [0]<1000ns,
		// [1]<10,000ns, [2]<100,000ns, [3]<1,000,000ns, [4]<10,000,000ns,
		// [5]>=10,000,000ns. Indexed by the same corrected (flush-
		// subtracted) per-event ns already fed to diag_compile_corr_*.
		uint64_t        diag_compile_duration_histogram[6];

		// TEMPORARY diagnostic (2026-09-17, fixed-checkpoint checkints experiment, Phase 0) - real
		// host wall-clock frequency of the two mechanisms that can deliver
		// an interrupt under DRC: static_generate_entry_point()'s
		// unconditional per-re-entry check, and static_generate_out_of_
		// cycles() (which doesn't check interrupts itself, but re-entering
		// through entry_point almost always follows it - see this diag's
		// own comment in mips3drc.cpp for the full motivation). Answers a
		// question this project's own 2026-07-26 investigation
		// (the mrupdatef livelock) left unexplained: given
		// entry-point's check was already active then and still missed a
		// real interrupt for 10 real wall-clock seconds, how often does
		// host C++ actually regain control in practice? Lightweight
		// in-memory counters + running interval stats only - no per-event
		// log, per this project's own hard-won lesson about unfiltered
		// diagnostics (the MAME_SGI_TASK84_WINDOW_TRACE incident - 20
		// iterations, 2.3M+ events, 367MB log). Reported once as a
		// clean-exit summary, same convention as every other *_DIAG in
		// this file.
		uint64_t        diag_entry_reentry_count;
		uint64_t        diag_entry_last_timestamp_ns;   // 0 means "no prior sample yet"
		uint64_t        diag_entry_interval_total_ns;
		uint64_t        diag_entry_interval_max_ns;
		uint64_t        diag_entry_interval_min_ns;      // starts at UINT64_MAX, see reset in device_start()
		uint64_t        diag_out_of_cycles_count;

		// Added 2026-09-17 (fasthz A/B) - alongside the wall-
		// clock interval above, also snapshot device_execute_interface::
		// total_cycles() (real guest cycles executed, cumulative) at every
		// re-entry, and remember the guest-cycle delta specifically for
		// whichever interval turned out to be the longest in wall-clock
		// terms. Lets the longest gap be read two ways: a "legitimately
		// large guest-time quantum" (many guest cycles - consistent with
		// scheduler behavior, schedule.cpp's own target computation) shows
		// a large delta here; a "host execution stalled" gap (DRC compile
		// storm, I/O, etc.) shows a small delta despite the long wall
		// time. Distinguishes the two explanations directly instead of
		// guessing from the wall-clock number alone.
		uint64_t        diag_entry_last_total_cycles;
		uint64_t        diag_entry_max_interval_guest_cycles;

		// MAME_DRC_BUGFIXES_NULL_SEQLAST_DIAG (mips3drc.cpp) - how often
		// code_compile_block()'s null-seqlast fallback fires (the
		// permanent fix itself is unconditional, see its own comment) and
		// the seqhead PC of the most recent occurrence. Reported once as
		// a clean-exit summary in device_stop() (mips3.cpp). Root cause:
		// build_sequence() in drcfe.ipp discarding a reclaimed
		// branch-likely description's end_sequence() flag.
		uint32_t        diag_null_seqlast_count;
		uint32_t        diag_null_seqlast_last_pc;

		// Standing counters (2026-08-22, not a temporary diagnostic -
		// this is the fix itself, MAME_SGI_TLB_MISMATCH_REFRESH_FIX):
		// how often static_generate_tlb_mismatch()'s skip branch
		// was able to redispatch through the hash table instead of
		// falling through to EXECUTE_MISSING_CODE (refresh_hits -
		// recompile avoided) out of every arrival at that branch
		// (skip_total). See mips3drc.cpp's own comment on the fix for
		// the full rationale.
		uint32_t        diag_tlb_mismatch_skip_total;
		uint32_t        diag_tlb_mismatch_refresh_hits;

		// Part of MAME_SGI_TLB_MISMATCH_REFRESH_FIX - the last
		// guest PC the shared tlb_mismatch handler redispatched rather
		// than recompiling. Read back on the next arrival at that
		// handler's skip branch: if it is the same PC, the redispatch
		// landed straight back on the very stub that raised the mismatch
		// (a compiler_page_fault() sequence head registers its hash entry
		// at its own PC), so retrying is futile and the real recompile is
		// taken instead. Initialised to ~0, which is never a valid
		// word-aligned PC. See static_generate_tlb_mismatch().
		uint32_t        tlb_mismatch_retry_pc;

		// Standing counters (2026-09-16, QoS investigation -
		// MAME_SGI_NOCODE_REFRESH_FIX): the shared nocode handler
		// (static_generate_nocode_handler) is the universal fallthrough for
		// every UML_HASHJMP miss in this file, not just the tlb_mismatch
		// path above - static_dispatch_key() bakes its resolved key as a
		// compile-time constant, which goes stale whenever a page's
		// writability classification changes after that jump was compiled
		// (routine under ASID switches), producing a hash miss even though
		// valid code already exists under the CURRENT resolved key.
		// Measured before this fix (MAME_SGI_VALIDATE_TLB_0FAC5FFC_DIAG,
		// 2026-09-16): 99.985% of arrivals at pc=0x0fac5ffc's compile were
		// exactly this - a "key_mismatch" (resolved-key code already
		// present, raw/stale key missed). Same shape, same fix pattern as
		// diag_tlb_mismatch_skip_total/refresh_hits above - see
		// static_generate_nocode_handler() for the mechanism.
		uint32_t        diag_nocode_skip_total;
		uint32_t        diag_nocode_refresh_hits;

		// Part of MAME_SGI_NOCODE_REFRESH_FIX - mirrors tlb_mismatch_retry_pc
		// above, one-shot loop guard for the nocode handler's own redispatch
		// attempt. Initialised to ~0 (never a valid word-aligned PC).
		uint32_t        nocode_retry_pc;

		// TEMPORARY diagnostic (2026-08-07) - real compiled-sequence
		// length distribution (seqhead-to-seqlast, in guest instructions),
		// for characterizing DRC dispatch fragments after the interpreter-
		// vs-DRC comparison showed only a ~2.15x wall-time win - far short
		// of typical JIT speedups. An existing comment in code_compile_
		// block() (2026-08-02 investigation) already found "231K+ compiles,
		// all identical single-instruction sequences" for one specific
		// case; this makes that observation a standing aggregate instead
		// of an anecdote from one investigation. Plain host C++ counters
		// (mips3drc.cpp), not JIT'd - counted once per sequence at compile
		// time, not per dispatch.
		uint32_t        diag_seq_count;
		uint64_t        diag_seq_instr_total;
		uint32_t        diag_seq_len1_count;
		uint32_t        diag_seq_len_max;

		// TEMPORARY diagnostic (2026-08-08) - categorizes *why* each
		// sequence ended, using seqlast's own flags (mirroring
		// describe()'s set_end_sequence() call sites in mips3fe.cpp).
		// IRIX's PIC ABI resolves nearly every cross-DSO call through
		// indirect JALR (established 2026-08-06/07), and JR/JALR always
		// end a sequence since the target isn't statically known - the
		// leading hypothesis for why sequences average only ~7
		// instructions (2026-08-07 measurement). This splits that
		// hypothesis from the alternatives (static jumps/calls,
		// syscall/break, mode-changing COP0 writes/ERET, or simply
		// running off the end of the describe_code() window/
		// COMPILE_MAX_SEQUENCE cap with no terminating instruction at
		// all) with real counts instead of just plausibility.
		uint32_t        diag_seqend_indirect_jump;   // JR/JALR (targetpc == BRANCH_TARGET_DYNAMIC)
		uint32_t        diag_seqend_static_jump;     // J/JAL/unconditional-idiom branches (static target)
		uint32_t        diag_seqend_exception;       // SYSCALL/BREAK
		uint32_t        diag_seqend_mode_change;     // MTC0-Status/ERET
		uint32_t        diag_seqend_window_limit;    // none of the above - window/max-sequence boundary

		// TEMPORARY diagnostic (2026-08-08) - which of the three sequence-
		// exit transfer tiers actually gets used for the non-branch
		// fall-through case (code_compile_block()'s per-sequence dispatch
		// selection, ~line 1151): free (zero-cost straight-line fall-
		// through - the very next sequence in this same compile batch
		// already starts at nextpc, no code emitted at all),
		// mode_change_hashjmp (seqlast->can_change_modes(), a real runtime
		// hash-table lookup via live_dispatch_key()), or static_hashjmp
		// (seqlast->next() is null or doesn't start at nextpc - also a
		// real hash-table lookup, just with static_dispatch_key()'s
		// compile-time-resolved key). Tests whether the dominant
		// branch_target sequence-termination cause (2026-08-08, ~31% of
		// all terminations) already gets the free tier for free (the
		// split-off sequence is typically the very next node in program
		// order) or actually pays the expensive hash-lookup path despite
		// being reachable without one.
		uint32_t        diag_seqexit_free_fallthrough;
		uint32_t        diag_seqexit_mode_change_hashjmp;
		uint32_t        diag_seqexit_static_hashjmp;

		// TEMPORARY diagnostic (2026-08-08) - of the static_hashjmp cases
		// above (66% of non-branch sequence exits), how many actually have
		// nextpc's own opcode_desc compiled somewhere in the SAME batch's
		// desclist (just not literally seqlast->next() - list order is
		// whatever order do_describe_code()'s branch-target-stack walk
		// discovered sequences in, not address order) versus genuinely
		// outside this batch, needing the real hash lookup regardless.
		// Sizes the actual opportunity for extending the existing
		// intrablock_branch mechanism (currently gated to curdesc-
		// >is_branch() only, mips3drc.cpp:4391) to this fall-through
		// dispatch path too.
		uint32_t        diag_seqexit_hashjmp_target_in_batch;
		uint32_t        diag_seqexit_hashjmp_target_outside_batch;

		// TEMPORARY diagnostic (2026-08-08) - live values read by
		// func_callsite_ic_probe() (mips3drc.cpp), fed from the jmpdest
		// HASHJMP call site (generate_delay_slot_and_branch). diag_ic_probe_
		// callsite is desc->pc, baked in at compile time as this JR/JALR's
		// own address (the call-site identity); diag_ic_probe_mode is the
		// live resolved dispatch key (I3). jmpdest itself already holds the
		// live target pc, read directly by the probe - no separate field
		// needed. See m_diag_callsite_ic_state's comment for the full
		// mechanism/purpose.
		uint32_t        diag_ic_probe_callsite;
		uint32_t        diag_ic_probe_mode;

		// PROTOTYPE (2026-08-08) - live values read by
		// func_ic_learn_and_arm() (mips3drc.cpp), fed from the jmpdest
		// guard-miss/establish path (generate_delay_slot_and_branch).
		// ic_live_mode is the live resolved dispatch key (I3), stashed to
		// memory because it's not guaranteed to survive the CALLC's
		// underlying host call. ic_slot_index identifies which slot just
		// missed, a compile-time immediate at each call site
		// (= hash(desc->pc) % MAME_SGI_IC_SITE_COUNT). jmpdest itself
		// already holds the live target pc. An earlier version of this
		// design also needed a third field here (the call site's own
		// instruction address, for byte-patching) - no longer necessary
		// now that OP_DIRECTJMP (uml.h/drcbex64.cpp) reads its target
		// from a fixed, compile-time-known host address
		// (&m_core->ic_site_target[idx]) rather than needing any runtime-
		// captured instruction address at all. See the class-scope
		// comment near m_diag_callsite_ic_state for the full mechanism.
		uint32_t        ic_live_mode;
		uint32_t        ic_slot_index;

		// PROTOTYPE (2026-08-08): the per-call-site guard-based inline
		// cache's slot table itself - parallel arrays (not an array of
		// structs, to avoid a nested-struct forward reference to a type
		// declared later in this class) of MAME_SGI_IC_SITE_COUNT guard
		// expectations plus their resolved targets, indexed by
		// ic_slot_index. Deliberately embedded directly in
		// internal_mips3_state so it's automatically near-cache-resident
		// (same allocation as m_core itself) - critically, this also
		// means &ic_site_target[idx] is a fixed, compile-time-known host
		// address usable directly as OP_DIRECTJMP's PTR parameter, with
		// none of the "capture an assembled instruction's address" timing
		// problems the earlier byte-patch design fought - see the
		// class-scope comment near m_diag_callsite_ic_state for the full
		// design/history.
		uint32_t        ic_site_expected_mode[MAME_SGI_IC_SITE_COUNT];
		uint32_t        ic_site_expected_pc[MAME_SGI_IC_SITE_COUNT];
		uint64_t        ic_site_target[MAME_SGI_IC_SITE_COUNT];

													/* parameters for subroutines */
		uint64_t        numcycles;                  /* return value from gettotalcycles */
		const char *    format;                     /* format string for print_debug */
		uint32_t        arg0;                       /* print_debug argument 1 */
		uint32_t        arg1;                       /* print_debug argument 2 */
		uint64_t        fp_convert_src;             /* source of a float to integer conversion (MAME_SGI_FP_CONVERT_FIX) */

		uint64_t        count_zero_time;
		uint32_t        compare_armed;
		uint32_t        jmpdest;                    /* destination jump target */
		uint32_t        diag_call_trace_target;     /* TEMPORARY (2026-09-09): JAL/JALR target for func_call_trace_diag */
		uint32_t        diag_exc_type;               /* TEMPORARY (2026-09-10): exception code for log_drc_exception */
		uint32_t        diag_tlb_mismatch_pc;        /* TEMPORARY (2026-09-10): recovered PC for log_tlb_mismatch_entry */
		uint32_t        diag_cpperror_callsite_pc;  /* TEMPORARY (2026-09-10): this call's own desc->pc for func_cpperror_callsite_diag */
		uint64_t        diag_cpperror_a2;            /* TEMPORARY (2026-09-10): $a2 (message ptr) at the real cpp_error call */
		uint64_t        diag_cpperror_s3;            /* TEMPORARY (2026-09-10): $s3 at the real cpp_error call */
		uint64_t        diag_cond_v1;                /* TEMPORARY (2026-09-10): $v1 (token type) for func_cpperror_cond_trace_diag */
		uint64_t        diag_cond_s1;                /* TEMPORARY (2026-09-10): $s1 for func_cpperror_cond_trace_diag */
		uint64_t        diag_srctext_s0;             /* TEMPORARY (2026-09-10): $s0 (pfile) for func_source_text_diag */
		uint64_t        diag_srctext_s3;             /* TEMPORARY (2026-09-10): $s3 (loop iteration/acount) for func_source_text_diag */
		uint64_t        diag_token_result_v0;       /* TEMPORARY (2026-09-09): stashed $v0 for func_token_result_diag */
		uint64_t        diag_beql_s3;                /* TEMPORARY (2026-09-10): $s3 read right at the beql compare itself, for func_beql_s3_diag */
		uint64_t        diag_beql_v0;                /* TEMPORARY (2026-09-10): $v0 at the same point, for func_beql_s3_diag */
		uint64_t        diag_breakcheck_v0;          /* TEMPORARY (2026-09-10): $v0 (token type) at the loop's own break check, 0x1007df68 */
		uint64_t        diag_breakcheck_s3;          /* TEMPORARY (2026-09-10): $s3 at the same point */
		uint64_t        diag_s3_increment_newval;    /* TEMPORARY (2026-09-10): $s3's new value right after the loop's own acount++ */
		uint64_t        diag_tlbmiss_redispatch_s3;  /* TEMPORARY (2026-09-10): $s3 at the moment static_generate_tlb_mismatch redispatches back to 0x1007DFFC (mid-delay-slot TLB miss test) */
		uint32_t        diag_tlbmiss_redispatch_firstpc; /* TEMPORARY (2026-09-10): first UML_RECOVER's (delay-slot-rounded-up) PC, for the same event */
		uint32_t        diag_new_epc;                /* TEMPORARY (2026-07-27): stashed I0 value about to become EPC in static_generate_exception's EXCEPTION_INTERRUPT case, read by func_interrupt_delivery_probe */
		uint64_t        cop0_store_value;            /* value being written by mtc0, preserved across the interrupt check that precedes the store */
		uint32_t        stub_container_pc;           /* MAME_SGI_STALE_STUB_FIX: entry pc of the block whose compiler_page_fault stub last fired */
		uint32_t        stub_container_mode;         /* MAME_SGI_STALE_STUB_FIX: that block's hash key */
		uint32_t        stub_site_pc;                /* MAME_SGI_STALE_STUB_FIX: pc tlb_mismatch recovers for that stub (~0 when none pending) */
		uint32_t        diag_study_serial;           /* MAME_SGI_RECOMPILE_STUDY_DIAG: compile serial baked into a traced validate_tlb check */
		uint32_t        diag_study_baked;            /* MAME_SGI_RECOMPILE_STUDY_DIAG: compile-time vtlb entry of the last failed validate_tlb check */
		uint32_t        diag_recompile_origin;       /* MAME_SGI_RECOMPILE_STUDY_DIAG: which handler requested the pending recompile (1 = nocode, 2 = tlb_mismatch) */
		uint32_t        diag_new_status;             /* TEMPORARY (2026-07-27): stashed I0 value about to be stored to COP0_Status in generate_set_cop0_reg, read by func_status_store_probe */
		uint32_t        diag_status_store_pc;        /* TEMPORARY (2026-07-27): desc->pc of the mtc0 about to run, stashed alongside diag_new_status */
		uint64_t        diag_mtcz_i0_stash;          /* TEMPORARY (2026-07-27): I0 stashed/restored around func_mtcz_read_probe's CALLC so it can't clobber the value generate_set_cop0_reg consumes right after */
		uint32_t        diag_sd_i0_stash;            /* TEMPORARY (2026-07-27): I0 (store address) stashed/restored around func_sd_value_probe's CALLC in the SD opcode case */
		uint64_t        diag_sd_i1_stash;            /* TEMPORARY (2026-07-27): I1 (store value) stashed/restored around func_sd_value_probe's CALLC in the SD opcode case */
		uint32_t        diag_mxcsr_desync_pc;        /* TEMPORARY (2026-08-24): guest pc of the instruction boundary where real MXCSR RC diverged from what CCR1(31)'s RM field predicts, read by func_mxcsr_desync_diag */
		uint32_t        diag_mxcsr_desync_guest_rm;  /* TEMPORARY (2026-08-24): CCR1(31)&3 at the same boundary, stashed alongside diag_mxcsr_desync_pc */
		uint32_t        diag_mxcsr_desync_real_rc;   /* TEMPORARY (2026-08-24): real live MXCSR RC bits (13:14) read via the repurposed GETFMOD, stashed alongside diag_mxcsr_desync_pc */
		uint64_t        diag_task84_inert_scratch;   /* TEMPORARY (2026-09-04): register-inert control-test destination for MAME_SGI_TASK84_INERT_ONLY - must live in m_core, not a free-standing static, so drcbe_x64's compact RBP-relative addressing can reach it (a free-standing static crashed with "offset_from_rbp: delta out of range") */
		uint32_t        diag_task84_armed;           /* TEMPORARY (2026-09-04): MAME_SGI_TASK84_WINDOW_TRACE - set to 1 at pc==0x1007df50 (right after cpp_get_token() returns), cleared at pc==0x1007dffc (the loop-exit check) - confines save/reload tracing below to the exact narrow window between them, regardless of which handler/process's activity fires it */
		uint32_t        diag_task84_trace_site;      /* TEMPORARY (2026-09-04): site tag set immediately before each MAME_SGI_TASK84_WINDOW_TRACE CALLC (1=nocode, 2=out_of_cycles, 3=tlb_mismatch, 4=exception) so one shared logging function can identify which save_fast_iregs() call site fired */
		uint64_t        diag_task84_last_saved_r2;   /* TEMPORARY (2026-09-04): shadow copy of m_core->r[2] written by save_fast_iregs() itself (so ALL 9 call sites are covered by construction, not by patching each one). The entry point compares the live r[2] against this immediately before load_fast_iregs() - a mismatch means r[2] changed between the last save and this reload, which is precisely the stale-reload the fast-register audit is hunting. */
		uint32_t        diag_ctc1_mxcsr_pc;          /* TEMPORARY (2026-08-24): guest pc of the CTC1 whose own just-emitted UML_SETFMOD didn't produce the expected real MXCSR RC, read by func_ctc1_mxcsr_check_diag */
		uint32_t        diag_ctc1_mxcsr_fmod;        /* TEMPORARY (2026-08-24): the fmod value (uml::float_rounding_mode) that CTC1 just requested via UML_SETFMOD, stashed alongside diag_ctc1_mxcsr_pc */
		uint32_t        diag_ctc1_mxcsr_real_rc;     /* TEMPORARY (2026-08-24): real live MXCSR RC bits actually observed right after that UML_SETFMOD, stashed alongside diag_ctc1_mxcsr_pc */
		uint32_t        diag_sd_pc;                  /* TEMPORARY (2026-07-27): desc->pc of the SD instruction, for func_sd_value_probe to report */
		uint64_t        diag_task84_save_count;      /* TEMPORARY (2026-09-05): MAME_SGI_TASK84_SAVELOAD_BALANCE_DIAG - incremented by save_fast_iregs() itself (covers all 4 active call sites by construction), read alongside diag_task84_load_count to check the save/load structural balance the user proposed. */
		uint64_t        diag_task84_load_count;      /* TEMPORARY (2026-09-05): companion to diag_task84_save_count, incremented by load_fast_iregs() (the single active call site, static_generate_entry_point). Expected to stay within {0,1} of diag_task84_save_count if save/load structurally alternate as assumed - an imbalance beyond that would be a genuine, previously-unverified structural finding. */
		uint64_t        diag_task84_bracket_save_entry; /* TEMPORARY (2026-09-05): MAME_SGI_TASK84_SAVELOAD_BRACKET_DIAG - snapshot of diag_task84_save_count taken at cpp_get_token's own call site (0x1007df48), so func_task84_saveload_bracket_exit (at the return site, 0x1007df50) can compute the exact save/load activity delta for ONE invocation in guest-instruction order, not wall-clock. */
		uint64_t        diag_task84_bracket_load_entry; /* TEMPORARY (2026-09-05): companion to diag_task84_bracket_save_entry, for diag_task84_load_count. */
		uint64_t        diag_v0_shadow;               /* TEMPORARY (2026-09-07): MAME_SGI_TASK84_V0_SHADOW_DIAG - mirrors the live value of R64(2) ($v0), updated unconditionally after every guest instruction (generate_sequence_instruction). Compared against the live value again before the NEXT instruction; any mismatch is an out-of-band change (something touched $v0 without going through this same instrumented path) - see the macro's own comment for the full design. */
		uint32_t        diag_v0_divergence_pc;        /* TEMPORARY (2026-09-07): desc->pc of the instruction boundary where a divergence was NOTICED (not necessarily where it was caused), stashed for func_v0_shadow_diag. */
		uint64_t        diag_v0_divergence_current;   /* TEMPORARY (2026-09-07): the live R64(2) value at the moment of the divergence, stashed alongside diag_v0_divergence_pc - diag_v0_shadow itself still holds the OLD (pre-divergence) value at logging time, so both are visible. */
	};

	/* core state */
	internal_mips3_state *m_core;
	uint8_t *m_dcache;
	uint8_t *m_icache;

	address_space_config m_program_config;
	memory_access<32, 2, 0, ENDIANNESS_LITTLE>::cache m_cache32le;
	memory_access<32, 3, 0, ENDIANNESS_LITTLE>::cache m_cache64le;
	memory_access<32, 2, 0, ENDIANNESS_BIG>::cache m_cache32be;
	memory_access<32, 3, 0, ENDIANNESS_BIG>::cache m_cache64be;

	mips3_flavor    m_flavor;

	/* internal stuff */
	uint32_t        m_ppc;
	uint32_t        m_nextpc;
	uint32_t        m_pcbase;
	uint8_t         m_cf[4][8];
	bool            m_delayslot;
	int             m_op;
	int             m_interrupt_cycles;
	uint32_t        m_ll_value;
	uint64_t        m_lld_value;
	uint32_t        m_badcop_value;

	/* endian-dependent load/store */
	typedef void (mips3_device::*loadstore_func)(uint32_t op);
	loadstore_func  m_lwl;
	loadstore_func  m_lwr;
	loadstore_func  m_swl;
	loadstore_func  m_swr;
	loadstore_func  m_ldl;
	loadstore_func  m_ldr;
	loadstore_func  m_sdl;
	loadstore_func  m_sdr;

	address_space * m_program;
	uint32_t        m_data_bits;
	delegate<u32 (offs_t)> m_pr32;
	std::function<const void * (offs_t)> m_prptr;
	uint32_t        c_system_clock;
	uint32_t        m_cpu_clock;
	emu_timer *     m_compare_int_timer;
	uint32_t        m_tlb_seed;

	/* derived info based on flavor */
	uint32_t        m_pfnmask;
	uint8_t         m_tlbentries;

	/* memory accesses */
	bool            m_bigendian;
	uint32_t        m_byte_xor;
	uint32_t        m_word_xor;
	uint32_t        m_dword_xor;

	/* cache memory */
	size_t          c_icache_size;
	size_t          c_dcache_size;
	uint8_t         c_secondary_cache_line_size;

	/* MMU */
	mips3_tlb_entry m_tlb[MIPS3_MAX_TLB_ENTRIES];

	/* fast RAM */
	uint32_t        m_fastram_select;
	struct {
		offs_t      start;                      /* start of the RAM block */
		offs_t      end;                        /* end of the RAM block */
		bool        readonly;                   /* true if read-only */
		void *      base;                       /* base in memory where the RAM lives */
		uint8_t *   offset_base8;               /* base in memory where the RAM lives, 8-bit pointer, with the start offset pre-applied */
		uint16_t *  offset_base16;              /* base in memory where the RAM lives, 16-bit pointer, with the start offset pre-applied  */
		uint32_t *  offset_base32;              /* base in memory where the RAM lives, 32-bit pointer, with the start offset pre-applied  */
	}               m_fastram[MIPS3_MAX_FASTRAM];

	uint32_t        m_debugger_temp;

	/* core state */
	drc_cache       m_drc_cache;                /* pointer to the DRC code cache */
	std::unique_ptr<drcuml_state>  m_drcuml;    /* DRC UML generator state */
	std::unique_ptr<frontend>      m_drcfe;     /* pointer to the DRC front-end state */
	uint32_t        m_drcoptions;               /* configurable DRC options */

												/* internal stuff */
	uint8_t         m_drc_cache_dirty;          /* true if we need to flush the cache */

												/* tables */
	uint8_t         m_fpmode[4];                /* FPU mode table */

												/* register mappings */
	uml::parameter  m_regmap[34];               /* parameter to register mappings for all 32 integer registers */
	uml::parameter  m_regmaplo[34];             /* parameter to register mappings for all 32 integer registers */

												/* subroutines */
	uml::code_handle *   m_entry;                      /* entry point */
	uml::code_handle *   m_nocode;                     /* nocode exception handler */
	uml::code_handle *   m_out_of_cycles;              /* out of cycles exception handler */
	uml::code_handle *   m_tlb_mismatch;               /* tlb mismatch handler */
	uml::code_handle *   m_read8[3];                   /* read byte */
	uml::code_handle *   m_write8[3];                  /* write byte */
	uml::code_handle *   m_read16[3];                  /* read half */
	uml::code_handle *   m_write16[3];                 /* write half */
	uml::code_handle *   m_read32[3];                  /* read word */
	uml::code_handle *   m_read32mask[3];              /* read word masked */
	uml::code_handle *   m_write32[3];                 /* write word */
	uml::code_handle *   m_write32mask[3];             /* write word masked */
	uml::code_handle *   m_read64[3];                  /* read double */
	uml::code_handle *   m_read64mask[3];              /* read double masked */
	uml::code_handle *   m_write64[3];                 /* write double */
	uml::code_handle *   m_write64mask[3];             /* write double masked */
	uml::code_handle *   m_exception[18/*EXCEPTION_COUNT*/]; /* array of exception handlers */
	uml::code_handle *   m_exception_norecover[18/*EXCEPTION_COUNT*/];   /* array of no-recover exception handlers */

																		 /* hotspots */
	uint32_t        m_hotspot_select;
	struct {
		offs_t      pc;                         /* PC to consider */
		uint32_t    opcode;                     /* required opcode at that PC */
		uint32_t    cycles;                     /* number of cycles to eat when hit */
	}               m_hotspot[MIPS3_MAX_HOTSPOTS];
	bool            m_isdrc;

	// MAME_DRC_BUGFIXES (2026-07-28): persistent, address-keyed record of PCs
	// that have ever been found to need checkints/checksoftints armed by any
	// of code_compile_block()'s detection scans. describe_code() rebuilds its
	// sequence/branch-target analysis from scratch on every compile, scoped
	// only to that call's own window - the same address can be correctly
	// flagged this time and silently missed next time purely because a
	// different neighboring PC triggered the compile, causing the same
	// (pc,state) to be recompiled and rehashed over and over (confirmed live:
	// one hot PC recompiled 108 times across only 13 cache flushes). Once an
	// address earns checkints here, via any window, it stays armed on every
	// future compile regardless of what that particular call's own analysis
	// finds - see code_compile_block() for where this is consulted/updated.
	std::unordered_set<offs_t> m_checkints_addrs;

	// TEMPORARY diagnostic (2026-09-17, checkints scoping) - what fraction
	// of all distinct compiled PCs actually end up in m_checkints_addrs
	// above? Answers the open scoping question directly rather than
	// guessing between "bound growth via clear-on-flush" (sound if this is
	// a modest minority - the sticky-set granularity is doing real work)
	// and "question the whole mechanism" (if it's a large majority,
	// per-PC tracking is solving the wrong problem and checkints should
	// probably be closer to unconditional instead). Tracks the same
	// seqhead->pc key, at the same call site, unconditionally (regardless
	// of whether checkints ends up armed) - see code_compile_block() for
	// where this is populated. Reserved generously up front (mirroring
	// this session's own m_diag_usermode_recompile_counts fix) so this
	// diagnostic doesn't introduce a new rehash-driven stall while
	// measuring one.
	std::unordered_set<offs_t> m_diag_checkints_all_compiled_pcs;

	// TEMPORARY diagnostic (2026-07-28): the literal `pc` argument passed to
	// the currently-executing code_compile_block() call - the address that
	// genuinely caused an EXECUTE_MISSING_CODE miss. Set at the top of every
	// call; consulted deep inside per-instruction codegen (generate_set_
	// cop0_reg's COMPILETIME diagnostic) to tell a genuine forced recompile
	// of a hot PC apart from that same PC merely being walked as a member of
	// some *other* sequence's compile - the two were being conflated into
	// one inflated count throughout tonight's redundant-recompile
	// investigation.
	offs_t m_diag_compile_entry_pc = 0;

	// TEMPORARY diagnostic (2026-08-01): validating a theory about
	// USER-mode (application) code recompilation during multi-process
	// operation (rc-script boot phase and beyond) - the DRC hash table
	// key is (mode, virtual pc) with no ASID/process discrimination, and
	// the per-block checksum guard only proves "content at this frozen
	// physical address is unchanged," not "this virtual PC still
	// resolves to that address under the current process/ASID." Two
	// different processes whose private code occupies the same virtual
	// range (the common case under a fixed-load-address ABI) could
	// plausibly thrash each other's cached compiles on every context
	// switch. Counts recompile events per (mode, pc) key, USER mode
	// only - see code_compile_block() for where this is populated and
	// mips3.cpp's device_stop() for where it's dumped (unconditionally
	// safe when empty/unused - see MAME_DRC_USERMODE_RECOMPILE_DIAG in
	// mips3drc.cpp for the gate on actually populating it).
	std::unordered_map<uint64_t, uint32_t> m_diag_usermode_recompile_counts;

	// MAME_SGI_RECOMPILE_STUDY_DIAG (mips3drc.cpp): classifies every
	// USER-mode compile request by cause and records the code content, for
	// the recompile study. Populated in code_compile_block(),
	// written to a TSV file by recompile_study_dump() from device_stop().
	struct recompile_study_key
	{
		uint32_t count = 0;
		uint32_t last_flush = 0;
		uint64_t last_hash = 0;
		uint64_t last_own_hash = 0;         // hash of the window's descriptors on pc's own page only
		uint32_t last_physpc = 0;
		bool last_was_stub = false;         // the previous compile was a page-fault stub at pc itself
	};
	struct recompile_study_variant
	{
		uint64_t hash;
		uint32_t count;
		uint32_t words[8];
	};
	struct recompile_study_pc
	{
		uint32_t cls[5][2] = {};            // [class][content changed vs. this key's previous compile]
		uint32_t shared_key = 0;            // compiles registered under the ASID-independent key
		uint32_t asid_key = 0;              // compiles registered under an ASID-scoped key
		uint32_t stub_same_page = 0;        // compiles whose window hit a page fault on pc's own page
		uint32_t stub_next_page = 0;        // ...on a later page (block crosses into an unmapped page)
		uint32_t tlb[4][4] = {};            // as m_recompile_study_tlb, for this pc
		std::vector<recompile_study_variant> variants;
		std::vector<uint32_t> physpcs;
		std::unordered_set<uint32_t> requesters;
	};
	std::unordered_map<uint64_t, recompile_study_key> m_recompile_study_keys;
	std::unordered_map<uint32_t, recompile_study_pc> m_recompile_study_pcs;
	uint64_t m_recompile_study_totals[5][2] = {};
	void recompile_study_record(uint32_t key_mode, offs_t pc, const opcode_desc *desclist);
	void recompile_study_dump();
	void report_hash_stats(char const *when);  // MAME_SGI_HASH_TABLE_STATS_DIAG
	uint32_t m_recompile_study_serial = 0;
	bool m_drc_hash32 = false;                  // MAME_SGI_HASH32, see set_drc_hash32()
	uint32_t m_compile_entry_pc = 0;            // MAME_SGI_STALE_STUB_FIX: entry pc/key of the block being compiled
	uint32_t m_compile_entry_mode = 0;
	uint64_t m_stale_stub_container_recompiles = 0;
	uint32_t m_recompile_study_cur_serial = 0;
	uint64_t m_recompile_study_handler[4] = {};
	// TLB-class recompiles split by what sent them (index: 0 = validate_tlb
	// mismatch with a different physical page, 1 = validate_tlb mismatch with
	// only the flag bits different, 2 = stub at its own block head, 3 = stub
	// embedded in another block or unknown) and by outcome (0 = previous
	// compile was a stub at pc, 1 = window identical, 2 = own page identical
	// but the window differs, 3 = own page changed)
	uint64_t m_recompile_study_tlb[4][4] = {};
	std::map<uint32_t, uint64_t> m_recompile_study_flag_xor;    // validate mismatches by (baked ^ live) & 0xfff
	uint32_t m_recompile_study_last_kind = 3;
	uint32_t m_recompile_study_depth = 0;       // code_compile_block nesting
	uint32_t m_recompile_study_bucket = 6;      // time bucket of the current top-level request
	uint64_t m_recompile_study_ns[7] = {};
	uint64_t m_recompile_study_n[7] = {};

	struct recompile_study_log_entry { uint32_t pc; uint32_t req; uint64_t cycles; };
	std::vector<recompile_study_log_entry> m_recompile_study_log;

	// TEMPORARY diagnostic (2026-08-01): sanity check for the
	// dispatch_mode fix above - every KERNEL-privilege compile
	// (dispatch_mode < 4, i.e. asid-component bits clear) should produce
	// exactly one of only two possible values (0 or 1, differing only by
	// the FR bit) regardless of which ASID happens to be live at the
	// time. If this set ever grows past {0,1}, that's direct proof
	// kernel code is being incorrectly fragmented by ASID - the one
	// thing the fixed-sentinel design is specifically supposed to
	// prevent. Populated in code_compile_block(), dumped alongside
	// m_diag_usermode_recompile_counts in device_stop() (mips3.cpp).
	std::unordered_set<uint32_t> m_diag_kernel_dispatch_mode_values;

	// TEMPORARY diagnostic (2026-08-08): measures whether a 1-slot
	// per-call-site inline cache would actually hit at jmpdest (JR/JALR),
	// the hottest HASHJMP dispatch site - the deciding question before
	// implementing the self-patching/inline-caching redesign discussed
	// 2026-08-08 (targets HASHJMP's 3-dependent-load table walk,
	// shown to cost DRC 38.6x more cache-misses/instruction than the
	// interpreter on identical guest work). Keyed by desc->pc (this JR/JALR
	// instruction's own guest address - the call-site identity, stable
	// across re-executions of the same compiled instance). Each entry
	// tracks the last resolved {mode,pc} pair seen at that call site plus
	// hit/miss counts against that 1-slot model. JR $ra (function return)
	// is the specific risk: the same physical call site's target varies by
	// caller, the classic polymorphic case that thrashes a monomorphic
	// cache. Populated in func_callsite_ic_probe() (mips3drc.cpp), gated on
	// MAME_SGI_CALLSITE_IC_HITRATE_DIAG there; dumped alongside the other
	// diagnostic maps in mips3.cpp's device_stop().
	struct callsite_ic_entry
	{
		bool     seen = false;
		uint32_t last_mode = 0;
		uint32_t last_pc = 0;
		uint32_t hits = 0;
		uint32_t misses = 0;
	};
	std::unordered_map<uint32_t, callsite_ic_entry> m_diag_callsite_ic_state;

	// PROTOTYPE (2026-08-08): per-call-site guard-based inline cache for
	// the jmpdest HASHJMP site (targets the measured 38.6x cache-
	// miss/instruction gap between DRC and the interpreter, traced to
	// HASHJMP's 3 sequential dependent loads through large, sparse
	// global tables). Two full redesigns preceded this one: (1) a
	// per-call-site heap allocation, broken by this backend's mem()/
	// MABS() addressing requiring every operand within 2GB of rbp and
	// by the near-cache's tiny fixed 128KB budget; (2) a `UML_CALLH` +
	// runtime byte-patching scheme (convert a compiled `call` into a
	// `jmp` in place), which got as far as a working address-capture
	// mechanism but was then found to be UNSAFE - `op_hashjmp` resets
	// the host stack pointer before every jump (`mov rsp, [m_near.
	// stacksave]`), an invariant a raw patched jmp can't replicate
	// (5 bytes, no room, and the field is private to drcbe_x64 anyway)
	// - confirmed by a real guest-level `Memory fault(coredump)` on the
	// first proper boot test. That's what motivated OP_DIRECTJMP (uml.h/
	// drcbex64.cpp) - a new backend primitive that does the same rsp
	// reset HASHJMP does, then one indirect jump through a memory slot,
	// no table walk.
	//
	// Current (third) design, using OP_DIRECTJMP: a small, FIXED-SIZE
	// table of MAME_SGI_IC_SITE_COUNT slots, embedded directly in
	// internal_mips3_state as parallel arrays (ic_site_expected_mode/
	// ic_site_expected_pc/ic_site_target - not an array-of-structs, to
	// avoid a nested-struct forward reference to a type declared later
	// in this class), automatically near-cache-resident along with
	// m_core itself. A call site's own slot index is a cheap,
	// compile-time hash of its own desc->pc, mod MAME_SGI_IC_SITE_COUNT
	// - no allocation at compile time, and critically, NO per-instruction
	// address capture either: `&m_core->ic_site_target[idx]` is a fixed,
	// compile-time-known host address the moment m_core itself is
	// allocated (device_start()), so it can be passed directly as
	// OP_DIRECTJMP's PTR parameter with none of the "capture an
	// assembled instruction's address" timing problems the byte-patch
	// design fought. Different call sites CAN collide on the same slot;
	// this is safe, not a correctness bug - the guard always
	// re-validates live {mode,pc} against the slot's current expectation
	// before trusting the fast path, so a collision just means more
	// guard misses (degraded to the always-correct HASHJMP fallback)
	// for the colliding sites, never a wrong jump.
	//
	// expected_mode/expected_pc are reset to the sentinel UINT32_MAX (a
	// MIPS jmpdest value that can never legitimately occur) for every
	// slot at the start of every flush (code_flush_cache(), mips3drc.cpp)
	// - this also covers the very first, pre-flush setup, since
	// code_flush_cache() unconditionally runs once before any dynamic
	// compile (execute_run(), mips3.cpp). func_ic_learn_and_arm()
	// (mips3drc.cpp) commits expected_mode/expected_pc and ic_site_target
	// TOGETHER, in the same operation, and only once hash_exists()/
	// get_codeptr() have actually succeeded - guaranteeing "established"
	// (non-sentinel expected_pc) always implies a valid, resolved
	// ic_site_target, so a guard-hit can never reach OP_DIRECTJMP with
	// an unresolved (null) target. Never updated again until the next
	// flush resets them.
	void generate_exception(int exception, int backup);
	void generate_tlb_exception(int exception, offs_t address);

	void static_generate_memory_mode_checks(drcuml_block &block, uml::code_handle &exception_addrerr, int &label, int mode);
	void static_generate_fastram_accessor(drcuml_block &block, int &label, int size, bool iswrite, bool ismasked);
	void static_generate_memory_rw(drcuml_block &block, int size, bool iswrite, bool ismasked);
	virtual void static_generate_memory_accessor(drcuml_block &block, int &label, int mode, int size, bool iswrite, bool ismasked, const char *name, uml::code_handle *&handleptr);

	virtual void check_irqs();
	virtual void handle_mult(uint32_t op);
	virtual void handle_multu(uint32_t op);

public:
	void mips3com_update_cycle_counting();
	void mips3com_asid_changed();
	void mips3com_tlbr();
	void mips3com_tlbwi();
	void mips3com_tlbwr();
	void mips3com_tlbp();
	void fp_convert_fixup();
private:
	uint32_t compute_config_register();
	uint32_t compute_prid_register();
	uint32_t compute_fpu_prid_register();

	uint32_t generate_tlb_index();
	void tlb_map_entry(int tlbindex);
	void tlb_write_common(int tlbindex);

	uint64_t get_cop0_reg(int idx);
	uint64_t get_cop0_creg(int idx);
	void set_cop0_creg(int idx, uint64_t val);
	void handle_cop0(uint32_t op);

	uint32_t get_cop1_reg32(int idx);
	uint64_t get_cop1_reg64(int idx);
	void set_cop1_reg32(int idx, uint32_t val);
	void set_cop1_reg64(int idx, uint64_t val);
	uint64_t get_cop1_creg(int idx);
	void set_cop1_creg(int idx, uint64_t val);
	int64_t fp_convert(double v, int mode, bool wide);
	void handle_cop1_fr0(uint32_t op);
	void handle_cop1_fr1(uint32_t op);
	void handle_cop1x_fr0(uint32_t op);
	void handle_cop1x_fr1(uint32_t op);

	virtual uint64_t get_cop2_reg(int idx);
	virtual void set_cop2_reg(int idx, uint64_t val);
	virtual uint64_t get_cop2_creg(int idx);
	virtual void set_cop2_creg(int idx, uint64_t val);
	void handle_cop2(uint32_t op);

	void handle_special(uint32_t op);
	void handle_regimm(uint32_t op);
	virtual void handle_extra_base(uint32_t op);
	virtual void handle_extra_special(uint32_t op);
	virtual void handle_extra_regimm(uint32_t op);
	virtual void handle_extra_cop0(uint32_t op);
	virtual void handle_extra_cop1(uint32_t op);
	virtual void handle_extra_cop2(uint32_t op);
	virtual void handle_idt(uint32_t op);
	virtual void handle_ldc2(uint32_t op);
	virtual void handle_sdc2(uint32_t op);
	virtual void handle_dmfc2(uint32_t op);
	virtual void handle_dmtc2(uint32_t op);
	virtual void handle_cache(uint32_t op) { /* Handle as a no-op in most implementations */ }

	void lwl_be(uint32_t op);
	void lwr_be(uint32_t op);
	void ldl_be(uint32_t op);
	void ldr_be(uint32_t op);
	void swl_be(uint32_t op);
	void swr_be(uint32_t op);
	void sdl_be(uint32_t op);
	void sdr_be(uint32_t op);
	void lwl_le(uint32_t op);
	void lwr_le(uint32_t op);
	void ldl_le(uint32_t op);
	void ldr_le(uint32_t op);
	void swl_le(uint32_t op);
	void swr_le(uint32_t op);
	void sdl_le(uint32_t op);
	void sdr_le(uint32_t op);
	void load_fast_iregs(drcuml_block &block);
	void save_fast_iregs(drcuml_block &block);
	void code_flush_cache();
	void code_compile_block(uint32_t mode, offs_t pc);
public:
	void func_get_cycles();
	void func_callsite_ic_probe();
	void func_ic_learn_and_arm();
	void func_printf_exception();
	void func_lockstate_probe(int site);
	void func_mutex_wake_entry_probe();
	void func_install_mutex_tap();
	void func_mutex_unlock_entry_probe();
	void func_sv_wait_sig_unlock_probe();
	void func_sv_wait_sig_worker_entry_probe();
	void func_sv_timedwait_entry_probe();
	void func_sv_wait_entry_probe();
	void func_xlog_state_clean_log_entry_probe();
	void func_biodone_dispatch_probe();
	void func_xlog_iodone_entry_probe();
	void func_xlog_complete_entry_probe();
	void func_xlog_inner_worker_stack_tap();
	void func_xlog_tlb_recheck_probe();
	void func_xfs_log_done_entry_probe();
	void func_xlog_commit_record_entry_probe();
	void func_xlog_write_call_capture_probe();
	void func_xlog_write_return_check_probe();
	void func_xfs_trans_commit_iclog_consume_probe();
	void func_xlog_write_entry_log_probe();
	void func_mutex_wake_dequeue_probe();
	void recompile_study_validate_probe();
	bool recompile_study_traced(uint32_t pc, bool near);
	void recompile_study_tlb_handler_probe();
	void syscall_log_diag();                    // MAME_SGI_SYSCALL_LOG_DIAG
	void func_iclogspace_splhi_probe();
	void func_iclogspace_splx_probe();
	void func_interrupt_taken_probe();
	void func_mutex_lock_enqueue_probe();
	void func_mutex_lock_enqueue_end_probe();
	void func_us_delay_entry_probe();
	void func_mrupdatef_entry_probe();
	void func_rmap_del_panic_probe(int site);
	void func_tlbmiss_entry_probe();
	void func_fault_store_mode_probe(int site);
	void func_interrupt_delivery_probe();
	void func_status_store_probe();
	void func_premature_interrupt_probe();
	void func_pckm_mutex_lock_call_probe();
	void func_pckm_mutex_snapshot_probe();
	void func_andi_ori_probe(int site);
	void func_mtcz_read_probe(int compiletime_is_branch_target);
	void func_sd_value_probe();
	void func_printf_debug();
	void func_0fac5ffc_anomaly_diag();
	void func_validate_tlb_mismatch_class_diag();
	void func_mxcsr_desync_diag();
	void func_ctc1_mxcsr_check_diag();
	void func_fpop_mxcsr_check_diag();
	void func_nocode_0fac5ffc_diag();
	void func_rodata_load_addr_diag();
	void func_fastram_compare_diag();
	void func_printf_probe();
	void func_debug_break();
	void func_unimplemented();
	void func_checksum_mismatch_diag(void *ctx);
	void func_task84_tlb_mismatch_diag();
	void func_task84_v0_diag();
	void func_task84_s3_check_diag();
	void func_task84_window_trace_save();
	void func_task84_window_trace_reload();
	void func_task84_entry_pc_diag();
	void func_v0_shadow_diag();
	void func_t9_got_diag();
	void func_call_trace_diag();
	void func_cpperror_callsite_diag();
	void func_cpperror_cond_trace_diag();
	void func_source_text_diag();
	void func_beql_s3_diag();
	void func_breakcheck_s3_diag();
	void func_s3_increment_diag();
	void func_token_result_diag();
	void func_pid_calibration_diag();
	void log_drc_event(const char *type, uint32_t pc);
	void log_drc_exception();
	void log_tlb_mismatch_entry();
	void func_tlbmiss_redispatch_diag();
	void func_tlbmiss_redispatch_e020_diag();
	uint32_t read_guest_dword_translated(offs_t addr);
	uint32_t read_current_pid();
	void func_task84_t9_once_diag();
	void func_task84_reload_mismatch();
	void func_task84_entry_irq_diag();
	void func_task84_saveload_bracket_entry();
	void func_task84_saveload_bracket_exit();
	void func_task101_entry_reentry_diag();
	void func_task101_out_of_cycles_diag();
private:
	/* internal compiler state */
	struct compiler_state
	{
		compiler_state &operator=(compiler_state &) = delete;

		uint32_t         cycles;                     /* accumulated cycles */
		uint8_t          checkints;                  /* need to check interrupts before next instruction */
		uint8_t          checksoftints;              /* need to check software interrupts before next instruction */
		uml::code_label  labelnum;                   /* index for local labels */
	};

	void static_generate_entry_point(drcuml_block &block, int &label);
	void static_generate_nocode_handler(drcuml_block &block, int &label);
	void static_generate_out_of_cycles(drcuml_block &block, int &label);
	void static_generate_tlb_mismatch(drcuml_block &block, int &label);
	void static_generate_exception(drcuml_block &block, int &label, uint8_t exception, int recover, const char *name);

	void generate_update_mode(drcuml_block &block);
	// MAME_SGI_DISPATCH_MODE_ASID_FIX value 2 (mips3drc.cpp): host-side
	// (compile-time) writability check for a statically-known target PC
	// - IRIX's own kernel uses exactly this signal (execmap(), $IRIXSRC)
	// to decide MAP_SHARED (read-only) vs MAP_PRIVATE (writable), so it's
	// used the same way here to decide whether a dispatch needs ASID
	// folded into its key at all. Cheap: a single VTLB table lookup, no
	// TLB miss handling - matches the live-VTLB-check idiom already used
	// in static_generate_tlb_mismatch() for the same table.
	//
	// REVERTED (2026-08-07): tried restricting FETCH_ALLOWED-awareness to
	// the delay-slot page only, reasoning that pc's own page is always
	// pre-verified fetchable. That reasoning only holds for
	// code_compile_block()'s own `pc` (entered solely via
	// EXECUTE_MISSING_CODE, i.e. a HASHJMP just proved pc fetchable by
	// attempting it) - it does NOT hold for static_dispatch_key()/
	// live_dispatch_key()/generate_select_dispatch_key(), all of which
	// evaluate a *future, not-yet-attempted* target's page, with nothing
	// guaranteeing it's currently mapped. Reintroduced the exact same
	// failure class this whole investigation is about, just on the
	// target's own page instead of the delay page - a genuinely-unmapped
	// own page (indistinguishable from a legitimately-shared one under
	// WRITE_ALLOWED alone) got misclassified as shareable, poisoning the
	// shared key with a degenerate page-fault stub. Reproduced a real
	// livelock immediately (2026-08-06/07) - reverted to
	// FETCH_ALLOWED-aware for both pages, the proven-safe version.
	// page_shareable() used for both pc and pc+4 in dispatch_page_writable()
	// below.
	bool page_shareable(offs_t p) const noexcept
	{
		vtlb_entry const entry = vtlb_table()[p >> 12];
		return (entry & (FETCH_ALLOWED | WRITE_ALLOWED)) == FETCH_ALLOWED;
	}
	// dispatch_page_writable(pc) itself checks pc's own page AND pc+4
	// (2026-08-06 "Fix 1"): a compiled block also depends on its delay
	// slot's page if pc is a branch/jump - always exactly pc+4 in the
	// MIPS ISA, so this is a pure function of pc alone, safe to apply
	// unconditionally even when pc isn't actually a branch (pc+4 is then
	// just the next sequential instruction, itself a legitimate
	// dependency of nothing in particular, but checking it anyway costs
	// nothing and is always correct to require). This one function is
	// the single choke point every registration/lookup site in this file
	// funnels through (directly, or via static_dispatch_key()/
	// live_dispatch_key() below). Two earlier, narrower attempts
	// (2026-08-06) instead special-cased the fix at the registration
	// site only, causing a registration/lookup key mismatch and an infinite
	// EXECUTE_MISSING_CODE/recompile livelock. generate_select_
	// dispatch_key() (mips3drc.cpp) is the one call site that can't
	// route through this host-side function (it needs the equivalent
	// check emitted as JIT'd runtime code for targets not known until
	// the guest is actually running) - kept in lockstep with this
	// definition by hand; if this function's logic changes, that one
	// must change with it.
	// A/B toggle (2026-08-06, reverted to off 2026-08-07) - undefined to
	// run dispatch_page_writable() (and generate_select_dispatch_key()'s
	// matching JIT'd codegen, mips3drc.cpp) on the original single-page,
	// WRITE_ALLOWED-only check (pre-2026-08-06, the 2026-08-02
	// baseline config: 13-15 flushes for a full boot). The two-page,
	// FETCH_ALLOWED-aware check is the *more correct* one (closes a real
	// delay-slot race, 2026-08-06), but 2026-08-07 00:14-00:52
	// measurements showed that FETCH_ALLOWED-awareness itself - not page
	// count - is what's expensive: a single-page variant that kept
	// FETCH_ALLOWED (`ab_singlepage2`, still calling page_shareable())
	// was just as slow (415s) as the two-page version (424s) at the
	// `/.oldtmp` milestone, both ~4x over the ~93-105s baseline. Only
	// dropping FETCH_ALLOWED entirely - not disabling ASID_FIX==2 itself,
	// a separate, older fix for cross-process DRC cache thrashing, kept
	// on either way - restores the fast pacing. The delay-slot race this
	// trades back in has never been observed to cause anything worse than
	// an extra, self-correcting recompile (a stale stub throws EXH, a
	// fresh compile follows) - never a crash or wrong result - so until
	// the epoch/generation-counter redesign (2026-08-07, "Fix 3")
	// lands, paying the smaller known cost is the better trade.
	// Re-define to switch back to the FETCH_ALLOWED-aware, two-page,
	// slower-but-more-correct behavior.
// #define MAME_SGI_DISPATCH_TWO_PAGE_CHECK
	bool dispatch_page_writable(offs_t pc) const noexcept
	{
#ifdef MAME_SGI_DISPATCH_TWO_PAGE_CHECK
		return !page_shareable(pc) || !page_shareable(pc + 4);
#else
		vtlb_entry const entry = vtlb_table()[pc >> 12];
		return (entry & WRITE_ALLOWED) != 0;
#endif
	}
	// value 2: host-side (compile-time) choice of which key to BAKE as
	// an immediate, for the sites that already bake m_core->dispatch_mode
	// directly (proven safe by seqlast->can_change_modes() analysis at
	// that specific call site - not something this fix changes).
	uint32_t static_dispatch_key(offs_t target_pc) const noexcept
	{
		return dispatch_page_writable(target_pc) ? m_core->dispatch_mode : m_core->mode;
	}
	// value 2: host-side choice of *which field* to read live, for a
	// statically-known target - the writability check is baked (a target
	// PC's page identity doesn't change compile to compile), but the key
	// VALUE itself still has to be read live (mem()), since an ASID-scoped
	// key depends on whichever process is executing when this dispatch
	// actually runs, not what was current at compile time.
	uml::parameter live_dispatch_key(offs_t target_pc) const
	{
		return dispatch_page_writable(target_pc) ? uml::mem(&m_core->dispatch_mode) : uml::mem(&m_core->mode);
	}
	void generate_select_dispatch_key(drcuml_block &block, uml::parameter target_pc, uml::parameter dest, int &labelnum);
private:
	void generate_update_cycles(drcuml_block &block, compiler_state &compiler, uml::parameter param, bool allow_exception, bool diag_tag = false, uint32_t diag_desc_pc = 0);
	void generate_checksum_block(drcuml_block &block, compiler_state &compiler, const opcode_desc *seqhead, const opcode_desc *seqlast, const opcode_desc *codelast);
	void generate_sequence_instruction(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void generate_delay_slot_and_branch(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, uint8_t linkreg);
	void generate_likely_branch_skip_fixup(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void generate_fpop_mxcsr_precheck(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);

	bool generate_opcode(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool generate_special(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool generate_regimm(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool generate_idt(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);

	bool generate_set_cop0_reg(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, uint8_t reg);
	bool generate_get_cop0_reg(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc, uint8_t reg, bool is64 = false);
	bool generate_cop0(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	void generate_fp_convert_save(drcuml_block &block, uint32_t op);
	void generate_fp_convert_fixup(drcuml_block &block, compiler_state &compiler, uint32_t op, bool wide);
	bool generate_cop1(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);
	bool generate_cop1x(drcuml_block &block, compiler_state &compiler, const opcode_desc *desc);

	void check_cop0_access(drcuml_block &block);
	void check_cop1_access(drcuml_block &block);
	void generate_badcop(drcuml_block &block, const int cop);

	void log_add_disasm_comment(drcuml_block &block, uint32_t pc, uint32_t op);
	const char *log_desc_flags_to_string(opcode_desc const &desc);
	void log_register_list(const char *string, const std::bitset<68> &reglist, const std::bitset<68> *regnostarlist);
	void log_opcode_desc(const opcode_desc *desclist, int indent);

	void load_elf();
	[[maybe_unused]] void do_o2_dprintf(uint32_t fmt_addr, uint32_t a1, uint32_t a2, uint32_t a3, uint32_t stack);
};


class r4000be_device : public mips3_device {
public:
	// construction/destruction
	r4000be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4000BE, tag, owner, clock, MIPS3_TYPE_R4000, ENDIANNESS_BIG, 64)
	{
	}
};

class r4000le_device : public mips3_device {
public:
	// construction/destruction
	r4000le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4000LE, tag, owner, clock, MIPS3_TYPE_R4000, ENDIANNESS_LITTLE, 64)
	{
	}
};

class r4400be_device : public mips3_device {
public:
	// construction/destruction
	r4400be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4400BE, tag, owner, clock, MIPS3_TYPE_R4400, ENDIANNESS_BIG, 64)
	{
	}
};

class r4400le_device : public mips3_device {
public:
	// construction/destruction
	r4400le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4400LE, tag, owner, clock, MIPS3_TYPE_R4400, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};

class vr4300be_device : public mips3_device {
public:
	// construction/destruction
	vr4300be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, VR4300BE, tag, owner, clock, MIPS3_TYPE_VR4300, ENDIANNESS_BIG, 32)
	{
	}
};

class vr4300le_device : public mips3_device {
public:
	// construction/destruction
	vr4300le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, VR4300LE, tag, owner, clock, MIPS3_TYPE_VR4300, ENDIANNESS_LITTLE, 32)
	{
	}
};

class vr4310be_device : public mips3_device {
public:
	// construction/destruction
	vr4310be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, VR4310BE, tag, owner, clock, MIPS3_TYPE_VR4300, ENDIANNESS_BIG, 32)
	{
	}
};

class vr4310le_device : public mips3_device {
public:
	// construction/destruction
	vr4310le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, VR4310LE, tag, owner, clock, MIPS3_TYPE_VR4300, ENDIANNESS_LITTLE, 32)
	{
	}
};

class r4600be_device : public mips3_device {
public:
	// construction/destruction
	r4600be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4600BE, tag, owner, clock, MIPS3_TYPE_R4600, ENDIANNESS_BIG, 64)
	{
	}
};

class r4600le_device : public mips3_device {
public:
	// construction/destruction
	r4600le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4600LE, tag, owner, clock, MIPS3_TYPE_R4600, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};

class r4650_device : public mips3_device {
public:
	// construction/destruction
	r4650_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, endianness_t endianness)
		: mips3_device(mconfig, type, tag, owner, clock, MIPS3_TYPE_R4650, endianness, 32) // Should be 64 bits
	{
	}

protected:
	virtual bool memory_translate(int spacenum, int intention, offs_t &address, address_space *&target_space) override;

	virtual void static_generate_memory_accessor(drcuml_block &block, int &label, int mode, int size, bool iswrite, bool ismasked, const char *name, uml::code_handle *&handleptr) override;

	virtual bool RBYTE(offs_t address, uint32_t *result) override;
	virtual bool RHALF(offs_t address, uint32_t *result) override;
	virtual bool RWORD(offs_t address, uint32_t *result, bool insn = false) override;
	virtual bool RWORD_MASKED(offs_t address, uint32_t *result, uint32_t mem_mask) override;
	virtual bool RDOUBLE(offs_t address, uint64_t *result) override;
	virtual bool RDOUBLE_MASKED(offs_t address, uint64_t *result, uint64_t mem_mask) override;
	virtual void WBYTE(offs_t address, uint8_t data) override;
	virtual void WHALF(offs_t address, uint16_t data) override;
	virtual void WWORD(offs_t address, uint32_t data) override;
	virtual void WWORD_MASKED(offs_t address, uint32_t data, uint32_t mem_mask) override;
	virtual void WDOUBLE(offs_t address, uint64_t data) override;
	virtual void WDOUBLE_MASKED(offs_t address, uint64_t data, uint64_t mem_mask) override;

	virtual void set_cop0_reg(int idx, uint64_t val) override;
};

class r4650be_device : public r4650_device {
public:
	// construction/destruction
	r4650be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: r4650_device(mconfig, R4650BE, tag, owner, clock, ENDIANNESS_BIG)
	{
	}
};

class r4650le_device : public r4650_device {
public:
	// construction/destruction
	r4650le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: r4650_device(mconfig, R4650LE, tag, owner, clock, ENDIANNESS_LITTLE)
	{
	}
};

class r4700be_device : public mips3_device {
public:
	// construction/destruction
	r4700be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4700BE, tag, owner, clock, MIPS3_TYPE_R4700, ENDIANNESS_BIG, 32) // Should be 64 bits
	{
	}
};

class r4700le_device : public mips3_device {
public:
	// construction/destruction
	r4700le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R4700LE, tag, owner, clock, MIPS3_TYPE_R4700, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};

class tx4925be_device : public mips3_device {
public:
	// construction/destruction
	tx4925be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, TX4925BE, tag, owner, clock, MIPS3_TYPE_TX4925, ENDIANNESS_BIG, 32) // Should be 64 bits
	{
	}
};

class tx4925le_device : public mips3_device {
public:
	// construction/destruction
	tx4925le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, TX4925LE, tag, owner, clock, MIPS3_TYPE_TX4925, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};

class r5000be_device : public mips3_device {
public:
	// construction/destruction
	r5000be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R5000BE, tag, owner, clock, MIPS3_TYPE_R5000, ENDIANNESS_BIG, 64)
	{
	}

protected:
	void handle_cache(uint32_t op) override;
};

class r5000le_device : public mips3_device {
public:
	// construction/destruction
	r5000le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, R5000LE, tag, owner, clock, MIPS3_TYPE_R5000, ENDIANNESS_LITTLE, 32) // FIXME: Should be 64 bits, Galileo blows up though
	{
	}
};

class vr5500be_device : public mips3_device {
public:
	// construction/destruction
	vr5500be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, VR5500BE, tag, owner, clock, MIPS3_TYPE_R5000, ENDIANNESS_BIG, 32) // Should be 64 bits
	{
	}
};

class vr5500le_device : public mips3_device {
public:
	// construction/destruction
	vr5500le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, VR5500LE, tag, owner, clock, MIPS3_TYPE_R5000, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};

class r5900_device : public mips3_device {
protected:
	// construction/destruction
	r5900_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock, endianness_t endianness)
		: mips3_device(mconfig, type, tag, owner, clock, MIPS3_TYPE_R5900, endianness, 64)
		, m_vu0(*this, finder_base::DUMMY_TAG)
	{
	}

	virtual void device_start() override ATTR_COLD;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	bool RBYTE(offs_t address, uint32_t *result) override;
	bool RHALF(offs_t address, uint32_t *result) override;
	bool RWORD(offs_t address, uint32_t *result, bool insn = false) override;
	bool RWORD_MASKED(offs_t address, uint32_t *result, uint32_t mem_mask) override;
	bool RDOUBLE(offs_t address, uint64_t *result) override;
	bool RDOUBLE_MASKED(offs_t address, uint64_t *result, uint64_t mem_mask) override;
	void WBYTE(offs_t address, uint8_t data) override;
	void WHALF(offs_t address, uint16_t data) override;
	void WWORD(offs_t address, uint32_t data) override;
	void WWORD_MASKED(offs_t address, uint32_t data, uint32_t mem_mask) override;
	void WDOUBLE(offs_t address, uint64_t data) override;
	void WDOUBLE_MASKED(offs_t address, uint64_t data, uint64_t mem_mask) override;

	virtual bool RQUAD(offs_t address, uint64_t *result_hi, uint64_t *result_lo) = 0;
	virtual void WQUAD(offs_t address, uint64_t data_hi, uint64_t data_lo) = 0;

	uint64_t get_cop2_reg(int idx) override;
	void set_cop2_reg(int idx, uint64_t val) override;
	uint64_t get_cop2_creg(int idx) override;
	void set_cop2_creg(int idx, uint64_t val) override;

	void handle_extra_base(uint32_t op) override;
	void handle_extra_special(uint32_t op) override;
	void handle_extra_regimm(uint32_t op) override;
	void handle_extra_cop0(uint32_t op) override;
	void handle_extra_cop1(uint32_t op) override;
	void handle_extra_cop2(uint32_t op) override;
	void handle_idt(uint32_t op) override;
	void handle_mult(uint32_t op) override;
	void handle_multu(uint32_t op) override;
	void handle_mmi0(uint32_t op);
	void handle_mmi1(uint32_t op);
	void handle_mmi2(uint32_t op);
	void handle_mmi3(uint32_t op);
	void handle_ldc2(uint32_t op) override;
	void handle_sdc2(uint32_t op) override;
	void handle_dmfc2(uint32_t op) override;
	void handle_dmtc2(uint32_t op) override;

	void check_irqs() override;

	required_device<sonyvu0_device> m_vu0;
};

class r5900le_device : public r5900_device {
public:
	// construction/destruction
	template <typename T>
	r5900le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, T &&vu0_tag)
		: r5900le_device(mconfig, tag, owner, clock)
	{
		m_vu0.set_tag(std::forward<T>(vu0_tag));
	}

	r5900le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: r5900_device(mconfig, R5900LE, tag, owner, clock, ENDIANNESS_LITTLE)
	{
	}

protected:
	bool RQUAD(offs_t address, uint64_t *result_hi, uint64_t *result_lo) override;
	void WQUAD(offs_t address, uint64_t data_hi, uint64_t data_lo) override;
};

class r5900be_device : public r5900_device {
public:
	// construction/destruction
	template <typename T>
	r5900be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, T &&vu0_tag)
		: r5900be_device(mconfig, tag, owner, clock)
	{
		m_vu0.set_tag(std::forward<T>(vu0_tag));
	}

	r5900be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: r5900_device(mconfig, R5900BE, tag, owner, clock, ENDIANNESS_BIG)
	{
	}

protected:
	bool RQUAD(offs_t address, uint64_t *result_hi, uint64_t *result_lo) override;
	void WQUAD(offs_t address, uint64_t data_hi, uint64_t data_lo) override;
};

class qed5271be_device : public mips3_device {
public:
	// construction/destruction
	qed5271be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, QED5271BE, tag, owner, clock, MIPS3_TYPE_QED5271, ENDIANNESS_BIG, 32) // Should be 64 bits
	{
	}
};

class qed5271le_device : public mips3_device {
public:
	// construction/destruction
	qed5271le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, QED5271LE, tag, owner, clock, MIPS3_TYPE_QED5271, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};

class rm7000be_device : public mips3_device {
public:
	// construction/destruction
	rm7000be_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, RM7000BE, tag, owner, clock, MIPS3_TYPE_RM7000, ENDIANNESS_BIG, 32) // Should be 64 bits
	{
	}
};

class rm7000le_device : public mips3_device {
public:
	// construction/destruction
	rm7000le_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
		: mips3_device(mconfig, RM7000LE, tag, owner, clock, MIPS3_TYPE_RM7000, ENDIANNESS_LITTLE, 32) // Should be 64 bits
	{
	}
};


/***************************************************************************
COMPILER-SPECIFIC OPTIONS
***************************************************************************/

#define MIPS3DRC_STRICT_VERIFY      0x0001          /* verify all instructions */
#define MIPS3DRC_STRICT_COP0        0x0002          /* validate all COP0 instructions */
#define MIPS3DRC_STRICT_COP1        0x0004          /* validate all COP1 instructions */
#define MIPS3DRC_STRICT_COP2        0x0008          /* validate all COP2 instructions */
#define MIPS3DRC_DISABLE_INTRABLOCK 0x0010          /* disable intrablock branching */
#define MIPS3DRC_CHECK_OVERFLOWS    0x0020          /* actually check overflows on add/sub instructions */
#define MIPS3DRC_ACCURATE_DIVZERO   0x0040          /* load correct values into HI/LO on integer divide-by-zero */
#define MIPS3DRC_EXTRA_INSTR_CHECK  0x0080          /* adds the last instruction value to all validation entry locations, used with STRICT_VERIFY */

#define MIPS3DRC_COMPATIBLE_OPTIONS (MIPS3DRC_STRICT_VERIFY | MIPS3DRC_STRICT_COP1 | MIPS3DRC_STRICT_COP0 | MIPS3DRC_STRICT_COP2)
#define MIPS3DRC_FASTEST_OPTIONS    (0)

#endif // MAME_CPU_MIPS_MIPS3_H
