// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    mips3com.cpp

    Common MIPS III/IV definitions and functions

***************************************************************************/

#include "emu.h"
#include "mips3com.h"
#include "ps2vu.h"

#include <cstdio>
#include <iostream>

// dispatch_page_writable() (mips3.h) hardcodes this shift as a literal
// since MIPS3_MIN_PAGE_SHIFT isn't visible from that header's own
// include order - catch drift here, where both are visible.
static_assert(MIPS3_MIN_PAGE_SHIFT == 12, "dispatch_page_writable()'s hardcoded shift in mips3.h must match");

// DISABLED (2026-08-21) - concluded, task #36 (measured ASID-switch vs
// TLBWI/TLBWR mutation frequency) completed weeks ago; was left
// enabled unconditionally since, firing fprintf(stderr) regularly with
// no active investigation using it. TEMPORARY diagnostic toggle
// (2026-08-07) - real frequency split between ASID-switch-triggered
// and TLBWI/TLBWR-triggered vtlb_table() mutations, see
// diag_asid_changed_events'/diag_tlbwi_tlbwr_events' comments in
// mips3.h. Re-enable if that question needs revisiting.
// #define MAME_SGI_VTLB_MUTATION_FREQ_DIAG

// DISABLED (2026-08-21) - concluded alongside the above, same reason.
// TEMPORARY diagnostic toggle (2026-08-07) - dedup potential for
// tlb_map_entry()'s vtlb_load() writes, see diag_tlb_write_total's
// comment in mips3.h.
// #define MAME_SGI_TLB_WRITE_DEDUP_DIAG

// TEMPORARY diagnostic (2026-09-01): task #84's gcc <built-in>
// "predicate's answer is empty" flake. Earlier work on this task ruled
// out on-disk/filesystem corruption (cmp/md5sum byte-identical) and
// showed a plain buffered read of cc1 is always clean immediately after
// an exec()-triggered failure (100% match, 0 mismatches over 22 real
// failures) - meaning whatever's wrong is specific to how a fresh
// executable/rodata page gets mapped into a process's address space,
// not a generic "file content is wrong" problem. This targets that
// mapping moment directly: tlb_map_entry() (called identically by both
// mips3com_tlbwi() and mips3com_tlbwr(), which are in turn called
// identically by the DRC-compiled path via cfunc_mips3com_tlbwi/wr and
// the plain interpreter - this is the ONE place TLB entries actually
// get installed into vtlb_table(), genuinely shared code, not
// DRC-specific) is where a virtual page gets bound to a physical frame
// number. If IRIX's kernel installs a TLB mapping for cc1's .rodata
// page pointing at a physical frame whose SCSI/HPC3 DMA transfer hasn't
// actually finished landing the real bytes yet - a race between the
// guest kernel's own page-in bookkeeping and MAME's own DMA-completion
// emulation - that would explain everything reproduced so far: only
// shows up around fresh page-in, self-heals (a later remap eventually
// wins the race), and would apply equally whether the code executing at
// the time is DRC-compiled or interpreted (since this specific
// mapping-installation code is identical either way) - the DRC-vs-
// interpreter split would then come from DRC's different execution
// *speed* changing when in real time the guest reaches this moment
// relative to the DMA controller's own event schedule, not from any
// bug in DRC's own logic. Watches specifically for TLB installs
// covering the guest virtual page containing the known failing string
// (0x10343970, established via static analysis of the extracted cc1
// binary - see wiki/console.md's task #84 section), and immediately
// reads the physical memory the new mapping points at, to see directly
// whether the promised bytes are actually there yet at install time.
// RESULT (2026-09-02): ran against a matched-load compile loop that
// reproduced a 73-consecutive-failure stuck streak - 1,579 TLB installs
// observed for this exact page, ALL with correct, non-corrupted content
// (including throughout the entire stuck streak), zero
// ALL_ZERO_SUSPICIOUS hits. Physical memory is provably fine at every
// TLB-install moment; this rules out the DMA-completion-race theory as
// framed here. See wiki/console.md's task #84 section for the pivot
// this motivated (checking whether DRC's own sequence-window compiler
// ever misdecodes into this same page, in mips3drc.cpp - a different
// mechanism that wouldn't touch this page's stored bytes at all).
// #define MAME_SGI_CC1_RODATA_TLB_RACE_DIAG


void mips3_device::execute_set_input(int inputnum, int state)
{
	if (inputnum >= MIPS3_IRQ0 && inputnum <= MIPS3_IRQ5)
	{
		if (state != CLEAR_LINE)
			m_core->cpr[0][COP0_Cause] |= 0x400 << inputnum;
		else
			m_core->cpr[0][COP0_Cause] &= ~(0x400 << inputnum);
	}
}


/***************************************************************************
    INITIALIZATION AND SHUTDOWN
***************************************************************************/

/*-------------------------------------------------
    mips3com_update_cycle_counting - update cycle
    counts and the timers
-------------------------------------------------*/

void mips3_device::mips3com_update_cycle_counting()
{
	/* modify the timer to go off */
	if (m_core->compare_armed)
	{
		uint32_t count = (total_cycles() - m_core->count_zero_time) / 2;
		uint32_t compare = m_core->cpr[0][COP0_Compare];
		uint32_t delta = compare - count;
		m_core->compare_armed = 0;
		attotime newtime = cycles_to_attotime((uint64_t)delta * 2);
		m_compare_int_timer->adjust(newtime);
		return;
	}
}

/***************************************************************************
    TLB HANDLING
***************************************************************************/

/*-------------------------------------------------
    mips3com_asid_changed - remap all non-global
    TLB entries
-------------------------------------------------*/

void mips3_device::mips3com_asid_changed()
{
	int tlbindex;

#ifdef MAME_SGI_VTLB_MUTATION_FREQ_DIAG
	// TEMPORARY diagnostic (2026-08-07) - see diag_asid_changed_events'
	// comment in mips3.h. One event per real ASID switch (already gated
	// by the caller on an actual EntryHi ASID-field change), regardless
	// of how many individual TLB entries this loop ends up remapping.
	{
		static uint32_t s_last_reported = 0;
		uint32_t const total = ++m_core->diag_asid_changed_events;
		if (s_last_reported == 0 || total >= s_last_reported + 1000)
		{
			fprintf(stderr, "[VTLB_MUTATION_FREQ] asid_changed_events=%u tlbwi_tlbwr_events=%u\n",
				total, m_core->diag_tlbwi_tlbwr_events);
			fflush(stderr);
			s_last_reported = total;
		}
	}
#endif

	/* iterate over all non-global TLB entries and remap them */
	for (tlbindex = 0; tlbindex < m_tlbentries; tlbindex++)
		if (!m_tlb[tlbindex].is_global())
			tlb_map_entry(tlbindex);
}


/*-------------------------------------------------
    mips3com_tlbr - execute the tlbr instruction
-------------------------------------------------*/

void mips3_device::mips3com_tlbr()
{
	uint32_t tlbindex = m_core->cpr[0][COP0_Index] & 0x3f;

	/* only handle entries within the TLB */
	if (tlbindex < m_tlbentries)
	{
		const mips3_tlb_entry &entry = m_tlb[tlbindex];

		/* copy data from the TLB entry into the COP0 registers */
		m_core->cpr[0][COP0_PageMask] = entry.page_mask;
		m_core->cpr[0][COP0_EntryHi] = entry.entry_hi;
		m_core->cpr[0][COP0_EntryLo0] = entry.entry_lo[0];
		m_core->cpr[0][COP0_EntryLo1] = entry.entry_lo[1];
	}
}


/*-------------------------------------------------
    mips3com_tlbwi - execute the tlbwi instruction
-------------------------------------------------*/

void mips3_device::mips3com_tlbwi()
{
	/* use the common handler and write based off the COP0 Index register */
	tlb_write_common(m_core->cpr[0][COP0_Index] & 0x3f);
}


/*-------------------------------------------------
generate_tlb_index - generate a random tlb index
-------------------------------------------------*/

uint32_t mips3_device::generate_tlb_index()
{
	// Actual hardware uses a free running counter to generate the index.
	// This implementation uses a linear congruential generator so that DRC and non-DRC code sequences match.
	m_tlb_seed = 214013 * m_tlb_seed + 2531011;
	return (m_tlb_seed >> 16) & 0x3f;
}

/*-------------------------------------------------
    mips3com_tlbwr - execute the tlbwr instruction
-------------------------------------------------*/

void mips3_device::mips3com_tlbwr()
{
	uint32_t wired = m_core->cpr[0][COP0_Wired] & 0x3f;
	uint32_t unwired = m_tlbentries - wired;
	uint32_t tlbindex = m_tlbentries - 1;

	/* "random" is based off of linear congruential sequence through the non-wired pages */
	if (unwired > 0)
		tlbindex = (generate_tlb_index() % unwired) + wired;

	/* use the common handler to write to this tlbindex */
	tlb_write_common(tlbindex);
}


/*-------------------------------------------------
    mips3com_tlbp - execute the tlbp instruction
-------------------------------------------------*/

void mips3_device::mips3com_tlbp()
{
	uint32_t tlbindex;

	/* iterate over TLB entries */
	for (tlbindex = 0; tlbindex < m_tlbentries; tlbindex++)
	{
		const mips3_tlb_entry &entry = m_tlb[tlbindex];
		const uint64_t mask = ~((entry.page_mask >> 13) & 0xfff) << 13;

		/* if the relevant bits of EntryHi match the relevant bits of the TLB */
		if ((entry.entry_hi & mask) == (m_core->cpr[0][COP0_EntryHi] & mask))

			/* and if we are either global or matching the current ASID, then stop */
			if ((entry.entry_hi & 0xff) == (m_core->cpr[0][COP0_EntryHi] & 0xff) || ((entry.entry_lo[0] & entry.entry_lo[1]) & TLB_GLOBAL))
				break;
	}

	/* validate that our tlb_table was in sync */
//  vpn = ((m_cores->cpr[0][COP0_EntryHi] >> 13) & 0x07ffffff) << 1;
	if (tlbindex != m_tlbentries)
		m_core->cpr[0][COP0_Index] = tlbindex;
	else
		m_core->cpr[0][COP0_Index] = 0x80000000;
}



/***************************************************************************
    INTERNAL HELPERS
***************************************************************************/

/*-------------------------------------------------
    compare_int_callback - callback that fires
    whenever a compare interrupt is generated
-------------------------------------------------*/

TIMER_CALLBACK_MEMBER( mips3_device::compare_int_callback )
{
	m_compare_int_timer->adjust(attotime::never);
	set_input_line(MIPS3_IRQ5, ASSERT_LINE);
}


/*-------------------------------------------------
    compute_config_register - compute the value
    of the config register
-------------------------------------------------*/

uint32_t mips3_device::compute_config_register()
{
	/* set the cache line size to 32 bytes */
	uint32_t configreg = 0x00026030;

	m_dcache = nullptr;
	m_icache = nullptr;

	// NEC VR series does not use a 100% compatible COP0/TLB implementation
	if (m_flavor == MIPS3_TYPE_VR4300)
	{
		/*
		    For VR43xx, Config is as follows:
		    bit 31 = always 0
		    bits 28-30 = EC
		    bits 24-27 = EP
		    bits 16-23 = always b0000010
		    bit 15 = endian indicator as standard MIPS III
		    bits 4-14 = always b11001000110
		    bit 3 = CU
		    bits 0-2 = K0 ("Coherency algorithm of kseg0")
		*/

		configreg = 0x6460;
	}
	else if (m_flavor == MIPS3_TYPE_VR5500)
	{
		/*
		    For VR55xx, Config is as follows:
		    bit 31 = always 0
		    bits 28-30 = EC
		    bits 24-27 = EP
		    bits 23-22 = EM
		    bits 21-20 = always b11
		    bits 19-18 = EW
		    bits 17-16 = always b10
		    bit 15 = endian indicator as standard MIPS III
		    bits 3-14 = always b110011011110
		    bits 0-2 = K0 ("Coherency algorithm of kseg0")
		*/

		configreg = 0x6460;
	}
	else
	{
		/* set the data cache size */
				if (c_dcache_size <= 0x01000) configreg |= 0 << 6;
		else if (c_dcache_size <= 0x02000) configreg |= 1 << 6;
		else if (c_dcache_size <= 0x04000) configreg |= 2 << 6;
		else if (c_dcache_size <= 0x08000) configreg |= 3 << 6;
		else if (c_dcache_size <= 0x10000) configreg |= 4 << 6;
		else if (c_dcache_size <= 0x20000) configreg |= 5 << 6;
		else if (c_dcache_size <= 0x40000) configreg |= 6 << 6;
		else                                   configreg |= 7 << 6;

		/* set the instruction cache size */
				if (c_icache_size <= 0x01000) configreg |= 0 << 9;
		else if (c_icache_size <= 0x02000) configreg |= 1 << 9;
		else if (c_icache_size <= 0x04000) configreg |= 2 << 9;
		else if (c_icache_size <= 0x08000) configreg |= 3 << 9;
		else if (c_icache_size <= 0x10000) configreg |= 4 << 9;
		else if (c_icache_size <= 0x20000) configreg |= 5 << 9;
		else if (c_icache_size <= 0x40000) configreg |= 6 << 9;
		else                                   configreg |= 7 << 9;

		if (c_secondary_cache_line_size != 0) {
			configreg &= ~((0xf << 20) | (1 << 17));
					if (c_secondary_cache_line_size <= 0x10) configreg |= 0 << 22;
			else if (c_secondary_cache_line_size <= 0x20) configreg |= 1 << 22;
			else if (c_secondary_cache_line_size <= 0x40) configreg |= 2 << 22;
			else                                          configreg |= 3 << 22;
		}
		/* set the system clock divider */
		int divisor = 2;
		if (c_system_clock != 0)
		{
			divisor = m_cpu_clock / c_system_clock;
			if (c_system_clock * divisor != m_cpu_clock)
			{
				configreg |= 0x80000000;
				divisor = m_cpu_clock * 2 / c_system_clock;
			}
		}
		configreg |= (((divisor < 2) ? 2 : (divisor > 8) ? 8 : divisor) - 2) << 28;
	}

	/* set the endianness bit */
	if (m_bigendian)
		configreg |= 0x00008000;

	return configreg;
}


/*-------------------------------------------------
    compute_prid_register - compute the value
    of the PRId register
-------------------------------------------------*/

uint32_t mips3_device::compute_prid_register()
{
	switch (m_flavor)
	{
		case MIPS3_TYPE_R4000:
			return 0x0400;

		case MIPS3_TYPE_R4400:
			return 0x0440;

		case MIPS3_TYPE_VR4300:
			return 0x0b00;

		case MIPS3_TYPE_VR5500:
			return 0x5500;

		case MIPS3_TYPE_R4600:
			return 0x2020;

		case MIPS3_TYPE_R4650:
			return 0x2200;

		case MIPS3_TYPE_R4700:
			return 0x2100;

		case MIPS3_TYPE_TX4925:
			return 0x2d23;

		case MIPS3_TYPE_R5000:
			return 0x2300;

		case MIPS3_TYPE_QED5271:
			return 0x2800;

		case MIPS3_TYPE_RM7000:
			return 0x2700;

		case MIPS3_TYPE_R5900:
			return 0x2e14;

		default:
			fatalerror("Unknown MIPS flavor specified\n");
	}
	// never executed
	//return 0x2000;
}

uint32_t mips3_device::compute_fpu_prid_register()
{
	/* For most CPUs they are same */
	return compute_prid_register();
}

/*-------------------------------------------------
    tlb_map_entry - map a single TLB
    entry
-------------------------------------------------*/

void mips3_device::tlb_map_entry(int tlbindex)
{
	int current_asid = m_core->cpr[0][COP0_EntryHi] & 0xff;
	const mips3_tlb_entry &entry = m_tlb[tlbindex];
	uint32_t count, vpn;
	int which;

	/* the ASID doesn't match the current ASID, and if the page isn't global, unmap it from the TLB */
	if (!entry.matches_asid(current_asid) && !entry.is_global())
	{
#ifdef MAME_SGI_TLB_WRITE_DEDUP_DIAG
		m_core->diag_tlb_write_degenerate += 2;
#endif
		vtlb_load(2 * tlbindex + 0, 0, 0, 0);
		vtlb_load(2 * tlbindex + 1, 0, 0, 0);
		return;
	}

	/* extract the VPN index; ignore if the virtual address is beyond 32 bits */
	vpn = ((entry.entry_hi >> 13) & 0x07ffffff) << 1;
	if (vpn >= (1 << (MIPS3_MAX_PADDR_SHIFT - MIPS3_MIN_PAGE_SHIFT)))
	{
#ifdef MAME_SGI_TLB_WRITE_DEDUP_DIAG
		m_core->diag_tlb_write_degenerate += 2;
#endif
		vtlb_load(2 * tlbindex + 0, 0, 0, 0);
		vtlb_load(2 * tlbindex + 1, 0, 0, 0);
		return;
	}

	/* get the number of pages from the page mask */
	/* R5900: if the S bit is set in EntryLo, it is the scratchpad, and is always 4 pages. */
	if ((entry.entry_lo[0] & 0x80000000) && m_flavor == MIPS3_TYPE_R5900)
		count = 4;
	else
		count = ((entry.page_mask >> 13) & 0x00fff) + 1;

	/* loop over both the even and odd pages */
	for (which = 0; which < 2; which++)
	{
		uint32_t effvpn = vpn + count * which;
		uint64_t lo = entry.entry_lo[which];
		uint32_t pfn;
		uint32_t flags = 0;

		/* compute physical page index */
		pfn = (lo >> 6) & m_pfnmask;

		/* valid? */
		if ((lo & 2) != 0)
		{
			flags |= FLAG_VALID | READ_ALLOWED | FETCH_ALLOWED;

			/* writable? */
			if ((lo & 4) != 0)
				flags |= WRITE_ALLOWED;

			/* mirror the flags for user mode if the VPN is in user space */
			if (effvpn < (0x80000000 >> MIPS3_MIN_PAGE_SHIFT))
				flags |= (flags << 4) & (USER_READ_ALLOWED | USER_WRITE_ALLOWED | USER_FETCH_ALLOWED);
		}

		/* load the virtual TLB with the corresponding entries */
		if ((effvpn + count) <= (0x80000000 >> MIPS3_MIN_PAGE_SHIFT) || effvpn >= (0xc0000000 >> MIPS3_MIN_PAGE_SHIFT))
		{
#ifdef MAME_SGI_TLB_WRITE_DEDUP_DIAG
			// TEMPORARY diagnostic (2026-08-07) - see diag_tlb_write_total's
			// comment in mips3.h. Snapshot before/after rather than
			// replicating vtlb_load()'s own FLAG_FIXED/pagenum-shift
			// encoding here - simpler and can't drift out of sync with it.
			vtlb_entry const before = vtlb_table()[effvpn];
#endif
			vtlb_load(2 * tlbindex + which, count, effvpn << MIPS3_MIN_PAGE_SHIFT, (pfn << MIPS3_MIN_PAGE_SHIFT) | flags);
#ifdef MAME_SGI_CC1_RODATA_TLB_RACE_DIAG
			// Target page: guest VA 0x10343970 (the known failing string's
			// address), page-aligned VPN = 0x10343970 >> MIPS3_MIN_PAGE_SHIFT.
			// Only matches the common 4KB-page case (count==1 covering
			// exactly this VPN) - a larger IRIX page size spanning this
			// address with a different base VPN would be missed by this
			// exact-match check; acceptable for a first pass, note if this
			// comes back silent despite reproducing the flake.
			{
				constexpr uint32_t target_vpn = 0x10343970u >> MIPS3_MIN_PAGE_SHIFT;
				if (target_vpn >= effvpn && target_vpn < effvpn + count && (flags & FETCH_ALLOWED))
				{
					static uint64_t s_event_count = 0;
					s_event_count++;
					uint32_t const page_pfn = pfn + (target_vpn - effvpn);
					uint32_t const phys = (page_pfn << MIPS3_MIN_PAGE_SHIFT) | (0x10343970u & 0xfffu);
					uint32_t const w0 = m_program->read_dword(phys & ~3u);
					uint32_t const w1 = m_program->read_dword((phys & ~3u) + 4);
					uint32_t const w2 = m_program->read_dword((phys & ~3u) + 8);
					uint32_t const w3 = m_program->read_dword((phys & ~3u) + 12);
					bool const all_zero = (w0 == 0 && w1 == 0 && w2 == 0 && w3 == 0);
					fprintf(stderr, "[MAME_SGI_CC1_RODATA_TLB_RACE_DIAG] #%llu asid=0x%02x effvpn=0x%05x count=%u pfn=0x%05x phys=0x%08x words=%08x %08x %08x %08x %s\n",
						(unsigned long long)s_event_count, current_asid, effvpn, count, page_pfn, phys,
						w0, w1, w2, w3, all_zero ? "ALL_ZERO_SUSPICIOUS" : "nonzero");
					fflush(stderr);
				}
			}
#endif
#ifdef MAME_SGI_TLB_WRITE_DEDUP_DIAG
			{
				static uint32_t s_last_reported = 0;
				vtlb_entry const after = vtlb_table()[effvpn];
				uint32_t const total = ++m_core->diag_tlb_write_total;
				if (after == before)
					++m_core->diag_tlb_write_nochange;
				if (total >= s_last_reported + 50000)
				{
					fprintf(stderr, "[TLB_WRITE_DEDUP] total=%u nochange=%u (%.4f%%) degenerate=%u\n",
						total, m_core->diag_tlb_write_nochange,
						100.0 * double(m_core->diag_tlb_write_nochange) / double(total),
						m_core->diag_tlb_write_degenerate);
					fflush(stderr);
					s_last_reported = total;
				}
			}
#endif
		}
		else
		{
#ifdef MAME_SGI_TLB_WRITE_DEDUP_DIAG
			m_core->diag_tlb_write_degenerate++;
#endif
			vtlb_load(2 * tlbindex + which, 0, 0, 0);
		}
	}
}


/*-------------------------------------------------
    tlb_write_common - common routine for writing
    a TLB entry
-------------------------------------------------*/

void mips3_device::tlb_write_common(int tlbindex)
{
	/* only handle entries within the TLB */
	if (tlbindex < m_tlbentries)
	{
#ifdef MAME_SGI_VTLB_MUTATION_FREQ_DIAG
		// TEMPORARY diagnostic (2026-08-07) - see diag_tlbwi_tlbwr_events'
		// comment in mips3.h. One event per guest TLBWI/TLBWR instruction,
		// independent of any ASID switch - the trigger the epoch design
		// needs a real frequency number for before choosing global vs.
		// per-ASID granularity.
		++m_core->diag_tlbwi_tlbwr_events;
#endif
		mips3_tlb_entry &entry = m_tlb[tlbindex];

		/* fill in the new TLB entry from the COP0 registers */
		entry.page_mask = m_core->cpr[0][COP0_PageMask];
		entry.entry_hi = m_core->cpr[0][COP0_EntryHi] & ~(entry.page_mask & u64(0x0000000001ffe000U));
		entry.entry_lo[0] = m_core->cpr[0][COP0_EntryLo0];
		entry.entry_lo[1] = m_core->cpr[0][COP0_EntryLo1];

		/* remap this TLB entry */
		tlb_map_entry(tlbindex);
		/* log the two halves once they are in */
		entry.log_half(tlbindex, 0);
		entry.log_half(tlbindex, 1);
	}
}


/*-------------------------------------------------
    tlb_entry_log_half - log half of a single TLB
    entry
-------------------------------------------------*/

void mips3_device::mips3_tlb_entry::log_half(int tlbindex, int which) const
{
	if (PRINTF_TLB)
	{
		const uint64_t hi = entry_hi;
		const uint64_t lo = entry_lo[which];
		const uint32_t vpn = (((hi >> 13) & 0x07ffffff) << 1);
		const uint32_t asid = hi & 0xff;
		const uint32_t r = (hi >> 62) & 3;
		const uint32_t pfn = (lo >> 6) & 0x00ffffff;
		const uint32_t c = (lo >> 3) & 7;
		const uint32_t pagesize = (((page_mask >> 13) & 0xfff) + 1) << MIPS3_MIN_PAGE_SHIFT;
		const uint64_t vaddr = ((uint64_t)vpn * MIPS3_MIN_PAGE_SIZE) + (pagesize * which);
		const uint64_t paddr = (uint64_t)pfn * MIPS3_MIN_PAGE_SIZE;

		util::stream_format(std::cout,
				"index=%08X  pagesize=%08X  vaddr=%016X  paddr=%016X  asid=%02X  r=%X  c=%X  dvg=%c%c%c\n",
				tlbindex, pagesize, vaddr, paddr,
				asid, r, c, (lo & 4) ? 'd' : '.', (lo & 2) ? 'v' : '.', (lo & 1) ? 'g' : '.');
	}
}
