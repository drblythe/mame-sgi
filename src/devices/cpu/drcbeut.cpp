// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    Utility functions for dynamic recompiling backends.

***************************************************************************/

// DISABLED (2026-07-29): the get_value() scan-distance instrumentation
// below (see drcbeut.h's own comment on the m_diag_scan_* fields) is
// gated behind this rather than left unconditionally active, after the
// user raised a real concern this file's own header comment didn't
// address: raw per-call cost is negligible, but if the actual bug under
// investigation has any race-condition/timing-sensitive character,
// *any* perturbation - even a few nanoseconds added consistently to a
// path called thousands of times a second - could change whether it
// manifests, making "negligible overhead" not the same guarantee as
// "doesn't affect the bug." Re-enable only for a deliberate, isolated
// measurement pass, not as a standing default.
//
// Re-enabled briefly 2026-09-30 to finally answer the question this
// comment poses (measured result: call-frequency-dominated, not
// scan-distance-dominated) - then disabled again below, as intended.
// #define MAME_SGI_GETVALUE_SCAN_DIAG

// Memoise drc_map_variables::get_value(). Each call (the RECOVER opcode,
// used by exception handlers to recover the guest PC and cycle count)
// otherwise scans forward from the code pointer for the block's map data
// and walks it, and exception-heavy guests make that a measurable share of
// host time. See the value_memo comment in drcbeut.h.
#define MAME_SGI_RECOVER_MEMO

#include "emu.h"
#include "drcbeut.h"

#include <algorithm>
#include <numeric>


namespace drc {

namespace {

template <typename T>
std::size_t block_bytes(unsigned count, std::align_val_t align)
{
	std::size_t const blocksize = count * sizeof(T);
	return ((blocksize + (std::size_t(align) - 1)) / std::size_t(align)) * std::size_t(align);
}

} // anonymous namespace

using namespace uml;


//**************************************************************************
//  DEBUGGING
//**************************************************************************

#define LOG_RECOVER         (0)



//**************************************************************************
//  DRC HASH TABLE
//**************************************************************************

//-------------------------------------------------
//  drc_hash_table - constructor
//-------------------------------------------------

template <typename Entry>
drc_hash_table_t<Entry>::drc_hash_table_t(
		drc_cache &cache,
		uint32_t modes,
		uint8_t addrbits,
		uint8_t ignorebits,
		uint32_t max_sequence_length,
		std::align_val_t modealign,
		std::align_val_t l1align,
		std::align_val_t l2align) :
	m_invariant_modes(modes, false),
	m_next_l1_block(0),
	m_next_l2_block(0),
	m_invariant_mode_count(0),
	m_cache(cache),
	m_modealign(std::align_val_t(std::lcm(std::size_t(modealign), alignof(Entry **)))),
	m_l1align(std::align_val_t(std::lcm(std::size_t(l1align), alignof(Entry *)))),
	m_l2align(std::align_val_t(std::lcm(std::size_t(l2align), alignof(Entry)))),
	m_nocodeptr(nullptr),
	m_modes(modes),
	m_max_sequence_length(max_sequence_length),
	m_l1bits((addrbits - ignorebits) / 2),
	m_l2bits((addrbits - ignorebits) - m_l1bits),
	m_l1shift(ignorebits + m_l2bits),
	m_l2shift(ignorebits),
	m_l1mask((1 << m_l1bits) - 1),
	m_l2mask((1 << m_l2bits) - 1),
	m_base(reinterpret_cast<Entry ***>(cache.alloc(modes * sizeof(Entry **), m_modealign))),
	m_emptyl1(reinterpret_cast<Entry **>(m_cache.alloc(sizeof(Entry *) << m_l1bits, m_l1align))),
	m_emptyl2(reinterpret_cast<Entry *>(m_cache.alloc(sizeof(Entry) << m_l2bits, m_l2align))),
	m_l1alloc(std::min<std::size_t>(L1_ALLOCATION, m_modes) * block_bytes<Entry *>(1 << m_l1bits, m_l1align)),
	m_l2alloc(L2_ALLOCATION * block_bytes<Entry>(1 << m_l2bits, m_l2align))
{
#if defined(MAME_DEBUG)
	osd_printf_verbose(
			"drc_hash_table: parameters:\n"
			"%u modes, %u address bits, %u ignored, %u L1 bits ((pc >> %u) & 0x%x), %u L2 bits ((pc >> %u) >> 0x%x)\n"
			"L1 allocation %u (%u * %u), L2 allocation %u (%u * %u)\n",
			m_modes, addrbits, ignorebits,
			m_l1bits, m_l1shift, m_l1mask,
			m_l2bits, m_l2shift, m_l2mask,
			m_l1alloc, block_bytes<Entry *>(1 << m_l1bits, m_l1align), std::min<std::size_t>(L1_ALLOCATION, m_modes),
			m_l2alloc, block_bytes<Entry>(1 << m_l2bits, m_l2align), L2_ALLOCATION);
#endif

	m_l1blocks.reserve(std::min<std::size_t>(L1_ALLOCATION, m_modes));
	m_l2blocks.reserve(L2_ALLOCATION);

	if (m_emptyl1)
		std::fill_n(m_emptyl1, 1 << m_l1bits, m_emptyl2);

	if (m_emptyl2)
		std::fill_n(m_emptyl2, 1 << m_l2bits, encode(m_nocodeptr));

	reset();
}


//-------------------------------------------------
//  reset - flush existing hash tables and create
//  new ones
//-------------------------------------------------

template <typename Entry>
bool drc_hash_table_t<Entry>::reset() noexcept
{
	// start from the beginning of the allocated blocks
	m_next_l1_block = m_invariant_mode_count;
	m_next_l2_block = 0;

	if (!m_base || !m_emptyl1 || !m_emptyl2)
		return false;

	// reset the top-level table
	for (uint32_t i = 0; m_modes > i; ++i)
	{
		if (!m_invariant_modes[i])
			m_base[i] = m_emptyl1;
		else
			std::fill_n(m_base[i], 1 << m_l1bits, m_emptyl2);
	}

	return true;
}


//-------------------------------------------------
//  populate_mode - try to populate mode table
//  entry
//-------------------------------------------------

template <typename Entry>
bool drc_hash_table_t<Entry>::populate_mode(uint32_t mode) noexcept
{
	assert(mode < m_modes);
	if (m_base[mode] != m_emptyl1)
		return true;

	// copy-on-write for the L1 hash table
	if (m_l1blocks.size() == m_next_l1_block)
	{
		try { m_l1blocks.reserve(m_l1blocks.size() + std::min<std::size_t>(L1_ALLOCATION, m_modes)); }
		catch (std::bad_alloc const &) { return false; }

		Entry **const l1block = reinterpret_cast<Entry **>(m_cache.alloc(m_l1alloc, m_l1align));
		if (!l1block)
			return false;

		for (unsigned i = 0; std::min<std::size_t>(L1_ALLOCATION, m_modes) > i; ++i)
		{
			void *const table = reinterpret_cast<uint8_t *>(l1block) + (block_bytes<Entry *>(1 << m_l1bits, m_l1align) * i);
			m_l1blocks.emplace_back(reinterpret_cast<Entry **>(table));
		}
	}

	m_base[mode] = m_l1blocks[m_next_l1_block++];
	std::fill_n(m_base[mode], 1 << m_l1bits, m_emptyl2);

	return true;
}


//-------------------------------------------------
//  block_begin - note the beginning of a block
//-------------------------------------------------

template <typename Entry>
void drc_hash_table_t<Entry>::stats(std::vector<std::pair<uint32_t, uint32_t>> &modes, std::vector<std::array<uint32_t, 4>> &l2s, std::array<uint64_t, 6> &slices_by_l2bits, std::size_t &l1_allocated, std::size_t &l2_allocated, std::size_t &table_bytes) const noexcept
{
	// MAME_SGI_HASH_TABLE_STATS
	for (uint32_t mode = 0; mode < m_modes; ++mode)
	{
		if (m_base[mode] == m_emptyl1)
			continue;
		uint32_t l2 = 0;
		for (uint32_t i = 0; i < (1U << m_l1bits); ++i)
		{
			if (m_base[mode][i] == m_emptyl2)
				continue;
			++l2;
			uint32_t entries = 0, pages = 0, lastpage = ~0U;
			uint32_t lastsub[6] = { ~0U, ~0U, ~0U, ~0U, ~0U, ~0U };
			for (uint32_t j = 0; j < (1U << m_l2bits); ++j)
			{
				if (m_base[mode][i][j] == encode(m_nocodeptr))
					continue;
				++entries;
				// distinct sub-slices this L2 would need with a 10..15-bit L2
				for (unsigned b = 0; b < 6; ++b)
				{
					uint32_t const sub = j >> (10 + b);
					if (sub != lastsub[b])
					{
						lastsub[b] = sub;
						++slices_by_l2bits[b];
					}
				}
				uint32_t const page = j >> (12 - m_l2shift);
				if (page != lastpage)
				{
					++pages;
					lastpage = page;
				}
			}
			l2s.push_back({ mode, i << m_l1shift, entries, pages });
		}
		modes.emplace_back(mode, l2);
	}
	l1_allocated = m_l1blocks.size();
	l2_allocated = m_l2blocks.size();
	table_bytes = l1_allocated * block_bytes<Entry *>(1 << m_l1bits, m_l1align) + l2_allocated * block_bytes<Entry>(1 << m_l2bits, m_l2align);
}


template <typename Entry>
void drc_hash_table_t<Entry>::block_begin(drcuml_block &block, const uml::instruction *instlist, uint32_t numinst)
{
	// before generating code, pre-allocate any hash entries; we do this by setting dummy hash values
	for (int inum = 0; inum < numinst; inum++)
	{
		const uml::instruction &inst = instlist[inum];

		if (inst.opcode() == OP_HASH)
		{
			// if the opcode is a hash, verify that it makes sense and then set a nullptr entry
			if (UNEXPECTED(block.invariant()))
				throw emu_fatalerror("UML HASH is not permitted in invariant blocks.\n");

			// if we fail to allocate, we must abort the block
			if (!set_codeptr(inst.param(0).immediate(), inst.param(1).immediate(), nullptr))
				block.abort();
		}
		else if (inst.opcode() == OP_HASHJMP)
		{
			if (inst.param(0).is_immediate())
			{
				const uint32_t mode = inst.param(0).immediate();
				if (block.invariant())
				{
					// mark mode as used in invariant code
					assert(m_next_l1_block == m_invariant_mode_count);

					if (!populate_mode(mode))
						block.abort();

					m_invariant_modes[mode] = true;
					m_invariant_mode_count = m_next_l1_block;
				}
				else if (inst.param(1).is_immediate())
				{
					// preallocate the tables for hashjmp to fixed location in transient code
					drccodeptr const code = get_codeptr(mode, inst.param(1).immediate());
					if (!set_codeptr(mode, inst.param(1).immediate(), code))
						block.abort();
				}
			}
		}
	}
}


//-------------------------------------------------
//  block_end - note the end of a block
//-------------------------------------------------

template <typename Entry>
void drc_hash_table_t<Entry>::block_end(drcuml_block &block)
{
	// nothing to do here, yet
}


//-------------------------------------------------
//  set_default_codeptr - change the default
//  codeptr
//-------------------------------------------------

template <typename Entry>
void drc_hash_table_t<Entry>::set_default_codeptr(drccodeptr nocodeptr) noexcept
{
	// nothing to do if the same
	drccodeptr const old = m_nocodeptr;
	if (old == nocodeptr)
		return;
	m_nocodeptr = nocodeptr;

	// update the empty L2 table first
	std::fill_n(m_emptyl2, 1 << m_l2bits, encode(nocodeptr));

	// now scan all existing L2 tables for entries
	for (uint32_t i = 0; m_next_l2_block > i; ++i)
	{
		for (uint32_t l2entry = 0; (1 << m_l2bits > l2entry); ++l2entry)
		{
			if (m_l2blocks[i][l2entry] == encode(old))
				m_l2blocks[i][l2entry] = encode(nocodeptr);
		}
	}
}


//-------------------------------------------------
//  set_codeptr - set the codeptr for the given
//  mode/pc
//-------------------------------------------------

template <typename Entry>
bool drc_hash_table_t<Entry>::set_codeptr(uint32_t mode, uint32_t pc, drccodeptr code) noexcept
{
	if (!populate_mode(mode))
		return false;

	// copy-on-write for the L2 hash table
	uint32_t const l1 = (pc >> m_l1shift) & m_l1mask;
	if (m_base[mode][l1] == m_emptyl2)
	{
		if (m_l2blocks.size() == m_next_l2_block)
		{
			try { m_l2blocks.reserve(m_l2blocks.size() + L2_ALLOCATION); }
			catch (std::bad_alloc const &) { return false; }

			Entry *const l2block = reinterpret_cast<Entry *>(m_cache.alloc(m_l2alloc, m_l2align));
			if (!l2block)
				return false;

			for (unsigned i = 0; L2_ALLOCATION > i; ++i)
			{
				void *const table = reinterpret_cast<uint8_t *>(l2block) + (block_bytes<Entry>(1 << m_l2bits, m_l2align) * i);
				m_l2blocks.emplace_back(reinterpret_cast<Entry *>(table));
			}
		}

		m_base[mode][l1] = m_l2blocks[m_next_l2_block++];
		std::fill_n(m_base[mode][l1], 1 << m_l2bits, encode(m_nocodeptr));
	}

	// set the new entry
	uint32_t const l2 = (pc >> m_l2shift) & m_l2mask;
	m_base[mode][l1][l2] = encode(code);
	return true;
}


//-------------------------------------------------
//  invalidate_range - force all blocks in the
//  given PC range to recompile on their next entry
//
//  TODO: Will need to reconsider this in the future
//  when the DRC system is expanded to handle 64-bit
//  program counters.
//-------------------------------------------------

template <typename Entry>
void drc_hash_table_t<Entry>::invalidate_range(uint32_t pcstart, uint32_t pcend) noexcept
{
	// Sequences are hashed on their starting address, so a sequence at the start of this
	// block may have crossed over from a previous block.  Back up the starting address by
	// the maximum size (in bytes) of a generated sequence for this DRC.
	pcstart -= std::min(pcstart, m_max_sequence_length);

	for (uint32_t mode = 0; mode < m_modes; mode++)
	{
		if (m_base[mode] == m_emptyl1)
		{
			continue;
		}

		uint32_t pc = pcstart;
		while (true)
		{
			const uint32_t slotend = std::min<uint64_t>(uint64_t(pc) | ((uint32_t(1) << m_l1shift) - 1), pcend);
			Entry *const l2 = m_base[mode][(pc >> m_l1shift) & m_l1mask];
			if (l2 != m_emptyl2)
			{
				for (uint32_t p = pc; p <= slotend; p += uint32_t(1) << m_l2shift)
				{
					l2[(p >> m_l2shift) & m_l2mask] = encode(m_nocodeptr);
					if (p > 0xffff'ffff - (uint32_t(1) << m_l2shift))
					{
						break;
					}
				}
			}

			if (slotend >= pcend)
			{
				break;
			}

			pc = slotend + 1;
		}
	}
}



template class drc_hash_table_t<drccodeptr>;
template class drc_hash_table_t<uint32_t>;  // MAME_SGI_HASH32


//**************************************************************************
//  DRC MAP VARIABLES
//**************************************************************************

//-------------------------------------------------
//  drc_map_variables - constructor
//-------------------------------------------------

drc_map_variables::drc_map_variables(drc_cache &cache, uint64_t uniquevalue) :
	m_cache(cache),
	m_uniquevalue(uniquevalue)
{
	std::fill(std::begin(m_mapvalue), std::end(m_mapvalue), 0);
#ifdef MAME_SGI_RECOVER_MEMO
	m_value_memo = std::make_unique<value_memo []>(size_t(1) << VALUE_MEMO_BITS);
#endif
}


//-------------------------------------------------
//  ~drc_map_variables - destructor
//-------------------------------------------------

drc_map_variables::~drc_map_variables()
{
	if (m_value_memo_hits || m_value_memo_misses)
		osd_printf_verbose("drc_map_variables: get_value memo %llu hits, %llu misses\n",
				(unsigned long long)m_value_memo_hits, (unsigned long long)m_value_memo_misses);
}


//-------------------------------------------------
//  block_begin - note the beginning of a block
//-------------------------------------------------

void drc_map_variables::block_begin(drcuml_block &block)
{
	// release any remaining live entries
	m_entry_list.clear();
	m_entry_list.reserve(128);

	// reset the variable values
	std::fill(std::begin(m_mapvalue), std::end(m_mapvalue), 0);
}


//-------------------------------------------------
//  block_end - note the end of a block
//-------------------------------------------------

void drc_map_variables::block_end(drcuml_block &block)
{
	// only process if we have data
	if (m_entry_list.empty())
		return;

	// now iterate over entries and store them
	std::array<uint32_t, MAPVAR_COUNT> curvalue;
	std::array<bool, MAPVAR_COUNT> changed;
	std::fill(curvalue.begin(), curvalue.end(), 0);
	std::fill(changed.begin(), changed.end(), false);
	m_out_temp.clear();
	m_out_temp.reserve(m_entry_list.size() * 2);
	drccodeptr lastptr = m_entry_list.front().codeptr;
	auto entry = m_entry_list.begin();
	while (m_entry_list.end() != entry)
	{
		// update the current value of the variable and detect changes
		if (curvalue[entry->mapvar] != entry->newval)
		{
			curvalue[entry->mapvar] = entry->newval;
			changed[entry->mapvar] = true;
		}

		// if the next code pointer is different, or if we're at the end, flush changes
		auto const next = std::next(entry);
		if ((m_entry_list.end() == next) || (next->codeptr != entry->codeptr))
		{
			// build a mask of changed variables
			int numchanged = 0;
			uint32_t varmask = 0;
			for (int varnum = 0; varnum < std::size(changed); varnum++)
			{
				if (changed[varnum])
				{
					changed[varnum] = false;
					varmask |= 1 << varnum;
					numchanged++;
				}
			}

			// if nothing really changed, skip it
			if (numchanged)
			{
				// first word is a code delta plus mask of changed variables
				uint32_t codedelta = entry->codeptr - lastptr;
				while (codedelta > 0xffff)
				{
					m_out_temp.emplace_back(uint32_t(0xffff) << 16);
					codedelta -= 0xffff;
				}
				m_out_temp.emplace_back((codedelta << 16) | (varmask << 4) | numchanged);

				// now output updated variable values
				for (int varnum = 0; varnum < changed.size(); varnum++)
				{
					if (BIT(varmask, varnum))
						m_out_temp.emplace_back(curvalue[varnum]);
				}

				// remember our lastptr
				lastptr = entry->codeptr;
			}
		}
		entry = next;
	}

	// add a terminator
	m_out_temp.emplace_back(0);

	// begin "code generation" aligned to an 8-byte boundary
	auto const required = sizeof(uint64_t) + sizeof(uint32_t) + (sizeof(uint32_t) * m_out_temp.size());
	auto const align = std::align_val_t(std::lcm<std::size_t>(alignof(uint64_t), 8));
	auto dest = reinterpret_cast<uint32_t *>(block.invariant()
			? m_cache.alloc_invariant(required, align)
			: m_cache.alloc_transient(required, align));
	if (!dest)
		block.abort();

	// store the cookie first
	*(uint64_t *)dest = m_uniquevalue;
	dest += 2;

	// get the pointer to the first item and store an initial backwards offset
	*dest = drccodeptr(dest) - m_entry_list.front().codeptr;
	dest++;

	// copy the actual entries
	dest = std::copy(m_out_temp.begin(), m_out_temp.end(), dest);
}


//-------------------------------------------------
//  set_value - set a map value for the given
//  code pointer
//-------------------------------------------------

void drc_map_variables::set_value(drccodeptr codebase, uint32_t mapvar, uint32_t newvalue)
{
	assert((mapvar >= MAPVAR_M0) && (mapvar < MAPVAR_END));

	// if this value isn't different, skip it
	if (m_mapvalue[mapvar - MAPVAR_M0] == newvalue)
		return;

	// allocate a new entry and fill it in
	auto &entry = m_entry_list.emplace_back();
	entry.codeptr = codebase;
	entry.mapvar = mapvar - MAPVAR_M0;
	entry.newval = newvalue;

	// update our state in the table as well
	m_mapvalue[mapvar - MAPVAR_M0] = newvalue;
}


//-------------------------------------------------
//  get_value - return a map value for the given
//  code pointer
//-------------------------------------------------

uint32_t drc_map_variables::get_value(drccodeptr codebase, uint32_t mapvar) const
{
	assert((mapvar >= MAPVAR_M0) && (mapvar < MAPVAR_END));
	mapvar -= MAPVAR_M0;

#ifdef MAME_SGI_RECOVER_MEMO
	value_memo *memo = nullptr;
	if (m_value_memo)
	{
		uintptr_t const key = uintptr_t(codebase);
		unsigned const index = unsigned((key ^ (key >> VALUE_MEMO_BITS) ^ (mapvar * 0x9e3779b1U)) & ((1U << VALUE_MEMO_BITS) - 1));
		memo = &m_value_memo[index];
		if ((memo->codebase == codebase) && (memo->mapvar == mapvar) && (memo->generation == m_cache.generation()))
		{
			m_value_memo_hits++;
			return memo->value;
		}
		m_value_memo_misses++;
	}
#endif

	// get an aligned pointer to start scanning
	auto curscan = reinterpret_cast<uint64_t const *>((uintptr_t(codebase) | 7) + 1);
	auto const endscan = reinterpret_cast<uint64_t const *>(m_cache.top());

#ifdef MAME_SGI_GETVALUE_SCAN_DIAG
	// TEMPORARY diagnostic (2026-07-29): measure real scan distance - see
	// this class's own comment in drcbeut.h for why (in-memory counters,
	// no I/O, read via a live debugger session rather than printed).
	auto const diag_scan_start = curscan;
#endif

	// look for the signature
	while ((curscan < endscan) && (*curscan++ != m_uniquevalue)) { }

#ifdef MAME_SGI_GETVALUE_SCAN_DIAG
	m_diag_scan_calls++;
	uint64_t const diag_words_scanned = curscan - diag_scan_start;
	m_diag_scan_words_total += diag_words_scanned;
	if (diag_words_scanned > m_diag_scan_words_max)
		m_diag_scan_words_max = diag_words_scanned;
#endif

	if (curscan >= endscan)
		return 0;

	// switch to 32-bit pointers for processing the rest
	auto data = reinterpret_cast<uint32_t const *>(curscan);

	// first get the 32-bit starting offset to the code
	drccodeptr curcode = (drccodeptr)data - *data;
	data++;

	// now loop until we advance past our target
	uint32_t const varmask = 0x10 << mapvar;
	uint32_t result = 0;
	while (true)
	{
		// a 0 is a terminator
		uint32_t controlword = *data++;
		if (controlword == 0)
			break;

		// update the codeptr; if this puts us past the end, we're done
		curcode += (controlword >> 16) & 0xffff;
		if (curcode > codebase)
			break;

		// if our mapvar has changed, process this word
		if (controlword & varmask)
		{
			// count how many words precede the one we care about
			int dataoffs = 0;
			for (uint32_t skipmask = (controlword & (varmask - 1)) >> 4; skipmask != 0; skipmask = skipmask & (skipmask - 1))
				dataoffs++;

			// fetch the one we want
			result = data[dataoffs];
		}

		// low 4 bits contain the total number of words of data
		data += controlword & 0x0f;
	}
	if (LOG_RECOVER)
		printf("recover %d @ %p = %08X\n", mapvar, codebase, result);
#ifdef MAME_SGI_RECOVER_MEMO
	if (memo)
	{
		memo->codebase = codebase;
		memo->generation = m_cache.generation();
		memo->mapvar = mapvar;
		memo->value = result;
	}
#endif
	return result;
}



//-------------------------------------------------
//  get_last_value - return the most recently set
//  map value
//-------------------------------------------------

uint32_t drc_map_variables::get_last_value(uint32_t mapvar)
{
	assert(mapvar >= MAPVAR_M0 && mapvar < MAPVAR_END);
	return m_mapvalue[mapvar - MAPVAR_M0];
}



//**************************************************************************
//  DRC LABEL LIST
//**************************************************************************

//-------------------------------------------------
//  drc_label_list - constructor
//-------------------------------------------------

drc_label_list::drc_label_list()
{
}


//-------------------------------------------------
//  ~drc_label_list - destructor
//-------------------------------------------------

drc_label_list::~drc_label_list()
{
}


//-------------------------------------------------
//  block_begin - note the beginning of a block
//-------------------------------------------------

void drc_label_list::block_begin(drcuml_block &block)
{
	// make sure the label list is clear, but don't fatalerror
	reset(false);
}


//-------------------------------------------------
//  block_end - note the end of a block
//-------------------------------------------------

void drc_label_list::block_end(drcuml_block &block)
{
	// service fixup requests
	for (auto const &fixup : m_fixup_list)
		fixup.callback(fixup.param, fixup.label->codeptr);

	// free all of the pending fixup requests
	m_free_fixups.splice(m_free_fixups.begin(), m_fixup_list);

	// make sure the label list is clear, and fatalerror if we missed anything
	reset(true);
}


//-------------------------------------------------
//  get_codeptr - find or allocate a new label;
//  returns nullptr and requests an OOB callback if
//  undefined
//-------------------------------------------------

drccodeptr drc_label_list::get_codeptr(uml::code_label label, drc_label_fixup_delegate const &callback, void *param)
{
	label_entry &curlabel = find_or_allocate(label);

	// if no code pointer, request an OOB callback
	if (!curlabel.codeptr && !callback.isnull())
	{
		label_fixup_list::iterator fixup = m_free_fixups.begin();
		if (m_free_fixups.end() == fixup)
			fixup = m_fixup_list.emplace(m_fixup_list.end());
		else
			m_fixup_list.splice(m_fixup_list.end(), m_free_fixups, fixup);
		fixup->label = &curlabel;
		fixup->param = param;
		fixup->callback = callback;
	}

	return curlabel.codeptr;
}


//-------------------------------------------------
//  set_codeptr - set the pointer to a new label
//-------------------------------------------------

void drc_label_list::set_codeptr(uml::code_label label, drccodeptr codeptr)
{
	// set the code pointer
	label_entry &curlabel = find_or_allocate(label);
	assert(!curlabel.codeptr);
	curlabel.codeptr = codeptr;
}


//-------------------------------------------------
//  reset - reset a label list (add all entries to
//  the free list)
//-------------------------------------------------

void drc_label_list::reset(bool fatal_on_leftovers)
{
	// loop until out of labels
	while (!m_list.empty())
	{
		// fatal if we were a leftover
		if (fatal_on_leftovers && !m_list.front().codeptr)
			throw emu_fatalerror("Label %08X never defined!\n", m_list.front().label.label());

		// free the label
		m_free_labels.splice(m_free_labels.begin(), m_list, m_list.begin());
	}
}


//-------------------------------------------------
//  find_or_allocate - look up a label and
//  allocate a new one if not found
//-------------------------------------------------

drc_label_list::label_entry &drc_label_list::find_or_allocate(uml::code_label label)
{
	// find the label, or else allocate a new one
	for (auto curlabel = m_list.begin(); m_list.end() != curlabel; ++curlabel)
	{
		if (curlabel->label == label)
			return *curlabel;
	}

	// if none found, allocate
	auto newlabel = m_free_labels.begin();
	if (m_free_labels.end() == newlabel)
		newlabel = m_list.emplace(m_list.end());
	else
		m_list.splice(m_list.end(), m_free_labels, newlabel);
	newlabel->label = label;
	newlabel->codeptr = nullptr;
	return *newlabel;
}



//**************************************************************************
//  RESOLVED MEMORY ACCESSORS
//**************************************************************************

//-------------------------------------------------
//  set - bind to address space
//-------------------------------------------------

void resolved_memory_accessors::set(address_space &space)
{
	read_byte          .set(space, static_cast<u8  (address_space::*)(offs_t)     >(&address_space::read_byte));
	read_byte_masked   .set(space, static_cast<u8  (address_space::*)(offs_t, u8) >(&address_space::read_byte));
	read_word          .set(space, static_cast<u16 (address_space::*)(offs_t)     >(&address_space::read_word));
	read_word_masked   .set(space, static_cast<u16 (address_space::*)(offs_t, u16)>(&address_space::read_word));
	read_dword         .set(space, static_cast<u32 (address_space::*)(offs_t)     >(&address_space::read_dword));
	read_dword_masked  .set(space, static_cast<u32 (address_space::*)(offs_t, u32)>(&address_space::read_dword));
	read_qword         .set(space, static_cast<u64 (address_space::*)(offs_t)     >(&address_space::read_qword));
	read_qword_masked  .set(space, static_cast<u64 (address_space::*)(offs_t, u64)>(&address_space::read_qword));

	write_byte         .set(space, static_cast<void (address_space::*)(offs_t, u8)      >(&address_space::write_byte));
	write_byte_masked  .set(space, static_cast<void (address_space::*)(offs_t, u8, u8)  >(&address_space::write_byte));
	write_word         .set(space, static_cast<void (address_space::*)(offs_t, u16)     >(&address_space::write_word));
	write_word_masked  .set(space, static_cast<void (address_space::*)(offs_t, u16, u16)>(&address_space::write_word));
	write_dword        .set(space, static_cast<void (address_space::*)(offs_t, u32)     >(&address_space::write_dword));
	write_dword_masked .set(space, static_cast<void (address_space::*)(offs_t, u32, u32)>(&address_space::write_dword));
	write_qword        .set(space, static_cast<void (address_space::*)(offs_t, u64)     >(&address_space::write_qword));
	write_qword_masked .set(space, static_cast<void (address_space::*)(offs_t, u64, u64)>(&address_space::write_qword));

	if (
			!read_byte  || !read_byte_masked  || !write_byte  || !write_byte_masked  ||
			!read_word  || !read_word_masked  || !write_word  || !write_word_masked  ||
			!read_dword || !read_dword_masked || !write_dword || !write_dword_masked ||
			!read_qword || !read_qword_masked || !write_qword || !write_qword_masked)
	{
		throw emu_fatalerror("Error resolving address space accessor member functions!\n");
	}
}

} // namespace drc
