// license:BSD-3-Clause
// copyright-holders:Olivier Galibert

#include "emu.h"
#include "emumem_hea.h"
#include "emumem_hem.h"

// TEMPORARY DIAGNOSTIC (2026-09-30, task #119). sgi/mc.cpp's add_fastram()
// registration (CPU-side DRC fast path) never sees a write that
// genuinely lands at physical 0x1014 (or its two RAM aliases,
// 0x08001014/0x0c001014) - confirmed by instrumenting the DRC's own
// static_generate_fastram_accessor() directly (both read and write
// branches, every size, masked or not): 500 captured events there are
// all reads, never a write. This hooks the OTHER side of the same
// write - the generic, device-agnostic slow path every RAM write
// ultimately funnels through, whether it comes from a CPU instruction's
// fallback dispatch, a DMA engine's own C++ write_*() call, or the
// debugger - to see whether a write to this address ever reaches here
// at all, and if so, from where. Filtered to 3 literal addresses so it
// stays harmless/no-op for every other driver in the tree; self-defined
// here rather than via sgi.lua's defines{} since this file is outside
// the mame_sgi project's own scope (src/emu/, shared by every driver).
// #define MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
namespace {
	inline void mame_sgi_fastram_write_trace(const char *fn, offs_t offset, uint64_t data, uint64_t mem_mask)
	{
		if (offset != 0x00001014 && offset != 0x08001014 && offset != 0x0c001014)
			return;
		static uint64_t s_count = 0;
		s_count++;
		FILE *f = fopen("/tmp/mame_sgi_fastram_dump.txt", "a");
		if (!f)
			return;
		fprintf(f, "[MAME_SGI_FASTRAM_WRITE_TRACE_DIAG] #%llu %s offset=0x%08x data=0x%016llx mask=0x%016llx\n",
			(unsigned long long)s_count, fn, offset, (unsigned long long)data, (unsigned long long)mem_mask);
		fclose(f);
	}
	inline void mame_sgi_fastram_read_trace(const char *fn, offs_t offset, const void *base, offs_t address_base, offs_t address_mask, uint64_t value)
	{
		if (offset != 0x00001014 && offset != 0x08001014 && offset != 0x0c001014)
			return;
		static uint64_t s_count = 0;
		s_count++;
		FILE *f = fopen("/tmp/mame_sgi_fastram_dump.txt", "a");
		if (!f)
			return;
		fprintf(f, "[MAME_SGI_FASTRAM_WRITE_TRACE_DIAG] #%llu %s offset=0x%08x m_base=%p address_base=0x%08x address_mask=0x%08x value=0x%016llx\n",
			(unsigned long long)s_count, fn, offset, base, address_base, address_mask, (unsigned long long)value);
		fclose(f);
	}
}
#endif

template<int Width, int AddrShift> emu::detail::handler_entry_size_t<Width> handler_entry_read_memory<Width, AddrShift>::read(offs_t offset, uX mem_mask) const
{
	uX const val = m_base[((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift)];
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
	mame_sgi_fastram_read_trace("read", offset, (void *)m_base, this->m_address_base, this->m_address_mask, (uint64_t)val);
#endif
	return val;
}

template<int Width, int AddrShift> emu::detail::handler_entry_size_t<Width> handler_entry_read_memory<Width, AddrShift>::read_interruptible(offs_t offset, uX mem_mask) const
{
	uX const val = m_base[((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift)];
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
	mame_sgi_fastram_read_trace("read_interruptible", offset, (void *)m_base, this->m_address_base, this->m_address_mask, (uint64_t)val);
#endif
	return val;
}

template<int Width, int AddrShift> std::pair<emu::detail::handler_entry_size_t<Width>, u16> handler_entry_read_memory<Width, AddrShift>::read_flags(offs_t offset, uX mem_mask) const
{
	uX const val = m_base[((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift)];
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
	mame_sgi_fastram_read_trace("read_flags", offset, (void *)m_base, this->m_address_base, this->m_address_mask, (uint64_t)val);
#endif
	return std::pair<uX, u16>(val, this->m_flags);
}

template<int Width, int AddrShift> u16 handler_entry_read_memory<Width, AddrShift>::lookup_flags(offs_t offset, uX mem_mask) const
{
	return this->m_flags;
}

template<int Width, int AddrShift> void *handler_entry_read_memory<Width, AddrShift>::get_ptr(offs_t offset) const
{
	return m_base + (((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift));
}

template<int Width, int AddrShift> std::string handler_entry_read_memory<Width, AddrShift>::name() const
{
	return util::string_format("memory@%x", this->m_address_base);
}


template<int Width, int AddrShift> void handler_entry_write_memory<Width, AddrShift>::write(offs_t offset, uX data, uX mem_mask) const
{
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
	mame_sgi_fastram_write_trace("write", offset, (uint64_t)data, (uint64_t)mem_mask);
#endif
	offs_t off = ((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift);
	m_base[off] = (m_base[off] & ~mem_mask) | (data & mem_mask);
}

template<int Width, int AddrShift> void handler_entry_write_memory<Width, AddrShift>::write_interruptible(offs_t offset, uX data, uX mem_mask) const
{
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
	mame_sgi_fastram_write_trace("write_interruptible", offset, (uint64_t)data, (uint64_t)mem_mask);
#endif
	offs_t off = ((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift);
	m_base[off] = (m_base[off] & ~mem_mask) | (data & mem_mask);
}

template<int Width, int AddrShift> u16 handler_entry_write_memory<Width, AddrShift>::write_flags(offs_t offset, uX data, uX mem_mask) const
{
#ifdef MAME_SGI_FASTRAM_WRITE_TRACE_DIAG
	mame_sgi_fastram_write_trace("write_flags", offset, (uint64_t)data, (uint64_t)mem_mask);
#endif
	offs_t off = ((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift);
	m_base[off] = (m_base[off] & ~mem_mask) | (data & mem_mask);
	return this->m_flags;
}

template<int Width, int AddrShift> u16 handler_entry_write_memory<Width, AddrShift>::lookup_flags(offs_t offset, uX mem_mask) const
{
	return this->m_flags;
}

template<> void handler_entry_write_memory<0, 0>::write(offs_t offset, u8 data, u8 mem_mask) const
{
	m_base[(offset - this->m_address_base) & this->m_address_mask] = data;
}

template<> void handler_entry_write_memory<0, 0>::write_interruptible(offs_t offset, u8 data, u8 mem_mask) const
{
	m_base[(offset - this->m_address_base) & this->m_address_mask] = data;
}

template<> u16 handler_entry_write_memory<0, 0>::write_flags(offs_t offset, u8 data, u8 mem_mask) const
{
	m_base[(offset - this->m_address_base) & this->m_address_mask] = data;
	return this->m_flags;
}

template<int Width, int AddrShift> void *handler_entry_write_memory<Width, AddrShift>::get_ptr(offs_t offset) const
{
	return m_base + (((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift));
}

template<int Width, int AddrShift> std::string handler_entry_write_memory<Width, AddrShift>::name() const
{
	return util::string_format("memory@%x", this->m_address_base);
}





template<int Width, int AddrShift> emu::detail::handler_entry_size_t<Width> handler_entry_read_memory_bank<Width, AddrShift>::read(offs_t offset, uX mem_mask) const
{
	return static_cast<uX *>(m_bank.base())[((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift)];
}

template<int Width, int AddrShift> emu::detail::handler_entry_size_t<Width> handler_entry_read_memory_bank<Width, AddrShift>::read_interruptible(offs_t offset, uX mem_mask) const
{
	return static_cast<uX *>(m_bank.base())[((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift)];
}

template<int Width, int AddrShift> std::pair<emu::detail::handler_entry_size_t<Width>, u16> handler_entry_read_memory_bank<Width, AddrShift>::read_flags(offs_t offset, uX mem_mask) const
{
	return std::pair<uX, u16>(static_cast<uX *>(m_bank.base())[((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift)], this->m_flags);
}

template<int Width, int AddrShift> u16 handler_entry_read_memory_bank<Width, AddrShift>::lookup_flags(offs_t offset, uX mem_mask) const
{
	return this->m_flags;
}

template<int Width, int AddrShift> void *handler_entry_read_memory_bank<Width, AddrShift>::get_ptr(offs_t offset) const
{
	return static_cast<uX *>(m_bank.base()) + (((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift));
}

template<int Width, int AddrShift> std::string handler_entry_read_memory_bank<Width, AddrShift>::name() const
{
	return m_bank.name();
}


template<int Width, int AddrShift> void handler_entry_write_memory_bank<Width, AddrShift>::write(offs_t offset, uX data, uX mem_mask) const
{
	offs_t off = ((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift);
	static_cast<uX *>(m_bank.base())[off] = (static_cast<uX *>(m_bank.base())[off] & ~mem_mask) | (data & mem_mask);
}

template<int Width, int AddrShift> void handler_entry_write_memory_bank<Width, AddrShift>::write_interruptible(offs_t offset, uX data, uX mem_mask) const
{
	offs_t off = ((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift);
	static_cast<uX *>(m_bank.base())[off] = (static_cast<uX *>(m_bank.base())[off] & ~mem_mask) | (data & mem_mask);
}

template<int Width, int AddrShift> u16 handler_entry_write_memory_bank<Width, AddrShift>::write_flags(offs_t offset, uX data, uX mem_mask) const
{
	offs_t off = ((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift);
	static_cast<uX *>(m_bank.base())[off] = (static_cast<uX *>(m_bank.base())[off] & ~mem_mask) | (data & mem_mask);
	return this->m_flags;
}

template<int Width, int AddrShift> u16 handler_entry_write_memory_bank<Width, AddrShift>::lookup_flags(offs_t offset, uX mem_mask) const
{
	return this->m_flags;
}

template<> void handler_entry_write_memory_bank<0, 0>::write(offs_t offset, u8 data, u8 mem_mask) const
{
	static_cast<uX *>(m_bank.base())[(offset - this->m_address_base) & this->m_address_mask] = data;
}

template<> void handler_entry_write_memory_bank<0, 0>::write_interruptible(offs_t offset, u8 data, u8 mem_mask) const
{
	static_cast<uX *>(m_bank.base())[(offset - this->m_address_base) & this->m_address_mask] = data;
}

template<> u16 handler_entry_write_memory_bank<0, 0>::write_flags(offs_t offset, u8 data, u8 mem_mask) const
{
	static_cast<uX *>(m_bank.base())[(offset - this->m_address_base) & this->m_address_mask] = data;
	return this->m_flags;
}

template<int Width, int AddrShift> void *handler_entry_write_memory_bank<Width, AddrShift>::get_ptr(offs_t offset) const
{
	return static_cast<uX *>(m_bank.base()) + (((offset - this->m_address_base) & this->m_address_mask) >> (Width + AddrShift));
}

template<int Width, int AddrShift> std::string handler_entry_write_memory_bank<Width, AddrShift>::name() const
{
	return m_bank.name();
}



template class handler_entry_read_memory<0,  1>;
template class handler_entry_read_memory<0,  0>;
template class handler_entry_read_memory<1,  3>;
template class handler_entry_read_memory<1,  0>;
template class handler_entry_read_memory<1, -1>;
template class handler_entry_read_memory<2,  3>;
template class handler_entry_read_memory<2,  0>;
template class handler_entry_read_memory<2, -1>;
template class handler_entry_read_memory<2, -2>;
template class handler_entry_read_memory<3,  0>;
template class handler_entry_read_memory<3, -1>;
template class handler_entry_read_memory<3, -2>;
template class handler_entry_read_memory<3, -3>;

template class handler_entry_write_memory<0,  1>;
template class handler_entry_write_memory<0,  0>;
template class handler_entry_write_memory<1,  3>;
template class handler_entry_write_memory<1,  0>;
template class handler_entry_write_memory<1, -1>;
template class handler_entry_write_memory<2,  3>;
template class handler_entry_write_memory<2,  0>;
template class handler_entry_write_memory<2, -1>;
template class handler_entry_write_memory<2, -2>;
template class handler_entry_write_memory<3,  0>;
template class handler_entry_write_memory<3, -1>;
template class handler_entry_write_memory<3, -2>;
template class handler_entry_write_memory<3, -3>;


template class handler_entry_read_memory_bank<0,  1>;
template class handler_entry_read_memory_bank<0,  0>;
template class handler_entry_read_memory_bank<1,  3>;
template class handler_entry_read_memory_bank<1,  0>;
template class handler_entry_read_memory_bank<1, -1>;
template class handler_entry_read_memory_bank<2,  3>;
template class handler_entry_read_memory_bank<2,  0>;
template class handler_entry_read_memory_bank<2, -1>;
template class handler_entry_read_memory_bank<2, -2>;
template class handler_entry_read_memory_bank<3,  0>;
template class handler_entry_read_memory_bank<3, -1>;
template class handler_entry_read_memory_bank<3, -2>;
template class handler_entry_read_memory_bank<3, -3>;

template class handler_entry_write_memory_bank<0,  1>;
template class handler_entry_write_memory_bank<0,  0>;
template class handler_entry_write_memory_bank<1,  3>;
template class handler_entry_write_memory_bank<1,  0>;
template class handler_entry_write_memory_bank<1, -1>;
template class handler_entry_write_memory_bank<2,  3>;
template class handler_entry_write_memory_bank<2,  0>;
template class handler_entry_write_memory_bank<2, -1>;
template class handler_entry_write_memory_bank<2, -2>;
template class handler_entry_write_memory_bank<3,  0>;
template class handler_entry_write_memory_bank<3, -1>;
template class handler_entry_write_memory_bank<3, -2>;
template class handler_entry_write_memory_bank<3, -3>;
