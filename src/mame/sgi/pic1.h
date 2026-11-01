// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_SGI_PIC1_H
#define MAME_SGI_PIC1_H

#pragma once

class sgi_pic1_device
	: public device_t
{
public:
	sgi_pic1_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);

	template <typename T> void set_bus(T &&tag, int spacenum) { m_bus.set_tag(std::forward<T>(tag), spacenum); }

	void map(address_map &map) ATTR_COLD;

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

	// read handlers
	u32 cpucfg_r() { return m_cpucfg; }
	u32 sid_r() { return m_sid; }
	u32 memcfg_r(offs_t offset) { return m_memcfg[offset]; }
	u32 dabr_r() { return m_dabr; }

	// write handlers
	void cpucfg_w(u32 data) { m_cpucfg = data; }
	void memcfg_w(offs_t offset, u32 data);
	void parcl_ws(u32 data);
	void dabr_w(u32 data) { m_dabr = data; }

	// helpers
	unsigned ram_size(unsigned bank) const;
	void unmap_banks(unsigned offset);
	void map_banks(unsigned offset);

private:
	required_address_space m_bus;
	required_ioport m_simms;

	// Each bank is allocated once at its largest size (4x8M SIMMs), so it
	// can be registered for save states; calloc() leaves the part a smaller
	// configuration never uses non-resident.
	struct calloc_deleter { void operator()(u8 *p) const noexcept { std::free(p); } };
	std::unique_ptr<u8[], calloc_deleter> m_ram[4];

	// bus ranges currently mapped to each bank (size 0 when unmapped)
	u32 m_mapped_base[4];
	u32 m_mapped_size[4];

	u32 m_cpucfg;    // cpu board control and status
	u32 m_sid;       // system id/coprocessor present
	u32 m_memcfg[2]; // memory configuration
	u32 m_dabr;      // descriptor array base
};

DECLARE_DEVICE_TYPE(SGI_PIC1, sgi_pic1_device)

#endif // MAME_SGI_PIC1_H
