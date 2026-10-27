// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_SGI_VC1_H
#define MAME_SGI_VC1_H

#pragma once

class sgi_vc1_device
	: public device_t
{
public:
	sgi_vc1_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock);

	u8 read(offs_t offset);
	void write(offs_t offset, u8 data);

	// cursor generator state, for the board's screen update
	u8 sysctl() const { return m_sysctl; }
	// display id: the frame table at DID_EP holds one line table pointer
	// per scan line; the line's first entry gives its display id, and the
	// display id's mode (two bytes each in the xmap table) selects the
	// RAMDAC palette in bits 2-3 of its second byte
	u16 did_ep() const { return u16(m_reg[0x40]) << 8 | m_reg[0x41]; }
	int line_palette(unsigned line) const
	{
		u16 const ep = did_ep();
		if (!ep)
			return -1;
		u16 const lp = u16(ram(ep + line * 2)) << 8 | ram(ep + line * 2 + 1);
		unsigned const did = ram(lp + 3) & 0x1f;
		return BIT(m_xmap[did * 2 + 1], 2, 2);
	}
	bool video_enabled() const { return m_sysctl & 0x04; } // cleared to blank the screen
	u16 cursor_ep() const { return u16(m_reg[0x20]) << 8 | m_reg[0x21]; }
	u16 cursor_x() const { return u16(m_reg[0x22]) << 8 | m_reg[0x23]; }
	u16 cursor_y() const { return u16(m_reg[0x24]) << 8 | m_reg[0x25]; }
	u16 cursor_mode() const { return u16(m_reg[0x26]) << 8 | m_reg[0x27]; }
	u8 ram(u16 address) const { return m_ram[address & 0x7fffU]; }

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

private:
	u16 m_addr;
	u8 m_reg[128];
	u8 m_xmap[128];
	u8 m_sysctl;

	std::unique_ptr<u8[]> m_ram;
};

DECLARE_DEVICE_TYPE(SGI_VC1, sgi_vc1_device)

#endif // MAME_SGI_VC1_H
