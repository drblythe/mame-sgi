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
