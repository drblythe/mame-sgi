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
	// display ids: the frame table at DID_EP holds one line list pointer
	// per scan line. A list is a count followed by that many 16-bit entries,
	// in x order, each the x where a display id starts (pixels, bits 5-15)
	// and the id (bits 0-4); the X server gives windows with their own
	// colour map their own id. The entries increase in x; one that doesn't
	// ends the list (software pads lists with entries the VC1 reads ahead).
	// An id's mode (two bytes each in the xmap table) selects the RAMDAC
	// palette in bits 2-3 of its second byte. Returns the line's runs as
	// start x and palette (at most max), or 0 without display id tables
	u16 did_ep() const { return u16(m_reg[0x40]) << 8 | m_reg[0x41]; }
	unsigned line_runs(unsigned line, u16 *x, u8 *palette, unsigned max) const
	{
		u16 const ep = did_ep();
		if (!ep)
			return 0;
		u16 const lp = ram16(ep + line * 2);
		unsigned const count = std::min<unsigned>(ram16(lp), max);
		unsigned runs = 0;
		for (unsigned i = 0; i < count; i++)
		{
			u16 const entry = ram16(lp + 2 + i * 2);
			if (runs && (entry >> 5) <= x[runs - 1])
				break;
			x[runs] = entry >> 5;
			palette[runs++] = BIT(m_xmap[(entry & 0x1f) * 2 + 1], 2, 2);
		}
		return runs;
	}
	bool video_enabled() const { return m_sysctl & 0x04; } // cleared to blank the screen
	// xmap control: the colour map (bits 4-5) and submap (bits 0-3) the
	// pop-up planes display through
	u8 auxvideo_map() const { return m_reg[0x62]; }
	u16 cursor_ep() const { return u16(m_reg[0x20]) << 8 | m_reg[0x21]; }
	u16 cursor_x() const { return u16(m_reg[0x22]) << 8 | m_reg[0x23]; }
	u16 cursor_y() const { return u16(m_reg[0x24]) << 8 | m_reg[0x25]; }
	u16 cursor_mode() const { return u16(m_reg[0x26]) << 8 | m_reg[0x27]; }
	u8 ram(u16 address) const { return m_ram[address & 0x7fffU]; }
	u16 ram16(u16 address) const { return u16(ram(address)) << 8 | ram(address + 1); }

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
