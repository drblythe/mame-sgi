// license:BSD-3-Clause
// copyright-holders:Ryan Holtz, Tyson Smith
/*
        Silicon Graphics LG1 "Light" graphics board used as
        entry level graphics in the Indigo and IRIS Crimson.
*/

/*
 * TODO:
 *  - devicify rex
 *  - slotify
 *  - later board revisions and LG2X2 dual-head variant
 *  - popup planes, VC1 display id modes beyond the palette select,
 *    command bit 11, color compare modes other than aux1 bit 8
 *
 * WIP:
 *  - passes power-on and field diagnostic graphics tests
 *  - usable for firmware graphic output
 *  - runs the IRIX desktop, with drawing errors
 */

#include "emu.h"
#include "light.h"

#define LOG_REX (1U << 1)

//#define VERBOSE (LOG_GENERAL|LOG_REX)
#include "logmacro.h"

static constexpr uint32_t x_res = 1024;
static constexpr uint32_t y_res = 768;

// VC1 cursor position of the screen's top left pixel
static constexpr int CURSOR_X_OFFSET = 140;
static constexpr int CURSOR_Y_OFFSET = 39;

enum
{
	REX15_OP_NOP                = 0x00000000,
	REX15_OP_DRAW               = 0x00000001,
	REX15_OP_LDPIXEL            = 0x00000003,
	REX15_OP_FLAG_BLOCK         = 0x00000008,
	REX15_OP_FLAG_LENGTH32      = 0x00000010,
	REX15_OP_FLAG_QUADMODE      = 0x00000020,
	REX15_OP_FLAG_XYCONTINUE    = 0x00000080,
	REX15_OP_FLAG_STOPONX       = 0x00000100,
	REX15_OP_FLAG_STOPONY       = 0x00000200,
	REX15_OP_FLAG_ENZPATTERN    = 0x00000400,
	REX15_OP_FLAG_RGBMODE       = 0x00004000,
	REX15_OP_FLAG_COLORCOMP     = 0x00010000,
	REX15_OP_FLAG_LOGICSRC      = 0x00080000,
	REX15_OP_FLAG_COLORAUX      = 0x00200000,
	REX15_OP_FLAG_ENLSPATTERN   = 0x00400000,
	REX15_OP_FLAG_ZOPAQUE       = 0x00800000,
	REX15_OP_FLAG_ZCONTINUE     = 0x01000000,
};

DEFINE_DEVICE_TYPE(SGI_LG1, sgi_lg1_device, "sgi_lg1", "SGI LG1")

sgi_lg1_device::sgi_lg1_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: device_t(mconfig, SGI_LG1, tag, owner, clock)
	, m_vc1(*this, "vc1")
	, m_lut1(*this, "lut1")
	, m_screen(*this, "screen")
{
}

void sgi_lg1_device::device_add_mconfig(machine_config &config)
{
	SGI_VC1(config, m_vc1, 64_MHz_XTAL);

	// TODO: does LUT1 differ from Bt479?
	BT479(config, m_lut1, 64_MHz_XTAL);

	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	// TODO: video timing from VC1
	m_screen->set_raw(1024 * 768 * 60, 1024, 0, 1024, 768, 0, 768);
	m_screen->set_screen_update(FUNC(sgi_lg1_device::screen_update));
}

void sgi_lg1_device::device_start()
{
	m_vram = std::make_unique<u8[]>(0x10'0000); // 8xTC524256BJ-10 262,144x4 multiport dram
	m_pup = std::make_unique<u8[]>(0x10'0000);
	m_cid = std::make_unique<u8[]>(0x10'0000);

	m_command = m_aux1 = m_xstate = m_xstart = m_ystart = m_xymove = 0;
	std::fill(std::begin(m_color), std::end(m_color), 0);
	std::fill(std::begin(m_slopecolor), std::end(m_slopecolor), 0);
	m_colorback = m_zpattern = m_lspattern = m_lsmode = m_aweight = 0;
	m_rwaux1 = m_rwaux2 = m_rwmask = m_xend = m_yend = m_xsave = m_minorslope = 0;
	std::fill(std::begin(m_smask), std::end(m_smask), 0);
	m_aux2 = m_diagvram = m_diagcid = m_wclock = m_configsel = 0;
	m_togglectxt = m_configmode = 0;
	m_xcurri = m_ycurri = 0;
	m_data = 0;

	save_pointer(NAME(m_vram), 0x10'0000);
	save_pointer(NAME(m_pup), 0x10'0000);
	save_pointer(NAME(m_cid), 0x10'0000);
}

void sgi_lg1_device::device_reset()
{
	m_wrmask = 0xff;
	m_logicop = 3; // source
	m_xywin = 0x0800'0800; // window origin 0x800, 0x800: no offset
	m_aux2_source = false;
}

void sgi_lg1_device::map(address_map &map)
{
	map(0x0000, 0x0003).select(0x0800).rw(FUNC(sgi_lg1_device::command_r), FUNC(sgi_lg1_device::command_w));
	map(0x0004, 0x0007).select(0x0800).rw(FUNC(sgi_lg1_device::aux1_r), FUNC(sgi_lg1_device::aux1_w));
	map(0x0008, 0x000b).select(0x0800).rw(FUNC(sgi_lg1_device::xstate_r), FUNC(sgi_lg1_device::xstate_w));
	map(0x000c, 0x000f).select(0x0800).rw(FUNC(sgi_lg1_device::xstarti_r), FUNC(sgi_lg1_device::xstarti_w));
	map(0x0010, 0x0013).select(0x0800).rw(FUNC(sgi_lg1_device::xstartf_r), FUNC(sgi_lg1_device::xstartf_w));
	map(0x0014, 0x0017).select(0x0800).rw(FUNC(sgi_lg1_device::xstart_r), FUNC(sgi_lg1_device::xstart_w));
	map(0x0018, 0x001b).select(0x0800).rw(FUNC(sgi_lg1_device::xendf_r), FUNC(sgi_lg1_device::xendf_w));
	map(0x001c, 0x001f).select(0x0800).rw(FUNC(sgi_lg1_device::ystarti_r), FUNC(sgi_lg1_device::ystarti_w));
	map(0x0020, 0x0023).select(0x0800).rw(FUNC(sgi_lg1_device::ystartf_r), FUNC(sgi_lg1_device::ystartf_w));
	map(0x0024, 0x0027).select(0x0800).rw(FUNC(sgi_lg1_device::ystart_r), FUNC(sgi_lg1_device::ystart_w));
	map(0x0028, 0x002b).select(0x0800).rw(FUNC(sgi_lg1_device::yendf_r), FUNC(sgi_lg1_device::yendf_w));
	map(0x002c, 0x002f).select(0x0800).rw(FUNC(sgi_lg1_device::xsave_r), FUNC(sgi_lg1_device::xsave_w));
	map(0x0030, 0x0033).select(0x0800).rw(FUNC(sgi_lg1_device::minorslope_r), FUNC(sgi_lg1_device::minorslope_w));
	map(0x0034, 0x0037).select(0x0800).rw(FUNC(sgi_lg1_device::xymove_r), FUNC(sgi_lg1_device::xymove_w));
	map(0x0038, 0x003b).select(0x0800).rw(FUNC(sgi_lg1_device::colori_r<0>), FUNC(sgi_lg1_device::colori_w<0>));
	map(0x003c, 0x003f).select(0x0800).rw(FUNC(sgi_lg1_device::colorf_r<0>), FUNC(sgi_lg1_device::colorf_w<0>));
	map(0x0040, 0x0043).select(0x0800).rw(FUNC(sgi_lg1_device::colori_r<1>), FUNC(sgi_lg1_device::colori_w<1>));
	map(0x0044, 0x0047).select(0x0800).rw(FUNC(sgi_lg1_device::colorf_r<1>), FUNC(sgi_lg1_device::colorf_w<1>));
	map(0x0048, 0x004b).select(0x0800).rw(FUNC(sgi_lg1_device::colori_r<2>), FUNC(sgi_lg1_device::colori_w<2>));
	map(0x004c, 0x004f).select(0x0800).rw(FUNC(sgi_lg1_device::colorf_r<2>), FUNC(sgi_lg1_device::colorf_w<2>));
	map(0x0050, 0x0053).select(0x0800).rw(FUNC(sgi_lg1_device::slopecolor_r<0>), FUNC(sgi_lg1_device::slopecolor_w<0>));
	map(0x0054, 0x0057).select(0x0800).rw(FUNC(sgi_lg1_device::slopecolor_r<1>), FUNC(sgi_lg1_device::slopecolor_w<1>));
	map(0x0058, 0x005b).select(0x0800).rw(FUNC(sgi_lg1_device::slopecolor_r<2>), FUNC(sgi_lg1_device::slopecolor_w<2>));
	map(0x005c, 0x005f).select(0x0800).rw(FUNC(sgi_lg1_device::colorback_r), FUNC(sgi_lg1_device::colorback_w));
	map(0x0060, 0x0063).select(0x0800).rw(FUNC(sgi_lg1_device::zpattern_r), FUNC(sgi_lg1_device::zpattern_w));
	map(0x0064, 0x0067).select(0x0800).rw(FUNC(sgi_lg1_device::lspattern_r), FUNC(sgi_lg1_device::lspattern_w));
	map(0x0068, 0x006b).select(0x0800).rw(FUNC(sgi_lg1_device::lsmode_r), FUNC(sgi_lg1_device::lsmode_w));
	map(0x006c, 0x006f).select(0x0800).rw(FUNC(sgi_lg1_device::aweight_r), FUNC(sgi_lg1_device::aweight_w));
	map(0x0070, 0x0073).select(0x0800).rw(FUNC(sgi_lg1_device::rwaux1_r), FUNC(sgi_lg1_device::rwaux1_w));
	map(0x0074, 0x0077).select(0x0800).rw(FUNC(sgi_lg1_device::rwaux2_r), FUNC(sgi_lg1_device::rwaux2_w));
	map(0x0078, 0x007b).select(0x0800).rw(FUNC(sgi_lg1_device::rwmask_r), FUNC(sgi_lg1_device::rwmask_w));
	map(0x007c, 0x007f).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<0>), FUNC(sgi_lg1_device::smask_w<0>));
	map(0x0080, 0x0083).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<1>), FUNC(sgi_lg1_device::smask_w<1>));
	map(0x0084, 0x0087).select(0x0800).rw(FUNC(sgi_lg1_device::xendi_r), FUNC(sgi_lg1_device::xendi_w));
	map(0x0088, 0x008b).select(0x0800).rw(FUNC(sgi_lg1_device::yendi_r), FUNC(sgi_lg1_device::yendi_w));

	// any access to a register through the GO page also starts the command
	map(0x0800, 0x088b).rw(FUNC(sgi_lg1_device::go_r), FUNC(sgi_lg1_device::go_w));

	// configuration registers
	map(0x4790, 0x4793).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<2>), FUNC(sgi_lg1_device::smask_w<2>));
	map(0x4794, 0x4797).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<3>), FUNC(sgi_lg1_device::smask_w<3>));
	map(0x4798, 0x479b).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<4>), FUNC(sgi_lg1_device::smask_w<4>));
	map(0x479c, 0x479f).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<5>), FUNC(sgi_lg1_device::smask_w<5>));
	map(0x47a0, 0x47a3).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<6>), FUNC(sgi_lg1_device::smask_w<6>));
	map(0x47a4, 0x47a7).select(0x0800).rw(FUNC(sgi_lg1_device::smask_r<7>), FUNC(sgi_lg1_device::smask_w<7>));
	map(0x47a8, 0x47ab).select(0x0800).rw(FUNC(sgi_lg1_device::aux2_r), FUNC(sgi_lg1_device::aux2_w));

	map(0x47d8, 0x47db).select(0x0800).rw(FUNC(sgi_lg1_device::diagvram_r), FUNC(sgi_lg1_device::diagvram_w));
	map(0x47dc, 0x47df).select(0x0800).rw(FUNC(sgi_lg1_device::diagcid_r), FUNC(sgi_lg1_device::diagcid_w));

	map(0x47e4, 0x47e7).select(0x0800).rw(FUNC(sgi_lg1_device::wclock_r), FUNC(sgi_lg1_device::wclock_w));
	map(0x47e8, 0x47eb).select(0x0800).rw(FUNC(sgi_lg1_device::dac_r), FUNC(sgi_lg1_device::dac_w));
	map(0x47ec, 0x47ef).select(0x0800).rw(FUNC(sgi_lg1_device::configsel_r), FUNC(sgi_lg1_device::configsel_w));
	map(0x47f0, 0x47f3).select(0x0800).rw(FUNC(sgi_lg1_device::vc1_r), FUNC(sgi_lg1_device::vc1_w));
	map(0x47f4, 0x47f7).select(0x0800).rw(FUNC(sgi_lg1_device::togglectxt_r), FUNC(sgi_lg1_device::togglectxt_w));
	map(0x47f8, 0x47fb).select(0x0800).rw(FUNC(sgi_lg1_device::configmode_r), FUNC(sgi_lg1_device::configmode_w));
	map(0x47fc, 0x47ff).select(0x0800).rw(FUNC(sgi_lg1_device::xywin_r), FUNC(sgi_lg1_device::xywin_w));
}

u32 sgi_lg1_device::screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect)
{
	// the PROM and IRIX blank the screen through the VC1
	if (!m_vc1->video_enabled())
	{
		bitmap.fill(rgb_t::black(), cliprect);
		return 0;
	}

	u8 const *src = &m_vram[0];
	u8 const *pup = &m_pup[0];
	// pop-up plane values 1-3 show the colours of the VC1's aux video map;
	// 0 is transparent
	pen_t const *const pup_pens = m_lut1->palette_pens(BIT(m_vc1->auxvideo_map(), 4, 2)) + BIT(m_vc1->auxvideo_map(), 0, 4) * 0x10;

	for (unsigned y = screen.visible_area().min_y; y <= screen.visible_area().max_y; y++)
	{
		// the VC1 selects each line's palette through its display id;
		// without display id tables, use the palette the RAMDAC selects
		int const palette = m_vc1->line_palette(y);
		pen_t const *const pens = m_lut1->palette_pens(palette);
		u8 const mask = m_lut1->read_mask();
		u32 *dst = &bitmap.pix(y, screen.visible_area().min_x);
		for (unsigned x = screen.visible_area().min_x; x <= screen.visible_area().max_x; x++, src++, pup++)
			*dst++ = *pup ? pup_pens[*pup] : pens[*src & mask];
	}

	// VC1 cursor: 32x32, two planes of 4 bytes per row (most significant
	// bit leftmost) in the VC1's SRAM at CUR_EP and CUR_EP + 0x80; the
	// planes form an index (0 is transparent) into the colour map and
	// submap the cursor mode register selects
	{
		u16 const ep = m_vc1->cursor_ep();
		int const cx = int(m_vc1->cursor_x()) - CURSOR_X_OFFSET;
		int const cy = int(m_vc1->cursor_y()) - CURSOR_Y_OFFSET;
		unsigned const cursor_pens = BIT(m_vc1->cursor_mode(), 14, 2) * 0x100 + BIT(m_vc1->cursor_mode(), 10, 4) * 0x10;

		for (int row = 0; row < 32; row++)
		{
			int const y = cy + row;
			if (y < cliprect.min_y || y > cliprect.max_y)
				continue;

			u32 const p0 = u32(m_vc1->ram(ep + row * 4 + 0)) << 24 | u32(m_vc1->ram(ep + row * 4 + 1)) << 16 | u32(m_vc1->ram(ep + row * 4 + 2)) << 8 | m_vc1->ram(ep + row * 4 + 3);
			u32 const p1 = u32(m_vc1->ram(ep + 0x80 + row * 4 + 0)) << 24 | u32(m_vc1->ram(ep + 0x80 + row * 4 + 1)) << 16 | u32(m_vc1->ram(ep + 0x80 + row * 4 + 2)) << 8 | m_vc1->ram(ep + 0x80 + row * 4 + 3);
			if (!(p0 | p1))
				continue;

			for (int col = 0; col < 32; col++)
			{
				int const x = cx + col;
				unsigned const index = BIT(p1, 31 - col) << 1 | BIT(p0, 31 - col);
				if (index && x >= cliprect.min_x && x <= cliprect.max_x)
					bitmap.pix(y, x) = m_lut1->pen_color(cursor_pens + index);
			}
		}
	}

	return 0;
}

void sgi_lg1_device::minorslope_w(u32 data)
{
	if (BIT(data, 31))
		data = ~(data & 0x8000'ffffU) + 1;

	m_minorslope = data & 0x1'ffffU;
}

template <unsigned Color> void sgi_lg1_device::slopecolor_w(u32 data)
{
	if (BIT(data, 31))
		data = ~(data & 0x800f'ffffU) + 1;

	m_slopecolor[Color] = ((data & 0x0008'0000U) << 1) | (data & 0x001f'ffffU);
}

/*
 * This logic assumes that there is an 8 bit latch used to interface to the
 * LUT1 and VC1. The GO signal controls whether the latch is read or written
 * by the host or by the addressed device.
 */

u32 sgi_lg1_device::dac_r(offs_t offset)
{
	if (offset & 0x200)
		m_data = m_lut1->read(m_configsel);

	return m_data;
}

void sgi_lg1_device::dac_w(offs_t offset, u32 data)
{
	// a host write loads the latch; a write to the GO address then also
	// passes it to the device (software often writes only the GO address)
	m_data = data;
	if (offset & 0x200)
		m_lut1->write(m_configsel, m_data);
}

u32 sgi_lg1_device::vc1_r(offs_t offset)
{
	if (offset & 0x200)
		m_data = m_vc1->read(m_configsel);

	return m_data;
}

void sgi_lg1_device::vc1_w(offs_t offset, u32 data)
{
	// a host write loads the latch; a write to the GO address then also
	// passes it to the device (software often writes only the GO address)
	m_data = data;
	if (offset & 0x200)
		m_vc1->write(m_configsel, m_data);
}

void sgi_lg1_device::rex_reg_w(unsigned reg, u32 data)
{
	switch (reg)
	{
	case 0x00: command_w(0, data); break;
	case 0x04: aux1_w(data); break;
	case 0x08: xstate_w(data); break;
	case 0x0c: xstarti_w(data); break;
	case 0x10: xstartf_w(data); break;
	case 0x14: xstart_w(data); break;
	case 0x18: xendf_w(data); break;
	case 0x1c: ystarti_w(data); break;
	case 0x20: ystartf_w(data); break;
	case 0x24: ystart_w(data); break;
	case 0x28: yendf_w(data); break;
	case 0x2c: xsave_w(data); break;
	case 0x30: minorslope_w(data); break;
	case 0x34: xymove_w(0, data); break;
	case 0x38: colori_w<0>(0, data); break;
	case 0x3c: colorf_w<0>(data); break;
	case 0x40: colori_w<1>(0, data); break;
	case 0x44: colorf_w<1>(data); break;
	case 0x48: colori_w<2>(0, data); break;
	case 0x4c: colorf_w<2>(data); break;
	case 0x50: slopecolor_w<0>(data); break;
	case 0x54: slopecolor_w<1>(data); break;
	case 0x58: slopecolor_w<2>(data); break;
	case 0x5c: colorback_w(data); break;
	case 0x60: zpattern_w(0, data); break;
	case 0x64: lspattern_w(data); break;
	case 0x68: lsmode_w(data); break;
	case 0x6c: aweight_w(data); break;
	case 0x70: rwaux1_w(data); break;
	case 0x74: rwaux2_w(data); break;
	case 0x78: rwmask_w(data); break;
	case 0x7c: smask_w<0>(data); break;
	case 0x80: smask_w<1>(data); break;
	case 0x84: xendi_w(0, data); break;
	case 0x88: yendi_w(0, data); break;
	}
}

u32 sgi_lg1_device::rex_reg_r(unsigned reg)
{
	switch (reg)
	{
	case 0x00: return m_command;
	case 0x04: return aux1_r();
	case 0x08: return xstate_r();
	case 0x0c: return xstarti_r();
	case 0x10: return xstartf_r();
	case 0x14: return xstart_r();
	case 0x18: return xendf_r();
	case 0x1c: return ystarti_r();
	case 0x20: return ystartf_r();
	case 0x24: return ystart_r();
	case 0x28: return yendf_r();
	case 0x2c: return xsave_r();
	case 0x30: return minorslope_r();
	case 0x34: return xymove_r();
	case 0x38: return colori_r<0>();
	case 0x3c: return colorf_r<0>();
	case 0x40: return colori_r<1>();
	case 0x44: return colorf_r<1>();
	case 0x48: return colori_r<2>();
	case 0x4c: return colorf_r<2>();
	case 0x50: return slopecolor_r<0>();
	case 0x54: return slopecolor_r<1>();
	case 0x58: return slopecolor_r<2>();
	case 0x5c: return colorback_r();
	case 0x60: return zpattern_r();
	case 0x64: return lspattern_r();
	case 0x68: return lsmode_r();
	case 0x6c: return aweight_r();
	case 0x70: return rwaux1_r();
	case 0x74: return rwaux2_r();
	case 0x78: return rwmask_r();
	case 0x7c: return smask_r<0>();
	case 0x80: return smask_r<1>();
	case 0x84: return xendi_r();
	case 0x88: return yendi_r();
	}
	return 0;
}

void sgi_lg1_device::go_w(offs_t offset, u32 data)
{
	rex_reg_w(offset << 2, data);
	if (!machine().side_effects_disabled())
	{
		// pixel data written through rwaux2 (the second half of a 64-bit
		// DMA word) supplies the next four pixels
		m_aux2_source = (offset << 2) == 0x74;
		do_rex_command();
		m_aux2_source = false;
	}
}

u32 sgi_lg1_device::go_r(offs_t offset)
{
	if (!machine().side_effects_disabled())
		do_rex_command();
	return rex_reg_r(offset << 2);
}

// read one pixel, in window coordinates
u8 sgi_lg1_device::vram_r(int x, int y) const
{
	x += xywin_x();
	y += xywin_y();
	return (x >= 0 && x < int(x_res) && y >= 0 && y < 1024) ? plane_buffer()[y * x_res + x] : 0;
}

bool sgi_lg1_device::smask_pass(int x, int y) const
{
	unsigned const enables = BIT(m_aux2, 8, 4);
	if (!enables)
		return true;

	for (unsigned n = 0; n < 4; n++)
	{
		if (!BIT(enables, n))
			continue;
		u32 const mx = m_smask[n * 2];
		u32 const my = m_smask[n * 2 + 1];
		if (x >= int(BIT(mx, 0, 10)) && x <= int(BIT(mx, 16, 10)) && y >= int(BIT(my, 0, 10)) && y <= int(BIT(my, 16, 10)))
			return true;
	}
	return false;
}

// write one pixel through the logic operation and the write mask
void sgi_lg1_device::plot(int x, int y, u8 src)
{
	x += xywin_x();
	y += xywin_y();
	if (x < 0 || x >= int(x_res) || y < 0 || y >= 1024)
		return;
	if (!smask_pass(x, y))
		return;

	u8 &pixel = plane_buffer()[y * x_res + x];
	u8 const dst = pixel;
	u8 result;
	switch (m_logicop & 15)
	{
	case 0:  result = 0; break;
	case 1:  result = src & dst; break;
	case 2:  result = src & ~dst; break;
	default: result = src; break;
	case 4:  result = ~src & dst; break;
	case 5:  result = dst; break;
	case 6:  result = src ^ dst; break;
	case 7:  result = src | dst; break;
	case 8:  result = ~(src | dst); break;
	case 9:  result = ~(src ^ dst); break;
	case 10: result = ~dst; break;
	case 11: result = src | ~dst; break;
	case 12: result = ~src; break;
	case 13: result = ~src | dst; break;
	case 14: result = ~(src & dst); break;
	case 15: result = 0xff; break;
	}
	u8 const wrmask = m_wrmask & plane_mask();
	pixel = (dst & ~wrmask) | (result & wrmask);
}

/*
 * The command register holds the opcode, mode flags and logic operation;
 * any access through the GO page runs it. Drawing steps from the start
 * towards the end coordinates in each axis (software orders them for the
 * direction it needs). Without BLOCK and STOPONX a draw writes one pixel;
 * with STOPONX it runs to the end of the span, and with BLOCK it then
 * wraps to the next row until the end row. Pattern draws use one 32-bit
 * zpattern word per GO (most significant bit first) and stop at the end of
 * a row; XYCONTINUE carries on from where the previous GO stopped.
 */
void sgi_lg1_device::do_rex_command()
{
	u32 const cmd = m_command;

	switch (cmd & 3)
	{
	case REX15_OP_NOP:
		m_xcurri = xstarti_r();
		m_ycurri = ystarti_r();
		return;

	case REX15_OP_LDPIXEL:
		{
			// pixels are returned leftmost in the high byte: four in quad
			// mode, otherwise one in bits 24-31
			int x = (cmd & REX15_OP_FLAG_XYCONTINUE) ? int(m_xcurri) : int(xstarti_r());
			int const y = (cmd & REX15_OP_FLAG_XYCONTINUE) ? int(m_ycurri) : int(ystarti_r());
			unsigned const count = (cmd & REX15_OP_FLAG_QUADMODE) ? 4 : 1;
			u32 data = 0;
			for (unsigned n = 0; n < count; n++, x++)
				data = (data << 8) | vram_r(x, y);
			m_rwaux1 = data << (8 * (4 - count));
			m_xcurri = x;
			m_ycurri = y;
		}
		return;

	case REX15_OP_DRAW:
		break;

	default:
		LOGMASKED(LOG_REX, "%s: unknown command: %08x\n", machine().describe_context(), cmd);
		return;
	}

	int const x0 = int(xstarti_r());
	int const y0 = int(ystarti_r());
	int const x1 = int(xendi_r());
	int const y1 = int(yendi_r());
	int const dx = (x1 >= x0) ? 1 : -1;
	int const dy = (y1 >= y0) ? 1 : -1;

	bool const pattern = cmd & REX15_OP_FLAG_ENZPATTERN;
	bool const stipple = cmd & REX15_OP_FLAG_ENLSPATTERN;
	bool const block = cmd & REX15_OP_FLAG_BLOCK;
	bool const span = block || (cmd & REX15_OP_FLAG_STOPONX);
	// host pixel data: each GO with XYCONTINUE supplies the next four
	// pixels; without it, the four repeat to the end of the span
	bool const hostdata = (cmd & REX15_OP_FLAG_COLORAUX) && (cmd & REX15_OP_FLAG_XYCONTINUE);
	u32 const auxdata = m_aux2_source ? m_rwaux2 : m_rwaux1;
	unsigned const limit = (pattern || (cmd & REX15_OP_FLAG_LENGTH32)) ? 32 : hostdata ? ((cmd & REX15_OP_FLAG_QUADMODE) ? 4 : 1) : (x_res * 1024);

	// block move offset from the destination to the source: signed 11-bit
	// x in bits 16-26 and y in bits 0-10
	int const mx = util::sext(m_xymove >> 16, 11);
	int const my = util::sext(m_xymove, 11);
	// RGB mode packs the red, green and blue integer parts into a 3-3-2
	// pixel: red in bits 5-7, blue in 3-4 and green in 0-2
	u8 const color = (cmd & REX15_OP_FLAG_RGBMODE)
		? u8((colori_r<0>() & 0xe0) | ((colori_r<2>() >> 3) & 0x18) | ((colori_r<1>() >> 5) & 0x07))
		: u8(colori_r<0>());

	int x = (cmd & REX15_OP_FLAG_XYCONTINUE) ? int(m_xcurri) : x0;
	int y = (cmd & REX15_OP_FLAG_XYCONTINUE) ? int(m_ycurri) : y0;

	// a block that has finished its last row draws nothing more
	if (block && (cmd & REX15_OP_FLAG_XYCONTINUE) && (dy > 0 ? y > y1 : y < y1))
		return;

	LOGMASKED(LOG_REX, "command %08x: %d,%d to %d,%d from %d,%d\n", cmd, x0, y0, x1, y1, x, y);

	for (unsigned n = 0; n < limit; n++)
	{
		bool draw = true;
		u8 src = color;
		if (stipple && !BIT(m_lspattern, 31 - ((x - x0) * dx & 31)))
			draw = false;
		else if (pattern)
		{
			if (!BIT(m_zpattern, 31 - n))
			{
				if (cmd & REX15_OP_FLAG_ZOPAQUE)
					src = u8(m_colorback);
				else
					draw = false;
			}
		}
		else if (cmd & REX15_OP_FLAG_COLORAUX)
			src = (cmd & REX15_OP_FLAG_QUADMODE) ? u8(auxdata >> (24 - 8 * (n & 3))) : u8(auxdata >> 24);
		else if (cmd & REX15_OP_FLAG_LOGICSRC)
		{
			src = vram_r(x + mx, y + my);
		}

		// colour compare: with aux1 bit 8 set, only pixels below the
		// source colour are written (the behaviour the firmware's REX
		// revision check identifies as revision 1)
		if (draw && (cmd & REX15_OP_FLAG_COLORCOMP) && BIT(m_aux1, 8) && vram_r(x, y) >= src)
			draw = false;

		if (draw)
			plot(x, y, src);

		if (!span)
		{
			// a single pixel
			x += dx;
			break;
		}

		if (x != x1)
		{
			x += dx;
			continue;
		}

		// end of the span
		if (!block || y == y1)
		{
			// past the end, so a following XYCONTINUE draws nothing
			x = x0;
			y += dy;
			break;
		}
		x = x0;
		y += dy;
		if (pattern || stipple)
			break;
	}

	m_xcurri = x;
	m_ycurri = y;
}
