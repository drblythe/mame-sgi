// license:BSD-3-Clause
// copyright-holders:Ryan Holtz, Tyson Smith
/*
        Silicon Graphics LG1 "Light" graphics board used as
        entry level graphics in the Indigo and IRIS Crimson.
*/

#ifndef MAME_SGI_LIGHT_H
#define MAME_SGI_LIGHT_H

#pragma once

#include "video/bt47x.h"
#include "vc1.h"

#include "screen.h"

class sgi_lg1_device : public device_t
{
public:
	auto write_vblank() { return  m_screen.lookup()->screen_vblank(); }

	sgi_lg1_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);

	void map(address_map &map) ATTR_COLD;

protected:
	// device_t implementation
	void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	u32 screen_update(screen_device &screen, bitmap_rgb32 &bitmap, rectangle const &cliprect);

	void do_rex_command();
	void plot(int x, int y, u8 src);
	bool compare_pass(u8 src, u8 dst) const;
	u8 logic_op(u8 src, u8 dst) const;
	// clipping: aux2 bits 4-7 enable screen masks 1-4, each an x and a y
	// register holding the first (bits 0-9) and last (bits 16-25) screen
	// coordinate, and drawing is limited to the union of the enabled masks;
	// aux2 bits 8-11 are a clip id match mask, bit n allowing drawing where
	// the clip id planes hold n (checked unless drawing into the clip id
	// planes themselves)
	bool clip_pass(int x, int y) const;
	u8 vram_r(int x, int y) const;
	// aux2 bits 29-30 select the planes drawing and pixel reads use: the 8
	// colour planes (1), the 2 pop-up planes (2) or the 2 clip id planes
	// (3), each a separate memory
	unsigned plane_select() const { return BIT(m_aux2, 29, 2); }
	u8 *plane_buffer() const { return (plane_select() == 2) ? m_pup.get() : (plane_select() == 3) ? m_cid.get() : m_vram.get(); }
	u8 plane_mask() const { return (plane_select() >= 2) ? 0x03 : 0xff; }
	// 4-bit double-buffered mode (aux1 bit 5): the colour planes hold two
	// 4-bit buffers, buffer 0 in bits 0-3 and buffer 1 in bits 4-7; aux1
	// bits 2 and 3 enable writes to buffers 0 and 1, bit 4 reads buffer 1
	bool dbl4() const { return BIT(m_aux1, 5) && plane_select() < 2; }
	// window origin: coordinates are offset by xywin, where 0x800 is none
	int xywin_x() const { return int((m_xywin >> 16) & 0x0fff) - 0x800; }
	int xywin_y() const { return int(m_xywin & 0x0fff) - 0x800; }
	void rex_reg_w(unsigned reg, u32 data);
	u32 rex_reg_r(unsigned reg);
	void go_w(offs_t offset, u32 data);
	u32 go_r(offs_t offset);

	// rex register read handlers
	u32 command_r() { return m_command; }
	u32 aux1_r() { return m_aux1; }
	u32 xstate_r() { return m_xstate; }
	u32 xstart_r() { return m_xstart; }
	u32 xstarti_r() { return m_xstart >> 15; }
	u32 xstartf_r() { return m_xstart >> 4; }
	u32 xendf_r() { return m_xend >> 4; }
	u32 ystarti_r() { return m_ystart >> 15; }
	u32 ystartf_r() { return m_ystart >> 4; }
	u32 ystart_r() { return m_ystart; }
	u32 yendf_r() { return m_yend >> 4; }
	u32 xsave_r() { return m_xsave; }
	u32 minorslope_r() { return m_minorslope; }
	u32 xymove_r() { return m_xymove; }
	template <unsigned Color> u32 colori_r() { return m_color[Color] >> 11; }
	template <unsigned Color> u32 colorf_r() { return m_color[Color]; }
	template <unsigned Color> u32 slopecolor_r() { return m_slopecolor[Color]; }
	u32 colorback_r() { return m_colorback; }
	u32 zpattern_r() { return m_zpattern; }
	u32 lspattern_r() { return m_lspattern; }
	u32 lsmode_r() { return m_lsmode; }
	u32 aweight_r() { return m_aweight; }
	u32 rwaux1_r() { return m_rwaux1; }
	u32 rwaux2_r() { return m_rwaux2; }
	u32 rwmask_r() { return m_rwmask; }
	u32 xendi_r() { return m_xend >> 15; }
	u32 yendi_r() { return m_yend >> 15; }

	// rex register write handlers
	void command_w(offs_t offset, u32 data) { m_command = data; m_logicop = data >> 28; }
	void aux1_w(u32 data) { m_aux1 = data & 0x03ffU; }
	// the drawing state: color index (bits 0-7), write mask (16-23) and
	// logic operation (24-27), also set through colorredi, rwmask and the
	// command
	void xstate_w(u32 data) { m_xstate = data; m_color[0] = (data & 0xffU) << 11; m_wrmask = u8(data >> 16); m_logicop = (data >> 24) & 15; }
	void xstarti_w(u32 data) { m_xstart = (data & 0x0fffU) << 15; m_xcurri = data & 0x0fffU; }
	void xstartf_w(u32 data) { m_xstart = (data & 0x007fffffU) << 4; m_xcurri = (data >> 11) & 0x0fffU; }
	void xstart_w(u32 data) { m_xstart = data & 0x07ffffffU; }
	void xendf_w(u32 data) { m_xend = (data & 0x007ff800U) << 4; }
	void ystarti_w(u32 data) { m_ystart = (data & 0x0fffU) << 15; m_ycurri = data & 0x0fffU; }
	void ystartf_w(u32 data) { m_ystart = (data & 0x007fffffU) << 4; m_ycurri = (data >> 11) & 0x0fffU; }
	void ystart_w(u32 data) { m_ystart = data & 0x07ffffffU; }
	void yendf_w(u32 data) { m_yend = (data & 0x007ff800U) << 4; }
	void xsave_w(u32 data) { m_xsave = data & 0x0fffU; }
	void minorslope_w(u32 data);
	void xymove_w(offs_t offset, u32 data) { m_xymove = data & 0x07ff07ffU; }
	template <unsigned Color> void colori_w(offs_t offset, u32 data) { m_color[Color] = (data & 0xffU) << 11; }
	template <unsigned Color> void colorf_w(u32 data) { m_color[Color] = data & 0x000fffffU; }
	template <unsigned Color> void slopecolor_w(u32 data);
	void colorback_w(u32 data) { m_colorback = data & 0xffU; }
	void zpattern_w(offs_t offset, u32 data) { m_zpattern = data; }
	void lspattern_w(u32 data) { m_lspattern = data; }
	void lsmode_w(u32 data) { m_lsmode = data & 0x000fffffU; }
	void aweight_w(u32 data) { m_aweight = data; }
	void rwaux1_w(u32 data) { m_rwaux1 = data; }
	void rwaux2_w(u32 data) { m_rwaux2 = data; }
	void rwmask_w(u32 data) { m_rwmask = data & 0xffffU; m_wrmask = u8(data); }
	void xendi_w(offs_t offset, u32 data) { m_xend = (data & 0x0fffU) << 15; }
	void yendi_w(offs_t offset, u32 data) { m_yend = (data & 0x0fffU) << 15; }

	// configuration register read handlers
	template <unsigned Index> u32 smask_r() { return m_smask[Index]; }
	u32 aux2_r() { return m_aux2; }
	u32 diagvram_r() { return m_diagvram; }
	u32 diagcid_r() { return m_diagcid; }
	// with configsel 4 this reads the board id: bits 0-2 board revision,
	// bits 3-5 monitor type, bit 6 set when no video option is fitted
	u32 wclock_r() { return m_configsel == 4 ? 0x40 : m_wclock; }
	u32 dac_r(offs_t offset);
	u32 configsel_r() { return m_configsel; }
	u32 vc1_r(offs_t offset);
	u32 togglectxt_r() { return m_togglectxt; }
	// bit 0 is the busy flag; commands complete at once, so never busy
	u32 configmode_r() { return m_configmode & ~1U; }
	u32 xywin_r() { return m_xywin; }

	// configuration register write handlers
	template <unsigned Index> void smask_w(u32 data) { m_smask[Index] = data & 0x03ff03ffU; }
	void aux2_w(u32 data) { m_aux2 = data & 0x7fffffffU; }
	void diagvram_w(u32 data) { m_diagvram = data; }
	void diagcid_w(u32 data) { m_diagcid = data; }
	void wclock_w(u32 data) { m_wclock = data & 0xffU; }
	void dac_w(offs_t offset, u32 data);
	void configsel_w(u32 data) { m_configsel = data & 7; }
	void vc1_w(offs_t offset, u32 data);
	void togglectxt_w(u32 data) { m_togglectxt = data; }
	void configmode_w(u32 data) { m_configmode = data; }
	void xywin_w(u32 data) { m_xywin = data & 0x0fff0fffU; }

private:
	required_device<sgi_vc1_device> m_vc1;
	required_device<bt479_device> m_lut1;
	required_device<screen_device> m_screen;
	std::unique_ptr<u8[]> m_vram;
	std::unique_ptr<u8[]> m_pup;
	std::unique_ptr<u8[]> m_cid;

	// rex registers
	u32 m_command;
	u32 m_aux1;
	u32 m_xstate;
	u32 m_xstart;
	u32 m_ystart;
	u32 m_xymove;
	u32 m_color[3];
	u32 m_slopecolor[3];
	u32 m_colorback;
	u32 m_zpattern;
	u32 m_lspattern;
	u32 m_lsmode;
	u32 m_aweight;
	u32 m_rwaux1;
	u32 m_rwaux2;
	u32 m_rwmask;
	u32 m_xend;
	u32 m_yend;
	u32 m_xsave;
	u32 m_minorslope;
	u8 m_wrmask;
	u8 m_logicop;
	bool m_aux2_source;

	// configuration registers
	u32 m_smask[8];
	u32 m_aux2;
	u32 m_diagvram;
	u32 m_diagcid;
	u32 m_wclock;
	u32 m_configsel;
	u32 m_togglectxt;
	u32 m_configmode;
	u32 m_xywin;

	// other state
	u32 m_xcurri;
	u32 m_ycurri;
	u8 m_data;
};

DECLARE_DEVICE_TYPE(SGI_LG1, sgi_lg1_device)

#endif // MAME_SGI_LIGHT_H
