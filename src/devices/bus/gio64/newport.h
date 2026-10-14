// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*********************************************************************

    newport.h

    SGI "Newport" graphics board emulation

*********************************************************************/

#ifndef MAME_BUS_GIO64_NEWPORT_H
#define MAME_BUS_GIO64_NEWPORT_H

#pragma once

// Fold the RAMDAC's gamma LUTs into the compositor's colour tables (a
// remapped copy of CMAP 0's palette plus the RGB-mode conversions),
// updated when a palette entry or LUT entry changes, instead of running
// every output pixel through the three LUTs.
#define MAME_SGI_NEWPORT_LUT_FOLD

// Composite each display row as runs of one DID display mode, clipped to the
// dirty rectangles' columns, with tight loops for the common modes instead
// of re-deciding the mode for every pixel. Needs MAME_SGI_NEWPORT_LUT_FOLD.
#define MAME_SGI_NEWPORT_SPAN_COMPOSITE

// RB2 pixel addressing from x/y: set_address_xy() keeps the column and
// row for the dirty tracker instead of dividing the flat address by the
// row stride for every pixel.
#define MAME_SGI_RB2_XY_ADDRESS

// RB2 dirty tracking per run: store_pixel() extends a run of adjacent
// pixels on one row and adds the run to the dirty rectangles when the
// next pixel isn't adjacent, instead of one add_rect() per pixel.
#define MAME_SGI_RB2_SPAN_DIRTY

// One call per drawn pixel: rb2_device::write_pixel_xy() does the address,
// lane expansion, logic op, masked store and dirty run inline, reading the
// destination only for logic ops that use it and storing directly when
// every bit is writable; output_pixel() is inlined into the draw loops.
// Needs MAME_SGI_RB2_XY_ADDRESS and MAME_SGI_RB2_SPAN_DIRTY.
#define MAME_SGI_REX3_FAST_PIXEL
// Diagnostic: also run the original per-pixel path for every pixel and
// count words that differ (logged to stderr).
//#define MAME_SGI_REX3_FAST_PIXEL_VERIFY
// Span and block draws without shading, blending or host data, whose logic
// op doesn't read the destination, run as clipped row fills: the clip
// rectangles are resolved to x intervals once per row, the stored value
// (or the four dither values of an unshaded RGB draw) once per command,
// and rb2_device::store_row() writes each run. Other draws keep the
// per-pixel path. Needs MAME_SGI_REX3_FAST_PIXEL.
#define MAME_SGI_REX3_FAST_SPAN
// Diagnostic: run each fast draw against the per-pixel path on a copy of the
// state and compare the VRAM rows and start registers (logged to stderr).
//#define MAME_SGI_REX3_FAST_SPAN_VERIFY

// Diagnostic: per draw-mode counts of REX3 commands, pixels and host time,
// printed to stderr every 60 s and at exit (REX3_CENSUS lines).
//#define MAME_SGI_REX3_MODE_CENSUS

#include "gio64.h"
#include "screen.h"

#define ENABLE_NEWVIEW_LOG      (0)


/*************************************
 *
 *  Dirty-region tracker (shared by rb2_device, vc2_device, and
 *  newport_base_device's own compositor)
 *
 *************************************/

// Small fixed-size (MAX_RECTS) list of disjoint-ish rectangles tracking
// the Newport compositor's dirty region. Was a single running bounding
// box until a 2026-09-25 measurement showed why that was costing real
// precision: an independently-moving cursor plus separately-updating
// windows produce widely-separated dirty regions that a single bbox
// coalesces into something close to the whole screen - confirmed live,
// not just theorized (a perf record trace found it). Kept behind the
// same narrow add_rect()/valid()/clear() interface every caller already used, so rb2_device,
// vc2_device, and newport_base_device's own compositor all benefit
// without their own call sites changing shape - only the few sites that
// need to iterate per-rect (the rb2_dirty->frame_dirty top_scanline
// conversion, and screen_update()'s own row-range loop) were touched.
//
// add_rect() always GROWS an existing slot or a fresh one - it never
// shrinks or drops coverage. When a new rect doesn't overlap/touch any
// existing slot and the list is already full, it folds into whichever
// existing slot costs the least extra area to absorb it (never invents
// a 5th slot, never discards the new rect) - this can occasionally merge
// two regions that didn't need to be merged (a missed optimization, safe
// by construction) but can never under-mark a pixel that needs recompute
// (which would be a real, visible correctness bug). x0()/y0()/x1()/y1()
// remain as an aggregate bounding box across every slot, for callers
// that only ever wanted coarse bounds (unchanged from the single-bbox
// era) - real per-rect consumers use count()/rect(i) instead.
class newport_dirty_rect
{
public:
	static constexpr int MAX_RECTS = 4;

	struct rect_t { int x0, y0, x1, y1; };

	void add_rect(int x0, int y0, int x1, int y1)
	{
		if (m_count == 0)
		{
			m_rects[0] = { x0, y0, x1, y1 };
			m_count = 1;
			return;
		}
		for (int i = 0; i < m_count; i++)
		{
			if (overlaps_or_touches(m_rects[i], x0, y0, x1, y1))
			{
				grow(m_rects[i], x0, y0, x1, y1);
				return;
			}
		}
		if (m_count < MAX_RECTS)
		{
			m_rects[m_count++] = { x0, y0, x1, y1 };
			return;
		}
		int best = 0;
		int64_t best_cost = merge_cost(m_rects[0], x0, y0, x1, y1);
		for (int i = 1; i < m_count; i++)
		{
			const int64_t cost = merge_cost(m_rects[i], x0, y0, x1, y1);
			if (cost < best_cost)
			{
				best_cost = cost;
				best = i;
			}
		}
		grow(m_rects[best], x0, y0, x1, y1);
	}

	void add_rect(const newport_dirty_rect &other)
	{
		for (int i = 0; i < other.m_count; i++)
			add_rect(other.m_rects[i].x0, other.m_rects[i].y0, other.m_rects[i].x1, other.m_rects[i].y1);
	}

	bool valid() const { return m_count > 0; }
	int count() const { return m_count; }
	const rect_t &rect(int i) const { return m_rects[i]; }

	int x0() const { int v = m_rects[0].x0; for (int i = 1; i < m_count; i++) v = std::min(v, m_rects[i].x0); return v; }
	int y0() const { int v = m_rects[0].y0; for (int i = 1; i < m_count; i++) v = std::min(v, m_rects[i].y0); return v; }
	int x1() const { int v = m_rects[0].x1; for (int i = 1; i < m_count; i++) v = std::max(v, m_rects[i].x1); return v; }
	int y1() const { int v = m_rects[0].y1; for (int i = 1; i < m_count; i++) v = std::max(v, m_rects[i].y1); return v; }

	void clear() { m_count = 0; }

private:
	// "Touching" (a shared edge, zero-pixel gap) counts as overlap too -
	// the +1 slack avoids leaving e.g. a cursor at column 99 right next
	// to a window starting at column 100 as two separate slots when
	// they're effectively one contiguous region.
	static bool overlaps_or_touches(const rect_t &r, int x0, int y0, int x1, int y1)
	{
		return x0 <= r.x1 + 1 && x1 >= r.x0 - 1 && y0 <= r.y1 + 1 && y1 >= r.y0 - 1;
	}

	static void grow(rect_t &r, int x0, int y0, int x1, int y1)
	{
		if (x0 < r.x0) r.x0 = x0;
		if (y0 < r.y0) r.y0 = y0;
		if (x1 > r.x1) r.x1 = x1;
		if (y1 > r.y1) r.y1 = y1;
	}

	static int64_t area(const rect_t &r)
	{
		return int64_t(r.x1 - r.x0 + 1) * int64_t(r.y1 - r.y0 + 1);
	}

	static int64_t merge_cost(const rect_t &r, int x0, int y0, int x1, int y1)
	{
		rect_t u = r;
		grow(u, x0, y0, x1, y1);
		return area(u) - area(r);
	}

	int m_count = 0;
	rect_t m_rects[MAX_RECTS]{};
};


/*************************************
 *
 *  Triggered stat-counter registry
 *
 *************************************/

// Named-counter registry for triggered measurement (2026-09-25).
// Motivation: this project repeatedly drew wrong
// conclusions from perf/diagnostic windows that turned out to still be
// dominated by boot/login/window-creation transients, because a fixed
// sleep/settle delay was treated as a validated proxy for "the system
// has reached steady state" - it wasn't. This lets a script (or a human,
// via gdb in interactive use) explicitly bracket an interval instead of
// guessing a delay: reset the counters right when the interval should
// start, run the guest workload, dump them right when it should end.
//
// Reached via the "sgistat" debugger console command (see
// newport_base_device::device_start() for registration, following
// sun4c_mmu.cpp's own established device-level pattern of
// machine().debugger().console().register_command(), gated on
// DEBUG_FLAG_ENABLED) - and in turn reachable non-interactively without
// an interactive debugger window, because MAME's own gdbstub backend
// (src/osd/modules/debugger/debuggdbstub.cpp) bridges GDB's "monitor
// <text>" command straight to debugger_console::execute_command(). A
// script can launch with "-debug -debugger gdbstub" (already supported
// by this project's own boot_test.sh) and drive it with e.g.
// `gdb -batch -ex "target remote :PORT" -ex "monitor sgistat reset"`.
//
// Deliberately separate from MAME's own save_item()/device_state_
// interface machinery - those serve save-states and CPU-register-style
// debugger views, a different purpose from this. newport.cpp is this
// registry's first consumer; kept generic (name -> counter pointer) on
// purpose so a second subsystem (e.g. a future MIPS3/DRC measurement)
// can register its own counters into the SAME "sgistat" command without
// newport.cpp changing at all.
class mame_sgi_stat_registry
{
public:
	static mame_sgi_stat_registry &instance()
	{
		static mame_sgi_stat_registry inst;
		return inst;
	}

	void register_counter(const char *name, uint64_t *counter)
	{
		m_counters.emplace_back(name, counter);
	}

	void reset_all()
	{
		for (auto &entry : m_counters)
			*entry.second = 0;
	}

	template <typename Sink>
	void for_each(Sink &&sink) const
	{
		for (auto &entry : m_counters)
			sink(entry.first, *entry.second);
	}

private:
	std::vector<std::pair<const char *, uint64_t *>> m_counters;
};


/*************************************
 *
 *  XMAP9 Device
 *
 *************************************/

class xmap9_device : public device_t
{
public:
	xmap9_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, uint32_t revision)
		: xmap9_device(mconfig, tag, owner, clock)
	{
		set_revision(revision);
	}

	xmap9_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	uint32_t read(uint32_t offset);
	void write(uint32_t offset, uint32_t data);

	// Getters
	uint32_t cursor_cmap() const { return m_cursor_cmap; }
	uint32_t popup_cmap() const { return m_popup_cmap; }
	uint32_t mode_entry(uint32_t entry) const { return m_mode_table[entry]; }
	bool is_8bpp() const { return BIT(m_config, 2); }

	void serialize(FILE *file);
	void deserialize(FILE *file);

private:
	// device_t overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// Initializers
	void set_revision(uint32_t revision) { m_revision = revision; }

	uint32_t m_config;
	uint32_t m_revision;
	uint32_t m_fifo_available;
	//uint32_t m_entries;
	uint32_t m_cursor_cmap;
	uint32_t m_popup_cmap;
	uint32_t m_mode_table_idx;
	uint32_t m_mode_table[0x20];
};

DECLARE_DEVICE_TYPE(XMAP9, xmap9_device)


/*************************************
 *
 *  CMAP Device
 *
 *************************************/

class cmap_device : public device_t
				  , public device_palette_interface
{
public:
	cmap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, uint32_t revision)
		: cmap_device(mconfig, tag, owner, clock)
	{
		set_revision(revision);
	}

	cmap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	uint32_t read(uint32_t offset);
	void write(uint32_t offset, uint32_t data);

	// Getters
	const uint32_t *palette_base() const { return m_palette; }
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	// range of palette entries written since the last call; false if none
	bool take_dirty_range(uint32_t &lo, uint32_t &hi)
	{
		if (m_dirty_lo > m_dirty_hi)
			return false;
		lo = m_dirty_lo;
		hi = m_dirty_hi;
		m_dirty_lo = 0x10000;
		m_dirty_hi = 0;
		return true;
	}
#endif

	void serialize(FILE *file);
	void deserialize(FILE *file);

private:
	// device_t overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_palette_interface overrides
	virtual uint32_t palette_entries() const noexcept override { return 0x2000; }

	// Initializers
	void set_revision(uint32_t revision) { m_revision = revision; }

	uint32_t m_status;
	uint32_t m_revision;
	uint16_t m_palette_idx;
	uint32_t m_palette[0x10000];
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	uint32_t m_dirty_lo = 0;
	uint32_t m_dirty_hi = 0xffff;
#endif
};

DECLARE_DEVICE_TYPE(CMAP, cmap_device)


/*************************************
 *
 *  VC2 Device
 *
 *************************************/

class vc2_device : public device_t
{
public:
	vc2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	uint32_t read(uint32_t offset);
	void write(uint32_t offset, uint32_t data, uint32_t mem_mask);

	// Getters
	int readout_x0() const { return m_readout_x0; }
	int readout_y0() const { return m_readout_y0; }
	int readout_x1() const { return m_readout_x1; }
	int readout_y1() const { return m_readout_y1; }
	uint16_t begin_did_line(int y);
	uint16_t next_did_line_entry();
	uint8_t get_cursor_pixel(int x, int y);

	// Current on-screen cursor box (same size logic as get_cursor_pixel(),
	// NOT is_cursor_active()'s narrower fixed-32 early-out) - distinct
	// from dirty()'s historical old-position-union-new-position box,
	// used to scope screen_update()'s separate cursor overlay stamp pass
	// to just the cursor's actual current extent.
	bool cursor_enabled() const { return m_enable_cursor; }
	void cursor_bounds(int &x0, int &y0, int &x1, int &y1) const
	{
		const bool size64 = BIT(m_display_ctrl, CURSOR_SIZE_BIT) == CURSOR_SIZE_64;
		const int size = size64 ? 64 : 32;
		x0 = (int)m_cursor_x - 31;
		y0 = (int)m_cursor_y - 31;
		x1 = x0 + size - 1;
		y1 = y0 + size - 1;
	}

	void vblank_w(int state);

	auto vert_int() { return m_vert_int.bind(); }
	auto screen_timing_changed() { return m_screen_timing_changed.bind(); } // Hack. TODO: Figure out a better way

	// Dirty-region tracking for the compositor (see newport_dirty_rect).
	// The cursor is composited on-the-fly by screen_update() from live
	// state - it never touches rb2_device's framebuffer - so it needs
	// its own dirty contribution, unioned in here whenever cursor state
	// changes, separate from rb2_device's framebuffer-write tracking.
	const newport_dirty_rect &dirty() const { return m_dirty; }
	void clear_dirty() { m_dirty.clear(); }

	void serialize(FILE *file);
	void deserialize(FILE *file);

private:
	// device_t overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	enum
	{
		CURSOR_FUNC_ENABLE_BIT = 4,
		CURSOR_ENABLE_BIT = 7,

		CURSOR_MODE_BIT = 8,
		CURSOR_MODE_GLYPH = 0,
		CURSOR_MODE_CROSSHAIR = 1,

		CURSOR_SIZE_BIT = 9,
		CURSOR_SIZE_32 = 0,
		CURSOR_SIZE_64 = 1
	};

	// Computes the cursor's current on-screen bounding box (same size
	// logic as get_cursor_pixel()'s own 32-vs-64 check) and unions it
	// into m_dirty - called from write() both before and after any state
	// change that could move/resize/enable/disable the cursor or touch
	// its bitmap RAM, so both the old and new positions get redrawn.
	void mark_cursor_dirty();

	void decode_vt_line(uint32_t line, uint32_t line_seq_ptr);
	void decode_vt_table();
	void update_screen_size();

	bool is_cursor_active(int x, int y);

	uint16_t m_vid_entry;
	uint16_t m_cursor_entry;
	uint16_t m_cursor_x;
	uint16_t m_cursor_y;
	uint16_t m_cur_cursor_x;
	uint16_t m_did_entry;
	uint16_t m_scanline_len;
	uint16_t m_ram_addr;
	uint16_t m_vt_frame_ptr;
	uint16_t m_vt_line_ptr;
	uint16_t m_vt_line_run;
	uint16_t m_vt_line_count;
	uint16_t m_cursor_table_ptr;
	uint16_t m_work_cursor_y;
	uint16_t m_did_frame_ptr;
	uint16_t m_did_line_ptr;
	uint16_t m_display_ctrl;
	uint16_t m_config;
	uint8_t m_reg_idx;
	uint16_t m_reg_data;
	std::unique_ptr<uint16_t[]> m_ram;

	newport_dirty_rect m_dirty;

	std::unique_ptr<uint32_t[]> m_vt_table;

	int m_readout_x0;
	int m_readout_y0;
	int m_readout_x1;
	int m_readout_y1;

	bool m_enable_cursor;

	devcb_write_line m_vert_int;
	devcb_write_line m_screen_timing_changed;

	static const size_t RAM_SIZE;
};

DECLARE_DEVICE_TYPE(VC2, vc2_device)


/*************************************
 *
 *  RB2 Device
 *
 *************************************/

class rb2_device : public device_t
{
public:
	rb2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock, uint32_t global_mask)
		: rb2_device(mconfig, tag, owner, clock)
	{
		set_global_mask(global_mask);
	}

	rb2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	// Getters
	const uint32_t *rgbci(int y) const { return &m_rgbci[1344 * y]; }
	const uint32_t *cidaux(int y) const { return &m_cidaux[1344 * y]; }
	uint32_t store_shift() { return m_src_shift; }

	// callbacks
	void set_write_mask(uint32_t data);
	void set_flags(uint16_t data);
	void set_address(uint32_t address);
#ifdef MAME_SGI_RB2_XY_ADDRESS
	void set_address_xy(int x, int y)
	{
		m_buf_ptr = &m_dest_buf[y * (1280 + 64) + x];
		m_cur_x = x;
		m_cur_y = y;
	}
#endif
	uint32_t read_pixel();
	void write_pixel(uint32_t data);
#ifdef MAME_SGI_REX3_FAST_PIXEL
	void write_pixel_xy(int x, int y, uint32_t data);
#endif
#ifdef MAME_SGI_REX3_FAST_SPAN
	bool stored_value(uint32_t data, uint32_t &value) const;
	void store_row(int y, int x0, int x1, const uint32_t fg[4], uint32_t pattern, int bit, bool has_bg, uint32_t bg);
	uint32_t *dest_row(int y) { return &m_dest_buf[y * (1280 + 64)]; }
#endif

	// Dirty-region tracking for the compositor (see newport_dirty_rect).
	// Updated per-pixel directly in store_pixel() - the single choke
	// point every framebuffer write (direct pixel ops and all REX3
	// draw/blit/fill/text operations) funnels through.
#ifdef MAME_SGI_RB2_SPAN_DIRTY
	// the pending run is added first
	const newport_dirty_rect &dirty() { flush_dirty_run(); return m_dirty; }
	void clear_dirty() { m_dirty.clear(); m_run_valid = false; }
#else
	const newport_dirty_rect &dirty() const { return m_dirty; }
	void clear_dirty() { m_dirty.clear(); }
#endif

	void serialize(FILE *file);
	void deserialize(FILE *file);

private:
	// device_t overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	void logic_pixel(uint32_t src);
	void store_pixel(uint32_t value);
	uint32_t expand_to_all_lanes(uint32_t src);

	void set_global_mask(uint32_t global_mask) { m_global_mask = global_mask; }

	uint32_t  m_global_mask;
	uint32_t  m_write_mask;
	bool      m_blend;
	bool      m_fast_clear;
	bool      m_rgbmode;
	bool      m_dblsrc;
	uint8_t   m_plane_enable;
	uint8_t   m_draw_depth;
	uint32_t  m_draw_bpp;
	uint8_t   m_logicop;
	uint8_t   m_src_shift;
	uint8_t   m_dst_shift;

	uint32_t *m_dest_buf;
	uint32_t *m_buf_ptr;

	// x/y derived from the flat address in set_address() (every call
	// site computes it as y * 1344 + x, matching rgbci()/cidaux()'s own
	// stride), cached here so store_pixel() can update m_dirty without
	// redoing the division/modulo on every single pixel write.
	int m_cur_x;
	int m_cur_y;
	newport_dirty_rect m_dirty;
#ifdef MAME_SGI_RB2_SPAN_DIRTY
	// run of written pixels on row m_run_y, columns m_run_x0..m_run_x1,
	// not yet added to m_dirty
	bool m_run_valid = false;
	int m_run_y = 0;
	int m_run_x0 = 0;
	int m_run_x1 = 0;
	void flush_dirty_run()
	{
		if (m_run_valid)
			m_dirty.add_rect(m_run_x0, m_run_y, m_run_x1, m_run_y);
		m_run_valid = false;
	}
#endif

	std::unique_ptr<uint32_t[]> m_rgbci;
	std::unique_ptr<uint32_t[]> m_cidaux;

	static const size_t BUFFER_SIZE;
};

DECLARE_DEVICE_TYPE(RB2, rb2_device)


/*************************************
 *
 *  Newport Device
 *
 *************************************/

class newport_base_device : public device_t
						  , public device_gio64_card_interface
{
public:
	newport_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock = 0);

	uint64_t rex3_r(offs_t offset, uint64_t mem_mask = ~0);
	void rex3_w(offs_t offset, uint64_t data, uint64_t mem_mask = ~0);

#ifdef MAME_SGI_RENDER_DIRTY_RECTS
	uint32_t screen_update(screen_device &device, bitmap_rgb32 &bitmap, const rectangle &cliprect, render_dirty_rects &dirty);
#else
	uint32_t screen_update(screen_device &device, bitmap_rgb32 &bitmap, const rectangle &cliprect);
#endif

	void vrint_w(int state);
	void update_screen_size(int state);

	void serialize(FILE *file);
	void deserialize(FILE *file);

protected:
	// device_t overrides
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	void device_add_mconfig(machine_config &config, uint32_t xmap_revision, uint32_t cmap_revision, uint32_t global_mask);
	void mem_map(address_map &map) override;

	TIMER_CALLBACK_MEMBER(dcb_timeout_tick);

	enum
	{
		DCB_ADDR_VC2,
		DCB_ADDR_CMAP01,
		DCB_ADDR_CMAP0,
		DCB_ADDR_CMAP1,
		DCB_ADDR_XMAP01,
		DCB_ADDR_XMAP0,
		DCB_ADDR_XMAP1,
		DCB_ADDR_RAMDAC,
		DCB_ADDR_CC1,
		DCB_ADDR_AB1,
		DCB_ADDR_PCD = 12
	};

	enum
	{
		STATUS_GFXBUSY          = (1 << 3),
		STATUS_BACKBUSY         = (1 << 4),
		STATUS_VRINT            = (1 << 5),
		STATUS_VIDEOINT         = (1 << 6),
		STATUS_GFIFOLEVEL_SHIFT = 7,
		STATUS_GFIFOLEVEL_MASK  = (0x3f << STATUS_GFIFOLEVEL_SHIFT),
		STATUS_BFIFOLEVEL_SHIFT = 13,
		STATUS_BFIFOLEVEL_MASK  = (0x1f << STATUS_BFIFOLEVEL_SHIFT),
		STATUS_BFIFO_INT        = 18,
		STATUS_GFIFO_INT        = 19
	};

	struct rex3_t
	{
		uint32_t m_draw_mode0;
		bool m_color_host;
		uint32_t m_draw_mode1;
		uint8_t m_plane_enable;
		uint8_t m_plane_depth;
		bool m_rwpacked;
		bool m_rwdouble;
		uint8_t m_hostdepth;
		uint8_t m_sfactor;
		uint8_t m_dfactor;

		uint32_t m_store_shift;
		uint32_t m_host_shift;

		uint32_t m_write_width;
		uint32_t m_ls_mode;
		uint32_t m_ls_pattern;
		uint32_t m_ls_pattern_saved;
		uint32_t m_z_pattern;
		uint32_t m_color_back;
		uint32_t m_color_vram;
		uint32_t m_alpha_ref;
		uint32_t m_setup;
		uint32_t m_step_z;
		int32_t m_x_start;
		int32_t m_y_start;
		int32_t m_x_end;
		int32_t m_y_end;
		int16_t m_x_start_frac;
		int16_t m_y_start_frac;
		int16_t m_x_end_frac;
		int16_t m_y_end_frac;
		int16_t m_x_save;
		uint32_t m_xy_move;
		int16_t m_x_move;
		int16_t m_y_move;
		uint32_t m_bres_d;
		uint32_t m_bres_s1;
		uint32_t m_bres_octant_inc1;
		uint32_t m_bres_round_inc2;
		uint32_t m_bres_e1;
		uint32_t m_bres_s2;
		uint32_t m_a_weight0;
		uint32_t m_a_weight1;
		uint32_t m_x_start_f;
		uint32_t m_y_start_f;
		uint32_t m_x_end_f;
		uint32_t m_y_end_f;
		int16_t m_x_start_i;
		uint32_t m_xy_start_i;
		int16_t m_y_start_i;
		uint32_t m_xy_end_i;
		int16_t m_x_end_i;
		int16_t m_y_end_i;
		uint32_t m_x_start_end_i;
		uint32_t m_color_red;
		uint32_t m_color_alpha;
		uint32_t m_color_green;
		uint32_t m_color_blue;
		uint32_t m_curr_color_red;
		uint32_t m_curr_color_alpha;
		uint32_t m_curr_color_green;
		uint32_t m_curr_color_blue;
		int32_t m_slope_red;
		int32_t m_slope_alpha;
		int32_t m_slope_green;
		int32_t m_slope_blue;
		uint32_t m_write_mask;
		uint32_t m_color_i;
		uint32_t m_zero_overflow;
		uint64_t m_host_dataport;
		uint32_t m_dcb_mode;
		uint32_t m_dcb_reg_select;
		uint32_t m_dcb_slave_select;
		uint32_t m_dcb_data_msw;
		uint32_t m_dcb_data_lsw;
		uint32_t m_smask_x[5];
		uint32_t m_smask_y[5];
		uint32_t m_top_scanline;
		uint32_t m_xy_window;
		int16_t m_x_window;
		int16_t m_y_window;
		uint32_t m_clip_mode;
		uint32_t m_config;
		uint32_t m_status;
		uint32_t m_dcb_mask;
		bool m_blend_back;
	};

	void ramdac_write(uint32_t data);

	void write_x_start(int32_t val);
	void write_y_start(int32_t val);
	void write_x_end(int32_t val);
	void write_y_end(int32_t val);

	bool pixel_clip_pass(int16_t x, int16_t y);
#ifdef MAME_SGI_REX3_FAST_SPAN
	int clip_row_intervals(int16_t y, int xa, int xb, int lo[5], int hi[5]);
	bool fast_draw(uint8_t adrmode, int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, int16_t dx, int16_t dy);
	bool m_fast_draw_disabled = false;
#endif
	void output_pixel(uint32_t color);
	void output_pixel(int16_t x, int16_t y, uint32_t color);
	void blend_pixel(uint32_t src);

	void iterate_shade();

	uint32_t get_default_color(uint32_t src);
	uint32_t get_host_color();
	uint32_t get_rgb_color(int16_t x, int16_t y);

	uint32_t convert_4bpp_bgr_to_8bpp(uint8_t pix_in);
	uint32_t convert_4bpp_bgr_to_12bpp(uint8_t pix_in);
	uint32_t convert_4bpp_bgr_to_24bpp(uint8_t pix_in);
	uint32_t convert_8bpp_bgr_to_4bpp(uint8_t pix_in);
	uint32_t convert_8bpp_bgr_to_12bpp(uint8_t pix_in);
	uint32_t convert_8bpp_bgr_to_24bpp(uint8_t pix_in);
	uint32_t convert_12bpp_bgr_to_4bpp(uint16_t pix_in);
	uint32_t convert_12bpp_bgr_to_8bpp(uint16_t pix_in);
	uint32_t convert_12bpp_bgr_to_24bpp(uint16_t pix_in);
	uint32_t convert_24bpp_bgr_to_4bpp(uint32_t pix_in);
	uint32_t convert_24bpp_bgr_to_8bpp(uint32_t pix_in);
	uint32_t convert_24bpp_bgr_to_12bpp(uint32_t pix_in);

	uint32_t convert_4bpp_bgr_to_24bpp_rgb(uint8_t pix_in);
	uint32_t convert_8bpp_bgr_to_24bpp_rgb(uint8_t pix_in);
	uint32_t convert_12bpp_bgr_to_24bpp_rgb(uint16_t pix_in);

	struct bresenham_octant_info_t
	{
		int16_t incrx1;
		int16_t incrx2;
		int16_t incry1;
		int16_t incry2;
		uint8_t loop;
	};
	uint8_t get_octant(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t dx, int32_t dy);
	void do_setup();
	void do_fline(uint32_t color);
	// A LENGTH32 fractional line stops after 32 pixels; the next command
	// continues it with the same Bresenham state when the guest has not
	// moved its start or end (it only re-issues the command).
	struct fline_continuation
	{
		bool valid = false;
		int32_t x = 0, y = 0;		// resume pixel, 1/16 units
		int32_t x2 = 0, y2 = 0;		// line end, 1/16 units
		int32_t d = 0, incr1 = 0, incr2 = 0;
		uint8_t octant = 0;
	};
	fline_continuation m_fline_cont;
	void do_iline(uint32_t color);

	uint32_t do_pixel_read();
	uint64_t do_pixel_word_read();

	void do_rex3_command();

	void ramdac_remap(uint32_t *dest);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	uint32_t ramdac_map(uint32_t rgb) const
	{
		return 0xff000000 | m_ramdac_lut_r[(uint8_t)(rgb >> 16)] | m_ramdac_lut_g[(uint8_t)(rgb >> 8)] | m_ramdac_lut_b[(uint8_t)rgb];
	}
	void lut_fold_update();
	void lut_fold_invalidate() { m_lut_fold_dirty = true; }
#endif
#ifdef MAME_SGI_NEWPORT_SPAN_COMPOSITE
	// one DID run's display mode, decoded from its XMAP mode table entry
	struct composite_mode
	{
		uint32_t table_entry;
		uint8_t pix_mode, pix_size, aux_pix_mode;
		uint16_t aux_msb, ci_msb, popup_msb;
	};
	void composite_span(const composite_mode &mode, const uint32_t *palette, uint32_t *dest,
			const uint32_t *src_rgbci, const uint32_t *src_cidaux, int count);
#endif

	// Marks the whole visible framebuffer region dirty (see
	// m_pending_dirty below).
	// Used for register-level state changes that alter how already-
	// resident VRAM data gets interpreted/displayed without themselves
	// going through store_pixel(), the only other source of dirty-rect
	// contributions - such a change is otherwise completely invisible to
	// the compositor. Still used for the VC2 DID/VT-table RAM-word write
	// case and reset: a written RAM address could be
	// a DID entry, a VT entry, or cursor data, and mapping it to the
	// display rows it affects is a reverse lookup through two levels of
	// indirection - harder, not yet precisely mapped. The
	// XMAP mode-table case below no longer uses this - see
	// mark_dirty_for_xmap_slot().
	void mark_full_dirty();

	// Companion to mark_full_dirty() above, for the case that IS
	// precisely mappable: an XMAP9 mode-table slot write. Looks up which
	// display rows the most recent screen_update() DID walk found
	// referencing this slot (see m_row_xmap_slot_mask below) and marks
	// only those dirty, instead of the whole screen. If the slot isn't
	// currently referenced by any row, marks nothing - correct, not a
	// missed update: with no row using this slot, the write has no
	// visible effect until some future DID/VT change makes a row
	// reference it, and any such change already forces a full-screen
	// recompute via mark_full_dirty() (the VC2 RAM-word case above),
	// which revisits every row and keeps m_row_xmap_slot_mask in sync
	// before that new reference could ever be missed.
	void mark_dirty_for_xmap_slot(uint8_t slot);

	// "sgistat reset" / "sgistat dump [name]" debugger console command -
	// see mame_sgi_stat_registry's own comment for the full rationale.
	void sgistat_command(const std::vector<std::string_view> &params);

	required_device<screen_device> m_screen;
	required_device_array<xmap9_device, 2> m_xmap;
	required_device_array<cmap_device, 2> m_cmap;
	required_device<vc2_device> m_vc2;
	required_device<rb2_device> m_rb2;

	uint32_t m_ramdac_lut_r[256];
	uint32_t m_ramdac_lut_g[256];
	uint32_t m_ramdac_lut_b[256];
	uint8_t m_ramdac_lut_index;
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	// derived from CMAP 0 and the LUTs, so not saved; rebuilt after a load
	bool m_lut_fold_dirty = true;
	uint32_t m_lut_palette[0x10000];    // CMAP 0 palette through the LUTs
	uint32_t m_lut_rgb4[0x10];          // RGB Map0 conversions through the LUTs
	uint32_t m_lut_rgb8[0x100];
	uint32_t m_lut_rgb12[0x1000];
#endif

	// Dirty-region tracking for screen_update() (see newport_dirty_rect).
	// m_pending_dirty accumulates everything dirtied since the previous
	// screen_update() call (register-level marks plus each frame's
	// framebuffer/cursor contributions); m_prev_dirty is what the
	// previous call itself consumed. MAME's screen_device double-buffers
	// its bitmap (m_bitmap[2], toggled every frame regardless of skip
	// status), so the buffer being drawn into this frame is two frames
	// stale: the region to refresh is exactly this frame's fresh dirt
	// unioned with the previous frame's. Nothing accumulates beyond
	// those two frames, so continuous drawing stays as tight as the
	// drawing itself rather than growing into a permanent union.
	// Starts full-screen so the first frames after boot/reset always do
	// a real recompute rather than assuming unknown initial framebuffer
	// content matches a blank bitmap.
	newport_dirty_rect m_pending_dirty;
	newport_dirty_rect m_prev_dirty;

	// Bitmask (bit N = slot N) of which XMAP9 mode-table slots each
	// bitmap row's DID chain referenced, as of the most recent
	// screen_update() call that actually walked that row. Only rows in
	// that call's row_start_y..row_end_y clipped range get overwritten
	// (see screen_update()'s own comment on the row-range clip) - safe
	// because a row's DID entries only change alongside a VC2 RAM write,
	// which always forces a full-screen recompute (mark_full_dirty()'s
	// VC2 call site), so every row gets revisited and this table fully
	// resynced whenever it could have gone stale. Consumed by
	// mark_dirty_for_xmap_slot(). Sized to the same 1088-row raw-buffer
	// bound mark_full_dirty() uses.
	uint32_t m_row_xmap_slot_mask[1088]{};

	rex3_t m_rex3;
	uint32_t m_global_mask;
	emu_timer *m_dcb_timeout_timer;

	void write_mask_w(uint32_t data) { m_rb2->set_write_mask(data); }
	void draw_flags_w(uint16_t data) { m_rb2->set_flags(data); }
	void set_address(uint32_t data) { m_rb2->set_address(data); }
#ifdef MAME_SGI_RB2_XY_ADDRESS
	void set_address_xy(int x, int y) { m_rb2->set_address_xy(x, y); }
#endif
	void write_pixel(uint32_t data) { m_rb2->write_pixel(data); }
	uint32_t read_pixel() { return m_rb2->read_pixel(); }

#if ENABLE_NEWVIEW_LOG
	void start_logging();
	void stop_logging();

	FILE *m_newview_log;
#endif

	static const uint32_t s_host_shifts[4];
};

class gio64_xl8_device : public newport_base_device
{
public:
	gio64_xl8_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0U);

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
};

class gio64_xl24_device : public newport_base_device
{
public:
	gio64_xl24_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0U);

protected:
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
};

DECLARE_DEVICE_TYPE(GIO64_XL8,  gio64_xl8_device)
DECLARE_DEVICE_TYPE(GIO64_XL24, gio64_xl24_device)

#endif // MAME_BUS_GIO64_NEWPORT_H
