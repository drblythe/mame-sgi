// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*********************************************************************

    newport.cpp

    SGI "Newport" graphics board emulation

    Newport is modular, consisting of the following custom chips:
    - REX3: Raster Engine, essentially a blitter which can also draw
            antialiased lines. REX also acts as the interface to the
            rest of the system - all the other chips on a Newport
            board are accessed through it.
    - RB2: Frame buffer input controller
    - RO1: Frame buffer output controller
    - XMAP9: Final display generator
    - CMAP: Palette mapper
    - VC2: Video timing controller / CRTC

    Newport-derived graphics boards were shipped for the Indy
    workstation and some Indigo 2 workstations.

    Known Issues:
    - Antialiased line drawing currently uses the fractional line
      drawing routine instead.
    - Timing-wise, all draw commands complete instantly and all
      FIFOs are always empty. Thus the performance of an emulated
      Newport board is likely to be better than real life in scenes
      that would otherwise be drawing-bound.
    - The REX3 emulation currently handles a bunch of functionality
      that should be the responsibility of RO1 and RB2. However,
      emulating it properly would bring down the emulation speed
      for not much practical benefit.

*********************************************************************/

#include "emu.h"
#include "newport.h"

#include "debug/debugcon.h"
#include "debugger.h"

#include <algorithm>
#include <cstdarg>
#include <unordered_map>
#if defined(MAME_SGI_REX3_MODE_CENSUS) && (defined(__x86_64__) || defined(__i386__))
#include <x86intrin.h>
#define MAME_SGI_CENSUS_CYCLES() __rdtsc()
#else
#define MAME_SGI_CENSUS_CYCLES() uint64_t(osd_ticks())
#endif

// Dedicated toggle (2026-08-21) for the mouse/cursor-focus diagnostic
// prints below - see osd/sdl/window.cpp's own copy of
// this macro/comment for the full rationale (self-defined per-file,
// per this project's src/devices/ scoping convention). Off by default;
// re-enable to revisit the mouse/cursor-focus investigation.
// #define MAME_SGI_MOUSEFOCUS_DIAG

// Originated as a TEMPORARY diagnostic (2026-09-17) - counting
// per-frame contributions to frame_dirty (screen_update()'s own dirty
// gathering) separately by source (rb2's real pixel writes vs vc2's
// cursor-position/enable writes), to find why a visually-static screen
// still marked dirty on every vblank. Promoted to a permanent, always-on
// facility (2026-09-25) after this project's own repeated
// experience this session that a fixed sleep/settle-time heuristic before
// sampling is not a validated proxy for "the system has reached steady
// state" - several perf measurements this session drew conclusions from
// windows that turned out to still be dominated by boot/login/window-
// creation transients. These counters (now unconditionally maintained,
// negligible cost - a handful of integer increments) are readable/
// resettable on demand via the "sgistat" debugger console command (see
// mame_sgi_stat_registry below and device_start()'s own registration) -
// reachable non-interactively through gdbstub's existing "monitor"
// passthrough (debuggdbstub.cpp bridges GDB "monitor <text>" straight to
// debugger_console::execute_command()), so a script can explicitly
// bracket "reset, run guest workload, dump" instead of guessing a delay.
// File-scope counters, not class members - avoids the exact ODR mistake
// this session already made once with mc.h (a conditionally-compiled
// member reached from multiple translation units); newport.h is single-
// consumer here but there's no reason to risk it. `MAME_SGI_DIRTY_SOURCE_
// DIAG` still exists purely for the OPTIONAL once-per-second osd_printf_
// info dump further below - off by default, unrelated to whether the
// counters themselves are maintained (they always are now).
#include <chrono>
// #define MAME_SGI_PENDING_RECTS_DIAG
namespace {
#ifdef MAME_SGI_DIRTY_SOURCE_DIAG
	// Per-frame counters for the OPTIONAL MAME_SGI_DIRTY_SOURCE_DIAG print
	// below only - reset to 0 every print (see that block), so NOT safe to
	// reuse as a cumulative/triggered-interval counter. Kept exactly as
	// this diagnostic originally used them.
	uint64_t g_mame_sgi_dirty_pixel_writes_this_frame = 0;
	uint64_t g_mame_sgi_dirty_cursor_calls_this_frame = 0;
#endif

	// Cumulative-since-last-"sgistat reset" counters (2026-09-25) -
	// separate variables from the pair above on purpose,
	// specifically so the optional per-frame print diagnostic's own
	// reset-every-print behavior can never zero these out from under a
	// script that's mid-way through bracketing a measurement interval.
	// g_mame_sgi_stat_rows_walked is the single most direct answer to "did
	// a dirty-rect change actually reduce the compositor's own work",
	// independent of host-side noise (guest CPU emulation, disk/net
	// emulation, other processes) that a total host instruction/cycle
	// count via `perf` can't separate out. See mark_dirty_for_xmap_slot()/
	// screen_update()'s own row-range-building code for where these
	// increment.
#ifdef MAME_SGI_PENDING_RECTS_DIAG
	bool g_pr_print = false;
	std::chrono::steady_clock::time_point g_pr_next{};
#endif
	uint64_t g_mame_sgi_stat_pixel_writes = 0;
	uint64_t g_mame_sgi_stat_cursor_calls = 0;
	uint64_t g_mame_sgi_stat_rows_walked = 0;
	uint64_t g_mame_sgi_stat_screen_update_recomputes = 0;
	uint64_t g_mame_sgi_stat_screen_update_skips = 0;
	uint64_t g_mame_sgi_stat_mark_full_dirty_calls = 0;
	uint64_t g_mame_sgi_stat_mark_xmap_slot_calls = 0;
	// Added 2026-09-27 to check a specific hypothesis: does IRIX's real GL
	// swap path wait on REX3's vertical-retrace interrupt (STATUS_VRINT),
	// such that screen_update() recomputes already land in sync with real
	// guest buffer swaps "for free", without MAME needing to do anything
	// deliberate to synchronize them? vblank_events counts every emulated
	// vblank (vc2_device::vblank_w(1), unconditional); vrint_asserts
	// counts only the subset that actually reached the guest as a real
	// interrupt (newport_base_device::vrint_w(1), gated on VC2's own
	// per-guest VRINT-enable bit) - comparing the two tells us whether
	// the guest has the interrupt enabled at all, and comparing
	// vrint_asserts against screen_update_recomputes tells us whether
	// they're actually correlated 1:1 (consistent with the guest waiting
	// on it before swapping) or not.
	uint64_t g_mame_sgi_stat_vblank_events = 0;
	uint64_t g_mame_sgi_stat_vrint_asserts = 0;
#ifdef MAME_SGI_DIRTY_SOURCE_DIAG
	// Time-throttled (one print per real second), not count-capped - a
	// flat count cap would exhaust itself during early boot's own
	// legitimate dirty activity (PROM/kernel text, real redraws) long
	// before ever reaching the specific later state under investigation
	// (a supposedly-static GUI login screen, many boot-seconds in).
	std::chrono::steady_clock::time_point g_mame_sgi_dirty_diag_next_print{};
#endif
}
// #define MAME_SGI_DIRTY_SOURCE_DIAG

// TEMPORARY diagnostic (2026-09-18) - answers two concrete questions
// about a reported low-resolution-fallback display issue (booting with
// unconfigured/default NVRAM, no console=d1, no ConsoleRes override):
// (1) does vc2_device::update_screen_size()/newport_base_device::
// update_screen_size() ever fire more than once during boot (a genuine
// resolution switch), or does the kernel just inherit whatever the PROM
// programs once, as expected; (2) what pixel mode/size does XMAP9's
// mode table actually carry for the active DID entries once painting
// starts, to separate "low resolution" (VC2 timing, this file) from
// "low bit depth" (XMAP9 mode table, an independent axis) empirically
// instead of by inference. Low call volume expected (a handful of
// resize events at most, one pixel-mode snapshot) - no throttling
// needed.
// #define MAME_SGI_LOWRES_BOOT_DIAG

// TEMPORARY, separate from the above - forces screen_update() to fully
// recompute every single frame (bypasses both the dirty_hold_frames
// early-exit and the dirty-rect row-range clip entirely), to isolate
// testing the top_scanline/vram_row register-interpretation fix from
// the separate dirty-rect coordinate-space question while both are
// still being nailed down live (2026-09-19). Real, measurable
// performance cost (full per-pixel recompute every frame instead of
// only dirtied rows) - never leave this defined in a committed build.
// #define MAME_SGI_LOWRES_BOOT_DIAG_FORCE_FULL_RECOMPUTE

// TEMPORARY diagnostic (2026-09-22) - NOT a real fix.
// mark_full_dirty() always marks the whole 1344x1088 raw-buffer
// region dirty, so screen_update()'s row-range clip (row_start_y/
// row_end_y, below) does nothing while any double-buffered app is
// actively swapping - every frame reprocesses all ~1088 rows through
// the per-pixel DID/XMAP interpretation loop, not just the rows a
// small app window actually occupies. Before building the real
// precise DID/VT-mapped dirty rect (a nontrivial reverse-lookup
// through two levels of indirection),
// this substitutes a hardcoded, deliberately-wrong small rect
// (roughly a single 300x300 GLUT window's footprint, generously
// rounded) wherever mark_full_dirty() would normally use the full
// screen bounds, purely to measure the *ceiling* on possible savings
// via a perf stat A/B against an otherwise-identical run. Wrong for
// any real content outside that rect (would show stale pixels) -
// exists only to get a real before/after number before committing to
// the real fix, per this file's own established measure-before-
// touching discipline.
// MEASUREMENT DONE (2026-09-22): n=5 perf stat A/B
// under sustained test/wave.cpp -db swapping showed -24.4% instructions,
// -17.4% cycles (both p<0.0001) versus the unmodified full-screen path -
// a real, statistically significant ceiling, not noise. The real precise
// DID/VT-mapped dirty rect is now approved to build; leave this #define
// OFF (diagnostic-only, wrong for real content outside its hardcoded
// box) - kept here only as the harness this measurement used, not a
// fallback implementation.
// #define MAME_SGI_DIRTY_RECT_SMALLRECT_EXPERIMENT

// TEMPORARY diagnostic (2026-09-22) - NOT a real fix, and NOT
// the small-rect experiment above. That experiment measured the ceiling
// on possible savings with a deliberately-wrong hardcoded rect; this one
// measures the REAL XMAP-slot-precise fix (mark_dirty_for_xmap_slot(),
// see newport.h) against the pre-fix baseline behavior it replaced
// (mark_full_dirty() unconditionally on every XMAP mode-register write),
// with everything else in the tree held constant - isolates this one
// change's actual measured effect from the earlier ceiling estimate.
// When defined, the three XMAP write call sites in rex3_w() fall back to
// mark_full_dirty() instead of the real per-slot version, reproducing the
// baseline this fix replaced. Built as binary A ("xmap_baseline") for a
// dirtyrect_ab.sh run against an otherwise-identical binary B built with
// this left undefined ("xmap_precise" - the real, currently-shipping
// fix). Leave OFF in any committed build - this exists only as an A/B
// measurement harness, not a real code path choice.
// #define MAME_SGI_XMAP_DIRTY_BASELINE_MEASUREMENT

// TEMPORARY diagnostic (2026-09-22) - NOT a real fix. Prints
// the row range mark_dirty_for_xmap_slot() actually computes (see that
// function's own comment) - directly answers whether the "precise" XMAP
// fix produces a tight row band matching a real window's height, or
// degenerates to near-full-screen for this system's real XMAP slot-
// allocation pattern, rather than inferring it from write frequency
// alone (which turned out to be an unreliable way to reason about this;
// this follows up the XMAP mode-table write-frequency finding noted at
// the VERBOSE definition below).
// #define MAME_SGI_XMAP_SLOT_RANGE_DIAG

// TEMPORARY diagnostic (2026-09-25) - NOT a real fix. Prints
// the actual row_start_y/row_end_y screen_update() uses for a real
// recompute (see that function's own comment) - checks whether some
// OTHER dirty-rect contribution (rb2_dirty from real pixel draws, or the
// vc2/cursor path - both merged into m_pending_dirty every frame via
// frame_dirty, independent of the XMAP fix) is unioning in a broader
// range than mark_dirty_for_xmap_slot()'s own confirmed-narrow
// contribution. Follows up directly on a perf-record profile showing
// screen_update()'s self-time share unchanged between the pre-fix
// baseline and the real fix despite the fix's own row-range being
// independently confirmed tight.
// #define MAME_SGI_ROW_RANGE_USED_DIAG

// TEMPORARY diagnostic (2026-09-25) - NOT a real fix. Prints a
// running histogram, once/second, of how many distinct dirty rects
// m_pending_dirty actually holds per real recompute (0 = full-recompute
// fallback, i.e. m_pending_dirty wasn't valid this call). Answers a
// concrete question before committing to the N=4 choice: for a realistic
// desktop workload (test/wave.cpp -db plus whatever else is on screen -
// a blinking cursor, at minimum), how many separate regions actually
// show up in practice? If real workloads rarely exceed 2, MAX_RECTS=4
// has real headroom; if they regularly hit 4 (and therefore the fold-
// into-cheapest-slot fallback), that's a sign MAX_RECTS should grow.
// #define MAME_SGI_RECT_COUNT_STATS_DIAG

#ifdef MAME_SGI_LOWRES_BOOT_DIAG
#include <unordered_set>
namespace {
	uint32_t g_mame_sgi_lowres_resize_calls = 0;
	uint32_t g_mame_sgi_lowres_pixmode_prints = 0;
	std::unordered_set<uint32_t> g_mame_sgi_lowres_seen_table_entries;
}
#endif

#define LOG_UNKNOWN     (1U << 1)
#define LOG_VC2         (1U << 2)
#define LOG_CMAP        (1U << 3)
#define LOG_XMAP        (1U << 4)
#define LOG_REX3        (1U << 5)
#define LOG_RAMDAC      (1U << 6)
#define LOG_COMMANDS    (1U << 7)
#define LOG_REJECTS     (1U << 8)
#define LOG_ALL         (LOG_UNKNOWN | LOG_VC2 | LOG_CMAP | LOG_XMAP | LOG_REX3 | LOG_RAMDAC | LOG_COMMANDS | LOG_REJECTS)

// VERBOSE/LOG_VC2/LOG_XMAP diagnostic (2026-09-22) used and
// reverted: confirmed live during test/wave.cpp -db's steady swap that
// XMAP mode-table writes fire frequently (~100/s, all to one shared
// slot) while VC2 RAM-word writes never fire at all. Left OFF (0) as the
// default.
#define VERBOSE         (0)
#include "logmacro.h"

namespace {
// MAME_SGI_UNDERLAY_FIX: an underlay pixel shows only where the normal
// planes are zero: the selected buffer's value at the mode's pixel size.
inline bool normal_planes_zero(uint32_t table_entry, uint32_t pix)
{
	const bool buffer1 = BIT(table_entry, 0);
	switch ((table_entry >> 10) & 3)
	{
	case 0:  return ((pix >> (buffer1 ? 4 : 0)) & 0x00f) == 0;
	case 1:  return ((pix >> (buffer1 ? 8 : 0)) & 0x0ff) == 0;
	case 2:  return ((pix >> (buffer1 ? 12 : 0)) & 0xfff) == 0;
	default: return (pix & 0xffffff) == 0;
	}
}
} // anonymous namespace

// TEMPORARY diagnostic (2026-08-17) - passive watch for a candidate
// mechanism behind a reported desktop black-to-white text/icon-color
// regression (the "color race").
// get_default_color() substitutes m_color_vram (the REX3 "VRAM
// Fastclear Color" register - see its own log message a few hundred
// lines below, a genuinely distinct register from the normal explicit
// drawing color) for the caller's real color argument whenever
// draw_mode1 bit 17 is set. If that bit is set for a command that
// should really be using the explicit color (a text/glyph draw, say),
// the pixel color silently comes from whatever was last used for an
// unrelated fast-clear instead - a plausible, Newport-specific
// mechanism for "shape correct, color wrong" with zero pixel-data
// corruption. Only logs when the branch choice is actually
// CONSEQUENTIAL (color_vram != src - if they already match, no
// visible difference exists regardless of which is "correct"), to
// stay low-volume and high-signal rather than firing on every
// legitimate fastclear. Meant to run passively in the background
// during unrelated $f20 debugging - if the color artifact recurs,
// this file will have caught the exact register state and screen
// coordinates at the moment it happened, without needing a dedicated
// repro session. Self-defined here, not via sgi.lua's defines{}
// block, since this file compiles as part of the shared "optional"
// project (see scripts/src/bus.lua), which sgi.lua's SGI-only
// defines{} block never reaches - see the project's own build-verify
// convention for this exact gotcha.
// DISABLED (2026-08-17) - the investigation concluded with no recurrence of
// the color regression ever logged here across the whole investigation.
// The one confirmed trigger (MAME_SGI_LWC1_DELAYSLOT_PROBE's fopen/
// fclose I/O timing perturbation) is being disabled in this same
// cleanup, so this diagnostic would have nothing left to passively
// watch for going forward. The underlying color-race mechanism itself
// remains unconfirmed/unresolved -
// this is diagnostic cleanup, not a claim the regression is fixed.
// Commented out, not deleted - reusable if the symptom recurs.
// #define MAME_SGI_COLOR_SRC_DIAG
#ifdef MAME_SGI_COLOR_SRC_DIAG
static void color_src_diag_log(const char *fmt, ...)
{
	FILE *f = fopen("/home/blythe/mame-sgi/color_src_diag.log", "a");
	if (!f)
		return;
	va_list args;
	va_start(args, fmt);
	vfprintf(f, fmt, args);
	va_end(args);
	fclose(f);
}
#endif

// REX3 writes to the auxiliary planes touch only the enabled plane. MAME
// keeps the CID, popup and overlay planes in one 32-bit word per pixel
// (CID bits 0-1/4-5, popup 2-3/6-7, overlay 8-15/16-23 for buffers 0/1) and
// restricted stores only by the REX3 write mask, so a popup-plane write with a
// wide mask (the 4Dwm menu shadow: popup plane, write mask 0xffffff) also
// landed in the overlay bits. The popup-only erase (write mask 0xcc) then
// left the overlay data behind as a stale rectangle after cascading
// Toolchest menus. On the hardware the planes are separate, so each store is
// limited to the enabled plane's bits.
#define MAME_SGI_REX3_PLANE_WRITE_MASK

// REX3 host data port reads without pixel prefetch. With prefetch disabled
// (DRAWMODE1 bit 26 clear), a GO read of the host data port waits for the
// read it starts and returns that data. Only with prefetch enabled does it
// return the data fetched by the previous GO, which is why drivers that use
// prefetch prime the pipeline with an extra GO before reading. MAME used the
// prefetch behaviour unconditionally, so IRIX's window backing-store save
// (MC DMA reading the data port with GO, prefetch off, no priming GO) stored
// every word one read late: the first word was stale data port contents and
// each saved row began with the previous row's last pixels. Restoring that
// left thin vertical lines along the left edge of menus and dialogs.
#define MAME_SGI_REX3_READ_NO_PREFETCH

// Diagnostic, disabled by default: trace of REX3 host-data (CPU-to-screen)
// transfers. Logs every Host Data Port write (value, access mask, and the
// RWPACKED/RWDOUBLE/HOSTDEPTH state) and every pixel the Span/Block draw
// loops take from host data (position, the command's end x, the per-dispatch
// limit, and m_host_shift before the pixel was extracted). Written to the
// file named by $MAME_SGI_REX3_HOSTDATA_LOG once emulated time reaches
// $MAME_SGI_REX3_HOSTDATA_START seconds (default 0), so a boot can be skipped.
// #define MAME_SGI_REX3_HOSTDATA_DIAG
#ifdef MAME_SGI_REX3_HOSTDATA_DIAG
static FILE *rex3_hostdata_diag_file(running_machine &machine)
{
	static FILE *file = nullptr;
	static bool opened = false;
	static double start = -1.0;
	if (start < 0.0)
	{
		const char *s = getenv("MAME_SGI_REX3_HOSTDATA_START");
		start = s ? atof(s) : 0.0;
	}
	if (machine.time().as_double() < start)
		return nullptr;
	if (!opened)
	{
		opened = true;
		if (const char *path = getenv("MAME_SGI_REX3_HOSTDATA_LOG"))
		{
			// append + line buffering, so another diagnostic (the MC DMA
			// trace) can share the file with lines in true order
			file = fopen(path, "a");
			if (file)
				setvbuf(file, nullptr, _IOLBF, 0);
		}
	}
	return file;
}
#endif

// TEMPORARY DIAGNOSTIC (2026-07-30/31), disabled by default. Times
// newport_base_device::screen_update()'s own CPU cost (the per-pixel
// software compositing loop below) in isolation, to compare directly
// against the separate OSD blit-to-window cost measured alongside it in
// video.cpp - splitting screen_device::vblank_begin's overall cost (found
// to dominate idle-time timer-callback share, 66-75%, via the schedule.cpp
// diagnostic) into "CPU-side framebuffer composition" vs. "push finished
// frame to the real window." Same periodic snapshot-and-reset pattern and
// same disabled-by-default discipline as that diagnostic.
// #define MAME_SGI_NEWPORT_COMPOSITE_TIMING_DIAG
#ifdef MAME_SGI_NEWPORT_COMPOSITE_TIMING_DIAG
namespace {
	uint64_t s_mame_sgi_composite_calls = 0;
	osd_ticks_t s_mame_sgi_composite_ticks = 0;
	osd_ticks_t s_mame_sgi_composite_next_dump = 0;
}
#endif

// Real per-region dirty-rectangle tracking replaced the original
// 2026-07-31 crude whole-board dirty-flag experiment (skip the full
// per-pixel recompute only when NOTHING was written since the last one,
// with no sense of *where*) - see newport_dirty_rect (newport.h),
// rb2_device::store_pixel()/vc2_device::mark_cursor_dirty() for how
// regions get marked, and newport_base_device::screen_update() for how
// they're consumed. The double-buffer flicker the crude experiment hit
// (screen_device's own m_bitmap[2] toggles every frame regardless of
// skip status, so a single-frame dirty mark only refreshes whichever
// buffer is current, leaving the other stale) still applies here and is
// still fixed the same way: any dirty region is held and reapplied for
// 2 consecutive real recomputes before a skip is allowed.

DEFINE_DEVICE_TYPE(XMAP9,      xmap9_device,      "xmap9",      "SGI XMAP9")
DEFINE_DEVICE_TYPE(CMAP,       cmap_device,       "cmap",       "SGI CMAP")
DEFINE_DEVICE_TYPE(VC2,        vc2_device,        "vc2",        "SGI VC2")
DEFINE_DEVICE_TYPE(RB2,        rb2_device,        "rb2",        "SGI RB2")
DEFINE_DEVICE_TYPE(GIO64_XL8,  gio64_xl8_device,  "gio64_xl8",  "SGI 8-bit XL board")
DEFINE_DEVICE_TYPE(GIO64_XL24, gio64_xl24_device, "gio64_xl24", "SGI 24-bit XL board")

/*************************************
 *
 *  XMAP9 Device
 *
 *************************************/

xmap9_device::xmap9_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, XMAP9, tag, owner, clock)
{
}

void xmap9_device::device_start()
{
	save_item(NAME(m_config));
	save_item(NAME(m_revision));
	save_item(NAME(m_fifo_available));
	save_item(NAME(m_cursor_cmap));
	save_item(NAME(m_popup_cmap));
	save_item(NAME(m_mode_table_idx));
	save_item(NAME(m_mode_table));
}

void xmap9_device::device_reset()
{
	m_config = 0;
	m_fifo_available = 0x2;
	m_cursor_cmap = 0;
	m_popup_cmap = 0;
	m_mode_table_idx = 0;
	std::fill(std::begin(m_mode_table), std::end(m_mode_table), 0);
}

void xmap9_device::serialize(FILE *file)
{
	fwrite(&m_config, sizeof(uint32_t), 1, file);
	fwrite(&m_revision, sizeof(uint32_t), 1, file);
	fwrite(&m_fifo_available, sizeof(uint32_t), 1, file);
	fwrite(&m_cursor_cmap, sizeof(uint32_t), 1, file);
	fwrite(&m_popup_cmap, sizeof(uint32_t), 1, file);
	fwrite(&m_mode_table_idx, sizeof(uint32_t), 1, file);
	fwrite(m_mode_table, sizeof(uint32_t), 0x20, file);
}

void xmap9_device::deserialize(FILE *file)
{
	fread(&m_config, sizeof(uint32_t), 1, file);
	fread(&m_revision, sizeof(uint32_t), 1, file);
	fread(&m_fifo_available, sizeof(uint32_t), 1, file);
	fread(&m_cursor_cmap, sizeof(uint32_t), 1, file);
	fread(&m_popup_cmap, sizeof(uint32_t), 1, file);
	fread(&m_mode_table_idx, sizeof(uint32_t), 1, file);
	fread(&m_mode_table, sizeof(uint32_t), 0x20, file);
}

uint32_t xmap9_device::read(uint32_t offset)
{
	switch (offset)
	{
	case 0:
		LOGMASKED(LOG_XMAP, "Config Read: %08x\n", m_config);
		return m_config;
	case 1:
		LOGMASKED(LOG_XMAP, "Revision Read: %08x\n", m_revision);
		return m_revision;
	case 2:
		LOGMASKED(LOG_XMAP, "FIFO Availability Read: %08x\n", m_fifo_available);
		return m_fifo_available;
	case 3:
		LOGMASKED(LOG_XMAP, "Cursor CMAP MSB Read: %08x\n", m_cursor_cmap);
		return m_cursor_cmap;
	case 4:
		LOGMASKED(LOG_XMAP, "Pop Up CMAP MSB Read: %08x\n", m_popup_cmap);
		return m_popup_cmap;
	case 5:
	{
		const uint8_t mode_idx = (m_mode_table_idx & 0x7c) >> 2;
		switch (m_mode_table_idx & 3)
		{
		case 0:
		{
			const uint8_t ret = (uint8_t)(m_mode_table[mode_idx] >> 16);
			LOGMASKED(LOG_XMAP, "Mode Register Read: %02x (Byte 0): %08x\n", mode_idx, ret);
			return ret;
		}
		case 1:
		{
			const uint8_t ret = (uint8_t)(m_mode_table[mode_idx] >> 8);
			LOGMASKED(LOG_XMAP, "Mode Register Read: %02x (Byte 1): %08x\n", mode_idx, ret);
			return ret;
		}
		case 2:
		{
			const uint8_t ret = (uint8_t)m_mode_table[mode_idx];
			LOGMASKED(LOG_XMAP, "Mode Register Read: %02x (Byte 2): %08x\n", mode_idx, ret);
			return ret;
		}
		}
		break;
	}
	case 6:
		LOGMASKED(LOG_XMAP, "Unused Read: %08x\n", 0);
		return 0;
	case 7:
		LOGMASKED(LOG_XMAP, "Mode Table Address Read: %08x\n", m_mode_table_idx);
		return m_mode_table_idx;
	default:
		LOGMASKED(LOG_XMAP | LOG_UNKNOWN, "Unknown DCB Register Select Value: %02x, returning 0\n", offset);
		return 0;
	}
	return 0;
}

void xmap9_device::write(uint32_t offset, uint32_t data)
{
	switch (offset)
	{
	case 0:
		LOGMASKED(LOG_XMAP, "Config Write: %02x\n", (uint8_t)data);
		m_config = (uint8_t)data;
		break;
	case 1:
		LOGMASKED(LOG_XMAP, "Revision Write (Ignored): %02x\n", (uint8_t)data);
		break;
	case 2:
		LOGMASKED(LOG_XMAP, "FIFO Availability Write (Ignored): %02x\n", (uint8_t)data);
		break;
	case 3:
		LOGMASKED(LOG_XMAP, "Cursor CMAP MSB Write: %02x\n", (uint8_t)data);
		m_cursor_cmap = (uint8_t)data;
		break;
	case 4:
		LOGMASKED(LOG_XMAP, "Pop Up CMAP MSB Write: %02x\n", (uint8_t)data);
		m_popup_cmap = (uint8_t)data;
		break;
	case 5:
		LOGMASKED(LOG_XMAP, "Mode Register Write: %02x = %06x\n", data >> 24, data & 0xffffff);
#ifdef MAME_SGI_LOWRES_BOOT_DIAG
		if (m_mode_table[data >> 24] != (data & 0xffffff))
		{
			m_mode_table[data >> 24] = data & 0xffffff;
			osd_printf_info("MAME_SGI_LOWRES_BOOT_DIAG mode_table changed (slot %u):\n", data >> 24);
			for (uint32_t i = 0; i < 0x20; i++)
			{
				const uint32_t e = m_mode_table[i];
				osd_printf_info("  [%u]=%06x pix_mode=%u pix_size=%u\n", i, e, (e >> 8) & 3, (e >> 10) & 3);
			}
		}
#else
		m_mode_table[data >> 24] = data & 0xffffff;
#endif
		break;
	case 6:
		LOGMASKED(LOG_XMAP, "Unused Write (Ignored): %08x\n", data);
		break;
	case 7:
		LOGMASKED(LOG_XMAP, "Mode Table Address Write: %02x\n", (uint8_t)data);
		m_mode_table_idx = (uint8_t)data;
		break;
	default:
		LOGMASKED(LOG_XMAP | LOG_UNKNOWN, "Unknown DCB Register Select Value: %02x = %08x\n", offset, data);
		break;
	}
}


/*************************************
 *
 *  CMAP Device
 *
 *************************************/

cmap_device::cmap_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, CMAP, tag, owner, clock)
	, device_palette_interface(mconfig, *this)
{
}

void cmap_device::device_start()
{
	save_item(NAME(m_status));
	save_item(NAME(m_revision));
	save_item(NAME(m_palette_idx));
	save_item(NAME(m_palette));
}

void cmap_device::device_reset()
{
	m_status = 8;
	m_palette_idx = 0;
	std::fill(std::begin(m_palette), std::end(m_palette), 0);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	m_dirty_lo = 0;
	m_dirty_hi = 0xffff;
#endif
}

void cmap_device::serialize(FILE *file)
{
	fwrite(&m_status, sizeof(uint32_t), 1, file);
	fwrite(&m_revision, sizeof(uint32_t), 1, file);
	fwrite(&m_palette_idx, sizeof(uint16_t), 1, file);
	fwrite(m_palette, sizeof(uint32_t), 0x10000, file);
}

void cmap_device::deserialize(FILE *file)
{
	fread(&m_status, sizeof(uint32_t), 1, file);
	fread(&m_revision, sizeof(uint32_t), 1, file);
	fread(&m_palette_idx, sizeof(uint16_t), 1, file);
	fread(m_palette, sizeof(uint32_t), 0x10000, file);
}

// Diagnostic: log CMAP palette writes to entries 0x1800 and up or of pure
// green, and writes to unhandled registers, to stderr as CMAP_TRACE lines.
//#define MAME_SGI_CMAP_WRITE_TRACE

void cmap_device::write(uint32_t offset, uint32_t data)
{
	switch (offset)
	{
	case 0x00:
		LOGMASKED(LOG_CMAP, "Palette Index Write: %04x\n", data & 0xffff);
		m_palette_idx = (uint16_t)data;
		break;
	case 0x02:
#ifdef MAME_SGI_CMAP_WRITE_TRACE
		if (m_palette_idx >= 0x1800 || ((data >> 8) & 0xffffff) == 0x00ff00)
			fprintf(stderr, "CMAP_TRACE %s idx=%04x data=%06x\n", tag(), m_palette_idx, (data >> 8) & 0xffffff);
#endif
		m_palette[m_palette_idx] = data >> 8;
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
		m_dirty_lo = std::min<uint32_t>(m_dirty_lo, m_palette_idx);
		m_dirty_hi = std::max<uint32_t>(m_dirty_hi, m_palette_idx);
#endif
		if (m_palette_idx < 0x2000)
			set_pen_color(m_palette_idx, rgb_t((uint8_t)(data >> 24), (uint8_t)(data >> 16), (uint8_t)(data >> 8)));
		LOGMASKED(LOG_CMAP, "Palette Entry %04x Write: %08x\n", m_palette_idx, data >> 8);
		break;
	case 0x03:
		// Command register. The guest writes 7 before and 3 after each
		// palette write (bit 2 disables CBLANK while the palette is
		// written) and 3 also sets the palette FIFO to trip at half full.
		// Neither changes the palette contents or how they are displayed,
		// so the emulation ignores it.
		break;
	default:
#ifdef MAME_SGI_CMAP_WRITE_TRACE
		fprintf(stderr, "CMAP_TRACE %s unknown reg %d data=%08x idx=%04x\n", tag(), offset, data, m_palette_idx);
#endif
		LOGMASKED(LOG_CMAP | LOG_UNKNOWN, "Unknown Register %d Write: %08x\n", offset, data);
		break;
	}
}

uint32_t cmap_device::read(uint32_t offset)
{
	switch (offset)
	{
	case 0x04:
		LOGMASKED(LOG_CMAP, "Status Read: %08x\n", m_status);
		return m_status;
	case 0x06:
		LOGMASKED(LOG_CMAP, "Revision Read: %08x\n", m_revision);
		return m_revision;
	default:
		LOGMASKED(LOG_CMAP | LOG_UNKNOWN, "Unknown Register %d Read\n", offset);
		return 0;
	}
}


/*************************************
 *
 *  VC2 Device
 *
 *************************************/

/*static*/ const size_t vc2_device::RAM_SIZE = 0x8000;

vc2_device::vc2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, VC2, tag, owner, clock)
	, m_vert_int(*this)
	, m_screen_timing_changed(*this)
{
}

void vc2_device::device_start()
{
	m_ram = std::make_unique<uint16_t[]>(RAM_SIZE);
	m_vt_table = make_unique_clear<uint32_t[]>(2048 * 2048);

	save_item(NAME(m_vid_entry));
	save_item(NAME(m_cursor_entry));
	save_item(NAME(m_cursor_x));
	save_item(NAME(m_cursor_y));
	save_item(NAME(m_cur_cursor_x));
	save_item(NAME(m_did_entry));
	save_item(NAME(m_scanline_len));
	save_item(NAME(m_ram_addr));
	save_item(NAME(m_vt_frame_ptr));
	save_item(NAME(m_vt_line_ptr));
	save_item(NAME(m_vt_line_run));
	save_item(NAME(m_vt_line_count));
	save_item(NAME(m_cursor_table_ptr));
	save_item(NAME(m_work_cursor_y));
	save_item(NAME(m_did_frame_ptr));
	save_item(NAME(m_did_line_ptr));
	save_item(NAME(m_display_ctrl));
	save_item(NAME(m_config));
	save_item(NAME(m_reg_idx));
	save_item(NAME(m_reg_data));
	save_pointer(NAME(&m_ram[0]), RAM_SIZE);

	save_pointer(NAME(&m_vt_table[0]), 2048 * 2048);

	save_item(NAME(m_readout_x0));
	save_item(NAME(m_readout_y0));
	save_item(NAME(m_readout_x1));
	save_item(NAME(m_readout_y1));

	save_item(NAME(m_enable_cursor));

	// m_dirty (the compositor's dirty-rectangle accumulator, see
	// mark_cursor_dirty()'s own comment) is deliberately not save_item()'d -
	// it's a pure rendering-optimization cache, not real emulated hardware
	// state. That means a -state load restores m_cursor_x/y/m_enable_cursor
	// correctly but leaves m_dirty exactly as freshly-constructed (empty),
	// so the cursor's on-screen region never gets marked for a redraw -
	// nothing else drives it dirty except a live register write, which the
	// guest has no reason to reissue for state it already believes is
	// current. Re-derive the dirty contribution once after any load so the
	// compositor picks up the cursor's just-restored position/state on the
	// next frame instead of never drawing it at all.
	machine().save().register_postload(save_prepost_delegate(FUNC(vc2_device::mark_cursor_dirty), this));
}

void vc2_device::device_reset()
{
	m_vid_entry = 0;
	m_cursor_entry = 0;
	m_cursor_x = 0;
	m_cursor_y = 0;
	m_cur_cursor_x = 0;
	m_did_entry = 0;
	m_scanline_len = 0;
	m_ram_addr = 0;
	m_vt_frame_ptr = 0;
	m_vt_line_ptr = 0;
	m_vt_line_run = 0;
	m_vt_line_count = 0;
	m_cursor_table_ptr = 0;
	m_work_cursor_y = 0;
	m_did_frame_ptr = 0;
	m_did_line_ptr = 0;
	m_display_ctrl = 0;
	m_config = 0;
	m_reg_idx = 0;
	m_reg_data = 0;
	memset(&m_ram[0], 0, sizeof(uint16_t) * RAM_SIZE);

	memset(&m_vt_table[0], 0, sizeof(uint32_t) * 2048 * 2048);

	m_readout_x0 = 0;
	m_readout_y0 = 0;
	m_readout_x1 = 0;
	m_readout_y1 = 0;

	m_enable_cursor = false;

	m_dirty.clear();
}

void vc2_device::serialize(FILE *file)
{
	fwrite(&m_vid_entry, sizeof(uint16_t), 1, file);
	fwrite(&m_cursor_entry, sizeof(uint16_t), 1, file);
	fwrite(&m_cursor_x, sizeof(uint16_t), 1, file);
	fwrite(&m_cursor_y, sizeof(uint16_t), 1, file);
	fwrite(&m_cur_cursor_x, sizeof(uint16_t), 1, file);
	fwrite(&m_did_entry, sizeof(uint16_t), 1, file);
	fwrite(&m_scanline_len, sizeof(uint16_t), 1, file);
	fwrite(&m_ram_addr, sizeof(uint16_t), 1, file);
	fwrite(&m_vt_frame_ptr, sizeof(uint16_t), 1, file);
	fwrite(&m_vt_line_ptr, sizeof(uint16_t), 1, file);
	fwrite(&m_vt_line_run, sizeof(uint16_t), 1, file);
	fwrite(&m_vt_line_count, sizeof(uint16_t), 1, file);
	fwrite(&m_cursor_table_ptr, sizeof(uint16_t), 1, file);
	fwrite(&m_work_cursor_y, sizeof(uint16_t), 1, file);
	fwrite(&m_did_frame_ptr, sizeof(uint16_t), 1, file);
	fwrite(&m_did_line_ptr, sizeof(uint16_t), 1, file);
	fwrite(&m_display_ctrl, sizeof(uint16_t), 1, file);
	fwrite(&m_config, sizeof(uint16_t), 1, file);
	fwrite(&m_reg_idx, sizeof(uint8_t), 1, file);
	fwrite(&m_ram[0], sizeof(uint16_t), RAM_SIZE, file);
	fwrite(&m_vt_table[0], sizeof(uint32_t), 2048 * 2048, file);
	fwrite(&m_readout_x0, sizeof(int), 1, file);
	fwrite(&m_readout_y0, sizeof(int), 1, file);
	fwrite(&m_readout_x1, sizeof(int), 1, file);
	fwrite(&m_readout_y1, sizeof(int), 1, file);
	fwrite(&m_enable_cursor, sizeof(bool), 1, file);
}

void vc2_device::deserialize(FILE *file)
{
	fread(&m_vid_entry, sizeof(uint16_t), 1, file);
	fread(&m_cursor_entry, sizeof(uint16_t), 1, file);
	fread(&m_cursor_x, sizeof(uint16_t), 1, file);
	fread(&m_cursor_y, sizeof(uint16_t), 1, file);
	fread(&m_cur_cursor_x, sizeof(uint16_t), 1, file);
	fread(&m_did_entry, sizeof(uint16_t), 1, file);
	fread(&m_scanline_len, sizeof(uint16_t), 1, file);
	fread(&m_ram_addr, sizeof(uint16_t), 1, file);
	fread(&m_vt_frame_ptr, sizeof(uint16_t), 1, file);
	fread(&m_vt_line_ptr, sizeof(uint16_t), 1, file);
	fread(&m_vt_line_run, sizeof(uint16_t), 1, file);
	fread(&m_vt_line_count, sizeof(uint16_t), 1, file);
	fread(&m_cursor_table_ptr, sizeof(uint16_t), 1, file);
	fread(&m_work_cursor_y, sizeof(uint16_t), 1, file);
	fread(&m_did_frame_ptr, sizeof(uint16_t), 1, file);
	fread(&m_did_line_ptr, sizeof(uint16_t), 1, file);
	fread(&m_display_ctrl, sizeof(uint16_t), 1, file);
	fread(&m_config, sizeof(uint16_t), 1, file);
	fread(&m_reg_idx, sizeof(uint8_t), 1, file);
	fread(&m_ram[0], sizeof(uint16_t), RAM_SIZE, file);
	fread(&m_vt_table[0], sizeof(uint32_t), 2048 * 2048, file);
	fread(&m_readout_x0, sizeof(int), 1, file);
	fread(&m_readout_y0, sizeof(int), 1, file);
	fread(&m_readout_x1, sizeof(int), 1, file);
	fread(&m_readout_y1, sizeof(int), 1, file);
	fread(&m_enable_cursor, sizeof(bool), 1, file);
}

void vc2_device::mark_cursor_dirty()
{
	g_mame_sgi_stat_cursor_calls++;
#ifdef MAME_SGI_DIRTY_SOURCE_DIAG
	g_mame_sgi_dirty_cursor_calls_this_frame++;
#endif
#ifdef MAME_SGI_MOUSEFOCUS_DIAG
	osd_printf_verbose("MOUSEFOCUS_DIAG: mark_cursor_dirty() called, m_enable_cursor=%d m_cursor_x=%d m_cursor_y=%d\n",
		(int)m_enable_cursor, (int)m_cursor_x, (int)m_cursor_y);
#endif

	if (!m_enable_cursor)
		return;

	int x0, y0, x1, y1;
	cursor_bounds(x0, y0, x1, y1);
#ifdef MAME_SGI_MOUSEFOCUS_DIAG
	osd_printf_verbose("MOUSEFOCUS_DIAG: mark_cursor_dirty() bounds x0=%d y0=%d x1=%d y1=%d\n", x0, y0, x1, y1);
#endif
	m_dirty.add_rect(x0, y0, x1, y1);
}

void vc2_device::write(uint32_t offset, uint32_t data, uint32_t mem_mask)
{
	// The cursor is composited on-the-fly by newport_base_device's
	// screen_update() from this device's live state - unlike everything
	// else here, it never touches rb2_device's framebuffer, so a cursor
	// position/content/enable change needs its own dirty contribution.
	// Captured before any mutation below so the OLD position's box (not
	// just the new one) gets marked - otherwise the cursor would leave a
	// visible ghost at its previous position once it moves, since nothing
	// else would ever tell the compositor that region needs a fresh
	// (now cursor-free) recompute. Only actually applied for writes that
	// can plausibly affect the cursor (register cases below, or a RAM
	// word write that might touch its bitmap) - see cursor_relevant.
	const uint16_t old_cursor_x = m_cursor_x;
	const uint16_t old_cursor_y = m_cursor_y;
	const bool old_enable_cursor = m_enable_cursor;
	bool cursor_relevant = false;

	if (mem_mask == 0x000000ff)
	{
		// Register Select
		if (offset == 0)
		{
			m_reg_idx = (uint8_t)data;
			LOGMASKED(LOG_VC2, "Register Select: %02x\n", m_reg_idx);
		}
		else
		{
			LOGMASKED(LOG_VC2 | LOG_UNKNOWN, "Unknown Register Select: Register %02x, data = %08x\n", offset, data);
		}
	}
	else if (mem_mask == 0x0000ffff)
	{
		// RAM Word
		if (offset == 3)
		{
			LOGMASKED(LOG_VC2, "RAM Write: %04x = %08x\n", m_ram_addr, (uint16_t)data);
			// Could be writing into the cursor's own bitmap - not worth
			// tracking the exact address range precisely (rare event,
			// and RAM also holds the DID/VT tables), so conservatively
			// treat any RAM write as cursor-relevant when the cursor is
			// currently enabled.
			cursor_relevant = m_enable_cursor;
			m_ram[m_ram_addr] = (uint16_t)data;
			m_ram_addr++;
			if (m_ram_addr >= RAM_SIZE)
			{
				m_ram_addr = 0;
			}
		}
		else
		{
			LOGMASKED(LOG_VC2 | LOG_UNKNOWN, "Unknown Word Write: Register %02x, data = %08x\n", offset, data);
		}
	}
	else if (mem_mask == 0xffffffff)
	{
		// Register Write
		if (offset == 0)
		{
			LOGMASKED(LOG_VC2, "Register Setup:\n");
			m_reg_idx = data >> 24;
			m_reg_data = (uint16_t)(data >> 8);
			switch (m_reg_idx)
			{
				case 0x00:
					m_vid_entry = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Video Entry Pointer, %04x\n", m_vid_entry);
					update_screen_size();
					break;
				case 0x01:
					m_cursor_entry = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Cursor Entry Pointer, %04x\n", m_cursor_entry);
					cursor_relevant = true;
					break;
				case 0x02:
					m_cursor_x = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Cursor X, %04x\n", m_cursor_x);
					cursor_relevant = true;
					break;
				case 0x03:
					m_cursor_y = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Cursor Y, %04x\n", m_cursor_y);
					m_cur_cursor_x = m_cursor_x;
					cursor_relevant = true;
					break;
				case 0x04:
					m_cur_cursor_x = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Current Cursor X, %04x\n", m_cur_cursor_x);
					break;
				case 0x05:
					m_did_entry = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: DID Entry Pointer, %04x\n", m_did_entry);
					break;
				case 0x06:
					m_scanline_len = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Scanline Length, %04x\n", m_scanline_len);
					break;
				case 0x07:
					m_ram_addr = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: RAM Address, %04x\n", m_ram_addr);
					break;
				case 0x08:
					m_vt_frame_ptr = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: VT Frame Table Ptr, %04x\n", m_vt_frame_ptr);
					break;
				case 0x09:
					m_vt_line_ptr = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: VT Line Sequence Pointer, %04x\n", m_vt_line_ptr);
					break;
				case 0x0a:
					m_vt_line_run = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: VT Lines in Run, %04x\n", m_vt_line_run);
					break;
				case 0x0b:
					m_vt_line_count = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Vertical Line Count, %04x\n", m_vt_line_count);
					break;
				case 0x0c:
					m_cursor_table_ptr = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Cursor Table Pointer, %04x\n", m_cursor_table_ptr);
					break;
				case 0x0d:
					m_work_cursor_y = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Working Cursor Y, %04x\n", m_work_cursor_y);
					break;
				case 0x0e:
					m_did_frame_ptr = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: DID Frame Table Pointer, %04x\n", m_did_frame_ptr);
					break;
				case 0x0f:
					m_did_line_ptr = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: DID Line Table Pointer, %04x\n", m_did_line_ptr);
					break;
				case 0x10:
					m_display_ctrl = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Display Control, %04x\n", m_display_ctrl);
					m_enable_cursor = BIT(m_display_ctrl, CURSOR_FUNC_ENABLE_BIT) != 0
								   && BIT(m_display_ctrl, CURSOR_ENABLE_BIT) != 0
								   && BIT(m_display_ctrl, CURSOR_MODE_BIT) == CURSOR_MODE_GLYPH;
					cursor_relevant = true;
					break;
				case 0x1f:
					m_config = m_reg_data;
					LOGMASKED(LOG_VC2, "Register Write: Configuration, %04x\n", m_config);
					break;
				default:
					LOGMASKED(LOG_VC2 | LOG_UNKNOWN, "Register Write: Unknown Register: %02x = %04x\n", m_reg_idx, m_reg_data);
					break;
			}
		}
		else
		{
			LOGMASKED(LOG_VC2 | LOG_UNKNOWN, "Unknown Register Write: %02x = %08x\n", offset, data);
		}
	}
	else
	{
		LOGMASKED(LOG_VC2 | LOG_UNKNOWN, "Unknown Memory Mask: %08x, Offset %02x, Value %08x\n", mem_mask, offset, data);
	}

	if (cursor_relevant)
	{
		if (old_enable_cursor)
		{
			// Restore the pre-write cursor position/enable state before
			// marking the old box - mark_cursor_dirty() reads current
			// state (including m_enable_cursor, not just position), so
			// temporarily put ALL of it back rather than duplicating the
			// box-math here. Swapping enable too matters: without it, a
			// write that both moves the cursor AND disables it in the
			// same call would skip marking the old position entirely,
			// since mark_cursor_dirty() would see the already-updated
			// (now false) m_enable_cursor and bail out.
			const uint16_t new_cursor_x = m_cursor_x;
			const uint16_t new_cursor_y = m_cursor_y;
			const bool new_enable_cursor = m_enable_cursor;
			m_cursor_x = old_cursor_x;
			m_cursor_y = old_cursor_y;
			m_enable_cursor = old_enable_cursor;
			mark_cursor_dirty();
			m_cursor_x = new_cursor_x;
			m_cursor_y = new_cursor_y;
			m_enable_cursor = new_enable_cursor;
		}
		mark_cursor_dirty();
	}
}

uint32_t vc2_device::read(uint32_t offset)
{
	switch (offset)
	{
	case 0x01: /* Register Read */
		switch (m_reg_idx)
		{
			case 0x00:
				LOGMASKED(LOG_VC2, "Register Read: Video Entry Pointer, %08x\n", m_vid_entry);
				return m_vid_entry;
			case 0x01:
				LOGMASKED(LOG_VC2, "Register Read: Cursor Entry Pointer, %08x\n", m_cursor_entry);
				return m_cursor_entry;
			case 0x02:
				LOGMASKED(LOG_VC2, "Register Read: Cursor X, %08x\n", m_cursor_x);
				return m_cursor_x;
			case 0x03:
				LOGMASKED(LOG_VC2, "Register Read: Cursor Y, %08x\n", m_cursor_y);
				return m_cursor_y;
			case 0x04:
				LOGMASKED(LOG_VC2, "Register Read: Current Cursor X, %08x\n", m_cur_cursor_x);
				return m_cur_cursor_x;
			case 0x05:
				LOGMASKED(LOG_VC2, "Register Read: DID Entry, %08x\n", m_did_entry);
				return m_did_entry;
			case 0x06:
				LOGMASKED(LOG_VC2, "Register Read: Scanline Length, %08x\n", m_scanline_len);
				return m_scanline_len;
			case 0x07:
				LOGMASKED(LOG_VC2, "Register Read: RAM Address, %08x\n", m_ram_addr);
				return m_ram_addr;
			case 0x08:
				LOGMASKED(LOG_VC2, "Register Read: VT Frame Pointer, %08x\n", m_vt_frame_ptr);
				return m_vt_frame_ptr;
			case 0x09:
				LOGMASKED(LOG_VC2, "Register Read: VT Line Sequence Pointer, %08x\n", m_vt_line_ptr);
				return m_vt_line_ptr;
			case 0x0a:
				LOGMASKED(LOG_VC2, "Register Read: VT Lines in Run, %08x\n", m_vt_line_run);
				return m_vt_line_run;
			case 0x0b:
				LOGMASKED(LOG_VC2, "Register Read: Vertical Line Count, %08x\n", m_vt_line_count);
				return m_vt_line_count;
			case 0x0c:
				LOGMASKED(LOG_VC2, "Register Read: Cursor Table Pointer, %08x\n", m_cursor_table_ptr);
				return m_cursor_table_ptr;
			case 0x0d:
				LOGMASKED(LOG_VC2, "Register Read: Working Cursor Y, %08x\n", m_work_cursor_y);
				return m_work_cursor_y;
			case 0x0e:
				LOGMASKED(LOG_VC2, "Register Read: DID Frame Pointer, %08x\n", m_did_frame_ptr);
				return m_did_frame_ptr;
			case 0x0f:
				LOGMASKED(LOG_VC2, "Register Read: DID Line Pointer, %08x\n", m_did_line_ptr);
				return m_did_line_ptr;
			case 0x10:
				LOGMASKED(LOG_VC2, "Register Read: Display Control, %08x\n", m_display_ctrl);
				return m_display_ctrl;
			case 0x1f:
				LOGMASKED(LOG_VC2, "Register Read: Configuration, %08x\n", m_config);
				return m_config;
			default:
				return 0;
		}
		break;
	case 0x03: /* RAM Read */
	{
		LOGMASKED(LOG_VC2, "RAM Read: %04x = %08x\n", m_ram_addr, m_ram[m_ram_addr]);
		uint16_t ret = m_ram[m_ram_addr];
		m_ram_addr++;
		if (m_ram_addr >= RAM_SIZE)
		{
			m_ram_addr = 0;
		}
		return ret;
	}
	default:
		LOGMASKED(LOG_VC2 | LOG_UNKNOWN, "Unknown Register Read: %02x\n", offset);
		return 0;
	}
}

uint16_t vc2_device::begin_did_line(int y)
{
	m_did_frame_ptr = m_did_entry + (uint16_t)y;
	m_did_line_ptr = m_ram[m_did_frame_ptr];
	return m_ram[m_did_line_ptr];
}

uint16_t vc2_device::next_did_line_entry()
{
	m_did_line_ptr++;
	return m_ram[m_did_line_ptr];
}

void vc2_device::decode_vt_line(uint32_t line, uint32_t line_seq_ptr)
{
	bool eol_sa = false;
	bool eol_sb = false;
	uint32_t vt_entry = 0;
	uint32_t i = 0;
	do
	{
		vt_entry &= ~(0x7f << 14);
		vt_entry |= (m_ram[line_seq_ptr] & 0x7f) << 14;
		eol_sa = BIT(m_ram[line_seq_ptr], 15);
		if (!BIT(m_ram[line_seq_ptr], 7))
		{
			eol_sb = BIT(m_ram[line_seq_ptr + 1], 15);
			vt_entry &= (0x7f << 14);
			vt_entry |= (m_ram[line_seq_ptr + 1] & 0x7f00) >> 1;
			vt_entry |= m_ram[line_seq_ptr + 1] & 0x7f;
			line_seq_ptr += 2;
		}
		else
		{
			eol_sb = true;
			line_seq_ptr++;
		}
		for (uint32_t j = 0; j < ((m_ram[line_seq_ptr] & 0x7f00) >> 7); i++, j++)
		{
			const uint8_t sa = ((vt_entry >> 14) & 0x7f) << 1;
			const uint8_t sb = ((vt_entry >> 7) & 0x7f) << 1;
			const uint8_t sc = ((vt_entry >> 0) & 0x7f) << 1;
			m_vt_table[line * 2048 + i] = 0xff000000 | (sa << 16) | (sb << 8) | sc;
		}
	} while (!(eol_sa && eol_sb));
}

void vc2_device::decode_vt_table()
{
	memset(&m_vt_table[0], 0, sizeof(uint32_t) * 2048 * 2048);
	uint32_t curr_vt_entry = m_vid_entry;
	uint32_t line_counter = 0;
	uint32_t line_seq_len = 0;
	do
	{
		const uint32_t line_seq_ptr = m_ram[curr_vt_entry];
		line_seq_len = m_ram[curr_vt_entry + 1];
		if (line_seq_len)
		{
			for (uint32_t i = 0; i < line_seq_len; i++, line_counter++)
			{
				decode_vt_line(line_counter, line_seq_ptr);
			}
		}
		curr_vt_entry += 2;
	} while (line_seq_len != 0);
}

void vc2_device::update_screen_size()
{
	decode_vt_table();

	bool x_started = false;
	bool y_started = false;
	bool x_done = false;
	bool y_done = false;
	bool done = false;
	m_readout_x0 = 0;
	m_readout_y0 = 0;
	m_readout_x1 = 0;
	m_readout_y1 = 0;

	for (int y = 0; y < 2048 && !done; y++)
	{
		uint32_t *src = &m_vt_table[y * 2048];
		for (int x = 0; x < 2048 && !done; x++)
		{
			if (BIT(*src, 7))
			{
				if (!x_started)
				{
					x_started = true;
					m_readout_x0 = x;
				}
				if (!y_started)
				{
					y_started = true;
					m_readout_y0 = y;
				}
			}
			else
			{
				if (x_started && !x_done)
				{
					m_readout_x1 = x;
					x_done = true;
				}
				if (y_started && !y_done && x == m_readout_x0)
				{
					m_readout_y1 = y;
					y_done = true;
				}
			}
			done = x_done && y_done;
			src++;
		}
	}

	m_screen_timing_changed(1);
}

bool vc2_device::is_cursor_active(int x, int y)
{
	return x >= (m_cursor_x - 31) && x <= m_cursor_x && y >= (m_cursor_y - 31) && y <= m_cursor_y && m_enable_cursor;
}

uint8_t vc2_device::get_cursor_pixel(int x, int y)
{
	if (!is_cursor_active(x, y))
		return 0;

	x -= (int)m_cursor_x - 31;
	y -= (int)m_cursor_y - 31;

	if (x < 0 || y < 0)
		return 0;

	bool monochrome_cursor = BIT(m_display_ctrl, CURSOR_SIZE_BIT) == CURSOR_SIZE_64;

	int size = monochrome_cursor ? 64 : 32;
	if (x >= size || y >= size)
		return 0;

	const int shift = 15 - (x % 16);

	if (monochrome_cursor)
	{
		const int address = y * 4 + (x / 16);
		const uint16_t word = m_ram[m_cursor_entry + address];
		return BIT(word, shift);
	}
	else
	{
		const int address = y * 2 + (x / 16);
		const uint16_t word0 = m_ram[m_cursor_entry + address];
		const uint16_t word1 = m_ram[m_cursor_entry + address + 64];
		return BIT(word0, shift) | (BIT(word1, shift) << 1);
	}
}

void vc2_device::vblank_w(int state)
{
	if (state)
	{
		g_mame_sgi_stat_vblank_events++;
		if (BIT(m_display_ctrl, 0))
		{
			m_vert_int(1);
		}
	}
}


/*************************************
 *
 *  RB2 Device
 *
 *************************************/

/*static*/ const size_t rb2_device::BUFFER_SIZE = (1280 + 64) * (1024 + 64);

rb2_device::rb2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, RB2, tag, owner, clock)
{
}

void rb2_device::device_start()
{
	m_rgbci = std::make_unique<uint32_t[]>(BUFFER_SIZE);
	m_cidaux = std::make_unique<uint32_t[]>(BUFFER_SIZE);

	save_item(NAME(m_global_mask));
	save_item(NAME(m_write_mask));
	save_item(NAME(m_blend));
	save_item(NAME(m_fast_clear));
	save_item(NAME(m_rgbmode));
	save_item(NAME(m_dblsrc));
	save_item(NAME(m_plane_enable));
	save_item(NAME(m_draw_depth));
	save_item(NAME(m_draw_bpp));
	save_item(NAME(m_logicop));
	save_item(NAME(m_src_shift));
	save_item(NAME(m_dst_shift));

	save_pointer(NAME(&m_rgbci[0]), BUFFER_SIZE);
	save_pointer(NAME(&m_cidaux[0]), BUFFER_SIZE);

	machine().save().register_postload(save_prepost_delegate(FUNC(rb2_device::update_write_mask), this));
}

void rb2_device::device_reset()
{
	m_write_mask = 0;
	m_write_mask_eff = 0;
	m_blend = false;
	m_fast_clear = false;
	m_rgbmode = false;
	m_dblsrc = false;
	m_plane_enable = 0;
	m_draw_depth = 0;
	m_draw_bpp = 8;
	m_logicop = 0;
	m_src_shift = 0;
	m_dst_shift = 0;

	m_dest_buf = &m_rgbci[0];
	m_buf_ptr = &m_rgbci[0];

	m_cur_x = 0;
	m_cur_y = 0;
	m_dirty.clear();
}

void rb2_device::serialize(FILE *file)
{
	fwrite(&m_global_mask, sizeof(uint32_t), 1, file);
	fwrite(&m_write_mask, sizeof(uint32_t), 1, file);
	fwrite(&m_blend, sizeof(bool), 1, file);
	fwrite(&m_fast_clear, sizeof(bool), 1, file);
	fwrite(&m_rgbmode, sizeof(bool), 1, file);
	fwrite(&m_dblsrc, sizeof(bool), 1, file);
	fwrite(&m_plane_enable, sizeof(uint8_t), 1, file);
	fwrite(&m_draw_depth, sizeof(uint8_t), 1, file);
	fwrite(&m_logicop, sizeof(uint8_t), 1, file);
	fwrite(&m_src_shift, sizeof(uint8_t), 1, file);
	fwrite(&m_dst_shift, sizeof(uint8_t), 1, file);
}

void rb2_device::deserialize(FILE *file)
{
	fread(&m_global_mask, sizeof(uint32_t), 1, file);
	fread(&m_write_mask, sizeof(uint32_t), 1, file);
	fread(&m_blend, sizeof(bool), 1, file);
	fread(&m_fast_clear, sizeof(bool), 1, file);
	fread(&m_rgbmode, sizeof(bool), 1, file);
	fread(&m_dblsrc, sizeof(bool), 1, file);
	fread(&m_plane_enable, sizeof(uint8_t), 1, file);
	fread(&m_draw_depth, sizeof(uint8_t), 1, file);
	fread(&m_logicop, sizeof(uint8_t), 1, file);
	fread(&m_src_shift, sizeof(uint8_t), 1, file);
	fread(&m_dst_shift, sizeof(uint8_t), 1, file);
}

uint32_t rb2_device::expand_to_all_lanes(uint32_t src)
{
	switch (m_draw_depth)
	{
	case 0:
		src |= src << 4;
		src |= src << 8;
		src |= src << 16;
		break;
	case 1:
		src |= src << 8;
		src |= src << 16;
		break;
	case 2:
		src |= src << 12;
		break;
	case 3:
		break;
	}
	switch (m_plane_enable)
	{
	case 1: // RGB/CI
	case 2: // RGBA
	case 6: // CID
		return src;
	case 4: // OLAY
		return src << 8;
	case 5: // PUP
		return src << 2;
	}
	return src;
}

#ifdef MAME_SGI_REX3_FAST_PIXEL
// Same result and RB2 state as set_address_xy(x, y) then write_pixel(data).
ATTR_FORCE_INLINE inline void rb2_device::write_pixel_xy(int x, int y, uint32_t data)
{
#ifdef MAME_SGI_REX3_FAST_PIXEL_VERIFY
	static uint64_t s_verify_pixels, s_verify_bad;
	uint32_t *const vp = &m_dest_buf[y * (1280 + 64) + x];
	const uint32_t before = *vp;
	set_address_xy(x, y);
	write_pixel(data);
	const uint32_t expected = *vp;
	*vp = before;
#endif
	uint32_t *const p = &m_dest_buf[y * (1280 + 64) + x];
	m_buf_ptr = p;
	m_cur_x = x;
	m_cur_y = y;

	const uint32_t src = expand_to_all_lanes(data);
	uint32_t value;
	if (m_blend || m_fast_clear)
	{
		value = src;
	}
	else
	{
		switch (m_logicop)
		{
			case 0:     value = 0x000000;        break;
			case 3:     value = src;             break;
			case 12:    value = ~src;            break;
			case 15:    value = 0xffffff;        break;
			default:
			{
				const uint32_t dst = *p >> m_src_shift;
				switch (m_logicop)
				{
					case 1:     value = src & dst;       break;
					case 2:     value = src & ~dst;      break;
					case 4:     value = ~src & dst;      break;
					case 5:     value = dst;             break;
					case 6:     value = src ^ dst;       break;
					case 7:     value = src | dst;       break;
					case 8:     value = ~(src | dst);    break;
					case 9:     value = ~(src ^ dst);    break;
					case 10:    value = ~dst;            break;
					case 11:    value = src | ~dst;      break;
					case 13:    value = ~src | dst;      break;
					default:    value = ~(src & dst);    break; // 14
				}
				break;
			}
		}
	}

#ifdef MAME_SGI_REX3_PLANE_WRITE_MASK
	static const uint32_t s_plane_bits[8] = {
		0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffff00, 0x000000cc, 0x00000033, 0xffffffff };
	const uint32_t write_mask = m_write_mask_eff & m_global_mask & s_plane_bits[m_plane_enable & 7];
#else
	const uint32_t write_mask = m_write_mask_eff & m_global_mask;
#endif
	// VRAM bits outside the global mask are never written and stay zero, so
	// a write mask covering the whole global mask needs no read of the old
	// word
	if (write_mask == m_global_mask)
		*p = value & write_mask;
	else
		*p = (*p & ~write_mask) | (value & write_mask);

	if (m_run_valid && y == m_run_y && x >= m_run_x0 - 1 && x <= m_run_x1 + 1)
	{
		if (x < m_run_x0)
			m_run_x0 = x;
		else if (x > m_run_x1)
			m_run_x1 = x;
	}
	else
	{
		flush_dirty_run();
		m_run_valid = true;
		m_run_y = y;
		m_run_x0 = m_run_x1 = x;
	}
	g_mame_sgi_stat_pixel_writes++;
#ifdef MAME_SGI_DIRTY_SOURCE_DIAG
	g_mame_sgi_dirty_pixel_writes_this_frame++;
#endif
#ifdef MAME_SGI_REX3_FAST_PIXEL_VERIFY
	s_verify_pixels++;
	if (*vp != expected && ++s_verify_bad <= 20)
		fprintf(stderr, "REX3_FAST_PIXEL mismatch x=%d y=%d data=%08x got=%08x want=%08x logicop=%d blend=%d fc=%d plane=%d depth=%d wm=%08x\n",
				x, y, data, *vp, expected, m_logicop, m_blend, m_fast_clear, m_plane_enable, m_draw_depth, m_write_mask);
	if ((s_verify_pixels & ((1 << 24) - 1)) == 0)
		fprintf(stderr, "REX3_FAST_PIXEL verified=%llu bad=%llu\n", (unsigned long long)s_verify_pixels, (unsigned long long)s_verify_bad);
#endif
}
#endif

#ifdef MAME_SGI_REX3_FAST_SPAN
// The word write_pixel_xy() would store for data, when that doesn't depend
// on the destination; false when the logic op reads it.
bool rb2_device::stored_value(uint32_t data, uint32_t &value) const
{
	uint32_t src = data;
	switch (m_draw_depth)
	{
	case 0: src |= src << 4; src |= src << 8; src |= src << 16; break;
	case 1: src |= src << 8; src |= src << 16; break;
	case 2: src |= src << 12; break;
	case 3: break;
	}
	switch (m_plane_enable)
	{
	case 4: src <<= 8; break;
	case 5: src <<= 2; break;
	default: break;
	}
	if (m_blend || m_fast_clear)
	{
		value = src;
		return true;
	}
	switch (m_logicop)
	{
	case 0:  value = 0x000000; return true;
	case 3:  value = src;      return true;
	case 12: value = ~src;     return true;
	case 15: value = 0xffffff; return true;
	default: return false;
	}
}

// Stores pixels x0..x1 of row y (VRAM coordinates), x0 <= x1: pixel x gets
// fg[x & 3] when pattern bit 'bit' is set (bit counts down from the first
// pixel, modulo 32), else bg when has_bg, else is left alone. The values
// come from stored_value(). Same VRAM contents and dirty area as
// write_pixel_xy() for each stored pixel.
#ifdef MAME_SGI_REX3_MODE_CENSUS
namespace {
// store_row() pixels by loop: [fill/dither/pattern][direct/masked], with the
// last masked write mask seen. Printed to stderr at exit.
struct store_row_census
{
	uint64_t pixels[3][2] = {};
	uint32_t masked_mask = 0;
	~store_row_census()
	{
		static const char *const s_kind[3] = { "fill", "dither", "pattern" };
		for (int k = 0; k < 3; k++)
			fprintf(stderr, "REX3_STORE_ROW %-7s direct=%llu masked=%llu\n", s_kind[k],
					(unsigned long long)pixels[k][0], (unsigned long long)pixels[k][1]);
		fprintf(stderr, "REX3_STORE_ROW last masked write mask %08x\n", masked_mask);
	}
} s_store_row_census;
} // anonymous namespace
#endif

void rb2_device::store_row(int y, int x0, int x1, const uint32_t fg[4], uint32_t pattern, int bit, bool has_bg, uint32_t bg)
{
#ifdef MAME_SGI_REX3_PLANE_WRITE_MASK
	static const uint32_t s_plane_bits[8] = {
		0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0x00ffff00, 0x000000cc, 0x00000033, 0xffffffff };
	const uint32_t write_mask = m_write_mask_eff & m_global_mask & s_plane_bits[m_plane_enable & 7];
#else
	const uint32_t write_mask = m_write_mask_eff & m_global_mask;
#endif
	uint32_t *const row = &m_dest_buf[y * (1280 + 64)];
	const bool direct = write_mask == m_global_mask;
	uint32_t stored = 0;
	int last = -1;
	if (pattern == 0xffffffff)
	{
		// Every pixel stored: a fill (clears, flat spans) or a 4-pixel
		// repeating dither. Masked stores keep the unwritten planes, e.g. the
		// other buffer of a double-buffered 12bpp visual. Written in groups
		// of four from a 4-aligned x so the compiler can use vector stores.
		const uint32_t keep = direct ? 0 : ~write_mask;
		const uint32_t v0 = fg[0] & write_mask, v1 = fg[1] & write_mask, v2 = fg[2] & write_mask, v3 = fg[3] & write_mask;
		const uint32_t v[4] = { v0, v1, v2, v3 };
		int x = x0;
		if (v0 == v1 && v0 == v2 && v0 == v3)
		{
			if (direct)
				std::fill(row + x0, row + x1 + 1, v0);
			else
				for (; x <= x1; x++)
					row[x] = (row[x] & keep) | v0;
		}
		else
		{
			const auto dither = [&] (auto direct_store)
			{
				const auto put = [&] (int px, uint32_t pv)
				{
					row[px] = decltype(direct_store)::value ? pv : (row[px] & keep) | pv;
				};
				for (; x <= x1 && (x & 3); x++)
					put(x, v[x & 3]);
				for (; x + 3 <= x1; x += 4)
				{
					put(x + 0, v0);
					put(x + 1, v1);
					put(x + 2, v2);
					put(x + 3, v3);
				}
				for (; x <= x1; x++)
					put(x, v[x & 3]);
			};
			if (direct)
				dither(std::true_type());
			else
				dither(std::false_type());
		}
		stored = x1 - x0 + 1;
		last = x1;
#ifdef MAME_SGI_REX3_MODE_CENSUS
		s_store_row_census.pixels[(v0 == v1 && v0 == v2 && v0 == v3) ? 0 : 1][direct ? 0 : 1] += stored;
		if (!direct)
			s_store_row_census.masked_mask = write_mask;
#endif
	}
	else
	{
#ifdef MAME_SGI_REX3_MODE_CENSUS
		s_store_row_census.pixels[2][direct ? 0 : 1] += x1 - x0 + 1;
#endif
		for (int x = x0; x <= x1; x++, bit = (bit - 1) & 0x1f)
		{
			uint32_t v;
			if (BIT(pattern, bit))
				v = fg[x & 3];
			else if (has_bg)
				v = bg;
			else
				continue;
			if (direct)
				row[x] = v & write_mask;
			else
				row[x] = (row[x] & ~write_mask) | (v & write_mask);
			stored++;
			last = x;
		}
	}
	if (last < 0)
		return;
	m_buf_ptr = &row[last];
	m_cur_x = last;
	m_cur_y = y;
	flush_dirty_run();
	m_dirty.add_rect(x0, y, last, y);
	g_mame_sgi_stat_pixel_writes += stored;
}
#endif

// MAME_SGI_WRMASK_LAYOUT_FIX: the write mask is in the framebuffer's own bit
// layout (REX3 spec table 22), which interleaves the RGB bits: 24bpp holds
// B0 R0 G0 B1 R1 G1 .. B7 R7 G7 from bit 23 down, each 12bpp buffer
// B4 R4 G4 .. B7 R7 G7 from its bit 11 down, 8bpp R5 G5 B6 R6 G6 B7 R7 G7
// and each 4bpp buffer G6 B7 R7 G7. Software builds its masks that way
// (OpenGL's glColorMask(1,0,1,1) at 24bpp is 0xdb6db6), while this model
// keeps RGB pixels as packed R, G, B fields, so RGB-mode masks are moved
// into that layout. Colour-index and auxiliary plane masks are as is.
namespace {
struct rb2_wrmask_tables
{
	// [depth][byte of the mask][byte value] -> the bits in this model's layout
	uint32_t t[4][3][256];
	rb2_wrmask_tables()
	{
		uint8_t map[4][24];	// framebuffer bit -> this model's bit
		for (int n = 0; n < 8; n++)	// 24bpp: colour bit n
		{
			map[3][23 - 3 * n] = 16 + n;	// blue
			map[3][22 - 3 * n] = n;		// red
			map[3][21 - 3 * n] = 8 + n;	// green
		}
		for (int b = 0; b < 24; b += 12)	// 12bpp: per buffer, bits 4-7
			for (int j = 0; j < 4; j++)
			{
				map[2][b + 11 - 3 * j] = b + 8 + j;	// blue
				map[2][b + 10 - 3 * j] = b + j;		// red
				map[2][b + 9 - 3 * j] = b + 4 + j;	// green
			}
		static const uint8_t s_8bpp[8] = { 5, 2, 7, 4, 1, 6, 3, 0 };	// G7 R7 B7 G6 R6 B6 G5 R5
		static const uint8_t s_4bpp[4] = { 2, 0, 3, 1 };				// G7 R7 B7 G6
		for (int i = 0; i < 24; i++)
		{
			map[1][i] = (i & ~7) + s_8bpp[i & 7];
			map[0][i] = (i & ~3) + s_4bpp[i & 3];
		}
		for (int d = 0; d < 4; d++)
			for (int k = 0; k < 3; k++)
				for (int v = 0; v < 256; v++)
				{
					uint32_t m = 0;
					for (int i = 0; i < 8; i++)
						if (BIT(v, i))
							m |= 1u << map[d][8 * k + i];
					t[d][k][v] = m;
				}
	}
};
const rb2_wrmask_tables s_rb2_wrmask;
} // anonymous namespace

void rb2_device::update_write_mask()
{
	const uint32_t m = m_write_mask;
	if (m_rgbmode && m_plane_enable == 1 && (m & 0xffffff) != 0xffffff)
	{
		const auto &t = s_rb2_wrmask.t[m_draw_depth & 3];
		m_write_mask_eff = t[0][m & 0xff] | t[1][(m >> 8) & 0xff] | t[2][(m >> 16) & 0xff] | (m & 0xff000000);
	}
	else
		m_write_mask_eff = m;
}

void rb2_device::set_write_mask(uint32_t data)
{
	m_write_mask = data;
	update_write_mask();
}

void rb2_device::set_flags(uint16_t data)
{
	m_blend = BIT(data, 12);
	m_fast_clear = BIT(data, 11);
	m_rgbmode = BIT(data, 10);
	m_dblsrc = BIT(data, 9);
	m_plane_enable = (data & 0x1c0) >> 6;
	m_draw_depth = (data & 0x30) >> 4;
	m_logicop = data & 0xf;

	static const uint32_t s_draw_bpp[4] = { 4, 8, 12, 24 };
	static const uint32_t s_store_shift[8][4][2] = {
		{   { 0,  0 }, // None, 4bpp, Buffer 0/1
			{ 0,  0 }, // None, 8bpp, Buffer 0/1
			{ 0,  0 }, // None, 12bpp, Buffer 0/1
			{ 0,  0 }, // None, 24bpp, Buffer 0/1 (not valid)
		},
		// MAME_SGI_4BPP_DB_FIX: a 4bpp buffer 1 is the upper nibble, as the
		// display reads it
		{   { 0,  4 }, // RGB/CI, 4bpp, Buffer 0/1
			{ 0,  8 }, // RGB/CI, 8bpp, Buffer 0/1
			{ 0, 12 }, // RGB/CI, 12bpp, Buffer 0/1
			{ 0,  0 }, // RGB/CI, 24bpp, Buffer 0/1 (not valid)
		},
		{   { 0,  4 }, // RGBA, 4bpp, Buffer 0/1
			{ 0,  8 }, // RGBA, 8bpp, Buffer 0/1
			{ 0, 12 }, // RGBA, 12bpp, Buffer 0/1
			{ 0,  0 }, // RGBA, 24bpp, Buffer 0/1 (not valid)
		},
		{   { 0,  0 }, // Invalid, 4bpp, Buffer 0/1
			{ 0,  0 }, // Invalid, 8bpp, Buffer 0/1
			{ 0,  0 }, // Invalid, 12bpp, Buffer 0/1
			{ 0,  0 }, // Invalid, 24bpp, Buffer 0/1 (not valid)
		},
		{   { 8, 16 }, // Overlay, 4bpp, Buffer 0/1
			{ 8, 16 }, // Overlay, 8bpp, Buffer 0/1
			{ 8, 16 }, // Overlay, 12bpp, Buffer 0/1
			{ 8, 16 }, // Overlay, 24bpp, Buffer 0/1 (not valid)
		},
		{   { 2,  6 }, // Popup, 4bpp, Buffer 0/1
			{ 2,  6 }, // Popup, 8bpp, Buffer 0/1
			{ 2,  6 }, // Popup, 12bpp, Buffer 0/1
			{ 2,  6 }, // Popup, 24bpp, Buffer 0/1 (not valid)
		},
		{   { 0,  4 }, // CID, 4bpp, Buffer 0/1
			{ 0,  4 }, // CID, 8bpp, Buffer 0/1
			{ 0,  4 }, // CID, 12bpp, Buffer 0/1
			{ 0,  4 }, // CID, 24bpp, Buffer 0/1 (not valid)
		},
		{   { 0,  0 }, // Invalid, 4bpp, Buffer 0/1
			{ 0,  0 }, // Invalid, 8bpp, Buffer 0/1
			{ 0,  0 }, // Invalid, 12bpp, Buffer 0/1
			{ 0,  0 }, // Invalid, 24bpp, Buffer 0/1 (not valid)
		},
	};

	m_src_shift = s_store_shift[m_plane_enable][m_draw_depth][m_dblsrc];
	m_draw_bpp = s_draw_bpp[m_draw_depth];

	switch (m_plane_enable)
	{
		case 1: // RGB/CI planes
		case 2: // RGBA planes
			m_dest_buf = &m_rgbci[0];
			break;
		case 4: // Overlay planes
		case 5: // Popup planes
		case 6: // CID planes
			m_dest_buf = &m_cidaux[0];
			break;
	}
	update_write_mask();
}

void rb2_device::set_address(uint32_t address)
{
	m_buf_ptr = &m_dest_buf[address];
	// Every call site computes address as y * 1344 + x (matching
	// rgbci()/cidaux()'s own stride) - cached here so store_pixel() can
	// update the dirty tracker without redoing the division/modulo on
	// every single pixel write.
	m_cur_x = (int)(address % 1344);
	m_cur_y = (int)(address / 1344);
}

uint32_t rb2_device::read_pixel()
{
	return *m_buf_ptr >> m_src_shift;
}

void rb2_device::write_pixel(uint32_t data)
{
	data = expand_to_all_lanes(data);
	if (m_blend || m_fast_clear)
		store_pixel(data);
	else
		logic_pixel(data);
}

void rb2_device::logic_pixel(uint32_t src)
{
	const uint32_t dst = read_pixel();

	switch (m_logicop)
	{
		case 0:     store_pixel(0x000000);        break;
		case 1:     store_pixel(src & dst);       break;
		case 2:     store_pixel(src & ~dst);      break;
		case 3:     store_pixel(src);             break;
		case 4:     store_pixel(~src & dst);      break;
		case 5:     store_pixel(dst);             break;
		case 6:     store_pixel(src ^ dst);       break;
		case 7:     store_pixel(src | dst);       break;
		case 8:     store_pixel(~(src | dst));    break;
		case 9:     store_pixel(~(src ^ dst));    break;
		case 10:    store_pixel(~dst);            break;
		case 11:    store_pixel(src | ~dst);      break;
		case 12:    store_pixel(~src);            break;
		case 13:    store_pixel(~src | dst);      break;
		case 14:    store_pixel(~(src & dst));    break;
		case 15:    store_pixel(0xffffff);        break;
	}
}

void rb2_device::store_pixel(uint32_t value)
{
#ifdef MAME_SGI_REX3_PLANE_WRITE_MASK
	// bits of the destination word owned by each DRAWMODE1 plane-enable value
	static const uint32_t s_plane_bits[8] = {
		0xffffffff, // none
		0xffffffff, // RGB/CI
		0xffffffff, // RGBA
		0xffffffff, // invalid
		0x00ffff00, // overlay, buffers 0/1
		0x000000cc, // popup, buffers 0/1
		0x00000033, // CID, buffers 0/1
		0xffffffff, // invalid
	};
	const uint32_t write_mask = m_write_mask_eff & m_global_mask & s_plane_bits[m_plane_enable & 7];
#else
	const uint32_t write_mask = m_write_mask_eff & m_global_mask;
#endif
	*m_buf_ptr &= ~write_mask;
	*m_buf_ptr |= value & write_mask;
#ifdef MAME_SGI_RB2_SPAN_DIRTY
	// extend the current run when this pixel is next to it on the same row
	// (REX3 spans run in either direction), otherwise start a new one
	if (m_run_valid && m_cur_y == m_run_y && m_cur_x >= m_run_x0 - 1 && m_cur_x <= m_run_x1 + 1)
	{
		if (m_cur_x < m_run_x0)
			m_run_x0 = m_cur_x;
		else if (m_cur_x > m_run_x1)
			m_run_x1 = m_cur_x;
	}
	else
	{
		flush_dirty_run();
		m_run_valid = true;
		m_run_y = m_cur_y;
		m_run_x0 = m_run_x1 = m_cur_x;
	}
#else
	m_dirty.add_rect(m_cur_x, m_cur_y, m_cur_x, m_cur_y);
#endif
	g_mame_sgi_stat_pixel_writes++;
#ifdef MAME_SGI_DIRTY_SOURCE_DIAG
	g_mame_sgi_dirty_pixel_writes_this_frame++;
#endif
}


/*************************************
 *
 *  Newport Device
 *
 *************************************/

/*static*/ const uint32_t newport_base_device::s_host_shifts[4] = { 8, 8, 16, 32 };

newport_base_device::newport_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, type, tag, owner, clock)
	, device_gio64_card_interface(mconfig, *this)
	, m_screen(*this, "screen")
	, m_xmap(*this, "xmap%u", 0U)
	, m_cmap(*this, "cmap%u", 0U)
	, m_vc2(*this, "vc2")
	, m_rb2(*this, "rb2")
{
}

gio64_xl8_device::gio64_xl8_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: newport_base_device(mconfig, GIO64_XL8, tag, owner, clock)
{
}

gio64_xl24_device::gio64_xl24_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: newport_base_device(mconfig, GIO64_XL24, tag, owner, clock)
{
}

//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------

void newport_base_device::device_start()
{
	m_dcb_timeout_timer = timer_alloc(FUNC(newport_base_device::dcb_timeout_tick), this);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	machine().save().register_postload(save_prepost_delegate(FUNC(newport_base_device::lut_fold_invalidate), this));
#endif

	// Triggered stat-counter registry (2026-09-25) - see
	// mame_sgi_stat_registry's own comment in newport.h for the full
	// rationale. Counter registration always happens (cheap, and lets
	// "sgistat dump" work even without -debug, if ever called some other
	// way in the future); the debugger command itself is only registered
	// once (guarded - register_command isn't meant to be called twice for
	// the same name) and only when a debugger is actually attached.
	mame_sgi_stat_registry::instance().register_counter("newport.pixel_writes", &g_mame_sgi_stat_pixel_writes);
	mame_sgi_stat_registry::instance().register_counter("newport.cursor_calls", &g_mame_sgi_stat_cursor_calls);
	mame_sgi_stat_registry::instance().register_counter("newport.rows_walked", &g_mame_sgi_stat_rows_walked);
	mame_sgi_stat_registry::instance().register_counter("newport.screen_update_recomputes", &g_mame_sgi_stat_screen_update_recomputes);
	mame_sgi_stat_registry::instance().register_counter("newport.screen_update_skips", &g_mame_sgi_stat_screen_update_skips);
	mame_sgi_stat_registry::instance().register_counter("newport.mark_full_dirty_calls", &g_mame_sgi_stat_mark_full_dirty_calls);
	mame_sgi_stat_registry::instance().register_counter("newport.mark_xmap_slot_calls", &g_mame_sgi_stat_mark_xmap_slot_calls);
	mame_sgi_stat_registry::instance().register_counter("newport.vblank_events", &g_mame_sgi_stat_vblank_events);
	mame_sgi_stat_registry::instance().register_counter("newport.vrint_asserts", &g_mame_sgi_stat_vrint_asserts);
	if (machine().debug_flags & DEBUG_FLAG_ENABLED)
	{
		static bool s_sgistat_command_registered = false;
		if (!s_sgistat_command_registered)
		{
			s_sgistat_command_registered = true;
			machine().debugger().console().register_command(
					"sgistat", CMDFLAG_NONE, 1, 2,
					std::bind(&newport_base_device::sgistat_command, this, std::placeholders::_1));
		}
	}

	save_item(NAME(m_rex3.m_draw_mode0));
	save_item(NAME(m_rex3.m_color_host));
	save_item(NAME(m_rex3.m_alpha_host));
	save_item(NAME(m_rex3.m_draw_mode1));
	save_item(NAME(m_rex3.m_plane_enable));
	save_item(NAME(m_rex3.m_plane_depth));
	save_item(NAME(m_rex3.m_rwpacked));
	save_item(NAME(m_rex3.m_hostdepth));
	save_item(NAME(m_rex3.m_rwdouble));
	save_item(NAME(m_rex3.m_sfactor));
	save_item(NAME(m_rex3.m_dfactor));
	save_item(NAME(m_rex3.m_store_shift));
	save_item(NAME(m_rex3.m_write_width));
	save_item(NAME(m_rex3.m_ls_mode));
	save_item(NAME(m_rex3.m_ls_pattern));
	save_item(NAME(m_rex3.m_ls_pattern_saved));
	save_item(NAME(m_rex3.m_z_pattern));
	save_item(NAME(m_rex3.m_color_back));
	save_item(NAME(m_rex3.m_color_vram));
	save_item(NAME(m_rex3.m_alpha_ref));
	save_item(NAME(m_rex3.m_smask_x));
	save_item(NAME(m_rex3.m_smask_y));
	save_item(NAME(m_rex3.m_setup));
	save_item(NAME(m_rex3.m_step_z));
	save_item(NAME(m_rex3.m_x_start));
	save_item(NAME(m_rex3.m_y_start));
	save_item(NAME(m_rex3.m_x_end));
	save_item(NAME(m_rex3.m_y_end));
	save_item(NAME(m_rex3.m_x_start_frac));
	save_item(NAME(m_rex3.m_y_start_frac));
	save_item(NAME(m_rex3.m_x_end_frac));
	save_item(NAME(m_rex3.m_y_end_frac));

	save_item(NAME(m_rex3.m_x_save));
	save_item(NAME(m_rex3.m_xy_move));
	save_item(NAME(m_rex3.m_x_move));
	save_item(NAME(m_rex3.m_y_move));
	save_item(NAME(m_rex3.m_bres_d));
	save_item(NAME(m_fline_cont.valid));
	save_item(NAME(m_fline_cont.x));
	save_item(NAME(m_fline_cont.y));
	save_item(NAME(m_fline_cont.x2));
	save_item(NAME(m_fline_cont.y2));
	save_item(NAME(m_fline_cont.d));
	save_item(NAME(m_fline_cont.incr1));
	save_item(NAME(m_fline_cont.incr2));
	save_item(NAME(m_fline_cont.octant));
	save_item(NAME(m_aline_cont.valid));
	save_item(NAME(m_aline_cont.x1));
	save_item(NAME(m_aline_cont.y1));
	save_item(NAME(m_aline_cont.x2));
	save_item(NAME(m_aline_cont.y2));
	save_item(NAME(m_aline_cont.xs));
	save_item(NAME(m_aline_cont.ys));
	save_item(NAME(m_aline_cont.col));
	save_item(NAME(m_rex3.m_bres_s1));
	save_item(NAME(m_rex3.m_bres_octant_inc1));
	save_item(NAME(m_rex3.m_bres_round_inc2));
	save_item(NAME(m_rex3.m_bres_e1));
	save_item(NAME(m_rex3.m_bres_s2));
	save_item(NAME(m_rex3.m_a_weight0));
	save_item(NAME(m_rex3.m_a_weight1));
	save_item(NAME(m_rex3.m_x_start_f));
	save_item(NAME(m_rex3.m_y_start_f));
	save_item(NAME(m_rex3.m_x_end_f));
	save_item(NAME(m_rex3.m_y_end_f));
	save_item(NAME(m_rex3.m_x_start_i));
	save_item(NAME(m_rex3.m_y_start_i));
	save_item(NAME(m_rex3.m_x_end_i));
	save_item(NAME(m_rex3.m_y_end_i));
	save_item(NAME(m_rex3.m_xy_start_i));
	save_item(NAME(m_rex3.m_xy_end_i));
	save_item(NAME(m_rex3.m_x_start_end_i));
	save_item(NAME(m_rex3.m_color_red));
	save_item(NAME(m_rex3.m_color_alpha));
	save_item(NAME(m_rex3.m_color_green));
	save_item(NAME(m_rex3.m_color_blue));
	save_item(NAME(m_rex3.m_curr_color_red));
	save_item(NAME(m_rex3.m_curr_color_alpha));
	save_item(NAME(m_rex3.m_curr_color_green));
	save_item(NAME(m_rex3.m_curr_color_blue));
	save_item(NAME(m_rex3.m_slope_red));
	save_item(NAME(m_rex3.m_slope_alpha));
	save_item(NAME(m_rex3.m_slope_green));
	save_item(NAME(m_rex3.m_slope_blue));
	save_item(NAME(m_rex3.m_write_mask));
	save_item(NAME(m_rex3.m_color_i));
	save_item(NAME(m_rex3.m_zero_overflow));
	save_item(NAME(m_rex3.m_host_dataport));
	save_item(NAME(m_rex3.m_dcb_mode));
	save_item(NAME(m_rex3.m_dcb_reg_select));
	save_item(NAME(m_rex3.m_dcb_slave_select));
	save_item(NAME(m_rex3.m_dcb_data_msw));
	save_item(NAME(m_rex3.m_dcb_data_lsw));
	save_item(NAME(m_rex3.m_top_scanline));
	save_item(NAME(m_rex3.m_xy_window));
	save_item(NAME(m_rex3.m_x_window));
	save_item(NAME(m_rex3.m_y_window));
	save_item(NAME(m_rex3.m_clip_mode));
	save_item(NAME(m_rex3.m_config));
	save_item(NAME(m_rex3.m_status));
	save_item(NAME(m_rex3.m_dcb_mask));

	save_item(NAME(m_ramdac_lut_r));
	save_item(NAME(m_ramdac_lut_g));
	save_item(NAME(m_ramdac_lut_b));
	save_item(NAME(m_ramdac_lut_index));
}

//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------

void newport_base_device::mark_full_dirty()
{
	g_mame_sgi_stat_mark_full_dirty_calls++;
#ifdef MAME_SGI_DIRTY_RECT_SMALLRECT_EXPERIMENT
	m_pending_dirty.add_rect(0, 0, 400 - 1, 400 - 1);
#else
	m_pending_dirty.add_rect(0, 0, 1280 + 64 - 1, 1024 + 64 - 1);
#endif
}

void newport_base_device::mark_dirty_for_xmap_slot(uint8_t slot)
{
	g_mame_sgi_stat_mark_xmap_slot_calls++;
	const uint32_t bit = 1u << (slot & 0x1f);
	int y0 = -1, y1 = -1;
	for (int y = 0; y < 1088; y++)
	{
		if (m_row_xmap_slot_mask[y] & bit)
		{
			if (y0 < 0)
				y0 = y;
			y1 = y;
		}
	}
#ifdef MAME_SGI_XMAP_SLOT_RANGE_DIAG
	// TEMPORARY diagnostic (2026-09-22) - NOT a real fix, just
	// visibility. Prints the actual row range computed for this slot,
	// throttled to the first 40 INFORMATIVE prints (y0 >= 0, i.e. the
	// slot was actually referenced by some row) - directly answers "is
	// this precise (a tight band matching a window's real height) or
	// does it degenerate to near-full-screen for this system's real XMAP
	// slot-allocation pattern", rather than inferring it from write
	// frequency/value patterns alone. First attempt at this (2026-09-22)
	// counted EVERY call including uninformative y0=-1 ones (a slot
	// written before screen_update() has ever populated the row mask for
	// it, e.g. sequential mode-table setup at boot before the first
	// frame renders) - that exhausted the whole budget on boot-time noise
	// before wave.cpp ever launched, printing nothing useful. Revert once
	// answered.
	{
		static int g_mame_sgi_xmap_slot_range_diag_prints = 0;
		if (y0 >= 0 && g_mame_sgi_xmap_slot_range_diag_prints < 40)
		{
			g_mame_sgi_xmap_slot_range_diag_prints++;
			osd_printf_info("MAME_SGI_XMAP_SLOT_RANGE_DIAG slot=%u y0=%d y1=%d span=%d\n",
					slot, y0, y1, y1 - y0 + 1);
		}
	}
#endif
	if (y0 >= 0)
	{
		m_pending_dirty.add_rect(0, y0, 1280 + 64 - 1, y1);
	}
}

void newport_base_device::sgistat_command(const std::vector<std::string_view> &params)
{
	auto &console = machine().debugger().console();
	if (params[0] == "reset")
	{
		mame_sgi_stat_registry::instance().reset_all();
		console.printf("sgistat: all counters reset to 0\n");
	}
	else if (params[0] == "dump")
	{
		const std::string filter = (params.size() > 1) ? std::string(params[1]) : std::string();
		mame_sgi_stat_registry::instance().for_each([&](const char *name, uint64_t value)
		{
			if (filter.empty() || filter == name)
				console.printf("%s=%llu\n", name, (unsigned long long)value);
		});
	}
	else
	{
		console.printf("usage: sgistat reset | sgistat dump [name]\n");
	}
}

void newport_base_device::device_reset()
{
	memset(&m_rex3, 0, sizeof(rex3_t));
	memset(m_ramdac_lut_r, 0, sizeof(uint32_t) * 256);
	memset(m_ramdac_lut_g, 0, sizeof(uint32_t) * 256);
	memset(m_ramdac_lut_b, 0, sizeof(uint32_t) * 256);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	lut_fold_invalidate();
#endif

	m_rex3.m_draw_mode0 = 0x00000000;
	m_rex3.m_draw_mode1 = 0x3002f001;
	m_rex3.m_dcb_mode = 0x00000780;
	m_rex3.m_x_window = 0x1000;
	m_rex3.m_y_window = 0x1000;

	// Force a real recompute for the first frames after reset - initial
	// framebuffer content is unknown, so the dirty trackers' own
	// "nothing written yet" default (m_valid == false) would be wrong
	// here. Bounds match BUFFER_SIZE's own max (1280+64)x(1024+64) rather
	// than querying current screen dimensions, which aren't necessarily
	// settled yet at reset time.
	m_pending_dirty.clear();
	m_pending_dirty.add_rect(0, 0, 1280 + 64 - 1, 1024 + 64 - 1);
	m_prev_dirty.clear();

#if ENABLE_NEWVIEW_LOG
	m_newview_log = nullptr;
#endif
}

void newport_base_device::mem_map(address_map &map)
{
	map(0x000f0000, 0x000f1fff).rw(FUNC(newport_base_device::rex3_r), FUNC(newport_base_device::rex3_w));
}

#if ENABLE_NEWVIEW_LOG
void newport_base_device::start_logging()
{
	uint16_t log_index = 0xffff;
	char log_name_buf[128];
	FILE *log_test = nullptr;
	do
	{
		log_index++;
		snprintf(log_name_buf, 128, "newview%04d.log", log_index);
		log_test = fopen(log_name_buf, "rb");
	} while(log_test != nullptr);

	m_newview_log = fopen(log_name_buf, "wb");

	popmessage("Recording Newport to %s", log_name_buf);

	m_vc2->serialize(m_newview_log);
	printf("vc2: %08x\n", (uint32_t)ftell(m_newview_log));
	m_xmap[0]->serialize(m_newview_log);
	printf("xmap0: %08x\n", (uint32_t)ftell(m_newview_log));
	m_xmap[1]->serialize(m_newview_log);
	printf("xmap1: %08x\n", (uint32_t)ftell(m_newview_log));
	fwrite(&m_rex3, sizeof(rex3_t), 1, m_newview_log);
	printf("rex3: %08x\n", (uint32_t)ftell(m_newview_log));
	m_cmap[0]->serialize(m_newview_log);
	printf("cmap0: %08x\n", (uint32_t)ftell(m_newview_log));
	m_cmap[1]->serialize(m_newview_log);
	printf("cmap1: %08x\n", (uint32_t)ftell(m_newview_log));
	m_rb2->serialize(m_newview_log);
	printf("rb2: %08x\n", (uint32_t)ftell(m_newview_log));
	fwrite(m_rb2->rgbci(0), sizeof(uint32_t), (1280+64)*(1024+64), m_newview_log);
	printf("rgbci: %08x\n", (uint32_t)ftell(m_newview_log));
	fwrite(m_rb2->cidaux(0), sizeof(uint32_t), (1280+64)*(1024+64), m_newview_log);
	printf("cidaux: %08x\n", (uint32_t)ftell(m_newview_log));
}

void newport_base_device::stop_logging()
{
	popmessage("Newport recording stopped.");
	fclose(m_newview_log);
	m_newview_log = nullptr;
}
#endif

TIMER_CALLBACK_MEMBER(newport_base_device::dcb_timeout_tick)
{
	m_rex3.m_status &= ~STATUS_BACKBUSY;
}

#ifdef MAME_SGI_RENDER_DIRTY_RECTS
uint32_t newport_base_device::screen_update(screen_device &device, bitmap_rgb32 &bitmap, const rectangle &visible_clip, render_dirty_rects &dirty)
#else
uint32_t newport_base_device::screen_update(screen_device &device, bitmap_rgb32 &bitmap, const rectangle &visible_clip)
#endif
{
	// MAME_SGI_DISPLAY_GEOMETRY_FIX: the visible area starts a couple of
	// columns into the readout (see update_screen_size()), but bitmap
	// column x is VRAM column x, so composite from column 0. The code
	// below treats the clip's max_x as exclusive: pass one past the last
	// visible column.
	rectangle cliprect = visible_clip;
	cliprect.min_x = 0;
	cliprect.max_x = visible_clip.max_x + 1;

#ifdef MAME_SGI_NEWPORT_COMPOSITE_TIMING_DIAG
	osd_ticks_t mame_sgi_composite_t0 = osd_ticks();
#endif
	uint32_t mame_sgi_result = 0;

	// See this same variable's own fuller comment further down (near
	// RB2_MAX_ROW) for the full rationale on both the register itself
	// and the 0x3ff sentinel check - computed up front here (and reused
	// unchanged below, not recomputed) since rb2_dirty's rows need the
	// identical conversion, and it has to happen before frame_dirty/
	// vc2_dirty are merged together, not after (see the comment on
	// rb2_dirty just below for why the two can't share one conversion
	// applied later to the merged result).
	// MAME_SGI_TOP_SCANLINE_FIX
	const int top_scanline = (m_rex3.m_top_scanline == 0x3ff) ? 0 : (int)m_rex3.m_top_scanline;

	// Gather this frame's fresh dirty contributions (framebuffer writes +
	// cursor state changes) and merge into the held region. A hold-and-
	// reapply cycle (not a simple "skip if nothing new since last frame"
	// flag) is needed because MAME's own screen_device double buffering
	// (m_bitmap[2], toggled every frame regardless of skip status) only
	// has whichever buffer is current refreshed by a single recompute,
	// leaving the other stale - same fix the original crude experiment
	// needed, just carrying a real rectangle instead of a bool.
	{
		newport_dirty_rect frame_dirty;
		// rb2_dirty's rows are raw VRAM rows (rb2_device::store_pixel()'s
		// own m_dirty.add_rect(m_cur_x, m_cur_y, ...) - m_cur_y already
		// carries the m_y_window write-side offset), but vc2_dirty's rows
		// (from vc2_device::mark_cursor_dirty()'s cursor_bounds(), driven
		// by m_cursor_x/m_cursor_y) are already bitmap/display-relative -
		// the cursor's position is inherently a display concept, never a
		// framebuffer write. Converting rb2_dirty here, before the two
		// are merged into one rectangle, keeps that distinction intact;
		// converting the merged result afterward (tried and reverted
		// live, 2026-09-19) applies the same offset to both regardless
		// of which one actually needs it, corrupting the cursor's own
		// erase/redraw box whenever it - not a pixel write - ends up
		// deciding the merged rectangle's bounds, producing exactly the
		// "stale pointer image / delayed position update" seen live.
		// Converted per-rect, not via rb2_dirty's own aggregate x0()/y0()/
		// x1()/y1() - the N-rect upgrade (2026-09-25) means
		// rb2_dirty can hold multiple disjoint regions (e.g. wave's own
		// window plus an unrelated desktop element drawing elsewhere at
		// the same time); converting the AGGREGATE bbox instead of each
		// rect individually would re-coalesce them right back into one
		// wide box here, throwing away exactly the precision the N-rect
		// list exists to keep.
		const newport_dirty_rect &rb2_dirty = m_rb2->dirty();
#ifdef MAME_SGI_PENDING_RECTS_DIAG
		g_pr_print = false;
		if (rb2_dirty.valid() && std::chrono::steady_clock::now() >= g_pr_next)
		{
			g_pr_next = std::chrono::steady_clock::now() + std::chrono::seconds(1);
			g_pr_print = true;
			osd_printf_info("PENDING_RECTS_DIAG top_scanline=%d rb2 raw n=%d:", top_scanline, rb2_dirty.count());
			for (int i = 0; i < rb2_dirty.count(); i++)
				osd_printf_info(" (%d,%d)-(%d,%d)", rb2_dirty.rect(i).x0, rb2_dirty.rect(i).y0, rb2_dirty.rect(i).x1, rb2_dirty.rect(i).y1);
			osd_printf_info(" vc2=%s\n", m_vc2->dirty().valid() ? "valid" : "none");
		}
#endif
		for (int i = 0; i < rb2_dirty.count(); i++)
		{
			const auto &r = rb2_dirty.rect(i);
			frame_dirty.add_rect(r.x0, std::max(r.y0 - top_scanline, 0), r.x1, r.y1 - top_scanline);	// MAME_SGI_TOP_SCANLINE_FIX
		}
		frame_dirty.add_rect(m_vc2->dirty());
		m_rb2->clear_dirty();
		m_vc2->clear_dirty();

		if (frame_dirty.valid())
		{
#ifdef MAME_SGI_MOUSEFOCUS_DIAG
			osd_printf_verbose("MOUSEFOCUS_DIAG: screen_update() frame_dirty.valid()=true\n");
#endif
			m_pending_dirty.add_rect(frame_dirty);
		}
#ifdef MAME_SGI_DIRTY_SOURCE_DIAG
		if (frame_dirty.valid())
		{
			auto const mame_sgi_now = std::chrono::steady_clock::now();
			if (mame_sgi_now >= g_mame_sgi_dirty_diag_next_print)
			{
				g_mame_sgi_dirty_diag_next_print = mame_sgi_now + std::chrono::seconds(1);
				auto const wall = std::chrono::system_clock::now();
				auto const t = std::chrono::system_clock::to_time_t(wall);
				auto const us = std::chrono::duration_cast<std::chrono::microseconds>(wall.time_since_epoch()) % 1'000'000;
				std::tm tm_buf{};
				localtime_r(&t, &tm_buf);
				char tsbuf[24];
				std::snprintf(tsbuf, sizeof(tsbuf), "%02d:%02d:%02d.%06d", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, int(us.count()));
				osd_printf_info(
						"MAME_SGI_DIRTY_SOURCE_DIAG wallclock=%s frame dirty: pixel_writes=%llu cursor_calls=%llu rect=(%d,%d)-(%d,%d)\n",
						tsbuf,
						(unsigned long long)g_mame_sgi_dirty_pixel_writes_this_frame,
						(unsigned long long)g_mame_sgi_dirty_cursor_calls_this_frame,
						frame_dirty.x0(), frame_dirty.y0(), frame_dirty.x1(), frame_dirty.y1());
			}
		}
		g_mame_sgi_dirty_pixel_writes_this_frame = 0;
		g_mame_sgi_dirty_cursor_calls_this_frame = 0;
#endif
	}

#ifdef MAME_SGI_LOWRES_BOOT_DIAG_FORCE_FULL_RECOMPUTE
	constexpr bool mame_sgi_force_full_recompute = true;
#else
	constexpr bool mame_sgi_force_full_recompute = false;
#endif
	// This frame's region to refresh: fresh dirt plus what the previous
	// call consumed (see m_prev_dirty's comment). Consumed here, before
	// the skip decision, so a skipped frame correctly leaves nothing
	// carried over.
	newport_dirty_rect walk_dirty = m_pending_dirty;
	walk_dirty.add_rect(m_prev_dirty);
	m_prev_dirty = m_pending_dirty;
	m_pending_dirty.clear();

	if (!mame_sgi_force_full_recompute && !walk_dirty.valid())
	{
		// Nothing pending, both buffers already in sync - tell MAME's own
		// frame_update()/finish_screen_updates() machinery this call
		// didn't change anything, so it can skip the OSD blit too.
		g_mame_sgi_stat_screen_update_skips++;
		mame_sgi_result = UPDATE_HAS_NOT_CHANGED;
		goto mame_sgi_composite_done;
	}
	g_mame_sgi_stat_screen_update_recomputes++;

	{
	const uint16_t cursor_msb = (uint16_t)m_xmap[0]->cursor_cmap() << 5;
	const uint16_t popup_msb = (uint16_t)m_xmap[0]->popup_cmap() << 5;
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
	lut_fold_update();
	const uint32_t *palette = m_lut_palette;
#else
	const uint32_t *palette = m_cmap[0]->palette_base();
#endif

	const int y_start = m_vc2->readout_y0();
	const int y_end = m_vc2->readout_y1();

	// MAME_SGI_TOP_SCANLINE_FIX: REX3's "Top of Screen Scanline" register (m_top_scanline, set via
	// the 0x1320 write case below) tells real hardware which row of RB2's
	// framebuffer RAM corresponds to the first visible display row - a
	// pan/scroll register, not a drawing-coordinate offset (that's
	// m_x_window/m_y_window, applied on the write side in output_pixel()).
	// Confirmed live (2026-09-19, low-resolution NVRAM-default boot):
	// IRIX's kernel programs this to a real, non-zero value (255, after a
	// transition sequence 1023->682->341->0->1023->255 during the PROM-
	// to-kernel handoff) while drawing through a matching non-zero
	// m_y_window offset - but nothing here previously read m_top_scanline
	// at all, so the compositor always displayed framebuffer row 0 as the
	// first row regardless of what the guest asked for, leaving content
	// drawn at row_window_offset..(row_window_offset+height) invisible at
	// its intended position and whatever was at rows 0..row_window_offset
	// (typically never touched by the guest, since it isn't part of the
	// window it thinks it's drawing into) showing instead. Clamped to
	// rb2_device::BUFFER_SIZE's own row count (1088, `(1024+64)` tall) -
	// m_top_scanline is guest-controlled and only masked to 10 bits
	// (0-1023) at the write site, so a value near that ceiling combined
	// with a tall cliprect could otherwise index past the allocated
	// buffer. Left out of begin_did_line()'s own addressing on purpose -
	// the DID table enumerates per-visible-line pixel-mode metadata in
	// the guest's own display-relative order already, independent of
	// which underlying framebuffer row backs any given line.
	constexpr int RB2_MAX_ROW = 1088 - 1;
	// top_scanline itself (including the 0x3ff "no panning configured"
	// sentinel check - stuck-observed live, 2026-09-19, on a normal,
	// correctly-configured indy_4610 boot, where m_x_window/m_y_window
	// stayed at their neutral no-offset baseline the entire time despite
	// it, which a real "pan 1023 rows" instruction wouldn't be
	// consistent with) is computed once, up front in this function -
	// see that declaration's own comment for the full rationale. Reused
	// unchanged here.

	// Clip the row range to the pending dirty rect intersected with
	// cliprect - rows outside it are left untouched, correct since the
	// bitmap already holds their unchanged content from a previous
	// recompute. X range deliberately stays the full cliprect width for
	// now: the DID mode-table walk below (begin_did_line()/
	// next_did_line_entry()) can only be walked forward from a row's
	// start, not seeked to an arbitrary starting column, so a real
	// per-row x-clip needs that walker extended first - left for a
	// restructure of this loop around DID runs.
	//
	// m_pending_dirty is already bitmap-relative by this point - the
	// rb2-sourced contribution was converted from raw VRAM rows at the
	// point it was merged in above (see that block's own comment for why
	// that has to happen before the merge, not here on the combined
	// rectangle), and the vc2/cursor-sourced contribution was bitmap-
	// relative from the start. No further conversion needed - used
	// directly as bitmap-row bounds.
	// Row ranges to walk this recompute - one per disjoint dirty rect
	// (the N-rect upgrade, 2026-09-25: m_pending_dirty can now
	// hold up to newport_dirty_rect::MAX_RECTS separate regions instead
	// of always coalescing into one bounding box), each intersected with
	// cliprect. Gaps between ranges (e.g. an unrelated desktop element
	// updating far from the app actually being redrawn) are skipped
	// entirely rather than walked and thrown away.
	struct row_range_t { int y0, y1, x0, x1; };	// rows inclusive, columns [x0, x1)
	row_range_t row_ranges[newport_dirty_rect::MAX_RECTS];
	int num_row_ranges = 0;
#ifndef MAME_SGI_LOWRES_BOOT_DIAG_FORCE_FULL_RECOMPUTE
	if (walk_dirty.valid())
	{
		for (int i = 0; i < walk_dirty.count(); i++)
		{
			const auto &r = walk_dirty.rect(i);
			const int ry0 = std::max(r.y0, cliprect.min_y);
			const int ry1 = std::min(r.y1, cliprect.max_y);
			if (ry0 <= ry1)
				row_ranges[num_row_ranges++] = { ry0, ry1, std::max(r.x0, cliprect.min_x), std::min(r.x1 + 1, cliprect.max_x) };
		}
	}
	else
	{
		row_ranges[num_row_ranges++] = { cliprect.min_y, cliprect.max_y, cliprect.min_x, cliprect.max_x };
	}
#else
	row_ranges[num_row_ranges++] = { cliprect.min_y, cliprect.max_y, cliprect.min_x, cliprect.max_x };
#endif
#ifdef MAME_SGI_RENDER_DIRTY_RECTS
	// Report what the walk below writes: each range's rows (only its
	// columns when compositing spans, whole rows otherwise), plus the
	// cursor box on those rows, which the cursor stamp rewrites.
	{
		int cx0 = 0, cy0 = 0, cx1 = -1, cy1 = -1;
		if (m_vc2->cursor_enabled())
			m_vc2->cursor_bounds(cx0, cy0, cx1, cy1);
		for (int i = 0; i < num_row_ranges; i++)
		{
			const row_range_t &r = row_ranges[i];
#ifdef MAME_SGI_NEWPORT_SPAN_COMPOSITE
			dirty.add(rectangle(r.x0, r.x1 - 1, r.y0, r.y1));
#else
			dirty.add(rectangle(cliprect.min_x, cliprect.max_x, r.y0, r.y1));
#endif
			if (cx1 >= cx0)
				dirty.add(rectangle(std::max(cx0, cliprect.min_x), std::min(cx1, cliprect.max_x), std::max(cy0, r.y0), std::min(cy1, r.y1)));
		}
	}
#endif
#ifdef MAME_SGI_PENDING_RECTS_DIAG
	if (g_pr_print)
	{
		g_pr_print = false;
		osd_printf_info("PENDING_RECTS_DIAG pending valid=%d n=%d:", walk_dirty.valid(), walk_dirty.count());
		for (int i = 0; i < walk_dirty.count(); i++)
			osd_printf_info(" (%d,%d)-(%d,%d)", walk_dirty.rect(i).x0, walk_dirty.rect(i).y0, walk_dirty.rect(i).x1, walk_dirty.rect(i).y1);
		osd_printf_info("\n");
	}
#endif
#ifdef MAME_SGI_RECT_COUNT_STATS_DIAG
	{
		static uint64_t g_mame_sgi_rect_count_histogram[newport_dirty_rect::MAX_RECTS + 1] = {};
		static std::chrono::steady_clock::time_point g_mame_sgi_rect_count_next_print{};
		const int bucket = walk_dirty.valid() ? std::min(walk_dirty.count(), newport_dirty_rect::MAX_RECTS) : 0;
		g_mame_sgi_rect_count_histogram[bucket]++;
		const auto now = std::chrono::steady_clock::now();
		if (now >= g_mame_sgi_rect_count_next_print)
		{
			g_mame_sgi_rect_count_next_print = now + std::chrono::seconds(1);
			osd_printf_info("MAME_SGI_RECT_COUNT_STATS_DIAG histogram[fallback=0,1,2,3,4]=[%llu,%llu,%llu,%llu,%llu]\n",
					(unsigned long long)g_mame_sgi_rect_count_histogram[0],
					(unsigned long long)g_mame_sgi_rect_count_histogram[1],
					(unsigned long long)g_mame_sgi_rect_count_histogram[2],
					(unsigned long long)g_mame_sgi_rect_count_histogram[3],
					(unsigned long long)g_mame_sgi_rect_count_histogram[4]);
		}
	}
#endif
#ifdef MAME_SGI_ROW_RANGE_USED_DIAG
	// TEMPORARY diagnostic (2026-09-25) - NOT a real fix, just
	// visibility. Prints the ACTUAL row range(s) this recompute is about
	// to walk, once/second throughout the whole run (matching
	// MAME_SGI_RECT_COUNT_STATS_DIAG's own time-throttled design) - NOT a
	// "first N prints from boot" count. That was this diagnostic's first
	// version's real bug (2026-09-25): screen_update() runs continuously
	// from early boot, so a "first 40 real recomputes" budget could be
	// entirely exhausted during login-screen rendering, desktop/toolchest
	// icons appearing, or the app's own window being created - all
	// legitimately-broad transient activity - well before the actual
	// steady-state swap this diagnostic meant to characterize ever
	// started. Sampling continuously across the whole run instead avoids
	// front-loading on whatever happens to occur first.
	{
		static std::chrono::steady_clock::time_point g_mame_sgi_row_range_used_diag_next_print{};
		const auto now = std::chrono::steady_clock::now();
		if (now >= g_mame_sgi_row_range_used_diag_next_print)
		{
			g_mame_sgi_row_range_used_diag_next_print = now + std::chrono::seconds(1);
			for (int i = 0; i < num_row_ranges; i++)
			{
				osd_printf_info("MAME_SGI_ROW_RANGE_USED_DIAG rect=%d/%d row_start_y=%d row_end_y=%d span=%d cliprect=[%d,%d]\n",
						i, num_row_ranges, row_ranges[i].y0, row_ranges[i].y1,
						row_ranges[i].y1 - row_ranges[i].y0 + 1, cliprect.min_y, cliprect.max_y);
			}
		}
	}
#endif

	// X range deliberately stays the full cliprect width for now: the DID
	// mode-table walk below (begin_did_line()/next_did_line_entry()) can
	// only be walked forward from a row's start, not seeked to an
	// arbitrary starting column, so a real per-row x-clip needs that
	// walker extended first - left for a restructure of
	// this loop around DID runs.
	//
	// m_pending_dirty is already bitmap-relative by this point - the
	// rb2-sourced contribution was converted from raw VRAM rows at the
	// point it was merged in above (see that block's own comment for why
	// that has to happen before the merge, not here on the combined
	// rectangle), and the vc2/cursor-sourced contribution was bitmap-
	// relative from the start. No further conversion needed - used
	// directly as bitmap-row bounds.
	//
	// Deliberately NOT re-indenting the row loop's own (large) body below
	// for this outer wrap - keeps this change's diff reviewable. The
	// outer loop just repeats the same per-row work for each disjoint
	// range instead of one contiguous sweep.
	for (int range_idx = 0; range_idx < num_row_ranges; range_idx++)
	{
	const int row_start_y = row_ranges[range_idx].y0;
	const int row_end_y = row_ranges[range_idx].y1;
	const int sy_offset = row_start_y - cliprect.min_y;
	g_mame_sgi_stat_rows_walked += uint64_t(row_end_y - row_start_y + 1);

	/* loop over rows and copy to the destination */
	for (int y = row_start_y, sy = y_start + sy_offset; y <= row_end_y && sy < y_end; y++, sy++)
	{
		uint32_t *dest = &bitmap.pix(y, cliprect.min_x);
		const int vram_row = std::min(y + top_scanline, RB2_MAX_ROW);	// MAME_SGI_TOP_SCANLINE_FIX
		const uint32_t *src_rgbci = m_rb2->rgbci(vram_row);
		const uint32_t *src_cidaux = m_rb2->cidaux(vram_row);

		// Accumulates which XMAP mode-table slots (bit N = slot N) this
		// row's DID chain references, as each entry is fetched below -
		// written out to m_row_xmap_slot_mask[y] once the row is done.
		// See that member's own comment and mark_dirty_for_xmap_slot().
		uint32_t row_xmap_mask = 0;

#ifdef MAME_SGI_NEWPORT_SPAN_COMPOSITE
		// Walk this row's DID entries as runs of one display mode and
		// composite only each run's columns inside [span_x0, span_x1).
		// The walk is the per-pixel loop's own: the first entry applies
		// from the left edge, each later entry only where x equals its
		// column exactly (an entry at or left of the current column is
		// never reached, so its run extends to the row's end), and the
		// same entries are fetched in the same order for row_xmap_mask.
		const int span_x0 = row_ranges[range_idx].x0;
		const int span_x1 = row_ranges[range_idx].x1;
		const auto decode_mode = [popup_msb] (uint32_t table_entry)
		{
			composite_mode mode;
			mode.table_entry = table_entry;
			mode.pix_mode = (table_entry >> 8) & 3;
			mode.pix_size = (table_entry >> 10) & 3;
			mode.aux_pix_mode = (table_entry >> 16) & 7;
			mode.aux_msb = (table_entry >> 11) & 0x1f00;
			mode.popup_msb = popup_msb;
			switch ((table_entry >> 8) & 3)
			{
				case 0: mode.ci_msb = (table_entry & 0xf8) << 5; break;
				case 1: mode.ci_msb = 0x1d00; break;
				case 2: mode.ci_msb = 0x1e00; break;
				default: mode.ci_msb = 0x1f00; break;
			}
			return mode;
		};
		uint16_t curr_did_entry = m_vc2->begin_did_line(y);
		row_xmap_mask |= 1u << (curr_did_entry & 0x1f);
		composite_mode mode = decode_mode(m_xmap[0]->mode_entry(curr_did_entry & 0x1f));
		curr_did_entry = m_vc2->next_did_line_entry();
		row_xmap_mask |= 1u << (curr_did_entry & 0x1f);
		bool entry_reachable = true;
		for (int x = cliprect.min_x; x < cliprect.max_x; )
		{
			const int next_x = curr_did_entry >> 5;
			if (entry_reachable && next_x == x)
			{
				mode = decode_mode(m_xmap[0]->mode_entry(curr_did_entry & 0x1f));
				curr_did_entry = m_vc2->next_did_line_entry();
				row_xmap_mask |= 1u << (curr_did_entry & 0x1f);
				entry_reachable = false;	// the next entry can't take effect at this same column
				continue;
			}
			const int run_end = (next_x > x && next_x < cliprect.max_x) ? next_x : cliprect.max_x;
			const int a = std::max(x, span_x0);
			const int b = std::min(run_end, span_x1);
			if (a < b)
			{
				const int col = a - cliprect.min_x;
				composite_span(mode, palette, dest + col, src_rgbci + col, src_cidaux + col, b - a);
			}
			x = run_end;
			entry_reachable = true;
		}
#else
		// Fetch the initial DID entry
		uint16_t curr_did_entry = m_vc2->begin_did_line(y);
		row_xmap_mask |= 1u << (curr_did_entry & 0x1f);
		uint32_t table_entry = m_xmap[0]->mode_entry(curr_did_entry & 0x1f);
		uint8_t pix_mode = (table_entry >> 8) & 3;
		uint8_t pix_size = (table_entry >> 10) & 3;
		uint8_t aux_pix_mode = (table_entry >> 16) & 7;
		uint16_t aux_msb = (table_entry >> 11) & 0x1f00;
#ifdef MAME_SGI_LOWRES_BOOT_DIAG
		if (g_mame_sgi_lowres_seen_table_entries.find(table_entry) == g_mame_sgi_lowres_seen_table_entries.end()
				&& g_mame_sgi_lowres_pixmode_prints < 50)
		{
			g_mame_sgi_lowres_seen_table_entries.insert(table_entry);
			g_mame_sgi_lowres_pixmode_prints++;
			osd_printf_info(
					"MAME_SGI_LOWRES_BOOT_DIAG pixmode #%u (new table_entry): row=%d did_entry=%04x table_entry=%08x pix_mode=%u pix_size=%u aux_pix_mode=%u\n",
					g_mame_sgi_lowres_pixmode_prints, y, curr_did_entry, table_entry,
					pix_mode, pix_size, aux_pix_mode);
		}
#endif
		uint16_t ci_msb = 0;
		switch ((table_entry >> 8) & 3)
		{
			case 0: ci_msb = (table_entry & 0xf8) << 5; break;
			case 1: ci_msb = 0x1d00; break;
			case 2: ci_msb = 0x1e00; break;
			case 3: ci_msb = 0x1f00; break;
		}

		// Prepare for the next DID entry
		curr_did_entry = m_vc2->next_did_line_entry();
		row_xmap_mask |= 1u << (curr_did_entry & 0x1f);

		// loop over columns
		for (int x = cliprect.min_x; x < cliprect.max_x; x++)
		{
			if ((uint16_t)x == (curr_did_entry >> 5))
			{
				table_entry = m_xmap[0]->mode_entry(curr_did_entry & 0x1f);
				pix_mode = (table_entry >> 8) & 3;
				pix_size = (table_entry >> 10) & 3;
				aux_pix_mode = (table_entry >> 16) & 7;
				aux_msb = (table_entry >> 11) & 0x1f00;
				switch ((table_entry >> 8) & 3)
				{
					case 0: ci_msb = (table_entry & 0xf8) << 5; break;
					case 1: ci_msb = 0x1d00; break;
					case 2: ci_msb = 0x1e00; break;
					case 3: ci_msb = 0x1f00; break;
				}
				curr_did_entry = m_vc2->next_did_line_entry();
				row_xmap_mask |= 1u << (curr_did_entry & 0x1f);
			}

			// Cursor pixels are NOT composited here anymore - background
			// only in this loop. See the separate overlay stamp pass
			// after this row's x-loop, which draws the cursor over
			// whatever this pass wrote, scoped to just the cursor's own
			// small x-span instead of checked per-pixel across the whole
			// row width.
			bool pixel_replaced = false;
			if (*src_cidaux & 0xcc)
			{
				const uint32_t src = (*src_cidaux >> 2) & 3;
				*dest++ = palette[popup_msb | src];
				pixel_replaced = true;
			}
			else if (aux_pix_mode != 0)
			{
				if (m_xmap[0]->is_8bpp())
				{
					switch (aux_pix_mode)
					{
						case 1: // 2-Bit Underlay, shown where the normal planes are zero (MAME_SGI_UNDERLAY_FIX)
							if (normal_planes_zero(table_entry, *src_rgbci))
							{
								*dest++ = palette[aux_msb | ((*src_cidaux >> 8) & 3)];
								pixel_replaced = true;
							}
							break;
						case 2: // 2-Bit Overlay
						{
							const uint32_t pix_in = (*src_cidaux >> 8) & 3;
							if (pix_in)
							{
								*dest++ = palette[aux_msb | pix_in];
								pixel_replaced = true;
							}
							break;
						}
						case 6: // 1-Bit Overlay
						{
							const uint32_t shift = BIT(table_entry, 1) ? 9 : 8;
							const uint32_t pix_in = (*src_cidaux >> shift) & 1;
							if (pix_in)
							{
								*dest++ = palette[aux_msb | pix_in];
								pixel_replaced = true;
							}
							break;
						}
						case 7: // 1-Bit Overlay, 1-Bit Underlay
						{
							const uint32_t pix_in = (*src_cidaux >> 8) & 1;
							if (pix_in)
							{
								*dest++ = palette[aux_msb | pix_in];
								pixel_replaced = true;
							}
							else if (normal_planes_zero(table_entry, *src_rgbci))	// MAME_SGI_UNDERLAY_FIX
							{
								*dest++ = palette[aux_msb | ((*src_cidaux >> 9) & 1)];
								pixel_replaced = true;
							}
							break;
						}
						default:
							break;
					}
				}
				else
				{
					switch (aux_pix_mode)
					{
						case 1: // 8-Bit Underlay, shown where the normal planes are zero (MAME_SGI_UNDERLAY_FIX)
							if (normal_planes_zero(table_entry, *src_rgbci))
							{
								*dest++ = palette[aux_msb | ((*src_cidaux >> 8) & 0xff)];
								pixel_replaced = true;
							}
							break;
						case 2: // 8-Bit Overlay
						{
							const uint32_t pix_in = ((*src_cidaux >> 8) & 0xf) | ((*src_cidaux >> 16) & 0xf0);
							if (pix_in)
							{
								*dest++ = palette[aux_msb | pix_in];
								pixel_replaced = true;
							}
							break;
						}
						case 6: // 4-Bit Overlay
						{
							const uint32_t shift = BIT(table_entry, 1) ? 12 : 8;
							const uint32_t pix_in = (*src_cidaux >> shift) & 0xf;
							if (pix_in)
							{
								*dest++ = palette[aux_msb | pix_in];
								pixel_replaced = true;
							}
							break;
						}
						case 7: // 4-Bit Overlay, 4-Bit Underlay
						{
							const uint32_t pix_in = (*src_cidaux >> 8) & 0xf;
							if (pix_in)
							{
								*dest++ = palette[aux_msb | pix_in];
								pixel_replaced = true;
							}
							else if (normal_planes_zero(table_entry, *src_rgbci))	// MAME_SGI_UNDERLAY_FIX
							{
								*dest++ = palette[aux_msb | ((*src_cidaux >> 12) & 0xf)];
								pixel_replaced = true;
							}
							break;
						}
						default:
							break;
					}
				}
			}

			if (!pixel_replaced)
			{
				switch (pix_mode)
				{
					case 0: // CI
						switch (pix_size)
						{
							case 0: // 4bpp
							{
								const uint8_t shift = BIT(table_entry, 0) ? 4 : 0;
								const uint32_t pix_in = *src_rgbci;
								*dest++ = palette[ci_msb | ((pix_in >> shift) & 0x0f)];
								break;
							}
							case 1: // 8bpp
							{
								// MAME_SGI_CI8_BUFSEL_FIX: buffer 1 of a double-buffered 8bpp visual is bits 8-15
								const uint8_t shift = BIT(table_entry, 0) ? 8 : 0;
								*dest++ = palette[ci_msb | (uint8_t)(*src_rgbci >> shift)];
								break;
							}
							case 2: // 12bpp
							{
								const uint8_t shift = BIT(table_entry, 0) ? 12 : 0;
								const uint32_t pix_in = (*src_rgbci >> shift) & 0x00000fff;
								*dest++ = palette[(ci_msb & 0x1000) | pix_in];
								break;
							}
							case 3: // 24bpp (not yet supported)
								dest++;
								break;
						}
						break;
					case 1: // RGB Map0
						switch (pix_size)
						{
							case 0: // 4bpp
							{
								const uint8_t shift = BIT(table_entry, 0) ? 4 : 0;
								const uint8_t pix_in = (uint8_t)(*src_rgbci >> shift);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
								*dest++ = m_lut_rgb4[pix_in & 0xf];
#else
								*dest++ = convert_4bpp_bgr_to_24bpp_rgb(pix_in);
#endif
								break;
							}
							case 1: // 8bpp
							{
								const uint8_t shift = BIT(table_entry, 0) ? 8 : 0;
								const uint8_t pix_in = (uint8_t)(*src_rgbci >> shift);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
								*dest++ = m_lut_rgb8[pix_in];
#else
								*dest++ = convert_8bpp_bgr_to_24bpp_rgb(pix_in);
#endif
								break;
							}
							case 2: // 12bpp
							{
								const uint8_t shift = BIT(table_entry, 0) ? 12 : 0;
								const uint16_t pix_in = (uint16_t)(*src_rgbci >> shift);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
								*dest++ = m_lut_rgb12[pix_in & 0xfff];
#else
								*dest++ = convert_12bpp_bgr_to_24bpp_rgb(pix_in);
#endif
								break;
							}
							case 3: // 24bpp
							{
								const uint32_t pix_in = (uint32_t)*src_rgbci;
								const uint8_t r = (uint8_t)(pix_in >> 0);
								const uint8_t g = (uint8_t)(pix_in >> 8);
								const uint8_t b = (uint8_t)(pix_in >> 16);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
								*dest++ = 0xff000000 | m_ramdac_lut_r[r] | m_ramdac_lut_g[g] | m_ramdac_lut_b[b];
#else
								*dest++ = (r << 16) | (g << 8) | b;
#endif
								break;
							}
						}
						break;
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
					case 2: // RGB Map1 (not yet supported)
						*dest++ = ramdac_map(0xff00ff);
						break;
					case 3: // RGB Map2 (not yet supported)
						*dest++ = ramdac_map(0x00ff00);
						break;
#else
					case 2: // RGB Map1 (not yet supported)
						*dest++ = 0xff00ff;
						break;
					case 3: // RGB Map2 (not yet supported)
						*dest++ = 0x00ff00;
						break;
#endif
				}
			}

#ifndef MAME_SGI_NEWPORT_LUT_FOLD
			ramdac_remap(dest - 1);
#endif

			src_rgbci++;
			src_cidaux++;
		}
#endif

		// Cursor overlay stamp - separate small pass over just the
		// cursor's own x-span, only for rows it actually occupies,
		// drawn on top of whatever the background pass above just wrote.
		if (m_vc2->cursor_enabled())
		{
			int cx0, cy0, cx1, cy1;
			m_vc2->cursor_bounds(cx0, cy0, cx1, cy1);
			if (y >= cy0 && y <= cy1)
			{
				int stamp_x0 = cx0 > cliprect.min_x ? cx0 : cliprect.min_x;
				int stamp_x1 = cx1 < cliprect.max_x - 1 ? cx1 : cliprect.max_x - 1;
				for (int x = stamp_x0; x <= stamp_x1; x++)
				{
					const uint8_t cursor_pixel = m_vc2->get_cursor_pixel(x, y);
					if (cursor_pixel)
					{
						uint32_t &dest = bitmap.pix(y, x);
						dest = palette[cursor_msb | cursor_pixel];
						// The main loop below runs every pixel, cursor
						// included, through the RAMDAC's own LUTs
						// (ramdac_remap()) before this compositing pass
						// was split out - omitting it here left the raw
						// pre-LUT palette value on screen (wrong cursor
						// color, e.g. white instead of red).
#ifndef MAME_SGI_NEWPORT_LUT_FOLD
						ramdac_remap(&dest);
#endif
					}
				}
			}
		}

		// Bounds-check rather than trust y is always in range - it comes
		// from cliprect/m_pending_dirty, both guest/host-configurable, and
		// an out-of-range write here would corrupt an adjacent member
		// instead of just losing one row's XMAP-slot tracking.
		if (y >= 0 && y < 1088)
			m_row_xmap_slot_mask[y] = row_xmap_mask;
	}
	} // end of "for (int range_idx ...)" - see its own opening comment

	}

mame_sgi_composite_done:

#if ENABLE_NEWVIEW_LOG
	if (machine().input().code_pressed_once(KEYCODE_TILDE))
	{
		if (m_newview_log == nullptr)
			start_logging();
		else
			stop_logging();
	}
#endif

#if ENABLE_NEWVIEW_LOG
	if (m_newview_log != nullptr)
	{
		uint32_t offset_lo = 0x80000000;
		uint32_t data_hi = 0;
		uint32_t data_lo = 0;
		uint32_t mem_mask_hi = 0;
		uint32_t mem_mask_lo = 0;

		fwrite(&offset_lo, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&data_hi, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&data_lo, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&mem_mask_hi, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&mem_mask_lo, sizeof(uint32_t), 1, m_newview_log);
	}
#endif

#ifdef MAME_SGI_NEWPORT_COMPOSITE_TIMING_DIAG
	osd_ticks_t mame_sgi_composite_t1 = osd_ticks();
	s_mame_sgi_composite_calls++;
	s_mame_sgi_composite_ticks += (mame_sgi_composite_t1 - mame_sgi_composite_t0);
	if (mame_sgi_composite_t1 >= s_mame_sgi_composite_next_dump)
	{
		osd_printf_info("MAME_SGI_NEWPORT_COMPOSITE_TIMING_DIAG: calls=%12llu ticks=%12llu avg_ticks=%llu\n",
			(unsigned long long)s_mame_sgi_composite_calls,
			(unsigned long long)s_mame_sgi_composite_ticks,
			s_mame_sgi_composite_calls ? (unsigned long long)(s_mame_sgi_composite_ticks / s_mame_sgi_composite_calls) : 0ULL);
		s_mame_sgi_composite_calls = 0;
		s_mame_sgi_composite_ticks = 0;
		s_mame_sgi_composite_next_dump = mame_sgi_composite_t1 + osd_ticks_per_second() * 5;
	}
#endif

	return mame_sgi_result;
}

#ifdef MAME_SGI_NEWPORT_SPAN_COMPOSITE
#ifndef MAME_SGI_NEWPORT_LUT_FOLD
#error MAME_SGI_NEWPORT_SPAN_COMPOSITE needs MAME_SGI_NEWPORT_LUT_FOLD
#endif
// TEMPORARY diag: composited pixels per display mode, printed every 5 s
// #define MAME_SGI_SPAN_MODE_DIAG
#ifdef MAME_SGI_SPAN_MODE_DIAG
static uint64_t s_span_mode_pixels[128];
#endif

// Composite count pixels of one DID run. The common modes without an aux
// plane get their own loops; everything else goes through the general
// per-pixel code.
void newport_base_device::composite_span(const composite_mode &mode, const uint32_t *palette, uint32_t *dest,
		const uint32_t *src_rgbci, const uint32_t *src_cidaux, int count)
{
	const uint32_t table_entry = mode.table_entry;
	const uint8_t pix_mode = mode.pix_mode;
	const uint8_t pix_size = mode.pix_size;
	const uint8_t aux_pix_mode = mode.aux_pix_mode;
	const uint16_t aux_msb = mode.aux_msb;
	const uint16_t ci_msb = mode.ci_msb;
	const uint16_t popup_msb = mode.popup_msb;
#ifdef MAME_SGI_SPAN_MODE_DIAG
	{
		static std::chrono::steady_clock::time_point next_print{};
		s_span_mode_pixels[(aux_pix_mode << 4) | (pix_mode << 2) | pix_size] += count;
		const auto now = std::chrono::steady_clock::now();
		if (now >= next_print)
		{
			next_print = now + std::chrono::seconds(5);
			osd_printf_info("MAME_SGI_SPAN_MODE_DIAG (aux,mode,size)=pixels:");
			for (int i = 0; i < 128; i++)
				if (s_span_mode_pixels[i])
					osd_printf_info(" (%d,%d,%d)=%llu", i >> 4, (i >> 2) & 3, i & 3, (unsigned long long)s_span_mode_pixels[i]);
			osd_printf_info("\n");
		}
	}
#endif

	if (aux_pix_mode == 0)
	{
		switch ((pix_mode << 2) | pix_size)
		{
		case (0 << 2) | 0: // CI 4bpp
		{
			const uint8_t shift = BIT(table_entry, 0) ? 4 : 0;
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)] : palette[ci_msb | ((src_rgbci[i] >> shift) & 0x0f)];
			}
			return;
		}
		case (0 << 2) | 1: // CI 8bpp
		{
			// MAME_SGI_CI8_BUFSEL_FIX: buffer 1 of a double-buffered 8bpp visual is bits 8-15
			const uint8_t shift = BIT(table_entry, 0) ? 8 : 0;
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)] : palette[ci_msb | (uint8_t)(src_rgbci[i] >> shift)];
			}
			return;
		}
		case (0 << 2) | 2: // CI 12bpp
		{
			const uint8_t shift = BIT(table_entry, 0) ? 12 : 0;
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)] : palette[(ci_msb & 0x1000) | ((src_rgbci[i] >> shift) & 0x0fff)];
			}
			return;
		}
		case (1 << 2) | 0: // RGB Map0 4bpp
		{
			const uint8_t shift = BIT(table_entry, 0) ? 4 : 0;
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)] : m_lut_rgb4[(uint8_t)(src_rgbci[i] >> shift) & 0x0f];
			}
			return;
		}
		case (1 << 2) | 1: // RGB Map0 8bpp
		{
			const uint8_t shift = BIT(table_entry, 0) ? 8 : 0;
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)] : m_lut_rgb8[(uint8_t)(src_rgbci[i] >> shift)];
			}
			return;
		}
		case (1 << 2) | 2: // RGB Map0 12bpp
		{
			const uint8_t shift = BIT(table_entry, 0) ? 12 : 0;
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)] : m_lut_rgb12[(uint16_t)(src_rgbci[i] >> shift) & 0x0fff];
			}
			return;
		}
		case (1 << 2) | 3: // RGB Map0 24bpp
			for (int i = 0; i < count; i++)
			{
				const uint32_t aux = src_cidaux[i];
				const uint32_t pix_in = src_rgbci[i];
				dest[i] = (aux & 0xcc) ? palette[popup_msb | ((aux >> 2) & 3)]
						: 0xff000000 | m_ramdac_lut_r[(uint8_t)pix_in] | m_ramdac_lut_g[(uint8_t)(pix_in >> 8)] | m_ramdac_lut_b[(uint8_t)(pix_in >> 16)];
			}
			return;
		default:
			break;
		}
	}

	// general case: the per-pixel compositor
	for (int i = 0; i < count; i++)
	{

		bool pixel_replaced = false;
		if (*src_cidaux & 0xcc)
		{
			const uint32_t src = (*src_cidaux >> 2) & 3;
			*dest++ = palette[popup_msb | src];
			pixel_replaced = true;
		}
		else if (aux_pix_mode != 0)
		{
			if (m_xmap[0]->is_8bpp())
			{
				switch (aux_pix_mode)
				{
					case 1: // 2-Bit Underlay, shown where the normal planes are zero (MAME_SGI_UNDERLAY_FIX)
						if (normal_planes_zero(table_entry, *src_rgbci))
						{
							*dest++ = palette[aux_msb | ((*src_cidaux >> 8) & 3)];
							pixel_replaced = true;
						}
						break;
					case 2: // 2-Bit Overlay
					{
						const uint32_t pix_in = (*src_cidaux >> 8) & 3;
						if (pix_in)
						{
							*dest++ = palette[aux_msb | pix_in];
							pixel_replaced = true;
						}
						break;
					}
					case 6: // 1-Bit Overlay
					{
						const uint32_t shift = BIT(table_entry, 1) ? 9 : 8;
						const uint32_t pix_in = (*src_cidaux >> shift) & 1;
						if (pix_in)
						{
							*dest++ = palette[aux_msb | pix_in];
							pixel_replaced = true;
						}
						break;
					}
					case 7: // 1-Bit Overlay, 1-Bit Underlay
					{
						const uint32_t pix_in = (*src_cidaux >> 8) & 1;
						if (pix_in)
						{
							*dest++ = palette[aux_msb | pix_in];
							pixel_replaced = true;
						}
						else if (normal_planes_zero(table_entry, *src_rgbci))	// MAME_SGI_UNDERLAY_FIX
						{
							*dest++ = palette[aux_msb | ((*src_cidaux >> 9) & 1)];
							pixel_replaced = true;
						}
						break;
					}
					default:
						break;
				}
			}
			else
			{
				switch (aux_pix_mode)
				{
					case 1: // 8-Bit Underlay, shown where the normal planes are zero (MAME_SGI_UNDERLAY_FIX)
						if (normal_planes_zero(table_entry, *src_rgbci))
						{
							*dest++ = palette[aux_msb | ((*src_cidaux >> 8) & 0xff)];
							pixel_replaced = true;
						}
						break;
					case 2: // 8-Bit Overlay
					{
						const uint32_t pix_in = ((*src_cidaux >> 8) & 0xf) | ((*src_cidaux >> 16) & 0xf0);
						if (pix_in)
						{
							*dest++ = palette[aux_msb | pix_in];
							pixel_replaced = true;
						}
						break;
					}
					case 6: // 4-Bit Overlay
					{
						const uint32_t shift = BIT(table_entry, 1) ? 12 : 8;
						const uint32_t pix_in = (*src_cidaux >> shift) & 0xf;
						if (pix_in)
						{
							*dest++ = palette[aux_msb | pix_in];
							pixel_replaced = true;
						}
						break;
					}
					case 7: // 4-Bit Overlay, 4-Bit Underlay
					{
						const uint32_t pix_in = (*src_cidaux >> 8) & 0xf;
						if (pix_in)
						{
							*dest++ = palette[aux_msb | pix_in];
							pixel_replaced = true;
						}
						else if (normal_planes_zero(table_entry, *src_rgbci))	// MAME_SGI_UNDERLAY_FIX
						{
							*dest++ = palette[aux_msb | ((*src_cidaux >> 12) & 0xf)];
							pixel_replaced = true;
						}
						break;
					}
					default:
						break;
				}
			}
		}

		if (!pixel_replaced)
		{
			switch (pix_mode)
			{
				case 0: // CI
					switch (pix_size)
					{
						case 0: // 4bpp
						{
							const uint8_t shift = BIT(table_entry, 0) ? 4 : 0;
							const uint32_t pix_in = *src_rgbci;
							*dest++ = palette[ci_msb | ((pix_in >> shift) & 0x0f)];
							break;
						}
						case 1: // 8bpp
						{
							// MAME_SGI_CI8_BUFSEL_FIX: buffer 1 of a double-buffered 8bpp visual is bits 8-15
							const uint8_t shift = BIT(table_entry, 0) ? 8 : 0;
							*dest++ = palette[ci_msb | (uint8_t)(*src_rgbci >> shift)];
							break;
						}
						case 2: // 12bpp
						{
							const uint8_t shift = BIT(table_entry, 0) ? 12 : 0;
							const uint32_t pix_in = (*src_rgbci >> shift) & 0x00000fff;
							*dest++ = palette[(ci_msb & 0x1000) | pix_in];
							break;
						}
						case 3: // 24bpp (not yet supported)
							dest++;
							break;
					}
					break;
				case 1: // RGB Map0
					switch (pix_size)
					{
						case 0: // 4bpp
						{
							const uint8_t shift = BIT(table_entry, 0) ? 4 : 0;
							const uint8_t pix_in = (uint8_t)(*src_rgbci >> shift);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
							*dest++ = m_lut_rgb4[pix_in & 0xf];
#else
							*dest++ = convert_4bpp_bgr_to_24bpp_rgb(pix_in);
#endif
							break;
						}
						case 1: // 8bpp
						{
							const uint8_t shift = BIT(table_entry, 0) ? 8 : 0;
							const uint8_t pix_in = (uint8_t)(*src_rgbci >> shift);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
							*dest++ = m_lut_rgb8[pix_in];
#else
							*dest++ = convert_8bpp_bgr_to_24bpp_rgb(pix_in);
#endif
							break;
						}
						case 2: // 12bpp
						{
							const uint8_t shift = BIT(table_entry, 0) ? 12 : 0;
							const uint16_t pix_in = (uint16_t)(*src_rgbci >> shift);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
							*dest++ = m_lut_rgb12[pix_in & 0xfff];
#else
							*dest++ = convert_12bpp_bgr_to_24bpp_rgb(pix_in);
#endif
							break;
						}
						case 3: // 24bpp
						{
							const uint32_t pix_in = (uint32_t)*src_rgbci;
							const uint8_t r = (uint8_t)(pix_in >> 0);
							const uint8_t g = (uint8_t)(pix_in >> 8);
							const uint8_t b = (uint8_t)(pix_in >> 16);
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
							*dest++ = 0xff000000 | m_ramdac_lut_r[r] | m_ramdac_lut_g[g] | m_ramdac_lut_b[b];
#else
							*dest++ = (r << 16) | (g << 8) | b;
#endif
							break;
						}
					}
					break;
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
				case 2: // RGB Map1 (not yet supported)
					*dest++ = ramdac_map(0xff00ff);
					break;
				case 3: // RGB Map2 (not yet supported)
					*dest++ = ramdac_map(0x00ff00);
					break;
#else
				case 2: // RGB Map1 (not yet supported)
					*dest++ = 0xff00ff;
					break;
				case 3: // RGB Map2 (not yet supported)
					*dest++ = 0x00ff00;
					break;
#endif
			}
		}

		src_rgbci++;
		src_cidaux++;
	}
}
#endif
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
void newport_base_device::lut_fold_update()
{
	uint32_t lo, hi;
	const bool cmap_dirty = m_cmap[0]->take_dirty_range(lo, hi);
	if (m_lut_fold_dirty)
	{
		lo = 0;
		hi = 0xffff;
		for (uint32_t i = 0; i < 0x10; i++)
			m_lut_rgb4[i] = ramdac_map(convert_4bpp_bgr_to_24bpp_rgb(i));
		for (uint32_t i = 0; i < 0x100; i++)
			m_lut_rgb8[i] = ramdac_map(convert_8bpp_bgr_to_24bpp_rgb(i));
		for (uint32_t i = 0; i < 0x1000; i++)
			m_lut_rgb12[i] = ramdac_map(convert_12bpp_bgr_to_24bpp_rgb(i));
		m_lut_fold_dirty = false;
	}
	else if (!cmap_dirty)
	{
		return;
	}
	const uint32_t *const palette = m_cmap[0]->palette_base();
	for (uint32_t i = lo; i <= hi; i++)
		m_lut_palette[i] = ramdac_map(palette[i]);
}

#endif
void newport_base_device::ramdac_remap(uint32_t *dest)
{
	uint32_t out = 0xff000000;
	out |= m_ramdac_lut_r[(uint8_t)(*dest >> 16)];
	out |= m_ramdac_lut_g[(uint8_t)(*dest >>  8)];
	out |= m_ramdac_lut_b[(uint8_t)(*dest >>  0)];
	*dest = out;
}

uint32_t newport_base_device::convert_4bpp_bgr_to_8bpp(uint8_t pix_in)
{
	const uint8_t r = 0xff * BIT(pix_in, 0);
	const uint8_t g = (0xaa * BIT(pix_in, 2)) | (0x55 * BIT(pix_in, 1));
	const uint8_t b = 0xff * BIT(pix_in, 3);
	return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
}

uint32_t newport_base_device::convert_4bpp_bgr_to_12bpp(uint8_t pix_in)
{
	const uint32_t r = 0xff * BIT(pix_in, 0);
	const uint32_t g = (0xaa * BIT(pix_in, 2)) | (0x55 * BIT(pix_in, 1));
	const uint32_t b = 0xff * BIT(pix_in, 3);
	return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
}

uint32_t newport_base_device::convert_4bpp_bgr_to_24bpp(uint8_t pix_in)
{
	const uint8_t r = 0xff * BIT(pix_in, 0);
	const uint8_t g = (0xaa * BIT(pix_in, 2)) | (0x55 * BIT(pix_in, 1));
	const uint8_t b = 0xff * BIT(pix_in, 3);
	return (b << 16) | (g << 8) | r;
}

uint32_t newport_base_device::convert_8bpp_bgr_to_4bpp(uint8_t pix_in)
{
	const uint8_t r = (0x92 * BIT(pix_in, 2)) | (0x49 * BIT(pix_in, 1)) | (0x24 * BIT(pix_in, 0));
	const uint8_t g = (0x92 * BIT(pix_in, 5)) | (0x49 * BIT(pix_in, 4)) | (0x24 * BIT(pix_in, 3));
	const uint8_t b = (0xaa * BIT(pix_in, 7)) | (0x55 * BIT(pix_in, 6));
	return (BIT(b, 7) << 3) | ((g & 0xc0) >> 5) | BIT(r, 7);
}

uint32_t newport_base_device::convert_8bpp_bgr_to_12bpp(uint8_t pix_in)
{
	const uint8_t r = (0x92 * BIT(pix_in, 2)) | (0x49 * BIT(pix_in, 1)) | (0x24 * BIT(pix_in, 0));
	const uint8_t g = (0x92 * BIT(pix_in, 5)) | (0x49 * BIT(pix_in, 4)) | (0x24 * BIT(pix_in, 3));
	const uint8_t b = (0xaa * BIT(pix_in, 7)) | (0x55 * BIT(pix_in, 6));
	return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
}

uint32_t newport_base_device::convert_8bpp_bgr_to_24bpp(uint8_t pix_in)
{
	const uint8_t r = (0x92 * BIT(pix_in, 2)) | (0x49 * BIT(pix_in, 1)) | (0x24 * BIT(pix_in, 0));
	const uint8_t g = (0x92 * BIT(pix_in, 5)) | (0x49 * BIT(pix_in, 4)) | (0x24 * BIT(pix_in, 3));
	const uint8_t b = (0xaa * BIT(pix_in, 7)) | (0x55 * BIT(pix_in, 6));
	return (b << 16) | (g << 8) | r;
}

uint32_t newport_base_device::convert_12bpp_bgr_to_4bpp(uint16_t pix_in)
{
	const uint8_t r = 0x11 * ((pix_in >> 0) & 0xf);
	const uint8_t g = 0x11 * ((pix_in >> 4) & 0xf);
	const uint8_t b = 0x11 * ((pix_in >> 8) & 0xf);
	return (BIT(b, 7) << 3) | ((g & 0xc0) >> 5) | BIT(r, 7);
}

uint32_t newport_base_device::convert_12bpp_bgr_to_8bpp(uint16_t pix_in)
{
	const uint8_t r = 0x11 * ((pix_in >> 0) & 0xf);
	const uint8_t g = 0x11 * ((pix_in >> 4) & 0xf);
	const uint8_t b = 0x11 * ((pix_in >> 8) & 0xf);
	return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
}

uint32_t newport_base_device::convert_12bpp_bgr_to_24bpp(uint16_t pix_in)
{
	const uint8_t r = 0x11 * ((pix_in >> 0) & 0xf);
	const uint8_t g = 0x11 * ((pix_in >> 4) & 0xf);
	const uint8_t b = 0x11 * ((pix_in >> 8) & 0xf);
	return (b << 16) | (g << 8) | r;
}

uint32_t newport_base_device::convert_24bpp_bgr_to_4bpp(uint32_t pix_in)
{
	const uint8_t r = (uint8_t)(pix_in >> 0);
	const uint8_t g = (uint8_t)(pix_in >> 8);
	const uint8_t b = (uint8_t)(pix_in >> 16);
	return (BIT(b, 7) << 3) | ((g & 0xc0) >> 5) | BIT(r, 7);
}

uint32_t newport_base_device::convert_24bpp_bgr_to_8bpp(uint32_t pix_in)
{
	const uint8_t r = (uint8_t)(pix_in >> 0);
	const uint8_t g = (uint8_t)(pix_in >> 8);
	const uint8_t b = (uint8_t)(pix_in >> 16);
	return (b & 0xc0) | ((g & 0xe0) >> 2) | ((r & 0xe0) >> 5);
}

uint32_t newport_base_device::convert_24bpp_bgr_to_12bpp(uint32_t pix_in)
{
	const uint8_t r = (uint8_t)(pix_in >> 0);
	const uint8_t g = (uint8_t)(pix_in >> 8);
	const uint8_t b = (uint8_t)(pix_in >> 16);
	return ((b & 0xf0) << 4) | (g & 0xf0) | ((r & 0xf0) >> 4);
}

uint32_t newport_base_device::convert_4bpp_bgr_to_24bpp_rgb(uint8_t pix_in)
{
	const uint8_t r = 0xff * BIT(pix_in, 0);
	const uint8_t g = (0xaa * BIT(pix_in, 2)) | (0x55 * BIT(pix_in, 1));
	const uint8_t b = 0xff * BIT(pix_in, 3);
	return (r << 16) | (g << 8) | b;
}

uint32_t newport_base_device::convert_8bpp_bgr_to_24bpp_rgb(uint8_t pix_in)
{
	const uint8_t r = (0x92 * BIT(pix_in, 2)) | (0x49 * BIT(pix_in, 1)) | (0x24 * BIT(pix_in, 0));
	const uint8_t g = (0x92 * BIT(pix_in, 5)) | (0x49 * BIT(pix_in, 4)) | (0x24 * BIT(pix_in, 3));
	const uint8_t b = (0xaa * BIT(pix_in, 7)) | (0x55 * BIT(pix_in, 6));
	return (r << 16) | (g << 8) | b;
}

uint32_t newport_base_device::convert_12bpp_bgr_to_24bpp_rgb(uint16_t pix_in)
{
	const uint8_t r = 0x11 * ((pix_in >> 0) & 0xf);
	const uint8_t g = 0x11 * ((pix_in >> 4) & 0xf);
	const uint8_t b = 0x11 * ((pix_in >> 8) & 0xf);
	return (r << 16) | (g << 8) | b;
}

void newport_base_device::ramdac_write(uint32_t data)
{
	switch (m_rex3.m_dcb_reg_select)
	{
	case 0:
		LOGMASKED(LOG_RAMDAC, "RAMDAC LUT index write: %08x\n", data);
		m_ramdac_lut_index = (uint8_t)data;
		break;
	case 1:
		m_ramdac_lut_r[m_ramdac_lut_index] = (uint8_t)(data >>  8) << 16;
		m_ramdac_lut_g[m_ramdac_lut_index] = (uint8_t)(data >> 16) << 8;
		m_ramdac_lut_b[m_ramdac_lut_index] = (uint8_t)(data >> 24) << 0;
		m_ramdac_lut_index++;
#ifdef MAME_SGI_NEWPORT_LUT_FOLD
		lut_fold_invalidate();
#endif
		// MAME_SGI_CMAP_DIRTY_FIX: every displayed pixel goes through the LUTs
		mark_full_dirty();
		LOGMASKED(LOG_RAMDAC, "RAMDAC LUT entry write: %08x\n", data);
		break;
	default:
		LOGMASKED(LOG_RAMDAC | LOG_UNKNOWN, "Unknown RAMDAC register %02x write: %08x\n", m_rex3.m_dcb_reg_select, data);
		break;
	}
}

void newport_base_device::vrint_w(int state)
{
	if (state)
	{
		g_mame_sgi_stat_vrint_asserts++;
		m_rex3.m_status |= STATUS_VRINT;
		m_gio64->interrupt<2>(ASSERT_LINE);
	}
}

// TODO: Figure out a better way of doing this
void newport_base_device::update_screen_size(int state)
{
	const int x_start = m_vc2->readout_x0();
	const int y_start = m_vc2->readout_y0();
	const int x_end = m_vc2->readout_x1();
	const int y_end = m_vc2->readout_y1();

#ifdef MAME_SGI_LOWRES_BOOT_DIAG
	g_mame_sgi_lowres_resize_calls++;
	osd_printf_info(
			"MAME_SGI_LOWRES_BOOT_DIAG resize #%u: readout=(%d,%d)-(%d,%d) new_size=%dx%d\n",
			g_mame_sgi_lowres_resize_calls, x_start, y_start, x_end, y_end,
			x_end - x_start, y_end - y_start);
#endif

	m_screen->set_size((uint16_t)(x_end - x_start), (uint16_t)(y_end - y_start));
	// MAME_SGI_DISPLAY_GEOMETRY_FIX: the VC2 readout spans 8 columns more
	// than the displayed picture (1288 for 1280x1024, 1032 for 1024x768).
	// IRIX keeps the leading columns black and draws through an x window
	// offset, so the picture starts 2 columns in; the rest is overscan.
	// Showing only the picture also avoids scaling it to fit the window.
	constexpr int READOUT_EXTRA = 8, PICTURE_X0 = 2;
	const int width = x_end - x_start;
	if (width > READOUT_EXTRA)
		m_screen->set_visarea(PICTURE_X0, PICTURE_X0 + width - READOUT_EXTRA - 1, 0, y_end - y_start - 1);
	else
		m_screen->set_visarea_full();
}

uint64_t newport_base_device::rex3_r(offs_t offset, uint64_t mem_mask)
{
	uint64_t ret = 0;
#ifdef MAME_SGI_REX3_READ_NO_PREFETCH
	// A GO read of the host data port with prefetch off runs its command
	// before the data port is returned, not after.
	const bool go_before_read = (offset & 0x00000100) && (offset & ~(0x800/8)) == 0x0230/8 && !BIT(m_rex3.m_draw_mode1, 26);
	if (go_before_read)
		do_rex3_command();
#else
	const bool go_before_read = false;
#endif
	switch (offset & ~(0x800/8))
	{
	case 0x0000/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Draw Mode 1 Read: %08x\n", m_rex3.m_draw_mode1);
			ret |= (uint64_t)m_rex3.m_draw_mode1 << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Draw Mode 0 Read: %08x\n", m_rex3.m_draw_mode0);
			ret |= m_rex3.m_draw_mode0;
		}
		break;
	case 0x0008/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Line Stipple Mode Read: %08x\n", m_rex3.m_ls_mode);
			ret |= (uint64_t)m_rex3.m_ls_mode << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Line Stipple Pattern Read: %08x\n", m_rex3.m_ls_pattern);
			ret |= m_rex3.m_ls_pattern;
		}
		break;
	case 0x0010/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Line Stipple Pattern (Save) Read: %08x\n", m_rex3.m_ls_pattern_saved);
			ret |= (uint64_t)m_rex3.m_ls_pattern_saved << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Pattern Register Read: %08x\n", m_rex3.m_z_pattern);
			ret |= m_rex3.m_z_pattern;
		}
		break;
	case 0x0018/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Opaque Pattern / Blendfunc Dest Color Read: %08x\n", m_rex3.m_color_back);
			ret |= (uint64_t)m_rex3.m_color_back << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 VRAM Fastclear Color Read: %08x\n", m_rex3.m_color_vram);
			ret |= m_rex3.m_color_vram;
		}
		break;
	case 0x0020/8:
		LOGMASKED(LOG_REX3, "REX3 AFUNCTION Reference Alpha Read: %08x\n", m_rex3.m_alpha_ref);
		ret |= (uint64_t)m_rex3.m_alpha_ref << 32;
		break;
	case 0x0028/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 0 X Min/Max Read: %08x\n", m_rex3.m_smask_x[0]);
			ret |= (uint64_t)m_rex3.m_smask_x[0] << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 0 Y Min/Max Read: %08x\n", m_rex3.m_smask_y[0]);
			ret |= m_rex3.m_smask_y[0];
		}
		break;
	case 0x0030/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Line/Span Setup Read: %08x\n", m_rex3.m_setup);
			ret |= (uint64_t)m_rex3.m_setup << 32;
		}
		break;
	case 0x0100/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "%s: REX3 X Start Read: %08x\n", machine().describe_context(), m_rex3.m_x_start);
			ret |= (uint64_t)m_rex3.m_x_start << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 YStart Read: %08x\n", m_rex3.m_y_start);
			ret |= m_rex3.m_y_start;
		}
		break;
	case 0x0108/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XEnd Read: %08x\n", m_rex3.m_x_end);
			ret |= (uint64_t)m_rex3.m_x_end << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 YEnd Read: %08x\n", m_rex3.m_y_end);
			ret |= m_rex3.m_y_end;
		}
		break;
	case 0x0110/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XSave Read: %08x\n", m_rex3.m_x_save);
			ret |= (uint64_t)(uint16_t)m_rex3.m_x_save << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 XYMove Read: %08x\n", m_rex3.m_xy_move);
			ret |= m_rex3.m_xy_move;
		}
		break;
	case 0x0118/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham D Read: %08x\n", m_rex3.m_bres_d);
			ret |= (uint64_t)m_rex3.m_bres_d << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham S1 Read: %08x\n", m_rex3.m_bres_s1);
			ret |= m_rex3.m_bres_s1;
		}
		break;
	case 0x0120/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham Octant & Incr1 Read: %08x\n", m_rex3.m_bres_octant_inc1);
			ret |= (uint64_t)m_rex3.m_bres_octant_inc1 << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham Octant Rounding Mode & Incr2 Read: %08x\n", m_rex3.m_bres_round_inc2);
			ret |= m_rex3.m_bres_round_inc2;
		}
		break;
	case 0x0128/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham E1 Read: %08x\n", m_rex3.m_bres_e1);
			ret |= (uint64_t)m_rex3.m_bres_e1 << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham S2 Read: %08x\n", m_rex3.m_bres_s2);
			ret |= m_rex3.m_bres_s2;
		}
		break;
	case 0x0130/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 AA Line Weight Table 1/2 Read: %08x\n", m_rex3.m_a_weight0);
			ret |= (uint64_t)m_rex3.m_a_weight0 << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 AA Line Weight Table 2/2 Read: %08x\n", m_rex3.m_a_weight1);
			ret |= m_rex3.m_a_weight1;
		}
		break;
	case 0x0138/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 GL XStart Read: %08x\n", m_rex3.m_x_start_f);
			ret |= (uint64_t)m_rex3.m_x_start_f << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 GL YStart Read: %08x\n", m_rex3.m_y_start_f);
			ret |= m_rex3.m_y_start_f;
		}
		break;
	case 0x0140/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 GL XEnd Read: %08x\n", m_rex3.m_x_end_f);
			ret |= (uint64_t)m_rex3.m_x_end_f << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 GL YEnd Read: %08x\n", m_rex3.m_y_end_f);
			ret |= m_rex3.m_y_end_f;
		}
		break;
	case 0x0148/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XStart (integer) Read: %08x\n", m_rex3.m_x_start_i);
			ret |= (uint64_t)m_rex3.m_x_start_i << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 GL XEnd (copy) Read: %08x\n", m_rex3.m_x_end_f);
			ret |= m_rex3.m_x_end_f;
		}
		break;
	case 0x0150/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XYStart (integer) Read: %08x\n", m_rex3.m_xy_start_i);
			ret |= (uint64_t)m_rex3.m_xy_start_i << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 XYEnd (integer) Read: %08x\n", m_rex3.m_xy_end_i);
			ret |= m_rex3.m_xy_end_i;
		}
		break;
	case 0x0158/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XStartEnd (integer) Read: %08x\n", m_rex3.m_x_start_end_i);
			ret |= (uint64_t)m_rex3.m_x_start_end_i << 32;
		}
		break;
	case 0x0200/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Red/CI Full State Read: %08x\n", m_rex3.m_color_red);
			ret |= (uint64_t)m_rex3.m_color_red << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Alpha Full State Read: %08x\n", m_rex3.m_color_alpha);
			ret |= m_rex3.m_color_alpha;
		}
		break;
	case 0x0208/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Green Full State Read: %08x\n", m_rex3.m_color_green);
			ret |= (uint64_t)m_rex3.m_color_green << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Blue Full State Read: %08x\n", m_rex3.m_color_blue);
			ret |= m_rex3.m_color_blue;
		}
		break;
	case 0x0210/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Red/CI Slope Read: %08x\n", m_rex3.m_slope_red);
			ret |= (uint64_t)m_rex3.m_slope_red << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Alpha Slope Read: %08x\n", m_rex3.m_slope_alpha);
			ret |= m_rex3.m_slope_alpha;
		}
		break;
	case 0x0218/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Green Slope Read: %08x\n", m_rex3.m_slope_green);
			ret |= (uint64_t)m_rex3.m_slope_green << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Blue Slope Read: %08x\n", m_rex3.m_slope_blue);
			ret |= m_rex3.m_slope_blue;
		}
		break;
	case 0x0220/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Write Mask Read: %08x\n", m_rex3.m_write_mask);
			ret |= (uint64_t)m_rex3.m_write_mask << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Packed Color Fractions Read: %08x\n", m_rex3.m_color_i);
			ret |= m_rex3.m_color_i;
		}
		break;
	case 0x0228/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Color Index Zeros Overflow Read: %08x\n", m_rex3.m_zero_overflow);
			ret |= (uint64_t)m_rex3.m_zero_overflow << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Red/CI Slope (copy) Read: %08x\n", m_rex3.m_slope_red);
			ret |= m_rex3.m_slope_red;
		}
		break;
	case 0x0230/8:
		LOGMASKED(LOG_REX3, "%s: REX3 Host Data Port Read: %08x%08x\n", machine().describe_context(), (uint32_t)(m_rex3.m_host_dataport >> 32),
			(uint32_t)m_rex3.m_host_dataport);
		ret = m_rex3.m_host_dataport;
#ifdef MAME_SGI_REX3_HOSTDATA_DIAG
		if (FILE *f = rex3_hostdata_diag_file(machine()))
			fprintf(f, "Q t=%.6f ctx=%s off=%x ret=%016llx mask=%016llx go=%d m0=%08x m1=%08x pf=%d x=%d y=%d\n",
					machine().time().as_double(), machine().describe_context().c_str(), offset, (unsigned long long)ret,
					(unsigned long long)mem_mask, (offset & 0x100) ? 1 : 0, m_rex3.m_draw_mode0, m_rex3.m_draw_mode1,
					BIT(m_rex3.m_draw_mode1, 26), m_rex3.m_x_start_i, m_rex3.m_y_start_i);
#endif
		break;
	case 0x0238/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Display Control Bus Mode Read: %08x\n", m_rex3.m_dcb_mode);
			ret |= (uint64_t)m_rex3.m_dcb_mode << 32;
		}
		break;
	case 0x0240/8:
		if (ACCESSING_BITS_32_63)
		{
			switch (m_rex3.m_dcb_slave_select)
			{
			case DCB_ADDR_VC2:
				ret |= (uint64_t)m_vc2->read(m_rex3.m_dcb_reg_select) << 32;
				break;
			case DCB_ADDR_CMAP0:
				ret |= (uint64_t)m_cmap[0]->read(m_rex3.m_dcb_reg_select) << 32;
				break;
			case DCB_ADDR_CMAP1:
				ret |= (uint64_t)m_cmap[1]->read(m_rex3.m_dcb_reg_select) << 32;
				break;
			case DCB_ADDR_XMAP0:
				ret |= (uint64_t)m_xmap[0]->read(m_rex3.m_dcb_reg_select) << 32;
				break;
			case DCB_ADDR_XMAP1:
				ret |= (uint64_t)m_xmap[1]->read(m_rex3.m_dcb_reg_select) << 32;
				break;
			case DCB_ADDR_RAMDAC:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Read from RAMDAC (not yet implemented)\n");
				break;
			case DCB_ADDR_CC1:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Read from CC1 (not yet implemented)\n");
				break;
			case DCB_ADDR_AB1:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Read from AB1 (not yet implemented)\n");
				break;
			case DCB_ADDR_PCD:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Read from PCD (not yet implemented)\n");
				// Presenter not connected; simulate a bus timeout
				m_rex3.m_status |= STATUS_BACKBUSY;
				m_dcb_timeout_timer->adjust(attotime::from_msec(1));
				break;
			default:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Read: %08x\n", m_rex3.m_dcb_data_msw);
				ret |= (uint64_t)m_rex3.m_dcb_data_msw << 32;
				break;
			}
			if (BIT(m_rex3.m_dcb_mode, 3))
			{
				m_rex3.m_dcb_reg_select++;
				m_rex3.m_dcb_reg_select &= 7;
			}
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data LSW Read: %08x\n", m_rex3.m_dcb_data_lsw);
			ret |= m_rex3.m_dcb_data_lsw;
		}
		break;
	case 0x1300/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 1 X Min/Max Read: %08x\n", m_rex3.m_smask_x[1]);
			ret |= (uint64_t)m_rex3.m_smask_x[1] << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 1 Y Min/Max Read: %08x\n", m_rex3.m_smask_y[1]);
			ret |= m_rex3.m_smask_y[1];
		}
		break;
	case 0x1308/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 2 X Min/Max Read: %08x\n", m_rex3.m_smask_x[2]);
			ret |= (uint64_t)m_rex3.m_smask_x[2] << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 2 Y Min/Max Read: %08x\n", m_rex3.m_smask_y[2]);
			ret |= m_rex3.m_smask_y[2];
		}
		break;
	case 0x1310/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 3 X Min/Max Read: %08x\n", m_rex3.m_smask_x[3]);
			ret |= (uint64_t)m_rex3.m_smask_x[3] << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 3 Y Min/Max Read: %08x\n", m_rex3.m_smask_y[3]);
			ret |= m_rex3.m_smask_y[3];
		}
		break;
	case 0x1318/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 4 X Min/Max Read: %08x\n", m_rex3.m_smask_x[4]);
			ret |= (uint64_t)m_rex3.m_smask_x[4] << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 4 Y Min/Max Read: %08x\n", m_rex3.m_smask_y[4]);
			ret |= m_rex3.m_smask_y[4];
		}
		break;
	case 0x1320/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Top of Screen Scanline Read: %08x\n", m_rex3.m_top_scanline);
			ret |= (uint64_t)m_rex3.m_top_scanline << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 XY Window Read: %08x\n", m_rex3.m_xy_window);
			ret |= m_rex3.m_xy_window;
		}
		break;
	case 0x1328/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Clipping Mode Read: %08x\n", m_rex3.m_clip_mode);
			ret |= (uint64_t)m_rex3.m_clip_mode << 32;
		}
		break;
	case 0x1330/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Config Read: %08x\n", m_rex3.m_config);
			ret |= (uint64_t)m_rex3.m_config << 32;
		}
		break;
	case 0x1338/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Status Read: %08x\n", m_rex3.m_status);
			uint32_t old_status = m_rex3.m_status;
			m_rex3.m_status &= ~STATUS_VRINT;
			m_gio64->interrupt<2>(CLEAR_LINE);
			ret |= (uint64_t)(old_status | 3) << 32;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 User Status Read: %08x\n", m_rex3.m_status);
			ret |= m_rex3.m_status;
		}
		break;
	default:
		LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "Unknown REX3 Read: %08x (%08x%08x)\n", 0x1f0f0000 + (offset << 2), (uint32_t)(mem_mask >> 32), (uint32_t)mem_mask);
		return 0;
	}

	if ((offset & 0x00000100) && !go_before_read)
	{
		do_rex3_command();
	}

#if ENABLE_NEWVIEW_LOG
	if (m_newview_log != nullptr)
	{
		uint32_t offset_lo = (uint32_t)offset | 0x40000000;
		uint32_t data_hi = (uint32_t)(ret >> 32);
		uint32_t data_lo = (uint32_t)ret;
		uint32_t mem_mask_hi = (uint32_t)(mem_mask >> 32);
		uint32_t mem_mask_lo = (uint32_t)mem_mask;

		fwrite(&offset_lo, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&data_hi, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&data_lo, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&mem_mask_hi, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&mem_mask_lo, sizeof(uint32_t), 1, m_newview_log);
	}
#endif

	return ret;
}

// the next host pixel from HOSTRW, as the host wrote it (HOSTDEPTH wide)
uint32_t newport_base_device::next_host_pixel()
{
	static const uint32_t s_color_masks[4] = { 0xf, 0xff, 0xfff, 0xffffffff };
	// MAME_SGI_SWAPENDIAN_FIX: SWAPENDIAN swaps the bytes of each short (12-bit)
	// or long (32-bit) host pixel, before the depth conversion
	uint32_t color = (uint32_t)(m_rex3.m_host_dataport >> m_rex3.m_host_shift);
	if (BIT(m_rex3.m_draw_mode1, 11))
		color = swap_host_pixel(color, m_rex3.m_hostdepth);
	color &= s_color_masks[m_rex3.m_hostdepth];
	if (m_rex3.m_rwpacked)
	{
		if ((m_rex3.m_rwdouble && m_rex3.m_host_shift > 0) || m_rex3.m_host_shift > 32)
			m_rex3.m_host_shift -= s_host_shifts[m_rex3.m_hostdepth];
		else
			m_rex3.m_host_shift = 64 - s_host_shifts[m_rex3.m_hostdepth];
	}
	return color;
}

// MAME_SGI_ALPHAHOST_FIX: the pixel colour for a draw: from HOSTRW with
// COLORHOST, else from the DDA (or the flat colour) with its alpha from
// HOSTRW when ALPHAHOST is set: the alpha byte of the 32-bit host pixel,
// or with a narrower HOSTDEPTH the alpha byte of HOSTRW0 (IRIS GL's
// z-buffered antialiased points leave HOSTDEPTH at 4 bits). IRIS GL sends
// antialiased point coverage this way, as alpha in RGB mode and in the
// index's low bits (COLORHOST) in CI mode.
uint32_t newport_base_device::draw_color(int16_t x, int16_t y, uint32_t flat, bool dda)
{
	if (m_rex3.m_color_host)
		return get_host_color();
	const uint32_t c = dda ? get_rgb_color(x, y) : flat;
	if (!m_rex3.m_alpha_host)
		return c;
	const uint32_t alpha = m_rex3.m_hostdepth == 3 ? next_host_pixel() >> 24 : uint32_t(m_rex3.m_host_dataport >> 56);
	return (c & 0x00ffffff) | (alpha << 24);
}

// MAME_SGI_CI_HOSTDEPTH_FIX: host data are converted between depths as BGR
// colours only in RGB mode; a colour index is passed through (right-aligned
// in its host field), so OpenGL's 16-bit index writes and reads (HOSTDEPTH
// 12) of 8-bit colour-index planes keep their values
uint32_t newport_base_device::get_host_color()
{
	uint32_t color = next_host_pixel();
	if (!BIT(m_rex3.m_draw_mode1, 15))
		return color & ci_max();
	uint8_t convert_index = (m_rex3.m_hostdepth << 2) | m_rex3.m_plane_depth;
	switch (convert_index & 15)
	{
	default:
		// No conversion needed
		break;
	case 1:     // 4bpp -> 8bpp
		color = convert_4bpp_bgr_to_8bpp((uint8_t)color);
		break;
	case 2:     // 4bpp -> 12bpp
		color = convert_4bpp_bgr_to_12bpp((uint8_t)color);
		break;
	case 3:     // 4bpp -> 24bpp
		color = convert_4bpp_bgr_to_24bpp((uint8_t)color);
		break;
	case 4:     // 8bpp -> 4bpp
		color = convert_8bpp_bgr_to_4bpp((uint8_t)color);
		break;
	case 6:     // 8bpp -> 12bpp
		color = convert_8bpp_bgr_to_12bpp((uint8_t)color);
		break;
	case 7:     // 8bpp -> 24bpp
		color = convert_8bpp_bgr_to_24bpp((uint8_t)color);
		break;
	case 8:     // 12bpp -> 4bpp
		color = convert_12bpp_bgr_to_4bpp((uint16_t)color);
		break;
	case 9:     // 12bpp -> 8bpp
		color = convert_12bpp_bgr_to_8bpp((uint16_t)color);
		break;
	case 11:    // 12bpp -> 24bpp
		color = convert_12bpp_bgr_to_24bpp((uint16_t)color);
		break;
	case 12:    // 32bpp -> 4bpp
		color = convert_24bpp_bgr_to_4bpp(color);
		break;
	case 13:    // 32bpp -> 8bpp
		color = convert_24bpp_bgr_to_8bpp(color);
		break;
	case 14:    // 32bpp -> 12bpp
		color = convert_24bpp_bgr_to_12bpp(color);
		break;
	}
	return color;
}

uint32_t newport_base_device::swap_host_pixel(uint32_t v, uint8_t hostdepth)
{
	if (hostdepth == 3)
		return swapendian_int32(v);
	if (hostdepth == 2)
		return ((v & 0xff) << 8) | ((v >> 8) & 0xff);
	return v;
}

void newport_base_device::output_pixel(uint32_t color)
{
	const bool shade = BIT(m_rex3.m_draw_mode0, 18);
	const bool rgbmode = BIT(m_rex3.m_draw_mode1, 15);
	output_pixel(m_rex3.m_x_start_i, m_rex3.m_y_start_i,
			draw_color(m_rex3.m_x_start_i, m_rex3.m_y_start_i, color, (shade || rgbmode) && !m_rex3.m_color_host));
}

#ifdef MAME_SGI_REX3_FAST_PIXEL
ATTR_FORCE_INLINE inline
#endif
bool newport_base_device::pixel_clip_pass(int16_t x, int16_t y)
{
	bool mask0_pass = true;
	if (BIT(m_rex3.m_clip_mode, 0))
	{
		const int16_t min_x = (int16_t)(m_rex3.m_smask_x[0] >> 16);
		const int16_t min_y = (int16_t)(m_rex3.m_smask_y[0] >> 16);
		const int16_t max_x = (int16_t)m_rex3.m_smask_x[0];
		const int16_t max_y = (int16_t)m_rex3.m_smask_y[0];

		if (x < min_x)
			mask0_pass = false;
		else if (y < min_y)
			mask0_pass = false;
		else if (x > max_x)
			mask0_pass = false;
		else if (y > max_y)
			mask0_pass = false;
	}

	if (!mask0_pass)
	{
		LOGMASKED(LOG_REJECTS, "Rejecting pixel at %d,%d due to Mask 0 clipping (%d,%d - %d,%d)\n", x, y,
			(int16_t)(m_rex3.m_smask_x[0] >> 16), (int16_t)(m_rex3.m_smask_y[0] >> 16), (int16_t)m_rex3.m_smask_x[0], (int16_t)m_rex3.m_smask_y[0]);
		return false;
	}

	x += m_rex3.m_x_window;
	y += m_rex3.m_y_window;
	x -= 0x1000;
	y -= 0x1000;

	if (m_rex3.m_clip_mode & 0x1e)
	{
		uint8_t bit = 1;
		for (; bit < 5; bit++)
		{
			if (!BIT(m_rex3.m_clip_mode, bit))
				continue;

			int16_t min_x = (int16_t)(m_rex3.m_smask_x[bit] >> 16) - 0x1000;
			int16_t min_y = (int16_t)(m_rex3.m_smask_y[bit] >> 16) - 0x1000;
			int16_t max_x = (int16_t)m_rex3.m_smask_x[bit] - 0x1000;
			int16_t max_y = (int16_t)m_rex3.m_smask_y[bit] - 0x1000;

			if (x < min_x)
			{
				LOGMASKED(LOG_REJECTS, "Skipping Mask %d because %04x,%04x is outside %04x,%04x to %04x,%04x (MinX)\n", bit, x, y, min_x, min_y, max_x, max_y);
				continue;
			}
			if (x > max_x)
			{
				LOGMASKED(LOG_REJECTS, "Skipping Mask %d because %04x,%04x is outside %04x,%04x to %04x,%04x (MaxX)\n", bit, x, y, min_x, min_y, max_x, max_y);
				continue;
			}
			if (y < min_y)
			{
				LOGMASKED(LOG_REJECTS, "Skipping Mask %d because %04x,%04x is outside %04x,%04x to %04x,%04x (MinY)\n", bit, x, y, min_x, min_y, max_x, max_y);
				continue;
			}
			if (y > max_y)
			{
				LOGMASKED(LOG_REJECTS, "Skipping Mask %d because %04x,%04x is outside %04x,%04x to %04x,%04x (MaxY)\n", bit, x, y, min_x, min_y, max_x, max_y);
				continue;
			}
			break;
		}
		if (bit == 5)
		{
			LOGMASKED(LOG_REJECTS, "Rejecting pixel at %d,%d due to Mask 1-4 clipping\n", x, y);
			return false;
		}
	}

	if (x < 0 || y < 0 || x >= (1280+64) || y >= 1024)
	{
		LOGMASKED(LOG_REJECTS, "Rejecting pixel at %d,%d due to VRAM clipping\n", x, y);
		return false;
	}

	return true;
}

#ifdef MAME_SGI_REX3_FAST_PIXEL
ATTR_FORCE_INLINE inline
#endif
void newport_base_device::output_pixel(int16_t x, int16_t y, uint32_t color)
{
	if (m_pixel_write_suppress || !pixel_clip_pass(x, y))
	{
		return;
	}

	// MAME_SGI_ALPHA_TEST_FIX: DRAWMODE1 bits 12-14 are the alpha function:
	// the pixel is written if the AFUNCTION reference is greater than
	// (bit 12), equal to (bit 13) or less than (bit 14) its alpha. All three
	// set (the usual value) always passes.
	const uint32_t afunc = (m_rex3.m_draw_mode1 >> 12) & 7;
	if (afunc != 7 && BIT(m_rex3.m_draw_mode1, 15))
	{
		const uint8_t alpha = (uint8_t)(color >> 24);
		const uint8_t ref = m_rex3.m_alpha_ref;
		if (!((BIT(afunc, 0) && ref > alpha) || (BIT(afunc, 1) && ref == alpha) || (BIT(afunc, 2) && ref < alpha)))
			return;
	}

	x += m_rex3.m_x_window;
	y += m_rex3.m_y_window;
	x -= 0x1000;
	y -= 0x1000;

	// MAME_SGI_CI_COMPARE_FIX: in CI mode the same DRAWMODE1 bits are the
	// colour compare of antialiased lines (A_LINE): the pixel is written if
	// its index is less than (bit 12), equal to (bit 13) or greater than
	// (bit 14) the framebuffer's. IRIS GL loads them from zfunction under
	// zsource(ZSRC_COLOR), so antialiased CI lines keep the brighter
	// coverage where they meet.
	const bool ci_compare = afunc != 7 && !BIT(m_rex3.m_draw_mode1, 15) && ((m_rex3.m_draw_mode0 >> 2) & 7) == 4;
#ifdef MAME_SGI_REX3_FAST_PIXEL
	if (!BIT(m_rex3.m_draw_mode1, 18) && !ci_compare)
	{
		m_rb2->write_pixel_xy(x, y, color);
		return;
	}
#endif
#ifdef MAME_SGI_RB2_XY_ADDRESS
	set_address_xy(x, y);
#else
	const uint32_t address = (uint32_t)(y * (1280 + 64) + x);
	set_address(address);
#endif
	if (ci_compare)
	{
		const uint32_t mask = m_rex3.m_plane_depth == 0 ? 0xf : m_rex3.m_plane_depth == 1 ? 0xff : 0xfff;
		const uint32_t src = color & mask, dst = read_pixel() & mask;
		if (!((BIT(afunc, 0) && src < dst) || (BIT(afunc, 1) && src == dst) || (BIT(afunc, 2) && src > dst)))
			return;
	}

	if (BIT(m_rex3.m_draw_mode1, 18))
		blend_pixel(color);
	else
		write_pixel(color);
}

void newport_base_device::blend_pixel(uint32_t src)
{
	const uint32_t dst = read_pixel();

	float sa = ((src >> 24) & 0xff) / 255.0f;
	float sb = 0.0f;
	float sg = 0.0f;
	float sr = 0.0f;

	float db = 0.0f;
	float dg = 0.0f;
	float dr = 0.0f;

	float sbb = 0.0f;
	float sgb = 0.0f;
	float srb = 0.0f;

	float dbb = 0.0f;
	float dgb = 0.0f;
	float drb = 0.0f;

	switch (m_rex3.m_plane_depth)
	{
	case 0: // 4bpp
		// MAME_SGI_4BPP_DB_FIX: 4bpp RGB is B1 G2 R1 (bit 3, bits 2-1, bit 0),
		// as get_rgb_color() writes it
		sb = (0xff * BIT(src, 3)) / 255.0f;
		sg = (0x55 * ((src >> 1) & 3)) / 255.0f;
		sr = (0xff * BIT(src, 0)) / 255.0f;

		if (m_rex3.m_blend_back)
		{
			db = (uint8_t)(m_rex3.m_color_back >> 16) / 255.0f;
			dg = (uint8_t)(m_rex3.m_color_back >>  8) / 255.0f;
			dr = (uint8_t)(m_rex3.m_color_back >>  0) / 255.0f;
		}
		else
		{
			db = (0xff * BIT(dst, 3)) / 255.0f;
			dg = (0x55 * ((dst >> 1) & 3)) / 255.0f;
			dr = (0xff * BIT(dst, 0)) / 255.0f;
		}
		break;
	case 1: // 8bpp
		// MAME_SGI_BLEND_8BPP_FIX: 8bpp RGB is B2 G3 R3 (bits 7-6, 5-3, 2-0),
		// for the source and the destination alike, normalized like the
		// other depths
		sb = ((0xaa * BIT(src, 7)) | (0x55 * BIT(src, 6))) / 255.0f;
		sg = ((0x92 * BIT(src, 5)) | (0x49 * BIT(src, 4)) | (0x24 * BIT(src, 3))) / 255.0f;
		sr = ((0x92 * BIT(src, 2)) | (0x49 * BIT(src, 1)) | (0x24 * BIT(src, 0))) / 255.0f;

		if (m_rex3.m_blend_back)
		{
			db = (uint8_t)(m_rex3.m_color_back >> 16) / 255.0f;
			dg = (uint8_t)(m_rex3.m_color_back >>  8) / 255.0f;
			dr = (uint8_t)(m_rex3.m_color_back >>  0) / 255.0f;
		}
		else
		{
			db = ((0xaa * BIT(dst, 7)) | (0x55 * BIT(dst, 6))) / 255.0f;
			dg = ((0x92 * BIT(dst, 5)) | (0x49 * BIT(dst, 4)) | (0x24 * BIT(dst, 3))) / 255.0f;
			dr = ((0x92 * BIT(dst, 2)) | (0x49 * BIT(dst, 1)) | (0x24 * BIT(dst, 0))) / 255.0f;
		}
		break;
	case 2: // 12bpp
		sb = (((src >> 8) & 15) * 0x11) / 255.0f;
		sg = (((src >> 4) & 15) * 0x11) / 255.0f;
		sr = (((src >> 0) & 15) * 0x11) / 255.0f;

		if (m_rex3.m_blend_back)
		{
			db = (uint8_t)(m_rex3.m_color_back >> 16) / 255.0f;
			dg = (uint8_t)(m_rex3.m_color_back >>  8) / 255.0f;
			dr = (uint8_t)(m_rex3.m_color_back >>  0) / 255.0f;
		}
		else
		{
			const uint32_t dstc = dst & 0xfff;
			db = (((dstc >> 8) & 15) * 0x11) / 255.0f;
			dg = (((dstc >> 4) & 15) * 0x11) / 255.0f;
			dr = (((dstc >> 0) & 15) * 0x11) / 255.0f;
		}
		break;
	case 3: // 24bpp
		sa = (uint8_t)(src >> 24) / 255.0f;
		sb = (uint8_t)(src >> 16) / 255.0f;
		sg = (uint8_t)(src >>  8) / 255.0f;
		sr = (uint8_t)(src >>  0) / 255.0f;

		if (m_rex3.m_blend_back)
		{
			db = (uint8_t)(m_rex3.m_color_back >> 16) / 255.0f;
			dg = (uint8_t)(m_rex3.m_color_back >>  8) / 255.0f;
			dr = (uint8_t)(m_rex3.m_color_back >>  0) / 255.0f;
		}
		else
		{
			const uint32_t dstc = dst;
			db = (uint8_t)(dstc >> 16) / 255.0f;
			dg = (uint8_t)(dstc >>  8) / 255.0f;
			dr = (uint8_t)(dstc >>  0) / 255.0f;
		}
		break;
	}

	switch (m_rex3.m_sfactor)
	{
	case 0: // 0
	default:
		break;
	case 1: // 1
		sbb = sb;
		sgb = sg;
		srb = sr;
		break;
	case 2: // dstc
		sbb = sb * db;
		sgb = sg * dg;
		srb = sr * dr;
		break;
	case 3: // 1 - dstc
		sbb = sb * (1.0f - db);
		sgb = sg * (1.0f - dg);
		srb = sr * (1.0f - dr);
		break;
	// MAME_SGI_BLEND_SFACTOR_FIX: source alpha applies whatever DRAWMODE1
	// bit 27 says, as for DFACTOR: IRIX's OpenGL leaves bit 27 clear for
	// GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA
	case 4: // srca
		sbb = sb * sa;
		sgb = sg * sa;
		srb = sr * sa;
		break;
	case 5: // 1 - srca
		sbb = sb * (1.0f - sa);
		sgb = sg * (1.0f - sa);
		srb = sr * (1.0f - sa);
		break;
	}

	switch (m_rex3.m_dfactor)
	{
	case 0: // 0
	default:
		break;
	case 1: // 1
		dbb = db;
		dgb = dg;
		drb = dr;
		break;
	case 2: // srcc
		dbb = db * sb;
		dgb = dg * sg;
		drb = dr * sr;
		break;
	case 3: // 1 - srcc
		dbb = db * (1.0f - sb);
		dgb = dg * (1.0f - sg);
		drb = dr * (1.0f - sr);
		break;
	case 4: // srca
		dbb = db * sa;
		dgb = dg * sa;
		drb = dr * sa;
		break;
	case 5: // 1 - srca
		dbb = db * (1.0f - sa);
		dgb = dg * (1.0f - sa);
		drb = dr * (1.0f - sa);
		break;
	}

	const float b_blend = sbb + dbb;
	const float g_blend = sgb + dgb;
	const float r_blend = srb + drb;

	const uint8_t b_blend_i = b_blend > 1.0f ? 255 : (b_blend < 0.0f ? 0 : (uint8_t)(b_blend * 255.0f));
	const uint8_t g_blend_i = g_blend > 1.0f ? 255 : (g_blend < 0.0f ? 0 : (uint8_t)(g_blend * 255.0f));
	const uint8_t r_blend_i = r_blend > 1.0f ? 255 : (r_blend < 0.0f ? 0 : (uint8_t)(r_blend * 255.0f));

	switch (m_rex3.m_plane_depth)
	{
	case 0: // 4bpp (MAME_SGI_4BPP_DB_FIX)
		write_pixel(((b_blend_i & 0x80) >> 4) | ((g_blend_i & 0xc0) >> 5) | ((r_blend_i & 0x80) >> 7));
		break;
	case 1: // 8bpp
		write_pixel((b_blend_i & 0xc0) | ((g_blend_i & 0xe0) >> 2) | ((r_blend_i & 0xe0) >> 5));
		break;
	case 2: // 12bpp
		write_pixel(((b_blend_i & 0xf0) << 4) | (g_blend_i & 0xf0) | ((r_blend_i & 0xf0) >> 4));
		break;
	case 3: // 24bpp
		write_pixel((b_blend_i << 16) | (g_blend_i << 8) | r_blend_i);
		break;
	}
}

// MAME_SGI_CI_DITHER_FIX: colour-index rounding and dithering. The 4-bit
// fraction below the index (CI iterator, 12.9 at 12bpp, 12.11 otherwise)
// adds 1 if it is above the dither matrix entry with DITHER on, or if its
// msb is set with DITHER off. The fraction is clamped first: the largest
// index never overflows.
static const uint8_t s_dither_matrix[4][4] = { { 0, 12, 3, 15 },{ 8, 4, 11, 7 },{ 2, 14, 1, 13 },{ 10, 6, 9, 5 } };

uint32_t newport_base_device::ci_max() const
{
	return m_rex3.m_plane_depth == 0 ? 0xf : m_rex3.m_plane_depth == 1 ? 0xff : 0xfff;
}

uint32_t newport_base_device::ci_round_dither(uint32_t iter, int16_t x, int16_t y) const
{
	const int fbits = m_rex3.m_plane_depth == 2 ? 9 : 11;
	uint32_t ci = iter >> fbits;
	const uint32_t frac = (iter >> (fbits - 4)) & 0xf;
	if ((ci & ci_max()) == ci_max())
		return ci;
	if (BIT(m_rex3.m_draw_mode1, 16) ? frac > s_dither_matrix[x & 3][y & 3] : BIT(frac, 3))
		ci++;
	return ci;
}

// the same for antialiased CI (A_LINE): the coverage replaces the index's
// 4 lsbs, which serve as the fraction instead, and the increment is 0x10;
// 4-bit antialiased pixels are neither rounded nor dithered
uint32_t newport_base_device::ci_aa_round_dither(uint32_t ci, uint32_t weight, int16_t x, int16_t y) const
{
	const uint32_t frac = ci & 0xf;
	uint32_t d = (ci & ~0xfu) | weight;
	if (m_rex3.m_plane_depth == 0 || ((ci | 0xf) & ci_max()) == ci_max())
		return d;
	if (BIT(m_rex3.m_draw_mode1, 16) ? frac > s_dither_matrix[x & 3][y & 3] : BIT(frac, 3))
		d += 0x10;
	return d;
}

uint32_t newport_base_device::get_rgb_color(int16_t x, int16_t y)
{
	static const uint8_t s_bayer[4][4] = { { 0, 12, 3, 15 },{ 8, 4, 11, 7 },{ 2, 14, 1, 13 },{ 10, 6, 9, 5 } };

	uint32_t red = ((m_rex3.m_curr_color_red >> 11) & 0x1ff);
	uint32_t green = ((m_rex3.m_curr_color_green >> 11) & 0x1ff);
	uint32_t blue = ((m_rex3.m_curr_color_blue >> 11) & 0x1ff);
	uint32_t alpha = ((m_rex3.m_curr_color_alpha >> 11) & 0x1ff);

	if (red >= 0x180 || BIT(m_rex3.m_curr_color_red, 31))
	{
		red = 0;
	}
	else if (red > 0xff)
	{
		red = 0xff;
	}

	if (green >= 0x180 || BIT(m_rex3.m_curr_color_green, 31))
	{
		green = 0;
	}
	else if (green > 0xff)
	{
		green = 0xff;
	}

	if (blue >= 0x180 || BIT(m_rex3.m_curr_color_blue, 31))
	{
		blue = 0;
	}
	else if (blue > 0xff)
	{
		blue = 0xff;
	}

	if (alpha >= 0x180 || BIT(m_rex3.m_curr_color_alpha, 31))
	{
		alpha = 0;
	}
	else if (alpha > 0xff)
	{
		alpha = 0xff;
	}
	alpha <<= 24;

	if (!BIT(m_rex3.m_draw_mode1, 15)) // RGB
	{
		// MAME_SGI_CI_SHADE_FIX: colour index: the integer part of the
		// red/CI iterator, a 12.9 fixed-point value at 12bpp and 12.11
		// otherwise (as the Red/CI register write decodes it)
		return get_default_color(ci_round_dither(m_rex3.m_curr_color_red, x, y));
	}

	if (BIT(m_rex3.m_draw_mode1, 16)) // Dithering
	{
		switch (m_rex3.m_plane_depth)
		{
		case 0: // 4bpp
		{
			const uint8_t sr = (red >> 3) - (red >> 4);
			const uint8_t sg = (green >> 2) - (green >> 4);
			const uint8_t sb = (blue >> 3) - (blue >> 4);

			uint8_t dr = BIT(sr, 4);
			uint8_t dg = (sg >> 4) & 3;
			uint8_t db = BIT(sb, 4);

			if ((sr & 0xf) > s_bayer[x & 3][y & 3]) dr++;
			if ((sg & 0xf) > s_bayer[x & 3][y & 3]) dg++;
			if ((sb & 0xf) > s_bayer[x & 3][y & 3]) db++;

			if (dr > 1) dr = 1;
			if (dg > 3) dg = 3;
			if (db > 1) db = 1;

			uint32_t color = (db << 3) | (dg << 1) | dr;
			return alpha | (color << 4) | color;
		}

		case 1: // 8bpp
		{
			const uint8_t sr = (red >> 1) - (red >> 4);
			const uint8_t sg = (green >> 1) - (green >> 4);
			const uint8_t sb = (blue >> 2) - (blue >> 4);

			uint8_t dr = (sr >> 4) & 7;
			uint8_t dg = (sg >> 4) & 7;
			uint8_t db = (sb >> 4) & 3;

			if ((sr & 0xf) > s_bayer[x & 3][y & 3]) dr++;
			if ((sg & 0xf) > s_bayer[x & 3][y & 3]) dg++;
			if ((sb & 0xf) > s_bayer[x & 3][y & 3]) db++;

			if (dr > 7) dr = 7;
			if (dg > 7) dg = 7;
			if (db > 3) db = 3;

			return alpha | (db << 6) | (dg << 3) | dr;
		}

		case 2: // 12bpp
		{
			const uint32_t sr = red - (red >> 4);
			const uint32_t sg = green - (green >> 4);
			const uint32_t sb = blue - (blue >> 4);

			uint32_t dr = (sr >> 4) & 15;
			uint32_t dg = (sg >> 4) & 15;
			uint32_t db = (sb >> 4) & 15;

			if ((sr & 0xf) > s_bayer[x & 3][y & 3]) dr++;
			if ((sg & 0xf) > s_bayer[x & 3][y & 3]) dg++;
			if ((sb & 0xf) > s_bayer[x & 3][y & 3]) db++;

			if (dr > 15) dr = 15;
			if (dg > 15) dg = 15;
			if (db > 15) db = 15;

			uint32_t color = (db << 8) | (dg << 4) | dr;
			return alpha | (color << 12) | color;
		}

		case 3: // 24bpp
			return alpha | (blue << 16) | (green << 8) | red;

		default:
			return 0;
		}
	}
	else
	{
		// MAME_SGI_RGB_ROUND_FIX: with dither off, an n-bit channel (n < 8) is
		// rounded, not truncated: S = P[7:4-n] - P[7:4] on n+4 bits (P scaled
		// by (2^n - 1)/255), D = S[n+3:4] + S[3]; 1-bit channels take P[7]
		const auto round = [] (uint32_t p, int n) -> uint32_t
		{
			const uint32_t s = (p >> (4 - n)) - (p >> 4);
			return (s >> 4) + BIT(s, 3);
		};
		switch (m_rex3.m_plane_depth)
		{
		case 0: // 4bpp
			return alpha | (BIT(blue, 7) << 3) | (round(green, 2) << 1) | BIT(red, 7);
		case 1: // 8bpp
			return alpha | (round(blue, 2) << 6) | (round(green, 3) << 3) | round(red, 3);
		case 2: // 12bpp
			return alpha | (round(blue, 4) << 8) | (round(green, 4) << 4) | round(red, 4);
		case 3: // 24bpp
			return alpha | (blue << 16) | (green << 8) | red;
		default:
			return 0;
		}
	}
}

uint8_t newport_base_device::get_octant(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t dx, int32_t dy)
{
	uint8_t octant = 0;

	// Octant format: {MSB, 2nd, LSB} = {XMAJOR, XDEC, YDEC}
	// Mapping of hardware octant to peseudocode octant in manual
	// | Pseudocode | hardware |  hardware(dec) |
	// |     1      | 3'b101   |       5        |
	// |     2      | 3'b001   |       1        |
	// |     3      | 3'b011   |       3        |
	// |     4      | 3'b111   |       7        |
	// |     5      | 3'b110   |       6        |
	// |     6      | 3'b010   |       2        |
	// |     7      | 3'b000   |       0        |
	// |     8      | 3'b100   |       4        |

	/* YDEC */
	if (y1 > y2)
		octant |= (1 << 0); // Bit 0

	/* XDEC */
	if (x1 > x2)
		octant |= (1 << 1); // Bit 1

	/* XMAJOR */
	if (dx > dy)
		octant |= (1 << 2); // Bit 2

	return octant;
}

void newport_base_device::do_setup(void)
{
	const int32_t x1 = util::sext(m_rex3.m_x_start >> 7, 20);
	const int32_t y1 = util::sext(m_rex3.m_y_start >> 7, 20);
	const int32_t x2 = util::sext(m_rex3.m_x_end >> 7, 20);
	const int32_t y2 = util::sext(m_rex3.m_y_end >> 7, 20);
	const int32_t dx = abs(x1 - x2);
	const int32_t dy = abs(y1 - y2);

	/* octant for line and block, span */
	const uint8_t octant = get_octant(x1, y1, x2, y2, dx, dy);
	m_rex3.m_bres_octant_inc1 &= ~(0x7 << 24);
	m_rex3.m_bres_octant_inc1 |= (uint32_t)octant << 24;
	/* FIXME: error terms (incr1, incr2) and Bresenham terms (d) */
	LOGMASKED(LOG_REX3, "do_setup: octant = %d, x1 = %d, y1 = %d, x2 = %d, y2 = %d, dx = %d, dy = %d\n", octant, x1, y1, x2, y2, dx, dy);
}

// MAME_SGI_LINE_STIPPLE_FIX: a line pixel, through the line stipple when
// DRAWMODE0 enables it (bit 13): LSPATTERN's bit 31 says whether the pixel is
// drawn (else drawn in COLORBACK when the stipple is opaque, bit 17). Each
// bit is used LSREPEAT + 1 times (LSMODE bits 8-15; bits 0-7 count down),
// then the pattern rotates left within its LSLENGTH + 17 most significant
// bits (LSMODE bits 24-27).
// MAME_SGI_LSADVLAST_FIX: with DRAWMODE0 LSADVLAST (bit 14) clear, the line
// stipple does not advance at the end of a line, i.e. at its endpoint pixel
// (end_pixel): connected lines draw the shared vertex twice, once per line,
// and the stipple then runs on across it. IRIS GL draws its polylines so.
void newport_base_device::line_pixel(int16_t x, int16_t y, uint32_t color, bool rgb, bool end_pixel)
{
	const uint32_t mode0 = m_rex3.m_draw_mode0;
	if (BIT(mode0, 13))
	{
		const bool on = BIT(m_rex3.m_ls_pattern, 31);
		if (!end_pixel || BIT(mode0, 14))
		{
			uint32_t count = m_rex3.m_ls_mode & 0xff;
			if (count == 0)
			{
				const int len = ((m_rex3.m_ls_mode >> 24) & 0xf) + 17;
				const uint32_t p = m_rex3.m_ls_pattern;
				if (len >= 32)
					m_rex3.m_ls_pattern = (p << 1) | (p >> 31);
				else
				{
					const uint32_t keep = (1u << (32 - len)) - 1;
					const uint32_t low = 1u << (32 - len);	// the pattern's last bit
					m_rex3.m_ls_pattern = ((p << 1) & ~(keep | low)) | ((p >> 31) ? low : 0) | (p & keep);
				}
				count = (m_rex3.m_ls_mode >> 8) & 0xff;
			}
			else
			{
				count--;
			}
			m_rex3.m_ls_mode = (m_rex3.m_ls_mode & ~0xffu) | count;
		}
		if (!on)
		{
			if (BIT(mode0, 17))
				output_pixel(x, y, m_rex3.m_color_back);
			return;
		}
	}
	output_pixel(x, y, draw_color(x, y, color, rgb));
}

void newport_base_device::do_fline(uint32_t color)
{
	const int32_t x1 = util::sext(m_rex3.m_x_start >> 7, 20);
	const int32_t y1 = util::sext(m_rex3.m_y_start >> 7, 20);
	const int32_t x2 = util::sext(m_rex3.m_x_end >> 7, 20);
	const int32_t y2 = util::sext(m_rex3.m_y_end >> 7, 20);

	const int32_t x10 = x1 & ~0xf;
	const int32_t y10 = y1 & ~0xf;
	const int32_t x20 = x2 & ~0xf;
	const int32_t y20 = y2 & ~0xf;

	const bool iterate_one = (m_rex3.m_draw_mode0 & 0x300) == 0;
	const bool skip_first = BIT(m_rex3.m_draw_mode0, 10);
	const bool skip_last = BIT(m_rex3.m_draw_mode0, 11);
	const bool shade = BIT(m_rex3.m_draw_mode0, 18);
	const bool rgbmode = BIT(m_rex3.m_draw_mode1, 15);

	int32_t x = x10;
	int32_t y = y10;

	int16_t x1_fract = m_rex3.m_x_start_frac;
	// MAME_SGI_FLINE_FRACT_FIX: the octant cases below take the fractions in
	// the stepping's y-up sense, while Y here increases down (the steps
	// subtract incry), so the start's y fraction is measured from the other
	// side of its pixel
	int16_t y1_fract = 0x10 - m_rex3.m_y_start_frac;

	int32_t dx = abs(x1 - x2);
	int32_t dy = abs(y1 - y2);

	// MAME_SGI_FLINE_PIXEL_STEP_FIX: positions are in 1/16 pixel; the
	// rasterizer steps and counts whole pixels (0x10), so a LENGTH32 line
	// command draws 32 pixels and leaves the start at the 33rd
	constexpr int32_t PIXEL = 0x10;
	const int32_t dx_i = abs(x10 - x20) / PIXEL;
	const int32_t dy_i = abs(y10 - y20) / PIXEL;

	static const bresenham_octant_info_t s_bresenham_infos[8] =
	{
		{  0,  1, -1, -1,  1 }, // Pseudo 7
		{  0,  1,  1,  1,  1 }, // Pseudo 2
		{  0, -1, -1, -1,  1 }, // Pseudo 6
		{  0, -1,  1,  1,  1 }, // Pseudo 3
		{  1,  1,  0, -1,  0 }, // Pseudo 8
		{  1,  1,  0,  1,  0 }, // Pseudo 1
		{ -1, -1,  0, -1,  0 }, // Pseudo 5
		{ -1, -1,  0,  1,  0 }  // Pseudo 4
	};

	const uint8_t octant = (m_rex3.m_bres_octant_inc1 >> 24) & 0x7;
	const int32_t incrx1 = s_bresenham_infos[octant].incrx1;
	const int32_t incrx2 = s_bresenham_infos[octant].incrx2;
	const int32_t incry1 = s_bresenham_infos[octant].incry1;
	const int32_t incry2 = s_bresenham_infos[octant].incry2;
	// MAME_SGI_FLINE_CONTINUE_FIX: a command that starts exactly where the
	// previous LENGTH32 command stopped, towards the same end, continues
	// that line (same error term) instead of setting it up again from an
	// integer start, which loses the start's fraction and bends the line
	const bool resume = m_fline_cont.valid && m_fline_cont.x == x1 && m_fline_cont.y == y1
			&& m_fline_cont.x2 == x2 && m_fline_cont.y2 == y2 && m_fline_cont.octant == octant;
	m_fline_cont.valid = false;
	int32_t loop = s_bresenham_infos[octant].loop ? dy_i : dx_i;
	const bool truncated = BIT(m_rex3.m_draw_mode0, 15) && loop > 32;
	if (truncated)
		loop = 32;
	const int32_t x_major = 1 - s_bresenham_infos[octant].loop;

	int32_t incr1 = s_bresenham_infos[octant].loop ? (2 * dx) : (2 * dy);
	int32_t incr2 = s_bresenham_infos[octant].loop ? (2 * (dx - dy)) : (2 * (dy - dx));
	int32_t d = s_bresenham_infos[octant].loop ? (3 * dx - 2 * dy) : (3 * dy - 2 * dx);

	switch (octant)
	{
	case 5: // Pseudo 1
		// Nothing special needed
		break;
	case 1: // Pseudo 2
	{
		const int16_t temp_fract = x1_fract;
		x1_fract = y1_fract;
		y1_fract = temp_fract;
		const int32_t temp_d = dx;
		dx = dy;
		dy = temp_d;
		break;
	}
	case 3: // Pseudo 3
	{
		const int16_t temp_fract = 0x10 - x1_fract;
		x1_fract = y1_fract;
		y1_fract = temp_fract;
		const int32_t temp_d = dx;
		dx = dy;
		dy = temp_d;
		break;
	}
	case 7: // Pseudo 4
		x1_fract = 0x10 - x1_fract;
		break;
	case 6: // Pseudo 5
		x1_fract = 0x10 - x1_fract;
		y1_fract = 0x10 - y1_fract;
		break;
	case 2: // Pseudo 6
	{
		const int16_t temp_fract = 0x10 - x1_fract;
		x1_fract = 0x10 - y1_fract;
		y1_fract = temp_fract;
		const int32_t temp_d = dx;
		dx = dy;
		dy = temp_d;
		break;
	}
	case 0: // Pseudo 7
	{
		const int16_t temp_fract = 0x10 - y1_fract;
		y1_fract = x1_fract;
		x1_fract = temp_fract;
		const int32_t temp_d = dx;
		dx = dy;
		dy = temp_d;
		break;
	}
	case 4: // Pseudo 8
		y1_fract = 0x10 - y1_fract;
		break;
	}

	if (resume)
	{
		x = x1;
		y = y1;
		d = m_fline_cont.d;
		incr1 = m_fline_cont.incr1;
		incr2 = m_fline_cont.incr2;
	}
	else
	{
		d += 2 * (((dx * y1_fract) >> 4) - ((dy * x1_fract) >> 4)); // Adjust d due to fractional endpoints
		const int32_t E = d - 2 * dx; // Variable used for adjusting the start point up one pixel
		if (E > 0)
		{
			d = E;
			x += incrx2 * (1 - x_major) * PIXEL;
			y -= incry2 * x_major * PIXEL;	// MAME_SGI_FLINE_FRACT_FIX: as the steps
		}
	}

	if (!skip_first || iterate_one || resume)
	{
		const int16_t x16 = (int16_t)(x >> 4);
		const int16_t y16 = (int16_t)(y >> 4);
		line_pixel(x16, y16, color, shade || rgbmode);

		if (shade)
			iterate_shade();

		if (d < 0)
		{
			x += incrx1 * PIXEL;
			y -= incry1 * PIXEL;
			d += incr1;
		}
		else
		{
			x += incrx2 * PIXEL;
			y -= incry2 * PIXEL;
			d += incr2;
		}

		if (iterate_one)
		{
			write_x_start(x << 7);
			write_y_start(y << 7);
			return;
		}
	}

	for (int32_t i = 1; i < loop; i++)
	{
		const int16_t x16 = (int16_t)(x >> 4);
		const int16_t y16 = (int16_t)(y >> 4);
		line_pixel(x16, y16, color, shade || rgbmode);

		if (shade)
			iterate_shade();

		if (d < 0)
		{
			x += incrx1 * PIXEL;
			y -= incry1 * PIXEL;
			d += incr1;
		}
		else
		{
			x += incrx2 * PIXEL;
			y -= incry2 * PIXEL;
			d += incr2;
		}
	}

	if (truncated)
	{
		// more of the line to come: stop at the next pixel
		write_x_start(x << 7);
		write_y_start(y << 7);
		m_fline_cont.valid = true;
		m_fline_cont.x = x;
		m_fline_cont.y = y;
		m_fline_cont.x2 = x2;
		m_fline_cont.y2 = y2;
		m_fline_cont.d = d;
		m_fline_cont.incr1 = incr1;
		m_fline_cont.incr2 = incr2;
		m_fline_cont.octant = octant;
	}
	else if (!skip_last)
	{
		const int16_t x16 = (int16_t)(x2 >> 4);
		const int16_t y16 = (int16_t)(y2 >> 4);
		line_pixel(x16, y16, color, shade || rgbmode, true);

		if (shade)
			iterate_shade();

		write_x_start(x2);
		write_y_start(y2);
	}
	else
	{
		write_x_start(x << 7);
		write_y_start(y << 7);
	}
}

// MAME_SGI_REX3_ALINE_FIX: A_LINE, the antialiased line. Each step along
// the major axis draws two pixels across the line: S, the pixel whose
// centre is at or below the line on the minor axis, and T, the one above.
// Their coverages come from the 16-entry AWEIGHT table (AWEIGHT1 bits 3-0
// are entry 0, AWEIGHT0 bits 31-28 entry 15), indexed by the line's offset
// from S's centre in sixteenths: T takes that entry and S its complement.
// The coverage is the alpha in RGB mode (the alpha iterator is not used:
// OpenGL leaves another primitive's alpha slope in place) and replaces the
// low 4 bits of the colour index in CI mode. ZPATTERN masks S and LSPATTERN
// masks T, one bit per step, msb first. With ENDPTFILTER a step holding an
// endpoint has its coverages shifted right by 0 to 3 as the line covers 8 or
// more, 4-7, 2-3 or 0-1 sixteenths of its column; a skipped end is not
// filtered. A LENGTH32 line draws 32 steps per GO and continues there on the
// next.
void newport_base_device::do_aline()
{
	const uint32_t mode0 = m_rex3.m_draw_mode0;
	const int32_t x1 = util::sext(m_rex3.m_x_start >> 7, 20);
	const int32_t y1 = util::sext(m_rex3.m_y_start >> 7, 20);
	const int32_t x2 = util::sext(m_rex3.m_x_end >> 7, 20);
	const int32_t y2 = util::sext(m_rex3.m_y_end >> 7, 20);

	const aline_continuation cont = m_aline_cont;
	const bool resume = cont.valid && cont.xs == x1 && cont.ys == y1 && cont.x2 == x2 && cont.y2 == y2;
	m_aline_cont.valid = false;
	const int32_t ox1 = resume ? cont.x1 : x1;
	const int32_t oy1 = resume ? cont.y1 : y1;

	// major (a) and minor (b) coordinates in 1/16 pixel, negated where they
	// decrease so that both increase along the line (pixel k becomes -k-1)
	const bool x_major = abs(x2 - ox1) >= abs(y2 - oy1);
	const int32_t sa = (x_major ? x2 >= ox1 : y2 >= oy1) ? 1 : -1;
	const int32_t sb = (x_major ? y2 >= oy1 : x2 >= ox1) ? 1 : -1;
	const int64_t a1 = sa * (x_major ? ox1 : oy1), b1 = sb * (x_major ? oy1 : ox1);
	const int64_t a2 = sa * (x_major ? x2 : y2), b2 = sb * (x_major ? y2 : x2);
	const int64_t da = a2 - a1, db = b2 - b1;
	const int32_t col_first = int32_t(a1 >> 4), col_last = int32_t(a2 >> 4);

	// the line's minor coordinate at a column's centre
	auto minor_at = [&] (int32_t col) -> int64_t
	{
		if (!da)
			return b1;
		const int64_t n = (int64_t(col) * 16 + 8 - a1) * db;
		return b1 + (n >= 0 ? n / da : -((-n + da - 1) / da));
	};
	auto to_xy = [&] (int64_t col, int64_t row, int16_t &px, int16_t &py)
	{
		const int16_t mj = int16_t(sa > 0 ? col : -col - 1);
		const int16_t mn = int16_t(sb > 0 ? row : -row - 1);
		px = x_major ? mj : mn;
		py = x_major ? mn : mj;
	};

	int32_t col = resume ? cont.col : col_first;
	int32_t steps = col_last - col + 1;
	const int32_t limit = (mode0 & 0x300) == 0 ? 1 : BIT(mode0, 15) ? 32 : steps;
	const bool truncated = steps > limit;
	if (truncated)
		steps = limit;

	const uint64_t table = (uint64_t(m_rex3.m_a_weight0) << 32) | m_rex3.m_a_weight1;
	const bool skip_first = BIT(mode0, 10);
	const bool skip_last = BIT(mode0, 11);
	const bool zmask = BIT(mode0, 12);
	const bool lsmask = BIT(mode0, 13);
	const bool shade = BIT(mode0, 18);
	const bool filter = BIT(mode0, 22);
	const bool rgbmode = BIT(m_rex3.m_draw_mode1, 15);

	auto pixel = [&] (int64_t c, int64_t r, int32_t index, int shift)
	{
		const uint32_t weight = ((table >> (4 * index)) & 0xf) >> shift;
		int16_t px, py;
		to_xy(c, r, px, py);
		uint32_t color;
		if (rgbmode)
		{
			color = (get_rgb_color(px, py) & 0x00ffffff) | ((weight * 0x11) << 24);
		}
		else
		{
			const uint32_t ci = shade ? m_rex3.m_curr_color_red >> (m_rex3.m_plane_depth == 2 ? 9 : 11) : m_rex3.m_color_i;
			color = get_default_color(ci_aa_round_dither(ci, weight, px, py));
		}
		output_pixel(px, py, color);
	};

	for (int32_t i = 0; i < steps; i++, col++)
	{
		const bool first = col == col_first;
		const bool last = col == col_last;
		if ((first && skip_first) || (last && skip_last))
			continue;
		const int64_t b = minor_at(col) - 8;
		const int64_t row = b >> 4;
		const int32_t t_index = int32_t(b - row * 16);
		int shift = 0;
		if (filter && (first || last))
		{
			const int32_t covered = (first ? 15 - int32_t(a1 & 15) : 15) + (last ? int32_t(a2 & 15) : 15) + 1 - 16;
			shift = covered >= 8 ? 0 : covered >= 4 ? 1 : covered >= 2 ? 2 : 3;
		}
		const int bit = 31 - (i & 31);
		if (!zmask || BIT(m_rex3.m_z_pattern, bit))
			pixel(col, row, 15 - t_index, shift);
		if (!lsmask || BIT(m_rex3.m_ls_pattern, bit))
			pixel(col, row + 1, t_index, shift);
		if (shade)
			iterate_shade();
	}

	if (truncated)
	{
		// the next piece starts at column col
		int16_t px, py;
		to_xy(col, (minor_at(col) - 8) >> 4, px, py);
		const int32_t xs = px * 16, ys = py * 16;
		write_x_start(xs << 7);
		write_y_start(ys << 7);
		m_aline_cont.valid = true;
		m_aline_cont.x1 = ox1;
		m_aline_cont.y1 = oy1;
		m_aline_cont.x2 = x2;
		m_aline_cont.y2 = y2;
		m_aline_cont.xs = xs;
		m_aline_cont.ys = ys;
		m_aline_cont.col = col;
	}
	else
	{
		write_x_start(x2 << 7);
		write_y_start(y2 << 7);
	}
}

void newport_base_device::do_iline(uint32_t color)
{
	int32_t x1 = m_rex3.m_x_start_i;
	int32_t y1 = m_rex3.m_y_start_i;
	int32_t x2 = m_rex3.m_x_end_i;
	int32_t y2 = m_rex3.m_y_end_i;

	const bool iterate_one = (m_rex3.m_draw_mode0 & 0x300) == 0;
	const bool skip_first = BIT(m_rex3.m_draw_mode0, 10);
	const bool skip_last = BIT(m_rex3.m_draw_mode0, 11);
	const bool shade = BIT(m_rex3.m_draw_mode0, 18);
	const bool rgbmode = BIT(m_rex3.m_draw_mode1, 15);

	int32_t x = x1;
	int32_t y = y1;
	int32_t dx = abs(x2 - x1);
	int32_t dy = abs(y2 - y1);

	static const bresenham_octant_info_t s_bresenham_infos[8] =
	{
		{  0,  1, -1, -1,  1 }, // Pseudo 7
		{  0,  1,  1,  1,  1 }, // Pseudo 2
		{  0, -1, -1, -1,  1 }, // Pseudo 6
		{  0, -1,  1,  1,  1 }, // Pseudo 3
		{  1,  1,  0, -1,  0 }, // Pseudo 8
		{  1,  1,  0,  1,  0 }, // Pseudo 1
		{ -1, -1,  0, -1,  0 }, // Pseudo 5
		{ -1, -1,  0,  1,  0 }  // Pseudo 4
	};

	const uint8_t octant = (m_rex3.m_bres_octant_inc1 >> 24) & 0x7;
	const int32_t incrx1 = s_bresenham_infos[octant].incrx1;
	const int32_t incrx2 = s_bresenham_infos[octant].incrx2;
	const int32_t incry1 = s_bresenham_infos[octant].incry1;
	const int32_t incry2 = s_bresenham_infos[octant].incry2;
	int32_t loop = s_bresenham_infos[octant].loop ? dy : dx;
	if (BIT(m_rex3.m_draw_mode0, 15) && loop > 32)
		loop = 32;

	const int32_t incr1 = 2 * (s_bresenham_infos[octant].loop ? dx : dy);
	const int32_t incr2 = 2 * (s_bresenham_infos[octant].loop ? (dx - dy) : (dy - dx));
	int32_t d = incr1 - (s_bresenham_infos[octant].loop ? dy : dx);

	if (!skip_first || iterate_one)
	{
		const int16_t x16 = (int16_t)x;
		const int16_t y16 = (int16_t)y;

		line_pixel(x16, y16, color, shade || rgbmode);

		if (shade)
			iterate_shade();

		if (d < 0)
		{
			x += incrx1;
			y -= incry1;
			d += incr1;
		}
		else
		{
			x += incrx2;
			y -= incry2;
			d += incr2;
		}

		if (iterate_one)
		{
			write_x_start(x << 11);
			write_y_start(y << 11);
			return;
		}
	}

	for (int32_t i = 1; i < loop; i++)
	{
		const int16_t x16 = (int16_t)x;
		const int16_t y16 = (int16_t)y;

		line_pixel(x16, y16, color, shade || rgbmode);

		if (shade)
			iterate_shade();

		if (d < 0)
		{
			x += incrx1;
			y -= incry1;
			d += incr1;
		}
		else
		{
			x += incrx2;
			y -= incry2;
			d += incr2;
		}
	}

	if (!skip_last)
	{
		const int16_t x16 = (int16_t)x2;
		const int16_t y16 = (int16_t)y2;

		line_pixel(x16, y16, color, shade || rgbmode, true);

		if (shade)
			iterate_shade();

		write_x_start(x2 << 11);
		write_y_start(y2 << 11);
	}
	else
	{
		write_x_start(x << 11);
		write_y_start(y << 11);
	}
}

uint32_t newport_base_device::do_pixel_read()
{
	const int16_t src_x = m_rex3.m_x_start_i + m_rex3.m_x_window - 0x1000;
	const int16_t src_y = m_rex3.m_y_start_i + m_rex3.m_y_window - 0x1000;
	uint32_t ret = 0;
#ifdef MAME_SGI_RB2_XY_ADDRESS
	set_address_xy(src_x, src_y);
#else
	set_address(src_y * (1280 + 64) + src_x);
#endif
	ret = read_pixel();
	if ((m_rex3.m_plane_enable == 1 || m_rex3.m_plane_enable == 2) && BIT(m_rex3.m_draw_mode1, 15)) // RGB or RGBA planes, RGB mode (MAME_SGI_CI_HOSTDEPTH_FIX)
	{
		uint8_t convert_index = (m_rex3.m_plane_depth << 2) | m_rex3.m_hostdepth;
		switch (convert_index & 15)
		{
		default:
			// No conversion needed
			break;
		case 1:     // 4bpp -> 8bpp
			ret = convert_4bpp_bgr_to_8bpp((uint8_t)ret);
			break;
		case 2:     // 4bpp -> 12bpp
			ret = convert_4bpp_bgr_to_12bpp((uint8_t)ret);
			break;
		case 3:     // 4bpp -> 24bpp
			ret = convert_4bpp_bgr_to_24bpp((uint8_t)ret);
			break;
		case 4:     // 8bpp -> 4bpp
			ret = convert_8bpp_bgr_to_4bpp((uint8_t)ret);
			break;
		case 6:     // 8bpp -> 12bpp
			ret = convert_8bpp_bgr_to_12bpp((uint8_t)ret);
			break;
		case 7:     // 8bpp -> 24bpp
			ret = convert_8bpp_bgr_to_24bpp((uint8_t)ret);
			break;
		case 8:     // 12bpp -> 4bpp
			ret = convert_12bpp_bgr_to_4bpp((uint16_t)ret);
			break;
		case 9:     // 12bpp -> 8bpp
			ret = convert_12bpp_bgr_to_8bpp((uint16_t)ret);
			break;
		case 11:    // 12bpp -> 24bpp
			ret = convert_12bpp_bgr_to_24bpp((uint16_t)ret);
			break;
		case 12:    // 32bpp -> 4bpp
			ret = convert_24bpp_bgr_to_4bpp(ret);
			break;
		case 13:    // 32bpp -> 8bpp
			ret = convert_24bpp_bgr_to_8bpp(ret);
			break;
		case 14:    // 32bpp -> 12bpp
			ret = convert_24bpp_bgr_to_12bpp(ret);
			break;
		}
	}
	LOGMASKED(LOG_COMMANDS, "Read %08x from %04x, %04x\n", ret, src_x, src_y);
	m_rex3.m_x_start_i++;
	int16_t dy = (m_rex3.m_y_end_i < m_rex3.m_y_start_i) ? -1 : 1;
	if (m_rex3.m_x_start_i > m_rex3.m_x_end_i)
	{
		m_rex3.m_y_start_i += dy;
		m_rex3.m_x_start_i = m_rex3.m_x_save;
	}

	write_x_start(m_rex3.m_x_start_i << 11);
	write_y_start(m_rex3.m_y_start_i << 11);
	return ret;
}

uint64_t newport_base_device::do_pixel_word_read()
{
	const uint16_t x_start = (uint16_t)(m_rex3.m_xy_start_i >> 16);
	const uint16_t x_end = (uint16_t)(m_rex3.m_xy_end_i >> 16);
	const bool doubleword = BIT(m_rex3.m_draw_mode1, 10);
	// MAME_SGI_SWAPENDIAN_FIX: short and long pixels are byte-swapped on reads too
	const bool swap = BIT(m_rex3.m_draw_mode1, 11);

	// MAME_SGI_REX3_READ_WIDTH_FIX: a read starting past XEND transfers a
	// full word. XEND is often left behind the read position by the last
	// draw: IRIX's OpenGL reads pixels one at a time by setting only XYSTART.
	// A start further past XEND already wrapped to a large count here, but
	// one exactly at XEND + 1 gave a count of 0 and read nothing.
	const int32_t count = (int16_t)(x_end - x_start) + 1;
	uint16_t width = count > 0 ? (uint16_t)count : 0xffff;
	uint64_t ret = 0;
	uint64_t shift = 0;
	switch ((m_rex3.m_draw_mode1 >> 8) & 3)
	{
		case 0: // 4bpp
		{
			// MAME_SGI_4BPP_DB_FIX: each host pixel is in an 8-bit field,
			// right-aligned and zero-filled, as at 8bpp (and as the host
			// write path unpacks it)
			const uint16_t max_width = doubleword ? 8 : 4;
			if (width > max_width)
				width = max_width;

			shift = 56;
			for (uint16_t i = 0; i < width; i++)
			{
				ret |= (uint64_t)(do_pixel_read() & 0x0000000f) << shift;
				shift -= 8;
			}
			break;
		}

		case 1: // 8bpp
		{
			const uint16_t max_width = doubleword ? 8 : 4;
			if (width > max_width)
				width = max_width;

			shift = 56;
			for (uint16_t i = 0; i < width; i++)
			{
				ret |= (uint64_t)(do_pixel_read() & 0x000000ff) << shift;
				shift -= 8;
			}
			break;
		}

		case 2: // 12bpp
		{
			const uint16_t max_width = doubleword ? 4 : 2;
			if (width > max_width)
				width = max_width;

			shift = 48;
			for (uint16_t i = 0; i < width; i++)
			{
				ret |= (uint64_t)(swap ? swap_host_pixel(do_pixel_read() & 0x00000fff, 2) : do_pixel_read() & 0x00000fff) << shift;
				shift -= 16;
			}
			break;
		}

		case 3: // 32bpp
		{
			const uint16_t max_width = doubleword ? 2 : 1;
			if (width > max_width)
				width = max_width;

			shift = 32;
			for (uint16_t i = 0; i < width; i++)
			{
				ret |= (uint64_t)(swap ? swap_host_pixel(do_pixel_read(), 3) : do_pixel_read()) << shift;
				shift -= 32;
			}
			break;
		}
	}
	return ret;
}

void newport_base_device::iterate_shade()
{
	if (m_rex3.m_slope_red & 0x7fffff)
	{
		// MAME_SGI_CI_SLOPE_FIX: a 12bpp colour index is 12.9 fixed point (see
		// the Red/CI full state write) but its slope is 12.11, as at the other
		// depths
		if (!BIT(m_rex3.m_draw_mode1, 15) && m_rex3.m_plane_depth == 2)
			m_rex3.m_curr_color_red += util::sext(m_rex3.m_slope_red, 24) >> 2;
		else
			m_rex3.m_curr_color_red += util::sext(m_rex3.m_slope_red, 24);
	}
	if (m_rex3.m_slope_green & 0x7ffff)
		m_rex3.m_curr_color_green += util::sext(m_rex3.m_slope_green, 20);
	if (m_rex3.m_slope_blue & 0x7ffff)
		m_rex3.m_curr_color_blue += util::sext(m_rex3.m_slope_blue, 20);
	if (m_rex3.m_slope_alpha & 0x7ffff)
		m_rex3.m_curr_color_alpha += util::sext(m_rex3.m_slope_alpha, 20);

	if (BIT(m_rex3.m_draw_mode0, 21)) // CIClamp
	{
		if (BIT(m_rex3.m_draw_mode1, 15)) // RGBMode
		{
			const uint32_t val_red = ((m_rex3.m_curr_color_red >> 11) & 0x1ff);
			const uint32_t val_grn = ((m_rex3.m_curr_color_green >> 11) & 0x1ff);
			const uint32_t val_blu = ((m_rex3.m_curr_color_blue >> 11) & 0x1ff);
			const uint32_t val_alpha = ((m_rex3.m_curr_color_alpha >> 11) & 0x1ff);

			if (val_red >= 0x180 || BIT(m_rex3.m_curr_color_red, 31))
				m_rex3.m_curr_color_red = 0;
			else if (val_red > 0xff)
				m_rex3.m_curr_color_red = 0x7ffff;

			if (val_grn >= 0x180 || BIT(m_rex3.m_curr_color_green, 31))
				m_rex3.m_curr_color_green = 0;
			else if (val_grn > 0xff)
				m_rex3.m_curr_color_green = 0x7ffff;

			if (val_blu >= 0x180 || BIT(m_rex3.m_curr_color_blue, 31))
				m_rex3.m_curr_color_blue = 0;
			else if (val_blu > 0xff)
				m_rex3.m_curr_color_blue = 0x7ffff;

			if (val_alpha >= 0x180 || BIT(m_rex3.m_curr_color_alpha, 31))
				m_rex3.m_curr_color_alpha = 0;
			else if (val_alpha > 0xff)
				m_rex3.m_curr_color_alpha = 0x7ffff;
		}
		else
		{
			switch ((m_rex3.m_draw_mode1 >> 3) & 3)
			{
			case 0: // 4bpp
				if (BIT(m_rex3.m_color_red, 15))
					m_rex3.m_color_red = 0x00007fff;
				break;
			case 1: // 8bpp
				if (BIT(m_rex3.m_color_red, 19))
					m_rex3.m_color_red = 0x0007ffff;
				break;
			case 2: // 12bpp
				if (BIT(m_rex3.m_color_red, 21))
					m_rex3.m_color_red = 0x001fffff;
				break;
			case 3: // 24bpp
					// No clamping on CI
				break;
			}
		}
	}
}

uint32_t newport_base_device::get_default_color(uint32_t src)
{
#ifdef MAME_SGI_COLOR_SRC_DIAG
	if (BIT(m_rex3.m_draw_mode1, 17) && m_rex3.m_color_vram != src)
	{
		color_src_diag_log("[COLOR_SRC_DIAG] fastclear-color substituted: "
			"mode0=0x%08x mode1=0x%08x color_vram=0x%08x src=0x%08x "
			"rect=(%d,%d)-(%d,%d) plane_depth=%d\n",
			m_rex3.m_draw_mode0, m_rex3.m_draw_mode1, m_rex3.m_color_vram, src,
			m_rex3.m_x_start_i, m_rex3.m_y_start_i, m_rex3.m_x_end_i, m_rex3.m_y_end_i,
			m_rex3.m_plane_depth);
	}
#endif
	uint32_t color = BIT(m_rex3.m_draw_mode1, 17) ? m_rex3.m_color_vram : src;
	// MAME_SGI_FASTCLEAR_RGB_FIX: in RGB mode COLORVRAM holds an 8-8-8 colour
	// (OpenGL loads one at every depth, already rounded to the depth), which
	// fast-clear converts by taking each channel's top bits: B1 G2 R1 at
	// 4bpp, B2 G3 R3 at 8bpp, as the 12bpp case below
	const uint32_t vram = m_rex3.m_color_vram;
	if (BIT(m_rex3.m_draw_mode1, 17) && BIT(m_rex3.m_draw_mode1, 15) && m_rex3.m_plane_depth < 2)
	{
		if (m_rex3.m_plane_depth == 0)
			color = (BIT(vram, 23) << 3) | (((vram >> 14) & 3) << 1) | BIT(vram, 7);
		else
			color = (((vram >> 22) & 3) << 6) | (((vram >> 13) & 7) << 3) | ((vram >> 5) & 7);
	}
	switch (m_rex3.m_plane_depth)
	{
		case 0: // 4bpp
			color &= 0xf;
			color |= color << 4;
			color |= color << 8;
			color |= color << 16;
			break;
		case 1: // 8bpp
			color &= 0xff;
			color |= color << 8;
			color |= color << 16;
			break;
		case 2: // 12bpp
			if (BIT(m_rex3.m_draw_mode1, 15))
				color = ((m_rex3.m_color_vram & 0xf00000) >> 12) | ((m_rex3.m_color_vram & 0xf000) >> 8) | ((m_rex3.m_color_vram & 0xf0) >> 4);
			else
				color &= 0x00000fff;
			color |= color << 12;
			break;
		case 3: // 24bpp
			color = m_rex3.m_color_vram & 0xffffff;
			break;
	}

	return color;
}

#ifdef MAME_SGI_REX3_FAST_SPAN
// The x in [xa, xb] (pre-window coordinates) for which pixel_clip_pass(x, y)
// holds, as up to 5 sorted, disjoint inclusive intervals lo[i]..hi[i];
// returns their count, or -1 if the window offset could wrap the 16-bit
// coordinates somewhere in the range (then use the per-pixel test).
int newport_base_device::clip_row_intervals(int16_t y, int xa, int xb, int lo[5], int hi[5])
{
	int ilo = xa, ihi = xb;
	if (BIT(m_rex3.m_clip_mode, 0))
	{
		const int16_t min_x = (int16_t)(m_rex3.m_smask_x[0] >> 16);
		const int16_t min_y = (int16_t)(m_rex3.m_smask_y[0] >> 16);
		const int16_t max_x = (int16_t)m_rex3.m_smask_x[0];
		const int16_t max_y = (int16_t)m_rex3.m_smask_y[0];
		if (y < min_y || y > max_y)
			return 0;
		ilo = std::max(ilo, int(min_x));
		ihi = std::min(ihi, int(max_x));
		if (ilo > ihi)
			return 0;
	}

	// window coordinates: wx = x + cx without 16-bit wraparound on [ilo, ihi]
	const int cx = int(m_rex3.m_x_window) - 0x1000;
	if (ilo + cx < -32768 || ihi + cx > 32767)
		return -1;
	const int16_t wy = (int16_t)(y + m_rex3.m_y_window - 0x1000);
	if (wy < 0 || wy >= 1024)
		return 0;
	ilo = std::max(ilo, 0 - cx);
	ihi = std::min(ihi, (1280 + 64 - 1) - cx);
	if (ilo > ihi)
		return 0;

	if (!(m_rex3.m_clip_mode & 0x1e))
	{
		lo[0] = ilo;
		hi[0] = ihi;
		return 1;
	}
	int n = 0;
	for (int bit = 1; bit < 5; bit++)
	{
		if (!BIT(m_rex3.m_clip_mode, bit))
			continue;
		const int16_t min_x = (int16_t)(m_rex3.m_smask_x[bit] >> 16) - 0x1000;
		const int16_t min_y = (int16_t)(m_rex3.m_smask_y[bit] >> 16) - 0x1000;
		const int16_t max_x = (int16_t)m_rex3.m_smask_x[bit] - 0x1000;
		const int16_t max_y = (int16_t)m_rex3.m_smask_y[bit] - 0x1000;
		if (wy < min_y || wy > max_y)
			continue;
		const int a = std::max(ilo, int(min_x) - cx), b = std::min(ihi, int(max_x) - cx);
		if (a <= b)
		{
			lo[n] = a;
			hi[n] = b;
			n++;
		}
	}
	// sort and merge overlapping or touching intervals
	for (int i = 1; i < n; i++)
		for (int j = i; j > 0 && lo[j] < lo[j - 1]; j--)
		{
			std::swap(lo[j], lo[j - 1]);
			std::swap(hi[j], hi[j - 1]);
		}
	int m = 0;
	for (int i = 0; i < n; i++)
	{
		if (m && lo[i] <= hi[m - 1] + 1)
			hi[m - 1] = std::max(hi[m - 1], hi[i]);
		else
		{
			lo[m] = lo[i];
			hi[m] = hi[i];
			m++;
		}
	}
	return m;
}

// Span (adrmode 0) and block (adrmode 1) draws as clipped row fills, for the
// cases where every pixel's value is known up front: no host data, shading
// or blending, a logic op that doesn't read the destination, left to right,
// and a span/block extent the per-pixel loops in do_rex3_command() would
// reach without wrapping. start/end are the command's x/y start/end
// (integer parts), before do_rex3_command()'s end += step. Returns false,
// having changed nothing, when the draw doesn't qualify.
bool newport_base_device::fast_draw(uint8_t adrmode, int16_t start_x, int16_t start_y, int16_t end_x, int16_t end_y, int16_t dx, int16_t dy)
{
	const uint32_t mode0 = m_rex3.m_draw_mode0;
	const uint32_t mode1 = m_rex3.m_draw_mode1;
	if (m_fast_draw_disabled || m_pixel_write_suppress || m_rex3.m_color_host || m_rex3.m_alpha_host || BIT(mode0, 18) || BIT(mode0, 23) || BIT(mode1, 18) || dx != 1)
		return false;
	// the alpha function is applied per pixel (output_pixel)
	if (((mode1 >> 12) & 7) != 7)
		return false;

	const bool stop_on_x = BIT(mode0, 8);
	const bool rgbmode = BIT(mode1, 15);
	const bool fastclear = BIT(mode1, 17);
	const bool opaque = BIT(mode0, 16) || BIT(mode0, 17);
	const uint32_t pattern = BIT(mode0, 12) ? m_rex3.m_z_pattern : (BIT(mode0, 13) ? m_rex3.m_ls_pattern : 0xffffffff);

	// first row's pixel count and the row list
	int first_count, rows;
	int16_t row_x0 = m_rex3.m_x_save;
	const int16_t end_x1 = end_x + dx;
	int16_t end_y1 = end_y;
	if (adrmode == 0)
	{
		int16_t prim_end_x = end_x1;
		if (BIT(mode0, 15) && abs(end_x1 - start_x) > 32)
			prim_end_x = start_x + 32 * dx;
		if (!stop_on_x)
			first_count = 1;
		else
		{
			// the loop stops at whichever of prim_end_x, end_x1 it reaches first
			const int d1 = prim_end_x - start_x, d2 = end_x1 - start_x;
			if (d1 <= 0 || d2 <= 0)
				return false;
			first_count = std::min(d1, d2);
		}
		rows = 1;
	}
	else
	{
		end_y1 = end_y + dy;
		if (!stop_on_x || !BIT(mode0, 9) || BIT(mode0, 15))
			return false;
		first_count = end_x1 - start_x;
		rows = (end_y1 - start_y) * dy;
		if (first_count <= 0 || rows <= 0 || (rows > 1 && end_x1 - row_x0 <= 0))
			return false;
	}

	// stored values: foreground per x & 3 (dither) for each row, background
	const uint32_t color = get_default_color(m_rex3.m_color_i);
	uint32_t bg = 0;
	if (opaque && !m_rb2->stored_value(m_rex3.m_color_back, bg))
		return false;
	uint32_t fg_flat;
	if (!m_rb2->stored_value(color, fg_flat))
		return false;
	const bool per_pixel_color = rgbmode && !fastclear;

#ifdef MAME_SGI_REX3_FAST_SPAN_VERIFY
	// Run the per-pixel path on the current state, keep its VRAM rows and
	// start registers, restore, and compare with the fast draw below.
	struct row_copy { int vy; std::vector<uint32_t> data; };
	std::vector<row_copy> before, slow;
	const auto save_rows = [&] (std::vector<row_copy> &out)
	{
		out.clear();
		for (int r = 0, y = start_y; r < rows; r++, y += dy)
		{
			const int vy = (int16_t)(y + m_rex3.m_y_window - 0x1000);
			if (vy < 0 || vy >= 1024)
				continue;
			const uint32_t *p = m_rb2->dest_row(vy);
			out.push_back({ vy, std::vector<uint32_t>(p, p + 1280 + 64) });
		}
	};
	save_rows(before);
	const rex3_t rex3_before = m_rex3;
	m_fast_draw_disabled = true;
	do_rex3_command();
	m_fast_draw_disabled = false;
	save_rows(slow);
	const rex3_t rex3_slow = m_rex3;
	for (const auto &r : before)
		std::copy(r.data.begin(), r.data.end(), m_rb2->dest_row(r.vy));
	m_rex3 = rex3_before;
#endif

	int lo[5], hi[5];
	int16_t y = start_y;
	int16_t x = start_x;
	for (int r = 0; r < rows; r++, y += dy)
	{
		const int count = r ? end_x1 - row_x0 : first_count;
		const int16_t rx = r ? row_x0 : start_x;
		uint32_t fg[4];
		if (per_pixel_color)
		{
			for (int i = 0; i < 4; i++)
				if (!m_rb2->stored_value(get_rgb_color(i, y), fg[i]))
					return false;	// can't happen: same logic op as the flat color
		}
		else
			fg[0] = fg[1] = fg[2] = fg[3] = fg_flat;
		// store_row() indexes by VRAM x & 3; the dither uses x & 3 before the
		// window offset
		uint32_t fgv[4];
		const int wcx = int(m_rex3.m_x_window) - 0x1000;
		for (int i = 0; i < 4; i++)
			fgv[(i + wcx) & 3] = fg[i];

		const int n = clip_row_intervals(y, rx, rx + count - 1, lo, hi);
		if (n < 0)
		{
			// window offset wraps within the row: per-pixel clip test
			for (int i = 0; i < count; i++)
			{
				const int16_t px = rx + i;
				if (!pixel_clip_pass(px, y))
					continue;
				const int16_t wx = px + m_rex3.m_x_window - 0x1000;
				const int16_t wy = y + m_rex3.m_y_window - 0x1000;
				m_rb2->store_row(wy, wx, wx, fgv, BIT(pattern, (31 - i) & 0x1f) ? 0xffffffff : 0, 0, opaque, bg);
			}
		}
		for (int k = 0; k < n; k++)
		{
			const int16_t wy = y + m_rex3.m_y_window - 0x1000;
			const int cx = int(m_rex3.m_x_window) - 0x1000;
			// pattern bit of pixel lo[k] is 31 minus its offset in the row
			m_rb2->store_row(wy, lo[k] + cx, hi[k] + cx, fgv, pattern, (31 - (lo[k] - rx)) & 0x1f, opaque, bg);
		}
		x = rx + count;
	}

	// end state, as the per-pixel loops leave it
	if (adrmode == 0)
	{
		if (x >= end_x1)
		{
			m_rex3.m_curr_color_red = m_rex3.m_color_red;
			m_rex3.m_curr_color_alpha = m_rex3.m_color_alpha;
			m_rex3.m_curr_color_green = m_rex3.m_color_green;
			m_rex3.m_curr_color_blue = m_rex3.m_color_blue;
		}
		write_x_start(x << 11);
	}
	else
	{
		m_rex3.m_curr_color_red = m_rex3.m_color_red;
		m_rex3.m_curr_color_alpha = m_rex3.m_color_alpha;
		m_rex3.m_curr_color_green = m_rex3.m_color_green;
		m_rex3.m_curr_color_blue = m_rex3.m_color_blue;
		write_x_start(m_rex3.m_x_save << 11);
		write_y_start(end_y1 << 11);
	}

#ifdef MAME_SGI_REX3_FAST_SPAN_VERIFY
	{
		static struct verify_counts { uint64_t draws = 0, bad = 0; ~verify_counts() { fprintf(stderr, "REX3_FAST_SPAN exit verified=%llu bad=%llu\n", (unsigned long long)draws, (unsigned long long)bad); } } s_counts;
		uint64_t &s_draws = s_counts.draws, &s_bad = s_counts.bad;
		std::vector<row_copy> fast;
		save_rows(fast);
		bool ok = fast.size() == slow.size();
		for (size_t i = 0; ok && i < fast.size(); i++)
			ok = fast[i].vy == slow[i].vy && fast[i].data == slow[i].data;
		const bool regs_ok = m_rex3.m_x_start == rex3_slow.m_x_start && m_rex3.m_y_start == rex3_slow.m_y_start &&
				m_rex3.m_xy_start_i == rex3_slow.m_xy_start_i && m_rex3.m_curr_color_red == rex3_slow.m_curr_color_red &&
				m_rex3.m_curr_color_green == rex3_slow.m_curr_color_green && m_rex3.m_curr_color_blue == rex3_slow.m_curr_color_blue &&
				m_rex3.m_curr_color_alpha == rex3_slow.m_curr_color_alpha;
		s_draws++;
		if ((!ok || !regs_ok) && ++s_bad <= 20)
			fprintf(stderr, "REX3_FAST_SPAN mismatch adr=%d m0=%08x m1=%08x start=%d,%d end=%d,%d xsave=%d vram=%d regs=%d xs=%08x/%08x ys=%08x/%08x\n",
					adrmode, mode0, mode1, start_x, start_y, end_x, end_y, m_rex3.m_x_save, ok, regs_ok,
					m_rex3.m_x_start, rex3_slow.m_x_start, m_rex3.m_y_start, rex3_slow.m_y_start);
		if ((s_draws & ((1 << 20) - 1)) == 0)
			fprintf(stderr, "REX3_FAST_SPAN verified=%llu bad=%llu\n", (unsigned long long)s_draws, (unsigned long long)s_bad);
		if (!ok || !regs_ok)
		{
			// keep the per-pixel result
			for (const auto &r : slow)
				std::copy(r.data.begin(), r.data.end(), m_rb2->dest_row(r.vy));
			m_rex3 = rex3_slow;
		}
	}
#endif
	return true;
}
#endif

#ifdef MAME_SGI_REX3_MODE_CENSUS
namespace {
// Per draw-mode statistics of REX3 commands: key = DRAWMODE0 << 32 |
// DRAWMODE1, counting commands, pixels written and host TSC cycles
// (rdtsc: osd_ticks() is a clock_gettime() call, too slow to bracket
// one-pixel commands on hosts without a TSC clocksource). Printed to
// stderr every 60 s and at exit, sorted by cycles.
struct rex3_mode_census
{
	struct entry { uint64_t commands = 0, pixels = 0, ticks = 0; };
	std::unordered_map<uint64_t, entry> table;
	uint64_t next_dump = 0;

	static std::string decode(uint64_t key)
	{
		static const char *const s_op[4] = { "noop", "read", "draw", "scr2scr" };
		static const char *const s_adr[8] = { "span", "block", "iline", "fline", "aline", "adr5", "adr6", "adr7" };
		static const char *const s_planes[8] = { "none", "rgb/ci", "rgba", "pl3", "olay", "pup", "cid", "pl7" };
		static const char *const s_depth[4] = { "4", "8", "12", "24" };
		const uint32_t m0 = uint32_t(key >> 32), m1 = uint32_t(key);
		std::string r = util::string_format("%s %s %s/%sbpp", s_op[m0 & 3], s_adr[(m0 >> 2) & 7], s_planes[m1 & 7], s_depth[(m1 >> 3) & 3]);
		static const struct { int word, bit; const char *name; } s_flags[] = {
			{ 0, 6, "colorhost" }, { 0, 7, "alphahost" }, { 0, 8, "stopx" }, { 0, 9, "stopy" },
			{ 0, 12, "zpat" }, { 0, 13, "lspat" }, { 0, 15, "len32" }, { 0, 16, "zopaque" },
			{ 0, 17, "lsopaque" }, { 0, 18, "shade" }, { 0, 19, "lronly" }, { 0, 21, "ciclamp" },
			{ 1, 5, "dblsrc" }, { 1, 7, "rwpacked" }, { 1, 10, "rwdouble" }, { 1, 15, "rgbmode" },
			{ 1, 16, "dither" }, { 1, 17, "fastclear" }, { 1, 18, "blend" }, { 1, 25, "backblend" } };
		for (const auto &f : s_flags)
			if (BIT(f.word ? m1 : m0, f.bit))
				r += std::string(" ") + f.name;
		if (BIT(m1, 18))
			r += util::string_format(" sf=%d df=%d", (m1 >> 19) & 7, (m1 >> 22) & 7);
		r += util::string_format(" lop=%d", m1 >> 28);
		return r;
	}

	void dump(const char *when)
	{
		std::vector<std::pair<uint64_t, entry>> rows(table.begin(), table.end());
		std::sort(rows.begin(), rows.end(), [] (const auto &a, const auto &b) { return a.second.ticks > b.second.ticks; });
		uint64_t ticks = 0, pixels = 0, commands = 0;
		for (const auto &r : rows) { ticks += r.second.ticks; pixels += r.second.pixels; commands += r.second.commands; }
		fprintf(stderr, "REX3_CENSUS %s modes=%d commands=%llu pixels=%llu Gcycles=%.2f\n", when, int(rows.size()),
				(unsigned long long)commands, (unsigned long long)pixels, ticks * 1e-9);
		for (size_t i = 0; i < rows.size() && i < 40; i++)
		{
			const auto &e = rows[i].second;
			fprintf(stderr, "REX3_CENSUS  %5.1f%%t %5.1f%%px %10llu cmd %12llu px %6.1f px/cmd  %08x %08x  %s\n",
					100.0 * e.ticks / std::max<uint64_t>(ticks, 1), 100.0 * e.pixels / std::max<uint64_t>(pixels, 1),
					(unsigned long long)e.commands, (unsigned long long)e.pixels, double(e.pixels) / std::max<uint64_t>(e.commands, 1),
					uint32_t(rows[i].first >> 32), uint32_t(rows[i].first), decode(rows[i].first).c_str());
		}
	}
	~rex3_mode_census() { dump("exit"); }
} s_rex3_census;

struct rex3_census_scope
{
	uint64_t key;
	uint64_t pixels0;
	uint64_t t0;
	rex3_census_scope(uint32_t mode0, uint32_t mode1) : key((uint64_t(mode0) << 32) | mode1), pixels0(g_mame_sgi_stat_pixel_writes), t0(MAME_SGI_CENSUS_CYCLES()) { }
	~rex3_census_scope()
	{
		const uint64_t t1 = MAME_SGI_CENSUS_CYCLES();
		auto &e = s_rex3_census.table[key];
		e.commands++;
		e.pixels += g_mame_sgi_stat_pixel_writes - pixels0;
		e.ticks += t1 - t0;
		if (t1 >= s_rex3_census.next_dump)
		{
			if (s_rex3_census.next_dump)
				s_rex3_census.dump("periodic");
			s_rex3_census.next_dump = t1 + 60 * 3'000'000'000ULL;
		}
	}
};
} // anonymous namespace
#endif

// Diagnostic: log each REX3 command whose start point (screen coordinates)
// lies in the rectangle MAME_SGI_REX3_TRACE_RECT="x0,y0,x1,y1" (environment),
// with its draw modes, colour registers, write mask, Bresenham registers
// and fixed-point start, to stderr as
// REX3_TRACE lines, at most MAME_SGI_REX3_TRACE_MAX (default 2000) of them;
// with MAME_SGI_REX3_TRACE_M1MASK (or _M0MASK) set, only commands with any
// of those DRAWMODE1 (DRAWMODE0) bits set (DRAWMODE1 0x40000: blended;
// DRAWMODE0 0x300: stop on x/y, i.e. more than one pixel). With
// MAME_SGI_REX3_TRACE_FLAG naming a file, tracing is on only while that file
// exists (checked every 65536 commands), to capture a moment of a running
// program.
//#define MAME_SGI_REX3_CMD_TRACE
#ifdef MAME_SGI_REX3_CMD_TRACE
#include <unistd.h>
namespace {
struct rex3_trace_config
{
	bool on = false;
	int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
	long remaining = 2000;
	uint32_t m1_mask = 0, m0_mask = 0;
	const char *flag = nullptr;
	bool flag_on = true;
	uint32_t flag_count = 0;
	// whether to trace now: the flag file, if any, is rechecked every 65536 calls
	bool active()
	{
		if (flag && (flag_count++ & 0xffff) == 0)
			flag_on = access(flag, F_OK) == 0;
		return flag_on;
	}
	rex3_trace_config()
	{
		flag = getenv("MAME_SGI_REX3_TRACE_FLAG");
		if (flag)
			flag_on = false;
		if (const char *m = getenv("MAME_SGI_REX3_TRACE_M0MASK"))
			m0_mask = strtoul(m, nullptr, 0);
		if (const char *m = getenv("MAME_SGI_REX3_TRACE_M1MASK"))
			m1_mask = strtoul(m, nullptr, 0);
		const char *r = getenv("MAME_SGI_REX3_TRACE_RECT");
		on = r && sscanf(r, "%d,%d,%d,%d", &x0, &y0, &x1, &y1) == 4;
		if (const char *m = getenv("MAME_SGI_REX3_TRACE_MAX"))
			remaining = atol(m);
	}
} s_rex3_trace;
} // anonymous namespace
#endif

void newport_base_device::do_rex3_command()
{
#ifdef MAME_SGI_REX3_MODE_CENSUS
	rex3_census_scope census_scope(m_rex3.m_draw_mode0, m_rex3.m_draw_mode1);
#endif
#ifdef MAME_SGI_REX3_CMD_TRACE
	if (s_rex3_trace.on && s_rex3_trace.remaining > 0 && s_rex3_trace.active())
	{
		const int sx = m_rex3.m_x_start_i + int(m_rex3.m_x_window) - 0x1000;
		const int sy = m_rex3.m_y_start_i + int(m_rex3.m_y_window) - 0x1000;
		if (sx >= s_rex3_trace.x0 && sx <= s_rex3_trace.x1 && sy >= s_rex3_trace.y0 && sy <= s_rex3_trace.y1
				&& (!s_rex3_trace.m1_mask || (m_rex3.m_draw_mode1 & s_rex3_trace.m1_mask))
				&& (!s_rex3_trace.m0_mask || (m_rex3.m_draw_mode0 & s_rex3_trace.m0_mask)))
		{
			s_rex3_trace.remaining--;
			fprintf(stderr, "REX3_TRACE m0=%08x m1=%08x start=%d,%d end=%d,%d win=%d,%d ci=%08x vram=%08x rgba=%08x/%08x/%08x/%08x back=%08x wmask=%08x zpat=%08x bres=%08x/%08x/%08x/%08x startf=%08x,%08x\n",
					m_rex3.m_draw_mode0, m_rex3.m_draw_mode1, m_rex3.m_x_start_i, m_rex3.m_y_start_i,
					m_rex3.m_x_end_i, m_rex3.m_y_end_i, int(m_rex3.m_x_window) - 0x1000, int(m_rex3.m_y_window) - 0x1000,
					m_rex3.m_color_i, m_rex3.m_color_vram, m_rex3.m_color_red, m_rex3.m_color_green, m_rex3.m_color_blue,
					m_rex3.m_color_alpha, m_rex3.m_color_back, m_rex3.m_write_mask, m_rex3.m_z_pattern,
					m_rex3.m_bres_d, m_rex3.m_bres_s1, m_rex3.m_bres_octant_inc1, m_rex3.m_bres_round_inc2,
					m_rex3.m_x_start, m_rex3.m_y_start);
		}
	}
#endif
	static const char* const s_opcode_str[4] = { "Noop", "Read", "Draw", "Scr2Scr" };
	static const char* const s_adrmode_str[8] = {
		"Span", "Block", "IntLine", "FracLine", "AALine", "Unk5", "Unk6", "Unk7"
	};

	const uint32_t mode0 = m_rex3.m_draw_mode0;
	const uint32_t mode1 = m_rex3.m_draw_mode1;

	int16_t start_x = m_rex3.m_x_start_i;
	int16_t start_y = m_rex3.m_y_start_i;
	int16_t end_x = m_rex3.m_x_end_i;
	int16_t end_y = m_rex3.m_y_end_i;
	int16_t dx = 1, dy = 1;
	int8_t octant;

	LOGMASKED(LOG_COMMANDS, "REX3 Command: %08x|%08x - %s %s %s\n", mode0, mode1, s_opcode_str[mode0 & 3], s_adrmode_str[(mode0 >> 2) & 7], BIT(mode0, 5) ? "Setup" : "NoSetup");

	const uint8_t opcode = mode0 & 3;
	const uint8_t adrmode = (mode0 >> 2) & 7;

	switch (opcode)
	{
		case 0: // NoOp
			break;
		case 1: // Read
			if (BIT(mode0, 5))
				do_setup();
			// Our do_pixel_word_read does not use octant, but seems like hardware is still updating it
			m_rex3.m_host_dataport = do_pixel_word_read();
#ifdef MAME_SGI_REX3_HOSTDATA_DIAG
			if (FILE *f = rex3_hostdata_diag_file(machine()))
				fprintf(f, "RD t=%.6f m0=%08x m1=%08x pf=%d x=%d y=%d -> dp=%016llx\n", machine().time().as_double(),
						mode0, mode1, BIT(mode1, 26), m_rex3.m_x_start_i, m_rex3.m_y_start_i, (unsigned long long)m_rex3.m_host_dataport);
#endif
			break;
		case 2: // Draw
			if (BIT(mode0, 5))
				do_setup();
			octant = (m_rex3.m_bres_octant_inc1 >> 24) & 0x7;
			dx = BIT(octant, 1) ? -1 : 1;
			dy = BIT(octant, 0) ? -1 : 1;

			switch (adrmode)
			{
				case 0: // Span
				{
#ifdef MAME_SGI_REX3_FAST_SPAN
					if (fast_draw(0, start_x, start_y, end_x, end_y, dx, dy))
						break;
#endif
					end_x += dx;

					int16_t prim_end_x = end_x;
					bool stop_on_x = BIT(mode0, 8);

					if (BIT(mode0, 15) && abs(end_x - start_x) > 32)
						prim_end_x = start_x + 32 * dx;

					if ((m_rex3.m_color_host || m_rex3.m_alpha_host) && m_rex3.m_rwpacked)
					{
						stop_on_x = true;
						static const int16_t s_max_host_lengths[2][4] = { { 4, 4, 2, 1 }, { 8, 8, 4, 2 } };
						const int16_t max_length = s_max_host_lengths[m_rex3.m_rwdouble ? 1 : 0][m_rex3.m_hostdepth];
						int16_t length = abs(prim_end_x - start_x);
						if (length > max_length)
							prim_end_x = start_x + dx * max_length;
					}

					const bool shade = BIT(mode0, 18);
					const bool rgbmode = BIT(mode1, 15);
					const bool opaque = BIT(mode0, 16) || BIT(mode0, 17);
					const bool fastclear = BIT(mode1, 17);
					const uint32_t pattern = BIT(mode0, 12) ? m_rex3.m_z_pattern : (BIT(mode0, 13) ? m_rex3.m_ls_pattern : 0xffffffff);

					LOGMASKED(LOG_COMMANDS, "%04x, %04x to %04x, %04x = %08x\n", start_x, start_y, end_x, end_y, pattern);

					uint32_t color = get_default_color(m_rex3.m_color_i);
					const bool lr_abort = BIT(mode0, 19) && dx < 0;

					uint32_t bit = 31;
					do
					{
						if (lr_abort)
							break;

						if (BIT(pattern, bit))
						{
							if (m_rex3.m_color_host)
								{
#ifdef MAME_SGI_REX3_HOSTDATA_DIAG
									const uint32_t diag_shift = m_rex3.m_host_shift;
									const uint32_t diag_color = get_host_color();
									if (FILE *f = rex3_hostdata_diag_file(machine()))
										fprintf(f, "P am=%d x=%d y=%d end=%d pend=%d sh=%u c=%08x m0=%08x m1=%08x\n",
												adrmode, start_x, start_y, end_x, prim_end_x, diag_shift, diag_color, mode0, mode1);
									output_pixel(start_x, start_y, diag_color);
#else
									output_pixel(start_x, start_y, get_host_color());
#endif
								}
							else if (m_rex3.m_alpha_host)
								output_pixel(start_x, start_y, draw_color(start_x, start_y, color, (shade || rgbmode) && !fastclear));
							else if ((shade || rgbmode) && !fastclear)
								output_pixel(start_x, start_y, get_rgb_color(start_x, start_y));
							else
								output_pixel(start_x, start_y, color);
						}
						else if (opaque)
						{
							output_pixel(start_x, start_y, m_rex3.m_color_back);
						}

						// MAME_SGI_SHADE_ORDER_FIX: the first pixel takes the start colour;
						// the iterators step after each pixel, as in the line commands
						if (shade)
							iterate_shade();

						bit = (bit - 1) & 0x1f;
						start_x += dx;
					} while (start_x != prim_end_x && start_x != end_x && stop_on_x);

					if ((dx > 0 && start_x >= end_x) || (dx < 0 && start_x <= end_x) || lr_abort)
					{
						m_rex3.m_curr_color_red = m_rex3.m_color_red;
						m_rex3.m_curr_color_alpha = m_rex3.m_color_alpha;
						m_rex3.m_curr_color_green = m_rex3.m_color_green;
						m_rex3.m_curr_color_blue = m_rex3.m_color_blue;
					}

					write_x_start(start_x << 11);
					break;
				}

				case 1: // Block
				{
#ifdef MAME_SGI_REX3_FAST_SPAN
					if (fast_draw(1, start_x, start_y, end_x, end_y, dx, dy))
						break;
#endif
					end_x += dx;
					end_y += dy;

					int16_t prim_end_x = end_x;
					bool stop_on_x = BIT(mode0, 8);
					const bool stop_on_y = BIT(mode0, 9);

					if (BIT(mode0, 15) && (end_x - start_x) >= 32)
						prim_end_x = start_x + 32 * dx;

					if ((m_rex3.m_color_host || m_rex3.m_alpha_host) && m_rex3.m_rwpacked)
					{
						stop_on_x = true;
						static const int16_t s_max_host_lengths[2][4] = { { 4, 4, 2, 1 }, { 8, 8, 4, 2 } };
						const int16_t max_length = s_max_host_lengths[m_rex3.m_rwdouble ? 1 : 0][m_rex3.m_hostdepth];
						int16_t length = abs(prim_end_x - start_x);
						if (length > max_length)
							prim_end_x = start_x + dx * max_length;
					}

					const bool shade = BIT(mode0, 18);
					const bool rgbmode = BIT(mode1, 15);
					const bool opaque = BIT(mode0, 16) || BIT(mode0, 17);
					const bool fastclear = BIT(mode1, 17);
					const uint32_t pattern = BIT(mode0, 12) ? m_rex3.m_z_pattern : (BIT(mode0, 13) ? m_rex3.m_ls_pattern : 0xffffffff);

					uint32_t color = get_default_color(m_rex3.m_color_i);
					const bool lr_abort = BIT(mode0, 19) && dx < 0;

					do
					{
						uint32_t bit = 31;
						do
						{
							if (lr_abort)
								break;

							if (BIT(pattern, bit))
							{
								if (m_rex3.m_color_host)
									{
#ifdef MAME_SGI_REX3_HOSTDATA_DIAG
										const uint32_t diag_shift = m_rex3.m_host_shift;
										const uint32_t diag_color = get_host_color();
										if (FILE *f = rex3_hostdata_diag_file(machine()))
											fprintf(f, "P am=%d x=%d y=%d end=%d pend=%d sh=%u c=%08x m0=%08x m1=%08x\n",
													adrmode, start_x, start_y, end_x, prim_end_x, diag_shift, diag_color, mode0, mode1);
										output_pixel(start_x, start_y, diag_color);
#else
										output_pixel(start_x, start_y, get_host_color());
#endif
									}
								else if (m_rex3.m_alpha_host)
									output_pixel(start_x, start_y, draw_color(start_x, start_y, color, (shade || rgbmode) && !fastclear));
								else if ((shade || rgbmode) && !fastclear)
									output_pixel(start_x, start_y, get_rgb_color(start_x, start_y));
								else
									output_pixel(start_x, start_y, color);
							}
							else if (opaque)
							{
								output_pixel(start_x, start_y, m_rex3.m_color_back);
							}

							// MAME_SGI_SHADE_ORDER_FIX: the first pixel takes the start colour;
							// the iterators step after each pixel, as in the line commands
							if (shade)
								iterate_shade();

							bit = (bit - 1) & 0x1f;
							start_x += dx;
						} while (start_x != prim_end_x && start_x != end_x && stop_on_x);

						if ((dx > 0 && start_x >= end_x) || (dx < 0 && start_x <= end_x) || lr_abort)
						{
							m_rex3.m_curr_color_red = m_rex3.m_color_red;
							m_rex3.m_curr_color_alpha = m_rex3.m_color_alpha;
							m_rex3.m_curr_color_green = m_rex3.m_color_green;
							m_rex3.m_curr_color_blue = m_rex3.m_color_blue;
							start_x = m_rex3.m_x_save;
							// MAME_SGI_YSTRIDE_FIX: DRAWMODE0 YSTRIDE (bit 23) steps two
							// rows at the end of a block row (video fields); the block
							// then ends when Y passes YEND rather than meets it
							start_y += BIT(mode0, 23) ? 2 * dy : dy;
						}
					} while ((BIT(mode0, 23) ? (dy > 0 ? start_y < end_y : start_y > end_y) : start_y != end_y) && stop_on_y);

					write_x_start(start_x << 11);
					write_y_start(start_y << 11);
					break;
				}

				case 2: // I_Line
					do_iline(get_default_color(m_rex3.m_color_i));
					break;

				case 3: // F_Line
					do_fline(get_default_color(m_rex3.m_color_i));
					break;

				case 4: // A_Line
					do_aline();
					break;

				default: // Invalid
					break;
			}
			break;
		case 3: // Scr2Scr
			if (adrmode < 2)
			{
				const bool stop_on_x = BIT(mode0, 8);
				const bool stop_on_y = BIT(mode0, 9);

				if (BIT(mode0, 5))
					do_setup();
				octant = (m_rex3.m_bres_octant_inc1 >> 24) & 0x7;
				dx = BIT(octant, 1) ? -1 : 1;
				dy = BIT(octant, 0) ? -1 : 1;

				end_x += dx;
				end_y += dy;

				LOGMASKED(LOG_COMMANDS, "%04x, %04x - %04x, %04x to %04x, %04x\n", start_x, start_y, end_x, end_y, start_x + m_rex3.m_x_move, start_y + m_rex3.m_y_move);
				do
				{
					do
					{
#ifdef MAME_SGI_RB2_XY_ADDRESS
						set_address_xy(start_x + m_rex3.m_x_window - 0x1000, start_y + m_rex3.m_y_window - 0x1000);
#else
						set_address((start_y + m_rex3.m_y_window - 0x1000) * (1280 + 64) + (start_x + m_rex3.m_x_window - 0x1000));
#endif
						uint32_t src = read_pixel();

						output_pixel(start_x + m_rex3.m_x_move, start_y + m_rex3.m_y_move, src);

						start_x += dx;
					} while (start_x != end_x && stop_on_x);

					if (start_x == end_x)
					{
						start_x = m_rex3.m_x_save;
						start_y += dy;
					}
				} while (start_y != end_y && stop_on_y);

				write_x_start(start_x << 11);
				write_y_start(start_y << 11);
			}
			break;
	}
}

void newport_base_device::write_x_start(int32_t val)
{
	m_rex3.m_x_start = val & 0x07ffff80;
	m_rex3.m_x_start_frac = (val >> 7) & 0xf;
	m_rex3.m_x_start_i = (int16_t)(val >> 11);
	m_rex3.m_x_start_f = (uint32_t)val & 0x007fff80;
	m_rex3.m_xy_start_i = (m_rex3.m_xy_start_i & 0x0000ffff) | (m_rex3.m_x_start_i << 16);
}

void newport_base_device::write_y_start(int32_t val)
{
	m_rex3.m_y_start = val & 0x07ffff80;
	m_rex3.m_y_start_frac = (val >> 7) & 0xf;
	m_rex3.m_y_start_i = (int16_t)(val >> 11);
	m_rex3.m_y_start_f = (uint32_t)val & 0x007fff80;
	m_rex3.m_xy_start_i = (m_rex3.m_xy_start_i & 0xffff0000) | (uint16_t)m_rex3.m_y_start_i;
}

void newport_base_device::write_x_end(int32_t val)
{
	m_rex3.m_x_end = val & 0x07ffff80;
	m_rex3.m_x_end_frac = (val >> 7) & 0xf;
	m_rex3.m_x_end_i = (int16_t)(val >> 11);
	m_rex3.m_x_end_f = (uint32_t)val & 0x007fff80;
	m_rex3.m_xy_end_i = (m_rex3.m_xy_end_i & 0x0000ffff) | (m_rex3.m_x_end_i << 16);
}

void newport_base_device::write_y_end(int32_t val)
{
	m_rex3.m_y_end = val & 0x07ffff80;
	m_rex3.m_y_end_frac = (val >> 7) & 0xf;
	m_rex3.m_y_end_i = (int16_t)(val >> 11);
	m_rex3.m_y_end_f = (uint32_t)val & 0x007fff80;
	m_rex3.m_xy_end_i = (m_rex3.m_xy_end_i & 0xffff0000) | (uint16_t)m_rex3.m_y_end_i;
}

void newport_base_device::rex3_w(offs_t offset, uint64_t data, uint64_t mem_mask)
{
	// No dirty-marking needed here directly anymore - it now happens
	// precisely at the actual point of change (rb2_device::store_pixel()
	// for framebuffer writes, vc2_device::write()'s mark_cursor_dirty()
	// for cursor state), both of which this function ends up calling
	// into for whichever sub-path a given write actually takes.
#if ENABLE_NEWVIEW_LOG
	if (m_newview_log != nullptr)
	{
		uint32_t offset_lo = (uint32_t)offset;
		uint32_t data_hi = (uint32_t)(data >> 32);
		uint32_t data_lo = (uint32_t)data;
		uint32_t mem_mask_hi = (uint32_t)(mem_mask >> 32);
		uint32_t mem_mask_lo = (uint32_t)mem_mask;

		fwrite(&offset_lo, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&data_hi, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&data_lo, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&mem_mask_hi, sizeof(uint32_t), 1, m_newview_log);
		fwrite(&mem_mask_lo, sizeof(uint32_t), 1, m_newview_log);
	}
#endif

	switch (offset & ~(0x800/8))
	{
	case 0x0000/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 Draw Mode 1 Write: %08x\n", data32);
			switch (data32 & 7)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Planes Enabled:     None\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Planes Enabled:     R/W RGB/CI\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Planes Enabled:     R/W RGBA\n");
				break;
			case 0x04:
				LOGMASKED(LOG_REX3, "    Planes Enabled:     R/W OLAY\n");
				break;
			case 0x05:
				LOGMASKED(LOG_REX3, "    Planes Enabled:     R/W PUP\n");
				break;
			case 0x06:
				LOGMASKED(LOG_REX3, "    Planes Enabled:     R/W CID\n");
				break;
			default:
				LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "    Unknown Plane Enable Value\n");
				break;
			}
			switch ((data32 >> 3) & 3)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Plane Draw Depth:    4 bits\n");
				m_rex3.m_write_width = 0x0000000f;
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Plane Draw Depth:    8 bits\n");
				m_rex3.m_write_width = 0x000000ff;
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Plane Draw Depth:   12 bits\n");
				m_rex3.m_write_width = 0x000000ff; // TODO: 24-bit
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Plane Draw Depth:   32 bits\n");
				m_rex3.m_write_width = 0x000000ff; // TODO: 24-bit
				break;
			}
			LOGMASKED(LOG_REX3, "    DBuf Source Buffer: %d\n", BIT(data32, 5));
			LOGMASKED(LOG_REX3, "    GL Y Coordinates:   %d\n", BIT(data32, 6));
			LOGMASKED(LOG_REX3, "    Enable Pxl Packing: %d\n", BIT(data32, 7));
			switch ((data32 & 0x00000300) >> 8)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    HOSTRW Depth:        4 bits\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    HOSTRW Depth:        8 bits\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    HOSTRW Depth:       12 bits\n");
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    HOSTRW Depth:       32 bits\n");
				break;
			}
			LOGMASKED(LOG_REX3, "    DWord Transfers:    %d\n", BIT(data32, 10));
			LOGMASKED(LOG_REX3, "    Swap Endianness:    %d\n", BIT(data32, 11));
			LOGMASKED(LOG_REX3, "    Compare Src > Dest: %d\n", BIT(data32, 12));
			LOGMASKED(LOG_REX3, "    Compare Src = Dest: %d\n", BIT(data32, 13));
			LOGMASKED(LOG_REX3, "    Compare Src < Dest: %d\n", BIT(data32, 14));
			LOGMASKED(LOG_REX3, "    RGB Mode Select:    %d\n", BIT(data32, 15));
			LOGMASKED(LOG_REX3, "    Enable Dithering:   %d\n", BIT(data32, 16));
			LOGMASKED(LOG_REX3, "    Enable Fast Clear:  %d\n", BIT(data32, 17));
			LOGMASKED(LOG_REX3, "    Enable Blending:    %d\n", BIT(data32, 18));
			switch ((data32 >> 19) & 7)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Src Blend Factor:   0\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Src Blend Factor:   1\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Src Blend Factor:   Normalized Dest Color (or COLORBACK)\n");
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Src Blend Factor:   1 - Normalized Dest Color (or COLORBACK)\n");
				break;
			case 0x04:
				LOGMASKED(LOG_REX3, "    Src Blend Factor:   Normalized Src Alpha\n");
				break;
			case 0x05:
				LOGMASKED(LOG_REX3, "    Src Blend Factor:   1 - Normalized Src Alpha\n");
				break;
			default:
				LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "    Unknown Src Blend Factor: %02x\n", (data32 >> 19) & 7);
				break;
			}
			switch ((data32 >> 22) & 7)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Dest Blend Factor:  0\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Dest Blend Factor:  1\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Dest Blend Factor:  Normalized Src Color\n");
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Dest Blend Factor:  1 - Normalized Src Color\n");
				break;
			case 0x04:
				LOGMASKED(LOG_REX3, "    Dest Blend Factor:  Normalized Src Alpha\n");
				break;
			case 0x05:
				LOGMASKED(LOG_REX3, "    Dest Blend Factor:  1 - Normalized Src Alpha\n");
				break;
			default:
				LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "    Unknown Src Blend Factor: %02x\n", (data32 & 0x00380000) >> 19);
				break;
			}
			LOGMASKED(LOG_REX3, "  COLORBACK Dest Blend: %d\n", BIT(data32, 25));
			LOGMASKED(LOG_REX3, "   Enable Pxl Prefetch: %d\n", BIT(data32, 26));
			LOGMASKED(LOG_REX3, "    SFACTOR Src Alpha:  %d\n", BIT(data32, 27));
			switch ((data32 >> 28) & 15)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   0\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Src & Dst\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Src & ~Dst\n");
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Src\n");
				break;
			case 0x04:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~Src & Dst\n");
				break;
			case 0x05:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Dst\n");
				break;
			case 0x06:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Src ^ Dst\n");
				break;
			case 0x07:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Src | Dst\n");
				break;
			case 0x08:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~(Src | Dst)\n");
				break;
			case 0x09:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~(Src ^ Dst)\n");
				break;
			case 0x0a:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~Dst\n");
				break;
			case 0x0b:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   Src | ~Dst\n");
				break;
			case 0x0c:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~Src\n");
				break;
			case 0x0d:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~Src | Dst\n");
				break;
			case 0x0e:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   ~(Src & Dst)\n");
				break;
			case 0x0f:
				LOGMASKED(LOG_REX3, "    Logical Op. Type:   1\n");
				break;
			}
			m_rex3.m_draw_mode1 = data32;

			m_rex3.m_plane_enable = m_rex3.m_draw_mode1 & 7;
			m_rex3.m_plane_depth = (m_rex3.m_draw_mode1 >> 3) & 3;
			m_rex3.m_rwpacked = BIT(m_rex3.m_draw_mode1, 7);
			m_rex3.m_hostdepth = (m_rex3.m_draw_mode1 >> 8) & 3;
			m_rex3.m_rwdouble = BIT(m_rex3.m_draw_mode1, 10);
			m_rex3.m_sfactor = (m_rex3.m_draw_mode1 >> 19) & 7;
			m_rex3.m_dfactor = (m_rex3.m_draw_mode1 >> 22) & 7;
			m_rex3.m_blend_back = BIT(m_rex3.m_draw_mode1, 25);

			m_rex3.m_host_shift = 64 - s_host_shifts[m_rex3.m_hostdepth];

			uint16_t draw_flags = BIT(data32, 18) << 12;
			draw_flags |= BIT(data32, 17) << 11;
			draw_flags |= BIT(data32, 15) << 10;
			draw_flags |= BIT(data32,  5) << 9;
			draw_flags |= (data32 & 7) << 6;
			draw_flags |= (data32 & 0x18) << 1;
			draw_flags |= (data32 >> 28) & 0xf;
			draw_flags_w(draw_flags);
		}
		if (ACCESSING_BITS_0_31)
		{
			const uint32_t data32 = (uint32_t)data;
			LOGMASKED(LOG_REX3, "REX3 Draw Mode 0 Write: %08x\n", data32);
			switch (data32 & 3)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Primitive Function: No Op\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Primitive Function: Read From FB\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Primitive Function: Draw To FB\n");
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Primitive Function: Copy FB To FB\n");
				break;
			}
			switch ((data32 & 0x0000001c) >> 2)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Addressing Mode: Span/Point\n");
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Addressing Mode: Block\n");
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Addressing Mode: Bresenham Line, Integer Endpoints\n");
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Addressing Mode: Bresenham Line, Fractional Endpoints\n");
				break;
			case 0x04:
				LOGMASKED(LOG_REX3, "    Addressing Mode: AA Bresenham Line\n");
				break;
			default:
				LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "    Unknown Addressing Mode: %02x\n", (data32 & 0x0000001c) >> 2);
				break;
			}
			LOGMASKED(LOG_REX3, "    Iterator Setup:     %d\n", BIT(data32, 5));
			LOGMASKED(LOG_REX3, "    RGB/CI Draw Source: %d\n", BIT(data32, 6));
			LOGMASKED(LOG_REX3, "     Alpha Draw Source: %d\n", BIT(data32, 7));
			LOGMASKED(LOG_REX3, "    Stop On X:          %d\n", BIT(data32, 8));
			LOGMASKED(LOG_REX3, "    Stop On Y:          %d\n", BIT(data32, 9));
			LOGMASKED(LOG_REX3, "    Skip Start Point:   %d\n", BIT(data32, 10));
			LOGMASKED(LOG_REX3, "    Skip End Point:     %d\n", BIT(data32, 11));
			LOGMASKED(LOG_REX3, "    Enable Patterning:  %d\n", BIT(data32, 12));
			LOGMASKED(LOG_REX3, "    Enable Stippling:   %d\n", BIT(data32, 13));
			LOGMASKED(LOG_REX3, "    Stipple Advance:    %d\n", BIT(data32, 14));
			LOGMASKED(LOG_REX3, "    Limit Draw To 32px: %d\n", BIT(data32, 15));
			LOGMASKED(LOG_REX3, "     Z Opaque Stipple   %d\n", BIT(data32, 16));
			LOGMASKED(LOG_REX3, "    LS Opaque Stipple:  %d\n", BIT(data32, 17));
			LOGMASKED(LOG_REX3, "    Enable Lin. Shade:  %d\n", BIT(data32, 18));
			LOGMASKED(LOG_REX3, "    Left-Right Only:    %d\n", BIT(data32, 19));
			LOGMASKED(LOG_REX3, "    Offset by XYMove:   %d\n", BIT(data32, 20));
			LOGMASKED(LOG_REX3, "    Enable CI Clamping: %d\n", BIT(data32, 21));
			LOGMASKED(LOG_REX3, "    Enable End Filter:  %d\n", BIT(data32, 22));
			LOGMASKED(LOG_REX3, "    Enable Y+2 Stride:  %d\n", BIT(data32, 23));
			m_rex3.m_draw_mode0 = data32;

			m_rex3.m_color_host = BIT(m_rex3.m_draw_mode0, 6);
			m_rex3.m_alpha_host = BIT(m_rex3.m_draw_mode0, 7);
		}
		break;
	case 0x0008/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Line Stipple Mode Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_ls_mode = (uint32_t)(data >> 32) & 0xfffffff;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Line Stipple Pattern Write: %08x\n", (uint32_t)data);
			m_rex3.m_ls_pattern = (uint32_t)data;
		}
		break;
	case 0x0010/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Line Stipple Pattern (Save) Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_ls_pattern_saved = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Pattern Register Write: %08x\n", (uint32_t)data);
			m_rex3.m_z_pattern = (uint32_t)data;
		}
		break;
	case 0x0018/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Opaque Pattern / Blendfunc Dest Color Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_color_back = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 VRAM Fastclear Color Write: %08x\n", (uint32_t)data);
			m_rex3.m_color_vram = (uint32_t)data;
		}
		break;
	case 0x0020/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 AFUNCTION Reference Alpha Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_alpha_ref = (uint8_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Stall GFIFO Write: %08x\n", data);
			break;
		}
		break;
	case 0x0028/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 0 X Min/Max Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_smask_x[0] = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 0 Y Min/Max Write: %08x\n", (uint32_t)data);
			m_rex3.m_smask_y[0] = (uint32_t)data;
		}
		break;
	case 0x0030/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Line/Span Setup Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_setup = (uint32_t)(data >> 32);
			do_setup();
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 ZPattern Enable Write\n");
			m_rex3.m_step_z = 1;
		}
		break;
	case 0x0038/8:
		// MAME_SGI_LS_SAVE_COUNT_FIX: LSRESTORE and LSSAVE move the repeat
		// down counter too (LSMODE LSRCOUNT, bits 7-0, and its copy
		// LSRCNTSAVE, bits 23-16), so each stroke of a wide stippled line
		// starts the pattern at the same point
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Update LSPATTERN/LSRCOUNT\n");
			m_rex3.m_ls_pattern = m_rex3.m_ls_pattern_saved;
			m_rex3.m_ls_mode = (m_rex3.m_ls_mode & ~0xffu) | ((m_rex3.m_ls_mode >> 16) & 0xff);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Update LSPATSAVE/LSRCNTSAVE\n");
			m_rex3.m_ls_pattern_saved = m_rex3.m_ls_pattern;
			m_rex3.m_ls_mode = (m_rex3.m_ls_mode & ~0xff0000u) | ((m_rex3.m_ls_mode & 0xff) << 16);
		}
		break;
	case 0x0100/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XStart Write: %08x\n", (uint32_t)(data >> 32));
			write_x_start((int32_t)(data >> 32));
			m_rex3.m_x_save = m_rex3.m_x_start_i;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 YStart Write: %08x\n", (uint32_t)data);
			write_y_start((int32_t)data);
		}
		break;
	case 0x0108/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XEnd Write: %08x\n", (uint32_t)(data >> 32));
			write_x_end((int32_t)(data >> 32));
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 YEnd Write: %08x\n", (uint32_t)data);
			write_y_end((int32_t)data);
		}
		break;
	case 0x0110/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XSave Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_x_save = (int16_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 XYMove Write: %08x\n", (uint32_t)data);
			m_rex3.m_xy_move = (uint32_t)data;
			m_rex3.m_x_move = (int16_t)(m_rex3.m_xy_move >> 16);
			m_rex3.m_y_move = (int16_t)m_rex3.m_xy_move;
		}
		break;
	case 0x0118/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham D Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_bres_d = (uint32_t)(data >> 32) & 0x7ffffff;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham S1 Write: %08x\n", (uint32_t)data);
			m_rex3.m_bres_s1 = (uint32_t)data & 0x1ffff;
		}
		break;
	case 0x0120/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham Octant & Incr1 Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_bres_octant_inc1 = (uint32_t)(data >> 32) & 0x70fffff;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham Octant Rounding Mode & Incr2 Write: %08x\n", (uint32_t)data);
			m_rex3.m_bres_round_inc2 = data & 0xff1fffff;
		}
		break;
	case 0x0128/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham E1 Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_bres_e1 = (uint16_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Bresenham S2 Write: %08x\n", (uint32_t)data);
			m_rex3.m_bres_s2 = (uint32_t)data & 0x3ffffff;
		}
		break;
	case 0x0130/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 AA Line Weight Table 1/2 Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_a_weight0 = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 AA Line Weight Table 2/2 Write: %08x\n", (uint32_t)data);
			m_rex3.m_a_weight1 = (uint32_t)data;
		}
		break;
	case 0x0138/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 GL XStart Write: %08x\n", (uint32_t)(data >> 32));
			write_x_start((int32_t)(data >> 32) & 0x007fff80);
			m_rex3.m_x_save = m_rex3.m_x_start_i;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 GL YStart Write: %08x\n", (uint32_t)data);
			write_y_start((int32_t)data & 0x007fff80);
		}
		break;
	case 0x0140/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 GL XEnd Write: %08x\n", (uint32_t)(data >> 32));
			write_x_end((int32_t)(data >> 32) & 0x007fff80);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 GL YEnd Write: %08x\n", (uint32_t)data);
			write_y_end((int32_t)data & 0x007fff80);
		}
		break;
	case 0x0148/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 XStart (integer) Write: %08x\n", (uint32_t)(data >> 32));
			write_x_start(((int32_t)(int16_t)(data >> 32)) << 11);
			m_rex3.m_x_save = m_rex3.m_x_start_i;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 GL XEnd (copy) Write: %08x\n", (uint32_t)data);
			write_x_end((int32_t)data & 0x007fff80);
		}
		break;
	case 0x0150/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 XYStart (integer) Write: %08x\n", data32);
			write_x_start(((int32_t)(int16_t)(data >> 48)) << 11);
			write_y_start(((int32_t)(int16_t)(data >> 32)) << 11);
			m_rex3.m_x_save = m_rex3.m_x_start_i;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 XYEnd (integer) Write: %08x\n", (uint32_t)data);
			write_x_end(((int32_t)(int16_t)(data >> 16)) << 11);
			write_y_end(((int32_t)(int16_t)(data >>  0)) << 11);
		}
		break;
	case 0x0158/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 XStartEnd (integer) Write: %08x\n", data32);
			write_x_start(((int32_t)(int16_t)(data >> 48)) << 11);
			write_x_end(((int32_t)(int16_t)(data >> 32)) << 11);
			m_rex3.m_x_save = m_rex3.m_x_start_i;
		}
		break;
	case 0x0200/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Red/CI Full State Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_color_red = (int32_t)((data >> 32) & 0xffffff);
			m_rex3.m_curr_color_red = m_rex3.m_color_red;
			if (!BIT(m_rex3.m_draw_mode1, 15))
			{
				switch (m_rex3.m_plane_depth)
				{
				case 0: // 4bpp
					m_rex3.m_color_i = (uint32_t)((data >> 43) & 0xf);
					break;
				case 1: // 8bpp
					m_rex3.m_color_i = (uint32_t)((data >> 43) & 0xff);
					break;
				case 2: // 12bpp
					m_rex3.m_color_i = (uint32_t)((data >> 41) & 0xfff);
					break;
				case 3: // 32bpp
					// Invalid for CI mode
					break;
				}
			}
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Alpha Full State Write: %08x\n", (uint32_t)data);
			m_rex3.m_color_alpha = (int32_t)(data & 0xfffff);
			m_rex3.m_curr_color_alpha = m_rex3.m_color_alpha;
		}
		break;
	case 0x0208/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Green Full State Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_color_green = (int32_t)((data >> 32) & 0xfffff);
			m_rex3.m_curr_color_green = m_rex3.m_color_green;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Blue Full State Write: %08x\n", (uint32_t)data);
			m_rex3.m_color_blue = (int32_t)(data & 0xfffff);
			m_rex3.m_curr_color_blue = m_rex3.m_color_blue;
		}
		break;
	case 0x0210/8:
	{
		if (ACCESSING_BITS_32_63)
		{
			uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 Red/CI Slope Write: %08x\n", data32);
			data32 &= 0x807fffff;
			int32_t temp = 0;
			if (BIT(data32, 31))
			{
				temp  = 0x00800000 - (data32 & 0x7fffff);
				temp |= 0x00800000;
			}
			else
			{
				temp = data32 & 0x7fffff;
			}
			m_rex3.m_slope_red = temp;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Alpha Slope Write: %08x\n", (uint32_t)data);
			data &= 0x8007ffff;
			int32_t temp = 0;
			if (BIT(data, 31))
			{
				temp  = 0x00080000 - (data & 0x7ffff);
				temp |= 0x00080000;
			}
			else
			{
				temp = data & 0x7ffff;
			}
			m_rex3.m_slope_alpha = temp;
		}
		break;
	}
	case 0x0218/8:
	{
		if (ACCESSING_BITS_32_63)
		{
			uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 Green Slope Write: %08x\n", data32);
			data32 &= 0x8007ffff;
			int32_t temp = 0;
			if (BIT(data32, 31))
			{
				temp  = 0x00080000 - (data32 & 0x7ffff);
				temp |= 0x00080000;
			}
			else
			{
				temp = data32 & 0x7ffff;
			}
			m_rex3.m_slope_green = temp;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Blue Slope Write: %08x\n", (uint32_t)data);
			data &= 0x8007ffff;
			int32_t temp = 0;
			if (BIT(data, 31))
			{
				temp  = 0x00080000 - (data & 0x7ffff);
				temp |= 0x00080000;
			}
			else
			{
				temp = data & 0x7ffff;
			}
			m_rex3.m_slope_blue = temp;
		}
		break;
	}
	case 0x0220/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 Write Mask Write: %08x\n", data32);
			m_rex3.m_write_mask = data32 & 0xffffff;
			write_mask_w(m_rex3.m_write_mask);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Packed Color Write: %08x\n", (uint32_t)data);
			m_rex3.m_color_i = (uint32_t)data;
			if (BIT(m_rex3.m_draw_mode1, 15))
			{
				m_rex3.m_color_red = (data & 0xff) << 11;
				m_rex3.m_color_green = (data & 0xff00) << 3;
				m_rex3.m_color_blue = (data & 0xff0000) >> 5;

				m_rex3.m_curr_color_red = m_rex3.m_color_red;
				m_rex3.m_curr_color_green = m_rex3.m_color_green;
				m_rex3.m_curr_color_blue = m_rex3.m_color_blue;
			}
		}
		break;
	case 0x0228/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Color Index Zeros Overflow Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_zero_overflow = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Red/CI Slope (copy) Write: %08x\n", (uint32_t)data);
			m_rex3.m_slope_red = (uint32_t)data;
		}
		break;
	case 0x0230/8:
		LOGMASKED(LOG_REX3, "REX3 Host Data Port Write: %08x%08x & %08x%08x\n", (uint32_t)(data >> 32), (uint32_t)data, (uint64_t)(mem_mask >> 32), (uint32_t)mem_mask);
		COMBINE_DATA(&m_rex3.m_host_dataport);
		m_rex3.m_host_shift = 64 - s_host_shifts[m_rex3.m_hostdepth];
#ifdef MAME_SGI_REX3_HOSTDATA_DIAG
		if (FILE *f = rex3_hostdata_diag_file(machine()))
			fprintf(f, "W t=%.6f ctx=%s off=%x data=%016llx mask=%016llx dp=%016llx pk=%d dbl=%d hd=%d x=%d y=%d go=%d\n",
					machine().time().as_double(), machine().describe_context().c_str(), offset, (unsigned long long)data, (unsigned long long)mem_mask,
					(unsigned long long)m_rex3.m_host_dataport, m_rex3.m_rwpacked, m_rex3.m_rwdouble, m_rex3.m_hostdepth,
					m_rex3.m_x_start_i, m_rex3.m_y_start_i, (offset & 0x100) ? 1 : 0);
#endif
		break;
	case 0x0238/8:
		if (ACCESSING_BITS_32_63)
		{
			data >>= 32;
			LOGMASKED(LOG_REX3, "REX3 Display Control Bus Mode Write: %08x\n", (uint32_t)data);
			switch (data & 3)
			{
			case 0x00:
				LOGMASKED(LOG_REX3, "    Transfer Width:     4 bytes\n");
				m_rex3.m_dcb_mask = 0xffffffff;
				break;
			case 0x01:
				LOGMASKED(LOG_REX3, "    Transfer Width:     1 bytes\n");
				m_rex3.m_dcb_mask = 0x000000ff;
				break;
			case 0x02:
				LOGMASKED(LOG_REX3, "    Transfer Width:     2 bytes\n");
				m_rex3.m_dcb_mask = 0x0000ffff;
				break;
			case 0x03:
				LOGMASKED(LOG_REX3, "    Transfer Width:     3 bytes\n");
				m_rex3.m_dcb_mask = 0xffffffff;
				break;
			}
			LOGMASKED(LOG_REX3, "   Enable Data Packing: %d\n", BIT(data, 2));
			LOGMASKED(LOG_REX3, " Enable Auto-Increment: %d\n", BIT(data, 3));
			LOGMASKED(LOG_REX3, "    DCB Reg Select Adr: %d\n", (data & 0x00000070 ) >> 4);
			LOGMASKED(LOG_REX3, "     DCB Slave Address: %d\n", (data & 0x00000780 ) >> 7);
			LOGMASKED(LOG_REX3, "    Use Sync XFer ACK:  %d\n", (data & 0x00000800 ) >> 11);
			LOGMASKED(LOG_REX3, "    Use Async XFer ACK: %d\n", (data & 0x00001000 ) >> 12);
			LOGMASKED(LOG_REX3, "   GIO CLK Cycle Width: %d\n", (data & 0x0003e000 ) >> 13);
			LOGMASKED(LOG_REX3, "    GIO CLK Cycle Hold: %d\n", (data & 0x007c0000 ) >> 18);
			LOGMASKED(LOG_REX3, "   GIO CLK Cycle Setup: %d\n", (data & 0x0f800000 ) >> 23);
			LOGMASKED(LOG_REX3, "    Swap Byte Ordering: %d\n", (data & 0x10000000 ) >> 28);
			m_rex3.m_dcb_reg_select = (data & 0x00000070) >> 4;
			m_rex3.m_dcb_slave_select = (data & 0x00000780) >> 7;
			m_rex3.m_dcb_mode = data & 0x1fffffff;
		}
		break;
	case 0x0240/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			m_rex3.m_dcb_data_msw = data32;
			switch (m_rex3.m_dcb_slave_select)
			{
			case DCB_ADDR_VC2:
				m_vc2->write(m_rex3.m_dcb_reg_select, data32, m_rex3.m_dcb_mask);
				// A RAM word write (mem_mask 0x0000ffff, reg-select 3) can
				// modify the DID/VT tables vc2_device's own RAM also holds
				// (see that write() case's own comment) - unlike a direct
				// rb2 pixel write, its dirty marking only fires when the
				// cursor happens to be enabled, and even then only covers
				// the cursor's own small box, never the real display rows
				// a DID/VT change affects.
				if (m_rex3.m_dcb_mask == 0x0000ffff && m_rex3.m_dcb_reg_select == 3)
					mark_full_dirty();
				break;
			case DCB_ADDR_CMAP01:
				m_cmap[0]->write(m_rex3.m_dcb_reg_select, data32);
				m_cmap[1]->write(m_rex3.m_dcb_reg_select, data32);
				// MAME_SGI_CMAP_DIRTY_FIX: a palette entry can be on screen
				// anywhere: redraw it all
				if (m_rex3.m_dcb_reg_select == 2)
					mark_full_dirty();
				break;
			case DCB_ADDR_CMAP0:
				m_cmap[0]->write(m_rex3.m_dcb_reg_select, data32);
				if (m_rex3.m_dcb_reg_select == 2)	// MAME_SGI_CMAP_DIRTY_FIX
					mark_full_dirty();
				break;
			case DCB_ADDR_CMAP1:
				m_cmap[1]->write(m_rex3.m_dcb_reg_select, data32);
				break;
			case DCB_ADDR_XMAP01:
				m_xmap[0]->write(m_rex3.m_dcb_reg_select, data32);
				m_xmap[1]->write(m_rex3.m_dcb_reg_select, data32);
				// Mode Register Write (reg-select 5) changes m_mode_table[],
				// which screen_update()'s pixel loop reads to decide how to
				// interpret/display already-resident VRAM data - xmap9_device
				// itself does zero dirty-rect marking for this. Slot-precise,
				// not whole-screen - see mark_dirty_for_xmap_slot()'s own
				// comment.
				if (m_rex3.m_dcb_reg_select == 5)
#ifdef MAME_SGI_XMAP_DIRTY_BASELINE_MEASUREMENT
					mark_full_dirty();
#else
					mark_dirty_for_xmap_slot((uint8_t)(data32 >> 24));
#endif
				break;
			case DCB_ADDR_XMAP0:
				m_xmap[0]->write(m_rex3.m_dcb_reg_select, data32);
				if (m_rex3.m_dcb_reg_select == 5)
#ifdef MAME_SGI_XMAP_DIRTY_BASELINE_MEASUREMENT
					mark_full_dirty();
#else
					mark_dirty_for_xmap_slot((uint8_t)(data32 >> 24));
#endif
				break;
			case DCB_ADDR_XMAP1:
				m_xmap[1]->write(m_rex3.m_dcb_reg_select, data32);
				if (m_rex3.m_dcb_reg_select == 5)
#ifdef MAME_SGI_XMAP_DIRTY_BASELINE_MEASUREMENT
					mark_full_dirty();
#else
					mark_dirty_for_xmap_slot((uint8_t)(data32 >> 24));
#endif
				break;
			case DCB_ADDR_RAMDAC:
				ramdac_write(data32);
				break;
			case DCB_ADDR_CC1:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Write to CC1 (not yet implemented): %08x\n", data32);
				break;
			case DCB_ADDR_AB1:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Write to AB1 (not yet implemented): %08x\n", data32);
				break;
			case DCB_ADDR_PCD:
				LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data MSW Write to PCD (not yet implemented): %08x\n", data32);
				// Presenter not connected; simulate a bus timeout
				m_rex3.m_status |= STATUS_BACKBUSY;
				m_dcb_timeout_timer->adjust(attotime::from_msec(1));
				break;
			default:
				LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "REX3 Display Control Bus Data MSW Write: %08x\n", data32);
				break;
			}
			if (BIT(m_rex3.m_dcb_mode, 3))
			{
				m_rex3.m_dcb_reg_select++;
				m_rex3.m_dcb_reg_select &= 7;
			}
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Display Control Bus Data LSW Write: %08x\n", (uint32_t)data);
			m_rex3.m_dcb_data_lsw = (uint32_t)data;
		}
		break;
	case 0x1300/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 1 X Min/Max Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_smask_x[1] = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 1 Y Min/Max Write: %08x\n", (uint32_t)data);
			m_rex3.m_smask_y[1] = (uint32_t)data;
		}
		break;
	case 0x1308/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 2 X Min/Max Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_smask_x[2] = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 2 Y Min/Max Write: %08x\n", (uint32_t)data);
			m_rex3.m_smask_y[2] = (uint32_t)data;
		}
		break;
	case 0x1310/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 3 X Min/Max Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_smask_x[3] = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 3 Y Min/Max Write: %08x\n", (uint32_t)data);
			m_rex3.m_smask_y[3] = (uint32_t)data;
		}
		break;
	case 0x1318/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 4 X Min/Max Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_smask_x[4] = (uint32_t)(data >> 32);
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 Screenmask 4 Y Min/Max Write: %08x\n", (uint32_t)data);
			m_rex3.m_smask_y[4] = (uint32_t)data;
		}
		break;
	case 0x1320/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 Top of Screen Scanline Write: %08x\n", data32);
#ifdef MAME_SGI_LOWRES_BOOT_DIAG
			if (m_rex3.m_top_scanline != (data32 & 0x3ff))
			{
				osd_printf_info(
						"MAME_SGI_LOWRES_BOOT_DIAG top_scanline changed: old=%u new=%u\n",
						m_rex3.m_top_scanline, data32 & 0x3ff);
			}
#endif
			m_rex3.m_top_scanline = data32 & 0x3ff;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "REX3 XY Window Write: %08x\n", (uint32_t)data);
#ifdef MAME_SGI_LOWRES_BOOT_DIAG
			if (m_rex3.m_xy_window != (uint32_t)data)
			{
				osd_printf_info(
						"MAME_SGI_LOWRES_BOOT_DIAG xy_window changed: old=%08x new=%08x (x_window=%d y_window=%d)\n",
						m_rex3.m_xy_window, (uint32_t)data,
						(int)(int16_t)((uint32_t)data >> 16), (int)(int16_t)(uint32_t)data);
			}
#endif
			m_rex3.m_xy_window = (uint32_t)data;
			m_rex3.m_x_window = (int16_t)(m_rex3.m_xy_window >> 16);
			m_rex3.m_y_window = (int16_t)m_rex3.m_xy_window;
		}
		break;
	case 0x1328/8:
		if (ACCESSING_BITS_32_63)
		{
			const uint32_t data32 = (uint32_t)(data >> 32);
			LOGMASKED(LOG_REX3, "REX3 Clipping Mode Write: %08x\n", data32);
			m_rex3.m_clip_mode = data32 & 0x1fff;
		}
		if (ACCESSING_BITS_0_31)
		{
			LOGMASKED(LOG_REX3, "Request GFIFO Stall\n");
		}
		break;
	case 0x1330/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "REX3 Config Write: %08x\n", (uint32_t)(data >> 32));
			m_rex3.m_config = (data >> 32) & 0x1fffff;
		}
		break;
	case 0x1340/8:
		if (ACCESSING_BITS_32_63)
		{
			LOGMASKED(LOG_REX3, "Reset DCB Bus and Flush BFIFO\n");
		}
		break;
	default:
		LOGMASKED(LOG_REX3 | LOG_UNKNOWN, "Unknown REX3 Write: %08x (%08x): %08x\n", 0xbf0f0000 + (offset << 2), mem_mask, data);
		break;
	}

	if (offset & 0x00000100)
	{
		// MAME_SGI_REX3_STEPZ_FIX: a GO through STEPZ (0x0034) runs the
		// command without writing pixels, only stepping the position. IRIS
		// GL's software z-buffer draws a span a pixel at a time: colour and a
		// GO through DRAWMODE0 for a pixel that passes, a GO through STEPZ for
		// one that fails. Drawing the failed pixels (with the previous pixel's
		// colour) let textured polygons show through nearer ones.
		m_pixel_write_suppress = (offset & ~(0x800/8)) == 0x0030/8 && ACCESSING_BITS_0_31;
		do_rex3_command();
		m_pixel_write_suppress = false;
	}
}

void newport_base_device::device_add_mconfig(machine_config &config)
{
	VC2(config, m_vc2);
	m_vc2->vert_int().set(FUNC(newport_base_device::vrint_w));
	m_vc2->screen_timing_changed().set(FUNC(newport_base_device::update_screen_size));

	/* video hardware */
	SCREEN(config, m_screen, SCREEN_TYPE_RASTER);
	m_screen->set_refresh_hz(60);
	m_screen->set_vblank_time(ATTOSECONDS_IN_USEC(2500)); /* not accurate */
	//m_screen->set_size(2048, 2048);
	//m_screen->set_visarea(0, 2047, 0, 2047);
	m_screen->set_size(1280+64, 1024+64);
	m_screen->set_visarea(0, 1279, 0, 1023);
	// Real SGI Indy/Indigo2 monitors at this resolution are fixed-frequency,
	// square-pixel displays (5:4), not a 4:3 CRT - without this,
	// screen_device::physical_aspect() falls back to assuming a "standard
	// CRT" 4:3 aspect for any unconfigured SCREEN_TYPE_RASTER, and MAME's
	// default -keepaspect window sizing stretches the visible 1280x1024
	// area into a 4:3 box (e.g. 1376x1032) even when -resolution requests
	// an exact 1280x1024 window.
	m_screen->set_native_aspect();
	m_screen->set_screen_update(FUNC(newport_base_device::screen_update));
	m_screen->screen_vblank().set(m_vc2, FUNC(vc2_device::vblank_w));
}

void newport_base_device::device_add_mconfig(machine_config &config, uint32_t xmap_revision, uint32_t cmap_revision, uint32_t global_mask)
{
	newport_base_device::device_add_mconfig(config);

	XMAP9(config, m_xmap[0], 0, xmap_revision);
	XMAP9(config, m_xmap[1], 0, xmap_revision);
	CMAP(config, m_cmap[0], 0, cmap_revision);
	CMAP(config, m_cmap[1], 0, cmap_revision);
	RB2(config, m_rb2, 0, global_mask);
}

void gio64_xl8_device::device_add_mconfig(machine_config &config)
{
	newport_base_device::device_add_mconfig(config, 1, 0xa1, 0x000000ff);
}

void gio64_xl24_device::device_add_mconfig(machine_config &config)
{
	newport_base_device::device_add_mconfig(config, 3, 0x02, 0x00ffffff);
}
