// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/*********************************************************************\
*
*   SGI IP22/IP24 Indigo2/Indy workstation
*
*  Known Issues:
*  - The MAC address is supplied by the NVRAM, requiring the user to
*    use "setenv -f eaddr 08:00:69:xx:yy:zz" from the Indy boot PROM
*    before any IRIX installers will proceed.
*  - The Gentoo Linux live CD hangs on starting the kernel.
*
*  Memory map:
*
*  00000000 - 0007ffff      Alias for first 512kbyte of RAM
*  00080000 - 0008ffff      EISA I/O space (VINO on Indy)
*  00090000 - 0009ffff      EISA I/O space Alias (pullups on Indy)
*  000a0000 - 07ffffff      EISA Memory
*  08000000 - 17ffffff      Low System Memory
*  18000000 - 1effffff      RESERVED - Unused
*  1f000000 - 1f3fffff      GIO64 - GFX
*  1f400000 - 1f5fffff      GIO64 - EXP0
*  1f600000 - 1f9fffff      GIO64 - EXP1 - Unused
*  1fa00000 - 1fa1ffff      Memory Controller
*  1fb00000 - 1fb7ffff      HPC3 CHIP1
*  1fb80000 - 1fbfffff      HPC3 CHIP0
*  1fc00000 - 1fffffff      BIOS
*  20000000 - 2fffffff      High System Memory
*  30000000 - 7fffffff      Reserved
*  80000000 - ffffffff      EISA Memory
*
*  IP22/IP24 has 2 pieces of PC-compatible hardware: the 8042 PS/2 keyboard/mouse
*  interface and the 8254 PIT.  Both are licensed cores embedded in the IOC custom chip.
*
*  References used:
*    MipsLinux: http://www.mips-linux.org/
*      linux-2.6.6/include/newport.h
*      linux-2.6.6/include/asm-mips/sgi/gio.h
*      linux-2.6.6/include/asm-mips/sgi/mc.h
*      linux-2.6.6/include/asm-mips/sgi/hpc3.h
*    NetBSD: http://www.netbsd.org/
*    gxemul: http://gavare.se/gxemul/
*
*  Gentoo LiveCD r5 boot instructions:
*  - Specify an appropriate LiveCD image at the command line.
*  - Enter the command interpreter and type "sashARCS". Press enter and
*    it will autoboot.
*
*  IRIX boot instructions:
*  - Specify an appropriate IRIX CD image at the command line.
*  - At the menu, choose either "run diagnostics" or "install
*    system software".
*
\*********************************************************************/

#include "emu.h"

#include "hpc3.h"
#include "ioc2.h"
#include "mc.h"
#include "vino.h"

#include "bus/gio64/gio64.h"
#include "bus/nscsi/cd.h"
#include "bus/nscsi/hd.h"
#include "bus/nscsi/tape.h"
#include "bus/pc_kbd/pc_kbdc.h"
#include "cpu/mips/mips3.h"
#include "machine/ds1386.h"
#include "machine/edlc.h"
#include "machine/eepromser.h"
#include "machine/nscsi_bus.h"
#include "machine/saa7191.h"
#include "machine/wd33c9x.h"
#include "nwd33c9x.h"
#include "sound/cdda.h"

#include "emupal.h"
#include "screen.h"
#include "softlist.h"

#include "logmacro.h"

// See ip24_state::rtc_host_resync()'s own comment for the full rationale.
// Confirmed live: measured clock drift fell from 78s to 70s over a
// ~6.4-minute test and 81s to 51s over a ~20-minute test (both real net
// reductions, not plateaus), average speed unaffected in both. Also
// confirmed to correct the reversed direction under -nothrottle (guest
// gaining time instead of losing it), though doadjtime()'s bounded slew
// rate can't fully keep up with -nothrottle's own large, host-contention-
// dependent excess-speed bursts - not a concern for the throttled case
// this is actually meant for. Opt-in (kept toggleable rather than made
// unconditional), currently enabled.
#define MAME_SGI_RTC_HOST_RESYNC

// Opt-in/toggleable test switch: swaps the boot-
// disk SCSI controller from the stock WD33C93B+NSCSI_BUS+NSCSI_HARDDISK
// setup for nwd33c9x_device (src/mame/sgi/nwd33c9x.h/.cpp), the flat
// single-object reimplementation built to eliminate the per-byte cost
// measured in nscsi_full_device::step() and nscsi_bus_device::regen_ctrl()
// (measured 2026-09-20). Supports up to 8 embedded
// disks (slot index == SCSI ID, zero per-byte dispatch overhead - the
// actual performance target) and CD-ROMs via its own internal real
// NSCSI_BUS bridge (100% of MAME's stock CD-ROM emulation, unmodified -
// never the performance target, so not worth reimplementing - see
// nwd33c9x.h's header comment). Enabled by default as of 2026-09-20 (the
// validated ~2.9x real disk-throughput win over the stock device);
// comment out to A/B back against the stock device instead.
#define MAME_SGI_USE_NWD33C9X

namespace {

class ip24_state : public driver_device
{
public:
	ip24_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_maincpu(*this, "maincpu")
		, m_mem_ctrl(*this, "memctrl")
		, m_scsi_ctrl(*this, "wd33c93")
#ifdef MAME_SGI_USE_NWD33C9X
		, m_nwd33c9x_ctrl(*this, "nwd33c9x")
#endif
		, m_edlc(*this, "edlc")
		, m_enet_config(*this, "ENET")
		, m_eeprom(*this, "eeprom")
		, m_hal2(*this, "hal2")
		, m_hpc3(*this, "hpc3")
		, m_ioc2(*this, "ioc2")
		, m_rtc(*this, "rtc")
		, m_vino(*this, "vino")
		, m_dmsd(*this, "dmsd")
		, m_gio64(*this, "gio64")
		, m_gio64_gfx(*this, "gio64_gfx")
		, m_gio64_exp0(*this, "gio64_exp0")
		, m_gio64_exp1(*this, "gio64_exp1")
	{
	}

	void ip24_base(machine_config &config, uint32_t system_clock);
	void ip24(machine_config &config, uint32_t system_clock);
	void indy_5015(machine_config &config);
	void challenge_s(machine_config &config);
	void init_challenge_s();
	void indy_4613(machine_config &config);
	void indy_4610(machine_config &config);

	DECLARE_INPUT_CHANGED_MEMBER(enet_speed_changed);

protected:
	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

#ifdef MAME_SGI_RTC_HOST_RESYNC
	TIMER_CALLBACK_MEMBER(rtc_host_resync);
#endif

	template <uint32_t addr_base> uint64_t bus_error_r(offs_t offset, uint64_t mem_mask = ~0);
	template <uint32_t addr_base> void bus_error_w(offs_t offset, uint64_t data, uint64_t mem_mask = ~0);

	uint8_t volume_r(offs_t offset);
	void volume_w(offs_t offset, uint8_t data);

	void ip24_map(address_map &map) ATTR_COLD;
	void ip24_base_map(address_map &map) ATTR_COLD;
	void pio0_map(address_map &map) ATTR_COLD;
	void pio1_map(address_map &map) ATTR_COLD;
	void pio2_map(address_map &map) ATTR_COLD;
	void pio3_map(address_map &map) ATTR_COLD;
	void pio5_map(address_map &map) ATTR_COLD;
	void pio6_map(address_map &map) ATTR_COLD;

#ifdef MAME_SGI_HW_FIXES
	// Guinness IOC2's real register block is 64 registers (0x1fbd9800-
	// 0x1fbd98fc); MAME's compact addressing here divides that by 4, so
	// anything at or past offset 0x40 is off the end of the chip entirely
	// (confirmed against the real IOC2 datasheet, section 4 and 6 -
	// 0x1fbd9900 is documented as a Full House-only "Aux Register", not
	// present on Guinness/Indy at all). Real hardware bus-errors here; the
	// SGI boot code relies on that fault as part of its own hardware-probe
	// sequence, so MAME's default silent-zero fallback misleads it.
	uint8_t pio6_unmapped_r(offs_t offset);
	void pio6_unmapped_w(offs_t offset, uint8_t data);
#endif

	static void scsi_devices(device_slot_interface &device);

	required_device<mips3_device> m_maincpu;
	required_device<sgi_mc_device> m_mem_ctrl;
	// optional_device, not required_device: under MAME_SGI_USE_NWD33C9X
	// this device is never instantiated at all (nwd33c9x_device replaces
	// it entirely) - see ip24_base()'s scsi controller setup.
	optional_device<wd33c93b_device> m_scsi_ctrl;
#ifdef MAME_SGI_USE_NWD33C9X
	optional_device<nwd33c9x_device> m_nwd33c9x_ctrl;
#endif
	required_device<seeq80c03_device> m_edlc;
	required_ioport m_enet_config;
	required_device<eeprom_serial_93cxx_device> m_eeprom;
	required_device<hal2_device> m_hal2;
	required_device<hpc3_device> m_hpc3;
	required_device<ioc2_device> m_ioc2;
	required_device<ds1386_device> m_rtc;
	optional_device<vino_device> m_vino;
	bool m_vino_fitted = true;   // false for boards without video input (challenge_s)
	optional_device<saa7191_device> m_dmsd;
	optional_device<gio64_device> m_gio64;
	optional_device<gio64_slot_device> m_gio64_gfx;
	optional_device<gio64_slot_device> m_gio64_exp0;
	optional_device<gio64_slot_device> m_gio64_exp1;

	uint8_t m_volume_l = 0;
	uint8_t m_volume_r = 0;

#ifdef MAME_SGI_RTC_HOST_RESYNC
	emu_timer *m_rtc_host_resync_timer = nullptr;
#endif
};

class ip22_state : public ip24_state
{
public:
	ip22_state(const machine_config &mconfig, device_type type, const char *tag)
		: ip24_state(mconfig, type, tag)
		, m_scsi_ctrl2(*this, "wd33c93_2")
	{
	}

	void indigo2_4415(machine_config &config);

private:
	uint32_t eisa_io_r();

	void ip22_map(address_map &map) ATTR_COLD;
	void pio4_map(address_map &map) ATTR_COLD;
	void pio6_map(address_map &map) ATTR_COLD;

	required_device<wd33c93b_device> m_scsi_ctrl2;
};

template <uint32_t addr_base>
uint64_t ip24_state::bus_error_r(offs_t offset, uint64_t mem_mask)
{
	logerror("Bus error (read)\n");
	// FIXME: m_maincpu->bus_error();
	m_mem_ctrl->set_cpu_buserr(addr_base + (offset << 3), mem_mask);
	return 0;
}

template <uint32_t addr_base>
void ip24_state::bus_error_w(offs_t offset, uint64_t data, uint64_t mem_mask)
{
	logerror("Bus error (write)\n");
	// FIXME: m_maincpu->bus_error();
	m_mem_ctrl->set_cpu_buserr(addr_base + (offset << 3), mem_mask);
}

uint32_t ip22_state::eisa_io_r()
{
	return 0xffffffff;
}

uint8_t ip24_state::volume_r(offs_t offset)
{
	if (offset == 0)
		return m_volume_r;
	else
		return m_volume_l;
}

void ip24_state::volume_w(offs_t offset, uint8_t data)
{
	if (offset == 0)
	{
		m_volume_r = data;
		m_hal2->set_right_volume(data);
	}
	else
	{
		m_volume_l = data;
		m_hal2->set_left_volume(data);
	}
}

void ip24_state::ip24_base_map(address_map &map)
{
	map(0x1f000000, 0x1f9fffff).rw(m_gio64, FUNC(gio64_device::read), FUNC(gio64_device::write));
	map(0x1fa00000, 0x1fa1ffff).rw(m_mem_ctrl, FUNC(sgi_mc_device::read), FUNC(sgi_mc_device::write));
	map(0x1fb00000, 0x1fb7ffff).rw(FUNC(ip24_state::bus_error_r<0x1fb00000>), FUNC(ip24_state::bus_error_w<0x1fb00000>));
	map(0x1fb80000, 0x1fbfffff).m(m_hpc3, FUNC(hpc3_device::map));
	map(0x1fc00000, 0x1fc7ffff).rom().region("user1", 0);
}

void ip24_state::ip24_map(address_map &map)
{
	ip24_base_map(map);
	// decided at config time: device finders are not resolved yet when
	// address maps are built, so m_vino.found() would always be false here
	if (m_vino_fitted)
		map(0x00080000, 0x0009ffff).rw(m_vino, FUNC(vino_device::read), FUNC(vino_device::write));
	else
		map(0x00080000, 0x0009ffff).rw(FUNC(ip24_state::bus_error_r<0x00080000>), FUNC(ip24_state::bus_error_w<0x00080000>));
}

void ip24_state::pio0_map(address_map &map)
{
	map(0x00, 0xff).rw(m_hal2, FUNC(hal2_device::read), FUNC(hal2_device::write));
}

void ip24_state::pio1_map(address_map &map)
{
	// MAME_SGI_HAL2_AES: the AES receiver and transmitter chips
	map(0x00, 0xff).rw(m_hal2, FUNC(hal2_device::aes_r), FUNC(hal2_device::aes_w));
}

void ip24_state::pio2_map(address_map &map)
{
	map(0x00, 0x01).rw(FUNC(ip24_state::volume_r), FUNC(ip24_state::volume_w)).umask16(0x00ff);
}

#ifdef MAME_SGI_HW_FIXES
uint8_t ip24_state::pio6_unmapped_r(offs_t offset)
{
	logerror("%s: unmapped Guinness IOC2 register read from %02x (past end of real 64-register block)\n", machine().describe_context(), offset);
	m_mem_ctrl->set_cpu_buserr(0x1fbd9800 + (offset << 2), 0xff);
	return 0;
}

void ip24_state::pio6_unmapped_w(offs_t offset, uint8_t data)
{
	logerror("%s: unmapped Guinness IOC2 register write to %02x = %02x (past end of real 64-register block)\n", machine().describe_context(), offset, data);
	m_mem_ctrl->set_cpu_buserr(0x1fbd9800 + (offset << 2), 0xff);
}
#endif

void ip24_state::pio6_map(address_map &map)
{
	map(0x00, 0x2f).m("ioc2", FUNC(ioc2_guinness_device::map)).umask16(0x00ff);
#ifdef MAME_SGI_HW_FIXES
	map(0x30, 0xff).rw(FUNC(ip24_state::pio6_unmapped_r), FUNC(ip24_state::pio6_unmapped_w));
#endif
}

void ip22_state::ip22_map(address_map &map)
{
	ip22_state::ip24_base_map(map);
	map(0x00080000, 0x0009ffff).r(FUNC(ip22_state::eisa_io_r));
}

void ip22_state::pio4_map(address_map &map)
{
	map(0x00, 0x0f).m("ioc2", FUNC(ioc2_full_house_device::int2_map)).umask16(0x00ff);
}

void ip22_state::pio6_map(address_map &map)
{
	map(0x00, 0x1f).m("ioc2", FUNC(ioc2_full_house_device::map)).umask16(0x00ff);
}

void ip24_state::machine_start()
{
	save_item(NAME(m_volume_l));
	save_item(NAME(m_volume_r));

#ifdef MAME_SGI_RTC_HOST_RESYNC
	m_rtc_host_resync_timer = timer_alloc(FUNC(ip24_state::rtc_host_resync), this);
	m_rtc_host_resync_timer->adjust(attotime::from_seconds(15), 0, attotime::from_seconds(15));
#endif
}

#ifdef MAME_SGI_RTC_HOST_RESYNC
TIMER_CALLBACK_MEMBER(ip24_state::rtc_host_resync)
{
	// MAME's DS1386 RTC model (devices/machine/ds1386.cpp) ticks its own
	// registers off attotime (the same clock DRC compile overhead makes
	// fall behind real time during boot - measured at ~78-97 real seconds
	// of accumulated drift by the time login is reachable), not real host
	// time - so a hardware RTC read and the guest kernel's own free-running
	// software clock always agree with each other even as both drift
	// behind reality. IRIX's kernel already has a complete, working self-
	// correction mechanism for exactly this mismatch (irix/kern/ml/
	// clksupport.c's onesec_maint()/chktimedrift(), which reads the
	// hardware RTC every 32 real seconds via irix/kern/ml/IP22.c's rtodc()
	// and smoothly slews the software clock via doadjtime() - a genuine
	// adjtime(2)-style correction, never an abrupt jump - if they disagree
	// by more than 2 seconds) - it just never fires here because the RTC
	// never actually disagrees with the software clock. Periodically
	// overwriting the RTC's registers with genuine real host wallclock
	// (bypassing running_machine::current_datetime(), which is deliberately
	// anchored to elapsed emulated time rather than real time, for save-
	// state determinism) gives IRIX's own unmodified correction logic a
	// real discrepancy to notice and fix on its own. Scoped to this driver
	// via a periodic timer rather than changing ds1386_device's own ticking
	// behavior, which other MAME drivers using that chip may depend on.
	system_time live_time(::time(nullptr));
	m_rtc->set_current_time(live_time);

	// set_current_time() re-triggers device_rtc_interface::set_time()'s
	// generic "year % 100" encoding on every call, undoing the
	// year-BCD fix below (real hardware here uses years-since-1940, not
	// years-since-2000) every time this timer fires - confirmed live: the
	// guest's own `date` jumped to 1996 the first time this ran, exactly
	// matching irix/kern/ml/IP22.c's rtodc() migration heuristic
	// (year < 45 gets reinterpreted as a pre-1940-epoch value and
	// "upgraded"). Re-apply the correct encoding immediately after every
	// resync, not just once at boot.
	int const rtc_year = live_time.utc_time.year - 1940;
	if (rtc_year >= 0 && rtc_year <= 99)
	{
		uint8_t const bcd_year = uint8_t(((rtc_year / 10) % 10) << 4) | uint8_t(rtc_year % 10);
		m_rtc->set_year_bcd_direct(bcd_year);
	}
}
#endif

void ip24_state::machine_reset()
{
	m_edlc->set_unthrottled(m_enet_config->read() & 1);

	// MIPS3DRC_DISABLE_INTRABLOCK and MIPS3DRC_STRICT_VERIFY were each
	// tested here as controlled experiments for a DRC memory-corruption
	// bug (rmap_del panic / stuck LL-SC spin) - both ruled out (same hang,
	// no change either way). The actual fixes for that investigation
	// landed in mips3drc.cpp's COP0 register accessors
	// (generate_get_cop0_reg/generate_set_cop0_reg) and interrupt-check
	// codegen, unrelated to either of these options.
	//m_maincpu->mips3drc_set_options(MIPS3DRC_COMPATIBLE_OPTIONS | MIPS3DRC_CHECK_OVERFLOWS);

	// TEMPORARY diagnostic (2026-08-17): MIPS3DRC_STRICT_COP1
	// forces check_cop1_access() to actually raise a BADCOP exception when
	// SR.CU1 is clear, instead of being a no-op (mips3drc.cpp:6951-6957) -
	// the interpreter has no equivalent check at all (grepped mips3.cpp for
	// SR_CU1/cop1_unusable, zero hits). Testing whether real IRIX's lazy
	// FPU context-switch convention (kernel clears SR.CU1 on a context
	// switch instead of immediately saving FP registers, expecting a
	// COP1-unusable trap to trigger the real restore on next FP use) is
	// silently broken under this emulation with the trap unenforced -
	// xwsh's mainLoop loads $f20=1000.0 correctly (confirmed via live gdb,
	// source constant and the load's native codegen both verified correct)
	// but reads back 0.0 after a sighold() syscall sits in between, exactly
	// the kind of point a real kernel context switch could occur.
	// Bisection (2026-09-02): both MIPS3DRC_STRICT_VERIFY
	// (26/100 vs 29/100 baseline) and MIPS3DRC_DISABLE_INTRABLOCK
	// (20/100) made no measurable difference, each tested alone - both
	// within this project's own established run-to-run noise band.
	// Reverted to the normal STRICT_COP1-only state (a real, needed fix -
	// see the comment above); bisection continues in mips3drc.cpp against
	// the three local dispatch/validation patches instead.
	m_maincpu->mips3drc_set_options(MIPS3DRC_STRICT_COP1);

	// TEMPORARY diagnostic (2026-07-28): enable MAME's own built-in profiler
	// (normally only toggled via the in-emulator IPT_UI_SHOW_PROFILER hotkey,
	// rendered on-screen) from process start, so the accumulated per-category
	// breakdown (PROFILER_DRC_COMPILE, PROFILER_VIDEO, PROFILER_MEMREAD, etc.)
	// can be captured as real text at clean exit (see video_manager::exit()'s
	// matching addition) instead of guessed at via source-reading alone -
	// part of the "DRC still nowhere near 10x faster than -nodrc" investigation.
	g_profiler.enable(true);

	// Bug fix: correct the RTC's year register (2026-08-15).
	// Likely resolves the still-open eventmond/"kernel
	// modification time in the future" item - not yet directly
	// reconfirmed live against that specific symptom. running_machine::start()
	// unconditionally seeds every battery-backed RTC from real host time via
	// set_rtc_datetime() (machine.cpp), which runs before this hook - so by
	// the time machine_reset() fires (during soft_reset(), on every launch
	// including the path into a -state resume), the ds1386's year register
	// already holds device_rtc_interface::set_time()'s generic year % 100
	// value (the standard "last two digits of the real year" RTC
	// convention). That's wrong for this specific chip on IP22/IP24: real
	// IRIX kernel source (irix/kern/ml/IP22.c's rtodc(), irix/kern/sys/
	// i8254clock.h's DALLAS_YRREF=1940) shows the actual on-hardware
	// encoding is years-since-1940, not years-since-2000/1900. Feeding
	// year % 100 (e.g. 26 for 2026) isn't just the wrong number, it's also
	// small enough (< 45) to trip a real one-time-migration heuristic in
	// that same kernel function - intended to auto-upgrade genuinely old
	// pre-1940-epoch NVRAM content - silently "correcting" it to display as
	// 1996 instead. The right raw value is (real_year - 1940), e.g. 86 for
	// 2026: safely past that migration threshold, and directly correct once
	// IRIX adds DALLAS_YRREF back on its own read path. Uses
	// machine().base_datetime() rather than current_datetime() specifically
	// to match set_rtc_datetime()'s own reference time exactly (both derive
	// from the same m_base_time), not a live-ticking value.
	system_time base_time;
	machine().base_datetime(base_time);
	int const rtc_year = base_time.utc_time.year - 1940;
	if (rtc_year >= 0 && rtc_year <= 99)
	{
		uint8_t const bcd_year = uint8_t(((rtc_year / 10) % 10) << 4) | uint8_t(rtc_year % 10);
		m_rtc->set_year_bcd_direct(bcd_year);
	}
}

INPUT_CHANGED_MEMBER(ip24_state::enet_speed_changed)
{
	m_edlc->set_unthrottled(newval & 1);
}

static INPUT_PORTS_START( ip24 )
	PORT_START("ENET")
	PORT_CONFNAME(0x01, 0x00, "Ethernet Link Speed") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(ip24_state::enet_speed_changed), 0)
	PORT_CONFSETTING(0x00, "10 Mbit/s")
	PORT_CONFSETTING(0x01, "Unlimited")
INPUT_PORTS_END

void ip24_state::scsi_devices(device_slot_interface &device)
{
	device.option_add("cdrom", NSCSI_CDROM_SGI);
	device.option_add("harddisk", NSCSI_HARDDISK);
	device.option_add("tape", NSCSI_TAPE_SGI); // DDS-1 DAT drive (SIMH .tap image)
	//device.set_option_machine_config("cdrom", cdrom_config);
}

static DEVICE_INPUT_DEFAULTS_START(ip22_mc)
	DEVICE_INPUT_DEFAULTS("VALID", 0x0f, 0x07)
DEVICE_INPUT_DEFAULTS_END

// Guest DRAM for the ip24 (Indy) memory controller. Stock MAME already makes
// the SIMM banks selectable at runtime: mc.cpp's "SIMMS" ioport (one
// PORT_CONFNAME per bank) appears in the Machine Configuration menu for every
// bank enabled in "VALID", and the choice is saved in cfg/<system>.cfg. This
// only sets the default when the cfg has none: bank A "4x32M" = 128MB, the
// size this project's disk images were installed and validated with (the
// stock default is "4x4M" = 16MB, which IRIX 6.5 thrashes in). Banks A and B
// are visible (as in stock), so 256MB is bank B = "4x32M" in the menu. Change
// the size before a cold boot; PROM probes memory at power-on.
//
// Hardware rules learned the hard way (2026-09-27/28): use only the named
// settings (4x1M..4x32M). Each nibble's low 3 bits pick a density and bit 3
// marks dual rank, and PROM's memory test rejects unnamed raw values ("No
// usable memory found. Make sure you have a full bank (4 SIMMs)."). Banks are
// 4 identical SIMMs each and may differ from each other; 256MB (two 4x32M
// banks) is the documented maximum. DEVICE_INPUT_DEFAULTS matches one field
// by (port, mask), so each bank needs its own entry with its own mask (a
// combined 0x00ff mask is silently ignored).
//
// The host-side allocation (mc.cpp) always reserves the full 4 x 256MB
// lazily, and save states register all four banks at full size, so the size
// is purely a guest-visible setting.
static DEVICE_INPUT_DEFAULTS_START(ip24_mc)
	DEVICE_INPUT_DEFAULTS("VALID", 0x0f, 0x03)
	DEVICE_INPUT_DEFAULTS("SIMMS", 0x000f, 0x000e) // bank A, "4x32M" (dual-rank), 128MB
DEVICE_INPUT_DEFAULTS_END

void ip24_state::ip24_base(machine_config &config, uint32_t system_clock)
{
// TEMPORARY experiment (2026-09-17): tightens the scheduler's
// standing quantum ceiling below the stock 60Hz default (schedule.cpp's
// own machine().config().maximum_quantum(attotime::from_hz(60)) fallback)
// to test whether bounding worst-case interrupt-check latency at the
// scheduler level can substitute for mips3drc.cpp's compile-time
// self-loop/backward-branch heuristic (see MAME_SGI_DISABLE_CHECKINTS_
// HEURISTIC_EXPERIMENT in that file). Throwaway: never leave this defined in a committed build.
#ifdef MAME_SGI_QUANTUM_EXPERIMENT
	config.set_maximum_quantum(attotime::from_hz(MAME_SGI_QUANTUM_EXPERIMENT));
#endif
	SGI_MC(config, m_mem_ctrl, m_maincpu, m_eeprom, system_clock);
	m_mem_ctrl->int_dma_done_cb().set(m_ioc2, FUNC(ioc2_device::mc_dma_done_w));
	m_mem_ctrl->eisa_present().set_constant(1);

#ifdef MAME_SGI_USE_NWD33C9X
	// nwd33c9x_device owns its disk targets (up to 8, slot index == SCSI
	// ID - mount the boot disk at its own "disk1" sub-tag, matching this
	// project's existing ID-1 convention) and CD-ROMs (via its own
	// internal aux bus, defaulting a CD-ROM at ID 6 same as the stock
	// scsibus below) directly - see nwd33c9x.h's header comment.
	NWD33C9X(config, m_nwd33c9x_ctrl, 10000000);
	m_nwd33c9x_ctrl->irq_cb().set(m_ioc2, FUNC(ioc2_device::scsi0_int_w));
	m_nwd33c9x_ctrl->drq_cb().set(m_hpc3, FUNC(hpc3_device::scsi0_drq));
#if MAME_SGI_SCSI_BULK_DMA
	m_nwd33c9x_ctrl->set_bulk_dma_in(m_hpc3, FUNC(hpc3_device::scsi0_dma_bulk_in));
	m_nwd33c9x_ctrl->set_bulk_dma_out(m_hpc3, FUNC(hpc3_device::scsi0_dma_bulk_out));
#endif
#else
	auto &scsibus(NSCSI_BUS(config, "scsibus"));

	WD33C93B(config, m_scsi_ctrl, 10000000);
	scsibus.set_external_device(0, m_scsi_ctrl);
	m_scsi_ctrl->irq_cb().set(m_ioc2, FUNC(ioc2_device::scsi0_int_w));
	m_scsi_ctrl->drq_cb().set(m_hpc3, FUNC(hpc3_device::scsi0_drq));

	NSCSI_CONNECTOR(config, "scsibus:1", scsi_devices, "harddisk", false);
	NSCSI_CONNECTOR(config, "scsibus:2", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:3", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:4", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:5", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus:6", scsi_devices, "cdrom", false);
	NSCSI_CONNECTOR(config, "scsibus:7", scsi_devices, nullptr, false);
#endif

	// GIO64
	GIO64(config, m_gio64, m_maincpu);
	m_gio64->interrupt_cb<0>().set(m_ioc2, FUNC(ioc2_device::gio_int0_w));
	m_gio64->interrupt_cb<1>().set(m_ioc2, FUNC(ioc2_device::gio_int1_w));
	m_gio64->interrupt_cb<2>().set(m_ioc2, FUNC(ioc2_device::gio_int2_w));
	GIO64_SLOT(config, m_gio64_gfx, m_gio64, gio64_slot_device::GIO64_SLOT_GFX, gio64_cards, "xl24");
	GIO64_SLOT(config, m_gio64_exp0, m_gio64, gio64_slot_device::GIO64_SLOT_EXP0, gio64_cards, nullptr);
	GIO64_SLOT(config, m_gio64_exp1, m_gio64, gio64_slot_device::GIO64_SLOT_EXP1, gio64_cards, nullptr);

	SGI_HPC3(config, m_hpc3, m_hal2);
	m_hpc3->set_gio64_space(m_maincpu, AS_PROGRAM);
#ifdef MAME_SGI_HW_FIXES
	m_hpc3->set_mem_ctrl_tag(m_mem_ctrl);
#endif
	m_hpc3->set_addrmap(hpc3_device::AS_PIO0, &ip24_state::pio0_map);
	m_hpc3->set_addrmap(hpc3_device::AS_PIO1, &ip24_state::pio1_map);
	m_hpc3->set_addrmap(hpc3_device::AS_PIO2, &ip24_state::pio2_map);
	m_hpc3->enet_intr_out_cb().set(m_ioc2, FUNC(ioc2_device::enet_int_w));
#ifdef MAME_SGI_USE_NWD33C9X
	m_hpc3->hd_rd_cb<0>().set(m_nwd33c9x_ctrl, FUNC(nwd33c9x_device::indir_r));
	m_hpc3->hd_wr_cb<0>().set(m_nwd33c9x_ctrl, FUNC(nwd33c9x_device::indir_w));
	m_hpc3->hd_dma_rd_cb<0>().set(m_nwd33c9x_ctrl, FUNC(nwd33c9x_device::dma_r));
	m_hpc3->hd_dma_wr_cb<0>().set(m_nwd33c9x_ctrl, FUNC(nwd33c9x_device::dma_w));
	m_hpc3->hd_reset_cb<0>().set(m_nwd33c9x_ctrl, FUNC(nwd33c9x_device::reset_w));
#else
	m_hpc3->hd_rd_cb<0>().set(m_scsi_ctrl, FUNC(wd33c93b_device::indir_r));
	m_hpc3->hd_wr_cb<0>().set(m_scsi_ctrl, FUNC(wd33c93b_device::indir_w));
	m_hpc3->hd_dma_rd_cb<0>().set(m_scsi_ctrl, FUNC(wd33c93b_device::dma_r));
	m_hpc3->hd_dma_wr_cb<0>().set(m_scsi_ctrl, FUNC(wd33c93b_device::dma_w));
	m_hpc3->hd_reset_cb<0>().set(m_scsi_ctrl, FUNC(wd33c93b_device::reset_w));
#endif
	m_hpc3->bbram_rd_cb().set(m_rtc, FUNC(ds1386_device::data_r));
	m_hpc3->bbram_wr_cb().set(m_rtc, FUNC(ds1386_device::data_w));
	m_hpc3->eeprom_dati_cb().set(m_eeprom, FUNC(eeprom_serial_93cxx_device::do_read));
	m_hpc3->eeprom_dato_cb().set(m_eeprom, FUNC(eeprom_serial_93cxx_device::di_write));
	m_hpc3->eeprom_clk_cb().set(m_eeprom, FUNC(eeprom_serial_93cxx_device::clk_write));
	m_hpc3->eeprom_cs_cb().set(m_eeprom, FUNC(eeprom_serial_93cxx_device::cs_write));
	//m_hpc3->eeprom_pre_cb().set(m_eeprom, FUNC(eeprom_serial_93cxx_device::pre_write));
	m_hpc3->dma_complete_int_cb().set(m_ioc2, FUNC(ioc2_device::hpc_dma_done_w));

	SEEQ80C03(config, m_edlc);
	m_edlc->out_int_cb().set(m_hpc3, FUNC(hpc3_device::enet_intr_in_w));
	m_edlc->out_rxrdy_cb().set(m_hpc3, FUNC(hpc3_device::enet_rxrdy_w));
	m_edlc->out_txrdy_cb().set(m_hpc3, FUNC(hpc3_device::enet_txrdy_w));
	m_hpc3->set_enet(m_edlc);

	SGI_HAL2(config, m_hal2);
	EEPROM_93C56_16BIT(config, m_eeprom);

	SOFTWARE_LIST(config, "sgi_mips").set_original("sgi_mips");
	SOFTWARE_LIST(config, "sgi_mips_hdd").set_original("sgi_mips_hdd");
}

// Callers add the CPU before calling this. Devices execute in the order they
// were added, so the CPU then runs before the keyboard controller's MCU in
// every timeslice: the MCU trails the CPU instead of running ahead of it,
// and a command the CPU writes never lands in the MCU's past.
void ip24_state::ip24(machine_config &config, uint32_t system_clock)
{
	ip24_base(config, system_clock);
	m_mem_ctrl->set_input_default(DEVICE_INPUT_DEFAULTS_NAME(ip24_mc));

	m_hpc3->set_addrmap(hpc3_device::AS_PIO6, &ip24_state::pio6_map);

	SGI_IOC2_GUINNESS(config, m_ioc2, m_maincpu);

	SAA7191(config, m_dmsd);
	VINO(config, m_vino);
	m_vino->set_gio64_space(m_maincpu, AS_PROGRAM);
	m_vino->i2c_data_out().set(m_dmsd, FUNC(saa7191_device::i2c_data_w));
	m_vino->i2c_data_in().set(m_dmsd, FUNC(saa7191_device::i2c_data_r));
	m_vino->i2c_stop().set(m_dmsd, FUNC(saa7191_device::i2c_stop_w));
	m_vino->interrupt_cb().set(m_ioc2, FUNC(ioc2_device::video_int_w));

	DS1386_8K(config, m_rtc, 32768);
	// Real SGI hardware's battery-backed RTC stores UTC/GMT, not local
	// time - confirmed via real IRIX kernel source (irix/kern/ml/IP22.c's
	// rtodc(), explicitly commented "returns seconds from 1970 00:00:00
	// GMT"); IRIX applies the local timezone offset separately (TZ) only
	// when displaying/using the time, never bakes it into the hardware
	// register itself. device_rtc_interface defaults to local time
	// (m_use_utc = false, dirtc.cpp) unless a driver opts in - SGI never
	// did, so every field (hour/day/month, not just the separately-fixed
	// year encoding) was being seeded from host local time instead of
	// UTC, producing a double timezone-shifted result once IRIX's own TZ
	// conversion was applied on top.
	m_rtc->set_use_utc(true);
}

void ip24_state::indy_5015(machine_config &config)
{
	constexpr uint32_t system_clock = 50'000'000;
	R5000BE(config, m_maincpu, 3 * system_clock);
	m_maincpu->set_system_clock(system_clock);
	m_maincpu->set_icache_size(0x8000);
	m_maincpu->set_drc_hash32(true);   // MAME_SGI_HASH32: 32-bit DRC hash table entries
	m_maincpu->set_drc_asid_dispatch(true);   // IRIX runs many processes at the same virtual addresses
	m_maincpu->set_drc_cache_mb(128);   // 32MB thrashes under IRIX (recompile storm)
	m_maincpu->set_dcache_size(0x8000);
	m_maincpu->set_addrmap(AS_PROGRAM, &ip24_state::ip24_map);
	ip24(config, system_clock);
}

// Factory NVRAM default for the Challenge S, used only when no nvram file
// exists: the PROM environment with console=d1 (serial port 1) and AutoLoad
// on. Without it the PROM finds no keyboard and stops at the System
// Maintenance Menu instead of booting; with console=d1 it skips the keyboard
// probe. The base image is a PROM-written NVRAM from this project;
// init_challenge_s() sets console explicitly and recomputes the checksum,
// so fields can be changed there. Bytes below 0xe are clock registers and
// are ignored (ds1386_device::nvram_default()).
static const uint8_t challenge_s_default_rtc[0x140] = {
	0x07, 0x45, 0x26, 0x00, 0x01, 0x00, 0x07, 0x00, 0x10, 0x10, 0x86, 0x84, 0x10, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x4d, 0x08, 0x64, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x6d, 0x65, 0x6d, 0x00, 0x6e, 0x75, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x64, 0x6b, 0x73, 0x63,
	0x00, 0x00, 0x00, 0x00, 0x73, 0x63, 0x73, 0x69, 0x28, 0x25, 0x64, 0x29, 0x64, 0x69, 0x73, 0x6b,
	0x28, 0x25, 0x64, 0x29, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x6d, 0x65, 0x6d, 0x00, 0x6e, 0x75, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x6d, 0x65, 0x6d, 0x00, 0x6e, 0x75, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x64, 0x6b,
	0x73, 0x63, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x39, 0x36, 0x30, 0x30, 0x00, 0x30, 0x50, 0x53, 0x54, 0x38, 0x50, 0x44,
	0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x6d, 0x65,
	0x6d, 0x00, 0x6e, 0x75, 0x6c, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x64, 0x6b, 0x73, 0x63, 0x00, 0x00,
	0x00, 0x00, 0x73, 0x63, 0x73, 0x69, 0x28, 0x25, 0x64, 0x29, 0x64, 0x69, 0x73, 0x6b, 0x28, 0x25,
	0x64, 0x29, 0x59, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x69, 0x6e, 0x69,
	0x74, 0x5f, 0x65, 0x6e, 0x76, 0x28, 0x00, 0x00, 0x00, 0x00, 0x69, 0x6e, 0x69, 0x74, 0x5f, 0x65,
	0x6e, 0x76, 0x28, 0x29, 0x0d, 0x0a, 0x00, 0x00, 0x38, 0x30, 0x00, 0x00, 0x79, 0x00, 0x00, 0x79,
	0x68, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x68, 0x00, 0x08, 0x00, 0x69, 0x12, 0x34, 0x56,
};

void ip24_state::init_challenge_s()
{
	uint8_t *const rtc = memregion("rtc")->base();
	std::copy(std::begin(challenge_s_default_rtc), std::end(challenge_s_default_rtc), rtc);

	// PROM environment block (IRIX sys/IP22nvram.h layout) at DS1386 offset
	// 0x40: checksum, layout revision, then the 2-byte console variable
	constexpr unsigned ENV = 0x40, ENV_LEN = 256, OFF_CONSOLE = 2;
	rtc[ENV + OFF_CONSOLE + 0] = 'd';
	rtc[ENV + OFF_CONSOLE + 1] = '1';

	// recompute the checksum the PROM verifies (libsk dallas.c nvchecksum():
	// XOR every byte but the checksum into a seed of 0xa5, rotating the
	// running value left one bit after each odd index)
	uint8_t sum = 0xa5;
	for (unsigned i = 0; i < ENV_LEN; i++)
	{
		if (i != 0)
			sum ^= rtc[ENV + i];
		if (i & 1)
			sum = uint8_t((sum << 1) | (sum >> 7));
	}
	rtc[ENV] = sum;
}

// Challenge S (IP24 server, SGI 007-2314-005): the Indy system board with
// no graphics board, keyboard or mouse; the console is serial port 1. This
// models the 180MHz R5000SC module. The CPU-to-system-bus ratio is assumed
// to be 3:1 as for the 150MHz R5000 (60MHz system clock). Its 512KB
// secondary cache is not modelled yet. The slots stay configurable
// (-gio64_gfx xl24 etc. add the hardware back); RAM comes from the SIMMS
// menu (default 128MB, 16-256MB on real hardware). Audio and video input are
// not fitted on this board.
void ip24_state::challenge_s(machine_config &config)
{
	constexpr uint32_t system_clock = 60'000'000;
	R5000BE(config, m_maincpu, 3 * system_clock);
	m_maincpu->set_system_clock(system_clock);
	m_maincpu->set_icache_size(0x8000);
	m_maincpu->set_drc_hash32(true);   // MAME_SGI_HASH32: 32-bit DRC hash table entries
	m_maincpu->set_drc_asid_dispatch(true);   // IRIX runs many processes at the same virtual addresses
	m_maincpu->set_drc_cache_mb(128);   // 32MB thrashes under IRIX (recompile storm)
	m_maincpu->set_dcache_size(0x8000);
	m_maincpu->set_addrmap(AS_PROGRAM, &ip24_state::ip24_map);
	ip24(config, system_clock);

	subdevice<gio64_slot_device>("gio64_gfx")->set_default_option(nullptr);
	subdevice<pc_kbdc_device>("ioc2:kbd")->set_default_option(nullptr);
	subdevice<pc_kbdc_device>("ioc2:aux")->set_default_option(nullptr);

	// no audio or video input: HAL2 reads as absent; Vino and its SAA7191
	// decoder are not fitted, so their address range bus-errors
	m_hal2->set_absent(true);
	m_vino_fitted = false;
	config.device_remove("vino");
	config.device_remove("dmsd");
}

// The 133MHz module derives its pipeline clock from a 66.666MHz crystal
// (x2) and runs SysAD, and so the MC, at a third of that: 44.444MHz. IRIX
// relies on this, assuming a 45ns MC RPSS tick (SysAD/2) on this module
// rather than computing it from the CPU frequency.
void ip24_state::indy_4613(machine_config &config)
{
	constexpr uint32_t system_clock = 44'444'444;
	R4600BE(config, m_maincpu, 3 * system_clock);
	m_maincpu->set_system_clock(system_clock);
	m_maincpu->set_icache_size(0x4000);
	m_maincpu->set_drc_hash32(true);   // MAME_SGI_HASH32: 32-bit DRC hash table entries
	m_maincpu->set_drc_asid_dispatch(true);   // IRIX runs many processes at the same virtual addresses
	m_maincpu->set_drc_cache_mb(128);   // 32MB thrashes under IRIX (recompile storm)
	m_maincpu->set_dcache_size(0x4000);
	m_maincpu->set_addrmap(AS_PROGRAM, &ip24_state::ip24_map);
	ip24(config, system_clock);
}

void ip24_state::indy_4610(machine_config &config)
{
	constexpr uint32_t system_clock = 50'000'000;
	R4600BE(config, m_maincpu, 2 * system_clock);
	m_maincpu->set_system_clock(system_clock);
	m_maincpu->set_icache_size(0x4000);
	m_maincpu->set_drc_hash32(true);   // MAME_SGI_HASH32: 32-bit DRC hash table entries
	m_maincpu->set_drc_asid_dispatch(true);   // IRIX runs many processes at the same virtual addresses
	m_maincpu->set_drc_cache_mb(128);   // 32MB thrashes under IRIX (recompile storm)
	m_maincpu->set_dcache_size(0x4000);
	m_maincpu->set_addrmap(AS_PROGRAM, &ip24_state::ip24_map);
	ip24(config, system_clock);
}

void ip22_state::indigo2_4415(machine_config &config)
{
	constexpr uint32_t system_clock = 50'000'000;
	R4400BE(config, m_maincpu, 3 * system_clock);
	m_maincpu->set_system_clock(system_clock);
	m_maincpu->set_icache_size(0x4000);
	m_maincpu->set_drc_hash32(true);   // MAME_SGI_HASH32: 32-bit DRC hash table entries
	m_maincpu->set_drc_asid_dispatch(true);   // IRIX runs many processes at the same virtual addresses
	m_maincpu->set_drc_cache_mb(128);   // 32MB thrashes under IRIX (recompile storm)
	m_maincpu->set_dcache_size(0x4000);
	m_maincpu->set_addrmap(AS_PROGRAM, &ip22_state::ip22_map);

	ip24_base(config, system_clock);
	m_mem_ctrl->set_input_default(DEVICE_INPUT_DEFAULTS_NAME(ip22_mc));

	auto &scsibus2(NSCSI_BUS(config, "scsibus2"));

	WD33C93B(config, m_scsi_ctrl2, 10000000);
	scsibus2.set_external_device(0, m_scsi_ctrl2);
	m_scsi_ctrl2->irq_cb().set(m_ioc2, FUNC(ioc2_device::scsi1_int_w));
	m_scsi_ctrl2->drq_cb().set(m_hpc3, FUNC(hpc3_device::scsi1_drq));

	NSCSI_CONNECTOR(config, "scsibus2:1", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus2:2", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus2:3", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus2:4", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus2:5", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus2:6", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsibus2:7", scsi_devices, nullptr, false);

	m_hpc3->set_addrmap(hpc3_device::AS_PIO4, &ip22_state::pio4_map);
	m_hpc3->set_addrmap(hpc3_device::AS_PIO6, &ip22_state::pio6_map);
	m_hpc3->hd_rd_cb<1>().set(m_scsi_ctrl2, FUNC(wd33c93b_device::indir_r));
	m_hpc3->hd_wr_cb<1>().set(m_scsi_ctrl2, FUNC(wd33c93b_device::indir_w));
	m_hpc3->hd_dma_rd_cb<1>().set(m_scsi_ctrl2, FUNC(wd33c93b_device::dma_r));
	m_hpc3->hd_dma_wr_cb<1>().set(m_scsi_ctrl2, FUNC(wd33c93b_device::dma_w));
	m_hpc3->hd_reset_cb<1>().set(m_scsi_ctrl2, FUNC(wd33c93b_device::reset_w));

	SGI_IOC2_FULL_HOUSE(config, m_ioc2, m_maincpu);
	DS1286(config, m_rtc, 32768);
}

#define INDY_BIOS_FLAGS(bios) ROM_GROUPDWORD | ROM_BIOS(bios)

#define INDY_BIOS_R5K \
	ROM_SYSTEM_BIOS( 0, "b10", "Version 5.3 Rev B10 R4X00/R5000 Feb 12, 1996" ) \
	ROMX_LOAD( "ip24prom.070-9101-011.bin", 0x000000, 0x080000, CRC(5e7f7e3a) SHA1(ac663a4db68528e400487e98cbf901f620fb30ce), INDY_BIOS_FLAGS(0) )

#define INDY_BIOS_R4K \
	INDY_BIOS_R5K \
	ROM_SYSTEM_BIOS( 1, "b7", "Version 5.3 Rev B7 R4X00 IP24 Feb 16, 1995" ) \
	ROMX_LOAD( "ip24prom.070-9101-008.bin", 0x000000, 0x080000, CRC(ee0b55c4) SHA1(a752a4aef7e2c6086b8b0244e9f064861a11870f), INDY_BIOS_FLAGS(1) ) \
	ROM_SYSTEM_BIOS( 2, "b6", "Version 5.0 Rev B6 Sep 28, 1994" ) \
	ROMX_LOAD( "ip24prom.070-9101-007.bin", 0x000000, 0x080000, CRC(70d8d1b1) SHA1(ade54cd2ecb7064957f8602894f05685e2f4e8fb), INDY_BIOS_FLAGS(2) ) \
	ROM_SYSTEM_BIOS( 3, "b4", "Version 5.1.2 Rev B4 R4X00 Dec 9, 1993" ) \
	ROMX_LOAD( "ip24prom.070-9101-005.bin", 0x000000, 0x080000, CRC(f5e41008) SHA1(28b769b28218a1fcd0400dceef9a284dcfbdda5b), INDY_BIOS_FLAGS(3) )

/* SCC init ip225015
 * Channel A
 * 09 <- c0 Master Interrup Control: Force HW reset + enable SWI INTACK
 * 04 <- 44 Clocks: x16 mode, 1 stop bits, no parity
 * 03 <- c0 Receiver: 8 bit data, auto enables, Rx disabled
 * 05 <- e2 Transmitter: DTR set, 8 bit data, RTS set, Tx disabled
 * 0b <- 50 Clock Mode: TRxC: XTAL output, TRxC: Output, TxC from BRG, RxC from BRG
 * 0c <- 0a Low const BRG  3.6864Mhz CLK => 9600 baud
 * 0d <- 00 High Const BRG = (CLK / (2 x Desired Rate x BR Clock period)) - 2
 * 0e <- 01 Mics: BRG enable
 * 03 <- c1 Receiver: as above + Receiver enable
 * 05 <- ea Transmitter: as above + Transmitter enable
 *
 * Channel A and B init - only BRG low const differs
 * 09 <- 80 channel A reset
 * 04 <- 44 Clocks: x16 mode, 1 stop bits, no parity
 * 0f <- 81 External/Status Control: Break/Abort enabled, WR7 prime enabled
 * 07p<- 40 External read enable (RR9=WR3, RR4=WR4, RR5=WR5, RR14=WR7 and RR11=WR10)
 * 03 <- c0 Receiver: 8 bit data, auto enables, Rx disabled
 * 05 <- e2 Transmitter: DTR set, 8 bit data, RTS set, Tx disabled
 * 0b <- 50 Clock Mode: TRxC: XTAL output, TRxC: Output, TxC from BRG, RxC from BRG
 * 0e <- 00 Mics: BRG disable
 * 0c <- 0a/04 Low const BRG, 3.6864Mhz CLK => Chan A:9600 Chan B:38400
 * 0d <- 00 High Const BRG = (CLK / (2 x Desired Rate x BR Clock period)) - 2
 * 0e <- 01 Mics: BRG enable
 * 03 <- c1 Receiver: as above + Receiver enable
 * 05 <- ea Transmitter: as above + Transmitter enable
 * 00 <- 10 Reset External/status IE
*/

ROM_START( indy_4610 )
	ROM_REGION64_BE( 0x80000, "user1", 0 )
	INDY_BIOS_R4K
ROM_END

ROM_START( indy_4613 )
	ROM_REGION64_BE( 0x80000, "user1", 0 )
	INDY_BIOS_R4K
ROM_END

ROM_START( indy_5015 )
	ROM_REGION64_BE( 0x80000, "user1", 0 )
	INDY_BIOS_R5K
ROM_END

ROM_START( challenge_s )
	ROM_REGION64_BE( 0x80000, "user1", 0 )
	INDY_BIOS_R5K

	ROM_REGION( 0x2000, "rtc", ROMREGION_ERASE00 )   // filled by init_challenge_s()
ROM_END

ROM_START( indigo2_4415 )
	ROM_REGION64_BE( 0x80000, "user1", 0 )
	ROM_SYSTEM_BIOS( 0, "e", "Version 5.3 Rev E R4X00 Jan 29, 1996" )
	ROMX_LOAD( "ip22prom.070-1367-012.bin", 0x000000, 0x080000, CRC(54460c16) SHA1(330d87b3a02a05fb49c85a569f6f84904587cb35), ROM_GROUPDWORD | ROM_BIOS(0) )
	ROM_SYSTEM_BIOS( 1, "b4", "Version 5.1.2 Rev B4 R4X00 Dec 9, 1993" )
	ROMX_LOAD( "ip22prom.070-1367-002.bin", 0x000000, 0x080000, CRC(ae5ecd08) SHA1(422568ae95282ee23b2fe123267f9b915a1dc3dc), ROM_GROUPDWORD | ROM_BIOS(1) )
ROM_END

} // anonymous namespace


//    YEAR  NAME          PARENT     COMPAT  MACHINE       INPUT CLASS       INIT        COMPANY                 FULLNAME                   FLAGS
COMP( 1993, indy_4610,    0,         0,      indy_4610,    ip24, ip24_state, empty_init, "Silicon Graphics Inc", "Indy (R4600, 100MHz)",    MACHINE_IMPERFECT_SOUND )
COMP( 1993, indy_4613,    indy_4610, 0,      indy_4613,    ip24, ip24_state, empty_init, "Silicon Graphics Inc", "Indy (R4600, 133MHz)",    MACHINE_IMPERFECT_SOUND )
COMP( 1996, indy_5015,    indy_4610, 0,      indy_5015,    ip24, ip24_state, empty_init, "Silicon Graphics Inc", "Indy (R5000, 150MHz)",    MACHINE_IMPERFECT_SOUND )
COMP( 1996, challenge_s,  indy_4610, 0,      challenge_s,  ip24, ip24_state, init_challenge_s, "Silicon Graphics Inc", "Challenge S (R5000SC, 180MHz)", MACHINE_NO_SOUND_HW )
COMP( 1993, indigo2_4415, 0,         0,      indigo2_4415, ip24, ip22_state, empty_init, "Silicon Graphics Inc", "Indigo2 (R4400, 150MHz)", MACHINE_NOT_WORKING )
