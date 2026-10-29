// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * Silicon Graphics Indigo R4000/R4400 aka 4DRPC-50/Blackjack/HP2 (also VME-based V50)
 *
 * INT2    local I/O interrupt multiplexor
 * HPC1.5  high-performance peripheral controller
 * MC      memory controller
 *
 */
 /* board revs of IP20 are: 1 & 2 were early blackjack spins
  * 10 decimal == 0x0a == 1010binary == VME IP 20 board (V50)
  * V50 board lacks audio hardware
  * The upper bit used to change the standard (rev 2) board
  * to the V50 designation (i.e the 0x8 bit) used to be known
  * internally as the 'Hollywood processor 1 bit' or hp1 bit
  */

#include "emu.h"
#include "cpu/mips/mips3.h"
#include "cpu/mips/r4000.h"
#include "cpu/dsp56000/dsp56000.h"

#include "machine/dp8573a.h"
#include "machine/edlc.h"
#include "machine/eepromser.h"
#include "machine/input_merger.h"
#include "machine/nscsi_bus.h"
#include "machine/wd33c9x.h"
#include "machine/z80scc.h"

#include "bus/rs232/rs232.h"
#include "bus/rs232/hlemouse.h"
#include "bus/nscsi/cd.h"
#include "bus/nscsi/hd.h"

#include "sound/dac.h"
#include "speaker.h"

#include "hpc1.h"
#include "int2.h"
#include "kbd.h"
#include "light.h"
#include "mc.h"
#include "nwd33c9x.h"

// SCSI: nwd33c9x_device, a single-object WD33C93 with the disk targets
// built in (several times faster per byte than an initiator and targets
// talking over an NSCSI bus), or, without this, the stock WD33C93A and
// NSCSI bus
#define MAME_SGI_IP20_USE_NWD33C9X

//#define VERBOSE (LOG_GENERAL)
#include "logmacro.h"

namespace {

class ip20_state : public driver_device
{
public:
	ip20_state(const machine_config &mconfig, device_type type, const char *tag)
		: driver_device(mconfig, type, tag)
		, m_cpu(*this, "cpu")
		, m_eerom(*this, "eerom")
		, m_mc(*this, "mc")
		, m_hpc(*this, "hpc")
		, m_int(*this, "int")
		, m_nvram(*this, "nvram")
		, m_rtc(*this, "rtc")
#ifdef MAME_SGI_IP20_USE_NWD33C9X
		, m_scsi(*this, "nwd33c9x")
#else
		, m_scsi(*this, "wd33c93a")
#endif
		, m_eth(*this, "eth")
		, m_scc(*this, "scc%u", 0U)
		, m_dsp(*this, "dsp")
		, m_dsp_ram(*this, "dsp_ram", 0x8000 * 4, ENDIANNESS_BIG)
		, m_dsp_bank_lo(*this, "dsp_lo")
		, m_dsp_bank_hi(*this, "dsp_hi")
		, m_dsp_bank_y(*this, "dsp_y")
		, m_dac(*this, "dac%u", 0U)
		, m_gfx(*this, "gfx")
	{
	}

	void indigo_r4000(machine_config &config);
	void indigo_r4400(machine_config &config);
	void indigo_r4400m(machine_config &config);

protected:
	void ip20(machine_config &config);

	virtual void machine_start() override ATTR_COLD;
	virtual void machine_reset() override ATTR_COLD;

	void cpu_map(address_map &map) ATTR_COLD;

	template <u32 Base> u64 bus_error_r(offs_t offset, u64 mem_mask);
	template <u32 Base> void bus_error_w(offs_t offset, u64 data, u64 mem_mask);
	void dsp_miscsr_w(u8 data);
	void dsp_probe_check();
	void dsp_irqa_update();
	u32 dsp_counter_r();
	void dsp_p_map(address_map &map) ATTR_COLD;
	void dsp_x_map(address_map &map) ATTR_COLD;
	void dsp_y_map(address_map &map) ATTR_COLD;
	u16 audio_portb_r();
	void audio_portb_w(offs_t offset, u16 data, u16 mem_mask);
	void audio_ssi_tx(offs_t offset, u32 data);
	double audio_rate() const;
	void vblank_w(int state);
	void clearvri_w(int state);
	TIMER_CALLBACK_MEMBER(idle_scan);

	required_device<cpu_device> m_cpu;
	required_device<eeprom_serial_93cxx_device> m_eerom;
	required_device<sgi_mc_device> m_mc;
	required_device<hpc1_device> m_hpc;
	required_device<sgi_int2_device> m_int;
	required_device<eeprom_serial_93cxx_device> m_nvram;
	required_device<dp8572a_device> m_rtc;
#ifdef MAME_SGI_IP20_USE_NWD33C9X
	using scsi_ctrl_device = nwd33c9x_device;
#else
	using scsi_ctrl_device = wd33c93a_device;
#endif
	required_device<scsi_ctrl_device> m_scsi;
	required_device<seeq8003_device> m_eth;
	required_device_array<scc85c30_device, 3> m_scc;
	required_device<dsp56001_device> m_dsp;
	memory_share_creator<u32> m_dsp_ram;
	memory_bank_creator m_dsp_bank_lo; // SRAM words 0-0x3fff
	memory_bank_creator m_dsp_bank_hi; // SRAM words 0x4000-0x7fff
	memory_bank_creator m_dsp_bank_y;  // all of the SRAM
	required_device_array<dac_16bit_r2r_twos_complement_device, 2> m_dac;
	required_device<sgi_lg1_device> m_gfx;

	u8 m_dsp_miscsr = 0;
	bool m_dsp_hpc_irq = false;
	bool m_dsp_probe = false;

	// audio board (HAL1): Actel register file on DSP port B, headphone
	// gain MDACs, the channel of the next SSI word
	u8 m_hal1_reg[16] = {};
	u16 m_portb_in = 0;
	u16 m_portb_out = 0;
	u8 m_mdac[2] = {};
	u8 m_ssi_channel = 0;
	u16 m_ssi_left = 0;
	bool m_vblank = false;
	bool m_vr_enable = false;
	bool m_vr_latch = false;
	emu_timer *m_idle_scan = nullptr;
};

void ip20_state::machine_start()
{
	save_item(NAME(m_dsp_miscsr));
	save_item(NAME(m_dsp_hpc_irq));
	save_item(NAME(m_dsp_probe));
	save_item(NAME(m_hal1_reg));
	save_item(NAME(m_portb_in));
	save_item(NAME(m_portb_out));
	save_item(NAME(m_mdac));
	save_item(NAME(m_ssi_channel));
	save_item(NAME(m_ssi_left));
	save_item(NAME(m_vblank));
	save_item(NAME(m_vr_enable));
	save_item(NAME(m_vr_latch));

	m_idle_scan = timer_alloc(FUNC(ip20_state::idle_scan), this);

	m_dsp_bank_lo->set_base(&m_dsp_ram[0]);
	m_dsp_bank_hi->set_base(&m_dsp_ram[0x4000]);
	m_dsp_bank_y->set_base(&m_dsp_ram[0]);

	// one DAC write per sample
	for (auto &dac : m_dac)
		dac->set_stream_sample_rate(48000);
}

void ip20_state::machine_reset()
{
	std::fill(std::begin(m_hal1_reg), std::end(m_hal1_reg), 0);
	m_portb_in = 0;
	m_portb_out = 0x4000;
	m_ssi_channel = 0;
	for (auto &dac : m_dac)
		dac->write(0);

	if (auto *const mips3 = dynamic_cast<mips3_device *>(m_cpu.target()))
	{
		// IRIX hands the FPU to a process lazily: it clears Status CU1 at a
		// context switch and restores the process's FP registers when its
		// next FP instruction traps, so the recompiler must test CU1 too
		mips3->mips3drc_set_options(MIPS3DRC_STRICT_COP1);

		// look for IRIX's idle loop once a second until the kernel is loaded
		mips3->set_idle_pc(0, 0);
		m_idle_scan->adjust(attotime::from_seconds(1), 0, attotime::from_seconds(1));
	}
}

// The R4000/R4400 has no WAIT, so IRIX on the IP20 idles in a busy loop,
// while (local_idle() && !idler()), which only an interrupt ends. Find
// local_idle (it tests the per-CPU p_nextthread and c_threads; the field
// offsets are left unmatched) in the kernel's text and make it the MIPS III
// core's idle PC, where it gives up its timeslice while no interrupt is
// pending, instead of spinning at full emulated speed. IRIX relinks the
// kernel on each system, so its address isn't fixed.
TIMER_CALLBACK_MEMBER(ip20_state::idle_scan)
{
	static constexpr u32 sig[10] = {
		0x8c01'0000, 0x1420'0006, 0x0000'1025, 0x8c02'0000, 0x1440'0003,
		0x0000'1025, 0x03e0'0008, 0x2402'0001, 0x03e0'0008, 0x0000'0000 }; // lw at,(zero); bnez; move v0,zero; lw v0,(zero); bnez; ...
	static constexpr u32 mask[10] = {
		0xffff'0000, 0xffff'ffff, 0xffff'ffff, 0xffff'0000, 0xffff'ffff,
		0xffff'ffff, 0xffff'ffff, 0xffff'ffff, 0xffff'ffff, 0xffff'ffff };

	address_space &space = m_cpu->space(AS_PROGRAM);
	// the kernel's text, at kseg0 0x8800'0000
	for (offs_t a = 0x0800'0000; a < 0x0840'0000; a += 4)
	{
		if ((space.read_dword(a) & mask[0]) != sig[0])
			continue;
		unsigned n = 1;
		while (n < 10 && (space.read_dword(a + n * 4) & mask[n]) == sig[n])
			n++;
		if (n == 10)
		{
			downcast<mips3_device &>(*m_cpu).set_idle_pc(0x8000'0000 | a, space.read_dword(a));
			m_idle_scan->adjust(attotime::never);
			LOG("idle loop: local_idle at %08x\n", 0x8000'0000 | a);
			return;
		}
	}
}

// The vertical retrace interrupt (local I/O 1, GIO2) is latched at the
// start of each retrace. Bit 3 of the port configuration register
// (PCON_CLEARVRI) gates it: clearing the bit clears and disables the latch,
// setting it enables the next retrace, which IRIX's graphics interrupt
// handler does after each one.
void ip20_state::vblank_w(int state)
{
	if (state && !m_vblank && m_vr_enable && !m_vr_latch)
	{
		m_vr_latch = true;
		m_int->lio1_w<sgi_int2_device::LIO1_GIO2>(1);
	}
	m_vblank = state;
}

void ip20_state::clearvri_w(int state)
{
	m_vr_enable = state;
	if (!state && m_vr_latch)
	{
		m_vr_latch = false;
		m_int->lio1_w<sgi_int2_device::LIO1_GIO2>(0);
	}
}

// HPC1 miscellaneous control and status: bit 0 holds the DSP56001 in reset,
// bit 1 drives /IRQA to the level in bit 2.
void ip20_state::dsp_miscsr_w(u8 data)
{
	bool const release = BIT(m_dsp_miscsr, 0) && !BIT(data, 0);
	m_dsp_miscsr = data;

	m_dsp->set_input_line(INPUT_LINE_RESET, BIT(data, 0) ? ASSERT_LINE : CLEAR_LINE);
	dsp_irqa_update();

	if (BIT(data, 0))
	{
		// the DSP drives the audio board's control lines: reset stops it
		m_hal1_reg[10] = 0;
		m_dsp->set_ssi_rate(0);
	}
	else if (release)
	{
		// the boot PROM's audio probe arms the mailbox with -1 first
		m_dsp_probe = (m_cpu->pc() >> 20) == 0xbfc && m_dsp_ram[0] == 0x00ff'ffffU;

		// the boot PROM gives a program it just started 50us to answer
		// through the SRAM: let the DSP catch up with the MIPS
		m_cpu->abort_timeslice();
		machine().scheduler().perfect_quantum(attotime::from_usec(200));
	}
}

// The boot PROM reads the mailbox once, 50us after starting its program,
// and lists the audio board only if the DSP reached state 1 or 2 and the
// Actel revision has bit 7 clear. IRIX treats an Indigo without audio as a
// V50 (VME, no audio), and 6.5 then hangs early in boot, so say so here.
void ip20_state::dsp_probe_check()
{
	m_dsp_probe = false;
	u32 const state = m_dsp_ram[0];
	u32 const rev = m_dsp_ram[1];
	if ((state != 1 && state != 2) || BIT(rev, 7))
	{
		osd_printf_error("%s: the boot PROM's audio probe failed (DSP state %06x, Actel revision %06x): IRIX will see no audio and configure this Indigo as a V50 (VME)\n",
			machine().describe_context(), state, rev);
		popmessage("Audio probe failed: IRIX will treat this Indigo as a V50");
	}
}

// /IRQA: the HPC1's DSP interrupt, or forced by MISCSR
void ip20_state::dsp_irqa_update()
{
	bool const forced = BIT(m_dsp_miscsr, 1) && !BIT(m_dsp_miscsr, 2);
	m_dsp->set_input_line(dsp56001_device::IRQA_LINE, (forced || m_dsp_hpc_irq) ? ASSERT_LINE : CLEAR_LINE);
}

// The DSP's external buses reach the 32K-word SRAM the MIPS sees at
// 0x1fbe0000: Y addresses through A0-A14, P and X with A14 inverted, so
// P/X:$C000-$FFFF and Y:$8000-$BFFF are the first 16K words and
// Y:$C000-$FFFF the second. Y:$FFE0-$FFED are the HPC1's DSP interface
// registers, Y:$FFFC/$FFFD the headphone gain MDACs.
void ip20_state::dsp_p_map(address_map &map)
{
	// banks, so the DSP reads the SRAM directly (A15 isn't decoded)
	map(0x0000, 0x3fff).mirror(0x8000).bankrw(m_dsp_bank_hi);
	map(0x4000, 0x7fff).mirror(0x8000).bankrw(m_dsp_bank_lo);
}

void ip20_state::dsp_x_map(address_map &map)
{
	dsp_p_map(map);
}

void ip20_state::dsp_y_map(address_map &map)
{
	map(0x0000, 0x7fff).mirror(0x8000).bankrw(m_dsp_bank_y);
	map(0xffe0, 0xffed).rw(m_hpc, FUNC(hpc1_device::dsp_r), FUNC(hpc1_device::dsp_w));
	map(0xffe5, 0xffe5).r(FUNC(ip20_state::dsp_counter_r));
	map(0xfffc, 0xfffd).lrw32(
		NAME([this](offs_t offset) { return m_mdac[offset]; }),
		NAME([this](offs_t offset, u32 data)
		{
			m_mdac[offset] = data;
			m_dac[offset]->set_output_gain(ALL_OUTPUTS, (data & 0xff) / 256.0);
		}));
}

// SGI's audio microcode (hdsp.lod) idles in a loop that reads the HPC1
// counter until it reaches a deadline it keeps at Y:$C137:
//   e164: clr b / move y:>$ffe5,b1 / sub a,b / move b1,b / tst b / jlt e164
// Polling it costs nearly all of the DSP's instructions while no audio is
// moving. When the counter is read from that loop, skip the rest of the
// wait within the current timeslice, so interrupts are still taken on time.
u32 ip20_state::dsp_counter_r()
{
	u32 const count = m_hpc->dsp_r(hpc1_device::DSPIF_COUNTER_OFFSET);

	if (!machine().side_effects_disabled() && m_dsp->pcbase() == 0xe165)
	{
		static constexpr u32 loop[] = { 0x20001b, 0x5df000, 0x00ffe5, 0x20001c, 0x21af00, 0x20000b, 0x0af0a9, 0x00e164 };
		bool match = true;
		for (unsigned i = 0; match && i < std::size(loop); i++)
			match = m_dsp_ram[((0xe164 + i) & 0x7fff) ^ 0x4000] == loop[i];

		s32 const ticks = util::sext(m_dsp_ram[0xc137 & 0x7fff] - count, 24);
		if (match && ticks > 0)
		{
			// GIO clock ticks to DSP instruction cycles
			u64 const cycles = u64(ticks) * (m_dsp->clock() / 2) / 33'333'333;
			// up to the timeslice's end or the DSP's next serial word, so
			// its interrupts are taken on time
			int const remaining = m_dsp->cycles_remaining();
			if (remaining > 0)
				m_dsp->eat_cycles(int(std::min<u64>({ cycles, u64(remaining), m_dsp->cycles_to_event() })));
		}
	}

	return count;
}

// Port B is the bus to the audio board's Actel: bits 0-7 data, bits 8-11
// register, bit 13 set for a write (clear: the Actel drives the register
// onto the data bits), bit 14 the active-low write strobe. Reads, as SGI's
// audio microcode uses them (no documentation of the Actel is known):
// register 5 is the board's revision key (bit 7 clear when the board is
// present), register 8 status (bits 2 and 3 busy after a command, bits 0-3
// the interrupt cause), register 3 received data. Nothing is busy or
// pending here, and no data arrives. Register 10 is the clock control:
// bit 0 starts the clocks ("go"), and the SSI word clock runs from then,
// left channel first, at the sample rate from the other bits (as IRIX's
// rates set them): a 48 kHz base, or 44.1 kHz with bit 2, divided by bits
// 4-6 plus one, by 1.5 more with bit 1. Bit 3 selects the AES clock, here
// taken as 48 kHz with no AES input (the boot PROM's tune, which sets it
// together with bit 2, is computed for 48 kHz).
u16 ip20_state::audio_portb_r()
{
	return m_portb_in;
}

void ip20_state::audio_portb_w(offs_t offset, u16 data, u16 mem_mask)
{
	unsigned const reg = (data >> 8) & 15;

	if (BIT(data, 13))
	{
		if (BIT(m_portb_out, 14) && !BIT(data, 14))
		{
			bool const was_running = BIT(m_hal1_reg[10], 0);
			m_hal1_reg[reg] = data & 0xff;

			if (reg == 10 && BIT(data, 0))
			{
				// two words (left, right) per sample, one DAC write per
				// sample
				double const rate = audio_rate();
				for (auto &dac : m_dac)
					dac->set_stream_sample_rate(u32(rate + 0.5));
				if (!was_running)
					m_ssi_channel = 0;
				m_dsp->set_ssi_rate(u32(rate * 2 + 0.5));
			}
			else if (reg == 10)
				m_dsp->set_ssi_rate(0);
		}
		m_portb_in = 0;
	}
	else
		m_portb_in = (reg == 5) ? 0x0a : 0x00;

	m_portb_out = data;
}

// One word per slot from the SSI, left and right in turn, 24 bits with the
// sample in the most significant bits. Both DACs change together when the
// right word arrives, as a codec latches a whole frame.
void ip20_state::audio_ssi_tx(offs_t offset, u32 data)
{
	if (!m_ssi_channel)
		m_ssi_left = u16(data >> 8);
	else
	{
		m_dac[0]->write(m_ssi_left);
		m_dac[1]->write(u16(data >> 8));
	}
	m_ssi_channel ^= 1;
}

double ip20_state::audio_rate() const
{
	u8 const clock = m_hal1_reg[10];
	double rate = (BIT(clock, 2) && !BIT(clock, 3)) ? 44100.0 : 48000.0;
	rate /= ((clock >> 4) & 7) + 1;
	if (BIT(clock, 1))
		rate = rate * 2.0 / 3.0;
	return rate;
}

// Nothing answers in these ranges: the MC times out, records the address
// and raises a bus error. Reads take a data bus error exception, which is
// how the PROM and IRIX probe for optional boards (GIO cards, additional
// HPCs); writes only record the error.
template <u32 Base> u64 ip20_state::bus_error_r(offs_t offset, u64 mem_mask)
{
	if (!machine().side_effects_disabled())
	{
		m_mc->set_cpu_buserr(Base + (offset << 3), mem_mask);
		if (auto *const mips3 = dynamic_cast<mips3_device *>(m_cpu.target()))
			mips3->bus_error();
		else
			downcast<r4000_base_device &>(*m_cpu).bus_error();
	}
	return 0;
}

template <u32 Base> void ip20_state::bus_error_w(offs_t offset, u64 data, u64 mem_mask)
{
	if (!machine().side_effects_disabled())
		m_mc->set_cpu_buserr(Base + (offset << 3), mem_mask);
}

void ip20_state::cpu_map(address_map &map)
{
	//map(0x1fbd9000, 0x1fbd903f).rw(FUNC(indigo_state::int_r), FUNC(indigo_state::int_w));

	//map(0x0000'0000, 0x0007'ffff); // system memory alias (512K)
	//map(0x0008'0000, 0x0008'ffff); // EISA I/O space (64K)
	//map(0x0009'0000, 0x0009'ffff); // EISA I/O space alias (64K)
	//map(0x000a'0000, 0x07ff'ffff); // EISA memory (128M)
	//map(0x0800'0000, 0x17ff'ffff); // physical memory segment 0 (256M)
	//map(0x1800'0000, 0x1eff'ffff); // reserved (future GIO space) (112M)
	//map(0x1f00'0000, 0x1f3f'ffff); // graphics system (4M)
	//map(0x1f40'0000, 0x1f5f'ffff); // gio64 expansion slot 0 (2M)
	//map(0x1f60'0000, 0x1f9f'ffff); // gio64 expansion slot 1 (4M)
	//map(0x1fa0'0000, 0x1faf'ffff); // mc registers (1M)
	//map(0x1fb0'0000, 0x1fbf'ffff); // hpc and i/o devices (1M)
	//map(0x1fc0'0000, 0x1fff'ffff); // boot prom (4M)
	//map(0x2000'0000, 0x2fff'ffff); // physical memory segment 1 (256M)
	//map(0x3000'0000, 0x7fff'ffff); // reserved (1.25G)
	//map(0x8000'0000, 0xffff'ffff); // EISA memory (2G)

	map(0x1800'0000, 0x1eff'ffff).rw(FUNC(ip20_state::bus_error_r<0x1800'0000>), FUNC(ip20_state::bus_error_w<0x1800'0000>)); // reserved, vme
	map(0x1f40'0000, 0x1f9f'ffff).rw(FUNC(ip20_state::bus_error_r<0x1f40'0000>), FUNC(ip20_state::bus_error_w<0x1f40'0000>)); // gio slots 0 and 1
	map(0x1fb0'0000, 0x1fb7'ffff).rw(FUNC(ip20_state::bus_error_r<0x1fb0'0000>), FUNC(ip20_state::bus_error_w<0x1fb0'0000>)); // second hpc

	map(0x1f3f'0000, 0x1f3f'7fff).m(m_gfx, FUNC(sgi_lg1_device::map));

	map(0x1fa0'0000, 0x1fa1'ffff).rw(m_mc, FUNC(sgi_mc_device::read), FUNC(sgi_mc_device::write));

	map(0x1fb8'0000, 0x1fb8'ffff).m(m_hpc, FUNC(hpc1_device::map));
	map(0x1fb8'0100, 0x1fb8'011f).m(m_eth, FUNC(seeq8003_device::map)).umask32(0xff);
	map(0x1fb8'0120, 0x1fb8'0127).rw(m_scsi, FUNC(scsi_ctrl_device::indir_r), FUNC(scsi_ctrl_device::indir_w)).umask32(0xff00);
	map(0x1fb8'01c0, 0x1fb8'01ff).m(m_int, FUNC(sgi_int2_device::map)).umask32(0xff);
	map(0x1fb8'0d00, 0x1fb8'0d0f).rw(m_scc[0], FUNC(scc85c30_device::ab_dc_r), FUNC(scc85c30_device::ab_dc_w)).umask32(0xff);
	map(0x1fb8'0d10, 0x1fb8'0d1f).rw(m_scc[1], FUNC(scc85c30_device::ab_dc_r), FUNC(scc85c30_device::ab_dc_w)).umask32(0xff);
	//map(0x1fb8'0d20, 0x1fb8'0d2f).rw(m_scc[2], FUNC(scc85c30_device::ab_dc_r), FUNC(scc85c30_device::ab_dc_w)).umask32(0xff); // V50 only?
	map(0x1fb8'0d30, 0x1fb8'0d37).noprw(); // TODO: left/right headphone gain mdacs
	map(0x1fb8'0e00, 0x1fb8'0e7f).rw(m_rtc, FUNC(dp8572a_device::read), FUNC(dp8572a_device::write)).umask32(0xff);
	map(0x1fbd'0000, 0x1fbd'0003).lr32([]() { return 0x0000'8000; }, "board_rev"); // board revision register

	// DSP SRAM, 3xTC55328J-35 (24 bits x 32k), one word per 32 bits
	map(0x1fbe'0000, 0x1fbf'ffff).lrw32(
		NAME([this](offs_t offset)
		{
			if (offset == 0 && m_dsp_probe && !machine().side_effects_disabled())
				dsp_probe_check();
			return m_dsp_ram[offset];
		}),
		NAME([this](offs_t offset, u32 data) { m_dsp_ram[offset] = data & 0x00ff'ffffU; }));

	map(0x1fc0'0000, 0x1fc7'ffff).rom().region("prom", 0);
}

static DEVICE_INPUT_DEFAULTS_START(ip20_mc)
	DEVICE_INPUT_DEFAULTS("VALID", 0x0f, 0x07)
DEVICE_INPUT_DEFAULTS_END

#ifndef MAME_SGI_IP20_USE_NWD33C9X
static void scsi_devices(device_slot_interface &device)
{
	device.option_add("cdrom", NSCSI_CDROM_SGI).machine_config(
		[](device_t *device)
		{
			downcast<nscsi_cdrom_device &>(*device).set_block_size(512);
		});
	device.option_add("harddisk", NSCSI_HARDDISK);
}
#endif

void ip20_state::indigo_r4000(machine_config &config)
{
	R4000(config, m_cpu, 50'000'000);

	ip20(config);
}

void ip20_state::indigo_r4400(machine_config &config)
{
	R4400(config, m_cpu, 75'000'000);

	ip20(config);
}

// the same R4400 on MAME's MIPS III core, which has a recompiler
void ip20_state::indigo_r4400m(machine_config &config)
{
	r4400be_device &cpu = R4400BE(config, m_cpu, 150'000'000);
	cpu.set_system_clock(50'000'000);
	cpu.set_icache_size(0x4000);
	cpu.set_dcache_size(0x4000);
	cpu.set_drc_hash32(true);
	cpu.set_drc_asid_dispatch(true); // IRIX runs many processes at the same virtual addresses
	cpu.set_drc_cache_mb(128);
	// an uncached instruction fetch takes about 273ns: the boot PROM's
	// delay calibration counts timer ticks over loops run from the PROM
	cpu.set_uncached_fetch_cycles(41);

	ip20(config);
}

void ip20_state::ip20(machine_config &config)
{
	m_cpu->set_addrmap(AS_PROGRAM, &ip20_state::cpu_map);

	EEPROM_93C56_16BIT(config, m_eerom);

	SGI_MC(config, m_mc, m_cpu, m_eerom, 50'000'000);
	m_mc->eisa_present().set_constant(0);
	m_mc->int_dma_done_cb().set(m_int, FUNC(sgi_int2_device::lio0_w<sgi_int2_device::LIO0_GDMA>));
	m_mc->set_input_default(DEVICE_INPUT_DEFAULTS_NAME(ip20_mc));

	SGI_HPC1(config, m_hpc);
	m_hpc->set_gio(m_cpu, AS_PROGRAM);
	m_hpc->set_enet(m_eth);
	m_hpc->dsp_miscsr_w().set(FUNC(ip20_state::dsp_miscsr_w));
	m_hpc->dsp_irq_w().set([this](int state) { m_dsp_hpc_irq = state; dsp_irqa_update(); });
	m_hpc->cpu_dsp_irq_w().set(m_int, FUNC(sgi_int2_device::lio1_w<sgi_int2_device::LIO1_DSP>));
	m_hpc->dsp_ram_r().set([this](offs_t offset) { return m_dsp_ram[offset & 0x7fff]; });
	m_hpc->dsp_ram_w().set([this](offs_t offset, u32 data) { m_dsp_ram[offset & 0x7fff] = data & 0x00ff'ffffU; });
	m_hpc->int_w().set(m_int, FUNC(sgi_int2_device::lio0_w<sgi_int2_device::LIO0_ETHERNET>));
	m_hpc->dma_r_cb<0>().set(m_scsi, FUNC(scsi_ctrl_device::dma_r));
	m_hpc->dma_w_cb<0>().set(m_scsi, FUNC(scsi_ctrl_device::dma_w));
	m_hpc->scsi_reset_w().set(m_scsi, FUNC(scsi_ctrl_device::reset_w));
	m_hpc->eeprom_dati().set(m_nvram, FUNC(eeprom_serial_93cxx_device::do_read));
	m_hpc->eeprom_out().set(
		[this](u8 data)
		{
			// TODO: bit 0 console led
			m_nvram->di_write(BIT(data, 3));
			m_nvram->cs_write(BIT(data, 1));
			m_nvram->clk_write(BIT(data, 2));
		});

	SGI_INT2(config, m_int, 10_MHz_XTAL);
	m_int->write_intr<1>().set_inputline(m_cpu, INPUT_LINE_IRQ0);
	m_int->write_intr<2>().set_inputline(m_cpu, INPUT_LINE_IRQ1);
	m_int->write_intr<3>().set_inputline(m_cpu, INPUT_LINE_IRQ2); // timer 0
	m_int->write_intr<4>().set_inputline(m_cpu, INPUT_LINE_IRQ3); // timer 1
	m_int->write_intr<5>().set_inputline(m_cpu, INPUT_LINE_IRQ4); // bus error
	m_int->write_led<3>().set(FUNC(ip20_state::clearvri_w));
	// TODO: write_led and write_poweroff outputs
	//
	//  0   cache parity error?
	//  1   1Hz heartbeat
	//  2   off when cpu is idle
	//  3   vertical retrace interrupt enable (PCON_CLEARVRI)
	//  4   poweroff?

	EEPROM_93C56_16BIT(config, m_nvram);

	DP8572A(config, m_rtc, 32.768_kHz_XTAL).set_use_utc(true);
	m_rtc->set_year_base(1970); // IRIX stores years since 1970

#ifdef MAME_SGI_IP20_USE_NWD33C9X
	// disks are -hard1 (SCSI ID 1, the boot disk) to -hard7 (ID 7); CD-ROMs
	// sit on the controller's own bus, where the Indigo's is at ID 4
	NWD33C9X(config, m_scsi, 10_MHz_XTAL);
	m_scsi->irq_cb().set(m_int, FUNC(sgi_int2_device::lio0_w<sgi_int2_device::LIO0_SCSI>));
	m_scsi->drq_cb().set(m_hpc, FUNC(hpc1_device::write_drq<0>));
#if MAME_SGI_SCSI_BULK_DMA
	m_scsi->set_bulk_dma_in(m_hpc, FUNC(hpc1_device::scsi_dma_bulk_in));
	m_scsi->set_bulk_dma_out(m_hpc, FUNC(hpc1_device::scsi_dma_bulk_out));
#endif
	subdevice<nscsi_connector>("nwd33c9x:auxbus:6")->set_default_option(nullptr);
	subdevice<nscsi_connector>("nwd33c9x:auxbus:4")->set_default_option("cdrom");
	// the PROM reads CDs in 512-byte blocks without selecting that size
	subdevice<nscsi_connector>("nwd33c9x:auxbus:4")->set_option_machine_config("cdrom",
		[](device_t *device) { downcast<nscsi_cdrom_device &>(*device).set_block_size(512); });
#else
	auto &scsi(NSCSI_BUS(config, "scsi"));

	WD33C93A(config, m_scsi, 10_MHz_XTAL);
	scsi.set_external_device(0, m_scsi);
	m_scsi->irq_cb().set(m_int, FUNC(sgi_int2_device::lio0_w<sgi_int2_device::LIO0_SCSI>));
	m_scsi->drq_cb().set(m_hpc, FUNC(hpc1_device::write_drq<0>));

	NSCSI_CONNECTOR(config, "scsi:1", scsi_devices, "harddisk", false);
	NSCSI_CONNECTOR(config, "scsi:2", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:3", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:4", scsi_devices, "cdrom", false);
	NSCSI_CONNECTOR(config, "scsi:5", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:6", scsi_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "scsi:7", scsi_devices, nullptr, false);
#endif

	SEEQ8003(config, m_eth);
	m_eth->out_int_cb().set(m_hpc, FUNC(hpc1_device::write_int));
	m_eth->out_rxrdy_cb().set(m_hpc, FUNC(hpc1_device::write_drq<1>));
	m_hpc->dma_r_cb<1>().set(m_eth, FUNC(seeq8003_device::fifo_r));
	m_hpc->dma_w_cb<1>().set(m_eth, FUNC(seeq8003_device::fifo_w));

	input_merger_any_high_device &scc_irq(INPUT_MERGER_ANY_HIGH(config, "scc_irq"));
	scc_irq.output_handler().set(m_int, FUNC(sgi_int2_device::lio0_w<sgi_int2_device::LIO0_DUART>));

	// duart 0
	SCC85C30(config, m_scc[0], 10_MHz_XTAL);
	m_scc[0]->configure_channels(3'686'400, 0, 3'686'400, 0);
	m_scc[0]->out_int_callback().set(scc_irq, FUNC(input_merger_any_high_device::in_w<0>));

	sgi_kbd_port_device &kbd_port(SGI_KBD_PORT(config, "keyboard_port"));
	kbd_port.option_set("keyboard", SGI_KBD);
	rs232_port_device &mouse_port(RS232_PORT(config, "mouse_port"));
	mouse_port.option_set("mouse", SGI_HLE_SERIAL_MOUSE);
	m_scc[0]->out_txda_callback().set(kbd_port, FUNC(sgi_kbd_port_device::write_txd));
	kbd_port.rxd_handler().set(m_scc[0], FUNC(scc85c30_device::rxa_w));
	mouse_port.rxd_handler().set(m_scc[0], FUNC(scc85c30_device::rxb_w));

	// duart 1
	SCC85C30(config, m_scc[1], 10_MHz_XTAL);
	// channel B is the serial console (null_modem to a host socket): IRIX
	// writes bursts faster than 9600 baud drains, so keep the bytes the
	// real FIFOs would drop on overrun
	m_scc[1]->set_overflow_queue(1, true);
	m_scc[1]->configure_channels(3'686'400, 0, 3'686'400, 0);
	m_scc[1]->out_int_callback().set(scc_irq, FUNC(input_merger_any_high_device::in_w<1>));

	rs232_port_device &rs232a(RS232_PORT(config, "rs232a", default_rs232_devices, nullptr));
	rs232_port_device &rs232b(RS232_PORT(config, "rs232b", default_rs232_devices, nullptr));
	m_scc[1]->out_txda_callback().set(rs232a, FUNC(rs232_port_device::write_txd));
	m_scc[1]->out_dtra_callback().set(rs232a, FUNC(rs232_port_device::write_dtr));
	m_scc[1]->out_rtsa_callback().set(rs232a, FUNC(rs232_port_device::write_rts));
	m_scc[1]->out_txdb_callback().set(rs232b, FUNC(rs232_port_device::write_txd));
	m_scc[1]->out_dtrb_callback().set(rs232b, FUNC(rs232_port_device::write_dtr));
	m_scc[1]->out_rtsb_callback().set(rs232b, FUNC(rs232_port_device::write_rts));
	rs232a.cts_handler().set(m_scc[1], FUNC(scc85c30_device::ctsa_w));
	rs232a.dcd_handler().set(m_scc[1], FUNC(scc85c30_device::dcda_w));
	rs232a.rxd_handler().set(m_scc[1], FUNC(scc85c30_device::rxa_w));
	rs232b.cts_handler().set(m_scc[1], FUNC(scc85c30_device::ctsb_w));
	rs232b.dcd_handler().set(m_scc[1], FUNC(scc85c30_device::dcdb_w));
	rs232b.rxd_handler().set(m_scc[1], FUNC(scc85c30_device::rxb_w));

	// duart 2
	SCC85C30(config, m_scc[2], 10_MHz_XTAL);
	m_scc[2]->configure_channels(3'686'400, 0, 3'686'400, 0);
	m_scc[2]->out_int_callback().set(scc_irq, FUNC(input_merger_any_high_device::in_w<2>));

	DSP56001(config, m_dsp, 20_MHz_XTAL);
	m_dsp->set_mode(2); // reset at P:$E000
	m_dsp->set_addrmap(AS_PROGRAM, &ip20_state::dsp_p_map);
	m_dsp->set_addrmap(AS_DATA, &ip20_state::dsp_x_map);
	m_dsp->set_addrmap(AS_IO, &ip20_state::dsp_y_map);
	m_dsp->portb_r().set(FUNC(ip20_state::audio_portb_r));
	m_dsp->portb_w().set(FUNC(ip20_state::audio_portb_w));
	m_dsp->ssi_tx().set(FUNC(ip20_state::audio_ssi_tx));

	SPEAKER(config, "speaker", 2).front();
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_dac[0], 0).add_route(ALL_OUTPUTS, "speaker", 1.0, 0);
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_dac[1], 0).add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	SGI_LG1(config, m_gfx);
	m_gfx->write_vblank().set(FUNC(ip20_state::vblank_w));
}

ROM_START(indigo_r4000)
	ROM_REGION64_BE(0x80000, "prom", 0)
	ROM_SYSTEM_BIOS(0, "405g-rev-b", "SGI Version 4.0.5G Rev B IP20, Nov 10, 1992") // dumped over serial connection from boot monitor and swapped
	ROMX_LOAD("ip20prom.070-8116-005.bin", 0x000000, 0x080000, CRC(1875b645) SHA1(52f5d7baea3d1bc720eb2164104c177e23504345), ROM_GROUPDWORD | ROM_REVERSE | ROM_BIOS(0))

	ROM_SYSTEM_BIOS(1, "405d-rev-a", "SGI Version 4.0.5D Rev A IP20, Aug 19, 1992")
	ROMX_LOAD("ip20prom.070-8116-004.bin", 0x000000, 0x080000, CRC(940d960e) SHA1(596aba530b53a147985ff3f6f853471ce48c866c), ROM_GROUPDWORD | ROM_REVERSE | ROM_BIOS(1))

	// hand-made content sets eaddr 08:00:69:12:34:56 and netaddr 192.168.137.2
	ROM_REGION16_LE(0x100, "nvram", 0)
	ROM_LOAD("nvram.bin", 0x000, 0x100, CRC(b8367798) SHA1(61af4c9dba69e9f0552c10770f16044421730b6d))
ROM_END

#define rom_indigo_r4400 rom_indigo_r4000
#define rom_indigo_r4400m rom_indigo_r4000
} // anonymous namespace

// The Indigo's LG1 graphics and its sound (no audio input or AES) are
// imperfect, but MACHINE_IMPERFECT_GRAPHICS and MACHINE_IMPERFECT_SOUND are
// left off for now: their startup warning needs a keypress on every boot,
// which stops unattended test runs.
//   YEAR  NAME          PARENT  COMPAT  MACHINE       INPUT  CLASS       INIT        COMPANY             FULLNAME             FLAGS
COMP(1992, indigo_r4000, 0,      0,      indigo_r4000, 0,     ip20_state, empty_init, "Silicon Graphics", "IRIS Indigo R4000", 0)
COMP(1991, indigo_r4400, 0,      0,      indigo_r4400, 0,     ip20_state, empty_init, "Silicon Graphics", "IRIS Indigo R4400", 0)
COMP(1991, indigo_r4400m, indigo_r4400, 0, indigo_r4400m, 0,  ip20_state, empty_init, "Silicon Graphics", "IRIS Indigo R4400 (MIPS III recompiler)", MACHINE_SUPPORTS_SAVE)
