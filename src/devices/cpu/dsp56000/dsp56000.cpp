// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay, blythe

/*
 * An emulation of the Motorola DSP56000/DSP56001.
 *
 * Sources:
 *   - http://www.bitsavers.org/components/motorola/56000/1990_DSP56000_DSP56001_Users_Manual.pdf
 *   - DSP56300 Family Manual, for the instructions the two families share
 *
 * Implemented: the whole instruction set with parallel moves, address
 * modifiers (linear, modulo, reverse carry), data shifter and limiter
 * scaling modes, convergent rounding, hardware DO and REP loops, the system
 * stack, fast and long interrupts with IPR priorities, on-chip program and
 * data RAM, the data ROMs (sine, mu-law and A-law tables), port B general
 * purpose I/O, the SSI (externally clocked, normal and network modes) and
 * the SCI registers and timer interrupt.
 *
 * TODO:
 *   - bootstrap mode (mode 1)
 *   - SCI serial transfers, host interface transfers, port C I/O
 *   - SSI internal clock and frame sync generation, on-demand mode
 *   - pipeline effects (address register use one instruction after a
 *     write), exact instruction timing and external memory wait states
 *   - data ROM contents are computed, not dumped
 */

#include "emu.h"
#include "dsp56000.h"
#include "dsp56000d.h"

//#define VERBOSE (LOG_GENERAL)

#include "logmacro.h"

DEFINE_DEVICE_TYPE(DSP56000, dsp56000_device, "dsp56000", "Motorola DSP56000")
DEFINE_DEVICE_TYPE(DSP56001, dsp56001_device, "dsp56001", "Motorola DSP56001")

namespace {

constexpr u64 M56 = 0x00ff'ffff'ffff'ffffULL;

constexpr s64 sext56(u64 v) { return s64(v << 8) >> 8; }
constexpr u64 sext24_56(u32 v) { return u64(s64(s32(v << 8) >> 8) << 24) & M56; }
constexpr s32 sext24(u32 v) { return s32(v << 8) >> 8; }

// SSI status register bits
enum : u8
{
	SSISR_TFS = 0x04,
	SSISR_RFS = 0x08,
	SSISR_TUE = 0x10,
	SSISR_ROE = 0x20,
	SSISR_TDE = 0x40,
	SSISR_RDF = 0x80,
};

// SSI control register B bits
enum : u16
{
	CRB_MOD = 0x0800,
	CRB_TE  = 0x1000,
	CRB_RE  = 0x2000,
	CRB_TIE = 0x4000,
	CRB_RIE = 0x8000,
};

u16 bitrev16(u16 v)
{
	v = ((v & 0x5555) << 1) | ((v >> 1) & 0x5555);
	v = ((v & 0x3333) << 2) | ((v >> 2) & 0x3333);
	v = ((v & 0x0f0f) << 4) | ((v >> 4) & 0x0f0f);
	return (v << 8) | (v >> 8);
}

} // anonymous namespace

dsp56000_device_base::dsp56000_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock)
	: cpu_device(mconfig, type, tag, owner, clock)
	, m_p_config("p", ENDIANNESS_BIG, 32, 16, -2)
	, m_x_config("x", ENDIANNESS_BIG, 32, 16, -2)
	, m_y_config("y", ENDIANNESS_BIG, 32, 16, -2)
	, m_portb_r(*this, 0)
	, m_portb_w(*this)
	, m_ssi_tx(*this)
	, m_ssi_rx(*this, 0)
	, m_mode_pins(0)
	, m_icount(0)
{
}

dsp56000_device::dsp56000_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: dsp56000_device_base(mconfig, DSP56000, tag, owner, clock)
{
}

dsp56001_device::dsp56001_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock)
	: dsp56000_device_base(mconfig, DSP56001, tag, owner, clock)
{
}

void dsp56000_device_base::device_start()
{
	space(AS_PROGRAM).cache(m_pmem);
	space(AS_DATA).cache(m_xmem);
	space(AS_IO).cache(m_ymem);

	m_ssi_period = 0;
	m_ssi_count = 0;
	m_sci_period = 0;
	m_sci_count = 0;
	m_event_cycles = 0;
	m_event_icount = std::numeric_limits<int>::min();

	// Y data ROM: one cycle of a sine wave
	for (unsigned i = 0; i < 256; i++)
	{
		double const s = std::sin(2.0 * M_PI * i / 256.0) * 8388608.0;
		s32 const v = std::clamp<s32>(s32(std::lround(s)), -8388608, 8388607);
		m_yrom[i] = u32(v) & 0xffffff;
	}

	// X data ROM: mu-law then A-law expansion of the 128 positive codes,
	// as 24-bit fractions (ITU-T G.711 decoding)
	for (unsigned i = 0; i < 128; i++)
	{
		// mu-law: code i (complemented on the line), 14-bit magnitude
		unsigned const u = ~i & 0x7f;
		unsigned const exponent = (u >> 4) & 7;
		unsigned const mantissa = u & 15;
		s32 const mu = ((mantissa << 3) + 0x84) << exponent;
		m_xrom[i] = u32((mu - 0x84) << 10) & 0xffffff;

		// A-law: code i with even bits inverted, 13-bit magnitude
		unsigned const a = i ^ 0x55;
		unsigned const ae = (a >> 4) & 7;
		unsigned const am = a & 15;
		s32 const al = ae ? ((am << 4) + 0x108) << (ae - 1) : (am << 4) + 8;
		m_xrom[128 + i] = u32(al << 11) & 0xffffff;
	}

	save_item(NAME(m_pc));
	save_item(NAME(m_ppc));
	save_item(NAME(m_x));
	save_item(NAME(m_y));
	save_item(NAME(m_a));
	save_item(NAME(m_r));
	save_item(NAME(m_n));
	save_item(NAME(m_m));
	save_item(NAME(m_sr));
	save_item(NAME(m_omr));
	save_item(NAME(m_sp));
	save_item(NAME(m_la));
	save_item(NAME(m_lc));
	save_item(NAME(m_ssh));
	save_item(NAME(m_ssl));
	save_item(NAME(m_rep));
	save_item(NAME(m_rep_pc));
	save_item(NAME(m_rep_lc));
	save_item(NAME(m_irq_pending));
	save_item(NAME(m_irqa_state));
	save_item(NAME(m_irqb_state));
	save_item(NAME(m_fast_irq));
	save_item(NAME(m_irq_return));
	save_item(NAME(m_irq_level));
	save_item(NAME(m_wait));
	save_item(NAME(m_pram));
	save_item(NAME(m_xram));
	save_item(NAME(m_yram));
	save_item(NAME(m_ipr));
	save_item(NAME(m_bcr));
	save_item(NAME(m_pbc));
	save_item(NAME(m_pbddr));
	save_item(NAME(m_pbd));
	save_item(NAME(m_pcc));
	save_item(NAME(m_pcddr));
	save_item(NAME(m_pcd));
	save_item(NAME(m_cra));
	save_item(NAME(m_crb));
	save_item(NAME(m_ssisr));
	save_item(NAME(m_tsr_flag));
	save_item(NAME(m_tx));
	save_item(NAME(m_rx));
	save_item(NAME(m_ssi_slot));
	save_item(NAME(m_scr));
	save_item(NAME(m_ssr));
	save_item(NAME(m_sccr));
	save_item(NAME(m_host));
	save_item(NAME(m_ssi_period));
	save_item(NAME(m_ssi_count));
	save_item(NAME(m_sci_period));
	save_item(NAME(m_sci_count));

	state_add(STATE_GENPC, "GENPC", m_pc).noshow();
	state_add(STATE_GENPCBASE, "CURPC", m_ppc).noshow();

	state_add(DSP_PC, "PC", m_pc);
	state_add(DSP_SR, "SR", m_sr);
	state_add(DSP_OMR, "OMR", m_omr);
	state_add(DSP_SP, "SP", m_sp).mask(0x3f);
	state_add(DSP_LA, "LA", m_la);
	state_add(DSP_LC, "LC", m_lc);
	state_add(DSP_X0, "X0", m_x[0]).mask(0xffffff);
	state_add(DSP_X1, "X1", m_x[1]).mask(0xffffff);
	state_add(DSP_Y0, "Y0", m_y[0]).mask(0xffffff);
	state_add(DSP_Y1, "Y1", m_y[1]).mask(0xffffff);
	state_add(DSP_A, "A", m_a[0]).mask(M56);
	state_add(DSP_B, "B", m_a[1]).mask(M56);
	for (unsigned i = 0; i < 8; i++)
		state_add(DSP_R0 + i, util::string_format("R%d", i).c_str(), m_r[i]);
	for (unsigned i = 0; i < 8; i++)
		state_add(DSP_N0 + i, util::string_format("N%d", i).c_str(), m_n[i]);
	for (unsigned i = 0; i < 8; i++)
		state_add(DSP_M0 + i, util::string_format("M%d", i).c_str(), m_m[i]);
	state_add(DSP_IPR, "IPR", m_ipr);
	state_add(DSP_BCR, "BCR", m_bcr);

	set_icountptr(m_icount);

	std::fill(std::begin(m_x), std::end(m_x), 0);
	std::fill(std::begin(m_y), std::end(m_y), 0);
	std::fill(std::begin(m_a), std::end(m_a), 0);
	std::fill(std::begin(m_r), std::end(m_r), 0);
	std::fill(std::begin(m_n), std::end(m_n), 0);
	std::fill(std::begin(m_ssh), std::end(m_ssh), 0);
	std::fill(std::begin(m_ssl), std::end(m_ssl), 0);
	std::fill(std::begin(m_pram), std::end(m_pram), 0);
	std::fill(std::begin(m_xram), std::end(m_xram), 0);
	std::fill(std::begin(m_yram), std::end(m_yram), 0);
	std::fill(std::begin(m_host), std::end(m_host), 0);
	m_ppc = 0;
	m_la = 0;
	m_lc = 0;
	m_irqa_state = CLEAR_LINE;
	m_irqb_state = CLEAR_LINE;
}

void dsp56000_device_base::device_reset()
{
	std::fill(std::begin(m_m), std::end(m_m), 0xffff);
	m_sr = 0x0300;
	m_sp = 0;
	m_omr = m_mode_pins;
	m_rep = false;
	m_rep_pc = 0;
	m_rep_lc = 0;
	m_irq_pending = 0;
	m_fast_irq = 0;
	m_irq_return = 0;
	m_irq_level = 0;
	m_wait = false;
	m_pending_count = 0;

	// mode 2 starts at the external program memory reset address
	m_pc = (m_mode_pins == 2) ? 0xe000 : 0x0000;
	m_ppc = m_pc;

	m_bcr = 0xffff;
	m_ipr = 0;
	reset_peripherals();

	// banks are set up by now (machine_start runs before the first reset)
	build_pages(space(AS_PROGRAM), m_page_pr, m_page_pw);
	build_pages(space(AS_DATA), m_page_xr, m_page_xw);
	build_pages(space(AS_IO), m_page_yr, m_page_yw);
}

// Direct pointers for the 256-word pages of a space that are plain memory
// throughout (RAM, ROM or a bank, contiguous on the host): their accesses
// skip the handlers. Other pages, and memory remapped after reset, go
// through the address space.
void dsp56000_device_base::build_pages(address_space &space, u32 *(&r)[256], u32 *(&w)[256])
{
	for (unsigned page = 0; page < 256; page++)
	{
		u32 *const rbase = static_cast<u32 *>(space.get_read_ptr(page << 8));
		u32 *const wbase = static_cast<u32 *>(space.get_write_ptr(page << 8));
		bool rok = rbase != nullptr, wok = wbase != nullptr;
		for (unsigned i = 1; i < 256 && (rok || wok); i++)
		{
			if (rok && static_cast<u32 *>(space.get_read_ptr((page << 8) | i)) != rbase + i)
				rok = false;
			if (wok && static_cast<u32 *>(space.get_write_ptr((page << 8) | i)) != wbase + i)
				wok = false;
		}
		r[page] = rok ? rbase : nullptr;
		w[page] = wok ? wbase : nullptr;
	}
}

void dsp56000_device_base::reset_peripherals()
{
	m_pbc = 0;
	m_pbddr = 0;
	m_pbd = 0;
	m_pcc = 0;
	m_pcddr = 0;
	m_pcd = 0;
	m_cra = 0;
	m_crb = 0;
	m_ssisr = SSISR_TDE;
	m_tsr_flag = 0;
	m_tx = 0;
	m_rx = 0;
	m_ssi_slot = 0;
	m_scr = 0;
	m_ssr = 0x0003; // transmitter empty, transmit data register empty
	m_sccr = 0;
	m_sci_period = 0;
	portb_update();
}

device_memory_interface::space_config_vector dsp56000_device_base::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PROGRAM, &m_p_config),
		std::make_pair(AS_DATA, &m_x_config),
		std::make_pair(AS_IO, &m_y_config)
	};
}

std::unique_ptr<util::disasm_interface> dsp56000_device_base::create_disassembler()
{
	return std::make_unique<dsp56000_disassembler>();
}

void dsp56000_device_base::state_import(device_state_entry const &entry)
{
}

void dsp56000_device_base::state_export(device_state_entry const &entry)
{
}

void dsp56000_device_base::execute_set_input(int inputnum, int state)
{
	switch (inputnum)
	{
	case IRQA_LINE:
		// edge triggered when IPR IAL2 is set (on assertion of /IRQA)
		if (state != CLEAR_LINE && m_irqa_state == CLEAR_LINE)
			m_irq_pending |= 1U << (VEC_IRQA >> 1);
		m_irqa_state = state;
		break;
	case IRQB_LINE:
		if (state != CLEAR_LINE && m_irqb_state == CLEAR_LINE)
			m_irq_pending |= 1U << (VEC_IRQB >> 1);
		m_irqb_state = state;
		break;
	}
}

//-------------------------------------------------
//  memory
//-------------------------------------------------

u32 dsp56000_device_base::p_read(u16 address)
{
	// mode 3 disables the on-chip program RAM
	if (address < 0x200 && (m_omr & 3) != 3)
		return m_pram[address];

	if (u32 const *const page = m_page_pr[address >> 8])
		return page[address & 0xff] & 0x00ffffffU;
	return m_pmem.read_dword(address) & 0x00ffffffU;
}

void dsp56000_device_base::p_write(u16 address, u32 data)
{
	data &= 0x00ffffffU;
	if (address < 0x200 && (m_omr & 3) != 3)
		m_pram[address] = data;
	else if (u32 *const page = m_page_pw[address >> 8])
		page[address & 0xff] = data;
	else
		m_pmem.write_dword(address, data);
}

u32 dsp56000_device_base::x_read(u16 address)
{
	if (address < 0x100)
		return m_xram[address];
	if (address < 0x200 && BIT(m_omr, 2))
		return m_xrom[address & 0xff];
	if (address >= 0xffc0)
		return periph_r(address);

	if (u32 const *const page = m_page_xr[address >> 8])
		return page[address & 0xff] & 0x00ffffffU;
	return m_xmem.read_dword(address) & 0x00ffffffU;
}

void dsp56000_device_base::x_write(u16 address, u32 data)
{
	data &= 0x00ffffffU;
	if (address < 0x100)
		m_xram[address] = data;
	else if (address < 0x200 && BIT(m_omr, 2))
		; // data ROM
	else if (address >= 0xffc0)
		periph_w(address, data);
	else if (u32 *const page = m_page_xw[address >> 8])
		page[address & 0xff] = data;
	else
		m_xmem.write_dword(address, data);
}

u32 dsp56000_device_base::y_read(u16 address)
{
	if (address < 0x100)
		return m_yram[address];
	if (address < 0x200 && BIT(m_omr, 2))
		return m_yrom[address & 0xff];

	if (u32 const *const page = m_page_yr[address >> 8])
		return page[address & 0xff] & 0x00ffffffU;
	return m_ymem.read_dword(address) & 0x00ffffffU;
}

void dsp56000_device_base::y_write(u16 address, u32 data)
{
	data &= 0x00ffffffU;
	if (address < 0x100)
		m_yram[address] = data;
	else if (address < 0x200 && BIT(m_omr, 2))
		; // data ROM
	else if (u32 *const page = m_page_yw[address >> 8])
		page[address & 0xff] = data;
	else
		m_ymem.write_dword(address, data);
}

//-------------------------------------------------
//  on-chip peripherals
//-------------------------------------------------

u32 dsp56000_device_base::periph_r(u16 address)
{
	switch (address)
	{
	case 0xffe0: return m_pbc;
	case 0xffe1: return m_pcc;
	case 0xffe2: return m_pbddr;
	case 0xffe3: return m_pcddr;
	case 0xffe4:
		return ((m_pbd & m_pbddr) | (m_portb_r() & ~m_pbddr)) & 0x7fff;
	case 0xffe5: return m_pcd & 0x0fff;
	case 0xffe8: return m_host[0]; // host control
	case 0xffe9: return m_host[1] | 0x02; // host status: transmit empty
	case 0xffeb: return m_host[3]; // host receive
	case 0xffec: return m_cra;
	case 0xffed: return m_crb;
	case 0xffee:
		if (!machine().side_effects_disabled())
			m_tsr_flag = 1;
		return m_ssisr;
	case 0xffef:
		if (!machine().side_effects_disabled())
		{
			m_ssisr &= ~SSISR_RDF;
			if (m_tsr_flag)
				m_ssisr &= ~SSISR_ROE;
			m_tsr_flag = 0;
		}
		return m_rx;
	case 0xfff0: return m_scr;
	case 0xfff1: return m_ssr;
	case 0xfff2: return m_sccr;
	case 0xfffe: return m_bcr;
	case 0xffff: return m_ipr;
	default:
		return 0;
	}
}

void dsp56000_device_base::periph_w(u16 address, u32 data)
{
	switch (address)
	{
	case 0xffe0: m_pbc = data & 1; break;
	case 0xffe1: m_pcc = data & 0x0fff; break;
	case 0xffe2: m_pbddr = data & 0x7fff; portb_update(); break;
	case 0xffe3: m_pcddr = data & 0x0fff; break;
	case 0xffe4: m_pbd = data & 0x7fff; portb_update(); break;
	case 0xffe5: m_pcd = data & 0x0fff; break;
	case 0xffe8: m_host[0] = data & 0x1f; break;
	case 0xffeb: m_host[2] = data; break; // host transmit
	case 0xffec: m_cra = data & 0xffff; break;
	case 0xffed: m_crb = data & 0xffff; break;
	case 0xffee: break; // time slot register: skips one transmit slot
	case 0xffef:
		m_tx = data;
		m_ssisr &= ~SSISR_TDE;
		if (m_tsr_flag)
			m_ssisr &= ~SSISR_TUE;
		m_tsr_flag = 0;
		break;
	case 0xfff0: m_scr = data & 0x3fff; sci_timer_update(); break;
	case 0xfff2: m_sccr = data & 0xffff; sci_timer_update(); break;
	case 0xfffe: m_bcr = data & 0xffff; break;
	case 0xffff: m_ipr = data & 0xfc3f; break;
	}
}

void dsp56000_device_base::portb_update()
{
	if (!m_pbc)
		m_portb_w(0, m_pbd, m_pbddr);
}

// The SCI timer interrupts at the SCI baud rate clock divided by 16: the
// oscillator divided by 64, by 8 more with SCP, and by CD + 1 (half as many
// instruction cycles).
void dsp56000_device_base::sci_timer_update()
{
	advance();

	if (BIT(m_scr, 13))
	{
		unsigned const cd = (m_sccr & 0x0fff) + 1;
		unsigned const prescale = BIT(m_sccr, 13) ? 8 : 1;
		u64 const period = (u64(32) * prescale * cd) << 32;
		if (!m_sci_period)
			m_sci_count = period;
		m_sci_period = period;
	}
	else
		m_sci_period = 0;

	advance();
}

void dsp56000_device_base::set_ssi_rate(u32 words_per_second)
{
	// bring the counts up to now before changing them
	advance();

	if (!words_per_second)
		m_ssi_period = 0;
	else
	{
		u64 const period = (u64(clock() / 2) << 32) / words_per_second;
		if (!m_ssi_period)
			m_ssi_count = period;
		m_ssi_period = period;
	}

	advance();
}

u32 dsp56000_device_base::cycles_to_event() const
{
	// while executing, the counts are only brought up to date when an
	// event is due: measure from that deadline
	if (executing() && m_event_icount != std::numeric_limits<int>::min())
		return u32(std::max(m_icount - m_event_icount, 1));

	return counted_cycles_to_event();
}

u32 dsp56000_device_base::counted_cycles_to_event() const
{
	s64 next = std::numeric_limits<s32>::max();
	if (m_ssi_period)
		next = std::min<s64>(next, (m_ssi_count + 0xffff'ffffLL) >> 32);
	if (m_sci_period)
		next = std::min<s64>(next, (m_sci_count + 0xffff'ffffLL) >> 32);
	return u32(std::max<s64>(next, 1));
}

// Account the cycles executed since the last call to the internally clocked
// events. Measured with total_cycles(): an aborted timeslice zeroes the
// cycle count without time passing.
void dsp56000_device_base::advance()
{
	u64 const now = total_cycles();
	u64 const cycles = now - m_event_cycles;
	m_event_cycles = now;
	if (cycles)
		advance_events(cycles);

	// the next event is due when the icount reaches this
	m_event_icount = (m_ssi_period || m_sci_period) ? m_icount - int(counted_cycles_to_event()) : std::numeric_limits<int>::min();
}

void dsp56000_device_base::advance_events(u64 cycles)
{
	if (m_ssi_period)
	{
		m_ssi_count -= s64(cycles << 32);
		while (m_ssi_count <= 0 && m_ssi_period)
		{
			ssi_slot();
			m_ssi_count += m_ssi_period;
		}
	}

	if (m_sci_period)
	{
		m_sci_count -= s64(cycles << 32);
		while (m_sci_count <= 0)
		{
			m_irq_pending |= 1U << (VEC_SCI_TMR >> 1);
			m_sci_count += m_sci_period;
		}
	}
}

void dsp56000_device_base::ssi_slot()
{
	// SSI pins (port C bits 3-8) selected and enabled
	if (!(m_pcc & 0x01f8))
		return;

	bool const network = m_crb & CRB_MOD;
	unsigned const words = network ? ((m_cra >> 8) & 31) + 1 : 1;
	bool const frame = m_ssi_slot == 0;

	if (m_crb & CRB_TE)
	{
		if (m_ssisr & SSISR_TDE)
			m_ssisr |= SSISR_TUE;

		m_ssi_tx(m_ssi_slot, m_tx);
		m_ssisr |= SSISR_TDE;
		if (frame)
			m_ssisr |= SSISR_TFS;
		else
			m_ssisr &= ~SSISR_TFS;
	}

	if (m_crb & CRB_RE)
	{
		if (m_ssisr & SSISR_RDF)
		{
			m_ssisr |= SSISR_ROE;
		}

		m_rx = m_ssi_rx(m_ssi_slot) & 0x00ffffffU;
		m_ssisr |= SSISR_RDF;
		if (frame)
			m_ssisr |= SSISR_RFS;
		else
			m_ssisr &= ~SSISR_RFS;
	}

	m_ssi_slot = (m_ssi_slot + 1) % words;
}

//-------------------------------------------------
//  registers
//-------------------------------------------------

// Accumulator onto the 24-bit data bus: the data shifter applies the scaling
// mode, then the limiter substitutes the largest value of the sign when the
// value doesn't fit and sets L.
u32 dsp56000_device_base::acc_bus(unsigned d)
{
	u64 const v = m_a[d];
	u32 const a2 = (v >> 48) & 0xff;
	u32 const a1 = (v >> 24) & 0xffffff;
	u32 const a0 = v & 0xffffff;
	u32 const combined = (a2 << 24) | a1;

	unsigned const scaling = (m_sr >> 10) & 3;
	u32 value;
	switch (scaling)
	{
	case 1: value = (combined >> 1) & 0xffffff; break;
	case 2: value = ((combined << 1) | (a0 >> 23)) & 0xffffff; break;
	default: value = a1; break;
	}

	bool fits = (a2 == 0x00 && value <= 0x7fffff) || (a2 == 0xff && value >= 0x800000);
	if (scaling == 2)
		fits = fits && (BIT(a1, 23) == BIT(a2, 7));

	if (fits)
		return value;

	m_sr |= SR_L;
	return BIT(a2, 7) ? 0x800000 : 0x7fffff;
}

void dsp56000_device_base::acc_w24(unsigned d, u32 data)
{
	m_a[d] = sext24_56(data);
}

u32 dsp56000_device_base::reg_r(unsigned dddddd)
{
	switch (dddddd)
	{
	case 0x04: return m_x[0];
	case 0x05: return m_x[1];
	case 0x06: return m_y[0];
	case 0x07: return m_y[1];
	case 0x08: return m_a[0] & 0xffffff;
	case 0x09: return m_a[1] & 0xffffff;
	case 0x0a: return u32(s32(s8(m_a[0] >> 48))) & 0xffffff;
	case 0x0b: return u32(s32(s8(m_a[1] >> 48))) & 0xffffff;
	case 0x0c: return (m_a[0] >> 24) & 0xffffff;
	case 0x0d: return (m_a[1] >> 24) & 0xffffff;
	case 0x0e: return acc_bus(0);
	case 0x0f: return acc_bus(1);
	case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		return m_r[dddddd & 7];
	case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
		return m_n[dddddd & 7];
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
		return m_m[dddddd & 7];
	case 0x39: return m_sr;
	case 0x3a: return m_omr;
	case 0x3b: return m_sp;
	case 0x3c: return ssh_pop();
	case 0x3d: return m_ssl[m_sp & 15];
	case 0x3e: return m_la;
	case 0x3f: return m_lc;
	default:
		return 0;
	}
}

void dsp56000_device_base::reg_w(unsigned dddddd, u32 data)
{
	data &= 0x00ffffffU;
	switch (dddddd)
	{
	case 0x04: m_x[0] = data; break;
	case 0x05: m_x[1] = data; break;
	case 0x06: m_y[0] = data; break;
	case 0x07: m_y[1] = data; break;
	case 0x08: case 0x09:
		m_a[dddddd & 1] = (m_a[dddddd & 1] & ~u64(0xffffff)) | data;
		break;
	case 0x0a: case 0x0b:
		m_a[dddddd & 1] = (m_a[dddddd & 1] & 0x0000'ffff'ffff'ffffULL) | (u64(data & 0xff) << 48);
		break;
	case 0x0c: case 0x0d:
		m_a[dddddd & 1] = (m_a[dddddd & 1] & ~(u64(0xffffff) << 24)) | (u64(data) << 24);
		break;
	case 0x0e: case 0x0f:
		acc_w24(dddddd & 1, data);
		break;
	case 0x10: case 0x11: case 0x12: case 0x13: case 0x14: case 0x15: case 0x16: case 0x17:
		m_r[dddddd & 7] = data;
		break;
	case 0x18: case 0x19: case 0x1a: case 0x1b: case 0x1c: case 0x1d: case 0x1e: case 0x1f:
		m_n[dddddd & 7] = data;
		break;
	case 0x20: case 0x21: case 0x22: case 0x23: case 0x24: case 0x25: case 0x26: case 0x27:
		m_m[dddddd & 7] = data;
		break;
	case 0x39: sr_w(data); break;
	case 0x3a: m_omr = data & 0xc7; break;
	case 0x3b: m_sp = data & 0x3f; break;
	case 0x3c: ssh_push(data); break;
	case 0x3d: m_ssl[m_sp & 15] = data; break;
	case 0x3e: m_la = data; break;
	case 0x3f: m_lc = data; break;
	}
}

// System stack: 15 levels; SP bit 4 is the stack error flag, bit 5 the
// underflow flag.
void dsp56000_device_base::push(u16 high, u16 low)
{
	unsigned const sp = (m_sp & 15) + 1;
	if (sp > 15)
	{
		m_sp |= 0x10;
		m_irq_pending |= 1U << (VEC_STACK >> 1);
	}
	m_sp = (m_sp & 0x30) | (sp & 15);
	m_ssh[m_sp & 15] = high;
	m_ssl[m_sp & 15] = low;
}

void dsp56000_device_base::pop(u16 &high, u16 &low)
{
	high = m_ssh[m_sp & 15];
	low = m_ssl[m_sp & 15];
	if (!(m_sp & 15))
	{
		m_sp |= 0x30;
		m_irq_pending |= 1U << (VEC_STACK >> 1);
	}
	m_sp = (m_sp & 0x30) | ((m_sp - 1) & 15);
}

u16 dsp56000_device_base::ssh_pop()
{
	u16 high, low;
	pop(high, low);
	return high;
}

void dsp56000_device_base::ssh_push(u16 data)
{
	push(data, m_ssl[((m_sp & 15) + 1) & 15]);
}

//-------------------------------------------------
//  address generation
//-------------------------------------------------

// New value of Rn after adding delta under the modifier in Mn.
u16 dsp56000_device_base::agu_update(unsigned rrr, s32 delta)
{
	u16 const r = m_r[rrr];
	u16 const m = m_m[rrr];

	if (m == 0xffff)
		return r + delta;

	if (m == 0)
	{
		// reverse carry: the update is done on the bit-reversed address
		u16 const rev_r = bitrev16(r);
		u16 const rev_n = bitrev16(u16(std::abs(delta)));
		return bitrev16(delta >= 0 ? u16(rev_r + rev_n) : u16(rev_r - rev_n));
	}

	if (m & 0x8000)
		return r; // reserved

	// modulo M + 1: the buffer starts at the next lower multiple of the
	// smallest power of two not less than the modulus
	s32 const modulo = s32(m) + 1;
	s32 bufsize = 1;
	while (bufsize < modulo)
		bufsize <<= 1;

	s32 const lo = r & ~(bufsize - 1) & 0xffff;
	s32 const hi = lo + modulo - 1;
	s32 value = r;

	if (std::abs(delta) > modulo)
	{
		// wrapping by whole buffers is linear
		while (delta > bufsize) { value += bufsize; delta -= bufsize; }
		while (delta < -bufsize) { value -= bufsize; delta += bufsize; }
	}

	value += delta;
	if (delta > 0 && value > hi)
		value -= modulo;
	else if (delta < 0 && value < lo)
		value += modulo;

	return u16(value);
}

// Effective address MMMRRR: returns the operand address and applies the
// address register update; absolute and immediate modes consume the
// extension word.
u16 dsp56000_device_base::ea(unsigned MMMRRR, bool &immediate, u32 &ext, bool &extended)
{
	unsigned const rrr = MMMRRR & 7;
	immediate = false;
	extended = false;

	switch (MMMRRR >> 3)
	{
	case 0: { u16 const a = m_r[rrr]; m_r[rrr] = agu_update(rrr, -s32(m_n[rrr])); return a; }
	case 1: { u16 const a = m_r[rrr]; m_r[rrr] = agu_update(rrr, m_n[rrr]); return a; }
	case 2: { u16 const a = m_r[rrr]; m_r[rrr] = agu_update(rrr, -1); return a; }
	case 3: { u16 const a = m_r[rrr]; m_r[rrr] = agu_update(rrr, 1); return a; }
	case 4: return m_r[rrr];
	case 5: return agu_update(rrr, m_n[rrr]);
	case 6:
		ext = fetch(m_next_pc);
		m_last = m_next_pc++;
		extended = true;
		if (rrr == 4)
			immediate = true;
		return u16(ext);
	case 7: m_r[rrr] = agu_update(rrr, -1); return m_r[rrr];
	}

	return 0;
}

u16 dsp56000_device_base::ea_short(unsigned MMMRRR)
{
	bool immediate, extended;
	u32 ext;
	return ea(MMMRRR & 0x3f, immediate, ext, extended);
}

//-------------------------------------------------
//  data arithmetic logic unit
//-------------------------------------------------

void dsp56000_device_base::flags_nz(u64 r)
{
	u32 const a2 = (r >> 48) & 0xff;
	u32 const a1 = (r >> 24) & 0xffffff;
	bool e, u;

	switch ((m_sr >> 10) & 3)
	{
	case 1: // scale down
	{
		u32 const val = ((a2 << 1) | (a1 >> 23)) & 3;
		e = a2 != 0x00 && a2 != 0xff;
		u = val == 0 || val == 3;
		break;
	}
	case 2: // scale up
	{
		u32 const val_e = ((a2 << 2) | (a1 >> 22)) & 0x3ff;
		u32 const bits = a1 & 0x600000;
		e = val_e != 0 && val_e != 0x3ff;
		u = bits == 0 || bits == 0x600000;
		break;
	}
	default:
	{
		u32 const val_e = ((a2 << 1) | (a1 >> 23)) & 0x1ff;
		u32 const bits = a1 & 0xc00000;
		e = val_e != 0 && val_e != 0x1ff;
		u = bits == 0 || bits == 0xc00000;
		break;
	}
	}

	m_sr &= ~(SR_E | SR_U | SR_N | SR_Z);
	if (e)
		m_sr |= SR_E;
	if (u)
		m_sr |= SR_U;
	if (BIT(r, 55))
		m_sr |= SR_N;
	if (!(r & M56))
		m_sr |= SR_Z;
}

// Convergent rounding at the boundary the scaling mode selects.
u64 dsp56000_device_base::round(u64 v)
{
	u32 r2, r1, r0 = 0;

	switch ((m_sr >> 10) & 3)
	{
	case 1: // scale down: round at bit 24
	{
		u64 const sum = v + (u64(1) << 24);
		u32 const s0 = sum & 0xffffff;
		r1 = (sum >> 24) & 0xffffff;
		r2 = (sum >> 48) & 0xff;
		if (s0 == 0 && !(r1 & 1))
			r1 &= ~3U;
		r1 &= ~1U;
		break;
	}
	case 2: // scale up: round at bit 22
	{
		u64 const sum = v + (u64(1) << 22);
		r0 = sum & 0xffffff;
		r1 = (sum >> 24) & 0xffffff;
		r2 = (sum >> 48) & 0xff;
		if (!(r0 & 0x7fffff))
			r0 = 0;
		r0 &= 0x800000;
		break;
	}
	default: // round at bit 23
	{
		u64 const sum = v + (u64(1) << 23);
		u32 const s0 = sum & 0xffffff;
		r1 = (sum >> 24) & 0xffffff;
		r2 = (sum >> 48) & 0xff;
		if (s0 == 0)
			r1 &= ~1U;
		break;
	}
	}

	return (u64(r2) << 48) | (u64(r1) << 24) | r0;
}

// Fractional multiply: the 48-bit product shifted left one bit.
u64 dsp56000_device_base::mul(u32 s1, u32 s2)
{
	s64 const p = s64(sext24(s1)) * s64(sext24(s2));
	return u64(p << 1) & M56;
}

// d +/- s (with carry, after the ADDL/ADDR shift) and the standard flags.
// shift_mode: 0 none, 1 destination shifted left first, 2 shifted right.
void dsp56000_device_base::addsub(unsigned d, u64 s, bool sub, bool carry_in, bool store, unsigned shift_mode)
{
	u64 dv = m_a[d];
	bool v_shift = false;

	if (shift_mode == 1)
	{
		u64 const t = (dv << 1) & M56;
		v_shift = BIT(t, 55) != BIT(dv, 55);
		dv = t;
	}
	else if (shift_mode == 2)
		dv = u64(sext56(dv) >> 1) & M56;

	s &= M56;
	u64 const c = (carry_in && (m_sr & SR_C)) ? 1 : 0;
	u64 const raw = sub ? dv - s - c : dv + s + c;
	u64 const r = raw & M56;

	bool const carry = BIT(raw, 56);
	bool const overflow = sub
		? BIT((dv ^ s) & (dv ^ r), 55)
		: BIT(~(dv ^ s) & (dv ^ r), 55);

	if (store)
		m_a[d] = r;

	flags_nz(r);
	m_sr &= ~(SR_C | SR_V);
	if (carry)
		m_sr |= SR_C;
	if (overflow || v_shift)
		m_sr |= SR_V | SR_L;
}

bool dsp56000_device_base::condition(unsigned cccc) const
{
	bool const c = m_sr & SR_C;
	bool const v = m_sr & SR_V;
	bool const z = m_sr & SR_Z;
	bool const n = m_sr & SR_N;
	bool const u = m_sr & SR_U;
	bool const e = m_sr & SR_E;
	bool const l = m_sr & SR_L;

	switch (cccc & 15)
	{
	case 0x0: return !c;
	case 0x1: return !(n ^ v);
	case 0x2: return !z;
	case 0x3: return !n;
	case 0x4: return !(z || (!u && !e));
	case 0x5: return !e;
	case 0x6: return !l;
	case 0x7: return !(z || (n ^ v));
	case 0x8: return c;
	case 0x9: return n ^ v;
	case 0xa: return z;
	case 0xb: return n;
	case 0xc: return z || (!u && !e);
	case 0xd: return e;
	case 0xe: return l;
	case 0xf: return z || (n ^ v);
	}

	return false;
}

void dsp56000_device_base::alu(u8 op)
{
	unsigned const d = BIT(op, 3);

	if (BIT(op, 7))
	{
		// multiply: 1QQQ dkkk
		static u8 const QQQ[8][2] = { { 4, 4 }, { 6, 6 }, { 5, 4 }, { 7, 6 }, { 4, 7 }, { 6, 4 }, { 5, 6 }, { 7, 5 } };
		u8 const *const q = QQQ[(op >> 4) & 7];
		u64 p = mul(reg_r(q[0]), reg_r(q[1]));
		if (BIT(op, 2))
			p = (0 - p) & M56;

		u64 r;
		bool overflow = false;
		if (BIT(op, 1))
		{
			// mac
			u64 const dv = m_a[d];
			r = (dv + p) & M56;
			overflow = BIT(~(dv ^ p) & (dv ^ r), 55);
		}
		else
			r = p;

		if (BIT(op, 0))
		{
			u64 const t = round(r);
			if (BIT(t, 55) != BIT(r, 55) && !BIT(r, 55))
				overflow = true;
			r = t & M56;
		}

		m_a[d] = r;
		flags_nz(r);
		m_sr &= ~SR_V;
		if (overflow)
			m_sr |= SR_V | SR_L;
		return;
	}

	unsigned const jjj = (op >> 4) & 7;
	unsigned const kkk = op & 7;
	unsigned const other = d ^ 1;

	// 24-bit source operand for JJJ = 4-7
	static u8 const JJJ24[4] = { 4, 6, 5, 7 }; // x0, y0, x1, y1

	auto logical = [this, d](u32 r)
	{
		m_a[d] = (m_a[d] & ~(u64(0xffffff) << 24)) | (u64(r & 0xffffff) << 24);
		m_sr &= ~(SR_N | SR_Z | SR_V);
		if (BIT(r, 23))
			m_sr |= SR_N;
		if (!(r & 0xffffff))
			m_sr |= SR_Z;
	};
	auto a1 = [this, d]() { return u32(m_a[d] >> 24) & 0xffffff; };

	switch (jjj)
	{
	case 0:
		switch (kkk)
		{
		case 0: // move (or reserved with d set)
			break;
		case 1: // tfr other,d
			m_a[d] = m_a[other];
			break;
		case 2: // addr other,d
			addsub(d, m_a[other], false, false, true, 2);
			break;
		case 3: // tst d
			flags_nz(m_a[d]);
			m_sr &= ~(SR_V | SR_C);
			break;
		case 4: // reserved
			break;
		case 5: // cmp other,d
			addsub(d, m_a[other], true, false, false, 0);
			break;
		case 6: // subr other,d
			addsub(d, m_a[other], true, false, true, 2);
			break;
		case 7: // cmpm other,d
		{
			u64 const s = u64(std::abs(sext56(m_a[other]))) & M56;
			u64 const saved = m_a[d];
			m_a[d] = u64(std::abs(sext56(m_a[d]))) & M56;
			addsub(d, s, true, false, false, 0);
			m_a[d] = saved;
			break;
		}
		}
		break;

	case 1:
		switch (kkk)
		{
		case 0: // add other,d
			addsub(d, m_a[other], false, false, true, 0);
			break;
		case 1: // rnd d
		{
			u64 const v = m_a[d];
			u64 const r = round(v) & M56;
			m_a[d] = r;
			flags_nz(r);
			m_sr &= ~SR_V;
			if (BIT(r, 55) != BIT(v, 55) && !BIT(v, 55))
				m_sr |= SR_V | SR_L;
			break;
		}
		case 2: // addl other,d
			addsub(d, m_a[other], false, false, true, 1);
			break;
		case 3: // clr d
			m_a[d] = 0;
			m_sr = (m_sr & ~(SR_E | SR_N | SR_V)) | SR_U | SR_Z;
			break;
		case 4: // sub other,d
			addsub(d, m_a[other], true, false, true, 0);
			break;
		case 5: // reserved
			break;
		case 6: // subl other,d
			addsub(d, m_a[other], true, false, true, 1);
			break;
		case 7: // not d
			logical(~a1());
			break;
		}
		break;

	case 2:
	case 3:
	{
		// source x1:x0 or y1:y0 as a sign-extended 48-bit value
		u32 const hi = jjj == 2 ? m_x[1] : m_y[1];
		u32 const lo = jjj == 2 ? m_x[0] : m_y[0];
		u64 const s = (sext24_56(hi) | lo) & M56;

		switch (kkk)
		{
		case 0: addsub(d, s, false, false, true, 0); break; // add
		case 1: addsub(d, s, false, true, true, 0); break;  // adc
		case 4: addsub(d, s, true, false, true, 0); break;  // sub
		case 5: addsub(d, s, true, true, true, 0); break;   // sbc
		case 2:
			if (jjj == 2)
			{
				// asr d
				u64 const v = m_a[d];
				u64 const r = u64(sext56(v) >> 1) & M56;
				m_a[d] = r;
				flags_nz(r);
				m_sr &= ~(SR_V | SR_C);
				if (BIT(v, 0))
					m_sr |= SR_C;
			}
			else
			{
				// asl d
				u64 const v = m_a[d];
				u64 const r = (v << 1) & M56;
				m_a[d] = r;
				flags_nz(r);
				m_sr &= ~(SR_V | SR_C);
				if (BIT(v, 55))
					m_sr |= SR_C;
				if (BIT(v, 55) != BIT(v, 54))
					m_sr |= SR_V | SR_L;
			}
			break;
		case 3:
		{
			u32 const v = a1();
			if (jjj == 2)
			{
				// lsr d
				logical(v >> 1);
				m_sr &= ~SR_C;
				if (BIT(v, 0))
					m_sr |= SR_C;
			}
			else
			{
				// lsl d
				logical(v << 1);
				m_sr &= ~SR_C;
				if (BIT(v, 23))
					m_sr |= SR_C;
			}
			break;
		}
		case 6:
		{
			u64 const v = m_a[d];
			s64 const sv = sext56(v);
			u64 r;
			if (jjj == 2)
				r = u64(sv < 0 ? -sv : sv) & M56; // abs d
			else
				r = (0 - v) & M56; // neg d
			m_a[d] = r;
			flags_nz(r);
			m_sr &= ~SR_V;
			if (v == 0x0080'0000'0000'0000ULL)
				m_sr |= SR_V | SR_L;
			break;
		}
		case 7:
		{
			u32 const v = a1();
			bool const c = m_sr & SR_C;
			if (jjj == 2)
			{
				// ror d
				logical((v >> 1) | (c ? 0x800000 : 0));
				m_sr &= ~SR_C;
				if (BIT(v, 0))
					m_sr |= SR_C;
			}
			else
			{
				// rol d
				logical((v << 1) | (c ? 1 : 0));
				m_sr &= ~SR_C;
				if (BIT(v, 23))
					m_sr |= SR_C;
			}
			break;
		}
		}
		break;
	}

	default:
	{
		// source x0, y0, x1 or y1
		u32 const sv = reg_r(JJJ24[jjj - 4]);
		u64 const s = sext24_56(sv);

		switch (kkk)
		{
		case 0: addsub(d, s, false, false, true, 0); break; // add
		case 1: acc_w24(d, sv); break; // tfr
		case 2: logical(a1() | sv); break; // or
		case 3: logical(a1() ^ sv); break; // eor
		case 4: addsub(d, s, true, false, true, 0); break; // sub
		case 5: addsub(d, s, true, false, false, 0); break; // cmp
		case 6: logical(a1() & sv); break; // and
		case 7: // cmpm
		{
			u64 const sa = u64(std::abs(sext56(s))) & M56;
			u64 const saved = m_a[d];
			m_a[d] = u64(std::abs(sext56(m_a[d]))) & M56;
			addsub(d, sa, true, false, false, 0);
			m_a[d] = saved;
			break;
		}
		}
		break;
	}
	}
}

//-------------------------------------------------
//  parallel moves
//-------------------------------------------------

void dsp56000_device_base::pend_reg(unsigned dddddd, u32 data)
{
	m_pending[m_pending_count++] = pending_write{ pending_write::REG, u16(dddddd), data };
}

void dsp56000_device_base::pend_mem(bool y, u16 address, u32 data)
{
	m_pending[m_pending_count++] = pending_write{ y ? pending_write::YMEM : pending_write::XMEM, address, data };
}

void dsp56000_device_base::commit()
{
	for (unsigned i = 0; i < m_pending_count; i++)
	{
		pending_write const &w = m_pending[i];
		switch (w.type)
		{
		case pending_write::REG: reg_w(w.address, w.data); break;
		case pending_write::XMEM: x_write(w.address, w.data); break;
		case pending_write::YMEM: y_write(w.address, w.data); break;
		}
	}
	m_pending_count = 0;
}

// Instructions with a data ALU operation in the low byte and a parallel
// data move in bits 8-23. All sources (registers and memory) are read
// before the ALU operation, all destinations written after it.
void dsp56000_device_base::execute_parallel(u32 op)
{
	m_pending_count = 0;

	if (BIT(op, 23))
	{
		// XY memory data move: 1wmm eeff WrrM MRRR
		static u8 const MM[4] = { 4, 1, 2, 3 }; // (Rn), (Rn)+Nn, (Rn)-, (Rn)+
		static u8 const EE[4] = { 0x04, 0x05, 0x0e, 0x0f };
		static u8 const FF[4] = { 0x06, 0x07, 0x0e, 0x0f };

		unsigned const xr = (op >> 8) & 7;
		unsigned const yr = ((op >> 13) & 3) | (BIT(xr, 2) ? 0 : 4);

		u16 const xa = ea_short((MM[(op >> 11) & 3] << 3) | xr);
		u16 const ya = ea_short((MM[(op >> 20) & 3] << 3) | yr);

		unsigned const xreg = EE[(op >> 18) & 3];
		unsigned const yreg = FF[(op >> 16) & 3];

		if (BIT(op, 15))
			pend_reg(xreg, x_read(xa));
		else
			pend_mem(false, xa, reg_r(xreg));

		if (BIT(op, 22))
			pend_reg(yreg, y_read(ya));
		else
			pend_mem(true, ya, reg_r(yreg));
	}
	else if (BIT(op, 22))
	{
		bool const W = BIT(op, 15);
		unsigned const field = (op >> 8) & 63;

		bool immediate = false, extended;
		u32 ext;
		u16 address;
		if (BIT(op, 14))
			address = ea(field, immediate, ext, extended);
		else
			address = field;

		if (op & 0x340000U)
		{
			// X or Y memory data move: 01dd Sddd W1MM MRRR / W0aa aaaa
			bool const y = BIT(op, 19);
			unsigned const reg = ((op >> 17) & 0x18) | ((op >> 16) & 7);

			if (W)
				pend_reg(reg, immediate ? ext : xy_read(y, address));
			else
				pend_mem(y, address, reg_r(reg));
		}
		else
		{
			// long memory data move: 0100 L0LL W1MM MRRR / W0aa aaaa
			unsigned const lll = ((op >> 17) & 4) | ((op >> 16) & 3);

			if (W)
			{
				u32 const xv = x_read(address);
				u32 const yv = y_read(address);
				switch (lll)
				{
				case 0: pend_reg(0x0c, xv); pend_reg(0x08, yv); break; // a10
				case 1: pend_reg(0x0d, xv); pend_reg(0x09, yv); break; // b10
				case 2: pend_reg(0x05, xv); pend_reg(0x04, yv); break; // x
				case 3: pend_reg(0x07, xv); pend_reg(0x06, yv); break; // y
				case 4: pend_reg(0x0e, xv); pend_reg(0x08, yv); break; // a
				case 5: pend_reg(0x0f, xv); pend_reg(0x09, yv); break; // b
				case 6: pend_reg(0x0e, xv); pend_reg(0x0f, yv); break; // ab
				case 7: pend_reg(0x0f, xv); pend_reg(0x0e, yv); break; // ba
				}
			}
			else
			{
				u32 xv = 0, yv = 0;
				switch (lll)
				{
				case 0: xv = reg_r(0x0c); yv = reg_r(0x08); break;
				case 1: xv = reg_r(0x0d); yv = reg_r(0x09); break;
				case 2: xv = m_x[1]; yv = m_x[0]; break;
				case 3: xv = m_y[1]; yv = m_y[0]; break;
				case 4:
				case 5:
				{
					// 48-bit limiting
					unsigned const acc = lll & 1;
					u32 const hi = acc_bus(acc);
					if (hi == ((m_a[acc] >> 24) & 0xffffff) && !((m_sr >> 10) & 3))
						yv = m_a[acc] & 0xffffff;
					else if (hi == 0x7fffff && ((m_a[acc] >> 24) & 0xffffff) != 0x7fffff)
						yv = 0xffffff;
					else if (hi == 0x800000 && ((m_a[acc] >> 24) & 0xffffff) != 0x800000)
						yv = 0x000000;
					else
						yv = m_a[acc] & 0xffffff;
					xv = hi;
					break;
				}
				case 6: xv = acc_bus(0); yv = acc_bus(1); break;
				case 7: xv = acc_bus(1); yv = acc_bus(0); break;
				}
				pend_mem(false, address, xv);
				pend_mem(true, address, yv);
			}
		}
	}
	else if (BIT(op, 21))
	{
		if (op & 0x1c0000U)
		{
			// immediate short data move: 001d dddd iiii iiii
			unsigned const reg = (op >> 16) & 31;
			u32 const imm = (op >> 8) & 0xff;

			// fractional for the data ALU registers, integer otherwise
			if ((reg >= 0x04 && reg <= 0x07) || reg == 0x0e || reg == 0x0f)
				pend_reg(reg, imm << 16);
			else
				pend_reg(reg, imm);
		}
		else
		{
			switch (op & 0x03e000U)
			{
			case 0x000000U: // no parallel data move
				break;
			case 0x004000U: // address register update
				ea_short((op >> 8) & 31);
				break;
			default:
			{
				// register to register data move: 0010 00ee eeed dddd
				unsigned const src = (op >> 13) & 31;
				unsigned const dst = (op >> 8) & 31;
				pend_reg(dst, reg_r(src));
				break;
			}
			}
		}
	}
	else if (BIT(op, 20))
	{
		unsigned const MMMRRR = (op >> 8) & 63;
		bool const W = BIT(op, 15);
		bool immediate, extended;
		u32 ext;

		if (BIT(op, 14))
		{
			// register and Y memory data move (class I): 0001 deff W1MM MRRR
			static u8 const FF[4] = { 0x06, 0x07, 0x0e, 0x0f };
			unsigned const s1 = BIT(op, 19);
			unsigned const d1 = BIT(op, 18) ? 0x05 : 0x04;
			unsigned const yreg = FF[(op >> 16) & 3];

			u32 const v1 = acc_bus(s1);
			u16 const address = ea(MMMRRR, immediate, ext, extended);
			if (W)
				pend_reg(yreg, immediate ? ext : y_read(address));
			else
				pend_mem(true, address, reg_r(yreg));
			pend_reg(d1, v1);
		}
		else
		{
			// X memory and register data move (class I): 0001 ffdf W0MM MRRR
			static u8 const EE[4] = { 0x04, 0x05, 0x0e, 0x0f };
			unsigned const xreg = EE[(op >> 18) & 3];
			unsigned const s2 = BIT(op, 17);
			unsigned const d2 = BIT(op, 16) ? 0x07 : 0x06;

			u32 const v2 = acc_bus(s2);
			u16 const address = ea(MMMRRR, immediate, ext, extended);
			if (W)
				pend_reg(xreg, immediate ? ext : x_read(address));
			else
				pend_mem(false, address, reg_r(xreg));
			pend_reg(d2, v2);
		}
	}
	else
	{
		// class II: 0000 100d x0MM MRRR
		unsigned const d = BIT(op, 16);
		unsigned const MMMRRR = (op >> 8) & 63;
		u16 const address = ea_short(MMMRRR & 0x3f);

		if (BIT(op, 15))
		{
			// y0,d d,Y:ea
			pend_mem(true, address, acc_bus(d));
			pend_reg(0x0e | d, m_y[0]);
		}
		else
		{
			// d,X:ea x0,d
			pend_mem(false, address, acc_bus(d));
			pend_reg(0x0e | d, m_x[0]);
		}
	}

	alu(op & 0xff);
	commit();
}

//-------------------------------------------------
//  program control
//-------------------------------------------------

void dsp56000_device_base::jump_subroutine(u16 target)
{
	if (m_fast_irq)
	{
		// a subroutine call in an interrupt vector makes it a long
		// interrupt: return to the interrupted program, mask interrupts of
		// this and lower priority
		push(m_irq_return, m_sr);
		unsigned const mask = std::min(m_irq_level + 1, 3U);
		m_sr = (m_sr & ~(SR_I | SR_LF | SR_T)) | (mask << 8);
		m_fast_irq = 0;
	}
	else
		push(m_next_pc, m_sr);

	branch(target);
}

void dsp56000_device_base::do_loop(u16 count, u16 last)
{
	push(m_la, m_lc);
	m_la = last;
	m_lc = count;
	push(m_next_pc, m_sr);
	m_sr |= SR_LF;
}

void dsp56000_device_base::rep_start(u16 count)
{
	m_rep_lc = m_lc;
	m_lc = count;
	m_rep = true;
	m_rep_pc = m_next_pc;
}

void dsp56000_device_base::bit_op(u32 op)
{
	unsigned const bit = op & 31;
	unsigned const field = (op >> 8) & 63;
	bool const y = BIT(op, 6);

	// register forms and jumps through an effective address
	if ((op & 0xc000U) == 0xc000U)
	{
		if (BIT(op, 7))
		{
			// jmp/jcc/jsr/jscc ea
			bool immediate, extended;
			u32 ext;
			u16 const target = ea(field, immediate, ext, extended);
			bool const cond = !BIT(op, 5) || condition(op & 15);
			m_icount--;
			if (cond)
			{
				if (BIT(op, 16))
					jump_subroutine(target);
				else
					branch(target);
			}
			return;
		}

		u32 const v = reg_r(field);
		bool const set = BIT(v, bit);
		u32 r = v;

		switch (op & 0x010060U)
		{
		case 0x000000U: // jclr #n,S,xxxx
		case 0x000020U: // jset
		case 0x010000U: // jsclr
		case 0x010020U: // jsset
		{
			u16 const target = fetch(m_next_pc);
			m_last = m_next_pc++;
			m_icount -= 2;
			m_sr = (m_sr & ~SR_C) | (set ? SR_C : 0);
			if (set == BIT(op, 5))
			{
				if (BIT(op, 16))
					jump_subroutine(target);
				else
					branch(target);
			}
			return;
		}
		case 0x000040U: r &= ~(1U << bit); break; // bclr
		case 0x000060U: r |= 1U << bit; break;    // bset
		case 0x010040U: r ^= 1U << bit; break;    // bchg
		case 0x010060U: break;                     // btst
		}

		if ((op & 0x010060U) != 0x010060U)
			reg_w(field, r);

		// the status register keeps the modified bit except for C
		if (field != 0x39 || bit != 0)
			m_sr = (m_sr & ~SR_C) | (set ? SR_C : 0);
		return;
	}

	// memory forms: absolute short, effective address or peripheral
	u16 address;
	switch (op & 0xc000U)
	{
	case 0x0000U: address = field; break;
	case 0x4000U:
	{
		bool immediate, extended;
		u32 ext;
		address = ea(field, immediate, ext, extended);
		break;
	}
	default: address = 0xffc0 | field; break;
	}

	u32 const v = xy_read(y, address);
	bool const set = BIT(v, bit);
	m_sr = (m_sr & ~SR_C) | (set ? SR_C : 0);

	switch (op & 0x0100a0U)
	{
	case 0x000000U: xy_write(y, address, v & ~(1U << bit)); break; // bclr
	case 0x000020U: xy_write(y, address, v | (1U << bit)); break;  // bset
	case 0x010000U: xy_write(y, address, v ^ (1U << bit)); break;  // bchg
	case 0x010020U: break; // btst
	default:
	{
		// jclr/jset/jsclr/jsset #n,X:ea,xxxx
		u16 const target = fetch(m_next_pc);
		m_last = m_next_pc++;
		m_icount -= 2;
		if (set == BIT(op, 5))
		{
			if (BIT(op, 16))
				jump_subroutine(target);
			else
				branch(target);
		}
		break;
	}
	}
}

void dsp56000_device_base::execute_nonparallel(u32 op)
{
	switch ((op >> 16) & 15)
	{
	case 0x0:
		switch (op & 0xff)
		{
		case 0x00: break; // nop
		case 0x04: // rti
		{
			u16 pc, sr;
			pop(pc, sr);
			branch(pc);
			sr_w(sr);
			m_icount--;
			break;
		}
		case 0x05: // illegal
			m_irq_pending |= 1U << (VEC_ILLEGAL >> 1);
			break;
		case 0x06: // swi
			m_irq_pending |= 1U << (VEC_SWI >> 1);
			break;
		case 0x0c: // rts
		{
			u16 pc, sr;
			pop(pc, sr);
			branch(pc);
			m_icount--;
			break;
		}
		case 0x84: // reset
			reset_peripherals();
			break;
		case 0x86: // wait
		case 0x87: // stop
			m_wait = true;
			break;
		case 0x8c: // enddo
		{
			u16 pc, sr;
			pop(pc, sr);
			m_sr = (m_sr & ~SR_LF) | (sr & SR_LF);
			pop(m_la, m_lc);
			break;
		}
		case 0xf8: m_sr = (m_sr | ((op & 0xff00) >> 0)) & 0xaf7f; break; // ori #xx,mr
		case 0xf9: m_sr |= (op >> 8) & 0xff; break;                       // ori #xx,ccr
		case 0xfa: m_omr |= (op >> 8) & 0xc7; break;                       // ori #xx,omr
		case 0xb8: m_sr &= ((op & 0xff00) | 0x00ff); break;               // andi #xx,mr
		case 0xb9: m_sr &= (((op >> 8) & 0xff) | 0xff00); break;          // andi #xx,ccr
		case 0xba: m_omr &= (op >> 8) & 0xff; break;                       // andi #xx,omr
		default:
			m_irq_pending |= 1U << (VEC_ILLEGAL >> 1);
			break;
		}
		break;

	case 0x1:
		switch (op & 0x00f8c7U)
		{
		case 0x008040U: // div s,d
		{
			static u8 const JJ[4] = { 0x04, 0x06, 0x05, 0x07 }; // x0, y0, x1, y1
			unsigned const d = BIT(op, 3);
			u32 const s = reg_r(JJ[(op >> 4) & 3]);
			u64 const dv = m_a[d];

			bool const differ = BIT(dv, 55) != BIT(s, 23);
			u64 const shifted = (dv << 1) & M56;
			u64 const sa = sext24_56(s);
			u64 r = (differ ? shifted + sa : shifted - sa) & M56;
			r |= (m_sr & SR_C) ? 1 : 0;
			m_a[d] = r;

			m_sr &= ~(SR_C | SR_V);
			if (!BIT(r, 55))
				m_sr |= SR_C;
			if (BIT(dv, 55) != BIT(shifted, 55))
				m_sr |= SR_V | SR_L;
			break;
		}
		case 0x00d805U: // norm rn,d
		{
			unsigned const d = BIT(op, 3);
			unsigned const rn = (op >> 8) & 7;
			u64 const dv = m_a[d];
			bool const e = m_sr & SR_E;
			bool const u = m_sr & SR_U;
			bool const z = m_sr & SR_Z;
			u64 r = dv;
			bool v = false;

			if (e)
			{
				r = u64(sext56(dv) >> 1) & M56;
				m_r[rn]++;
			}
			else if (u && !z)
			{
				r = (dv << 1) & M56;
				v = BIT(r, 55) != BIT(dv, 55);
				m_r[rn]--;
			}

			m_a[d] = r;
			flags_nz(r);
			m_sr &= ~SR_V;
			if (v)
				m_sr |= SR_V | SR_L;
			break;
		}
		default:
			m_irq_pending |= 1U << (VEC_ILLEGAL >> 1);
			break;
		}
		break;

	case 0x2: // tcc s1,d1
	case 0x3: // tcc s1,d1 s2,d2
		if (condition((op >> 12) & 15))
		{
			unsigned const jjjd = (op >> 3) & 15;
			unsigned const d = jjjd & 1;
			switch (jjjd >> 1)
			{
			case 0: m_a[d] = m_a[d ^ 1]; break; // b,a or a,b
			case 4: acc_w24(d, m_x[0]); break;
			case 5: acc_w24(d, m_y[0]); break;
			case 6: acc_w24(d, m_x[1]); break;
			case 7: acc_w24(d, m_y[1]); break;
			}

			if (BIT(op, 16))
				m_r[op & 7] = m_r[(op >> 8) & 7];
		}
		break;

	case 0x4:
		switch (op & 0x0000e0U)
		{
		case 0x000000U:
		{
			// lua ea,d: the updated address without changing Rn
			unsigned const rrr = (op >> 8) & 7;
			u16 const saved = m_r[rrr];
			ea_short((op >> 8) & 31);
			u16 const updated = m_r[rrr];
			m_r[rrr] = saved;
			reg_w(op & 31, updated);
			break;
		}
		case 0x0000a0U:
			// movec s1,d1 (register to or from control register)
			if (BIT(op, 15))
			{
				u32 const v = reg_r((op >> 8) & 63);
				reg_w(op & 63, v);
			}
			else
			{
				u32 const v = reg_r(op & 63);
				reg_w((op >> 8) & 63, v);
			}
			break;
		default:
			m_irq_pending |= 1U << (VEC_ILLEGAL >> 1);
			break;
		}
		break;

	case 0x5:
		if (BIT(op, 7))
			// movec #xx,d1
			reg_w(op & 63, (op >> 8) & 0xff);
		else
		{
			bool const y = BIT(op, 6);
			u16 address;
			bool immediate = false, extended;
			u32 ext = 0;

			if (BIT(op, 14))
				address = ea((op >> 8) & 63, immediate, ext, extended);
			else
				address = (op >> 8) & 63;

			if (BIT(op, 15))
				reg_w(op & 63, immediate ? ext : xy_read(y, address));
			else
				xy_write(y, address, reg_r(op & 63));
		}
		break;

	case 0x6:
	{
		// rep or do
		u16 count;
		if (BIT(op, 7))
			count = ((op & 15) << 8) | ((op >> 8) & 0xff);
		else
		{
			bool const y = BIT(op, 6);
			unsigned const field = (op >> 8) & 63;
			switch (op & 0x00c000U)
			{
			case 0x000000U: count = xy_read(y, field); break;
			case 0x004000U:
			{
				bool immediate, extended;
				u32 ext;
				count = xy_read(y, ea(field, immediate, ext, extended));
				break;
			}
			default: count = reg_r(field); break;
			}
		}

		if (BIT(op, 5))
			rep_start(count);
		else
		{
			u16 const last = fetch(m_next_pc);
			m_last = m_next_pc++;
			m_icount -= 2;
			do_loop(count, last);
		}
		break;
	}

	case 0x7:
	{
		// movem: program memory
		u16 address;
		if (BIT(op, 7))
		{
			bool immediate, extended;
			u32 ext;
			address = ea((op >> 8) & 63, immediate, ext, extended);
		}
		else
			address = (op >> 8) & 63;

		m_icount--;
		if (BIT(op, 15))
			reg_w(op & 63, p_read(address));
		else
			p_write(address, reg_r(op & 63));
		break;
	}

	case 0x8:
	case 0x9:
	{
		// movep: peripheral at X or Y:$FFC0-$FFFF
		u16 const pp = 0xffc0 | (op & 63);
		bool const ps = BIT(op, 16);
		bool const W = BIT(op, 15);
		unsigned const field = (op >> 8) & 63;

		m_icount--;
		if (BIT(op, 7))
		{
			// X or Y memory
			bool immediate, extended;
			u32 ext;
			bool const ms = BIT(op, 6);
			u16 const address = ea(field, immediate, ext, extended);
			if (W)
				xy_write(ps, pp, immediate ? ext : xy_read(ms, address));
			else
				xy_write(ms, address, xy_read(ps, pp));
		}
		else if (BIT(op, 6))
		{
			// program memory
			bool immediate, extended;
			u32 ext;
			u16 const address = ea(field, immediate, ext, extended);
			if (W)
				xy_write(ps, pp, p_read(address));
			else
				p_write(address, xy_read(ps, pp));
		}
		else
		{
			// register
			if (W)
				xy_write(ps, pp, reg_r(field));
			else
				reg_w(field, xy_read(ps, pp));
		}
		break;
	}

	case 0xa:
	case 0xb:
		bit_op(op);
		break;

	case 0xc: // jmp xxx
		branch(op & 0xfff);
		m_icount--;
		break;
	case 0xd: // jsr xxx
		m_icount--;
		jump_subroutine(op & 0xfff);
		break;
	case 0xe: // jcc xxx
		m_icount--;
		if (condition((op >> 12) & 15))
			branch(op & 0xfff);
		break;
	case 0xf: // jscc xxx
		m_icount--;
		if (condition((op >> 12) & 15))
			jump_subroutine(op & 0xfff);
		break;
	}
}

//-------------------------------------------------
//  execution
//-------------------------------------------------

void dsp56000_device_base::interrupt_check()
{
	// collect the level of each pending source: -1 masked or disabled
	struct source { u8 vector; int level; };
	source sources[16];
	unsigned count = 0;

	auto ipl = [this](unsigned shift) { return int((m_ipr >> shift) & 3) - 1; };

	// non-maskable
	for (u8 vector : { VEC_STACK, VEC_ILLEGAL, VEC_SWI, VEC_TRACE })
		if (BIT(m_irq_pending, vector >> 1))
			sources[count++] = { vector, 3 };

	// IRQA and IRQB: level triggered unless IAL2/IBL2 select edge
	if (ipl(0) >= 0)
	{
		bool const edge = BIT(m_ipr, 2);
		if (edge ? BIT(m_irq_pending, VEC_IRQA >> 1) : (m_irqa_state != CLEAR_LINE))
			sources[count++] = { VEC_IRQA, ipl(0) };
	}
	if (ipl(3) >= 0)
	{
		bool const edge = BIT(m_ipr, 5);
		if (edge ? BIT(m_irq_pending, VEC_IRQB >> 1) : (m_irqb_state != CLEAR_LINE))
			sources[count++] = { VEC_IRQB, ipl(3) };
	}

	// SSI
	if (ipl(12) >= 0)
	{
		if ((m_crb & CRB_RIE) && (m_ssisr & SSISR_ROE))
			sources[count++] = { VEC_SSI_RXE, ipl(12) };
		else if ((m_crb & CRB_RIE) && (m_ssisr & SSISR_RDF))
			sources[count++] = { VEC_SSI_RX, ipl(12) };
		if ((m_crb & CRB_TIE) && (m_ssisr & SSISR_TUE))
			sources[count++] = { VEC_SSI_TXE, ipl(12) };
		else if ((m_crb & CRB_TIE) && (m_ssisr & SSISR_TDE))
			sources[count++] = { VEC_SSI_TX, ipl(12) };
	}

	// SCI timer
	if (ipl(14) >= 0 && BIT(m_scr, 13) && BIT(m_irq_pending, VEC_SCI_TMR >> 1))
		sources[count++] = { VEC_SCI_TMR, ipl(14) };

	if (!count)
		return;

	int const mask = (m_sr >> 8) & 3;
	int best = -1;
	for (unsigned i = 0; i < count; i++)
		if (sources[i].level >= mask && (best < 0 || sources[i].level > sources[best].level))
			best = i;

	if (best >= 0)
		interrupt_take(sources[best].vector, sources[best].level);
}

void dsp56000_device_base::interrupt_take(u8 vector, unsigned level)
{
	// latched sources are cleared when serviced
	switch (vector)
	{
	case VEC_STACK: case VEC_ILLEGAL: case VEC_SWI: case VEC_TRACE: case VEC_SCI_TMR:
	case VEC_IRQA: case VEC_IRQB:
		m_irq_pending &= ~(1U << (vector >> 1));
		break;
	}

	m_irq_return = m_pc;
	m_pc = vector;
	m_fast_irq = 2;
	m_irq_level = std::min(level, 2U);
	m_wait = false;
}

void dsp56000_device_base::execute_one()
{
	u16 const start = m_pc;
	u32 const op = fetch(start);
	bool const rep_iteration = m_rep && start == m_rep_pc;
	u8 const fast_irq = m_fast_irq;

	m_next_pc = start + 1;
	m_last = start;
	m_jump = false;
	m_icount--;

	if (!(op >> 20) && (op & 0x0e4000U) != 0x080000U)
		execute_nonparallel(op);
	else
		execute_parallel(op);

	if (!m_jump)
		m_pc = m_next_pc;
	else if (m_pc == start && !m_fast_irq && !m_rep)
	{
		// a jump to itself (a halt, or polling a flag that only an
		// interrupt or a peripheral clocked from outside can change) has
		// nothing more to do until the next event
		m_icount -= std::min<int>(m_icount, cycles_to_event());
	}

	if (rep_iteration)
	{
		if (--m_lc == 0)
		{
			m_rep = false;
			m_lc = m_rep_lc;
		}
		else
			m_pc = start;
	}
	else if (fast_irq && m_fast_irq)
	{
		// fast interrupt: two words in the vector, then back
		unsigned const words = m_next_pc - start;
		m_fast_irq = (words >= m_fast_irq) ? 0 : m_fast_irq - words;
		if (!m_fast_irq && !m_jump)
			m_pc = m_irq_return;
		return;
	}

	if (!m_rep && !m_jump && (m_sr & SR_LF) && m_last == m_la)
		loop_end();
}

// After the last instruction of a hardware loop: next iteration, or restore
// the enclosing loop's state.
void dsp56000_device_base::loop_end()
{
	if (m_lc == 1)
	{
		u16 pc, sr;
		pop(pc, sr);
		m_sr = (m_sr & ~SR_LF) | (sr & SR_LF);
		pop(m_la, m_lc);
	}
	else
	{
		m_lc--;
		m_pc = m_ssh[m_sp & 15];
	}
}

void dsp56000_device_base::execute_run()
{
	m_event_cycles = total_cycles();
	advance();

	while (m_icount > 0)
	{
		if (!m_rep && !m_fast_irq && irq_possible())
			interrupt_check();

		if (m_wait)
		{
			// sleep until the next internal event, or the timeslice's end
			m_icount -= std::min<int>(m_icount, cycles_to_event());
			advance();
			continue;
		}

		m_ppc = m_pc;
		debugger_instruction_hook(m_pc);

		// fast path for a repeated nop
		if (m_rep && m_pc == m_rep_pc && fetch(m_pc) == 0)
		{
			u32 const n = std::min<u32>({ m_lc ? m_lc : 0x10000u, u32(std::max(m_icount, 1)), cycles_to_event() });
			m_icount -= n;
			m_lc -= n;
			if (!m_lc)
			{
				m_rep = false;
				m_lc = m_rep_lc;
				m_pc++;
				// a repeated nop ending a hardware loop
				if ((m_sr & SR_LF) && u16(m_pc - 1) == m_la)
					loop_end();
			}
			advance();
			continue;
		}

		execute_one();

		// an event is due, or the timeslice was aborted (the icount zeroed)
		if (m_icount <= m_event_icount)
			advance();
	}

	// account the rest of the timeslice
	advance();
}
