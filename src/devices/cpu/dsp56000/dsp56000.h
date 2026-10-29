// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay, blythe

#ifndef MAME_CPU_DSP56000_DSP56000_H
#define MAME_CPU_DSP56000_DSP56000_H

#pragma once

class dsp56000_device_base : public cpu_device
{
public:
	enum
	{
		IRQA_LINE = 0, // MODA/IRQA
		IRQB_LINE = 1, // MODB/IRQB
	};

	// operating mode sampled from MODA/MODB at reset (0-3)
	void set_mode(u8 mode) { m_mode_pins = mode & 3; }

	// port B general purpose I/O (15 bits): the write callback gets the
	// output latch, its mask the data direction register (1 = output)
	auto portb_r() { return m_portb_r.bind(); }
	auto portb_w() { return m_portb_w.bind(); }

	// synchronous serial interface: words are 24 bits, most significant
	// bit first, as left in the transmit and receive registers
	auto ssi_tx() { return m_ssi_tx.bind(); }
	auto ssi_rx() { return m_ssi_rx.bind(); }

	// one SSI word slot, clocked from outside (SCKD and FSL configured
	// for external clock and frame sync); transmits and receives one word
	void ssi_slot();

	// or clock the SSI from outside at a fixed word rate (0 stops it): the
	// slots then happen at their exact cycles within the DSP's own
	// execution, with no scheduler timer
	void set_ssi_rate(u32 words_per_second);

	// instruction cycles until the next internally clocked event (SSI slot
	// or SCI timer), for idle skips that must not run past it
	u32 cycles_to_event() const;

	enum
	{
		DSP_PC = 1, DSP_SR, DSP_OMR, DSP_SP, DSP_LA, DSP_LC,
		DSP_X0, DSP_X1, DSP_Y0, DSP_Y1, DSP_A, DSP_B,
		DSP_R0, DSP_R1, DSP_R2, DSP_R3, DSP_R4, DSP_R5, DSP_R6, DSP_R7,
		DSP_N0, DSP_N1, DSP_N2, DSP_N3, DSP_N4, DSP_N5, DSP_N6, DSP_N7,
		DSP_M0, DSP_M1, DSP_M2, DSP_M3, DSP_M4, DSP_M5, DSP_M6, DSP_M7,
		DSP_IPR, DSP_BCR,
	};

protected:
	dsp56000_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock);

	// device_t overrides
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_execute_interface overrides
	virtual u64 execute_clocks_to_cycles(u64 clocks) const noexcept override { return (clocks + 2 - 1) / 2; }
	virtual u64 execute_cycles_to_clocks(u64 cycles) const noexcept override { return (cycles * 2); }
	virtual u32 execute_min_cycles() const noexcept override { return 1; }
	virtual u32 execute_max_cycles() const noexcept override { return 16; }
	virtual void execute_run() override;
	virtual void execute_set_input(int inputnum, int state) override;

	// device_memory_interface overrides
	virtual space_config_vector memory_space_config() const override;

	// device_disasm_interface overrides
	virtual std::unique_ptr<util::disasm_interface> create_disassembler() override;

	// device_state_interface overrides
	virtual void state_import(device_state_entry const &entry) override;
	virtual void state_export(device_state_entry const &entry) override;

private:
	// status register bits
	enum : u16
	{
		SR_C  = 0x0001,
		SR_V  = 0x0002,
		SR_Z  = 0x0004,
		SR_N  = 0x0008,
		SR_U  = 0x0010,
		SR_E  = 0x0020,
		SR_L  = 0x0040,
		SR_I  = 0x0300, // interrupt mask
		SR_S  = 0x0c00, // scaling mode
		SR_T  = 0x2000, // trace
		SR_LF = 0x8000, // loop flag
	};

	// interrupt vectors (program memory word addresses)
	enum : u8
	{
		VEC_RESET   = 0x00,
		VEC_STACK   = 0x02,
		VEC_TRACE   = 0x04,
		VEC_SWI     = 0x06,
		VEC_IRQA    = 0x08,
		VEC_IRQB    = 0x0a,
		VEC_SSI_RX  = 0x0c,
		VEC_SSI_RXE = 0x0e,
		VEC_SSI_TX  = 0x10,
		VEC_SSI_TXE = 0x12,
		VEC_SCI_RX  = 0x14,
		VEC_SCI_RXE = 0x16,
		VEC_SCI_TX  = 0x18,
		VEC_SCI_IDL = 0x1a,
		VEC_SCI_TMR = 0x1c,
		VEC_ILLEGAL = 0x3e,
	};

	// memory
	u32 p_read(u16 address);
	void p_write(u16 address, u32 data);
	u32 x_read(u16 address);
	void x_write(u16 address, u32 data);
	u32 y_read(u16 address);
	void y_write(u16 address, u32 data);
	u32 xy_read(bool y, u16 address) { return y ? y_read(address) : x_read(address); }
	void xy_write(bool y, u16 address, u32 data) { if (y) y_write(address, data); else x_write(address, data); }
	u32 fetch(u16 address) { return p_read(address); }

	void build_pages(address_space &space, u32 *(&r)[256], u32 *(&w)[256]);

	// on-chip peripherals (X:$FFC0-$FFFF)
	u32 periph_r(u16 address);
	void periph_w(u16 address, u32 data);
	void portb_update();
	void sci_timer_update();
	void advance();
	void advance_events(u64 cycles);
	u32 counted_cycles_to_event() const;

	// registers
	u32 reg_r(unsigned dddddd);
	void reg_w(unsigned dddddd, u32 data);
	u32 acc_bus(unsigned d); // accumulator through the data shifter and limiter
	void acc_w24(unsigned d, u32 data); // 24-bit value into an accumulator
	u16 sr() const { return m_sr; }
	void sr_w(u16 data) { m_sr = data & 0xaf7f; }
	void push(u16 high, u16 low);
	void pop(u16 &high, u16 &low);
	u16 ssh_pop();
	void ssh_push(u16 data);

	// address generation
	u16 agu_update(unsigned rrr, s32 delta);
	u16 ea(unsigned MMMRRR, bool &immediate, u32 &ext, bool &extended);
	u16 ea_short(unsigned MMMRRR); // effective address without extension word

	// data arithmetic logic unit
	void alu(u8 op);
	void flags_nz(u64 r); // E, U, N and Z from a 56-bit result
	u64 round(u64 v);
	u64 mul(u32 s1, u32 s2);
	void addsub(unsigned d, u64 s, bool sub, bool carry_in, bool store, unsigned shift_mode);
	bool condition(unsigned cccc) const;

	// instruction execution
	void execute_one();
	void execute_parallel(u32 op);
	void execute_nonparallel(u32 op);
	void jump_subroutine(u16 target);
	void branch(u16 target) { m_pc = target; m_jump = true; }
	void loop_end();
	void bit_op(u32 op);
	void do_loop(u16 count, u16 last);
	void rep_start(u16 count);
	void interrupt_check();

	// whether any interrupt source can be pending: a latched source, an IRQ
	// line asserted, or an enabled SSI condition
	bool irq_possible() const
	{
		return m_irq_pending || m_irqa_state != CLEAR_LINE || m_irqb_state != CLEAR_LINE
			|| ((m_crb & 0x8000) && (m_ssisr & 0xa0)) // RIE: RDF, ROE
			|| ((m_crb & 0x4000) && (m_ssisr & 0x50)); // TIE: TDE, TUE
	}
	void interrupt_take(u8 vector, unsigned level);
	void reset_peripherals();

	// pending parallel move writes
	struct pending_write
	{
		enum : u8 { REG, XMEM, YMEM } type;
		u16 address;
		u32 data;
	};
	void pend_reg(unsigned dddddd, u32 data);
	void pend_mem(bool y, u16 address, u32 data);
	void commit();

	// configuration
	address_space_config m_p_config;
	address_space_config m_x_config;
	address_space_config m_y_config;

	memory_access<16, 2, -2, ENDIANNESS_BIG>::cache m_pmem;
	memory_access<16, 2, -2, ENDIANNESS_BIG>::cache m_xmem;
	memory_access<16, 2, -2, ENDIANNESS_BIG>::cache m_ymem;

	// direct pointers to plain-memory pages, null for the others
	u32 *m_page_pr[256], *m_page_pw[256];
	u32 *m_page_xr[256], *m_page_xw[256];
	u32 *m_page_yr[256], *m_page_yw[256];

	devcb_read16 m_portb_r;
	devcb_write16 m_portb_w;
	devcb_write32 m_ssi_tx;
	devcb_read32 m_ssi_rx;

	u8 m_mode_pins;

	// internally clocked events: instruction cycles per event and until the
	// next one, in 32.32 fixed point; a period of 0 is stopped
	u64 m_ssi_period;
	s64 m_ssi_count;
	u64 m_sci_period;
	s64 m_sci_count;
	u64 m_event_cycles; // total_cycles() at the last advance()
	int m_event_icount; // the icount at which the next event is due

	// emulation state
	int m_icount;
	u16 m_pc;
	u16 m_ppc;
	u16 m_next_pc; // address following the current instruction
	u16 m_last;    // address of the current instruction's last word
	bool m_jump;   // the current instruction changed the program counter

	u32 m_x[2];
	u32 m_y[2];
	u64 m_a[2]; // 56-bit accumulators a and b
	u16 m_r[8];
	u16 m_n[8];
	u16 m_m[8];
	u16 m_sr;
	u8 m_omr;
	u8 m_sp;
	u16 m_la;
	u16 m_lc;
	u16 m_ssh[16];
	u16 m_ssl[16];

	// repeat state: the instruction at m_rep_pc runs until LC reaches 0
	bool m_rep;
	u16 m_rep_pc;
	u16 m_rep_lc;

	// interrupts
	u32 m_irq_pending; // one bit per vector pair (vector >> 1)
	int m_irqa_state;
	int m_irqb_state;
	u8 m_fast_irq;     // remaining words of a fast interrupt being run
	u16 m_irq_return;  // address the interrupted program resumes at
	unsigned m_irq_level;
	bool m_wait;       // WAIT or STOP until an interrupt

	pending_write m_pending[3];
	unsigned m_pending_count;

	// on-chip memory
	u32 m_pram[512];
	u32 m_xram[256];
	u32 m_yram[256];
	u32 m_xrom[256];
	u32 m_yrom[256];

	// peripherals
	u16 m_ipr;
	u16 m_bcr;
	u16 m_pbc;
	u16 m_pbddr;
	u16 m_pbd;
	u16 m_pcc;
	u16 m_pcddr;
	u16 m_pcd;
	u16 m_cra;
	u16 m_crb;
	u8 m_ssisr;
	u8 m_tsr_flag; // SSISR read since the last status event (clears TUE/ROE)
	u32 m_tx;
	u32 m_rx;
	u8 m_ssi_slot;
	u16 m_scr;
	u16 m_ssr;
	u16 m_sccr;
	u32 m_host[8];
};

class dsp56000_device : public dsp56000_device_base
{
public:
	dsp56000_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
};

class dsp56001_device : public dsp56000_device_base
{
public:
	dsp56001_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
};

DECLARE_DEVICE_TYPE(DSP56000, dsp56000_device)
DECLARE_DEVICE_TYPE(DSP56001, dsp56001_device)

#endif // MAME_CPU_DSP56000_DSP56000_H
