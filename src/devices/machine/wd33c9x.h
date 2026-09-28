// license:BSD-3-Clause
// copyright-holders:Tyson Smith
/*
 * wd33c9x.h
 */

#ifndef MAME_MACHINE_WD33C9X_H
#define MAME_MACHINE_WD33C9X_H

#pragma once

#include "machine/nscsi_bus.h"

// Coarser scheduling for bulk DATA_IN/DATA_OUT byte transfers: the
// per-byte REQ/ACK settle wait normally arms a real host-scheduler timer
// once for every single transferred byte, which dominates host CPU time
// during large disk transfers even though the guest driver never
// actually observes anything about the individual bytes going by (it
// only sees the final completion interrupt and register state). When
// enabled and only while a transfer is in progress in demand-mode DMA
// (never plain-polled I/O, which does depend on real per-byte register
// visibility), this skips arming that timer for most bytes and instead
// resumes the transfer state machine immediately, accumulating the
// total elapsed-cycle count until the run either leaves that narrow
// condition or hits a safety cap, at which point one real timer is
// armed for the accumulated total. Every REQ/ACK/FIFO/DRQ/IRQ side
// effect the guest can actually observe still happens once per byte,
// identically either way - only how the "wait, then resume" step
// between bytes is driven changes. Enabled by default as of 2026-09-19
// after validation (real-world data-integrity and throughput testing,
// see wiki/scsi-coarse-schedule-scoping.md): read determinism confirmed,
// ~2x real disk throughput measured, boot-to-login regression clean.
// The plain one-real-timer-per-byte path below remains the permanent
// fallback and is what runs whenever this is undefined - a reversible,
// side-by-side toggle, not a replacement, so it can still be disabled
// here if a future regression is ever suspected.
#define MAME_SGI_SCSI_COARSE_SCHEDULE

class wd33c9x_base_device : public device_t, public nscsi_device_interface
{
public:
	auto irq_cb() { return m_irq_cb.bind(); }
	auto drq_cb() { return m_drq_cb.bind(); }

	// Direct Addressing Interface
	uint8_t dir_r(offs_t offset);
	void dir_w(offs_t offset, uint8_t data);

	// Indirect Addressing Interface
	uint8_t indir_r(offs_t offset);
	void indir_w(offs_t offset, uint8_t data);

	// Alternative Indirect Addressing Interface
	uint8_t indir_addr_r();
	void indir_addr_w(uint8_t data);
	uint8_t indir_reg_r();
	void indir_reg_w(uint8_t data);

	uint8_t status_r();

	// Master Reset (MR) Interface
	void reset_w(int state);

	// DMA Interface (for use with DRQ)
	uint8_t dma_r();
	void dma_w(const uint8_t data);

protected:
	wd33c9x_base_device(const machine_config &mconfig, device_type type, const char *tag, device_t *owner, uint32_t clock);

	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	TIMER_CALLBACK_MEMBER(update_step);

	virtual void scsi_ctrl_changed() override;

private:
	static const char *const state_names[];
	static const char *const substate_names[];
	static constexpr uint8_t NUM_REGS = 0x20;
	static constexpr uint8_t REGS_MASK = NUM_REGS - 1;
	uint8_t m_addr;
	uint8_t m_regs[NUM_REGS];
	uint8_t m_command_length;
	uint8_t m_last_message;

	void start_command();

	// Every genuinely external entry point into the state machine (a
	// real timer firing, a SCSI bus line changing, a register access,
	// a freshly dispatched command) must call step_entry(), never step()
	// directly - it's the one place that knows how to keep resuming the
	// machine immediately when MAME_SGI_SCSI_COARSE_SCHEDULE decides to
	// skip arming a real timer (see delay()), so a coarsening decision
	// made deep inside any of those call paths can't get silently
	// dropped. step() calling itself internally (its own same-call-stack
	// cascades that don't need a real wait) is unaffected and keeps
	// calling step() directly, not step_entry() - only entry from
	// outside step()'s own body needs the wrapper.
	void step_entry(bool timeout);
	void step(bool timeout);
	void set_scsi_state(uint16_t state);
	void set_scsi_state_sub(uint8_t sub);
	uint16_t m_scsi_state;
	uint8_t m_mode;
	uint8_t m_xfr_phase;

	void load_transfer_count();
	bool decrement_transfer_count();
	uint32_t m_transfer_count;

	uint8_t data_fifo_pop();
	void data_fifo_push(const uint8_t data);
	bool data_fifo_empty() const;
	bool data_fifo_full() const;
	void data_fifo_reset();
	static constexpr uint8_t DATA_FIFO_SIZE = 12;
	uint8_t m_data_fifo[DATA_FIFO_SIZE];
	uint8_t m_data_fifo_pos;
	uint8_t m_data_fifo_size;

	uint32_t send_byte(const uint32_t value = 0, const uint32_t mask = 0);

	uint8_t irq_fifo_pop();
	void irq_fifo_push(const uint8_t status);
	bool irq_fifo_empty() const;
	bool irq_fifo_full() const;
	void irq_fifo_reset();
	static constexpr uint8_t IRQ_FIFO_SIZE = 8;
	uint8_t m_irq_fifo[IRQ_FIFO_SIZE];
	uint8_t m_irq_fifo_pos;
	uint8_t m_irq_fifo_size;

	void update_irq();
	devcb_write_line m_irq_cb;

	void set_drq();
	void clear_drq();
	devcb_write_line m_drq_cb;
	bool m_drq_state;

	void delay(uint32_t cycles);
	void delay_arb_poll();
	void delay_cycles(uint32_t cycles);
	emu_timer *m_timer;
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	bool coarse_schedule_eligible() const;
	bool m_coarse_continue;
	uint32_t m_coarse_cycle_debt;
	uint32_t m_coarse_iterations;
	// Counts concurrently-nested step_entry() calls on the current call
	// stack. nscsi_bus_device::regen_ctrl() dispatches SCSI control-line
	// changes to other devices synchronously (same call stack, not a
	// scheduled event) - any m_scsi_bus->ctrl_w() call below can
	// therefore synchronously trigger another device's own reaction,
	// which can synchronously call back into this device's own
	// scsi_ctrl_changed() before the call that made the original
	// ctrl_w() has returned. Only the outermost (depth-0) step_entry()
	// call may run its coarse-mode resume loop; any call reentered while
	// that depth is already nonzero must behave like the plain fine-mode
	// passthrough (call step() once, don't loop) - see step_entry()'s
	// own comment for why looping there too would grow this stack
	// without bound over a large transfer.
	uint32_t m_step_entry_depth;
#endif

	bool set_command_length(const uint8_t cc);
	uint8_t get_msg_out() const;
};

class wd33c92_device : public wd33c9x_base_device
{
public:
	wd33c92_device(const machine_config &mconfig, const char *tag, device_t *owner)
		: wd33c92_device(mconfig, tag, owner, 0)
	{
	}

	wd33c92_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class wd33c93_device : public wd33c9x_base_device
{
public:
	wd33c93_device(const machine_config &mconfig, const char *tag, device_t *owner)
		: wd33c93_device(mconfig, tag, owner, 0)
	{
	}

	wd33c93_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class wd33c93a_device : public wd33c9x_base_device
{
public:
	wd33c93a_device(const machine_config &mconfig, const char *tag, device_t *owner)
		: wd33c93a_device(mconfig, tag, owner, 0)
	{
	}

	wd33c93a_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};

class wd33c93b_device : public wd33c9x_base_device
{
public:
	wd33c93b_device(const machine_config &mconfig, const char *tag, device_t *owner)
		: wd33c93b_device(mconfig, tag, owner, 0)
	{
	}

	wd33c93b_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);
};


DECLARE_DEVICE_TYPE(WD33C92,  wd33c92_device)
DECLARE_DEVICE_TYPE(WD33C93,  wd33c93_device)
DECLARE_DEVICE_TYPE(WD33C93A, wd33c93a_device)
DECLARE_DEVICE_TYPE(WD33C93B, wd33c93b_device)

#endif // MAME_MACHINE_WD33C9X_H
