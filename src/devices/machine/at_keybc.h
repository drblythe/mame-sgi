// license:BSD-3-Clause
// copyright-holders:Vas Crabb
/***************************************************************************

    IBM PC/AT and PS/2 keyboard controllers

***************************************************************************/
#ifndef MAME_MACHINE_AT_KEYBC_H
#define MAME_MACHINE_AT_KEYBC_H

#pragma once

#include "cpu/mcs48/mcs48.h"


//**************************************************************************
//  KEYBOARD CONTROLLER DEVICE BASE
//**************************************************************************

class at_kbc_device_base : public device_t
{
public:
	virtual ~at_kbc_device_base();

	// outputs to host
	auto hot_res() { return m_hot_res_cb.bind(); }
	auto gate_a20() { return m_gate_a20_cb.bind(); }
	auto kbd_irq() { return m_kbd_irq_cb.bind(); }

	// outputs to keyboard
	auto kbd_clk() { return m_kbd_clk_cb.bind(); } // open collector with 10kΩ pull-up
	auto kbd_data() { return m_kbd_data_cb.bind(); } // open collector with 10kΩ pull-up

	// host interface
	virtual uint8_t data_r();
	virtual uint8_t status_r();
	void data_w(uint8_t data);
	void command_w(uint8_t data);

	// inputs from keyboard
	void kbd_clk_w(int state);
	void kbd_data_w(int state);

	// Keep the MCU close behind the host CPU across host accesses: a host
	// read that finds the MCU behind ends the host's timeslice, a host write
	// interleaves both for 1 ms. For hosts much faster than the firmware's
	// polling assumes; needs the MCU to run after the host CPU in each
	// timeslice. Off by default.
	void set_mcu_catch_up(bool enable) { m_mcu_catch_up = enable; }
	// Let the MCU skip firmware polling loops whose inputs can't change until
	// the next synchronize (MCS48 poll-loop skip; PS/2 controller only). Off
	// by default.
	void set_mcu_poll_loop_skip(bool enable) { m_mcu_poll_loop_skip = enable; }

protected:
	// trampoline constructor
	at_kbc_device_base(machine_config const &mconfig, device_type type, char const *tag, device_t *owner, u32 clock);

	// device_t implementation
	virtual void device_start() override ATTR_COLD;

	// host outputs - use 1 = asserted, 0 = deasserted
	void set_hot_res(u8 state);
	void set_gate_a20(u8 state);
	void set_kbd_irq(u8 state);

	// keyboard line drive - use 1 = pulled up, 0 = driven low
	void set_kbd_clk_out(u8 state);
	void set_kbd_data_out(u8 state);
	u8 kbd_clk_r() const;
	u8 kbd_data_r() const;

	required_device<upi41_cpu_device> m_mcu;
	bool m_mcu_catch_up = false;
	bool m_mcu_poll_loop_skip = false;

private:
	// internal sync helpers
	TIMER_CALLBACK_MEMBER(write_data);
	TIMER_CALLBACK_MEMBER(write_command);
	TIMER_CALLBACK_MEMBER(set_kbd_clk_in);
	TIMER_CALLBACK_MEMBER(set_kbd_data_in);

	devcb_write_line m_hot_res_cb, m_gate_a20_cb, m_kbd_irq_cb;
	devcb_write_line m_kbd_clk_cb, m_kbd_data_cb;

	u8 m_hot_res, m_gate_a20, m_kbd_irq;
	u8 m_kbd_clk_in, m_kbd_clk_out, m_kbd_data_in, m_kbd_data_out;
};


//**************************************************************************
//  PC/AT KEYBOARD CONTROLLER DEVICE
//**************************************************************************

class at_keyboard_controller_device : public at_kbc_device_base
{
public:
	// standard constructor
	at_keyboard_controller_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
	virtual ~at_keyboard_controller_device();

protected:
	// device_t implementation
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;

private:
	// MCU I/O handlers
	void p2_w(uint8_t data);
};


//**************************************************************************
//  PS/2 KEYBOARD/MOUSE CONTROLLER DEVICE
//**************************************************************************

class ps2_keyboard_controller_device : public at_kbc_device_base
{
public:
	// outputs to host
	auto aux_irq() { return m_aux_irq_cb.bind(); }

	// outputs to aux
	auto aux_clk() { return m_aux_clk_cb.bind(); } // open collector with 10kΩ pull-up
	auto aux_data() { return m_aux_data_cb.bind(); } // open collector with 10kΩ pull-up

	// host interface
	virtual uint8_t data_r() override;
	virtual uint8_t status_r() override;

	// inputs from aux
	void aux_clk_w(int state);
	void aux_data_w(int state);

	// standard constructor
	ps2_keyboard_controller_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock);
	virtual ~ps2_keyboard_controller_device();

protected:
	// device_t implementation
	virtual tiny_rom_entry const *device_rom_region() const override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;

private:
	// host outputs - use 1 = asserted, 0 = deasserted
	void set_aux_irq(u8 state);

	// mouse line drive - use 1 = pulled up, 0 = driven low
	void set_aux_clk_out(u8 state);
	void set_aux_data_out(u8 state);
	u8 aux_clk_r() const;
	u8 aux_data_r() const;

	// internal sync helpers
	TIMER_CALLBACK_MEMBER(set_aux_clk_in);
	TIMER_CALLBACK_MEMBER(set_aux_data_in);
	TIMER_CALLBACK_MEMBER(set_aux_irq_timer_callback);

	// MCU I/O handlers
	uint8_t p1_r();
	void p2_w(uint8_t data);

	emu_timer* m_aux_irq_timer;
	devcb_write_line m_aux_irq_cb;
	devcb_write_line m_aux_clk_cb, m_aux_data_cb;

	u8 m_aux_irq;
	u8 m_aux_clk_in, m_aux_clk_out, m_aux_data_in, m_aux_data_out;
	u8 m_p2_data;
};


//**************************************************************************
//  DEVICE TYPES
//**************************************************************************

DECLARE_DEVICE_TYPE(AT_KEYBOARD_CONTROLLER, at_keyboard_controller_device)
DECLARE_DEVICE_TYPE(PS2_KEYBOARD_CONTROLLER, ps2_keyboard_controller_device)

#endif // MAME_MACHINE_AT_KEYBC_H
