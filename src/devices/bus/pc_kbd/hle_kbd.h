// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

#ifndef MAME_BUS_PC_KBD_HLE_KBD_H
#define MAME_BUS_PC_KBD_HLE_KBD_H

#pragma once

#include "pc_kbdc.h"

class hle_ps2_keyboard_device
	: public device_t
	, public device_pc_kbd_interface
{
public:
	hle_ps2_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock);

protected:
	// device_t overrides
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_pc_kbd_interface overrides
	virtual void clock_write(int state) override;

private:
	bool const clock_held(unsigned usec) { return (machine().time() - m_clock_changed) >= attotime::from_usec(usec); }

	void resume();
	TIMER_CALLBACK_MEMBER(serial);
	void command(u8 const data);
	void respond(u8 const data);

	TIMER_CALLBACK_MEMBER(scan);
	TIMER_CALLBACK_MEMBER(repeat);
	void defaults();
	void key(unsigned const index, bool const make);
	void queue(u8 const data);

	required_ioport_array<7> m_keys;

	emu_timer *m_serial;
	emu_timer *m_scan;
	emu_timer *m_repeat;

	attotime m_clock_changed;

	// serial state
	enum serial_state : unsigned
	{
		IDLE,
		COMMAND,

		RX_START,
		RX_CLOCK_LO0,
		RX_CLOCK_LO1,
		RX_CLOCK_HI0,
		RX_CLOCK_HI1,

		TX_START,
		TX_CLOCK_LO0,
		TX_CLOCK_LO1,
		TX_CLOCK_HI0,
		TX_CLOCK_HI1,
	}
	m_state;
	unsigned m_bit;
	u8 m_data;
	unsigned m_parity;
	u8 m_rx_data;
	bool m_tx_response; // byte in transmission came from the response buffer
	u8 m_tx_last;       // last byte completely transmitted (for resend)

	// command responses take priority over key data
	u8 m_resp[4];
	unsigned m_resp_len;
	unsigned m_resp_pos;

	// key data output buffer
	u8 m_out[32];
	unsigned m_out_head;
	unsigned m_out_count;

	// settings
	u8 m_pending;       // command awaiting parameter(s), or 0
	u8 m_set;           // scan code set (2 or 3)
	u8 m_typematic;     // rate/delay byte
	u8 m_leds;
	bool m_enabled;
	u8 m_mode[256];     // scan code set 3 per-key mode: bit 0 break, bit 1 typematic

	// key state
	u16 m_state_keys[7];
	int m_repeat_key;
};

// device type definition
DECLARE_DEVICE_TYPE(HLE_PS2_KEYBOARD, hle_ps2_keyboard_device)

#endif // MAME_BUS_PC_KBD_HLE_KBD_H
