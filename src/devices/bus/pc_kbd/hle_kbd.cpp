// license:BSD-3-Clause
// copyright-holders:Patrick Mackinlay

/*
 * PS/2 keyboard high-level emulation.
 *
 * A 101/102/104-key keyboard without a microcontroller, so that a machine
 * using it does not pay for running keyboard firmware continuously. The
 * serial engine follows hle_mouse.cpp: a 40kHz timer generates the 10kHz
 * clock edges and samples or drives the data line in the middle of each
 * clock phase. Unlike the mouse, a byte stays queued until its stop bit
 * has been sent, so a transmission aborted by the host inhibiting the
 * clock is repeated as the protocol requires.
 *
 * Scan code sets 2 (the power-on default) and 3 are implemented, including
 * set 3's per-key make/break/typematic modes (commands f7-fd). SGI's IP22
 * PROM and IRIX both switch the keyboard to set 3. Set 1 is not
 * implemented; selecting it leaves the current set in effect.
 *
 * Sources:
 *
 *   https://web.archive.org/web/20180126072040/http://www.computer-engineering.org/ps2keyboard/
 *   https://www.win.tue.nl/~aeb/linux/kbd/scancodes-10.html
 *   IBM Personal System/2 Keyboard and Auxiliary Device Controller technical reference
 *
 * TODO
 *   - scan code set 1
 *   - set 2 "fake shift" sequences for navigation keys with num lock/shift
 *   - LED outputs
 */

#include "emu.h"
#include "hle_kbd.h"

#define LOG_RXTX    (1U << 1)
#define LOG_COMMAND (1U << 2)
#define LOG_KEY     (1U << 3)
#define LOG_STATE   (1U << 4)

//#define VERBOSE (LOG_COMMAND | LOG_KEY)

#include "logmacro.h"

DEFINE_DEVICE_TYPE(HLE_PS2_KEYBOARD, hle_ps2_keyboard_device, "hle_ps2_kbd", "HLE PS/2 Keyboard")

ALLOW_SAVE_TYPE(hle_ps2_keyboard_device::serial_state);

namespace {

enum key_flags : u8
{
	E0       = 0x01, // set 2: extended (e0 prefix)
	PRTSCR   = 0x02, // set 2: print screen sequence
	PAUSE    = 0x04, // set 2: pause sequence (make only)
};

struct key_code
{
	u8 set2;
	u8 flags;
	u8 set3;
};

// indexed by port * 16 + bit, matching the input ports below
key_code const keys[7 * 16] =
{
	// KEY0
	{ 0x76, 0,      0x08 }, // Esc
	{ 0x05, 0,      0x07 }, // F1
	{ 0x06, 0,      0x0f }, // F2
	{ 0x04, 0,      0x17 }, // F3
	{ 0x0c, 0,      0x1f }, // F4
	{ 0x03, 0,      0x27 }, // F5
	{ 0x0b, 0,      0x2f }, // F6
	{ 0x83, 0,      0x37 }, // F7
	{ 0x0a, 0,      0x3f }, // F8
	{ 0x01, 0,      0x47 }, // F9
	{ 0x09, 0,      0x4f }, // F10
	{ 0x78, 0,      0x56 }, // F11
	{ 0x07, 0,      0x5e }, // F12
	{ 0x7c, PRTSCR, 0x57 }, // Print Screen
	{ 0x7e, 0,      0x5f }, // Scroll Lock
	{ 0x77, PAUSE,  0x62 }, // Pause

	// KEY1
	{ 0x0e, 0,      0x0e }, // `
	{ 0x16, 0,      0x16 }, // 1
	{ 0x1e, 0,      0x1e }, // 2
	{ 0x26, 0,      0x26 }, // 3
	{ 0x25, 0,      0x25 }, // 4
	{ 0x2e, 0,      0x2e }, // 5
	{ 0x36, 0,      0x36 }, // 6
	{ 0x3d, 0,      0x3d }, // 7
	{ 0x3e, 0,      0x3e }, // 8
	{ 0x46, 0,      0x46 }, // 9
	{ 0x45, 0,      0x45 }, // 0
	{ 0x4e, 0,      0x4e }, // -
	{ 0x55, 0,      0x55 }, // =
	{ 0x66, 0,      0x66 }, // Backspace
	{ 0x0d, 0,      0x0d }, // Tab
	{ 0x15, 0,      0x15 }, // Q

	// KEY2
	{ 0x1d, 0,      0x1d }, // W
	{ 0x24, 0,      0x24 }, // E
	{ 0x2d, 0,      0x2d }, // R
	{ 0x2c, 0,      0x2c }, // T
	{ 0x35, 0,      0x35 }, // Y
	{ 0x3c, 0,      0x3c }, // U
	{ 0x43, 0,      0x43 }, // I
	{ 0x44, 0,      0x44 }, // O
	{ 0x4d, 0,      0x4d }, // P
	{ 0x54, 0,      0x54 }, // [
	{ 0x5b, 0,      0x5b }, // ]
	{ 0x5d, 0,      0x5c }, // Backslash
	{ 0x58, 0,      0x14 }, // Caps Lock
	{ 0x1c, 0,      0x1c }, // A
	{ 0x1b, 0,      0x1b }, // S
	{ 0x23, 0,      0x23 }, // D

	// KEY3
	{ 0x2b, 0,      0x2b }, // F
	{ 0x34, 0,      0x34 }, // G
	{ 0x33, 0,      0x33 }, // H
	{ 0x3b, 0,      0x3b }, // J
	{ 0x42, 0,      0x42 }, // K
	{ 0x4b, 0,      0x4b }, // L
	{ 0x4c, 0,      0x4c }, // ;
	{ 0x52, 0,      0x52 }, // '
	{ 0x5a, 0,      0x5a }, // Enter
	{ 0x12, 0,      0x12 }, // Left Shift
	{ 0x1a, 0,      0x1a }, // Z
	{ 0x22, 0,      0x22 }, // X
	{ 0x21, 0,      0x21 }, // C
	{ 0x2a, 0,      0x2a }, // V
	{ 0x32, 0,      0x32 }, // B
	{ 0x31, 0,      0x31 }, // N

	// KEY4
	{ 0x3a, 0,      0x3a }, // M
	{ 0x41, 0,      0x41 }, // ,
	{ 0x49, 0,      0x49 }, // .
	{ 0x4a, 0,      0x4a }, // /
	{ 0x59, 0,      0x59 }, // Right Shift
	{ 0x14, 0,      0x11 }, // Left Ctrl
	{ 0x11, 0,      0x19 }, // Left Alt
	{ 0x29, 0,      0x29 }, // Space
	{ 0x11, E0,     0x39 }, // Right Alt
	{ 0x14, E0,     0x58 }, // Right Ctrl
	{ 0x1f, E0,     0x8b }, // Left Win
	{ 0x27, E0,     0x8c }, // Right Win
	{ 0x2f, E0,     0x8d }, // Menu
	{ 0x70, E0,     0x67 }, // Insert
	{ 0x6c, E0,     0x6e }, // Home
	{ 0x7d, E0,     0x6f }, // Page Up

	// KEY5
	{ 0x71, E0,     0x64 }, // Delete
	{ 0x69, E0,     0x65 }, // End
	{ 0x7a, E0,     0x6d }, // Page Down
	{ 0x75, E0,     0x63 }, // Up
	{ 0x6b, E0,     0x61 }, // Left
	{ 0x72, E0,     0x60 }, // Down
	{ 0x74, E0,     0x6a }, // Right
	{ 0x77, 0,      0x76 }, // Num Lock
	{ 0x4a, E0,     0x77 }, // Keypad /
	{ 0x7c, 0,      0x7e }, // Keypad *
	{ 0x7b, 0,      0x84 }, // Keypad -
	{ 0x6c, 0,      0x6c }, // Keypad 7
	{ 0x75, 0,      0x75 }, // Keypad 8
	{ 0x7d, 0,      0x7d }, // Keypad 9
	{ 0x79, 0,      0x7c }, // Keypad +
	{ 0x6b, 0,      0x6b }, // Keypad 4

	// KEY6
	{ 0x73, 0,      0x73 }, // Keypad 5
	{ 0x74, 0,      0x74 }, // Keypad 6
	{ 0x69, 0,      0x69 }, // Keypad 1
	{ 0x72, 0,      0x72 }, // Keypad 2
	{ 0x7a, 0,      0x7a }, // Keypad 3
	{ 0x70, 0,      0x70 }, // Keypad 0
	{ 0x71, 0,      0x71 }, // Keypad .
	{ 0x5a, E0,     0x79 }, // Keypad Enter
	{ 0x61, 0,      0x13 }, // 102nd key (ISO < >)
};

} // anonymous namespace

#define KEY(bit, code, name) PORT_BIT(1U << (bit), IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_CODE(code) PORT_NAME(name)
#define KEYC(bit, code, name, ...) KEY(bit, code, name) PORT_CHAR(__VA_ARGS__)

INPUT_PORTS_START(hle_ps2_keyboard_device)
	PORT_START("KEY0")
	KEYC( 0, KEYCODE_ESC,        "Esc",          UCHAR_MAMEKEY(ESC))
	KEYC( 1, KEYCODE_F1,         "F1",           UCHAR_MAMEKEY(F1))
	KEYC( 2, KEYCODE_F2,         "F2",           UCHAR_MAMEKEY(F2))
	KEYC( 3, KEYCODE_F3,         "F3",           UCHAR_MAMEKEY(F3))
	KEYC( 4, KEYCODE_F4,         "F4",           UCHAR_MAMEKEY(F4))
	KEYC( 5, KEYCODE_F5,         "F5",           UCHAR_MAMEKEY(F5))
	KEYC( 6, KEYCODE_F6,         "F6",           UCHAR_MAMEKEY(F6))
	KEYC( 7, KEYCODE_F7,         "F7",           UCHAR_MAMEKEY(F7))
	KEYC( 8, KEYCODE_F8,         "F8",           UCHAR_MAMEKEY(F8))
	KEYC( 9, KEYCODE_F9,         "F9",           UCHAR_MAMEKEY(F9))
	KEYC(10, KEYCODE_F10,        "F10",          UCHAR_MAMEKEY(F10))
	KEYC(11, KEYCODE_F11,        "F11",          UCHAR_MAMEKEY(F11))
	KEYC(12, KEYCODE_F12,        "F12",          UCHAR_MAMEKEY(F12))
	KEYC(13, KEYCODE_PRTSCR,     "Print Screen", UCHAR_MAMEKEY(PRTSCR))
	KEYC(14, KEYCODE_SCRLOCK,    "Scroll Lock",  UCHAR_MAMEKEY(SCRLOCK))
	KEYC(15, KEYCODE_PAUSE,      "Pause",        UCHAR_MAMEKEY(PAUSE))

	PORT_START("KEY1")
	KEYC( 0, KEYCODE_TILDE,      "`  ~",         '`', '~')
	KEYC( 1, KEYCODE_1,          "1  !",         '1', '!')
	KEYC( 2, KEYCODE_2,          "2  @",         '2', '@')
	KEYC( 3, KEYCODE_3,          "3  #",         '3', '#')
	KEYC( 4, KEYCODE_4,          "4  $",         '4', '$')
	KEYC( 5, KEYCODE_5,          "5  %",         '5', '%')
	KEYC( 6, KEYCODE_6,          "6  ^",         '6', '^')
	KEYC( 7, KEYCODE_7,          "7  &",         '7', '&')
	KEYC( 8, KEYCODE_8,          "8  *",         '8', '*')
	KEYC( 9, KEYCODE_9,          "9  (",         '9', '(')
	KEYC(10, KEYCODE_0,          "0  )",         '0', ')')
	KEYC(11, KEYCODE_MINUS,      "-  _",         '-', '_')
	KEYC(12, KEYCODE_EQUALS,     "=  +",         '=', '+')
	KEYC(13, KEYCODE_BACKSPACE,  "Backspace",    8)
	KEYC(14, KEYCODE_TAB,        "Tab",          9)
	KEYC(15, KEYCODE_Q,          "q  Q",         'q', 'Q')

	PORT_START("KEY2")
	KEYC( 0, KEYCODE_W,          "w  W",         'w', 'W')
	KEYC( 1, KEYCODE_E,          "e  E",         'e', 'E')
	KEYC( 2, KEYCODE_R,          "r  R",         'r', 'R')
	KEYC( 3, KEYCODE_T,          "t  T",         't', 'T')
	KEYC( 4, KEYCODE_Y,          "y  Y",         'y', 'Y')
	KEYC( 5, KEYCODE_U,          "u  U",         'u', 'U')
	KEYC( 6, KEYCODE_I,          "i  I",         'i', 'I')
	KEYC( 7, KEYCODE_O,          "o  O",         'o', 'O')
	KEYC( 8, KEYCODE_P,          "p  P",         'p', 'P')
	KEYC( 9, KEYCODE_OPENBRACE,  "[  {",         '[', '{')
	KEYC(10, KEYCODE_CLOSEBRACE, "]  }",         ']', '}')
	KEYC(11, KEYCODE_BACKSLASH,  "\\  |",        '\\', '|')
	KEYC(12, KEYCODE_CAPSLOCK,   "Caps Lock",    UCHAR_MAMEKEY(CAPSLOCK))
	KEYC(13, KEYCODE_A,          "a  A",         'a', 'A')
	KEYC(14, KEYCODE_S,          "s  S",         's', 'S')
	KEYC(15, KEYCODE_D,          "d  D",         'd', 'D')

	PORT_START("KEY3")
	KEYC( 0, KEYCODE_F,          "f  F",         'f', 'F')
	KEYC( 1, KEYCODE_G,          "g  G",         'g', 'G')
	KEYC( 2, KEYCODE_H,          "h  H",         'h', 'H')
	KEYC( 3, KEYCODE_J,          "j  J",         'j', 'J')
	KEYC( 4, KEYCODE_K,          "k  K",         'k', 'K')
	KEYC( 5, KEYCODE_L,          "l  L",         'l', 'L')
	KEYC( 6, KEYCODE_COLON,      ";  :",         ';', ':')
	KEYC( 7, KEYCODE_QUOTE,      "'  \"",        '\'', '"')
	KEYC( 8, KEYCODE_ENTER,      "Enter",        13)
	KEYC( 9, KEYCODE_LSHIFT,     "Left Shift",   UCHAR_SHIFT_1)
	KEYC(10, KEYCODE_Z,          "z  Z",         'z', 'Z')
	KEYC(11, KEYCODE_X,          "x  X",         'x', 'X')
	KEYC(12, KEYCODE_C,          "c  C",         'c', 'C')
	KEYC(13, KEYCODE_V,          "v  V",         'v', 'V')
	KEYC(14, KEYCODE_B,          "b  B",         'b', 'B')
	KEYC(15, KEYCODE_N,          "n  N",         'n', 'N')

	PORT_START("KEY4")
	KEYC( 0, KEYCODE_M,          "m  M",         'm', 'M')
	KEYC( 1, KEYCODE_COMMA,      ",  <",         ',', '<')
	KEYC( 2, KEYCODE_STOP,       ".  >",         '.', '>')
	KEYC( 3, KEYCODE_SLASH,      "/  ?",         '/', '?')
	KEY ( 4, KEYCODE_RSHIFT,     "Right Shift")
	KEYC( 5, KEYCODE_LCONTROL,   "Left Ctrl",    UCHAR_SHIFT_2)
	KEYC( 6, KEYCODE_LALT,       "Left Alt",     UCHAR_MAMEKEY(LALT))
	KEYC( 7, KEYCODE_SPACE,      "Space",        ' ')
	KEYC( 8, KEYCODE_RALT,       "Right Alt",    UCHAR_MAMEKEY(RALT))
	KEYC( 9, KEYCODE_RCONTROL,   "Right Ctrl",   UCHAR_MAMEKEY(RCONTROL))
	KEYC(10, KEYCODE_LWIN,       "Left Win",     UCHAR_MAMEKEY(LWIN))
	KEYC(11, KEYCODE_RWIN,       "Right Win",    UCHAR_MAMEKEY(RWIN))
	KEYC(12, KEYCODE_MENU,       "Menu",         UCHAR_MAMEKEY(MENU))
	KEYC(13, KEYCODE_INSERT,     "Insert",       UCHAR_MAMEKEY(INSERT))
	KEYC(14, KEYCODE_HOME,       "Home",         UCHAR_MAMEKEY(HOME))
	KEYC(15, KEYCODE_PGUP,       "Page Up",      UCHAR_MAMEKEY(PGUP))

	PORT_START("KEY5")
	KEYC( 0, KEYCODE_DEL,        "Delete",       UCHAR_MAMEKEY(DEL))
	KEYC( 1, KEYCODE_END,        "End",          UCHAR_MAMEKEY(END))
	KEYC( 2, KEYCODE_PGDN,       "Page Down",    UCHAR_MAMEKEY(PGDN))
	KEYC( 3, KEYCODE_UP,         "Up",           UCHAR_MAMEKEY(UP))
	KEYC( 4, KEYCODE_LEFT,       "Left",         UCHAR_MAMEKEY(LEFT))
	KEYC( 5, KEYCODE_DOWN,       "Down",         UCHAR_MAMEKEY(DOWN))
	KEYC( 6, KEYCODE_RIGHT,      "Right",        UCHAR_MAMEKEY(RIGHT))
	KEYC( 7, KEYCODE_NUMLOCK,    "Num Lock",     UCHAR_MAMEKEY(NUMLOCK))
	KEYC( 8, KEYCODE_SLASH_PAD,  "Keypad /",     UCHAR_MAMEKEY(SLASH_PAD))
	KEYC( 9, KEYCODE_ASTERISK,   "Keypad *",     UCHAR_MAMEKEY(ASTERISK))
	KEYC(10, KEYCODE_MINUS_PAD,  "Keypad -",     UCHAR_MAMEKEY(MINUS_PAD))
	KEYC(11, KEYCODE_7_PAD,      "Keypad 7",     UCHAR_MAMEKEY(7_PAD))
	KEYC(12, KEYCODE_8_PAD,      "Keypad 8",     UCHAR_MAMEKEY(8_PAD))
	KEYC(13, KEYCODE_9_PAD,      "Keypad 9",     UCHAR_MAMEKEY(9_PAD))
	KEYC(14, KEYCODE_PLUS_PAD,   "Keypad +",     UCHAR_MAMEKEY(PLUS_PAD))
	KEYC(15, KEYCODE_4_PAD,      "Keypad 4",     UCHAR_MAMEKEY(4_PAD))

	PORT_START("KEY6")
	KEYC( 0, KEYCODE_5_PAD,      "Keypad 5",     UCHAR_MAMEKEY(5_PAD))
	KEYC( 1, KEYCODE_6_PAD,      "Keypad 6",     UCHAR_MAMEKEY(6_PAD))
	KEYC( 2, KEYCODE_1_PAD,      "Keypad 1",     UCHAR_MAMEKEY(1_PAD))
	KEYC( 3, KEYCODE_2_PAD,      "Keypad 2",     UCHAR_MAMEKEY(2_PAD))
	KEYC( 4, KEYCODE_3_PAD,      "Keypad 3",     UCHAR_MAMEKEY(3_PAD))
	KEYC( 5, KEYCODE_0_PAD,      "Keypad 0",     UCHAR_MAMEKEY(0_PAD))
	KEYC( 6, KEYCODE_DEL_PAD,    "Keypad .",     UCHAR_MAMEKEY(DEL_PAD))
	KEYC( 7, KEYCODE_ENTER_PAD,  "Keypad Enter", UCHAR_MAMEKEY(ENTER_PAD))
	KEY ( 8, KEYCODE_BACKSLASH2, "102nd key")
	PORT_BIT(0xfe00, IP_ACTIVE_HIGH, IPT_UNUSED)
INPUT_PORTS_END

#undef KEYC
#undef KEY

static constexpr attotime serial_cycle = attotime::from_usec(25);
static constexpr attotime scan_period = attotime::from_msec(5);

hle_ps2_keyboard_device::hle_ps2_keyboard_device(const machine_config &mconfig, const char *tag, device_t *owner, u32 clock)
	: device_t(mconfig, HLE_PS2_KEYBOARD, tag, owner, clock)
	, device_pc_kbd_interface(mconfig, *this)
	, m_keys(*this, "KEY%u", 0U)
{
}

ioport_constructor hle_ps2_keyboard_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(hle_ps2_keyboard_device);
}

void hle_ps2_keyboard_device::device_start()
{
	save_item(NAME(m_state));
	save_item(NAME(m_bit));
	save_item(NAME(m_data));
	save_item(NAME(m_parity));
	save_item(NAME(m_rx_data));
	save_item(NAME(m_tx_response));
	save_item(NAME(m_tx_last));

	save_item(NAME(m_resp));
	save_item(NAME(m_resp_len));
	save_item(NAME(m_resp_pos));

	save_item(NAME(m_out));
	save_item(NAME(m_out_head));
	save_item(NAME(m_out_count));

	save_item(NAME(m_pending));
	save_item(NAME(m_set));
	save_item(NAME(m_typematic));
	save_item(NAME(m_leds));
	save_item(NAME(m_enabled));
	save_item(NAME(m_mode));

	save_item(NAME(m_state_keys));
	save_item(NAME(m_repeat_key));
	save_item(NAME(m_clock_changed));

	set_pc_kbdc_device();

	m_serial = timer_alloc(FUNC(hle_ps2_keyboard_device::serial), this);
	m_scan = timer_alloc(FUNC(hle_ps2_keyboard_device::scan), this);
	m_repeat = timer_alloc(FUNC(hle_ps2_keyboard_device::repeat), this);

	m_clock_changed = attotime::zero;
}

void hle_ps2_keyboard_device::device_reset()
{
	defaults();
	m_set = 2;

	m_resp_len = 0;
	m_resp_pos = 0;
	m_out_head = 0;
	m_out_count = 0;
	m_tx_last = 0;
	m_tx_response = false;
	m_rx_data = 0;

	for (unsigned i = 0; i < std::size(m_state_keys); i++)
		m_state_keys[i] = m_keys[i]->read();

	// basic assurance test passed
	respond(0xaa);

	m_state = IDLE;

	// release clock and data lines
	m_pc_kbdc->data_write_from_kb(1);
	m_pc_kbdc->clock_write_from_kb(1);

	m_scan->adjust(scan_period, 0, scan_period);
}

void hle_ps2_keyboard_device::defaults()
{
	m_pending = 0;
	m_typematic = 0x2b; // 10.9 characters/second, 500ms delay
	m_leds = 0;
	m_enabled = true;
	m_repeat_key = -1;
	m_repeat->enable(false);

	// set 3: all keys make/break and typematic
	std::fill(std::begin(m_mode), std::end(m_mode), 3);
}

void hle_ps2_keyboard_device::clock_write(int state)
{
	// record when the clock signal last changed state
	m_clock_changed = machine().time();

	// resume serial communication
	resume();
}

void hle_ps2_keyboard_device::resume()
{
	if (clock_signal() && m_state == IDLE)
	{
		// check if receiving a start bit
		if (!data_signal())
			m_state = RX_START;

		// start serial communication
		if (!m_serial->enabled())
			m_serial->adjust(serial_cycle, 0, serial_cycle);
	}
}

void hle_ps2_keyboard_device::serial(s32 param)
{
	// host may inhibit device communication by holding the clock low for 100µs
	if (!clock_signal() && clock_held(100))
	{
		// an interrupted transmission is repeated, because the byte is
		// only dequeued once its stop bit has been sent
		m_state = IDLE;

		// release clock and data lines
		m_pc_kbdc->data_write_from_kb(1);
		m_pc_kbdc->clock_write_from_kb(1);

		// stop serial communication
		m_serial->enable(false);
		return;
	}

	LOGMASKED(LOG_STATE, "state %d clock %d data %d\n", m_state, clock_signal(), data_signal());

	switch (m_state)
	{
	case IDLE:
		if (clock_signal())
		{
			// check if the host is requesting to send
			if (!data_signal())
				m_state = RX_START;
			// stop serial communication if nothing to transmit
			else if (m_resp_pos == m_resp_len && !m_out_count)
				m_serial->enable(false);
			else
				// device may transmit after clock is high for 50µs
				if (clock_held(50))
					m_state = TX_START;
		}
		break;

	case RX_START:
		// check for start bit
		if (!data_signal())
		{
			m_state = RX_CLOCK_LO0;

			// prepare data
			m_data = 0;
			m_parity = 1;
			m_bit = 0;
		}
		else
			m_state = IDLE;
		break;

	case RX_CLOCK_LO0:
		// assert clock
		m_state = RX_CLOCK_LO1;
		m_pc_kbdc->clock_write_from_kb(0);
		break;

	case RX_CLOCK_LO1:
		// hold clock
		m_state = RX_CLOCK_HI0;
		break;

	case RX_CLOCK_HI0:
		// release clock
		m_state = RX_CLOCK_HI1;
		m_pc_kbdc->clock_write_from_kb(1);
		break;

	case RX_CLOCK_HI1:
		switch (m_bit)
		{
		case 8: // parity bit
			m_state = RX_CLOCK_LO0;
			m_parity += data_signal();
			m_bit++;
			break;

		case 9: // acknowledge
			if (data_signal())
			{
				m_state = RX_CLOCK_LO0;
				m_pc_kbdc->data_write_from_kb(0);
				m_bit++;
			}
			break;

		case 10: // finished
			m_pc_kbdc->data_write_from_kb(1);
			if (m_parity & 1)
			{
				LOGMASKED(LOG_RXTX, "rx parity error data 0x%02x\n", m_data);
				m_state = IDLE;
				m_resp_len = 0;
				m_resp_pos = 0;
				respond(0xfe);
			}
			else
			{
				LOGMASKED(LOG_RXTX, "rx data 0x%02x\n", m_data);
				m_rx_data = m_data;
				m_state = COMMAND;
			}
			break;

		default: // data bit
			m_state = RX_CLOCK_LO0;
			m_parity += data_signal();
			m_data |= (data_signal() << m_bit++);
			break;
		}
		break;

	case COMMAND:
		m_state = IDLE;
		command(m_rx_data);
		break;

	case TX_START:
		// prepare data
		m_tx_response = m_resp_pos < m_resp_len;
		m_data = m_tx_response ? m_resp[m_resp_pos] : m_out[m_out_head];
		m_parity = 1;
		m_bit = 0;

		// start bit
		m_state = TX_CLOCK_LO0;
		m_pc_kbdc->data_write_from_kb(0);
		break;

	case TX_CLOCK_LO0:
		// assert clock
		m_state = TX_CLOCK_LO1;
		m_pc_kbdc->clock_write_from_kb(0);
		break;

	case TX_CLOCK_LO1:
		// hold clock
		m_state = TX_CLOCK_HI0;
		break;

	case TX_CLOCK_HI0:
		// release clock
		m_state = TX_CLOCK_HI1;
		m_pc_kbdc->clock_write_from_kb(1);
		break;

	case TX_CLOCK_HI1:
		switch (m_bit)
		{
		case 8: // parity bit
			m_state = TX_CLOCK_LO0;
			m_pc_kbdc->data_write_from_kb(m_parity & 1);
			m_bit++;
			break;

		case 9: // stop bit
			m_state = TX_CLOCK_LO0;
			m_pc_kbdc->data_write_from_kb(1);
			m_bit++;
			break;

		case 10: // finished
			LOGMASKED(LOG_RXTX, "tx data 0x%02x\n", m_data);
			m_state = IDLE;
			m_tx_last = m_data;
			if (m_tx_response)
				m_resp_pos++;
			else if (m_out_count)
			{
				m_out_head = (m_out_head + 1) % std::size(m_out);
				m_out_count--;
			}
			break;

		default: // data
			m_state = TX_CLOCK_LO0;
			m_parity += BIT(m_data, m_bit);
			m_pc_kbdc->data_write_from_kb(BIT(m_data, m_bit));
			m_bit++;
			break;
		}
		break;
	}
}

void hle_ps2_keyboard_device::respond(u8 const data)
{
	if (m_resp_len < std::size(m_resp))
		m_resp[m_resp_len++] = data;
}

void hle_ps2_keyboard_device::command(u8 const data)
{
	// resend is answered without disturbing anything else
	if (data == 0xfe)
	{
		LOGMASKED(LOG_COMMAND, "resend 0x%02x\n", m_tx_last);
		m_resp_len = 0;
		m_resp_pos = 0;
		respond(m_tx_last);
		resume();
		return;
	}

	m_resp_len = 0;
	m_resp_pos = 0;

	// parameter bytes (all commands are 0xed and above, parameters below)
	if (m_pending && data < 0xed)
	{
		switch (m_pending)
		{
		case 0xed: // set leds
			LOGMASKED(LOG_COMMAND, "set leds 0x%02x\n", data);
			m_leds = data & 7;
			m_pending = 0;
			break;

		case 0xf0: // scan code set
			if (data == 0)
			{
				LOGMASKED(LOG_COMMAND, "get scan code set %d\n", m_set);
				respond(0xfa);
				respond(m_set);
				m_pending = 0;
				resume();
				return;
			}
			LOGMASKED(LOG_COMMAND, "set scan code set %d\n", data);
			if (data == 2 || data == 3)
				m_set = data;
			m_pending = 0;
			break;

		case 0xf3: // set typematic rate/delay
			LOGMASKED(LOG_COMMAND, "set typematic 0x%02x\n", data);
			m_typematic = data & 0x7f;
			m_pending = 0;
			break;

		case 0xfb: // set key typematic
			m_mode[data] = 2;
			break;

		case 0xfc: // set key make/break
			m_mode[data] = 1;
			break;

		case 0xfd: // set key make
			m_mode[data] = 0;
			break;
		}
		respond(0xfa);
		resume();
		return;
	}

	m_pending = 0;

	switch (data)
	{
	case 0xed: // set leds
	case 0xf0: // get/set scan code set
	case 0xf3: // set typematic rate/delay
	case 0xfb: // set key typematic
	case 0xfc: // set key make/break
	case 0xfd: // set key make
		LOGMASKED(LOG_COMMAND, "command 0x%02x\n", data);
		m_pending = data;
		respond(0xfa);
		break;

	case 0xee: // echo
		LOGMASKED(LOG_COMMAND, "echo\n");
		respond(0xee);
		break;

	case 0xf2: // read id
		LOGMASKED(LOG_COMMAND, "read id\n");
		respond(0xfa);
		respond(0xab);
		respond(0x83);
		break;

	case 0xf4: // enable
		LOGMASKED(LOG_COMMAND, "enable\n");
		m_enabled = true;
		m_out_count = 0;
		respond(0xfa);
		break;

	case 0xf5: // disable and set defaults
		LOGMASKED(LOG_COMMAND, "disable\n");
		defaults();
		m_enabled = false;
		m_out_count = 0;
		respond(0xfa);
		break;

	case 0xf6: // set defaults
		LOGMASKED(LOG_COMMAND, "set defaults\n");
		defaults();
		m_out_count = 0;
		respond(0xfa);
		break;

	case 0xf7: // set all keys typematic
	case 0xf8: // set all keys make/break
	case 0xf9: // set all keys make
	case 0xfa: // set all keys typematic and make/break
		{
			static u8 const mode[] = { 2, 1, 0, 3 };
			LOGMASKED(LOG_COMMAND, "set all keys mode %d\n", mode[data - 0xf7]);
			std::fill(std::begin(m_mode), std::end(m_mode), mode[data - 0xf7]);
			respond(0xfa);
		}
		break;

	case 0xff: // reset
		LOGMASKED(LOG_COMMAND, "reset\n");
		defaults();
		m_set = 2;
		m_out_count = 0;
		respond(0xfa);
		respond(0xaa);
		break;

	default:
		LOGMASKED(LOG_COMMAND, "unrecognized command 0x%02x\n", data);
		respond(0xfe);
		break;
	}

	resume();
}

void hle_ps2_keyboard_device::queue(u8 const data)
{
	if (m_out_count < std::size(m_out))
	{
		m_out[(m_out_head + m_out_count) % std::size(m_out)] = data;
		m_out_count++;
	}
	else
		// buffer overrun: replace the last byte with the overrun code
		m_out[(m_out_head + m_out_count - 1) % std::size(m_out)] = 0x00;
}

void hle_ps2_keyboard_device::key(unsigned const index, bool const make)
{
	key_code const &k = keys[index];
	unsigned const before = m_out_count;

	if (m_set == 3)
	{
		if (!make && !BIT(m_mode[k.set3], 0))
			return;

		LOGMASKED(LOG_KEY, "key %d %s set 3 0x%02x\n", index, make ? "make" : "break", k.set3);
		if (!make)
			queue(0xf0);
		queue(k.set3);
	}
	else if (k.flags & PAUSE)
	{
		if (make)
			for (u8 const b : { 0xe1, 0x14, 0x77, 0xe1, 0xf0, 0x14, 0xf0, 0x77 })
				queue(b);
	}
	else if (k.flags & PRTSCR)
	{
		if (make)
			for (u8 const b : { 0xe0, 0x12, 0xe0, 0x7c })
				queue(b);
		else
			for (u8 const b : { 0xe0, 0xf0, 0x7c, 0xe0, 0xf0, 0x12 })
				queue(b);
	}
	else
	{
		LOGMASKED(LOG_KEY, "key %d %s set 2 %s0x%02x\n", index, make ? "make" : "break", (k.flags & E0) ? "e0 " : "", k.set2);
		if (k.flags & E0)
			queue(0xe0);
		if (!make)
			queue(0xf0);
		queue(k.set2);
	}

	if (m_out_count != before)
		resume();
}

void hle_ps2_keyboard_device::scan(s32 param)
{
	for (unsigned port = 0; port < std::size(m_state_keys); port++)
	{
		u16 const now = m_keys[port]->read();
		u16 const changed = now ^ m_state_keys[port];
		if (!changed)
			continue;
		m_state_keys[port] = now;

		if (!m_enabled)
			continue;

		for (unsigned bit = 0; bit < 16; bit++)
		{
			if (!BIT(changed, bit))
				continue;

			unsigned const index = port * 16 + bit;
			bool const make = BIT(now, bit);

			key(index, make);

			if (make)
			{
				// the most recently pressed key repeats
				bool const typematic = (m_set == 3) ? BIT(m_mode[keys[index].set3], 1) : !(keys[index].flags & PAUSE);
				if (typematic)
				{
					m_repeat_key = index;
					m_repeat->adjust(attotime::from_msec(250 * (BIT(m_typematic, 5, 2) + 1)));
				}
				else if (m_repeat_key >= 0)
				{
					m_repeat_key = -1;
					m_repeat->enable(false);
				}
			}
			else if (int(index) == m_repeat_key)
			{
				m_repeat_key = -1;
				m_repeat->enable(false);
			}
		}
	}
}

void hle_ps2_keyboard_device::repeat(s32 param)
{
	if (m_repeat_key < 0 || !m_enabled)
		return;

	// skip a repeat rather than fill the buffer while the host is busy
	if (!m_out_count)
		key(m_repeat_key, true);

	// period = (8 + A) * 2^B * 4.17ms, A = bits 0-2, B = bits 3-4
	u32 const period_us = (8 + BIT(m_typematic, 0, 3)) * (1U << BIT(m_typematic, 3, 2)) * 4170;
	m_repeat->adjust(attotime::from_usec(period_us));
}
