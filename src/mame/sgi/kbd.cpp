// license:BSD-3-Clause
// copyright-holders:Ryan Holtz, Patrick Mackinlay

#include "emu.h"
#include "kbd.h"

#include "sound/beep.h"
#include "speaker.h"

DEFINE_DEVICE_TYPE(SGI_KBD_PORT, sgi_kbd_port_device, "sgi_kbd_port", "SGI Keyboard Port")
DEFINE_DEVICE_TYPE(SGI_KBD, sgi_kbd_device, "sgi_kbd", "SGI Keyboard")
DEFINE_DEVICE_TYPE(SGI_KBD_HLE, sgi_kbd_hle_device, "sgi_kbd_hle", "SGI Keyboard (HLE)")

/*
 * An emulation of a keyboard compatible with the Silicon Graphics 4D series
 * (Professional IRIS, Personal IRIS, PowerSeries, Crimson), Indigo, and Onyx
 * systems. The physical interface may be DB-15, DB-9 or DIN-6 depending on the
 * system, however these are all logically and electrically compatible. A mouse
 * port is also present on the keyboard, but its signals are routed directly
 * to/from the host. Later SGI machines use a PS/2 compatible keyboard.
 *
 * The emulated keyboard is a Key Tronic design with an Intel 8031 MCU and an
 * Exar 22-00958-000 capacitive sensing chip. The 8-position DIP switch is not
 * used by the keyboard itself, but is read and returned as configuration data
 * to the host, which uses it to identify the keyboard layout. The keyboard
 * protocol is RS-423 asynchronous serial at 600 baud, with 1 start, 8 data,
 * odd parity and 1 stop bit. Modifier keys and LEDs are managed by the host.
 *
 *  Part                Function
 *  ----                --------
 *  P8031AH             MCS-51 series microcontroller
 *  D2764A              8KiB eprom
 *  SN74LS373N          octal d-type transparent latch
 *  SN74LS374N          octal d-type edge triggered flip-flops
 *  22-00958-000C5145B  capacitive sensing chip
 *  DS26LS32ACN         quad differential line receivers
 *  DS3692N             differential line drivers
 *  NE555N              beep timer?
 *  11059 KSSOH         11.0592 MHz crystal
 *  8-pos DIP           keyboard layout configuration
 *
 * Sources:
 *  - IRIX keyboard(7) manual page
 *
 * TODO:
 *  - port 1 outputs and beeper/keyclick
 *
 */

sgi_kbd_port_device::sgi_kbd_port_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SGI_KBD_PORT, tag, owner, clock)
	, device_single_card_slot_interface<device_sgi_kbd_port_interface>(mconfig, *this)
	, m_rxd_handler(*this)
	, m_kbd(nullptr)
{
}

sgi_kbd_port_device::~sgi_kbd_port_device()
{
}

void sgi_kbd_port_device::device_config_complete()
{
	m_kbd = get_card_device();
}

void sgi_kbd_port_device::device_start()
{
	m_rxd_handler(1);
}

void sgi_kbd_port_device::write_txd(int state)
{
	if (m_kbd)
		m_kbd->write_txd(state);
}

device_sgi_kbd_port_interface::device_sgi_kbd_port_interface(machine_config const &mconfig, device_t &device)
	: device_interface(device, "sgi_kbd")
	, m_port(dynamic_cast<sgi_kbd_port_device *>(device.owner()))
{
}

device_sgi_kbd_port_interface::~device_sgi_kbd_port_interface()
{
}

sgi_kbd_device::sgi_kbd_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SGI_KBD, tag, owner, clock)
	, device_sgi_kbd_port_interface(mconfig, *this)
	, m_mcu(*this, "mcu")
	, m_matrix(*this, "col.%x", 0U)
	, m_led(*this, "led.%u", 0U)
	, m_col(0)
	, m_row(0)
	, m_p3(0)
{
}

/*
 * MCU port 1 WIP
 * --------------
 * p1.0  o  scan matrix
 * p1.1  o  ?
 * p1.2
 * p1.3  o  beeper?
 * p1.4
 * p1.5
 * p1.6
 * p1.7
*/

enum p3_mask : u8
{
	P3_RXD  = 0x01,
	P3_TXD  = 0x02,
	P3_INT0 = 0x04,
	P3_INT1 = 0x08,
	P3_T0   = 0x10,
	P3_T1   = 0x20,
	P3_WR   = 0x40,
	P3_RD   = 0x80,
};

void sgi_kbd_device::device_add_mconfig(machine_config &config)
{
	speaker_device &speaker(SPEAKER(config, "speaker"));
	speaker.front_center();

	beep_device &beeper(BEEP(config, "beeper", 480));
	beeper.add_route(ALL_OUTPUTS, speaker, 0.25);

	I8031(config, m_mcu, 11.0592_MHz_XTAL);
	m_mcu->set_addrmap(AS_PROGRAM, &sgi_kbd_device::map_mem);
	m_mcu->set_addrmap(AS_DATA, &sgi_kbd_device::map_pio);
	m_mcu->port_out_cb<1>().append(FUNC(sgi_kbd_device::scan_matrix)).bit(0);
	m_mcu->port_out_cb<1>().append(beeper, FUNC(beep_device::set_state)).bit(3);
	m_mcu->port_out_cb<3>().append(FUNC(sgi_kbd_device::write_rxd)).bit(1);
	m_mcu->port_in_cb<3>().set([this]() { return m_p3; });
}

void sgi_kbd_device::map_mem(address_map &map)
{
	map(0x0000, 0x1fff).rom().region("mcu", 0);
}

void sgi_kbd_device::map_pio(address_map &map)
{
	map(0x2000, 0x2000).select(0x0f00).mirror(0x00ff).lr8(
		[this](offs_t offset)
		{
			m_col = offset >> 8;
			m_row = 0;
			return 0;
		}, "set_column");

	map(0xff40, 0xff40).w(FUNC(sgi_kbd_device::led_w));
}

void sgi_kbd_device::device_start()
{
	save_item(NAME(m_col));
	save_item(NAME(m_row));
	save_item(NAME(m_p3));
}

void sgi_kbd_device::scan_matrix(int state)
{
	if (!state)
	{
		if (BIT(m_matrix[m_col]->read(), m_row & 7))
			m_p3 &= ~P3_T0;
		else
			m_p3 |= P3_T0;

		m_row++;
	}
}

void sgi_kbd_device::write_txd(int state)
{
	if (state)
		m_p3 |= P3_RXD;
	else
		m_p3 &= ~P3_RXD;
}

void sgi_kbd_device::led_w(u8 data)
{
	/*
	 * bit  output
	 *  0   L1
	 *  1   Num Lock
	 *  2   L2
	 *  3   Caps Lock
	 *  4   L3
	 *  5   Scroll Lock
	 *  6   L4
	 *  7   not used
	 */
	for (unsigned i = 0; i < 7; i++)
		m_led[i] = BIT(data, i);
}

namespace {

INPUT_PORTS_START(sgi_kbd)
	PORT_START("col.0")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_4)            PORT_CHAR('4') PORT_CHAR('$')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_5)            PORT_CHAR('5') PORT_CHAR('%')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_T)            PORT_CHAR('t') PORT_CHAR('T')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_R)            PORT_CHAR('r') PORT_CHAR('R')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_G)            PORT_CHAR('g') PORT_CHAR('G')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_F)            PORT_CHAR('f') PORT_CHAR('F')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_B)            PORT_CHAR('b') PORT_CHAR('B')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_V)            PORT_CHAR('v') PORT_CHAR('V')

	PORT_START("col.1")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F1")                PORT_CODE(KEYCODE_F1)           PORT_CHAR(UCHAR_MAMEKEY(F1))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F3")                PORT_CODE(KEYCODE_F3)           PORT_CHAR(UCHAR_MAMEKEY(F3))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F5")                PORT_CODE(KEYCODE_F5)           PORT_CHAR(UCHAR_MAMEKEY(F5))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F7")                PORT_CODE(KEYCODE_F7)           PORT_CHAR(UCHAR_MAMEKEY(F7))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F9")                PORT_CODE(KEYCODE_F9)           PORT_CHAR(UCHAR_MAMEKEY(F9))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F11")               PORT_CODE(KEYCODE_F11)          PORT_CHAR(UCHAR_MAMEKEY(F11))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Print Screen")      PORT_CODE(KEYCODE_PRTSCR)       PORT_CHAR(UCHAR_MAMEKEY(PRTSCR))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Pause")             PORT_CODE(KEYCODE_PAUSE)

	PORT_START("col.2")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_2)            PORT_CHAR('2') PORT_CHAR('@')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_3)            PORT_CHAR('3') PORT_CHAR('#')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_E)            PORT_CHAR('e') PORT_CHAR('E')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_W)            PORT_CHAR('w') PORT_CHAR('W')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_D)            PORT_CHAR('d') PORT_CHAR('D')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_S)            PORT_CHAR('s') PORT_CHAR('S')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_C)            PORT_CHAR('c') PORT_CHAR('C')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_X)            PORT_CHAR('x') PORT_CHAR('X')

	PORT_START("col.3")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_TILDE)        PORT_CHAR('`') PORT_CHAR('~')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_1)            PORT_CHAR('1') PORT_CHAR('!')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_Q)            PORT_CHAR('q') PORT_CHAR('Q')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Tab")               PORT_CODE(KEYCODE_TAB)          PORT_CHAR(9)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_A)            PORT_CHAR('a') PORT_CHAR('A')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Caps Lock")         PORT_CODE(KEYCODE_CAPSLOCK)     PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_Z)            PORT_CHAR('z') PORT_CHAR('Z')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("L Shift")           PORT_CODE(KEYCODE_LSHIFT)       PORT_CHAR(UCHAR_SHIFT_1)

	PORT_START("col.4")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Esc")               PORT_CODE(KEYCODE_ESC)          PORT_CHAR(UCHAR_MAMEKEY(ESC))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F2")                PORT_CODE(KEYCODE_F2)           PORT_CHAR(UCHAR_MAMEKEY(F2))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F4")                PORT_CODE(KEYCODE_F4)           PORT_CHAR(UCHAR_MAMEKEY(F4))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F6")                PORT_CODE(KEYCODE_F6)           PORT_CHAR(UCHAR_MAMEKEY(F6))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F8")                PORT_CODE(KEYCODE_F8)           PORT_CHAR(UCHAR_MAMEKEY(F8))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F10")               PORT_CODE(KEYCODE_F10)          PORT_CHAR(UCHAR_MAMEKEY(F10))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("F12")               PORT_CODE(KEYCODE_F12)          PORT_CHAR(UCHAR_MAMEKEY(F12))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Scroll Lock")       PORT_CODE(KEYCODE_SCRLOCK)      PORT_CHAR(UCHAR_MAMEKEY(SCRLOCK))

	PORT_START("col.5")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_6)            PORT_CHAR('6') PORT_CHAR('^')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_7)            PORT_CHAR('7') PORT_CHAR('&')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_U)            PORT_CHAR('u') PORT_CHAR('U')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_Y)            PORT_CHAR('y') PORT_CHAR('Y')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_J)            PORT_CHAR('j') PORT_CHAR('J')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_H)            PORT_CHAR('h') PORT_CHAR('H')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_M)            PORT_CHAR('m') PORT_CHAR('M')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_N)            PORT_CHAR('n') PORT_CHAR('N')

	PORT_START("col.6")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_8)            PORT_CHAR('8') PORT_CHAR('*')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_9)            PORT_CHAR('9') PORT_CHAR('(')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_O)            PORT_CHAR('o') PORT_CHAR('O')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_I)            PORT_CHAR('i') PORT_CHAR('I')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_L)            PORT_CHAR('l') PORT_CHAR('L')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_K)            PORT_CHAR('k') PORT_CHAR('K')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_STOP)         PORT_CHAR('.') PORT_CHAR('>')
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_COMMA)        PORT_CHAR(',') PORT_CHAR('<')

	PORT_START("col.7")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_0)            PORT_CHAR('0') PORT_CHAR(')')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_MINUS)        PORT_CHAR('-') PORT_CHAR('_')
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_OPENBRACE)    PORT_CHAR('[') PORT_CHAR('{')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_P)            PORT_CHAR('p') PORT_CHAR('P')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_QUOTE)        PORT_CHAR('\'') PORT_CHAR('"')
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_COLON)        PORT_CHAR(';') PORT_CHAR(':')
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("R Shift")           PORT_CODE(KEYCODE_RSHIFT)       PORT_CHAR(UCHAR_MAMEKEY(RSHIFT))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_SLASH)        PORT_CHAR('/') PORT_CHAR('?')

	PORT_START("col.8")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_EQUALS)       PORT_CHAR('=') PORT_CHAR('+')
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Backspace")         PORT_CODE(KEYCODE_BACKSPACE)    PORT_CHAR(8)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_BACKSLASH)    PORT_CHAR('\\') PORT_CHAR('|')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD)                                PORT_CODE(KEYCODE_CLOSEBRACE)   PORT_CHAR(']') PORT_CHAR('}')
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Enter")             PORT_CODE(KEYCODE_ENTER)        PORT_CHAR(13)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("col.9")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Home")              PORT_CODE(KEYCODE_HOME)         PORT_CHAR(UCHAR_MAMEKEY(HOME))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Insert")            PORT_CODE(KEYCODE_INSERT)       PORT_CHAR(UCHAR_MAMEKEY(INSERT))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Delete")            PORT_CODE(KEYCODE_DEL)          PORT_CHAR(UCHAR_MAMEKEY(DEL))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("End")               PORT_CODE(KEYCODE_END)          PORT_CHAR(UCHAR_MAMEKEY(END))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Page Up")           PORT_CODE(KEYCODE_PGUP)         PORT_CHAR(UCHAR_MAMEKEY(PGUP))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Page Down")         PORT_CODE(KEYCODE_PGDN)         PORT_CHAR(UCHAR_MAMEKEY(PGDN))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 0")              PORT_CODE(KEYCODE_0_PAD)        PORT_CHAR(UCHAR_MAMEKEY(0_PAD))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Up")                PORT_CODE(KEYCODE_UP)           PORT_CHAR(UCHAR_MAMEKEY(UP))

	PORT_START("col.a")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("L Control")         PORT_CODE(KEYCODE_LCONTROL)     PORT_CHAR(UCHAR_MAMEKEY(LCONTROL))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("L Alt")             PORT_CODE(KEYCODE_LALT)         PORT_CHAR(UCHAR_MAMEKEY(LALT))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Space")             PORT_CODE(KEYCODE_SPACE)        PORT_CHAR(' ')
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("R Alt")             PORT_CODE(KEYCODE_RALT)         PORT_CHAR(UCHAR_MAMEKEY(RALT))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Right")             PORT_CODE(KEYCODE_RIGHT)        PORT_CHAR(UCHAR_MAMEKEY(RIGHT))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("R Control")         PORT_CODE(KEYCODE_RCONTROL)     PORT_CHAR(UCHAR_MAMEKEY(RCONTROL))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Left")              PORT_CODE(KEYCODE_LEFT)         PORT_CHAR(UCHAR_MAMEKEY(LEFT))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Down")              PORT_CODE(KEYCODE_DOWN)         PORT_CHAR(UCHAR_MAMEKEY(DOWN))

	PORT_START("col.b")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Num Lock")          PORT_CODE(KEYCODE_NUMLOCK)      PORT_CHAR(UCHAR_MAMEKEY(NUMLOCK))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP /")              PORT_CODE(KEYCODE_SLASH_PAD)    PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 8")              PORT_CODE(KEYCODE_8_PAD)        PORT_CHAR(UCHAR_MAMEKEY(8_PAD))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 7")              PORT_CODE(KEYCODE_7_PAD)        PORT_CHAR(UCHAR_MAMEKEY(7_PAD))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 5")              PORT_CODE(KEYCODE_5_PAD)        PORT_CHAR(UCHAR_MAMEKEY(5_PAD))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 4")              PORT_CODE(KEYCODE_4_PAD)        PORT_CHAR(UCHAR_MAMEKEY(4_PAD))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 2")              PORT_CODE(KEYCODE_2_PAD)        PORT_CHAR(UCHAR_MAMEKEY(2_PAD))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 1")              PORT_CODE(KEYCODE_1_PAD)        PORT_CHAR(UCHAR_MAMEKEY(1_PAD))

	PORT_START("col.c")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP *")              PORT_CODE(KEYCODE_ASTERISK)     PORT_CHAR(UCHAR_MAMEKEY(ASTERISK))
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP -")              PORT_CODE(KEYCODE_MINUS_PAD)    PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD))
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP +")              PORT_CODE(KEYCODE_PLUS_PAD)     PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD))
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 9")              PORT_CODE(KEYCODE_9_PAD)        PORT_CHAR(UCHAR_MAMEKEY(9_PAD))
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("Enter")             PORT_CODE(KEYCODE_ENTER_PAD)    PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD))
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 6")              PORT_CODE(KEYCODE_6_PAD)        PORT_CHAR(UCHAR_MAMEKEY(6_PAD))
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP .")              PORT_CODE(KEYCODE_DEL_PAD)      PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_KEYBOARD) PORT_NAME("KP 3")              PORT_CODE(KEYCODE_3_PAD)        PORT_CHAR(UCHAR_MAMEKEY(3_PAD))

	PORT_START("col.d")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("col.e")
	PORT_BIT(0x01, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x02, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x04, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x08, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x10, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x20, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x40, IP_ACTIVE_HIGH, IPT_UNUSED)
	PORT_BIT(0x80, IP_ACTIVE_HIGH, IPT_UNUSED)

	PORT_START("col.f")
	PORT_DIPNAME(0xff, 0x00, "Layout")
	PORT_DIPSETTING(0x00, "us") // United States
	PORT_DIPSETTING(0x01, "de") // Germany
	PORT_DIPSETTING(0x02, "fr") // France
	PORT_DIPSETTING(0x03, "it") // Italy
	PORT_DIPSETTING(0x04, "dk") // Denmark
	PORT_DIPSETTING(0x05, "sp") // Spain
	PORT_DIPSETTING(0x06, "nw") // Norway
	PORT_DIPSETTING(0x07, "sw") // Sweden
	PORT_DIPSETTING(0x08, "sf") // Swiss French
	PORT_DIPSETTING(0x09, "uk") // United Kingdom
	PORT_DIPSETTING(0x0a, "be") // Belgium
	PORT_DIPSETTING(0x0b, "sg") // Swiss German
	PORT_DIPSETTING(0x0c, "nl") // Netherlands
	PORT_DIPSETTING(0x0d, "fn") // Finland
	PORT_DIPSETTING(0x0e, "po") // Portugal
	PORT_DIPSETTING(0x0f, "gr") // Greece
INPUT_PORTS_END

ROM_START(sgi_kbd)
	ROM_REGION(0x2000, "mcu", 0)
	ROM_LOAD("17159__1987__key_tronic.z6", 0x0, 0x2000, CRC(5502cb77) SHA1(a000a2b29e8629c686b29a5af22beedcc94710dc))
ROM_END

} // anonymous namespace

ioport_constructor sgi_kbd_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(sgi_kbd);
}

tiny_rom_entry const *sgi_kbd_device::device_rom_region() const
{
	return ROM_NAME(sgi_kbd);
}

void default_sgi_kbd_devices(device_slot_interface &device)
{
	device.option_add("keyboard", SGI_KBD);
	device.option_add("keyboard_hle", SGI_KBD_HLE);
}

// the Indigo's: the HLE keyboard by default, under the name the 8031 one
// had, so saved settings for the keyboard (layout, speaker) still apply
void indigo_sgi_kbd_devices(device_slot_interface &device)
{
	device.option_add("keyboard", SGI_KBD_HLE);
	device.option_add("keyboard_8031", SGI_KBD);
}

/*
 * The keyboard without its 8031, as its firmware behaves:
 *  - Each key is a matrix position (column x 8 + row) with a code from the
 *    firmware's table: pressing it sends the code with bit 7 clear,
 *    releasing it the code with bit 7 set. Bit 7 in the table marks the
 *    keys that don't click (modifiers and locks).
 *  - Bytes from the host with bit 0 clear: bit 1 short beep, bit 2 long
 *    beep, bit 3 key click off, bit 4 configuration request (answered with
 *    0x6e and the DIP switches), bits 5 and 6 LEDs (state bits 1 and 3),
 *    bit 7 auto-repeat on. With bit 0 set: bit 1 toggles LED state bits 1
 *    and 3, bits 2-6 set state bits 5, 0, 2, 4 and 6.
 *  - A 5 ms timer tick: the beeper sounds while its count runs down (a
 *    click is 2 ticks, a short beep 40, a long one 200); holding a key the
 *    firmware's table marks as repeating resends the last byte queued for
 *    the host after 130 ticks, then every 7, with a click.
 *  - Up to 8 bytes queue for the host; more are dropped.
 * Not reproduced: the firmware's debouncing (a release within ~80 ms of
 * its press is held back) and its restart when a repeat comes due with no
 * key down, which also flushes its queues (here the repeat just stops).
 */
namespace {

// key codes by matrix position, from the firmware
u8 const s_key_code[13 * 8] =
{
	0x15, 0x16, 0x18, 0x17, 0x19, 0x12, 0x23, 0x1c,
	0x56, 0x58, 0x5a, 0x5c, 0x5e, 0x60, 0x62, 0x64,
	0x0d, 0x0e, 0x10, 0x0f, 0x11, 0x0b, 0x1b, 0x14,
	0x36, 0x07, 0x09, 0x08, 0x0a, 0x83, 0x13, 0x85,
	0x06, 0x57, 0x59, 0x5b, 0x5d, 0x5f, 0x61, 0xe3,
	0x1d, 0x1e, 0x20, 0x1f, 0x21, 0x1a, 0x2b, 0x24,
	0x25, 0x26, 0x28, 0x27, 0x29, 0x22, 0x33, 0x2c,
	0x2d, 0x2e, 0x30, 0x2f, 0x31, 0x2a, 0x84, 0x34,
	0x35, 0x3c, 0x38, 0x37, 0x00, 0x32, 0x00, 0x00,
	0x66, 0x65, 0x3d, 0x68, 0x67, 0x69, 0x3a, 0x50,
	0x82, 0xd3, 0x52, 0xd4, 0x4f, 0xd5, 0x48, 0x49,
	0xea, 0x6b, 0x43, 0x42, 0x44, 0x3e, 0x3f, 0x39,
	0x6c, 0x4b, 0x6d, 0x4a, 0x51, 0x45, 0x41, 0x40,
};

// keys that repeat, one bit per row for each column, from the firmware
u8 const s_key_repeat[13] = { 0xff, 0x3f, 0xff, 0x5f, 0x7f, 0xff, 0xff, 0xbf, 0xff, 0xc4, 0xd4, 0xfe, 0xef };

enum : u8
{
	TICKS_CLICK = 2,
	TICKS_SHORT_BEEP = 0x28,
	TICKS_LONG_BEEP = 0xc8,
	TICKS_REPEAT_DELAY = 0x82,
	TICKS_REPEAT = 7,
};

} // anonymous namespace

sgi_kbd_hle_device::sgi_kbd_hle_device(machine_config const &mconfig, char const *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SGI_KBD_HLE, tag, owner, clock)
	, device_sgi_kbd_port_interface(mconfig, *this)
	, device_buffered_serial_interface(mconfig, *this)
	, m_matrix(*this, "col.%x", 0U)
	, m_beeper(*this, "beeper")
	, m_led(*this, "led.%u", 0U)
	, m_tick(nullptr)
{
}

void sgi_kbd_hle_device::device_add_mconfig(machine_config &config)
{
	speaker_device &speaker(SPEAKER(config, "speaker"));
	speaker.front_center();

	BEEP(config, m_beeper, 480);
	m_beeper->add_route(ALL_OUTPUTS, speaker, 0.25);
}

ioport_constructor sgi_kbd_hle_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(sgi_kbd);
}

void sgi_kbd_hle_device::device_start()
{
	m_tick = timer_alloc(FUNC(sgi_kbd_hle_device::tick), this);

	save_item(NAME(m_keys));
	save_item(NAME(m_leds));
	save_item(NAME(m_last_key));
	save_item(NAME(m_last_tx));
	save_item(NAME(m_repeat));
	save_item(NAME(m_beep));
	save_item(NAME(m_click));
	save_item(NAME(m_autorepeat));
}

void sgi_kbd_hle_device::device_reset()
{
	// 600 baud, 8 data bits, odd parity, 1 stop bit
	set_data_frame(1, 8, PARITY_ODD, STOP_BITS_1);
	set_rate(600);
	receive_register_reset();
	transmit_register_reset();
	clear_fifo();
	write_rxd(1);

	std::fill(std::begin(m_keys), std::end(m_keys), 0);
	m_leds = 0;
	m_last_key = 0;
	m_last_tx = 0;
	m_repeat = 0;
	m_beep = 0;
	m_click = true;
	m_autorepeat = false;
	m_beeper->set_state(0);
	update_leds();

	m_tick->adjust(attotime::from_msec(5), 0, attotime::from_msec(5));
}

void sgi_kbd_hle_device::write_txd(int state)
{
	rx_w(state);
}

void sgi_kbd_hle_device::tra_callback()
{
	write_rxd(transmit_register_get_data_bit());
}

void sgi_kbd_hle_device::rcv_complete()
{
	// the firmware drops a byte with bad parity
	receive_register_extract();
	if (!is_receive_parity_error())
		received_byte(get_received_char());
}

void sgi_kbd_hle_device::received_byte(u8 byte)
{
	if (!BIT(byte, 0))
	{
		if (BIT(byte, 1))
			m_beep = TICKS_SHORT_BEEP;
		if (BIT(byte, 2))
			m_beep = TICKS_LONG_BEEP;
		m_click = !BIT(byte, 3);
		if (BIT(byte, 4))
		{
			queue(0x6e);
			queue(m_matrix[15]->read());
		}
		m_leds = (m_leds & ~0x0a) | (BIT(byte, 5) << 1) | (BIT(byte, 6) << 3);
		m_autorepeat = BIT(byte, 7);
	}
	else
	{
		if (BIT(byte, 1))
			m_leds ^= 0x0a;
		m_leds = (m_leds & 0x8a) | (BIT(byte, 2) << 5) | (BIT(byte, 3) << 0) | (BIT(byte, 4) << 2) | (BIT(byte, 5) << 4) | (BIT(byte, 6) << 6);
	}
	update_leds();
}

void sgi_kbd_hle_device::update_leds()
{
	for (unsigned i = 0; i < 7; i++)
		m_led[i] = BIT(~m_leds, i);
}

void sgi_kbd_hle_device::queue(u8 data)
{
	if (fifo_full())
		return;
	transmit_byte(data);
	m_last_tx = data;
}

TIMER_CALLBACK_MEMBER(sgi_kbd_hle_device::tick)
{
	if (m_beep)
		m_beeper->set_state(--m_beep ? 1 : 0);

	if (m_repeat && !--m_repeat && m_autorepeat)
	{
		bool held = false;
		for (u8 const keys : m_keys)
			held = held || keys;
		if (held)
		{
			m_repeat = TICKS_REPEAT;
			queue(m_last_tx);
			if (m_click)
				m_beep = TICKS_CLICK;
		}
	}

	scan();
}

void sgi_kbd_hle_device::scan()
{
	for (unsigned col = 0; col < std::size(m_keys); col++)
	{
		u8 const keys = m_matrix[col]->read();
		u8 changed = keys ^ m_keys[col];
		if (!changed)
			continue;
		m_keys[col] = keys;
		for (unsigned row = 0; changed; row++, changed >>= 1)
		{
			if (!BIT(changed, 0))
				continue;
			u8 const index = col * 8 + row;
			u8 const code = s_key_code[index];
			if (BIT(keys, row))
			{
				if (!BIT(code, 7) && m_click)
					m_beep = TICKS_CLICK;
				queue(code & 0x7f);
				m_last_key = index;
				m_repeat = BIT(s_key_repeat[col], row) ? TICKS_REPEAT_DELAY : 0;
			}
			else
			{
				if (index == m_last_key)
				{
					m_last_key = 0;
					m_repeat = 0;
				}
				queue(code | 0x80);
			}
		}
	}
}
