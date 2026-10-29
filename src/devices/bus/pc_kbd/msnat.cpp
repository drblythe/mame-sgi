// license:BSD-3-Clause
// copyright-holders:Wilbert Pol
/***************************************************************************

    Microsoft Natural Keybaord emulation


TODO:
- Keyboard LEDs

***************************************************************************/

#include "emu.h"
#include "msnat.h"

#include <chrono>
#include <cstdio>
#include <ctime>


/***************************************************************************
    ONSTANTS
***************************************************************************/

#define LOG     0

// Diagnostic, compiled out by default, for tracing dropped keystrokes.
// input_sdl.cpp's MAME_SGI_KBD_RAW_EVENT_DIAG
// already proved the host SDL layer delivers every keystroke correctly
// during a heavy GUI-session-restore stall; this is the next hop down
// the chain, the embedded 8051 keyboard-controller firmware's own
// matrix-scan reads (p0_read(), called by the 8051 CPU itself whenever
// its firmware polls a currently-selected row). Logs only non-idle
// reads (data != 0xFF, i.e. the firmware just detected some key down on
// whatever row(s) m_p1/m_p2 currently have selected) - genuinely rare
// events (only real keypresses), no rate limiting needed, unlike the
// per-dispatch-style diagnostics elsewhere in this fork. Same
// wall-clock HH:MM:SS.mmm format as MAME_SGI_KBD_RAW_EVENT_DIAG for
// direct cross-reference. Uncomment the define below to enable.
// #define MAME_SGI_KBD_MATRIX_SCAN_DIAG

#ifdef MAME_SGI_KBD_MATRIX_SCAN_DIAG
namespace {
std::string kbd_matrix_scan_timestamp()
{
	using namespace std::chrono;
	auto const now = system_clock::now();
	auto const t = system_clock::to_time_t(now);
	auto const ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;
	std::tm tm_buf;
	localtime_r(&t, &tm_buf);
	char buf[16];
	std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, int(ms.count()));
	return buf;
}
}
#endif

// Diagnostic, compiled out by default, for keys dropped during
// a host stall: MAME_SGI_KBD_MATRIX_SCAN_DIAG proved a key that IS
// correctly toggled at the host layer (a real, guaranteed-visible
// state=0x80 window, confirmed with a host-side key state trace in
// input_sdl.cpp) can still never be observed by this
// firmware's own matrix scan. The remaining question is *why*: does the
// 8051 simply not run enough during a stall, or does it run plenty but
// its own row-select scan cycle - visiting all 16 rows in sequence -
// take longer than the host's guaranteed-visible window, so by the time
// the scan cycles back around to the ONE row a given key lives on, the
// window has already closed? Unconditionally counts every p0_read()
// call (every row-select sample, not just non-idle ones) and, per row-
// select value, the worst (largest) real-time gap ever seen between two
// consecutive samples of that *same* select value - i.e. the row's own
// scan-revisit period. Reported periodically (rate-limited), not
// per-call - p0_read() fires far too often (thousands/sec) for
// per-call logging like MATRIX_SCAN_DIAG uses. A measured worst_gap of
// 373.9ms correlated with a drc_cache flush. Uncomment the define below
// to enable.
// #define MAME_SGI_KBD_SCAN_RATE_DIAG

#ifdef MAME_SGI_KBD_SCAN_RATE_DIAG
namespace {
constexpr int KBD_SCAN_RATE_NUM_SELECTS = 16; // 8 P2.x rows + 8 P1.x rows, distinguished by which register is non-0xff
uint64_t s_scan_rate_total_calls = 0;
double s_scan_rate_last_seen_ms[KBD_SCAN_RATE_NUM_SELECTS] = {0};
double s_scan_rate_worst_gap_ms[KBD_SCAN_RATE_NUM_SELECTS] = {0};
double s_scan_rate_worst_gap_end_ms[KBD_SCAN_RATE_NUM_SELECTS] = {0}; // wall-clock time the worst gap's scan resumed - for correlating against other timestamped diagnostics (e.g. DRC compile-rate)
uint64_t s_scan_rate_visit_count[KBD_SCAN_RATE_NUM_SELECTS] = {0};
double s_scan_rate_first_seen_ms[KBD_SCAN_RATE_NUM_SELECTS] = {0};
double s_scan_rate_next_report_ms = 0;

double kbd_scan_rate_now_ms()
{
	using namespace std::chrono;
	return duration<double, std::milli>(system_clock::now().time_since_epoch()).count();
}

// Formats an arbitrary epoch-ms value (not necessarily "now") - used to
// print WHEN the worst gap was recorded, not just its size, so it can
// be lined up against other timestamped diagnostics (e.g. a DRC
// compile-rate counter using the same HH:MM:SS.mmm format).
std::string kbd_scan_rate_format_ms(double epoch_ms)
{
	using namespace std::chrono;
	auto const dur = duration<double, std::milli>(epoch_ms);
	auto const tp = system_clock::time_point(duration_cast<system_clock::duration>(dur));
	auto const t = system_clock::to_time_t(tp);
	auto const ms = int64_t(epoch_ms) % 1000;
	std::tm tm_buf;
	localtime_r(&t, &tm_buf);
	char buf[16];
	std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d", tm_buf.tm_hour, tm_buf.tm_min, tm_buf.tm_sec, int(ms));
	return buf;
}

// Returns the row index (0-15) this exact (p2,p1) select pair
// corresponds to, or -1 if not a single clean row select (multiple
// rows active at once, or none - both real, just not useful for this
// per-row cadence measurement).
int kbd_scan_rate_row_index(uint8_t p2, uint8_t p1)
{
	int p2_row = -1, p1_row = -1, p2_count = 0, p1_count = 0;
	for (int i = 0; i < 8; i++)
	{
		if (!(p2 & (1 << i))) { p2_row = i; p2_count++; }
		if (!(p1 & (1 << i))) { p1_row = i; p1_count++; }
	}
	if (p2_count == 1 && p1_count == 0)
		return p2_row;
	if (p1_count == 1 && p2_count == 0)
		return 8 + p1_row;
	return -1;
}
}
#endif


/*****************************************************************************
    INPUT PORTS
*****************************************************************************/

static INPUT_PORTS_START( microsoft_natural )
	PORT_START( "P2.0" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_Y)            PORT_CHAR('Y')                      // 15
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_H)            PORT_CHAR('H')                      // 23
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_J)            PORT_CHAR('J')                      // 24
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_N)            PORT_CHAR('N')                      // 31
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_M)            PORT_CHAR('M')                      // 32
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_U)            PORT_CHAR('U')                      // 16
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_6)            PORT_CHAR('6')                      // 07
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_7)            PORT_CHAR('7')                      // 08

	PORT_START( "P2.1" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_EQUALS)       PORT_CHAR('=')                      // 0D
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_CLOSEBRACE)   PORT_CHAR(']')                      // 1B
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_BACKSLASH)    PORT_CHAR('\\')                     // 2B    2 spots for backslash?
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_LEFT)         PORT_CHAR(UCHAR_MAMEKEY(LEFT))      // E0 4B
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_ENTER)        PORT_CHAR(13)                       // 1C
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_BACKSPACE)    PORT_CHAR(8)                        // 0E
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F5)           PORT_CHAR(UCHAR_MAMEKEY(F5))        // 3F
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F6)           PORT_CHAR(UCHAR_MAMEKEY(F6))        // 40

	PORT_START( "P2.2" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_9)            PORT_CHAR('9')                      // 0A
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_O)            PORT_CHAR('O')                      // 18
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_OPENBRACE)    PORT_CHAR('[')                      // 1A
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_L)            PORT_CHAR('L')                      // 26
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_STOP)         PORT_CHAR('.')                      // 34
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_MINUS)        PORT_CHAR('-')                      // 0C
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F7)           PORT_CHAR(UCHAR_MAMEKEY(F7))        // 41
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F8)           PORT_CHAR(UCHAR_MAMEKEY(F8))        // 42

	PORT_START( "P2.3" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_0)            PORT_CHAR('0')                      // 0B
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_COLON)        PORT_CHAR(';')                      // 27
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_QUOTE)        PORT_CHAR('\'')                     // 28
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_SLASH)        PORT_CHAR('/')                      // 35
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_DOWN)         PORT_CHAR(UCHAR_MAMEKEY(DOWN))      // E0 50
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_P)            PORT_CHAR('P')                      // 19
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F9)           PORT_CHAR(UCHAR_MAMEKEY(F9))        // 43
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F10)          PORT_CHAR(UCHAR_MAMEKEY(F10))       // 44

	PORT_START( "P2.4" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_UP)           PORT_CHAR(UCHAR_MAMEKEY(UP))        // E0 48
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )                                   PORT_NAME("Unknown 73")             // 73 TODO
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )                                   PORT_NAME("\\ 2nd?")                // 2B    2 spots for backslash??
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_RWIN)         PORT_CHAR(UCHAR_MAMEKEY(RWIN))      // E0 5C
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_RIGHT)        PORT_CHAR(UCHAR_MAMEKEY(RIGHT))     // E0 4D
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_SPACE)        PORT_CHAR(' ')                      // 39

	PORT_START( "P2.5" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_HOME)         PORT_CHAR(UCHAR_MAMEKEY(HOME))      // E0 47
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_8_PAD)        PORT_CHAR(UCHAR_MAMEKEY(8_PAD))     // 48
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_5_PAD)        PORT_CHAR(UCHAR_MAMEKEY(5_PAD))     // 4C
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_2_PAD)        PORT_CHAR(UCHAR_MAMEKEY(2_PAD))     // 50
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_0_PAD)        PORT_CHAR(UCHAR_MAMEKEY(0_PAD))     // 52
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_END)          PORT_CHAR(UCHAR_MAMEKEY(END))       // E0 4F
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F11)          PORT_CHAR(UCHAR_MAMEKEY(F11))       // 57
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F12)          PORT_CHAR(UCHAR_MAMEKEY(F12))       // 58

	PORT_START( "P2.6" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_INSERT)       PORT_CHAR(UCHAR_MAMEKEY(INSERT))    // E0 52
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_DEL)          PORT_CHAR(UCHAR_MAMEKEY(DEL))       // E0 53
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_6_PAD)        PORT_CHAR(UCHAR_MAMEKEY(6_PAD))     // 4D
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_3_PAD)        PORT_CHAR(UCHAR_MAMEKEY(3_PAD))     // 51
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_DEL_PAD)      PORT_CHAR(UCHAR_MAMEKEY(DEL_PAD))   // 53
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_9_PAD)        PORT_CHAR(UCHAR_MAMEKEY(9_PAD))     // 49
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F3)           PORT_CHAR(UCHAR_MAMEKEY(F3))        // 3D
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F4)           PORT_CHAR(UCHAR_MAMEKEY(F4))        // 3E

	PORT_START( "P2.7" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_SLASH_PAD)    PORT_CHAR(UCHAR_MAMEKEY(SLASH_PAD)) // E0
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_7_PAD)        PORT_CHAR(UCHAR_MAMEKEY(7_PAD))     // 47
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_4_PAD)        PORT_CHAR(UCHAR_MAMEKEY(4_PAD))     // 4B
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_PLUS_PAD)     PORT_CHAR(UCHAR_MAMEKEY(PLUS_PAD))  // 4E
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_1_PAD)        PORT_CHAR(UCHAR_MAMEKEY(1_PAD))     // 4F
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_ASTERISK)     PORT_CHAR(UCHAR_MAMEKEY(ASTERISK))  // 37   TODO
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_PRTSCR)       PORT_CHAR(UCHAR_MAMEKEY(PRTSCR))    // E0 2A E0 37
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_MENU)         PORT_CHAR(UCHAR_MAMEKEY(MENU))      // E0 5D

	PORT_START( "P1.0" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_LWIN)         PORT_CHAR(UCHAR_MAMEKEY(LWIN))      // E0 5B
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )                                   PORT_NAME("INT5 7E")                // 7E INT5
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_SCRLOCK)      PORT_CHAR(UCHAR_MAMEKEY(SCRLOCK))   // 46
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_ENTER_PAD)    PORT_CHAR(UCHAR_MAMEKEY(ENTER_PAD)) // E0 1C

	PORT_START( "P1.1" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_LCONTROL)     PORT_CHAR(UCHAR_MAMEKEY(LCONTROL))  // 1D
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_RCONTROL)     PORT_CHAR(UCHAR_MAMEKEY(RCONTROL))  // E0 1D
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_CAPSLOCK)     PORT_CHAR(UCHAR_MAMEKEY(CAPSLOCK))  // 3A
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF

	PORT_START( "P1.2" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_PGUP)         PORT_CHAR(UCHAR_MAMEKEY(PGUP))      // E0 49
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_2)            PORT_CHAR('2')                      // 03
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_W)            PORT_CHAR('W')                      // 11
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_S)            PORT_CHAR('S')                      // 1F
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_X)            PORT_CHAR('X')                      // 2D
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_PGDN)         PORT_CHAR(UCHAR_MAMEKEY(PGDN))      // E0 51
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F1)           PORT_CHAR(UCHAR_MAMEKEY(F1))        // 3B
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F2)           PORT_CHAR(UCHAR_MAMEKEY(F2))        // 3C

	PORT_START( "P1.3" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_LALT)         PORT_CHAR(UCHAR_MAMEKEY(LALT))      // 38
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_RALT)         PORT_CHAR(UCHAR_MAMEKEY(RALT))      // E0 38
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_MINUS_PAD)    PORT_CHAR(UCHAR_MAMEKEY(MINUS_PAD)) // 4A
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_NUMLOCK)      PORT_CHAR(UCHAR_MAMEKEY(NUMLOCK))   // 45

	PORT_START( "P1.4" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_I)            PORT_CHAR('I')                      // 17
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_K)            PORT_CHAR('K')                      // 25
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_D)            PORT_CHAR('D')                      // 20
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_C)            PORT_CHAR('C')                      // 2E
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_COMMA)        PORT_CHAR(',')                      // 33
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_E)            PORT_CHAR('E')                      // 12
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_3)            PORT_CHAR('3')                      // 04
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_8)            PORT_CHAR('8')                      // 09

	PORT_START( "P1.5" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_R)            PORT_CHAR('R')                      // 13
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_F)            PORT_CHAR('F')                      // 21
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_G)            PORT_CHAR('G')                      // 22
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_V)            PORT_CHAR('V')                      // 2F
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_B)            PORT_CHAR('B')                      // 30
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_T)            PORT_CHAR('T')                      // 14
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_4)            PORT_CHAR('4')                      // 05
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_5)            PORT_CHAR('5')                      // 06

	PORT_START( "P1.6" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_1)            PORT_CHAR('1')                      // 02
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_Q)            PORT_CHAR('Q')                      // 10
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_A)            PORT_CHAR('A')                      // 1E
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_KEYBOARD )                                   PORT_NAME("INT1 56")                // 56 INT1?
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_Z)            PORT_CHAR('Z')                      // 2C
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_TAB)          PORT_CHAR(9)                        // 0F
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_ESC)          PORT_CHAR(UCHAR_MAMEKEY(ESC))       // 01
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_TILDE)        PORT_CHAR('`')                      // 29

	PORT_START( "P1.7" )
	PORT_BIT( 0x01, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x02, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_LSHIFT)       PORT_CHAR(UCHAR_MAMEKEY(LSHIFT))    // 2A
	PORT_BIT( 0x04, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x08, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x10, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_RSHIFT)       PORT_CHAR(UCHAR_MAMEKEY(RSHIFT))    // 36
	PORT_BIT( 0x20, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF
	PORT_BIT( 0x40, IP_ACTIVE_LOW, IPT_KEYBOARD )   PORT_CODE(KEYCODE_CANCEL)       PORT_CHAR(UCHAR_MAMEKEY(CANCEL))    // E1 1D 45 E1 9D C5
	PORT_BIT( 0x80, IP_ACTIVE_LOW, IPT_UNUSED )                                                                         // FF

INPUT_PORTS_END


//**************************************************************************
//  GLOBAL VARIABLES
//**************************************************************************

DEFINE_DEVICE_TYPE(PC_KBD_MICROSOFT_NATURAL, pc_kbd_microsoft_natural_device, "kb_ms_natural", "Microsoft Natural Keyboard")

/***************************************************************************
    ROM DEFINITIONS
***************************************************************************/

ROM_START( microsoft_natural )
	ROM_REGION(0x1000, "ms_natrl_cpu", 0)
	ROM_LOAD("natural.bin", 0x0000, 0x1000, CRC(aa8243ab) SHA1(72134882a5c03e785db07cc54dfb7572c0a730d9))
ROM_END


pc_kbd_microsoft_natural_device::pc_kbd_microsoft_natural_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, PC_KBD_MICROSOFT_NATURAL, tag, owner, clock)
	, device_pc_kbd_interface(mconfig, *this)
	, m_cpu(*this, "ms_natrl_cpu")
	, m_p2_r(*this, "P2.%u", 0)
	, m_p1_r(*this, "P1.%u", 0)
	, m_p0(0), m_p1(0), m_p2(0), m_p3(0)
{
}


//-------------------------------------------------
//  device_start - device-specific startup
//-------------------------------------------------
void pc_kbd_microsoft_natural_device::device_start()
{
	set_pc_kbdc_device();

	/* setup savestates */
	save_item(NAME(m_p0));
	save_item(NAME(m_p1));
	save_item(NAME(m_p2));
	save_item(NAME(m_p3));
}


//-------------------------------------------------
//  device_reset - device-specific reset
//-------------------------------------------------
void pc_kbd_microsoft_natural_device::device_reset()
{
	/* set default values */
}


//-------------------------------------------------
//  device_add_mconfig - add device configuration
//-------------------------------------------------

void pc_kbd_microsoft_natural_device::device_add_mconfig(machine_config &config)
{
	I8051(config, m_cpu, XTAL(6'000'000));
	m_cpu->port_in_cb<0>().set(FUNC(pc_kbd_microsoft_natural_device::p0_read));
	m_cpu->port_out_cb<0>().set(FUNC(pc_kbd_microsoft_natural_device::p0_write));
	m_cpu->port_out_cb<1>().set(FUNC(pc_kbd_microsoft_natural_device::p1_write));
	m_cpu->port_out_cb<2>().set(FUNC(pc_kbd_microsoft_natural_device::p2_write));
	m_cpu->port_in_cb<3>().set(FUNC(pc_kbd_microsoft_natural_device::p3_read));
	m_cpu->port_out_cb<3>().set(FUNC(pc_kbd_microsoft_natural_device::p3_write));
}


ioport_constructor pc_kbd_microsoft_natural_device::device_input_ports() const
{
	return INPUT_PORTS_NAME( microsoft_natural );
}


//-------------------------------------------------
//  rom_region - device-specific ROM region
//-------------------------------------------------

const tiny_rom_entry *pc_kbd_microsoft_natural_device::device_rom_region() const
{
	return ROM_NAME( microsoft_natural );
}


void pc_kbd_microsoft_natural_device::clock_write(int state)
{
}


void pc_kbd_microsoft_natural_device::data_write(int state)
{
}


uint8_t pc_kbd_microsoft_natural_device::p0_read()
{
	uint8_t data = 0xFF;

	if (LOG)
		logerror("%s: P0 read. P1 = %02x, P2 = %02x\n", tag(), m_p1, m_p2 );

	if ( ! ( m_p2 & 0x01 ) )
		data &= m_p2_r[0]->read();

	if ( ! ( m_p2 & 0x02 ) )
		data &= m_p2_r[1]->read();

	if ( ! ( m_p2 & 0x04 ) )
		data &= m_p2_r[2]->read();

	if ( ! ( m_p2 & 0x08 ) )
		data &= m_p2_r[3]->read();

	if ( ! ( m_p2 & 0x10 ) )
		data &= m_p2_r[4]->read();

	if ( ! ( m_p2 & 0x20 ) )
		data &= m_p2_r[5]->read();

	if ( ! ( m_p2 & 0x40 ) )
		data &= m_p2_r[6]->read();

	if ( ! ( m_p2 & 0x80 ) )
		data &= m_p2_r[7]->read();

	if ( ! ( m_p1 & 0x01 ) )
		data &= m_p1_r[0]->read();

	if ( ! ( m_p1 & 0x02 ) )
		data &= m_p1_r[1]->read();

	if ( ! ( m_p1 & 0x04 ) )
		data &= m_p1_r[2]->read();

	if ( ! ( m_p1 & 0x08 ) )
		data &= m_p1_r[3]->read();

	if ( ! ( m_p1 & 0x10 ) )
		data &= m_p1_r[4]->read();

	if ( ! ( m_p1 & 0x20 ) )
		data &= m_p1_r[5]->read();

	if ( ! ( m_p1 & 0x40 ) )
		data &= m_p1_r[6]->read();

	if ( ! ( m_p1 & 0x80 ) )
		data &= m_p1_r[7]->read();

#ifdef MAME_SGI_KBD_MATRIX_SCAN_DIAG
	if (data != 0xFF)
	{
		osd_printf_info("MAME_SGI_KBD_MATRIX_SCAN_DIAG: %s select(P2)=%02x select(P1)=%02x data=%02x\n",
			kbd_matrix_scan_timestamp().c_str(), m_p2, m_p1, data);
	}
#endif

#ifdef MAME_SGI_KBD_SCAN_RATE_DIAG
	{
		double const now = kbd_scan_rate_now_ms();
		s_scan_rate_total_calls++;
		int const row = kbd_scan_rate_row_index(m_p2, m_p1);
		if (row >= 0)
		{
			double const last = s_scan_rate_last_seen_ms[row];
			if (last != 0.0)
			{
				double const gap = now - last;
				if (gap > s_scan_rate_worst_gap_ms[row])
				{
					s_scan_rate_worst_gap_ms[row] = gap;
					s_scan_rate_worst_gap_end_ms[row] = now; // when the scan resumed after the gap - the DRC compile-rate counter should show a spike shortly before this moment if the two are related
				}
			}
			else
			{
				s_scan_rate_first_seen_ms[row] = now;
			}
			s_scan_rate_last_seen_ms[row] = now;
			s_scan_rate_visit_count[row]++;
		}
		if (s_scan_rate_next_report_ms == 0.0)
			s_scan_rate_next_report_ms = now + 1000.0;
		if (now >= s_scan_rate_next_report_ms)
		{
			// Row 10 = P2.2 (the row 'O'/'L'/'9'/'.'/'-'/F7/F8 live on) -
			// the one directly implicated by MATRIX_SCAN_DIAG's evidence.
			// avg_period is real elapsed time since this row's first
			// visit divided by its visit count; worst_gap is the single
			// largest real gap ever seen between two consecutive visits
			// to that exact row - i.e. the longest window this row was
			// not being sampled at all.
			double const elapsed = s_scan_rate_visit_count[10] ? (now - s_scan_rate_first_seen_ms[10]) : 0.0;
			osd_printf_info("MAME_SGI_KBD_SCAN_RATE_DIAG: %s total_calls=%llu P2.2(row10): visits=%llu avg_period=%.3fms worst_gap=%.3fms worst_gap_end=%s\n",
				kbd_matrix_scan_timestamp().c_str(),
				(unsigned long long)s_scan_rate_total_calls,
				(unsigned long long)s_scan_rate_visit_count[10],
				s_scan_rate_visit_count[10] > 1 ? (elapsed / double(s_scan_rate_visit_count[10] - 1)) : 0.0,
				s_scan_rate_worst_gap_ms[10],
				s_scan_rate_worst_gap_end_ms[10] != 0.0 ? kbd_scan_rate_format_ms(s_scan_rate_worst_gap_end_ms[10]).c_str() : "n/a");
			s_scan_rate_next_report_ms = now + 1000.0;
		}
	}
#endif

	return data;
}


void pc_kbd_microsoft_natural_device::p0_write(uint8_t data)
{
	m_p0 = data;
}


void pc_kbd_microsoft_natural_device::p1_write(uint8_t data)
{
	m_p1 = data;
}


void pc_kbd_microsoft_natural_device::p2_write(uint8_t data)
{
	m_p2 = data;
}


uint8_t pc_kbd_microsoft_natural_device::p3_read()
{
	uint8_t data = m_p3 & ~0x21;

	// (Incoming) Clock signal is tied to the T1/P3.5 pin
	data |= (clock_signal() ? 0x20 : 0x00);

	// (Incoming) Data signal is tied to the RXD/P3.0 pin
	data |= ( data_signal() ? 0x01 : 0x00 );

	return data;
}


void pc_kbd_microsoft_natural_device::p3_write(uint8_t data)
{
	if ( m_pc_kbdc )
	{
		// (Outgoing) data signal is tied to the WR/P3.6 pin
		m_pc_kbdc->data_write_from_kb( BIT(data, 6) );

		// (Outgoing) clock signal is tied to the T0/P3.4 pin
		m_pc_kbdc->clock_write_from_kb( BIT(data, 4) );
	}

	m_p3 = data;
}
