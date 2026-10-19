// license:BSD-3-Clause
// copyright-holders:
/**********************************************************************

    The reduced SGI build's (SUBTARGET=sgi, MAME_SGI_REDUCED_BUILD)
    default_rs232_devices(), in place of bus/rs232/rs232.cpp's, which that
    build leaves out together with the ~19 historic serial peripherals it
    names (h19, ie15, scorpion, votraxtnt, mockingboard, ...). A full build
    uses rs232.cpp's list and compiles nothing from this file.

    Kept: what's useful for emulator I/O on this machine, a generic
    terminal, host-side console access via null_modem or pty, and a
    loopback for testing. ioc2.cpp (the caller) links against this by
    name.

**********************************************************************/

#include "emu.h"

#ifdef MAME_SGI_REDUCED_BUILD

#include "bus/rs232/rs232.h"
#include "bus/rs232/loopback.h"
#include "bus/rs232/null_modem.h"
#include "bus/rs232/pty.h"
#include "bus/rs232/terminal.h"

void default_rs232_devices(device_slot_interface &device)
{
	device.option_add("loopback",   RS232_LOOPBACK);
	device.option_add("null_modem", NULL_MODEM);
	device.option_add("pty",        PSEUDO_TERMINAL);
	device.option_add("terminal",   SERIAL_TERMINAL);
}

#endif // MAME_SGI_REDUCED_BUILD
