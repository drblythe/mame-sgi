// license:BSD-3-Clause
// copyright-holders:
/**********************************************************************

    Bespoke replacement for bus/rs232/rs232.cpp's default_rs232_devices().

    Upstream's version references ~19 historic serial peripheral types
    (h19, ie15, scorpion, votraxtnt, mockingboard, etc.) by name, none of
    which apply to an SGI Indy. That function was removed from rs232.cpp
    for this build (see the comment left there) because MAME's makedep.py
    dependency scanner reads #include lines as plain text with no
    #ifdef/#endif awareness, so a conditional guard couldn't stop it from
    pulling in every one of those files regardless of which branch would
    actually compile — only physically removing them works.

    This file defines the same function, under the same name and
    signature, with only what's actually useful for emulator I/O on this
    machine: a generic terminal, host-side console access via null_modem
    or pty, and a loopback for testing. ioc2.cpp (the only caller) is
    unmodified and links against this by name, same as it always did.

**********************************************************************/

#include "emu.h"
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
