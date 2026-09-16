// license:BSD-3-Clause
// copyright-holders:
/**********************************************************************

    Bespoke replacement for bus/centronics/ctronics.cpp's
    centronics_devices(). See sgi_rs232_devices.cpp for the full
    explanation — same reason, same pattern: upstream's version pulls in
    ~18 unrelated Centronics-port peripherals (chess computer interfaces,
    dot-matrix printers, sound adapters) that don't apply to an SGI Indy,
    and MAME's makedep.py dependency scanner can't be steered away from
    them with a preprocessor guard because it doesn't understand one.

    machine/pc_lpt.cpp (the only caller) wires up "printer" as its actual
    default device, so that's the only option kept here.

**********************************************************************/

#include "emu.h"
#include "bus/centronics/ctronics.h"
#include "bus/centronics/printer.h"

void centronics_devices(device_slot_interface &device)
{
	device.option_add("printer", CENTRONICS_PRINTER);
}
