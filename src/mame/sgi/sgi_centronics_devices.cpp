// license:BSD-3-Clause
// copyright-holders:smf, blythe
/**********************************************************************

    The reduced SGI build's (SUBTARGET=sgi, MAME_SGI_REDUCED_BUILD)
    centronics_devices(), in place of bus/centronics/ctronics.cpp's, which
    that build leaves out together with the ~18 peripherals it names (chess
    computer interfaces, dot-matrix printers, sound adapters). A full build
    uses ctronics.cpp's list and compiles nothing from this file.

    machine/pc_lpt.cpp (the only caller in the SGI drivers) wires up
    "printer" as its default device, so that's the only option kept here.

**********************************************************************/

#include "emu.h"

#ifdef MAME_SGI_REDUCED_BUILD

#include "bus/centronics/ctronics.h"
#include "bus/centronics/printer.h"

void centronics_devices(device_slot_interface &device)
{
	device.option_add("printer", CENTRONICS_PRINTER);
}

#endif // MAME_SGI_REDUCED_BUILD
