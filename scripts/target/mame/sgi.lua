-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   sgi.lua
--
--   Small driver-specific example makefile
--   Use make SUBTARGET=sgi to build
--
---------------------------------------------------------------------------


--------------------------------------------------
-- Specify all the CPU cores necessary for the
-- drivers referenced in sgi.lst.
--------------------------------------------------

--CPUS["COP400"] = true
--CPUS["M6502"] = true
--CPUS["M6800"] = true
--CPUS["M6805"] = true
--CPUS["M6809"] = true
--CPUS["M680X0"] = true
--CPUS["MCS48"] = true
--CPUS["MCS51"] = true
--CPUS["TMS9900"] = true
--CPUS["Z80"] = true
CPUS["ARM"] = true
CPUS["M6800"] = true
CPUS["M6805"] = true
CPUS["MCS48"] = true
CPUS["MCS51"] = true
CPUS["MIPS3"] = true

--------------------------------------------------
-- Specify all the sound cores necessary for the
-- drivers referenced in sgi.lst.
--------------------------------------------------

--SOUNDS["ASTROCADE"] = true
--SOUNDS["AY8910"] = true
--SOUNDS["CEM3394"] = true
--SOUNDS["DAC"] = true
--SOUNDS["DISCRETE"] = true
--SOUNDS["HC55516"] = true
--SOUNDS["OKIM6295"] = true
--SOUNDS["SAMPLES"] = true
--SOUNDS["SN76496"] = true
--SOUNDS["TMS5220"] = true
--SOUNDS["VOTRAX_SC01"] = true
--SOUNDS["YM2151"] = true
--SOUNDS["YM3812"] = true
SOUNDS["BEEP"] = true
SOUNDS["CDDA"] = true
SOUNDS["DAC"] = true


--------------------------------------------------
-- specify available video cores
--------------------------------------------------

--VIDEOS["MC6845"] = true
VIDEOS["MC6845"] = true
VIDEOS["PS2GIF"] = true
VIDEOS["PS2GS"] = true


--------------------------------------------------
-- specify available machine cores
--------------------------------------------------

--MACHINES["6821PIA"] = true
--MACHINES["68681"] = true
--MACHINES["ADC0808"] = true
--MACHINES["BANKDEV"] = true
--MACHINES["GEN_LATCH"] = true
--MACHINES["INPUT_MERGER"] = true
--MACHINES["MOS6530"] = true
--MACHINES["NETLIST"] = true
--MACHINES["OUTPUT_LATCH"] = true
--MACHINES["PIT8253"] = true
--MACHINES["SEGACRPT"] = true
--MACHINES["TICKET"] = true
--MACHINES["TIMEKPR"] = true
--MACHINES["TTL74148"] = true
--MACHINES["TTL74153"] = true
--MACHINES["TTL74157"] = true
--MACHINES["TTL74259"] = true
--MACHINES["TTL7474"] = true
--MACHINES["WATCHDOG"] = true
--MACHINES["Z80CTC"] = true
--MACHINES["Z80DAISY"] = true
--MACHINES["Z80PIO"] = true
MACHINES["6821PIA"] = true
MACHINES["ACIA6850"] = true
MACHINES["AT_KEYBC"] = true
MACHINES["BANKDEV"] = true
MACHINES["CHESSMACHINE"] = true
MACHINES["DS1386"] = true
MACHINES["EDLC"] = true
MACHINES["EEPROMDEV"] = true
MACHINES["INPUT_MERGER"] = true
MACHINES["INS8250"] = true
MACHINES["NSCSI"] = true
MACHINES["OUTPUT_LATCH"] = true
MACHINES["PC_LPT"] = true
MACHINES["PCF8573"] = true
MACHINES["PIT8253"] = true
MACHINES["PS2INTC"] = true  -- ps2gs.cpp (VIDEOS["PS2GS"]) needs ps2_intc_device
MACHINES["SAA7191"] = true
MACHINES["SWTPC8212"] = true
MACHINES["VOTRAXTNT"] = true
MACHINES["WD33C9X"] = true
MACHINES["Z80DAISY"] = true  -- z80scc.cpp (MACHINES["Z80SCC"]) needs device_z80daisy_interface
MACHINES["Z80SCC"] = true


--------------------------------------------------
-- specify available bus cores
--------------------------------------------------

--BUSES["CENTRONICS"] = true
-- BUSES["CENTRONICS"] = true  -- pulls in ~18 unrelated Centronics peripherals
-- (chess interfaces, dot-matrix printers, sound adapters) via ctronics.cpp's
-- centronics_devices(). Actually needed transitively — MACHINES["PC_LPT"]'s
-- pc_lpt.h #includes ctronics.h and wires up "printer" as its real default —
-- so centronics_devices() got the same physical-removal treatment as
-- default_rs232_devices() (see ctronics.cpp), with a trimmed replacement
-- (CENTRONICS_PRINTER only) in src/mame/sgi/sgi_centronics_devices.cpp.
BUSES["GIO64"] = true
-- BUSES["HEATHZENITH_H19"] = true         -- unused: no SGI driver file
-- BUSES["HEATHZENITH_INTR_CNTRL"] = true  -- references anything HeathZenith.
-- BUSES["NSCSI"] = true  -- pulls in all 18 NSCSI peripheral types (36 files);
-- indy_indigo2.cpp's own scsi_devices() only ever instantiates NSCSI_CDROM_SGI
-- and NSCSI_HARDDISK, so those two device files are hand-listed below instead.
BUSES["PC_KBD"] = true
-- BUSES["RS232"] = true  -- pulls in ~19 historic serial peripheral types (42
-- files) via rs232.cpp's default_rs232_devices(). That function was removed
-- from rs232.cpp entirely (see the comment left there) and replaced by a
-- trimmed same-name/signature version in src/mame/sgi/sgi_rs232_devices.cpp —
-- physical removal, not a flag or #ifdef, because MAME's makedep.py dependency
-- scanner has no preprocessor awareness and follows #include text regardless
-- of which branch would actually compile.
BUSES["SUNKBD"] = true


--------------------------------------------------
-- This is the list of files that are necessary
-- for building all of the drivers referenced
-- in sgi.lst
--------------------------------------------------

function createProjects_mame_sgi(_target, _subtarget)
	project ("mame_sgi")
	targetsubdir(_target .."_" .. _subtarget)
	kind (LIBTYPE)
	uuid (os.uuid("drv-mame-sgi"))
	addprojectflags()
	precompiledheaders_novs()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/mame/shared",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "mame/layout",
	}

	-- Guards genuine hardware-emulation bug fixes to the upstream SGI driver
	-- (not bespoke-build trimming) so they're clearly identifiable and easy
	-- to disable/upstream separately. See hpc3.cpp/indy_indigo2.cpp for uses.
	-- Confirmed NOT required to avoid the "Start System" panic (eaddr alone
	-- is sufficient — see wiki/assets.md) but kept enabled anyway: it's
	-- real, datasheet-verified hardware-accurate behavior with no observed
	-- downside across a full successful boot-to-shutdown cycle.
	defines {
		"MAME_SGI_HW_FIXES",
	}

	-- NOTE: MAME_DRC_BUGFIXES is NOT defined here. Files under
	-- src/devices/cpu/ (like mips3drc.cpp) compile as part of the shared
	-- "optional" project (liboptional.a), not this "mame_sgi" project, so
	-- a `defines` block here never reaches them - confirmed the hard way
	-- (fix appeared to build clean but the exact pre-fix machine code was
	-- still present in the binary). The marker is self-defined directly in
	-- mips3drc.cpp instead. See wiki/assets.md's DRC section.

files{
--	MAME_DIR .. "src/mame/bally/astrocde.cpp",
--	MAME_DIR .. "src/mame/bally/astrocde.h",
--	MAME_DIR .. "src/mame/bally/astrocde_v.cpp",
--	MAME_DIR .. "src/mame/bally/gridlee.cpp",
--	MAME_DIR .. "src/mame/bally/gridlee.h",
--	MAME_DIR .. "src/mame/bally/gridlee_a.cpp",
--	MAME_DIR .. "src/mame/bally/gridlee_v.cpp",
--	MAME_DIR .. "src/mame/shared/s11c_bg.cpp",
--	MAME_DIR .. "src/mame/shared/s11c_bg.h",
--	MAME_DIR .. "src/mame/exidy/nl_carpolo.cpp",
--	MAME_DIR .. "src/mame/exidy/nl_carpolo.h",
--	MAME_DIR .. "src/mame/exidy/carpolo.cpp",
--	MAME_DIR .. "src/mame/exidy/carpolo.h",
--	MAME_DIR .. "src/mame/exidy/carpolo_m.cpp",
--	MAME_DIR .. "src/mame/exidy/carpolo_v.cpp",
--	MAME_DIR .. "src/mame/exidy/circus_a.cpp",
--	MAME_DIR .. "src/mame/exidy/circus.cpp",
--	MAME_DIR .. "src/mame/exidy/circus.h",
--	MAME_DIR .. "src/mame/exidy/circus_v.cpp",
--	MAME_DIR .. "src/mame/exidy/exidy.cpp",
--	MAME_DIR .. "src/mame/shared/exidysound.cpp",
--	MAME_DIR .. "src/mame/shared/exidysound.h",
--	MAME_DIR .. "src/mame/exidy/exidy440_a.cpp",
--	MAME_DIR .. "src/mame/exidy/exidy440_a.h",
--	MAME_DIR .. "src/mame/exidy/nl_fireone.h",
--	MAME_DIR .. "src/mame/exidy/nl_fireone.cpp",
--	MAME_DIR .. "src/mame/exidy/nl_starfire.h",
--	MAME_DIR .. "src/mame/exidy/nl_starfire.cpp",
--	MAME_DIR .. "src/mame/exidy/starfire.cpp",
--	MAME_DIR .. "src/mame/exidy/starfire.h",
--	MAME_DIR .. "src/mame/exidy/starfire_v.cpp",
--	MAME_DIR .. "src/mame/exidy/vertigo.cpp",
--	MAME_DIR .. "src/mame/exidy/vertigo.h",
--	MAME_DIR .. "src/mame/exidy/vertigo_m.cpp",
--	MAME_DIR .. "src/mame/exidy/vertigo_v.cpp",
--	MAME_DIR .. "src/mame/exidy/victory.cpp",
--	MAME_DIR .. "src/mame/exidy/victory.h",
--	MAME_DIR .. "src/mame/exidy/victory_v.cpp",
--	MAME_DIR .. "src/mame/gaelco/gaelco.cpp",
--	MAME_DIR .. "src/mame/gaelco/gaelco.h",
--	MAME_DIR .. "src/mame/gaelco/gaelco_v.cpp",
--	MAME_DIR .. "src/mame/gaelco/gaelcrpt.cpp",
--	MAME_DIR .. "src/mame/gaelco/wrally.cpp",
--	MAME_DIR .. "src/mame/gaelco/gaelco_wrally_sprites.cpp",
--	MAME_DIR .. "src/mame/gaelco/gaelco_wrally_sprites.h",
--	MAME_DIR .. "src/mame/gaelco/gaelco_ds5002fp.cpp",
--	MAME_DIR .. "src/mame/gaelco/gaelco_ds5002fp.h",
--	MAME_DIR .. "src/mame/misc/goldnpkr.cpp",
--	MAME_DIR .. "src/mame/videogames/looping.cpp",
--	MAME_DIR .. "src/mame/videogames/supertnk.cpp",
--	MAME_DIR .. "src/mame/shared/williamssound.cpp",
--	MAME_DIR .. "src/mame/shared/williamssound.h",
--	MAME_DIR .. "src/mame/williams/williams.cpp",
--	MAME_DIR .. "src/mame/williams/williams.h",
--	MAME_DIR .. "src/mame/williams/williams_m.cpp",
--	MAME_DIR .. "src/mame/williams/williams_v.cpp",
--	MAME_DIR .. "src/mame/williams/williamsblitter.cpp",
--	MAME_DIR .. "src/mame/williams/williamsblitter.h",
	MAME_DIR .. "src/devices/bus/centronics/ctronics.cpp",
	MAME_DIR .. "src/devices/bus/centronics/ctronics.h",
	MAME_DIR .. "src/devices/bus/centronics/printer.cpp",
	MAME_DIR .. "src/devices/bus/centronics/printer.h",
	MAME_DIR .. "src/mame/sgi/sgi_centronics_devices.cpp",
	MAME_DIR .. "src/devices/bus/nscsi/cd.cpp",
	MAME_DIR .. "src/devices/bus/nscsi/cd.h",
	MAME_DIR .. "src/devices/bus/nscsi/hd.cpp",
	MAME_DIR .. "src/devices/bus/nscsi/hd.h",
	MAME_DIR .. "src/devices/bus/rs232/rs232.cpp",
	MAME_DIR .. "src/devices/bus/rs232/rs232.h",
	MAME_DIR .. "src/devices/bus/rs232/loopback.cpp",
	MAME_DIR .. "src/devices/bus/rs232/loopback.h",
	MAME_DIR .. "src/devices/bus/rs232/null_modem.cpp",
	MAME_DIR .. "src/devices/bus/rs232/null_modem.h",
	MAME_DIR .. "src/devices/bus/rs232/pty.cpp",
	MAME_DIR .. "src/devices/bus/rs232/pty.h",
	MAME_DIR .. "src/devices/bus/rs232/terminal.cpp",
	MAME_DIR .. "src/devices/bus/rs232/terminal.h",
	MAME_DIR .. "src/mame/sgi/sgi_rs232_devices.cpp",
	MAME_DIR .. "src/mame/sgi/indy_indigo2.cpp",
	MAME_DIR .. "src/mame/sgi/hal2.cpp",
	MAME_DIR .. "src/mame/sgi/hal2.h",
	MAME_DIR .. "src/mame/sgi/hpc3.cpp",
	MAME_DIR .. "src/mame/sgi/hpc3.h",
	MAME_DIR .. "src/mame/sgi/nwd33c9x.cpp",
	MAME_DIR .. "src/mame/sgi/nwd33c9x.h",
	MAME_DIR .. "src/mame/sgi/ioc2.cpp",
	MAME_DIR .. "src/mame/sgi/ioc2.h",
	MAME_DIR .. "src/mame/sgi/mc.cpp",
	MAME_DIR .. "src/mame/sgi/mc.h",
	MAME_DIR .. "src/mame/sgi/vino.cpp",
	MAME_DIR .. "src/mame/sgi/vino.h",
}
end

function linkProjects_mame_sgi(_target, _subtarget)
	links {
		"mame_sgi",
	}
end
