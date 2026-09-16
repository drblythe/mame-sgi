-- license:BSD-3-Clause
-- copyright-holders:MAMEdev Team

---------------------------------------------------------------------------
--
--   devices.lua
--
--   Rules for building device cores
--
---------------------------------------------------------------------------

function devicesProject(_target, _subtarget)

	disasm_files = { }
	disasm_dependency = { }
	disasm_custombuildtask = { }

	project ("optional")
	uuid (os.uuid("optional-" .. _target .."_" .. _subtarget))
	kind (LIBTYPE)
	targetsubdir(_target .."_" .. _subtarget)

	if (_OPTIONS["targetos"] ~= "asmjs") then
		options {
			"ArchiveSplit",
		}
	end

	addprojectflags()
	precompiledheaders()
	flags {
		-- Symbols (-g) enabled deliberately for host-gdb debugging of
		-- src/devices/ code (this project builds as "liboptional.a" -
		-- mips3drc.cpp/mips3.cpp among others - which otherwise has zero
		-- DWARF info, unlike src/mame.lua's main executable project).
		-- NoPCH is required alongside it: emu.h's precompiled header is
		-- built once, shared across every project in the whole build
		-- (build/linux_gcc/obj/<arch>/<config>/emu.h.gch, not scoped per
		-- project), by a separate "precompile" project whose own flags
		-- don't include -g - GCC correctly rejects the flag-mismatched
		-- PCH as invalid, and its fallback -include path points at the
		-- build-output directory (not the real source location), so it
		-- can't recover by finding a plain emu.h there either, producing
		-- a hard "No such file or directory" rather than a graceful
		-- on-the-fly recompile. Confirmed by direct comparison of this
		-- project's and precompile's generated release64 ALL_CXXFLAGS.
		"Symbols",
		"NoPCH",
	}

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		MAME_DIR .. "3rdparty/asmjit", -- required by asmjit internal use of #include <...> syntax
		GEN_DIR  .. "emu",
		GEN_DIR  .. "emu/layout",
		ext_includedir("asio"),
		ext_includedir("expat"),
		ext_includedir("flac"),
	}

	dofile(path.join("src", "cpu.lua"))

	dofile(path.join("src", "sound.lua"))

	dofile(path.join("src", "video.lua"))

	dofile(path.join("src", "machine.lua"))

	dofile(path.join("src", "bus.lua"))

	pchsource(MAME_DIR .. "src/devices/machine/timer.cpp")

if #disasm_files > 0 then
	project ("dasm")
	uuid ("f2d28b0a-6da5-4f78-b629-d834aa00429d")
	kind (LIBTYPE)
	targetsubdir(_target .."_" .. _subtarget)
	addprojectflags()
	precompiledheaders()

	includedirs {
		MAME_DIR .. "src/osd",
		MAME_DIR .. "src/emu",
		MAME_DIR .. "src/devices",
		MAME_DIR .. "src/lib",
		MAME_DIR .. "src/lib/util",
		MAME_DIR .. "3rdparty",
		GEN_DIR  .. "emu",
		ext_includedir("asio"),
		ext_includedir("expat"),
	}

	files {
		disasm_files
	}

	for key,value in pairs(disasm_files) do
		if string.endswith(value, ".cpp") then
			--print("calling pchsource with " .. value)
			pchsource(value)
			break
		end
	end

	if #disasm_dependency > 0 then
		dependency(disasm_dependency)
	end

	if #disasm_custombuildtask > 0 then
		custombuildtask(disasm_custombuildtask)
	end
end

end
