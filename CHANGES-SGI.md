# Changes in this fork

Releases are tagged `mame0289-sgiNNN`: the MAME release the fork is based
on, then a counter. Newest first. Items collect under "Not yet released"
until the next tag. Each release on GitHub has the same notes and the
built binaries.

## Not yet released

### Added

- **Indigo2** (`indigo2_4415`, R4400 150 MHz, XL graphics) runs IRIX 6.5:
  it boots the Indy's disk to the desktop and passes the same tests as the
  Indy (graphics, disks, tape, network, audio, save states). 128 MB by
  default, up to 384 MB. No longer marked as not working, so MAME no
  longer shows its warning screen at start-up. See
  [README-SGI.md](README-SGI.md) for the ROM set and the NVRAM settings it
  needs on first boot.

- **IRIS Indigo** (`indigo_r4000` R4000 100 MHz, `indigo_r4400` R4400
  150 MHz, LG1 graphics) runs IRIX 6.5.22 to the desktop: both models now
  on the MIPS III recompiler (boot to login in about a minute instead of
  six), LG1 drawing for the X server, IRIS GL and OpenGL, Ethernet, the
  fast SCSI path, save states. Audio output and input through a new
  DSP56000/DSP56001 core running SGI's own microcode, with the HPC1's DSP
  interface and the audio board's clocks, input level and source.
  Descriptions now give the CPU and its speed.
- **DSP56000/DSP56001 core** (shared MAME code): the instruction set,
  address generation, loops, interrupts and the SSI and SCI ports. The
  implementation includes some performance analysis and tuning.
- **PulseAudio recording** (`-sound pulse`): the PulseAudio module now
  captures from sources and monitors as well as playing, so recording
  (audio input on the Indy, Indigo2 and Indigo) works with it. It costs
  about 2% of a host core, where PortAudio through ALSA's pulse plugin
  polls constantly.
- **HLE SGI serial keyboard**, the Indigo default: the Key Tronic
  keyboard's protocol, key codes, auto-repeat, beeps and LEDs without
  running its 8031 firmware, which cost about a fifth of an idle Indigo's
  host CPU. `-keyboard_port keyboard_8031` selects the firmware keyboard
  (it needs `sgi_kbd.zip`); saved keyboard settings still apply.
- **Recompiler perf map** (shared MAME code): with `MAME_SGI_PERF_MAP` in
  the environment, the MIPS III recompiler writes `/tmp/perf-<pid>.map`
  naming its static handlers, so Linux `perf` can attribute host time to
  them.

### Fixed

- **MIPS III address errors** (shared MAME code, recompiler and
  interpreter): a misaligned halfword, word or doubleword load or store,
  or a jump to an unaligned address, now raises an address error, so IRIX
  kills the program with SIGBUS as on the hardware. Before, the access
  silently used the aligned word below, and a jump to 0xfffffffd-0xffffffff
  hung the recompiler at full host CPU. LWL/LWR and the other unaligned
  load and store instructions are unaffected; user code runs no extra
  host instructions for the check.
- **Sound input** (shared MAME code): a host input with fewer channels
  than the emulated one (a mono microphone feeding a stereo input) was
  read with the wrong stride and arrived as noise.
- **Virtual TLB** (shared MAME code): unmapping a TLB entry counted it as a
  mapping of virtual page 0, so a real mapping of page 0 survived being
  unmapped. IRIX never maps page 0; SGI's field diagnostics (IDE) do, and
  their TLB pid and UTLB-miss tests now pass.
- **HPC3 FIFO memories** read back what was written: an entry's upper and
  lower words no longer overwrite each other, and PBUS FIFO writes use
  the chip's 16-byte stride. SGI's diagnostics test these memories.
- **Newport colour map (CMAP)**: the palette address wraps after its 8K
  entries, as on the hardware. SGI's diagnostics rely on it when they read
  the palette back.
- **Newport VC2 timing tables**: a guest that writes test patterns into
  the VC2's table pointers (SGI's diagnostics do) no longer crashes MAME.
  Decoding stays within the VC2's RAM and the decoded table, and a timing
  table that describes no picture leaves the screen as it was.
- **Newport RAMDAC (BT445)**: its address register and gamma palette read
  back what was written, and palette writes no longer swap red and blue
  (visible only with a different gamma per channel).
- **Newport frame buffer reads** below the last row read the row the
  hardware wraps to, and reads outside the frame buffer no longer read past
  MAME's copy of it. SGI's diagnostics read just below a full-screen fill.
- **Newport colour index drawing** uses the colour set in COLORRED even
  when it was written while the drawing mode was RGB, as the hardware does.
- **MIPS R4000/R4400 floating point unit** (shared MAME code) reports the
  R4000-family implementation number, so IRIX names it correctly ("MIPS
  R4000 Floating Point Coprocessor", not "R6010").
- **Indigo2 vertical retrace interrupt** reaches IRIX where its graphics
  driver expects it, so OpenGL programs no longer hang waiting for a buffer
  swap and save states resume.
- **DSP56000 disassembler** (shared MAME code): MOVEP shows its peripheral
  address and Tcc its register pair correctly.
- **Newport picture and cursor framing** follow the video timing IRIX
  loads: on the Indigo2 the picture was off by six columns, with extra
  border, and the hardware cursor six columns left of the pointer.
- **MIPS III TLB and exceptions** (shared MAME code, recompiler and
  interpreter), found by booting NetBSD 11.0, which now reaches its
  installer on the Indy: the faulting address in BadVAddr (and EntryHi) is
  sign-extended for 32-bit addresses, as on the hardware, so a kernel that
  reads it with a 64-bit move finds kernel addresses in its kernel page
  table; TLB probes and mappings match on the 32-bit address, so entries
  written with sign-extended EntryHi values are found, invalidated and
  used; and the interpreter's TLTIU, TGEIU, TEQI and TNEI compare with the
  sign-extended immediate. IRIX was unaffected.
- **LG1 drawing** (Indigo): the X server's stipple fills, copy planes and
  opaque patterns (the desktop's icon shading), stippled lines, IRIS GL's
  one-step line segments and single-pixel reads (rounded on the minor
  axis), and antialiased colour-index points after blended or z-buffered
  lines.
- **Newport lines:** an X rectangle outline of zero width or height drew a
  second column, and integer lines now take the per-octant tie rounding
  the X server sets.
- **Newport pixel DMA:** the X server's pixmap-to-window copies from
  unaligned addresses came out shifted, with stray pixels.
- **Newport antialiased lines** under IRIS GL's z-buffer lost pixels where
  lines nearly coincide: each line's depth test result was applied to the
  wrong one of its two pixels in half the directions.
- **Newport 8-bit board (XL8):** IRIS GL's overlay drawing (in the pop-up
  planes) read back as empty.

### Changed

- **bgfx renderer** (the default on Windows): the screen texture is
  uploaded only when the screen changed, and then only the changed
  regions, as the OpenGL renderer already did. On the idle desktop host
  CPU fell by a third (0.091 to 0.060 cores, on Linux with bgfx's OpenGL
  back end); in a graphics benchmark bgfx now renders slightly faster than
  OpenGL, its uploads running on bgfx's own render thread.
- **Idle Indigo** host CPU 0.44 to 0.34 cores: the HLE keyboard above; a
  screen whose driver doesn't report changed regions (the LG1) is no longer
  uploaded to the OpenGL renderer again for frames that didn't change it
  (shared MAME code; it was 16 full uploads a second); and a cheaper read
  of the HPC1 counter SGI's audio microcode polls.
- **Indigo RAM:** 128 MB by default (bank A, four 32 MB SIMMs) instead of
  MAME's 16 MB, in which IRIX 6.5 pages heavily; up to 384 MB in Machine
  Configuration. A RAM setting saved for bank A under the old default is
  not applied, so check Machine Configuration after updating.

## mame0289-sgi001 (2026-10-24)

The first release: everything this fork adds to MAME 0.289.

### Added

- **Challenge S** (`challenge_s`, R5000SC 180 MHz): boots IRIX 6.5 to its
  serial console.
- **Audio input:** stereo recording from the host microphone or line in
  (codec B), with IRIX's input gain, source and mute, and a band-limited
  conversion from the host's sample rate to IRIX's.
- **AES digital audio** (the S/PDIF connector): the CS8411 receiver and
  CS8401A transmitter, with a machine setting for what is plugged in
  (None, Loopback, or Host audio for the host's own digital ports).
- **Save states** on the Indy models and the Challenge S, including
  `-autosave`.
- **Flat SGI SCSI controller** with embedded disks (`nwd33c9x`): about 16
  times faster disk reads in real time than MAME's stock SCSI path.
- **DDS DAT tape drive** on SIMH `.tap` images.
- **HLE PS/2 keyboard** (the Indy default).
- **Ethernet link speed setting:** 10 Mbit/s, paced like real hardware
  (default), or Unlimited.
- **Options:** `-drc_cache_mb`, `-sdl_input_buffer`,
  `-mouse_capture_focus`; RAM banks up to 256 MB in Machine Configuration.
- **`mamesgi`**, a reduced build with only the SGI machines
  (`make SUBTARGET=sgi`).

### Fixed

- **MIPS III recompiler:** stock 0.289 crashes during IRIX's first-boot
  kernel relink. Many correctness fixes (exceptions, FPU, TLB, delay
  slots, likely branches, interrupt checks) in the recompiler and its x64
  and C back ends. The code cache ignored the address space (ASID), so
  code compiled for one process could run in another; it now keeps them
  apart for writable pages, while shared code is still compiled once.
  IRIX, the desktop, the compilers and the test programs run with no
  known recompiler faults.
- **MIPS III interpreter (`-nodrc`):** the guest's FP rounding mode, NaN in
  FP compares, and FP-to-integer conversions (rounding and invalid
  results), now as on real hardware.
- **Save states:** complete and correct. Main RAM is in the state, and
  the HAL2 audio clocks and HPC3 DMA rates are recomputed after a load, so
  a session resumes exactly where it was saved.
- **Display:** the visible area is the picture itself at 1280x1024 and
  1024x768 (it was the whole video readout: a stripe of stale memory at
  the right edge, the last column cut off, and the window scaled to fit),
  and the low-resolution display IRIX uses with default NVRAM is no
  longer corrupted (the compositor ignored the scroll register).
- **Graphics (Newport/REX3):** line drawing (antialiased and subpixel
  lines), plane write masks, host-data reads, colour-map reads, the cursor
  and display modes, checked pixel by pixel against IRIX's OpenGL and
  IRIS GL output.
- **Audio:** 16-bit playback (it was 8-bit precise), the PROM chime,
  sample rates, gains and attenuation.
- **Ethernet:** lost frames and memory corruption (boot panics) under
  load.
- **Serial console:** dropped and garbled characters.
- **Shutdown:** IRIX no longer hangs MAME at `init 0` after a long
  session (a timer catch-up truncated at 2^32 cycles, in the shared PIT
  emulation).

#### In shared MAME code (not SGI-specific; on for every driver)

- **Recompiler x64 back end:** the host's floating-point rounding mode
  (MXCSR) wasn't resynced on entry to generated code, so guest FP results
  could use the wrong rounding (for every recompiled CPU on x64).
- **PIT (8253/8254):** a counter idle for 2^32 or more input cycles never
  caught up (a livelock); mode 4/5 counters waiting for a count re-armed
  their timer endlessly.
- **Z80 SCC serial controller:** a chip reset discarded a character
  that was still being sent.
- **gdb stub:** a debugger could connect only once per MAME session; it
  can now detach and reconnect.
- **SCSI hard disk:** SYNCHRONIZE CACHE did nothing, so writes weren't
  flushed to the CHD.
- **SCSI tape:** SPACE never moved backwards, MODE SELECT/SENSE and the
  tape-change unit attention were missing, and SIMH `.tap` writes left
  older data readable after them.
- **DS1386 RTC:** the day-of-month register was set from the seconds
  value, and its time registers weren't refreshed from RAM after a state
  load.
- **HLE PS/2 mouse:** fast movements lost distance when a report
  overflowed.
- **Lua:** `emu.thread()` used its Lua state from two threads at once
  (console errors, occasional crashes).
- **PortAudio:** crash at start-up when no audio device can be
  enumerated.
- **SDL input:** key and mouse presses shorter than a host stall were
  lost (`-sdl_input_buffer`, on by default); with `-mouse`, capturing on
  hover could leave input dead after dragging windows
  (`-mouse_capture_focus`).

### Changed

- **Faster:** IRIX boots to the login prompt in about a minute on a
  modern x86-64 Linux host; the idle desktop uses about 0.07 of a host
  core (was 0.5). See "Performance" in [README-SGI.md](README-SGI.md).
- **Less recompiling:** user-code recompiles per boot fell from ~330 k to
  235 k, and about half of the arrivals that used to force a recompile
  (TLB mismatches, missing code) are now redispatched to existing code;
  cheaper code generation, smaller hash tables (48.5 MB to 25.5 MB) and a
  larger, configurable code cache (`-drc_cache_mb`, 128 MB on the SGI
  machines) cut cache flushes.
- **No warning screen** on the Indy at start-up: the imperfect sound and
  graphics flags are gone, now that both work.
