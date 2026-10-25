# Changes in this fork

Releases are tagged `mame0289-sgiNNN`: the MAME release the fork is based
on, then a counter. Newest first. Items collect under "Not yet released"
until the next tag. Each release on GitHub has the same notes and the
built binaries.

## Not yet released

### Fixed

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

### Changed

- **bgfx renderer** (the default on Windows): the screen texture is
  uploaded only when the screen changed, and then only the changed
  regions, as the OpenGL renderer already did. On the idle desktop host
  CPU fell by a third (0.091 to 0.060 cores, on Linux with bgfx's OpenGL
  back end); in a graphics benchmark bgfx now renders slightly faster than
  OpenGL, its uploads running on bgfx's own render thread.

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
  core (was 0.5). See "Performance" in README-SGI.md.
- **Less recompiling:** user-code recompiles per boot fell from ~330 k to
  235 k, and about half of the arrivals that used to force a recompile
  (TLB mismatches, missing code) are now redispatched to existing code;
  cheaper code generation, smaller hash tables (48.5 MB to 25.5 MB) and a
  larger, configurable code cache (`-drc_cache_mb`, 128 MB on the SGI
  machines) cut cache flushes.
- **No warning screen** on the Indy at start-up: the imperfect sound and
  graphics flags are gone, now that both work.
