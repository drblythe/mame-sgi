# SGI Indy, Indigo2 and Challenge S: a fork of MAME 0.289

This is a fork of [MAME](https://github.com/mamedev/mame) 0.289 that makes
the SGI Indy and Challenge S (IP24) and the Indigo2 (IP22) run IRIX 6.5
well enough for regular use:
boot to the desktop, run the window system, OpenGL and IRIS GL programs,
networking, audio, SCSI disks, CD-ROMs and DAT tape, and save and restore
state. On a modern x86-64 Linux host an Indy boots IRIX to the login prompt
in about a minute.

It has been tested with many IRIS GL and OpenGL test and demo programs, and
many emulation bugs found that way are fixed: in the SGI drivers and in code
they share with the rest of MAME (the MIPS III recompiler, the x64 and C
recompiler back ends, the scheduler, SCSI, serial and keyboard controllers,
SDL input). There are also speedups and a few new devices and options. Each
change is a separate commit on top of the `mame0289` release tag.

The debugging, fixes and performance work were done with heavy assistance
from Anthropic's Claude (Claude Code, with the Claude Sonnet 5 and Claude
Opus 5.5 models); commits carry a `Co-Authored-By` line naming the model
that worked on them.

The tree builds two ways:

- **Full MAME** (every driver), as upstream. SGI-specific behaviour is
  switched on per device by the SGI drivers only, so other drivers keep
  upstream behaviour. `mame -validate` passes.
- **`mamesgi`**, a reduced build with only the SGI machines
  (`SUBTARGET=sgi`), much faster to compile.

## Machines

| Name | Machine |
|---|---|
| `indy_4610` | Indy, R4600 100 MHz |
| `indy_4613` | Indy, R4600 133 MHz |
| `indy_5015` | Indy, R5000 150 MHz |
| `challenge_s` | Challenge S, R5000SC 180 MHz (new; no graphics, serial console) |
| `indigo2_4415` | Indigo2, R4400 150 MHz, XL graphics (see [Indigo2](#indigo2)) |

The Indy with IRIX 6.5 is what is tested most.

With MAME's default throttling, each machine runs at its real speed: the
emulated CPU at its native clock. So on a fast host, a faster machine
(e.g. `indy_5015`) runs faster than a slower one, as on real hardware.
Throttling can be turned off (`-nothrottle`, or F10 with MAME's hotkeys
on) to run as fast as the host allows, but emulated time then runs ahead
of real time, so the real-time clock and IRIX's system time quickly drift.

## What you need

Not included, and not available from this project:

- **ROM sets.** For example `indy_4610.zip`, containing
  `ip24prom.070-9101-011.bin` and the keyboard controller ROM
  `72x8455.zm82`; for the Indigo2, `indigo2_4415.zip` with
  `ip22prom.070-1367-012.bin` and the same keyboard controller ROM. Put
  the zip in a `roms` directory below the directory you run MAME from, or
  point `-rompath` at it.
- **IRIX.** An IRIX 6.5 installation on a hard disk image (CHD), made from
  your own install media, or the prebuilt IRIX 6.5 CHD described in
  [this IRIXNet forum thread](https://forums.irixnet.org/thread-4081.html),
  which is what this fork was developed and tested with.
- **Demos (optional).** The standard SGI demos (buttonfly, powerflip,
  ideas, flight and others) are on the IRIX 6.5.8 Applications CD, which
  is available as an ISO and as a tarball at
  [jrra.zone/sgi](https://jrra.zone/sgi/).

[sgi.neocities.org](https://sgi.neocities.org/) is a general guide to
running the SGI machines in MAME, including how to make a CHD and install
IRIX on it.

## Building

The usual MAME build instructions apply (see the
[MAME documentation](https://docs.mamedev.org/initialsetup/compilingmame.html)).
This fork adds a reduced target for just the SGI machines:

```sh
make SUBTARGET=sgi -j$(nproc)          # mamesgi
make -j$(nproc)                        # full MAME, as upstream
```

The two builds compile some shared files differently, so to keep both in
one tree give the full build its own directory:
`make BUILDDIR=build-full -j$(nproc)`.

Built and tested with:

| Host | Toolchain | Tested on |
|---|---|---|
| Linux x86-64 | GCC 11 | Linux x86-64 (main development platform) |
| Linux x86-64, cross | [llvm-mingw](https://github.com/mstorsjo/llvm-mingw) 20260616 (LLVM 22), MAME's `windows_x64_clang` target | Windows 7 (SDL front end; OpenGL and bgfx renderers) |
| Linux x86-64, cross | Clang 22 with a Debian bookworm arm64 sysroot, `TARGETOS=linux PLATFORM=arm64` | Raspberry Pi 3, 64-bit Raspberry Pi OS |

LLVM 23 fails on a few upstream 0.289 files. MSVC builds are untested.

## Running

```sh
./mamesgi indy_4610 -hard1 irix.chd -window -mouse -nofilter -resolution 1280x1024
```

- `-hard1` is the boot disk (SCSI ID 1); `-hard2` to `-hard7` are further
  disks at those SCSI IDs.
- The Indy keyboard takes all keys. Scroll Lock toggles MAME's own hotkeys
  (Tab for the menu, F6/F7 to save/load state, Esc to quit). While they are
  on, the keys also reach the emulated keyboard.
- On a monitor the same size as the emulated screen, `-nowindow` (full
  screen) is more practical than a window.
- On Windows, MAME uses the bgfx renderer by default (Direct3D 11 where
  available; it needs the `bgfx` directory next to the binary, and
  without it MAME falls back to OpenGL). Both renderers upload only the
  changed parts of the screen: bgfx renders slightly faster, OpenGL uses
  a little less CPU. `-video bgfx` or `-video opengl` picks one.

Options and settings added by this fork:

| Option or setting | What it does |
|---|---|
| `-drc_cache_mb N` | MIPS recompiler code cache size (the SGI machines default to 128 MB) |
| Machine Configuration → RAM bank A/B | Each bank holds four SIMMs of 1, 2, 4, 8, 16 or 32 MB (4 to 128 MB per bank), so up to 256 MB; 128 MB by default. Change it before a cold boot |
| Machine Configuration → Ethernet Link Speed | 10 Mbit/s or Unlimited |
| `-sdl_input_buffer 0/1` | Buffer SDL key and mouse-button events so short presses during a host stall aren't lost (default 1) |
| `-mouse_capture_focus 0/1` | With `-mouse`, capture the pointer while the window has focus instead of while the pointer is over it (default 1). Hover capture grabs and releases on every enter/leave, and dragging windows in IRIX caused bursts of those that left keyboard and mouse input dead until the next Alt-Tab. Part of the SDL front end on every platform; the problem and the fix were seen on X11 |
| Machine Configuration → AES Digital Audio Connection | What is plugged into the AES/S/PDIF connector: None (default), Loopback (AES out to AES in), or Host audio, where AES out and in appear in the Audio Mixer to map to host devices such as an S/PDIF port (PCM audio; the mixer is not bit-transparent, so compressed pass-through formats don't survive) |

New devices: an HLE PS/2 keyboard (the Indy default); a flat SGI SCSI
controller with embedded disks (`nwd33c9x`), which reads the disk about 16
times faster in real time than MAME's stock SCSI emulation (50 MB/s against
3 MB/s reading the raw disk in IRIX, where the stock path also slows the
whole emulation down); and a DDS DAT tape drive on SIMH `.tap` images
(`-nwd33c9x:auxbus:7 tape -tape FILE.tap`).

Networking uses MAME's TAP interface: a `tap` device on Linux, the
TAP-Windows adapter on Windows. Pick it under Network Devices in the menu.

### Audio

- Use MAME's high-quality resampler (Audio Mixer in the menu, or
  `<resampler type="1" />` inside `<mixer>` in `cfg/default.cfg`). The
  default low-fidelity one is fine at 48 kHz but adds audible distortion
  to high frequencies when IRIX plays at 44.1 kHz or lower.
- Recording (microphone or line in) needs a sound module that can capture:
  on Linux that is `-sound portaudio`. The input comes from the host's
  default capture device, or the one mapped in the Audio Mixer.
- To monitor the input (apanel's monitor), give IRIX the same input and
  output sample rate: with different rates IRIX's monitor copies the
  samples without converting them, and the result is garbled.
- On the Indy, `-wavwrite` records four channels: the analog output (left,
  right), then the AES digital output (left, right).

### Challenge S console

The Challenge S has no graphics board or keyboard: its console is serial
port 1, MAME's `ioc2:rs232b` slot, at 9600 baud. Its default NVRAM already
selects that console, so it boots straight to it; what is needed is
something on the other end of the port. For example, a TCP socket to a
terminal program, started before MAME:

```sh
socat -,raw,echo=0 TCP-LISTEN:2323,reuseaddr      # in another terminal
./mamesgi challenge_s -hard1 irix.chd -ioc2:rs232b null_modem -bitb socket.127.0.0.1:2323
```

Once IRIX is up you can also log in over the network.

### Indigo2

The Indigo2 boots the same IRIX 6.5 disk as the Indy: IRIX rebuilds its
kernel for the Indigo2 on the first boot. It passes the same tests as the
Indy. Its PROM keeps its settings in a serial EEPROM, which starts out with
the PROM's defaults: the Ethernet address is `ff:ff:ff:ff:ff:ff`, which
IRIX's networking can't use. Set it, and a serial console if wanted, from
the PROM's Command Monitor (`setenv -f eaddr 08:00:69:xx:yy:zz`, exactly in
that form; `setenv console d1`), then leave MAME normally so the EEPROM is
saved. The machine has three RAM banks (Machine Configuration), 128 MB by
default, and a second SCSI bus.

### Save states

Save states work: F6 saves (pick a slot), F7 loads, and `-state SLOT` on
the command line starts from one. A state holds the machine, not the disk:
IRIX keeps writing to the CHD after the save, and loading the state over a
disk that has moved on corrupts the filesystem. Keep the disk with the
state: save, quit MAME, copy the CHD, and resume later from that copy
(`-hard1 copy.chd -state 1`). States may not load after updating the
fork.

With `-autosave`, MAME saves the machine when you quit and loads it again
the next time you start that machine without `-state`, so IRIX carries on
where it was. The saved state belongs to the machine, not the disk: it is
only safe if the disk isn't used in between (not booted without
`-autosave`, not swapped for another image or an older copy).

## Performance

Approximate gains, as measured in this project. Stock 0.289 can't be used
as the baseline for most rows: on the test disk image it crashes in the MIPS
recompiler during IRIX's first-boot kernel relink, and its serial console
drops characters, so "before" is usually the fork without that change, which
behaves like stock in that area.

| Area | Before | After | Before is |
|---|---|---|---|
| Booting IRIX 6.5 to the login prompt | crashes in the recompiler | about 1 minute (modern x86-64 Linux host) | stock 0.289 |
| Disk reads (raw, 197 MB) | 3.1 MB/s real time (7.8 MB/s guest time) | 50 MB/s | MAME's stock SCSI path |
| Ethernet, 10 Mbit link (default) | transmit unpaced, receive ~1.2-1.4 MB/s, lost frames and Ethernet memory corruption (boot panics) | ~1.2 MB/s each way, like real hardware | stock behaviour |
| Ethernet, Unlimited setting | not available | TCP ~8 MB/s out, ~10 MB/s in; UDP ~9.6 MB/s out with no loss | new |
| Recompiles of user code per boot | 321-351 k | 235 k; host instructions -20 %, cycles -18 % | stock MIPS3 recompiler |
| Recompiles avoided by redispatch (TLB-mismatch and missing-code handlers) | 0 % | ~50-56 % of arrivals | stock |
| Recompiler code generation cost | | boot host instructions -19 % (cache-line size read once, label map, RECOVER memo) | stock |
| Main memory access | | boot instructions -9.8 %, cycles -6 % (direct RAM path) | stock |
| FPU and exception paths | | boot cycles -18.5 %, branch misses -59 % | stock |
| Recompiler hash tables | 48.5 MB | 25.5 MB | stock |
| Host CPU at the idle desktop | 0.52 cores | ~0.07 cores | earlier fork, measured step by step (idle loop, HLE keyboard, keyboard controller, dirty-rectangle GL upload) |
| Audio | 8-bit playback precision, no recording, no digital audio | 16-bit playback, stereo recording, AES digital in/out | stock |

The percentages are each change on its own, measured with hardware counters
over a boot; they don't add up to a single figure.

## Status

- The SDL Windows build runs on Windows 7 with the Universal C Runtime;
  the native Windows front end needs Windows 10.
- On AArch64 the recompiler works but a Raspberry Pi 3 runs at about 40 %
  of real speed.
- Diagnostics used during development remain in the code as `MAME_SGI_*`
  macros, all off by default.

## Releases

Releases are tagged `mame0289-sgiNNN`, with built binaries for Linux
x86-64, Linux ARM64 and Windows x64 on the repository's Releases page.
What changed in each is in [CHANGES-SGI.md](CHANGES-SGI.md). Latest:
`mame0289-sgi001` (2026-10-24).

## Contact

Questions and bug reports: open an issue on this repository. Problems that
also happen in upstream MAME belong at [MAME Testers](https://mametesters.org/).

## License

Same as MAME: the project as a whole is GPL-2.0-or-later, and individual
files carry their own (mostly BSD-3-Clause) license. See [COPYING](COPYING)
and [docs/legal](docs/legal).

MAME is a registered trademark of Gregory Ember. This is an unofficial fork,
not endorsed by the MAME team.
