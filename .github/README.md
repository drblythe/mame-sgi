# SGI Indy and Challenge S: a fork of MAME 0.289

A fork of [MAME](https://github.com/mamedev/mame) 0.289 that makes the SGI
Indy and Challenge S (IP24) run IRIX 6.5 well enough for regular use: boot
to the desktop, the window system, OpenGL and IRIS GL programs,
networking, audio, SCSI disks, CD-ROMs and DAT tape, and save states. On a
modern x86-64 Linux host an Indy boots IRIX to the login prompt in about a
minute.

- **[README-SGI.md](../README-SGI.md)**: what you need, building, running,
  the options this fork adds, performance and status.
- **[CHANGES-SGI.md](../CHANGES-SGI.md)**: what the fork adds and fixes,
  including the fixes in shared MAME code.
- **[Releases](https://github.com/drblythe/mame-sgi/releases)**: tagged
  versions (`mame0289-sgiNNN`) with binaries for Linux x86-64, Linux ARM64
  and Windows x64.
- **Issues:** bug reports here; problems that also happen in official MAME
  belong at [MAME Testers](https://mametesters.org/).

This is an unofficial fork, not endorsed by the MAME team. Everything
else in the tree is upstream MAME 0.289: see [README.md](../README.md) for
MAME itself. License: as MAME (GPL-2.0-or-later as a whole; see
[COPYING](../COPYING)).
