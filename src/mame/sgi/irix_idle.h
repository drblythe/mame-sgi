// license:BSD-3-Clause
// copyright-holders:blythe
/*
 * IRIX's idle loop on CPUs without WAIT (R4000, R4400): the kernel idles
 * in a busy loop, while (local_idle() && !idler()), which only an
 * interrupt ends. irix_find_local_idle() finds local_idle (it tests the
 * per-CPU p_nextthread and c_threads; the field offsets are left
 * unmatched) in the kernel's text, for the MIPS III core's idle PC, where
 * it gives up its timeslice while no interrupt is pending instead of
 * spinning at full emulated speed. IRIX relinks the kernel on each
 * system, so its address isn't fixed: scan for it once the kernel is
 * loaded.
 */
#ifndef MAME_SGI_IRIX_IDLE_H
#define MAME_SGI_IRIX_IDLE_H

#pragma once

// Search physical [start, end) of space; returns the address of
// local_idle, or ~0 if it isn't there (yet).
inline offs_t irix_find_local_idle(address_space &space, offs_t start, offs_t end)
{
	static constexpr u32 sig[10] = {
		0x8c01'0000, 0x1420'0006, 0x0000'1025, 0x8c02'0000, 0x1440'0003,
		0x0000'1025, 0x03e0'0008, 0x2402'0001, 0x03e0'0008, 0x0000'0000 }; // lw at,(zero); bnez; move v0,zero; lw v0,(zero); bnez; ...
	static constexpr u32 mask[10] = {
		0xffff'0000, 0xffff'ffff, 0xffff'ffff, 0xffff'0000, 0xffff'ffff,
		0xffff'ffff, 0xffff'ffff, 0xffff'ffff, 0xffff'ffff, 0xffff'ffff };

	for (offs_t a = start; a < end; a += 4)
	{
		if ((space.read_dword(a) & mask[0]) != sig[0])
			continue;
		unsigned n = 1;
		while (n < 10 && (space.read_dword(a + n * 4) & mask[n]) == sig[n])
			n++;
		if (n == 10)
			return a;
	}
	return ~offs_t(0);
}

#endif // MAME_SGI_IRIX_IDLE_H
