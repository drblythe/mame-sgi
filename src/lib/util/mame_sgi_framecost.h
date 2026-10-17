// license:BSD-3-Clause
// copyright-holders:blythe
#ifndef MAME_LIB_UTIL_MAME_SGI_FRAMECOST_H
#define MAME_LIB_UTIL_MAME_SGI_FRAMECOST_H

#pragma once

// MAME_SGI_FRAME_LOAD attribution: wall time (TSC ticks) spent in a few
// subsystems, read per frame by video_manager to split each frame's busy
// time. Counted only while mame_sgi_frame_cost_on is set (it is when
// MAME_SGI_FRAME_LOAD_FILE is). The variables are defined in chd.cpp, which
// both the emulator and the CHD code link.

#include <cstdint>

#if defined(__x86_64__) || defined(_M_X64)
#include <x86intrin.h>

enum mame_sgi_frame_cost_category
{
	MSFC_COMPILE,       // DRC block compilation
	MSFC_FLUSH,         // DRC cache flushes
	MSFC_REX3,          // Newport REX3 drawing commands
	MSFC_COMPOSITE,     // Newport screen update (the compositor)
	MSFC_PRESENT,       // OSD update: render, texture upload, swap
	MSFC_CHD,           // CHD hunk reads (disk and CD-ROM)
	MSFC_CHD_WRITE,     // CHD hunk writes (allocation and map update included)
	MSFC_CPU,           // the main CPU's execute loop, inclusive of the
	                    // compile, flush and REX3 time spent inside it
	MSFC_THROTTLE,      // the throttle wait at each frame and the frame load
	                    // file writes (not busy time; counted so that time
	                    // nested in a timer callback can leave it out)
	MSFC_COUNT
};

extern bool mame_sgi_frame_cost_on;
extern uint64_t mame_sgi_frame_cost[MSFC_COUNT];

// adds the time from construction to destruction to a category
class mame_sgi_frame_cost_scope
{
public:
	explicit mame_sgi_frame_cost_scope(int category)
		: m_category(category), m_start(mame_sgi_frame_cost_on ? __rdtsc() : 0)
	{
	}
	~mame_sgi_frame_cost_scope()
	{
		if (m_start)
			mame_sgi_frame_cost[m_category] += __rdtsc() - m_start;
	}

private:
	int m_category;
	uint64_t m_start;
};
#define MAME_SGI_FRAME_COST(category) mame_sgi_frame_cost_scope mame_sgi_frame_cost_scope_(category)

// the time already attributed above that a timer callback or another
// executing device can contain (REX3, compile and flush are inside the CPU)
inline uint64_t mame_sgi_frame_cost_nested()
{
	return mame_sgi_frame_cost[MSFC_CPU] + mame_sgi_frame_cost[MSFC_COMPOSITE] + mame_sgi_frame_cost[MSFC_PRESENT]
			+ mame_sgi_frame_cost[MSFC_CHD] + mame_sgi_frame_cost[MSFC_CHD_WRITE] + mame_sgi_frame_cost[MSFC_THROTTLE];
}
#else
#define MAME_SGI_FRAME_COST(category) do { } while (0)
#endif

#endif // MAME_LIB_UTIL_MAME_SGI_FRAMECOST_H
