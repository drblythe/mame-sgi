// license:BSD-3-Clause
// copyright-holders:blythe
/*
 * Host audio input for the SGI audio devices: a stereo frame FIFO and a
 * converter from the host stream's rate to a guest sample rate.
 */
#ifndef MAME_SGI_AUDIO_IN_H
#define MAME_SGI_AUDIO_IN_H

#pragma once

#include <vector>

namespace sgi_audio_in {

// stereo 16-bit frames waiting for the guest; when full, a push drops the
// oldest frame. 2048 frames is about 43 ms at 48 kHz.
struct frame_fifo
{
	static constexpr unsigned FRAMES = 2048;
	int16_t data[FRAMES][2]{};
	unsigned read = 0;
	unsigned count = 0;

	void clear() { read = count = 0; }
	void push(int16_t l, int16_t r);
	bool pop(int16_t frame[2]);
	void trim(unsigned keep);
};

// host input at the stream's rate converted to a guest sample rate with
// a windowed-sinc filter (TAPS taps, coefficients for PHASES fractional
// positions, interpolated between them). Equal rates pass through.
// Output frames lag the input by TAPS / 2 input frames.
struct input_converter
{
	static constexpr int TAPS = 64;
	static constexpr int PHASES = 256;
	static constexpr unsigned HIST = 4096;   // frames of history before compacting
	double step = 0.0;          // output rate / input rate the filter was built for
	std::vector<float> coeff;   // (PHASES + 1) rows of TAPS
	float hist[2][HIST]{};
	unsigned count = 0;         // frames in hist
	double pos = 0.0;           // next output time, in input frames from the newest one

	void reset() { step = 0.0; count = 0; pos = 0.0; }
	void build(double new_step);
};

// one input frame at the stream's rate; pushes the output frames (at step
// times the input rate) that it completes
void convert_input(const int16_t frame[2], double step, input_converter &conv, frame_fifo &fifo);

} // namespace sgi_audio_in

#endif // MAME_SGI_AUDIO_IN_H
