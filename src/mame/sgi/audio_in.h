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

// Stereo host input (route a microphone or other source to inputs 0-1)
// converted to the guest's sample rate, one frame at a time. The stream
// runs at the host rate: changing a stream's rate while it has inputs isn't
// safe, so the conversion happens here.
class sgi_audio_input_device : public device_t, public device_sound_interface
{
public:
	sgi_audio_input_device(machine_config const &mconfig, char const *tag, device_t *owner, u32 clock = 0);

	// guest frames per second; 0 stops the conversion
	void set_rate(u32 rate);

	// linear gain before the ADC, per channel (default 1.0)
	void set_gain(unsigned channel, double gain);

	// the next frame at the guest rate; repeats the last one when the input
	// has nothing yet
	void read_frame(s16 frame[2]);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override;

private:
	sound_stream *m_stream = nullptr;
	sgi_audio_in::frame_fifo m_fifo;
	sgi_audio_in::input_converter m_conv;
	u32 m_rate = 0;
	s16 m_frame[2]{};
	// frames kept between the host input and the guest, and the level that
	// triggers a stream update: the input is brought up to date in batches
	// rather than for every frame the guest reads
	u32 m_lag = 0, m_low = 0;
	bool m_running = false;
	double m_gain[2]{ 1.0, 1.0 };
	// the test tone build's sine pair (audio_in.cpp); declared in every
	// build so the class layout doesn't depend on the option
	double m_tone_hz[2]{};
	double m_tone_phase[2]{};
};

DECLARE_DEVICE_TYPE(SGI_AUDIO_INPUT, sgi_audio_input_device)

#endif // MAME_SGI_AUDIO_IN_H
