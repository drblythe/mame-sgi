// license:BSD-3-Clause
// copyright-holders:blythe
/*
 * Host audio input for the SGI audio devices (audio_in.h).
 */
#include "emu.h"
#include "audio_in.h"

#include <algorithm>
#include <cmath>

namespace {

constexpr double PI = 3.14159265358979323846;

} // anonymous namespace

namespace sgi_audio_in {

void frame_fifo::push(int16_t l, int16_t r)
{
	if (count == FRAMES)
	{
		read = (read + 1) % FRAMES;
		count--;
	}
	const unsigned write = (read + count) % FRAMES;
	data[write][0] = l;
	data[write][1] = r;
	count++;
}

bool frame_fifo::pop(int16_t frame[2])
{
	if (count == 0)
		return false;
	frame[0] = data[read][0];
	frame[1] = data[read][1];
	read = (read + 1) % FRAMES;
	count--;
	return true;
}

void frame_fifo::trim(unsigned keep)
{
	while (count > keep)
	{
		read = (read + 1) % FRAMES;
		count--;
	}
}

// Kaiser-windowed sinc, cut off at 47% of
// the lower rate (about 90 dB stopband), each phase normalised to unity
// gain at DC
void input_converter::build(double new_step)
{
	constexpr double beta = 9.0;
	auto bessel_i0 = [] (double x)
	{
		double sum = 1.0, term = 1.0;
		for (int k = 1; k < 32; k++)
		{
			term *= (x / (2.0 * k)) * (x / (2.0 * k));
			sum += term;
		}
		return sum;
	};
	const double fc = 0.47 * std::min(1.0, new_step);   // cycles per input frame
	const double i0_beta = bessel_i0(beta);
	coeff.assign((PHASES + 1) * TAPS, 0.0f);
	for (int p = 0; p <= PHASES; p++)
	{
		const double frac = double(p) / PHASES;
		double sum = 0.0;
		double h[TAPS];
		for (int k = 0; k < TAPS; k++)
		{
			// tap k is input frame (floor(t) - TAPS/2 + 1 + k); its distance from t
			const double x = double(k - (TAPS / 2 - 1)) - frac;
			const double r = x / (TAPS / 2);
			const double w = (std::abs(r) < 1.0) ? bessel_i0(beta * std::sqrt(1.0 - r * r)) / i0_beta : 0.0;
			const double arg = 2.0 * fc * x;
			const double sinc = (arg == 0.0) ? 1.0 : std::sin(PI * arg) / (PI * arg);
			h[k] = 2.0 * fc * sinc * w;
			sum += h[k];
		}
		for (int k = 0; k < TAPS; k++)
			coeff[p * TAPS + k] = float(h[k] / sum);
	}
	step = new_step;
	count = 0;
	pos = -double(TAPS / 2);
}

void convert_input(const int16_t frame[2], double step, input_converter &conv, frame_fifo &fifo)
{
	if (step == 1.0)
	{
		if (conv.step != 0.0)
			conv.reset();
		fifo.push(frame[0], frame[1]);
		return;
	}
	if (step != conv.step)
		conv.build(step);

	constexpr int TAPS = input_converter::TAPS;
	if (conv.count == input_converter::HIST)
	{
		// keep the last TAPS + 4 frames, enough for any pending output
		const unsigned keep = TAPS + 4;
		for (int ch = 0; ch < 2; ch++)
			std::copy(conv.hist[ch] + conv.count - keep, conv.hist[ch] + conv.count, conv.hist[ch]);
		conv.count = keep;
	}
	conv.hist[0][conv.count] = frame[0];
	conv.hist[1][conv.count] = frame[1];
	conv.count++;
	conv.pos -= 1.0;

	// an output at time t (frames relative to the newest, which is 0) needs
	// frames floor(t) - TAPS/2 + 1 .. floor(t) + TAPS/2, all <= 0
	const double out_step = 1.0 / step;
	while (conv.pos < 1.0 - TAPS / 2)
	{
		const double base = std::floor(conv.pos);
		const double phase = (conv.pos - base) * input_converter::PHASES;
		const int p = std::min(int(phase), input_converter::PHASES - 1);
		const float a = float(phase - p);
		const int first = int(conv.count) - 1 + int(base) - TAPS / 2 + 1;
		if (first >= 0)
		{
			// blend the two nearest phases once for both channels, then
			// sum in 8 independent lanes so the compiler can vectorise it
			const float *const c0 = &conv.coeff[p * TAPS];
			const float *const c1 = c0 + TAPS;
			float c[TAPS];
			for (int k = 0; k < TAPS; k++)
				c[k] = c0[k] + a * (c1[k] - c0[k]);
			int16_t out[2];
			for (int ch = 0; ch < 2; ch++)
			{
				const float *const x = conv.hist[ch] + first;
				float acc[8] = {};
				for (int k = 0; k < TAPS; k += 8)
					for (int j = 0; j < 8; j++)
						acc[j] += x[k + j] * c[k + j];
				const float y = ((acc[0] + acc[1]) + (acc[2] + acc[3])) + ((acc[4] + acc[5]) + (acc[6] + acc[7]));
				out[ch] = int16_t(std::clamp<long>(std::lround(y), -32768, 32767));
			}
			fifo.push(out[0], out[1]);
		}
		conv.pos += out_step;
	}
}

} // namespace sgi_audio_in
