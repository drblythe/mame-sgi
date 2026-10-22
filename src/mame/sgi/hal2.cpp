// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

    SGI HAL2 Audio Controller emulation

**********************************************************************/

#include "emu.h"

// MAME_SGI_HAL2_INPUT_TEST_TONE: feed codec B (the ADC) a 1 kHz sine on the
// left and 500 Hz on the right instead of the microphone, to check the
// recording path end to end from IRIX. Off by default.
// #define MAME_SGI_HAL2_INPUT_TEST_TONE

#include "hal2.h"

#include <cmath>

#define LOG_UNKNOWN     (1U << 1)
#define LOG_READS       (1U << 2)
#define LOG_WRITES      (1U << 3)
#define LOG_ALL         (LOG_UNKNOWN | LOG_READS | LOG_WRITES)

#define VERBOSE         (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SGI_HAL2, hal2_device, "hal2", "SGI HAL2")

hal2_device::hal2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SGI_HAL2, tag, owner, clock)
	, device_sound_interface(mconfig, *this)
	, m_ldac(*this, "ldac")
	, m_rdac(*this, "rdac")
	, m_mic(*this, "mic")
{
}

void hal2_device::device_start()
{
	save_item(NAME(m_isr));
	save_item(NAME(m_iar));
	save_item(NAME(m_idr));
	save_item(NAME(m_codeca_ctrl));
	save_item(NAME(m_codeca_channel));
	save_item(NAME(m_codeca_clock));
	save_item(NAME(m_codeca_channel_count));
	save_item(NAME(m_codecb_ctrl));
	save_item(NAME(m_codecb_channel));
	save_item(NAME(m_codecb_clock));
	save_item(NAME(m_codecb_channel_count));
	save_item(NAME(m_bres_clock_sel));
	save_item(NAME(m_bres_clock_inc));
	save_item(NAME(m_bres_clock_modctrl));
	save_item(NAME(m_bres_clock_freq));
	save_item(NAME(m_curr_dac));
	save_item(NAME(m_curr_adc));
	save_item(NAME(m_codeca_ctrl2_hi));
	save_item(NAME(m_codecb_ctrl2_hi));
	save_item(NAME(m_volume));
	save_item(NAME(m_adc_frame));

	// MAME_SGI_HAL2_AUDIO_INPUT: two inputs (microphone left/right) at the
	// microphone's own rate, pulled by dma_read(). The rate never changes:
	// sound_stream::set_sample_rate() on a stream with inputs left MAME's
	// input resampler reading past its buffer (a crash when IRIX switched
	// codec B to 44.1 kHz), so sound_stream_update() converts to codec B's
	// rate itself.
	m_adc_stream = stream_alloc(2, 0, SAMPLE_RATE_INPUT_ADAPTIVE);

	// MAME_SGI_HAL2_BRES_POSTLOAD_FIX (permanent, no #ifdef - companion to
	// MAME_SGI_PBUS_DMA_DEGENERATE_RATE_REARM_FIX in hpc3.cpp, same task
	// #67 HPC3 PBUS DMA livelock investigation): m_bres_clock_rate[] is
	// derived from m_bres_clock_sel/_inc/_modctrl (all save_item()'d
	// above) via update_clock_freq(), but is itself not saved and is only
	// ever recomputed when the guest live-writes one of those source
	// registers. A -state load restores the raw fields correctly but
	// leaves this derived array at its default (attotime::zero, same as
	// device_reset()) until/unless the guest happens to rewrite the
	// Bresenham clock config again - which it has no reason to do if it
	// already configured this before the state was saved. get_rate()
	// reads m_bres_clock_rate[] directly, so any PBUS DMA start attempted
	// after a resume silently never arms (hpc3.cpp's pbusdma_w() only
	// starts the timer if get_rate() != attotime::zero) - confirmed live:
	// identical guest-triggered DMA start sequence works on a continuous
	// boot and hangs forever after a state reload, with this array being
	// the one meaningful difference.
	machine().save().register_postload(save_prepost_delegate(FUNC(hal2_device::postload), this));
}

void hal2_device::postload()
{
	for (int clock_gen = 0; clock_gen < 3; clock_gen++)
		update_clock_freq(clock_gen);
	m_adc_fifo_read = 0;
	m_adc_fifo_count = 0;
	update_output_gain();
	update_input_gain();
}

void hal2_device::device_reset()
{
	m_isr = 0;
	m_iar = 0;
	memset(m_idr, 0, sizeof(uint16_t) * 4);
	memset(m_codeca_ctrl, 0, sizeof(uint16_t) * 2);
	m_codeca_ctrl2_hi = 0;
	m_codeca_channel = 0;
	m_codeca_clock = 0;
	m_codeca_channel_count = 0;
	memset(m_codecb_ctrl, 0, sizeof(uint16_t) * 2);
	m_codecb_ctrl2_hi = 0;
	m_codecb_channel = 0;
	m_codecb_clock = 0;
	m_codecb_channel_count = 0;
	memset(m_bres_clock_sel, 0, sizeof(uint16_t) * 3);
	memset(m_bres_clock_inc, 0, sizeof(uint16_t) * 3);
	memset(m_bres_clock_modctrl, 0, sizeof(uint16_t) * 3);
	memset(m_bres_clock_freq, 0, sizeof(uint16_t) * 3);
	for (int i = 0; i < 3; i++)
	{
		m_bres_clock_rate[i] = attotime::zero;
	}
	m_curr_dac = 0;
	m_curr_adc = 0;
	m_adc_frame[0] = m_adc_frame[1] = 0;
	m_adc_fifo_read = 0;
	m_adc_fifo_count = 0;
	update_dac_rate();
	update_adc_rate();
	update_output_gain();
	update_input_gain();
}

uint16_t hal2_device::read(offs_t offset)
{
	switch (offset)
	{
	case STATUS_REG:
		LOGMASKED(LOG_READS, "%s: HAL2 Status Read: %08x\n", machine().describe_context(), m_isr);
		return m_isr;
	case REVISION_REG:
		LOGMASKED(LOG_READS, "%s: HAL2 Revision Read: 0x4010\n", machine().describe_context());
		return m_absent ? 0xffff : 0x4010;

	case INDIRECT_DATA0_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 0 Read: %04x\n", machine().describe_context(), m_idr[0]);
		return m_idr[0];

	case INDIRECT_DATA1_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 1 Read: %04x\n", machine().describe_context(), m_idr[1]);
		return m_idr[1];

	case INDIRECT_DATA2_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 2 Read: %04x\n", machine().describe_context(), m_idr[2]);
		return m_idr[2];

	case INDIRECT_DATA3_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 3 Read: %04x\n", machine().describe_context(), m_idr[3]);
		return m_idr[3];
	}
	LOGMASKED(LOG_READS | LOG_UNKNOWN, "%s: Unknown HAL2 read: %08x\n", machine().describe_context(), 0x1fbd8000 + offset*4);
	return 0;
}

void hal2_device::write(offs_t offset, uint16_t data)
{
	switch (offset)
	{
	case STATUS_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Status Write: 0x%04x\n", machine().describe_context(), data);
		LOGMASKED(LOG_WRITES, "    HAL2 Global Reset %s\n", (data & ISR_GLOBAL_RESET) ? "Inactive" : "Active");
		LOGMASKED(LOG_WRITES, "    HAL2 Codec Reset %s\n", (data & ISR_CODEC_RESET) ? "Inactive" : "Active");
		m_isr &= ~0x1c;
		m_isr |= data & 0x1c;
		break;
	case INDIRECT_ADDRESS_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Address Register Write: %04x\n", machine().describe_context(), data);
		m_iar = data;
		switch (data & IAR_TYPE)
		{
		case 0x1000:
			LOGMASKED(LOG_WRITES, "    DMA Port\n");
			switch (data & IAR_NUM)
			{
			case 0x0100:
				LOGMASKED(LOG_WRITES, "        Synth In\n");
				break;
			case 0x0200:
				LOGMASKED(LOG_WRITES, "        AES In\n");
				break;
			case 0x0300:
				LOGMASKED(LOG_WRITES, "        AES Out\n");
				break;
			case 0x0400:
			{
				LOGMASKED(LOG_WRITES, "        Codec A (DAC) Out\n");
				const uint32_t param = (data & IAR_PARAM) >> IAR_PARAM_SHIFT;
				switch (param)
				{
					case 1:
						LOGMASKED(LOG_WRITES, "            Control 1\n");
						if (data & IAR_ACCESS_SEL)
						{
							m_idr[0] = m_codeca_ctrl[0];
						}
						else
						{
							m_codeca_ctrl[0] = m_idr[0];
							m_codeca_channel = m_idr[0] & 3;
							m_codeca_clock = (m_idr[0] >> 3) & 3;
							m_codeca_channel_count = (m_idr[0] >> 8) & 3;
							update_dac_rate();
						}
						break;
					case 2:
						LOGMASKED(LOG_WRITES, "            Control 2\n");
						if (data & IAR_ACCESS_SEL)
						{
							m_idr[0] = m_codeca_ctrl[1];
							m_idr[1] = m_codeca_ctrl2_hi;
						}
						else
						{
							m_codeca_ctrl[1] = m_idr[0];
							m_codeca_ctrl2_hi = m_idr[1];
							update_output_gain();
						}
						break;
					default:
						LOGMASKED(LOG_WRITES, "            Unknown Register\n");
						break;
				}
				break;
			}
			case 0x0500:
			{
				LOGMASKED(LOG_WRITES, "        Codec B (ADC) Out\n");
				const uint32_t param = (data & IAR_PARAM) >> IAR_PARAM_SHIFT;
				switch (param)
				{
					case 1:
						LOGMASKED(LOG_WRITES, "            Control 1\n");
						if (data & IAR_ACCESS_SEL)
						{
							m_idr[0] = m_codecb_ctrl[0];
						}
						else
						{
							m_codecb_ctrl[0] = m_idr[0];
							m_codecb_channel = m_idr[0] & 3;
							m_codecb_clock = (m_idr[0] >> 3) & 3;
							m_codecb_channel_count = (m_idr[0] >> 8) & 3;
							m_curr_adc = 0;
							update_adc_rate();
						}
						break;
					case 2:
						LOGMASKED(LOG_WRITES, "            Control 2\n");
						if (data & IAR_ACCESS_SEL)
						{
							m_idr[0] = m_codecb_ctrl[1];
							m_idr[1] = m_codecb_ctrl2_hi;
						}
						else
						{
							m_codecb_ctrl[1] = m_idr[0];
							m_codecb_ctrl2_hi = m_idr[1];
							update_input_gain();
						}
						break;
					default:
						LOGMASKED(LOG_WRITES, "            Unknown Register\n");
						break;
				}
				break;
			}
			case 0x0600:
				LOGMASKED(LOG_WRITES, "        Synth Control\n");
				break;
			default:
				LOGMASKED(LOG_WRITES, "        Unknown\n");
				break;
			}
			break;
		case 0x2000:
		{
			LOGMASKED(LOG_WRITES, "    Bresenham\n");
			uint32_t clock_gen = (data & IAR_NUM) >> IAR_NUM_SHIFT;
			if (clock_gen >= 1 && clock_gen <= 3)
			{
				LOGMASKED(LOG_WRITES, "        Bresenham Clock Gen %d\n", clock_gen);
				clock_gen--;
				const uint32_t param = (data & IAR_PARAM) >> IAR_PARAM_SHIFT;
				if (param == 1)
				{
					LOGMASKED(LOG_WRITES, "            Control 1 (Clock Select)\n");
					if (data & IAR_ACCESS_SEL)
					{
						m_idr[0] = m_bres_clock_sel[clock_gen];
					}
					else
					{
						m_bres_clock_sel[clock_gen] = m_idr[0];
						switch (m_idr[0])
						{
							case 0:  LOGMASKED(LOG_WRITES, "                48kHz\n"); break;
							case 1:  LOGMASKED(LOG_WRITES, "                44.1kHz\n"); break;
							case 2:  LOGMASKED(LOG_WRITES, "                Off\n"); break;
							default: LOGMASKED(LOG_WRITES, "                Unknown (%d)\n", m_idr[0]); break;
						}
						update_clock_freq(clock_gen);
					}
				}
				else if (param == 2)
				{
					LOGMASKED(LOG_WRITES, "            Control 2 (Inc/Mod Ctrl)\n");
					if (data & IAR_ACCESS_SEL)
					{
						m_idr[0] = m_bres_clock_inc[clock_gen];
						m_idr[1] = m_bres_clock_modctrl[clock_gen];
					}
					else
					{
						m_bres_clock_inc[clock_gen] = m_idr[0];
						m_bres_clock_modctrl[clock_gen] = m_idr[1];
						LOGMASKED(LOG_WRITES, "                Inc:%04x, ModCtrl:%04x\n", m_idr[0], m_idr[1]);
						update_clock_freq(clock_gen);
					}
				}
				else
				{
					LOGMASKED(LOG_WRITES, "            Unknown Param\n");
				}
			}
			else
			{
				LOGMASKED(LOG_WRITES, "        Unknown\n");
			}
			break;
		}

		case 0x3000:
			LOGMASKED(LOG_WRITES, "    Unix Timer\n");
			switch (data & IAR_NUM)
			{
			case 0x0100:
				LOGMASKED(LOG_WRITES, "        Unix Timer\n");
				break;
			default:
				LOGMASKED(LOG_WRITES, "        Unknown\n");
				break;
			}
			break;

		case 0x9000:
			LOGMASKED(LOG_WRITES, "    Global DMA Control\n");
			switch (data & IAR_NUM)
			{
			case 0x0100:
				LOGMASKED(LOG_WRITES, "        DMA Control\n");
				break;
			default:
				LOGMASKED(LOG_WRITES, "        Unknown\n");
				break;
			}
			break;
		}

		if (data & IAR_ACCESS_SEL)
			LOGMASKED(LOG_WRITES, "    Read\n");
		else
			LOGMASKED(LOG_WRITES, "    Write\n");

		LOGMASKED(LOG_WRITES, "    Parameter: %01x\n", (data & IAR_PARAM) >> 2);
		return;

	case INDIRECT_DATA0_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 0 Write: %04x\n", machine().describe_context(), data);
		m_idr[0] = data;
		return;

	case INDIRECT_DATA1_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 1 Write: %04x\n", machine().describe_context(), data);
		m_idr[1] = data;
		return;

	case INDIRECT_DATA2_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 2 Write: %04x\n", machine().describe_context(), data);
		m_idr[2] = data;
		return;

	case INDIRECT_DATA3_REG:
		LOGMASKED(LOG_WRITES, "%s: HAL2 Indirect Data Register 3 Write: %04x\n", machine().describe_context(), data);
		m_idr[3] = data;
		return;

	default:
		LOGMASKED(LOG_WRITES, "%s: Unknown HAL2 Write: %08x = %04x\n", machine().describe_context(), 0x1fbd8000 + offset*4, data);
		break;
	}
}

void hal2_device::update_clock_freq(int clock_gen)
{
	switch (m_bres_clock_sel[clock_gen])
	{
		case 0: // 48kHz
			m_bres_clock_freq[clock_gen] = 48000;
			break;
		case 1: // 44.1kHz
			m_bres_clock_freq[clock_gen] = 44100;
			break;
		default:
			m_bres_clock_freq[clock_gen] = 0;
			m_bres_clock_rate[clock_gen] = attotime::zero;
			update_dac_rate();
			update_adc_rate();
			return;
	}

	// MODCTRL = (INC - MOD - 1) & 0xffff, Fout = Fs * INC / MOD. A modulus of
	// zero makes the Bresenham accumulator fire on every master tick, so the
	// output is the master rate whatever INC is. IRIX relies on this: when it
	// changes the master clock it first writes INC=0, MODCTRL=0xffff and then
	// skips rewriting INC/MOD if they are unchanged (e.g. 1/1 at 48kHz and
	// then at 44.1kHz), leaving the generator in this state.
	const uint32_t mod = 0x10000 - ((m_bres_clock_modctrl[clock_gen] + 1) - m_bres_clock_inc[clock_gen]);
	if (mod == 0)
		m_bres_clock_rate[clock_gen] = attotime::from_ticks(1, m_bres_clock_freq[clock_gen]);
	else
		m_bres_clock_rate[clock_gen] = attotime::from_ticks(mod, m_bres_clock_freq[clock_gen] * m_bres_clock_inc[clock_gen]);
	update_dac_rate();
	update_adc_rate();
}

// HPC3 DMA writes each DAC once per codec A sample, so run the DAC streams
// at that rate rather than the DAC device's oversampled default (48kHz x32),
// which otherwise costs a continuous ~3M samples/s of stream generation and
// resampling whether or not anything is playing.
void hal2_device::update_dac_rate()
{
	u32 rate = 48000;
	if (m_codeca_clock > 0 && m_bres_clock_rate[m_codeca_clock - 1] != attotime::zero)
		rate = u32(m_bres_clock_rate[m_codeca_clock - 1].as_hz() + 0.5);
	// a stopped generator (period "never") has no rate; keep a valid stream
	if (rate < 1000)
		rate = 48000;
	m_ldac->set_stream_sample_rate(rate);
	m_rdac->set_stream_sample_rate(rate);
}

// MAME_SGI_HAL2_AUDIO_INPUT: run the microphone stream at codec B's sample
// rate, so each receive DMA transfer takes one stream sample per channel.
void hal2_device::update_adc_rate()
{
	u32 rate = 48000;
	if (m_codecb_clock > 0 && m_bres_clock_rate[m_codecb_clock - 1] != attotime::zero)
		rate = u32(m_bres_clock_rate[m_codecb_clock - 1].as_hz() + 0.5);
	if (rate < 1000)
		rate = 48000;
	if (rate != m_adc_rate)
	{
		// audio converted at the old rate would play at the wrong speed
		m_adc_rate = rate;
		m_adc_fifo_read = 0;
		m_adc_fifo_count = 0;
	}
}

void hal2_device::sound_stream_update(sound_stream &stream)
{
	// one input sample advances the output position by this many codec B
	// samples; linear interpolation between the previous and current input
	if (stream.sample_rate() == 0)
		return;
	const double step = double(m_adc_rate) / double(stream.sample_rate());
	for (int i = 0; i < stream.samples(); i++)
	{
		int16_t frame[2];
#ifdef MAME_SGI_HAL2_INPUT_TEST_TONE
		static constexpr double freq[2] = { 1000.0, 500.0 };
		for (int ch = 0; ch < 2; ch++)
		{
			frame[ch] = int16_t(std::clamp<double>(std::lround(16384.0 * m_adc_gain[ch] * std::sin(m_tone_phase[ch])), -32768.0, 32767.0));
			m_tone_phase[ch] = std::fmod(m_tone_phase[ch] + 2.0 * M_PI * freq[ch] / stream.sample_rate(), 2.0 * M_PI);
		}
#else
		for (int ch = 0; ch < 2; ch++)
			frame[ch] = int16_t(std::clamp<double>(std::lround(stream.get(ch, i) * m_adc_gain[ch] * 32767.0), -32768.0, 32767.0));
#endif
		while (m_adc_pos < 1.0)
		{
			int16_t out[2];
			for (int ch = 0; ch < 2; ch++)
				out[ch] = int16_t(std::lround(m_adc_prev[ch] + (frame[ch] - m_adc_prev[ch]) * m_adc_pos));
			// keep the newest audio: drop the oldest frame when the FIFO is full
			if (m_adc_fifo_count == ADC_FIFO_FRAMES)
			{
				m_adc_fifo_read = (m_adc_fifo_read + 1) % ADC_FIFO_FRAMES;
				m_adc_fifo_count--;
			}
			const unsigned write = (m_adc_fifo_read + m_adc_fifo_count) % ADC_FIFO_FRAMES;
			m_adc_fifo[write][0] = out[0];
			m_adc_fifo[write][1] = out[1];
			m_adc_fifo_count++;
			m_adc_pos += 1.0 / step;
		}
		m_adc_pos -= 1.0;
		m_adc_prev[0] = frame[0];
		m_adc_prev[1] = frame[1];
	}
}

// MAME_SGI_HAL2_AUDIO_INPUT: one 16-bit sample per receive DMA transfer,
// left then right in stereo. An empty FIFO repeats the last frame.
int16_t hal2_device::dma_read(uint32_t channel)
{
	if (channel != m_codecb_channel && channel != (3 - m_codecb_channel))
		return 0;

	if (m_curr_adc == 0)
	{
		if (m_adc_fifo_count == 0)
			m_adc_stream->update();
		if (m_adc_fifo_count != 0)
		{
			m_adc_frame[0] = m_adc_fifo[m_adc_fifo_read][0];
			m_adc_frame[1] = m_adc_fifo[m_adc_fifo_read][1];
			m_adc_fifo_read = (m_adc_fifo_read + 1) % ADC_FIFO_FRAMES;
			m_adc_fifo_count--;
		}
	}

	const int16_t sample = m_adc_frame[m_curr_adc & 1];
	m_curr_adc++;
	if (m_curr_adc >= std::max<uint16_t>(m_codecb_channel_count, 1))
		m_curr_adc = 0;
	return sample;
}

// MAME_SGI_HAL2_CODEC_GAINS: codec control 2, as SGI's HAL2 tests set it up:
// first word bit 10 mute, bits 9..8 input source, bits 7..4 / 3..0 left /
// right A/D gain in 1.5 dB steps; second word bits 11..7 / 6..2 left / right
// D/A attenuation (1.5 dB steps). Codec A drives the DACs, codec B the ADCs.
void hal2_device::update_output_gain()
{
	const bool mute = BIT(m_codeca_ctrl[1], 10);
	const unsigned atten[2] = { unsigned(BIT(m_codeca_ctrl2_hi, 7, 5)), unsigned(BIT(m_codeca_ctrl2_hi, 2, 5)) };
	for (int ch = 0; ch < 2; ch++)
	{
		const double gain = mute ? 0.0 : (m_volume[ch] / 255.0) * std::pow(10.0, -1.5 * atten[ch] / 20.0);
		(ch ? m_rdac : m_ldac)->set_output_gain(ALL_OUTPUTS, gain);
	}
}

// The host input is already at a normal digital level, so each source's
// usual IRIX setting is taken as unity: line in (source 0) at 0 dB, the
// microphone (source 3, which needs a preamp on real hardware) at IRIX's
// default 15 dB. Other settings scale relative to that.
void hal2_device::update_input_gain()
{
	const bool mute = BIT(m_codecb_ctrl[1], 10);
	const double unity_db = (BIT(m_codecb_ctrl[1], 8, 2) == 3) ? 15.0 : 0.0;
	const unsigned steps[2] = { unsigned(BIT(m_codecb_ctrl[1], 4, 4)), unsigned(BIT(m_codecb_ctrl[1], 0, 4)) };
	for (int ch = 0; ch < 2; ch++)
		m_adc_gain[ch] = mute ? 0.0 : std::pow(10.0, (1.5 * steps[ch] - unity_db) / 20.0);
}

attotime hal2_device::get_rate(const uint32_t channel)
{
	if ((channel == m_codeca_channel || channel == (3 - m_codeca_channel)) && m_codeca_clock > 0 && m_codeca_channel_count > 0)
		return m_bres_clock_rate[m_codeca_clock - 1] / m_codeca_channel_count;
	if ((channel == m_codecb_channel || channel == (3 - m_codecb_channel)) && m_codecb_clock > 0 && m_codecb_channel_count > 0)
		return m_bres_clock_rate[m_codecb_clock - 1] / m_codecb_channel_count;
	return attotime::zero;
}

void hal2_device::dma_write(uint32_t channel, int16_t data)
{
	if (channel >= 2)
		return;

	if (m_curr_dac)
		m_rdac->write(data);
	else
		m_ldac->write(data);

	m_curr_dac++;
	if (m_curr_dac == m_codeca_channel_count)
		m_curr_dac = 0;
}

void hal2_device::device_add_mconfig(machine_config &config)
{
	SPEAKER(config, "speaker", 2).front();

	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_ldac, 0);
	// MAME_SGI_HAL2_CODEC_GAINS: unity, so IRIX's full speaker volume is
	// digital full scale (it was 0.25, 12 dB below that)
	m_ldac->add_route(ALL_OUTPUTS, "speaker", 1.0, 0);

	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_rdac, 0);
	m_rdac->add_route(ALL_OUTPUTS, "speaker", 1.0, 1);

	// MAME_SGI_HAL2_AUDIO_INPUT: the AD1843's stereo line/microphone input
	MICROPHONE(config, m_mic, 2).front();
	m_mic->add_route(0, DEVICE_SELF, 1.0, 0);
	m_mic->add_route(1, DEVICE_SELF, 1.0, 1);
}
