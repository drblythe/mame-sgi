// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

    SGI HAL2 Audio Controller emulation

**********************************************************************/

#include "emu.h"

// MAME_SGI_HAL2_INPUT_TEST_TONE: feed codec B (the ADC) a 1 kHz sine on the
// left and 500 Hz on the right instead of the microphone, to check the
// recording path end to end from IRIX. MAMESGI_TEST_TONE_HZ=LEFT,RIGHT in
// the environment picks other frequencies. Off by default.
// #define MAME_SGI_HAL2_INPUT_TEST_TONE

#include "hal2.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace {
// pi; not every C library's <cmath> defines the POSIX constant (MinGW)
constexpr double HAL2_PI = 3.14159265358979323846;
}

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
	, m_aes_ldac(*this, "aes_ldac")
	, m_aes_rdac(*this, "aes_rdac")
	, m_aes_mic(*this, "aes_in")
	, m_aes_config(*this, "AES")
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
	save_item(NAME(m_aestx_ctrl));
	save_item(NAME(m_aesrx_ctrl));
	save_item(NAME(m_aesrx_reg));
	save_item(NAME(m_aesrx_ier));
	save_item(NAME(m_aestx_reg));
	save_item(NAME(m_aes_ram));
	save_item(NAME(m_curr_aes_tx));
	save_item(NAME(m_curr_aes_rx));
	save_item(NAME(m_aes_tx_frame));
	save_item(NAME(m_aes_rx_frame));

	// MAME_SGI_HAL2_AUDIO_INPUT: four inputs (microphone left/right, AES in
	// left/right) at the host's own rate, pulled by dma_read(). The rate
	// never changes: sound_stream::set_sample_rate() on a stream with
	// inputs left MAME's input resampler reading past its buffer (a crash
	// when IRIX switched codec B to 44.1 kHz), so sound_stream_update()
	// converts to the guest rates itself.
	m_adc_stream = stream_alloc(4, 0, SAMPLE_RATE_INPUT_ADAPTIVE);

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
	// MAME_SGI_HAL2_AES: the received rate is derived, not saved
	m_aes_rx_hz = aes_rx_hz();
	for (int clock_gen = 0; clock_gen < 3; clock_gen++)
		update_clock_freq(clock_gen);
	m_adc_fifo.clear();
	m_aes_fifo.clear();
	m_aes_rx_sync = true;
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
	m_adc_fifo.clear();
	// MAME_SGI_HAL2_AES: both chips power up with every control bit clear
	// (held in reset)
	m_aestx_ctrl = 0;
	m_aesrx_ctrl = 0;
	std::fill(std::begin(m_aesrx_reg), std::end(m_aesrx_reg), 0);
	std::fill(std::begin(m_aesrx_ier), std::end(m_aesrx_ier), 0);
	std::fill(std::begin(m_aestx_reg), std::end(m_aestx_reg), 0);
	m_aes_fifo.clear();
	m_aes_rx_hz = 0;
	m_aes_rx_sync = true;
	m_curr_aes_tx = 0;
	m_curr_aes_rx = 0;
	m_aes_tx_frame[0] = m_aes_tx_frame[1] = 0;
	m_aes_rx_frame[0] = m_aes_rx_frame[1] = 0;
	update_aes_tx_rate();
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
			// MAME_SGI_HAL2_AES: control 1 of each AES port, laid out like the
			// codecs' (bits 2..0 DMA channel, 4..3 clock, 9..8 channel count)
			case 0x0200:
				LOGMASKED(LOG_WRITES, "        AES In\n");
				if (((data & IAR_PARAM) >> IAR_PARAM_SHIFT) == 1)
				{
					if (data & IAR_ACCESS_SEL)
						m_idr[0] = m_aesrx_ctrl;
					else
					{
						m_aesrx_ctrl = m_idr[0];
						m_curr_aes_rx = 0;
						m_aes_rx_sync = true;
					}
				}
				break;
			case 0x0300:
				LOGMASKED(LOG_WRITES, "        AES Out\n");
				if (((data & IAR_PARAM) >> IAR_PARAM_SHIFT) == 1)
				{
					if (data & IAR_ACCESS_SEL)
						m_idr[0] = m_aestx_ctrl;
					else
					{
						m_aestx_ctrl = m_idr[0];
						m_curr_aes_tx = 0;
						update_aes_tx_rate();
						update_aes_master();
					}
				}
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
						update_aes_master();   // MAME_SGI_HAL2_AES: the loopback rate follows the transmit clock
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
						update_aes_master();   // MAME_SGI_HAL2_AES
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
		case 2: // MAME_SGI_HAL2_AES: the AES receiver's recovered clock
			if (m_aes_rx_hz)
			{
				m_bres_clock_freq[clock_gen] = m_aes_rx_hz;
				break;
			}
			[[fallthrough]];   // not locked: no clock
		default:
			m_bres_clock_freq[clock_gen] = 0;
			m_bres_clock_rate[clock_gen] = attotime::zero;
			update_dac_rate();
			update_adc_rate();
			update_aes_tx_rate();
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
	update_aes_tx_rate();
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
		m_adc_fifo.clear();
		m_adc_conv.reset();
	}
}

void hal2_device::sound_stream_update(sound_stream &stream)
{
	if (stream.sample_rate() == 0)
		return;
	const double adc_step = double(m_adc_rate) / double(stream.sample_rate());
	const bool aes_host = aes_connection() == AES_HOST;
	const double aes_step = double(m_aes_rx_hz ? m_aes_rx_hz : 48000) / double(stream.sample_rate());
	for (int i = 0; i < stream.samples(); i++)
	{
		int16_t frame[2];
#ifdef MAME_SGI_HAL2_INPUT_TEST_TONE
		static double freq[2] = { 0.0, 0.0 };
		if (freq[0] == 0.0)
		{
			freq[0] = 1000.0;
			freq[1] = 500.0;
			if (const char *env = std::getenv("MAMESGI_TEST_TONE_HZ"))
				std::sscanf(env, "%lf,%lf", &freq[0], &freq[1]);
			osd_printf_info("HAL2 test tone: %.1f Hz left, %.1f Hz right, input stream at %d Hz\n", freq[0], freq[1], stream.sample_rate());
		}
		for (int ch = 0; ch < 2; ch++)
		{
			frame[ch] = int16_t(std::clamp<double>(std::lround(16384.0 * m_adc_gain[ch] * std::sin(m_tone_phase[ch])), -32768.0, 32767.0));
			m_tone_phase[ch] = std::fmod(m_tone_phase[ch] + 2.0 * HAL2_PI * freq[ch] / stream.sample_rate(), 2.0 * HAL2_PI);
		}
#else
		for (int ch = 0; ch < 2; ch++)
			frame[ch] = int16_t(std::clamp<double>(std::lround(stream.get(ch, i) * m_adc_gain[ch] * 32767.0), -32768.0, 32767.0));
#endif
		sgi_audio_in::convert_input(frame, adc_step, m_adc_conv, m_adc_fifo);

		// MAME_SGI_HAL2_AES: digital input, no gain stage
		if (aes_host && m_aes_rx_hz)
		{
			int16_t aes[2];
			for (int ch = 0; ch < 2; ch++)
				aes[ch] = int16_t(std::clamp<double>(std::lround(stream.get(2 + ch, i) * 32767.0), -32768.0, 32767.0));
			sgi_audio_in::convert_input(aes, aes_step, m_aes_conv, m_aes_fifo);
		}
	}
}

// MAME_SGI_HAL2_AUDIO_INPUT: one 16-bit sample per receive DMA transfer,
// left then right in stereo. An empty FIFO repeats the last frame.
int16_t hal2_device::dma_read(uint32_t channel)
{
	// MAME_SGI_HAL2_AES
	if (m_aesrx_ctrl && channel == aesrx_channel() && channel != m_codecb_channel)
		return aes_rx_sample();

	if (channel != m_codecb_channel && channel != (3 - m_codecb_channel))
		return 0;

	if (m_curr_adc == 0)
	{
		const attotime now = machine().time();
		if (now - m_adc_last_read > attotime::from_msec(20))
		{
			// a recording starts: drop the frames that queued up while idle
			m_adc_stream->update();
			m_adc_fifo.trim(ADC_START_FRAMES);
		}
		m_adc_last_read = now;
		if (m_adc_fifo.count == 0)
			m_adc_stream->update();
		m_adc_fifo.pop(m_adc_frame);
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
	if (channel == m_codeca_channel && m_codeca_clock > 0 && m_codeca_channel_count > 0)
		return m_bres_clock_rate[m_codeca_clock - 1] / m_codeca_channel_count;
	if (channel == m_codecb_channel && m_codecb_clock > 0 && m_codecb_channel_count > 0)
		return m_bres_clock_rate[m_codecb_clock - 1] / m_codecb_channel_count;

	// MAME_SGI_HAL2_AES: the transmitter runs from its Bresenham clock, the
	// receiver from the received stream (no clock when not locked). Before
	// this, the AES channels (2 and 3 under IRIX) ran at the codecs' rates
	// through the codec pairing below.
	if (m_aestx_ctrl && channel == aestx_channel())
		return aestx_clock() > 0 ? m_bres_clock_rate[aestx_clock() - 1] / aestx_count() : attotime::zero;
	if (m_aesrx_ctrl && channel == aesrx_channel())
		return m_aes_rx_hz ? attotime::from_hz(m_aes_rx_hz) / 2 : attotime::zero;

	if (channel == (3 - m_codeca_channel) && m_codeca_clock > 0 && m_codeca_channel_count > 0)
		return m_bres_clock_rate[m_codeca_clock - 1] / m_codeca_channel_count;
	if (channel == (3 - m_codecb_channel) && m_codecb_clock > 0 && m_codecb_channel_count > 0)
		return m_bres_clock_rate[m_codecb_clock - 1] / m_codecb_channel_count;
	return attotime::zero;
}

void hal2_device::dma_write(uint32_t channel, int16_t data)
{
	// MAME_SGI_HAL2_AES
	if (m_aestx_ctrl && channel == aestx_channel() && channel != m_codeca_channel)
	{
		aes_tx_sample(data);
		return;
	}

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

	// MAME_SGI_HAL2_AES: the AES/S/PDIF connector, used with the host
	// connection setting; map them in the Audio Mixer
	SPEAKER(config, "aes_out", 2).front();
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_aes_ldac, 0);
	m_aes_ldac->add_route(ALL_OUTPUTS, "aes_out", 1.0, 0);
	DAC_16BIT_R2R_TWOS_COMPLEMENT(config, m_aes_rdac, 0);
	m_aes_rdac->add_route(ALL_OUTPUTS, "aes_out", 1.0, 1);
	MICROPHONE(config, m_aes_mic, 2).front();
	m_aes_mic->add_route(0, DEVICE_SELF, 1.0, 2);
	m_aes_mic->add_route(1, DEVICE_SELF, 1.0, 3);
}

// MAME_SGI_HAL2_AES: what is plugged into the AES connector
INPUT_PORTS_START(hal2)
	PORT_START("AES")
	PORT_CONFNAME(0x03, 0x00, "AES Digital Audio Connection") PORT_CHANGED_MEMBER(DEVICE_SELF, FUNC(hal2_device::aes_connection_changed), 0)
	PORT_CONFSETTING(0x00, "None")
	PORT_CONFSETTING(0x01, "Loopback (AES out to AES in)")
	PORT_CONFSETTING(0x02, "Host audio")
INPUT_PORTS_END

ioport_constructor hal2_device::device_input_ports() const
{
	return INPUT_PORTS_NAME(hal2);
}

INPUT_CHANGED_MEMBER(hal2_device::aes_connection_changed)
{
	update_aes_master();
}

unsigned hal2_device::aes_connection() const
{
	return m_aes_config->read() & 3;
}

// The received sample rate: what IRIX can lock to. Zero with nothing
// connected or the receiver held in reset (CR1 RST clear); in loopback,
// the transmitter's frame rate while its drivers are on (CR2 RST), unless
// its own clock is the AES clock (no signal to recover); with the host
// connection, 48 kHz.
u32 hal2_device::aes_rx_hz() const
{
	if (m_absent || !BIT(m_aesrx_reg[2], 0))
		return 0;
	switch (aes_connection())
	{
	case AES_LOOPBACK:
	{
		const unsigned clock = aestx_clock();
		if (!m_aestx_ctrl || !BIT(m_aestx_reg[2], 0) || clock == 0 || m_bres_clock_sel[clock - 1] == 2 || m_bres_clock_rate[clock - 1] == attotime::zero)
			return 0;
		return u32(m_bres_clock_rate[clock - 1].as_hz() + 0.5);
	}
	case AES_HOST:
		return 48000;
	default:
		return 0;
	}
}

// Recompute the received rate and the Bresenham clocks that use it. A
// change of rate (or of lock) restarts the received stream.
void hal2_device::update_aes_master()
{
	const u32 hz = aes_rx_hz();
	if (hz != m_aes_rx_hz)
	{
		m_aes_rx_hz = hz;
		m_aes_fifo.clear();
		m_aes_rx_sync = true;
	}
	for (int clock_gen = 0; clock_gen < 3; clock_gen++)
		if (m_bres_clock_sel[clock_gen] == 2)
			update_clock_freq(clock_gen);
}

// one DAC write per transmitted frame, as for codec A
void hal2_device::update_aes_tx_rate()
{
	u32 rate = 48000;
	const unsigned clock = aestx_clock();
	if (m_aestx_ctrl && clock > 0 && m_bres_clock_rate[clock - 1] != attotime::zero)
		rate = u32(m_bres_clock_rate[clock - 1].as_hz() + 0.5);
	if (rate < 1000)
		rate = 48000;
	m_aes_ldac->set_stream_sample_rate(rate);
	m_aes_rdac->set_stream_sample_rate(rate);
}

// a transmitted sample: frames go to the receiver (loopback) or the host
// (CS8401A CR2: RST clear turns the line drivers off, MUTE clear sends zeros)
void hal2_device::aes_tx_sample(int16_t data)
{
	const unsigned count = aestx_count();
	if (m_curr_aes_tx < 2)
		m_aes_tx_frame[m_curr_aes_tx] = data;
	if (count == 1)
		m_aes_tx_frame[1] = data;
	if (++m_curr_aes_tx < count)
		return;
	m_curr_aes_tx = 0;

	if (!BIT(m_aestx_reg[2], 0))
		return;
	const bool mute = !BIT(m_aestx_reg[2], 1);
	const int16_t l = mute ? 0 : m_aes_tx_frame[0];
	const int16_t r = mute ? 0 : m_aes_tx_frame[1];
	switch (aes_connection())
	{
	case AES_LOOPBACK:
		if (m_aes_rx_hz)
			m_aes_fifo.push(l, r);
		break;
	case AES_HOST:
		m_aes_ldac->write(l);
		m_aes_rdac->write(r);
		break;
	}
}

// a received sample, left then right. Loopback starts each run a fixed
// number of frames behind the transmitter and sends zeros until then (and
// after an underrun), so the received audio is the transmitted audio,
// sample for sample.
int16_t hal2_device::aes_rx_sample()
{
	if (m_curr_aes_rx == 0)
	{
		if (aes_connection() == AES_HOST)
		{
			if (m_aes_fifo.count == 0)
				m_adc_stream->update();
			m_aes_fifo.pop(m_aes_rx_frame);
		}
		else
		{
			if (m_aes_rx_sync && m_aes_fifo.count >= AES_LOOPBACK_LATENCY)
			{
				m_aes_fifo.trim(AES_LOOPBACK_LATENCY);
				m_aes_rx_sync = false;
			}
			if (m_aes_rx_sync || !m_aes_fifo.pop(m_aes_rx_frame))
			{
				m_aes_rx_sync = true;
				m_aes_rx_frame[0] = m_aes_rx_frame[1] = 0;
			}
		}
	}
	const int16_t sample = m_aes_rx_frame[m_curr_aes_rx & 1];
	m_curr_aes_rx = (m_curr_aes_rx + 1) & 1;
	return sample;
}

// The chips' registers are bytes; the PBUS channel puts them on data lines
// 15..8 (IRIX's setting) or 7..0, so both halves carry the byte.
uint16_t hal2_device::aes_r(offs_t offset, uint16_t mem_mask)
{
	if (m_absent || offset >= 0x40)
		return m_aes_ram[offset & 0xff];
	const uint8_t data = (offset < 0x20) ? aesrx_read(offset) : m_aestx_reg[offset & 0x1f];
	return (uint16_t(data) << 8) | data;
}

void hal2_device::aes_w(offs_t offset, uint16_t data, uint16_t mem_mask)
{
	if (m_absent || offset >= 0x40)
	{
		COMBINE_DATA(&m_aes_ram[offset & 0xff]);
		return;
	}
	const uint8_t byte = ACCESSING_BITS_8_15 ? uint8_t(data >> 8) : uint8_t(data);
	if (offset < 0x20)
		aesrx_write(offset, byte);
	else
		aestx_write(offset & 0x1f, byte);
}

// CS8411 receiver, buffer mode 0 (IRIX's): 0 status 1 / interrupt enable 1,
// 1 status 2 / interrupt enable 2 (CR1 IER/SR selects), 2-3 control,
// 4-7 user data, 8-31 the received channel status bytes 0-23.
uint8_t hal2_device::aesrx_read(unsigned reg)
{
	const bool locked = m_aes_rx_hz != 0;
	switch (reg)
	{
	case 0:
		return BIT(m_aesrx_reg[2], 5) ? m_aesrx_ier[0] : 0x00;
	case 1:
	{
		if (BIT(m_aesrx_reg[2], 5))
			return m_aesrx_ier[1];
		// bit 3 LOCK: set while the PLL is not locked. FREQ (bits 7..5) is
		// only valid with the frequency comparator on (CR1 FCEN).
		uint8_t sr2 = locked ? 0x00 : 0x08;
		if (locked && BIT(m_aesrx_reg[2], 6))
			sr2 |= (m_aes_rx_hz == 48000) ? 0x80 : (m_aes_rx_hz == 44100) ? 0xa0 : (m_aes_rx_hz == 32000) ? 0xe0 : 0x00;
		return sr2;
	}
	case 2:
	case 3:
		return m_aesrx_reg[reg];
	default:
		if (!locked || reg < 8)
			return 0x00;
		if (aes_connection() == AES_LOOPBACK)
			return m_aestx_reg[reg];   // the transmitted channel status
		// host: consumer format, 48 kHz (byte 3 bits 3..0 = 2)
		return (reg == 8 + 3) ? 0x02 : 0x00;
	}
}

void hal2_device::aesrx_write(unsigned reg, uint8_t data)
{
	switch (reg)
	{
	case 0:
	case 1:
		if (BIT(m_aesrx_reg[2], 5))
			m_aesrx_ier[reg] = data;
		break;
	case 2:
		m_aesrx_reg[2] = data;
		update_aes_master();
		break;
	case 3:
		m_aesrx_reg[3] = data;
		break;
	default:   // the buffer is written by the receiver, not the host
		break;
	}
}

// CS8401A transmitter: 0 status (read only), 1-3 control, 4-7 user data,
// 8-31 channel status bytes 0-23 (buffer mode 0).
void hal2_device::aestx_write(unsigned reg, uint8_t data)
{
	if (reg == 0)
		return;
	m_aestx_reg[reg] = data;
	if (reg == 2)
		update_aes_master();
}
