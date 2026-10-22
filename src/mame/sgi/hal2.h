// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

    SGI HAL2 Audio Controller emulation

**********************************************************************/

#ifndef MAME_SGI_HAL2_H
#define MAME_SGI_HAL2_H

#pragma once

#include "sound/dac.h"
#include "speaker.h"

class hal2_device : public device_t, public device_sound_interface
{
public:
	hal2_device(const machine_config &mconfig, const char *tag, device_t *owner)
		: hal2_device(mconfig, tag, owner, (uint32_t)0)
	{
	}

	hal2_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock);

	void write(offs_t offset, uint16_t data);
	uint16_t read(offs_t offset);

	// Unpopulated chip (e.g. Challenge S): the revision register reads as a
	// floating bus, whose bit 15 tells the PROM and IRIX there is no audio.
	void set_absent(bool absent) { m_absent = absent; }

	attotime get_rate(const uint32_t channel);

	// headphone/speaker volume register: linear, IRIX shows 20*log10(vol) dB
	// (64 = its default, 255 = full scale)
	void set_right_volume(uint8_t vol) { m_volume[1] = vol; update_output_gain(); }
	void set_left_volume(uint8_t vol) { m_volume[0] = vol; update_output_gain(); }

	void dma_write(uint32_t channel, int16_t data);
	// MAME_SGI_HAL2_AUDIO_INPUT: next codec B (ADC) sample for a receive DMA
	// channel, alternating left and right
	int16_t dma_read(uint32_t channel);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override;

	void update_clock_freq(int clock_gen);
	void update_dac_rate();
	void update_adc_rate();
	void update_output_gain();   // MAME_SGI_HAL2_CODEC_GAINS
	void update_input_gain();    // MAME_SGI_HAL2_CODEC_GAINS
	void postload();  // MAME_SGI_HAL2_BRES_POSTLOAD_FIX - see device_start()'s comment

	enum
	{
		IAR_TYPE            = 0xf000,
		IAR_TYPE_SHIFT      = 12,
		IAR_NUM             = 0x0f00,
		IAR_NUM_SHIFT       = 8,
		IAR_ACCESS_SEL      = 0x0080,
		IAR_PARAM           = 0x000c,
		IAR_PARAM_SHIFT     = 2,
		IAR_RB_INDEX        = 0x0003
	};

	enum
	{
		ISR_TSTATUS      = 0x01,
		ISR_USTATUS      = 0x02,
		ISR_QUAD_MODE    = 0x04,
		ISR_GLOBAL_RESET = 0x08,
		ISR_CODEC_RESET  = 0x10,
	};

	enum
	{
		STATUS_REG           = 0x0010/4,
		REVISION_REG         = 0x0020/4,
		INDIRECT_ADDRESS_REG = 0x0030/4,
		INDIRECT_DATA0_REG   = 0x0040/4,
		INDIRECT_DATA1_REG   = 0x0050/4,
		INDIRECT_DATA2_REG   = 0x0060/4,
		INDIRECT_DATA3_REG   = 0x0070/4,
	};

	enum
	{
		DAC_L,
		DAC_R
	};

	required_device<dac_16bit_r2r_twos_complement_device> m_ldac;
	required_device<dac_16bit_r2r_twos_complement_device> m_rdac;
	required_device<microphone_device> m_mic;

	// MAME_SGI_HAL2_AUDIO_INPUT: the microphone feeds a two-input stream at
	// codec B's sample rate; its samples wait in a FIFO for receive DMA.
	// about 43 ms at 48 kHz: over two sound updates of slack, and the most
	// the input can lag when the FIFO is full between recordings
	static constexpr unsigned ADC_FIFO_FRAMES = 2048;
	sound_stream *m_adc_stream = nullptr;
	int16_t m_adc_fifo[ADC_FIFO_FRAMES][2]{};
	unsigned m_adc_fifo_read = 0;
	unsigned m_adc_fifo_count = 0;
	uint16_t m_curr_adc = 0;
	int16_t m_adc_frame[2]{};
	u32 m_adc_rate = 48000;       // codec B sample rate
	double m_adc_pos = 0.0;       // next output position between m_adc_prev and the current input
	int16_t m_adc_prev[2]{};
	double m_tone_phase[2]{};   // MAME_SGI_HAL2_INPUT_TEST_TONE only (unconditional: same layout in every file)

	uint16_t m_isr = 0;
	uint16_t m_iar = 0;
	uint16_t m_idr[4]{};

	uint16_t m_codeca_ctrl[2]{};
	uint16_t m_codeca_ctrl2_hi = 0;   // MAME_SGI_HAL2_CODEC_GAINS: control 2 second word (D/A attenuation)
	uint16_t m_codeca_channel = 0;
	uint16_t m_codeca_clock = 0;
	uint16_t m_codeca_channel_count = 0;

	uint16_t m_codecb_ctrl[2]{};
	uint16_t m_codecb_ctrl2_hi = 0;
	uint16_t m_codecb_channel = 0;
	uint16_t m_codecb_clock = 0;
	uint16_t m_codecb_channel_count = 0;

	uint16_t m_bres_clock_sel[3]{};
	uint16_t m_bres_clock_inc[3]{};
	uint16_t m_bres_clock_modctrl[3]{};
	uint16_t m_bres_clock_freq[3]{};
	attotime m_bres_clock_rate[3]{};

	uint16_t m_curr_dac = 0;

	// MAME_SGI_HAL2_CODEC_GAINS
	uint8_t m_volume[2] = { 255, 255 };
	double m_adc_gain[2] = { 1.0, 1.0 };

	static const uint32_t s_channel_pair[4];
	bool m_absent = false;
};

DECLARE_DEVICE_TYPE(SGI_HAL2, hal2_device)

#endif // MAME_SGI_HAL2_H
