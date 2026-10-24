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

#include <vector>

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

	// MAME_SGI_HAL2_AES: the CS8411 AES receiver (PBUS PIO1 words
	// 0x00-0x1f) and CS8401A transmitter (words 0x20-0x3f), one register
	// byte per word; the rest of the channel reads back as RAM
	uint16_t aes_r(offs_t offset, uint16_t mem_mask);
	void aes_w(offs_t offset, uint16_t data, uint16_t mem_mask);
	DECLARE_INPUT_CHANGED_MEMBER(aes_connection_changed);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;
	virtual ioport_constructor device_input_ports() const override ATTR_COLD;
	virtual void sound_stream_update(sound_stream &stream) override;

	void update_clock_freq(int clock_gen);
	void update_dac_rate();
	void update_adc_rate();
	void update_output_gain();   // MAME_SGI_HAL2_CODEC_GAINS
	void update_input_gain();    // MAME_SGI_HAL2_CODEC_GAINS
	void postload();  // MAME_SGI_HAL2_BRES_POSTLOAD_FIX - see device_start()'s comment

	// MAME_SGI_HAL2_AES
	enum
	{
		AES_NONE = 0,       // nothing connected: the receiver never locks
		AES_LOOPBACK,       // a cable from AES out to AES in
		AES_HOST            // AES out/in are MAME sound devices for the host
	};
	unsigned aes_connection() const;
	unsigned aestx_channel() const { return m_aestx_ctrl & 7; }
	unsigned aestx_clock() const { return (m_aestx_ctrl >> 3) & 3; }
	unsigned aestx_count() const { const unsigned n = (m_aestx_ctrl >> 8) & 3; return n ? n : 2; }
	unsigned aesrx_channel() const { return m_aesrx_ctrl & 7; }
	u32 aes_rx_hz() const;
	void update_aes_master();
	void update_aes_tx_rate();
	uint8_t aesrx_read(unsigned reg);
	void aesrx_write(unsigned reg, uint8_t data);
	void aestx_write(unsigned reg, uint8_t data);
	void aes_tx_sample(int16_t data);
	int16_t aes_rx_sample();

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

	// MAME_SGI_HAL2_AUDIO_INPUT: stereo frames waiting for receive DMA.
	// 2048 frames is about 43 ms at 48 kHz: over two sound updates of
	// slack, and the most the input can lag when the FIFO is full between
	// recordings. When full, the oldest frame is dropped.
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
	static void convert_input(const int16_t frame[2], double step, input_converter &conv, frame_fifo &fifo);

	// MAME_SGI_HAL2_AUDIO_INPUT: the microphone (inputs 0-1) and, with the
	// host AES connection, the AES input (inputs 2-3) feed one stream at
	// the host rate, converted into FIFOs at codec B's and the AES rate
	sound_stream *m_adc_stream = nullptr;
	frame_fifo m_adc_fifo;
	input_converter m_adc_conv;
	uint16_t m_curr_adc = 0;
	int16_t m_adc_frame[2]{};
	u32 m_adc_rate = 48000;       // codec B sample rate
	double m_tone_phase[2]{};   // MAME_SGI_HAL2_INPUT_TEST_TONE only (unconditional: same layout in every file)

	// MAME_SGI_HAL2_AES
	required_device<dac_16bit_r2r_twos_complement_device> m_aes_ldac;
	required_device<dac_16bit_r2r_twos_complement_device> m_aes_rdac;
	required_device<microphone_device> m_aes_mic;
	required_ioport m_aes_config;
	// loopback: frames buffered between transmit and receive DMA, which run
	// at the same rate; receive starts this many frames behind transmit so
	// timer ordering never underruns it
	static constexpr unsigned AES_LOOPBACK_LATENCY = 64;
	uint16_t m_aestx_ctrl = 0;    // HAL2 AES TX control: DMA channel, clock, channel count
	uint16_t m_aesrx_ctrl = 0;    // HAL2 AES RX control: DMA channel
	uint8_t m_aesrx_reg[32]{};    // CS8411: 0/1 status, 2/3 control, 4.. buffer
	uint8_t m_aesrx_ier[2]{};     // CS8411 interrupt enables (at 0/1 when CR1 IER/SR is set)
	uint8_t m_aestx_reg[32]{};    // CS8401A: 0 status, 1-3 control, 4-7 user data, 8.. channel status
	uint16_t m_aes_ram[0x100]{};  // the rest of PBUS PIO1, and all of it with no HAL2 fitted
	frame_fifo m_aes_fifo;        // AES input frames (loopback or host)
	input_converter m_aes_conv;
	u32 m_aes_rx_hz = 0;          // received sample rate, 0 when the receiver is not locked
	bool m_aes_rx_sync = true;    // loopback: waiting for the latency to build up
	uint16_t m_curr_aes_tx = 0;
	uint16_t m_curr_aes_rx = 0;
	int16_t m_aes_tx_frame[2]{};
	int16_t m_aes_rx_frame[2]{};

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
