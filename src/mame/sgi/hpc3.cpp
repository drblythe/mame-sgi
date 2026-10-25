// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

    SGI HPC3 "High-performance Peripheral Controller" emulation

**********************************************************************/

#include "emu.h"


#include "hpc3.h"

#include "endianness.h"

#ifdef MAME_SGI_HW_FIXES
#include "mc.h"
#endif

#include <algorithm>

#include "util/multibyte.h"

#define LOG_UNKNOWN     (1U << 1)
#define LOG_PBUS_DMA    (1U << 2)
#define LOG_SCSI        (1U << 3)
#define LOG_SCSI_DMA    (1U << 4)
#define LOG_SCSI_IRQ    (1U << 5)
#define LOG_ETHERNET    (1U << 6)
#define LOG_CHAIN       (1U << 7)
#define LOG_EEPROM      (1U << 8)
#define LOG_ALL         (LOG_UNKNOWN | LOG_PBUS_DMA | LOG_SCSI | LOG_SCSI_DMA | LOG_SCSI_IRQ | LOG_ETHERNET | LOG_CHAIN | LOG_EEPROM)

#define VERBOSE         (0)
#include "logmacro.h"

DEFINE_DEVICE_TYPE(SGI_HPC3, hpc3_device, "hpc3", "SGI HPC3")

hpc3_device::hpc3_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, SGI_HPC3, tag, owner, clock)
	, device_memory_interface(mconfig, *this)
	, m_pio_space_config{
		{"pio0", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio1", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio2", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio3", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio4", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio5", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio6", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio7", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio8", ENDIANNESS_LITTLE, 16, 8, -1},
		{"pio9", ENDIANNESS_LITTLE, 16, 8, -1}}
	, m_gio64_space(*this, finder_base::DUMMY_TAG, -1)
	, m_hal2(*this, finder_base::DUMMY_TAG)
	, m_enet(*this, finder_base::DUMMY_TAG)
#ifdef MAME_SGI_HW_FIXES
	, m_mem_ctrl(*this, finder_base::DUMMY_TAG)
#endif
	, m_enet_intr_out_cb(*this)
	, m_hd_rd_cb(*this, 0)
	, m_hd_wr_cb(*this)
	, m_hd_dma_rd_cb(*this, 0)
	, m_hd_dma_wr_cb(*this)
	, m_hd_reset_cb(*this)
	, m_bbram_rd_cb(*this, 0)
	, m_bbram_wr_cb(*this)
	, m_eeprom_dati_cb(*this, 0)
	, m_eeprom_dato_cb(*this)
	, m_eeprom_clk_cb(*this)
	, m_eeprom_cs_cb(*this)
	, m_eeprom_pre_cb(*this)
	, m_dma_complete_int_cb(*this)
{
}

device_memory_interface::space_config_vector hpc3_device::memory_space_config() const
{
	return space_config_vector {
		std::make_pair(AS_PIO0, &m_pio_space_config[0]),
		std::make_pair(AS_PIO1, &m_pio_space_config[1]),
		std::make_pair(AS_PIO2, &m_pio_space_config[2]),
		std::make_pair(AS_PIO3, &m_pio_space_config[3]),
		std::make_pair(AS_PIO4, &m_pio_space_config[4]),
		std::make_pair(AS_PIO5, &m_pio_space_config[5]),
		std::make_pair(AS_PIO6, &m_pio_space_config[6]),
		std::make_pair(AS_PIO7, &m_pio_space_config[7]),
		std::make_pair(AS_PIO8, &m_pio_space_config[8]),
		std::make_pair(AS_PIO9, &m_pio_space_config[9])
	};
}

void hpc3_device::device_start()
{
	for (uint32_t i = 0; i < 10; i++)
		m_pio_space[i] = &space(AS_PIO0 + i);

	m_gio64_cached = m_gio64_space->addr_width() == 32 && m_gio64_space->data_width() == 64
			&& m_gio64_space->addr_shift() == 0 && m_gio64_space->endianness() == ENDIANNESS_BIG;
	if (m_gio64_cached)
		m_gio64_space->cache(m_gio64_cache);

	save_item(NAME(m_intstat));
	save_item(NAME(m_misc));
	save_item(NAME(m_cpu_aux_ctrl));
	save_item(NAME(m_pio_config));

	for (uint32_t i = 0; i < 2; i++)
	{
		save_item(NAME(m_scsi_dma[i].m_cbp), i);
		save_item(NAME(m_scsi_dma[i].m_nbdp), i);
		save_item(NAME(m_scsi_dma[i].m_ctrl), i);
		save_item(NAME(m_scsi_dma[i].m_bc), i);
		save_item(NAME(m_scsi_dma[i].m_count), i);
		save_item(NAME(m_scsi_dma[i].m_dmacfg), i);
		save_item(NAME(m_scsi_dma[i].m_piocfg), i);
		save_item(NAME(m_scsi_dma[i].m_drq), i);
		save_item(NAME(m_scsi_dma[i].m_big_endian), i);
		save_item(NAME(m_scsi_dma[i].m_to_device), i);
		save_item(NAME(m_scsi_dma[i].m_active), i);
	}

	save_item(NAME(m_enet_rx_cbp));
	save_item(NAME(m_enet_rx_nbdp));
	save_item(NAME(m_enet_rx_bc));
	save_item(NAME(m_enet_rx_ctrl));
	save_item(NAME(m_enet_rx_gio));
	save_item(NAME(m_enet_rx_dev));

	save_item(NAME(m_enet_misc));
	save_item(NAME(m_enet_dmacfg));
	save_item(NAME(m_enet_piocfg));

	save_item(NAME(m_enet_tx_cbp));
	save_item(NAME(m_enet_tx_nbdp));
	save_item(NAME(m_enet_tx_bc));
	save_item(NAME(m_enet_tx_ctrl));
	save_item(NAME(m_enet_tx_ready));
	save_item(NAME(m_enet_tx_eox));
	save_item(NAME(m_enet_rx_ready));
	save_item(NAME(m_enet_tx_gio));
	save_item(NAME(m_enet_tx_dev));

	save_item(NAME(m_enet_rx_cbdp));
	save_item(NAME(m_enet_tx_cpfbdp));
	save_item(NAME(m_enet_tx_ppfbdp));

	for (uint32_t i = 0; i < 8; i++)
	{
		save_item(NAME(m_pbus_dma[i].m_active), i);
		save_item(NAME(m_pbus_dma[i].m_cur_ptr), i);
		save_item(NAME(m_pbus_dma[i].m_desc_ptr), i);
		save_item(NAME(m_pbus_dma[i].m_desc_flags), i);
		save_item(NAME(m_pbus_dma[i].m_next_ptr), i);
		save_item(NAME(m_pbus_dma[i].m_bytes_left), i);
		save_item(NAME(m_pbus_dma[i].m_config), i);
		save_item(NAME(m_pbus_dma[i].m_control), i);

		m_pbus_dma[i].m_timer = timer_alloc(FUNC(hpc3_device::do_pbus_dma), this);
		m_pbus_dma[i].m_timer->adjust(attotime::never);
	}

	m_pbus_fifo = make_unique_clear<uint32_t[]>(96);
	m_scsi_fifo[0] = make_unique_clear<uint32_t[]>(96);
	m_scsi_fifo[1] = make_unique_clear<uint32_t[]>(96);
	m_enet_fifo[ENET_RECV] = make_unique_clear<uint32_t[]>(32);
	m_enet_fifo[ENET_XMIT] = make_unique_clear<uint32_t[]>(40);

	save_pointer(NAME(m_pbus_fifo), 96);
	save_pointer(NAME(m_scsi_fifo[0]), 96);
	save_pointer(NAME(m_scsi_fifo[1]), 96);
	save_pointer(NAME(m_enet_fifo[ENET_RECV]), 32);
	save_pointer(NAME(m_enet_fifo[ENET_XMIT]), 40);

	m_enet_tx_timer = timer_alloc(FUNC(hpc3_device::enet_transmit), this);

}


void hpc3_device::device_reset()
{
	m_cpu_aux_ctrl = 0;

	std::fill(std::begin(m_scsi_dma), std::end(m_scsi_dma), scsi_dma_t());

	for (uint32_t i = 0; i < 8; i++)
	{
		m_pbus_dma[i].m_active = 0;
		m_pbus_dma[i].m_cur_ptr = 0;
		m_pbus_dma[i].m_desc_ptr = 0;
		m_pbus_dma[i].m_desc_flags = 0;
		m_pbus_dma[i].m_next_ptr = 0;
		m_pbus_dma[i].m_bytes_left = 0;
		m_pbus_dma[i].m_config = 0;
		m_pbus_dma[i].m_control = 0;

		m_pbus_dma[i].m_active = false;
		m_pbus_dma[i].m_timer->adjust(attotime::never);
	}

	m_intstat = 0;
	m_dma_complete_int_cb(0);

	m_enet_misc = MISC_RESET;
}

void hpc3_device::map(address_map &map)
{
	map(0x00000000, 0x0000ffff).rw(FUNC(hpc3_device::pbusdma_r), FUNC(hpc3_device::pbusdma_w));
	map(0x00010000, 0x0001ffff).rw(FUNC(hpc3_device::hd_enet_r), FUNC(hpc3_device::hd_enet_w));
	map(0x00020000, 0x000202ff).rw(FUNC(hpc3_device::fifo_r<FIFO_PBUS>), FUNC(hpc3_device::fifo_w<FIFO_PBUS>)); // PBUS FIFO
	map(0x00028000, 0x000282ff).rw(FUNC(hpc3_device::fifo_r<FIFO_SCSI0>), FUNC(hpc3_device::fifo_w<FIFO_SCSI0>)); // SCSI0 FIFO
	map(0x0002a000, 0x0002a2ff).rw(FUNC(hpc3_device::fifo_r<FIFO_SCSI1>), FUNC(hpc3_device::fifo_w<FIFO_SCSI1>)); // SCSI1 FIFO
	map(0x0002c000, 0x0002c0ff).rw(FUNC(hpc3_device::fifo_r<FIFO_ENET_RECV>), FUNC(hpc3_device::fifo_w<FIFO_ENET_RECV>)); // ENET Recv FIFO
	map(0x0002e000, 0x0002e13f).rw(FUNC(hpc3_device::fifo_r<FIFO_ENET_XMIT>), FUNC(hpc3_device::fifo_w<FIFO_ENET_XMIT>)); // ENET Xmit FIFO
	map(0x00030000, 0x00030003).r(FUNC(hpc3_device::intstat_r));
	map(0x00030004, 0x00030007).rw(FUNC(hpc3_device::misc_r), FUNC(hpc3_device::misc_w));
	map(0x00030008, 0x0003000b).rw(FUNC(hpc3_device::eeprom_r), FUNC(hpc3_device::eeprom_w));
	map(0x0003000c, 0x0003000f).r(FUNC(hpc3_device::intstat_r));
	map(0x00040000, 0x00047fff).rw(FUNC(hpc3_device::hd_r<0>), FUNC(hpc3_device::hd_w<0>));
	map(0x00048000, 0x0004ffff).rw(FUNC(hpc3_device::hd_r<1>), FUNC(hpc3_device::hd_w<1>));
	map(0x00054000, 0x000544ff).m(m_enet, FUNC(seeq80c03_device::map)).umask64(0x000000ff000000ff);
	map(0x00058000, 0x0005bfff).rw(FUNC(hpc3_device::pio_data_r), FUNC(hpc3_device::pio_data_w));
	map(0x0005c000, 0x0005cfff).rw(FUNC(hpc3_device::dma_config_r), FUNC(hpc3_device::dma_config_w));
	map(0x0005d000, 0x0005dfff).rw(FUNC(hpc3_device::pio_config_r), FUNC(hpc3_device::pio_config_w));
	map(0x00060000, 0x0007ffff).rw(FUNC(hpc3_device::bbram_r), FUNC(hpc3_device::bbram_w));
#ifdef MAME_SGI_HW_FIXES
	// Gap between the EEPROM/intstat cluster (ends 0x3000f) and the HD
	// registers (starts 0x40000); real hardware bus-errors here. Confirmed
	// hit during Indy boot ("Start System") immediately before a kernel
	// panic caused by the resulting false-success zero read - see
	// unmapped_r/w below.
	map(0x00030010, 0x0003ffff).rw(FUNC(hpc3_device::unmapped_r), FUNC(hpc3_device::unmapped_w));
#endif
}

#ifdef MAME_SGI_HW_FIXES
uint32_t hpc3_device::unmapped_r(offs_t offset, uint32_t mem_mask)
{
	LOGMASKED(LOG_UNKNOWN, "%s: unmapped HPC3 register read from %06x & %08x\n", machine().describe_context(), offset, mem_mask);
	if (m_mem_ctrl)
		m_mem_ctrl->set_cpu_buserr(0x1fb80000 + offset, mem_mask);
	return 0;
}

void hpc3_device::unmapped_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	LOGMASKED(LOG_UNKNOWN, "%s: unmapped HPC3 register write to %06x = %08x & %08x\n", machine().describe_context(), offset, data, mem_mask);
	if (m_mem_ctrl)
		m_mem_ctrl->set_cpu_buserr(0x1fb80000 + offset, mem_mask);
}
#endif

TIMER_CALLBACK_MEMBER(hpc3_device::do_pbus_dma)
{
	uint32_t channel = (uint32_t)param;
	pbus_dma_t &dma = m_pbus_dma[channel];

	if (dma.m_active && channel < 4)
	{
		// MAME_SGI_HAL2_SAMPLE_FORMAT_FIX: each 32-bit word in memory holds
		// one audio sample, 24-bit right-justified (HAL2's AES width), so a
		// codec's 16-bit sample is bits 23..8 - how SGI's own HAL2 tests build
		// their buffers (sample << 8). Taking bits 31..16 and swapping them
		// put the sample's high byte on top and its sign extension in the low
		// byte, losing the low 8 bits. Config bit 19 ("even address bytes on
		// 15..8") selects the other byte lane; IRIX leaves it clear.
		if (dma.m_control & PBUS_CTRL_RECV)
		{
			// MAME_SGI_HAL2_AUDIO_INPUT: receive (recording) DMA. The word is
			// sign-extended from bit 23, as software reads it as a signed int
			// (24-bit samples, SGI's HAL2 tests' buf[i] >> 8)
			const uint16_t sample = uint16_t(m_hal2->dma_read(channel));
			const uint16_t lane = BIT(dma.m_config, 19) ? swapendian_int16(sample) : sample;
			m_gio64_space->write_dword(dma.m_cur_ptr, uint32_t(int32_t(int16_t(lane)) * 256));
		}
		else
		{
			const uint16_t lane = uint16_t(m_gio64_space->read_dword(dma.m_cur_ptr) >> 8);
			m_hal2->dma_write(channel, int16_t(BIT(dma.m_config, 19) ? swapendian_int16(lane) : lane));
		}

		dma.m_cur_ptr += 4;
		dma.m_bytes_left -= 4;

		if (dma.m_bytes_left == 0)
		{
			if (BIT(dma.m_desc_flags, 29))
			{
				LOGMASKED(LOG_PBUS_DMA, "Raising channel %d IRQ\n", channel);
				m_intstat |= 1 << channel;
				m_dma_complete_int_cb(1);
			}
			if (!BIT(dma.m_desc_flags, 31))
			{
				dma.m_desc_ptr = dma.m_next_ptr;
				LOGMASKED(LOG_PBUS_DMA, "Channel %d Next PBUS_DMA_DescPtr = %08x\n", channel, dma.m_desc_ptr); fflush(stdout);
				dma.m_cur_ptr = m_gio64_space->read_dword(dma.m_desc_ptr);
				dma.m_desc_flags = m_gio64_space->read_dword(dma.m_desc_ptr + 4);
				dma.m_bytes_left = dma.m_desc_flags & 0x3fff;
				dma.m_next_ptr = m_gio64_space->read_dword(dma.m_desc_ptr + 8);
				LOGMASKED(LOG_PBUS_DMA, "Channel %d Next PBUS_DMA_CurPtr = %08x\n", channel, dma.m_cur_ptr); fflush(stdout);
				LOGMASKED(LOG_PBUS_DMA, "Channel %d Next PBUS_DMA_BytesLeft = %08x\n", channel, dma.m_bytes_left); fflush(stdout);
				LOGMASKED(LOG_PBUS_DMA, "Channel %d Next PBUS_DMA_NextPtr = %08x\n", channel, dma.m_next_ptr); fflush(stdout);
			}
			else
			{
				dma.m_active = false;
				dma.m_timer->adjust(attotime::never);
				return;
			}
		}
		// MAME_SGI_PBUS_DMA_DEGENERATE_RATE_REARM_FIX: pbusdma_w() only
		// starts a channel when get_rate() != attotime::zero (its own
		// m_codeca_clock/m_codecb_clock config defaults to 0, i.e. "not yet
		// configured" - see hal2_device::get_rate()), but this rearm path
		// re-queries the same rate on every sample without any guard at
		// all. Confirmed live 2026-08-14: a -state reload can resume a
		// channel mid-transfer at the exact instant PROM's HAL2
		// bresenham-clock-generator config (m_bres_clock_inc/_modctrl/_freq,
		// each written by a separate register write - see
		// hal2_device::update_clock_freq()) is only partway through being
		// written. A continuous boot never observes this - by the time the
		// DMA timer's next callback actually needs the rate, PROM has
		// already finished the whole sequence and get_rate() returns the
		// real audio period (~22.7us/44.1kHz, confirmed via a fresh-boot
		// comparison) - but a save taken squarely inside that multi-write
		// window freezes the inconsistent intermediate combination, which
		// can compute to an absurd near-zero (observed: 1 attosecond, the
		// engine's minimum representable nonzero attotime - not exactly
		// attotime::zero, so a `!= attotime::zero` guard alone doesn't
		// catch it). adjust() at that rate reschedules on essentially every
		// device_scheduler::timeslice() call, and since the transfer is a
		// multi-descriptor ring that doesn't set the "last descriptor" flag
		// (bit 31), it chains descriptor to descriptor without ever
		// stopping - a genuine host-side livelock (100% CPU stuck in
		// timeslice()/emu_timer::adjust(), MAME never returns to paint
		// another frame). No real audio hardware runs anywhere near this
		// fast, so treat anything below a generous floor (1 microsecond -
		// still over an order of magnitude faster than the highest sample
		// rate this hardware could plausibly run, ~192kHz/5.2us) as the
		// same "not actually configured yet" case pbusdma_w() already
		// guards against, and stop the channel instead of rearming.
		attotime rate = m_hal2->get_rate(channel);
		if (rate >= attotime::from_usec(1))
			dma.m_timer->adjust(rate, (int)channel);
		else
		{
			dma.m_active = false;
			dma.m_timer->adjust(attotime::never);
		}
	}
	else
	{
		dma.m_timer->adjust(attotime::never);
	}
}

uint32_t hpc3_device::hd_enet_r(offs_t offset, uint32_t mem_mask)
{
	switch (offset)
	{
	case 0x0000/4:
	case 0x2000/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Current Buffer Pointer Read: %08x & %08x\n", machine().describe_context(), channel, m_scsi_dma[channel].m_cbp, mem_mask);
		return m_scsi_dma[channel].m_cbp;
	}
	case 0x0004/4:
	case 0x2004/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Next Buffer Desc Pointer Read: %08x & %08x\n", machine().describe_context(), channel, m_scsi_dma[channel].m_nbdp, mem_mask);
		return m_scsi_dma[channel].m_nbdp;
	}
	case 0x1000/4:
	case 0x3000/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		const uint32_t ret = (m_scsi_dma[channel].m_count & 0x3fff) | (m_scsi_dma[channel].m_bc & 0xffffc000);
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Buffer Count Read: %08x & %08x\n", machine().describe_context(), channel, ret, mem_mask);
		return ret;
	}
	case 0x1004/4:
	case 0x3004/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		uint32_t ret = m_scsi_dma[channel].m_ctrl;
		if (BIT(m_intstat, channel + 8))
		{
			ret |= HPC3_DMACTRL_IRQ;
			if (!machine().side_effects_disabled())
			{
				LOGMASKED(LOG_SCSI_IRQ, "Lowering SCSI %d IRQ\n", channel);
				m_intstat &= ~(0x100 << channel);
				if (m_intstat == 0)
					m_dma_complete_int_cb(0);
			}
		}
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Control Read: %08x & %08x\n", machine().describe_context(), channel, ret, mem_mask);
		return ret;
	}
	case 0x1008/4:
	case 0x3008/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d GIO FIFO Pointer Read: %08x & %08x\n", machine().describe_context(), channel, 0, mem_mask);
		return 0;
	}
	case 0x100c/4:
	case 0x300c/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Device FIFO Pointer Read: %08x & %08x\n", machine().describe_context(), channel, 0, mem_mask);
		return 0;
	}
	case 0x1010/4:
	case 0x3010/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d DMA Config Read: %08x & %08x\n", machine().describe_context(), channel, m_scsi_dma[channel].m_dmacfg, mem_mask);
		return m_scsi_dma[channel].m_dmacfg;
	}
	case 0x1014/4:
	case 0x3014/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d PIO Config Read: %08x & %08x\n", machine().describe_context(), channel, m_scsi_dma[channel].m_piocfg, mem_mask);
		return m_scsi_dma[channel].m_piocfg;
	}
	case 0x4000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Current Buffer Pointer Read: %08x\n", machine().describe_context(), m_enet_rx_cbp);
		return m_enet_rx_cbp;
	case 0x4004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Next Buffer Desc Pointer Read: %08x\n", machine().describe_context(), m_enet_rx_nbdp);
		return m_enet_rx_nbdp;
	case 0x5000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Buffer Count Read: %08x\n", machine().describe_context(), m_enet_rx_bc);
		return m_enet_rx_bc;
	case 0x5004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver DMA Control Read: %08x\n", machine().describe_context(), m_enet_rx_ctrl);
		return m_enet_rx_ctrl;
	case 0x5008/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver GIO FIFO Pointer Read: %08x\n", machine().describe_context(), m_enet_rx_gio);
		return m_enet_rx_gio;
	case 0x500c/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Device FIFO Pointer Read: %08x\n", machine().describe_context(), m_enet_rx_dev);
		return m_enet_rx_dev;
	case 0x5014/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Reset Register Read: %08x\n", machine().describe_context(), m_enet_misc);
		return m_enet_misc;
	case 0x5018/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet DMA Config Read: %08x\n", machine().describe_context(), m_enet_dmacfg);
		return m_enet_dmacfg;
	case 0x501c/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet PIO Config Read: %08x\n", machine().describe_context(), m_enet_piocfg);
		return m_enet_piocfg;
	case 0x6000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Current Buffer Pointer Read: %08x\n", machine().describe_context(), m_enet_tx_cbp);
		return m_enet_tx_cbp;
	case 0x6004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Next Buffer Desc Pointer Read: %08x\n", machine().describe_context(), m_enet_tx_nbdp);
		return m_enet_tx_nbdp;
	case 0x7000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Buffer Count Read: %08x\n", machine().describe_context(), m_enet_tx_bc);
		return m_enet_tx_bc;
	case 0x7004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter DMA Control Read: %08x\n", machine().describe_context(), m_enet_tx_ctrl);
		return m_enet_tx_ctrl;
	case 0x7008/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter GIO FIFO Pointer Read: %08x\n", machine().describe_context(), m_enet_tx_gio);
		return m_enet_tx_gio;
	case 0x700c/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Device FIFO Pointer Read: %08x\n", machine().describe_context(), m_enet_tx_dev);
		return m_enet_tx_dev;
	case 0x8000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Current Buffer Descriptor Pointer Read: %08x\n", machine().describe_context(), m_enet_rx_cbdp);
		return m_enet_rx_cbdp;
	case 0xa000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Current/Previous First Buffer Descriptor Pointer Read: %08x\n", machine().describe_context(), m_enet_tx_cpfbdp);
		return m_enet_tx_cpfbdp;
	case 0xa004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Previous/Previous First Buffer Descriptor Pointer Read: %08x\n", machine().describe_context(), m_enet_tx_ppfbdp);
		return m_enet_tx_ppfbdp;
	default:
		LOGMASKED(LOG_UNKNOWN, "%s: Unknown HPC3 ENET/HDx Read: %08x & %08x\n", machine().describe_context(), 0x1fb90000 + (offset << 2), mem_mask);
		return 0;
	}
}

void hpc3_device::hd_enet_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	switch (offset)
	{
	case 0x0004/4:
	case 0x2004/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Next Buffer Desc Pointer Write: %08x\n", machine().describe_context(), channel, data);
		m_scsi_dma[channel].m_nbdp = data;
		break;
	}
	case 0x1000/4:
	case 0x3000/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d Buffer Count Write: %08x\n", machine().describe_context(), channel, data);
		m_scsi_dma[channel].m_bc = data;
		break;
	}
	case 0x1004/4:
	case 0x3004/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d DMA Control Write: %08x\n", machine().describe_context(), channel, data);
		const bool was_active = m_scsi_dma[channel].m_active;
		if (data & HPC3_DMACTRL_WRMASK)
		{
			m_scsi_dma[channel].m_ctrl = data & ~HPC3_DMACTRL_IRQ & ~HPC3_DMACTRL_ENABLE & ~HPC3_DMACTRL_WRMASK;
			if (was_active)
				m_scsi_dma[channel].m_ctrl |= HPC3_DMACTRL_ENABLE;
		}
		else
		{
			m_scsi_dma[channel].m_ctrl = data & ~HPC3_DMACTRL_IRQ & ~HPC3_DMACTRL_WRMASK;
			m_scsi_dma[channel].m_active = (m_scsi_dma[channel].m_ctrl & HPC3_DMACTRL_ENABLE);
		}
		m_scsi_dma[channel].m_to_device = (m_scsi_dma[channel].m_ctrl & HPC3_DMACTRL_DIR);
		m_scsi_dma[channel].m_big_endian = (m_scsi_dma[channel].m_ctrl & HPC3_DMACTRL_ENDIAN);
		if (!was_active && m_scsi_dma[channel].m_active)
		{
			fetch_chain(channel);
		}
		m_hd_reset_cb[channel](BIT(data, 6));
		if (BIT(data, 3))
		{
			scsi_fifo_flush(channel);
		}
		if (m_scsi_dma[channel].m_drq && m_scsi_dma[channel].m_active)
		{
			do_scsi_dma(channel);
		}
		break;
	}
	case 0x1010/4:
	case 0x3010/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d DMA Config Write: %08x\n", machine().describe_context(), channel, data);
		m_scsi_dma[channel].m_dmacfg = data;
		break;
	}
	case 0x1014/4:
	case 0x3014/4:
	{
		const uint32_t channel = (offset & 0x2000/4) ? 1 : 0;
		LOGMASKED(LOG_SCSI, "%s: HPC3 SCSI%d PIO Config Write: %08x\n", machine().describe_context(), channel, data);
		m_scsi_dma[channel].m_piocfg = data;
		break;
	}
	case 0x4000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Current Buffer Pointer Write: %08x\n", machine().describe_context(), data);
		m_enet_rx_cbp = data;
		break;
	case 0x4004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Next Buffer Desc Pointer Write: %08x\n", machine().describe_context(), data);
		m_enet_rx_nbdp = data;
		break;
	case 0x5000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Buffer Count Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x5004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver DMA Control Write: %08x\n", machine().describe_context(), data);
		if (m_enet_rx_ctrl & RXC_CAM)
			m_enet_rx_ctrl = (m_enet_rx_ctrl & (RXC_LC | RXC_ST)) | (data & ~(RXC_CA | RXC_LC | RXC_ST));
		else
			m_enet_rx_ctrl = (m_enet_rx_ctrl & (RXC_LC | RXC_ST)) | (data & ~((RXC_LC | RXC_ST)));

		// if a frame is already waiting in the edlc, transfer it now
		if ((m_enet_rx_ctrl & RXC_CA) && m_enet_rx_ready)
			enet_receive();
		break;
	case 0x5008/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver GIO FIFO Pointer Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x500c/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Device FIFO Pointer Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x5014/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Reset Register Write: %08x\n", machine().describe_context(), data);
		enet_misc_w(data);
		break;
	case 0x5018/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet DMA Config Write: %08x\n", machine().describe_context(), data);
		m_enet_dmacfg = data;
		break;
	case 0x501c/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet PIO Config Write: %08x\n", machine().describe_context(), data);
		m_enet_piocfg = data;
		break;
	case 0x6000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Current Buffer Pointer Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x6004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Next Buffer Desc Pointer Write: %08x\n", machine().describe_context(), data);
		m_enet_tx_nbdp = data;
		break;
	case 0x7000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Buffer Count Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x7004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter DMA Control Write: %08x\n", machine().describe_context(), data);
		if (m_enet_tx_ctrl & RXC_CAM)
			m_enet_tx_ctrl = (m_enet_tx_ctrl & (TXC_LC | TXC_ST)) | (data & ~(TXC_CA | TXC_LC | TXC_ST));
		else
			m_enet_tx_ctrl = (m_enet_tx_ctrl & (TXC_LC | TXC_ST)) | (data & ~((TXC_LC | TXC_ST)));

		// a software write starts a (new) chain, superseding any pending
		// end-of-chain stop from the packet still being transmitted
		if (m_enet_tx_ctrl & TXC_CA)
			m_enet_tx_eox = false;

		// only start the transmit dma if the edlc is ready for a frame,
		// otherwise enet_txrdy_w() starts it once it is
		if ((m_enet_tx_ctrl & TXC_CA) && m_enet_tx_ready && !m_enet_tx_timer->enabled())
			m_enet_tx_timer->adjust(attotime::zero);
		break;
	case 0x7008/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter GIO FIFO Pointer Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x700c/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Device FIFO Pointer Write (ignored): %08x\n", machine().describe_context(), data);
		break;
	case 0x8000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Receiver Current Buffer Descriptor Pointer Write: %08x\n", machine().describe_context(), data);
		m_enet_rx_cbdp = data;
		break;
	case 0xa000/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Current/Previous Buffer Descriptor Pointer Write: %08x\n", machine().describe_context(), data);
		m_enet_tx_cpfbdp = data;
		break;
	case 0xa004/4:
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet Transmitter Previous/Previous Buffer Descriptor Pointer Write: %08x\n", machine().describe_context(), data);
		m_enet_tx_ppfbdp = data;
		break;

	default:
		LOGMASKED(LOG_UNKNOWN, "%s: Unknown HPC3 ENET/HDx write: %08x = %08x & %08x\n", machine().describe_context(), 0x1fb90000 + (offset << 2), data, mem_mask);
		break;
	}
}

void hpc3_device::enet_rxrdy_w(int state)
{
	// the edlc's rxrdy output: asserted while its receive fifo holds a frame
	m_enet_rx_ready = bool(state);

	// check receive dma enabled
	if (state && (m_enet_rx_ctrl & RXC_CA))
		enet_receive();
}

void hpc3_device::enet_receive()
{
	// next descriptor becomes current
	m_enet_rx_cbdp = m_enet_rx_nbdp;

	// fetch the current descriptor
	m_enet_rx_cbp = m_gio64_space->read_dword(m_enet_rx_cbdp + 0);
	m_enet_rx_bc = m_gio64_space->read_dword(m_enet_rx_cbdp + 4);
	m_enet_rx_nbdp = m_gio64_space->read_dword(m_enet_rx_cbdp + 8);

	LOGMASKED(LOG_ETHERNET, "enet rx dma chain 0x%08x cbp 0x%08x bc 0x%08x nbdp 0x%08x\n",
		m_enet_rx_cbdp, m_enet_rx_cbp, m_enet_rx_bc, m_enet_rx_nbdp);

	// skip buffer alignment bytes
	if (enet_rx_bc_dec(2))
		m_enet_rx_cbp += 2;

	// transfer data from edlc fifo to memory; bytes beyond the end of the
	// buffer are drained and discarded (enet_rx_bc_dec() flags the
	// overflow), otherwise the fifo never empties and this never ends
	while (!m_enet->rxeof_r())
	{
		u8 const data = m_enet->fifo_r();
		if (enet_rx_bc_dec())
			m_gio64_space->write_byte(m_enet_rx_cbp++, data);
	}
}

void hpc3_device::enet_txrdy_w(int state)
{
	// the edlc's txrdy output: asserted when its transmit fifo can accept
	// a frame, i.e. after any previous frame has finished transmitting
	m_enet_tx_ready = bool(state);

	if (m_enet_tx_ready && (m_enet_tx_ctrl & TXC_CA) && !m_enet_tx_timer->enabled())
		m_enet_tx_timer->adjust(attotime::zero);
}

void hpc3_device::enet_intr_in_w(int state)
{
	if (state)
	{
		bool interrupt = false;

		// copy edlc status registers
		m_enet_rx_ctrl &= ~RXC_ST;
		m_enet_rx_ctrl |= m_enet->read(6) & RXC_ST;
		m_enet_tx_ctrl &= ~TXC_ST;
		m_enet_tx_ctrl |= m_enet->read(7) & TXC_ST;

		LOGMASKED(LOG_ETHERNET, "rx status 0x%02x tx status 0x%02x\n",
			u8(m_enet_rx_ctrl), u8(m_enet_tx_ctrl));

		// tx interrupt
		if (!(m_enet_tx_ctrl & TXC_ST_O))
		{
			// write txd and clear byte count
			if (m_enet_tx_ctrl & TXC_ST_S)
				m_gio64_space->write_word(m_enet_tx_cpfbdp + 6, BC_TXD);

			// interrupt host if xie or error
			if (m_enet_tx_bc & BC_XIE)
				interrupt = true;

			// the channel stays active until the packet ending the chain
			// has actually been transmitted (or on error)
			if (m_enet_tx_eox || (m_enet_tx_ctrl & (TXC_ST_U | TXC_ST_C | TXC_ST_R)))
			{
				m_enet_tx_eox = false;
				m_enet_tx_ctrl &= ~TXC_CA;
			}
			if (m_enet_tx_ctrl & (TXC_ST_U | TXC_ST_C | TXC_ST_R))
				interrupt = true;

			// transmit next packet (otherwise enet_txrdy_w() starts it
			// once the edlc is ready to accept it)
			if ((m_enet_tx_ctrl & TXC_CA) && m_enet_tx_ready && !m_enet_tx_timer->enabled())
				m_enet_tx_timer->adjust(attotime::zero);
		}

		// rx interrupt
		if (!(m_enet_rx_ctrl & RXC_ST_O))
		{
			// transfer the status byte
			if (enet_rx_bc_dec())
				m_gio64_space->write_byte(m_enet_rx_cbp++, u8(m_enet_rx_ctrl));

			// store the remaining buffer length
			m_gio64_space->write_word(m_enet_rx_cbdp + 6, u16(m_enet_rx_bc & BC_BC));

			// check for edlc overflow, receive buffer overflow or end of descriptor chain
			if ((m_enet_rx_ctrl & (RXC_ST_V | RXC_RBO)) || (m_enet_rx_bc & BC_EOX))
			{
				m_enet_rx_ctrl &= ~RXC_CA;
				interrupt = true;
			}
			else if (m_enet_rx_bc & BC_XIE)
				interrupt = true;
		}

		if (interrupt && !BIT(m_enet_misc, 1))
		{
			m_enet_misc |= MISC_INT;
			m_enet_intr_out_cb(interrupt);
		}
	}
}

// The FIFO RAMs as the CPU sees them: an entry is two words, upper then
// lower, as SGI's diagnostics write and read them back. Entries are read
// at an 8-byte stride (storage word = window word); the PBUS and Ethernet
// FIFOs are written at a 16-byte stride, the SCSI FIFOs at 8 bytes. Words
// past the end of a FIFO read as 0 and ignore writes.
template<hpc3_device::fifo_type_t Type>
uint32_t *hpc3_device::fifo_word(offs_t offset)
{
	uint32_t *base = nullptr;
	offs_t words = 0;
	if (Type == FIFO_PBUS)
		base = m_pbus_fifo.get(), words = 96;
	else if (Type == FIFO_SCSI0)
		base = m_scsi_fifo[0].get(), words = 96;
	else if (Type == FIFO_SCSI1)
		base = m_scsi_fifo[1].get(), words = 96;
	else if (Type == FIFO_ENET_RECV)
		base = m_enet_fifo[ENET_RECV].get(), words = 32;
	else if (Type == FIFO_ENET_XMIT)
		base = m_enet_fifo[ENET_XMIT].get(), words = 40;
	return (offset < words) ? &base[offset] : nullptr;
}

template<hpc3_device::fifo_type_t Type>
uint32_t hpc3_device::fifo_r(offs_t offset)
{
	uint32_t *const word = fifo_word<Type>(offset);
	const uint32_t ret = word ? *word : 0;
	logerror("Reading %08x from %d FIFO offset %08x\n", ret, Type, offset);
	return ret;
}

template<hpc3_device::fifo_type_t Type>
void hpc3_device::fifo_w(offs_t offset, uint32_t data)
{
	logerror("Writing %08x to %d FIFO offset %08x\n", data, Type, offset);
	const bool wide = Type == FIFO_PBUS || Type == FIFO_ENET_RECV || Type == FIFO_ENET_XMIT;
	if (uint32_t *const word = fifo_word<Type>(wide ? (((offset >> 2) << 1) | (offset & 1)) : offset))
		*word = data;
}

template uint32_t hpc3_device::fifo_r<hpc3_device::FIFO_PBUS>(offs_t offset);
template uint32_t hpc3_device::fifo_r<hpc3_device::FIFO_SCSI0>(offs_t offset);
template uint32_t hpc3_device::fifo_r<hpc3_device::FIFO_SCSI1>(offs_t offset);
template uint32_t hpc3_device::fifo_r<hpc3_device::FIFO_ENET_RECV>(offs_t offset);
template uint32_t hpc3_device::fifo_r<hpc3_device::FIFO_ENET_XMIT>(offs_t offset);
template void hpc3_device::fifo_w<hpc3_device::FIFO_PBUS>(offs_t offset, uint32_t data);
template void hpc3_device::fifo_w<hpc3_device::FIFO_SCSI0>(offs_t offset, uint32_t data);
template void hpc3_device::fifo_w<hpc3_device::FIFO_SCSI1>(offs_t offset, uint32_t data);
template void hpc3_device::fifo_w<hpc3_device::FIFO_ENET_RECV>(offs_t offset, uint32_t data);
template void hpc3_device::fifo_w<hpc3_device::FIFO_ENET_XMIT>(offs_t offset, uint32_t data);

template<uint32_t index>
uint32_t hpc3_device::hd_r(offs_t offset, uint32_t mem_mask)
{
	if (ACCESSING_BITS_0_7 && !m_hd_rd_cb[index].isunset())
	{
		const uint8_t ret = m_hd_rd_cb[index](offset);
		LOGMASKED(LOG_SCSI, "%s: SCSI%d Read %02x: %02x\n", machine().describe_context(), index, offset, ret);
		return ret;
	}
	else
	{
		LOGMASKED(LOG_SCSI | LOG_UNKNOWN, "%s: Unknown HPC3 HD%d Read: %08x & %08x\n", machine().describe_context(),
			index, 0x1fbc4000 + (offset << 2) + index * 0x8000, mem_mask);
		return 0;
	}
}

template<uint32_t index>
void hpc3_device::hd_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	if (ACCESSING_BITS_0_7 && !m_hd_wr_cb[index].isunset())
	{
		LOGMASKED(LOG_SCSI, "%s: SCSI%d Write %02x = %02x\n", machine().describe_context(), index, offset, (uint8_t)data);
		m_hd_wr_cb[index](offset, data & 0xff);
	}
	else
	{
		LOGMASKED(LOG_SCSI | LOG_UNKNOWN, "%s: Unknown HPC3 HD%d Write: %08x = %08x & %08x\n", machine().describe_context(),
			index, 0x1fbc4000 + (offset << 2) + index * 0x8000, data, mem_mask);
	}
}

template uint32_t hpc3_device::hd_r<0>(offs_t offset, uint32_t mem_mask);
template uint32_t hpc3_device::hd_r<1>(offs_t offset, uint32_t mem_mask);
template void hpc3_device::hd_w<0>(offs_t offset, uint32_t data, uint32_t mem_mask);
template void hpc3_device::hd_w<1>(offs_t offset, uint32_t data, uint32_t mem_mask);

uint32_t hpc3_device::pio_data_r(offs_t offset)
{
	uint32_t channel = (offset >> 8) & 15;
	if (channel >= 10)
	{
		channel = (channel & 1) ? 9 : 8;
	}

	switch ((m_pio_config[channel] >> 18) & 3)
	{
	default:
	case 0: // 8-bit, data on PBUS 7:0
		return m_pio_space[channel]->read_word(offset & 0xff, 0x00ff) & 0xff;

	case 2: // 8-bit, data on PBUS 15:8
		return m_pio_space[channel]->read_word(offset & 0xff, 0xff00) >> 8;

	case 1: // 16-bit, odd high
	case 3: // 16-bit, even high
		return m_pio_space[channel]->read_word(offset & 0xff, 0xffff);
	}
}

void hpc3_device::pio_data_w(offs_t offset, uint32_t data)
{
	uint32_t channel = (offset >> 8) & 15;
	if (channel >= 10)
	{
		channel = (channel & 1) ? 9 : 8;
	}

	switch ((m_pio_config[channel] >> 18) & 3)
	{
	case 0: // 8-bit, data on PBUS 7:0
		m_pio_space[channel]->write_word(offset & 0xff, data & 0xffff, 0x00ff);
		break;

	case 2: // 8-bit, data on PBUS 15:8
		m_pio_space[channel]->write_word(offset & 0xff, swapendian_int16(data & 0xffff), 0xff00);
		break;

	case 1: // 16-bit, odd high
	case 3: // 16-bit, even high
		m_pio_space[channel]->write_word(offset & 0xff, data & 0xffff, 0xffff);
		break;
	}
}

uint32_t hpc3_device::pbusdma_r(offs_t offset, uint32_t mem_mask)
{
	uint32_t channel = offset / (0x2000/4);
	pbus_dma_t &dma = m_pbus_dma[channel];

	uint32_t ret = 0;
	switch (offset & 0x07ff)
	{
	case 0x0000/4:
		ret = dma.m_cur_ptr;
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS DMA Channel %d Buffer Pointer Read: %08x & %08x\n", machine().describe_context(), channel, ret, mem_mask);
		break;
	case 0x0004/4:
		ret = dma.m_desc_ptr;
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS DMA Channel %d Descriptor Pointer Read: %08x & %08x\n", machine().describe_context(), channel, ret, mem_mask);
		break;
	case 0x1000/4:
		ret = (dma.m_timer->remaining() != attotime::never) ? 2 : 0;
		if (BIT(m_intstat, channel))
		{
			ret |= 1;
			if (!machine().side_effects_disabled())
			{
				LOGMASKED(LOG_PBUS_DMA, "Lowering channel %d IRQ\n", channel);
				m_intstat &= ~(1 << channel);
				if (m_intstat == 0)
					m_dma_complete_int_cb(0);
			}
		}
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS DMA Channel %d Control Read: %08x & %08x\n", machine().describe_context(), channel, ret, mem_mask);
		break;
	default:
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS DMA Channel %d Unknown Read: %08x & %08x\n", machine().describe_context(), channel, 0x1fb80000 + (offset << 2), mem_mask);
		break;
	}
	return ret;
}

void hpc3_device::pbusdma_w(address_space &space, offs_t offset, uint32_t data, uint32_t mem_mask)
{
	uint32_t channel = offset / (0x2000/4);
	pbus_dma_t &dma = m_pbus_dma[channel];

	switch (offset & 0x07ff)
	{
	case 0x0004/4:
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS DMA Channel %d Descriptor Pointer Write: %08x\n", machine().describe_context(), channel, data);
		dma.m_desc_ptr = data;
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS_DMA_DescPtr = %08x\n", machine().describe_context(), dma.m_desc_ptr);
		dma.m_cur_ptr = space.read_dword(dma.m_desc_ptr);
		dma.m_desc_flags = space.read_dword(dma.m_desc_ptr + 4);
		dma.m_next_ptr = space.read_dword(dma.m_desc_ptr + 8);
		dma.m_bytes_left = dma.m_desc_flags & 0x3fff;
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS_DMA_CurPtr = %08x\n", machine().describe_context(), dma.m_cur_ptr);
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS_DMA_BytesLeft = %08x\n", machine().describe_context(), dma.m_bytes_left);
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS_DMA_NextPtr = %08x\n", machine().describe_context(), dma.m_next_ptr);
		break;
	case 0x1000/4:
		LOGMASKED(LOG_PBUS_DMA, "%s: PBUS DMA Channel %d Control Register Write: %08x\n", machine().describe_context(), channel, data);
		dma.m_control = data;   // MAME_SGI_HAL2_AUDIO_INPUT: direction for do_pbus_dma()
		if (data & PBUS_CTRL_ENDIAN)
			LOGMASKED(LOG_PBUS_DMA, "    Little Endian\n");
		else
			LOGMASKED(LOG_PBUS_DMA, "    Big Endian\n");

		if (data & PBUS_CTRL_RECV)
			LOGMASKED(LOG_PBUS_DMA, "    RX DMA\n");
		else
			LOGMASKED(LOG_PBUS_DMA, "    TX DMA\n");

		if (data & PBUS_CTRL_FLUSH)
			LOGMASKED(LOG_PBUS_DMA, "    Flush for RX\n");
		if (data & PBUS_CTRL_DMASTART)
			LOGMASKED(LOG_PBUS_DMA, "    Start DMA\n");

		if (data & PBUS_CTRL_LOAD_EN)
			LOGMASKED(LOG_PBUS_DMA, "    Load Enable\n");

		LOGMASKED(LOG_PBUS_DMA, "    High Water Mark: %04x bytes\n", (data & PBUS_CTRL_HIGHWATER) >> 8);
		LOGMASKED(LOG_PBUS_DMA, "    FIFO Begin: Row %04x\n", (data & PBUS_CTRL_FIFO_BEG) >> 16);
		LOGMASKED(LOG_PBUS_DMA, "    FIFO End: Row %04x\n", (data & PBUS_CTRL_FIFO_END) >> 24);

		if (((data & PBUS_CTRL_DMASTART) && (data & PBUS_CTRL_LOAD_EN)) && channel < 4)
		{
			LOGMASKED(LOG_PBUS_DMA, "    Starting DMA\n");
			attotime rate = m_hal2->get_rate(channel);
			if (rate != attotime::zero)
			{
				dma.m_timer->adjust(rate, (int)channel);
				dma.m_active = true;
			}
		}
		break;
	default:
		LOGMASKED(LOG_PBUS_DMA | LOG_UNKNOWN, "%s: Unknown PBUS DMA Channel %d Write: %08x = %08x & %08x\n", machine().describe_context(), channel, 0x1fb80000 + offset*4, data, mem_mask);
		break;
	}
}

uint32_t hpc3_device::dma_config_r(offs_t offset, uint32_t mem_mask)
{
	const uint32_t channel = (offset >> 7) & 7;
	const uint32_t data = m_pbus_dma[channel].m_config;
	LOGMASKED(LOG_PBUS_DMA, "%s: Read Channel %d DMA Configuration: %08x & %08x\n", machine().describe_context(), channel, data, mem_mask);
	return data;
}

void hpc3_device::dma_config_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	const uint32_t channel = (offset >> 7) & 7;
	COMBINE_DATA(&m_pbus_dma[channel].m_config);

	LOGMASKED(LOG_PBUS_DMA, "%s: Write Channel %d DMA Configuration: %08x & %08x\n", machine().describe_context(), channel, data, mem_mask);
	LOGMASKED(LOG_PBUS_DMA, "    DMA Read State D3 gio_clk cycles: %d\n", BIT(data, 0) ? 2 : 3);
	LOGMASKED(LOG_PBUS_DMA, "    DMA Read State D4 gio_clk cycles: %d\n", (data >> 1) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    DMA Read State D5 gio_clk cycles: %d\n", (data >> 5) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    DMA Write State D3 gio_clk cycles: %d\n", BIT(data, 9) ? 2 : 3);
	LOGMASKED(LOG_PBUS_DMA, "    DMA Write State D4 gio_clk cycles: %d\n", (data >> 10) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    DMA Write State D5 gio_clk cycles: %d\n", (data >> 14) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    Device Bit Width: %d\n", BIT(data, 18) ? 16 : 8);
	LOGMASKED(LOG_PBUS_DMA, "    Even Address Bytes on %s\n", BIT(data, 19) ? "15..8" : "7..0");
	LOGMASKED(LOG_PBUS_DMA, "    Device %s Real-Time\n", BIT(data, 21) ? "is" : "is not");
	LOGMASKED(LOG_PBUS_DMA, "    Burst Count: %d\n", (data >> 22) & 0x1f);
	LOGMASKED(LOG_PBUS_DMA, "    %sUse Unsynchronized DREQ\n", BIT(data, 27) ? "" : "Do Not ");
}

uint32_t hpc3_device::pio_config_r(offs_t offset, uint32_t mem_mask)
{
	uint32_t channel = (offset >> 6) & 15;
	if (channel >= 10)
	{
		channel = (channel & 1) ? 9 : 8;
	}

	const uint32_t data = m_pio_config[channel];
	LOGMASKED(LOG_PBUS_DMA, "%s: Read Channel %d PIO Configuration: %08x & %08x\n", machine().describe_context(), channel, data, mem_mask);
	return data;
}

void hpc3_device::pio_config_w(offs_t offset, uint32_t data, uint32_t mem_mask)
{
	uint32_t channel = (offset >> 6) & 15;
	if (channel >= 10)
	{
		channel = (channel & 1) ? 9 : 8;
	}

	COMBINE_DATA(&m_pio_config[channel]);
	LOGMASKED(LOG_PBUS_DMA, "%s: Write Channel %d PIO Configuration: %08x & %08x\n", machine().describe_context(), channel, data, mem_mask);
	LOGMASKED(LOG_PBUS_DMA, "    PIO Read State P2 gio_clk cycles: %d\n", BIT(data, 0) ? 1 : 2);
	LOGMASKED(LOG_PBUS_DMA, "    PIO Read State P3 gio_clk cycles: %d\n", (data >> 1) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    PIO Read State P4 gio_clk cycles: %d\n", (data >> 5) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    PIO Write State P2 gio_clk cycles: %d\n", BIT(data, 9) ? 1 : 2);
	LOGMASKED(LOG_PBUS_DMA, "    PIO Write State P3 gio_clk cycles: %d\n", (data >> 10) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    PIO Write State P4 gio_clk cycles: %d\n", (data >> 14) & 0xf);
	LOGMASKED(LOG_PBUS_DMA, "    Device Bit Width: %d\n", BIT(data, 18) ? 16 : 8);
	LOGMASKED(LOG_PBUS_DMA, "    Even Address Bytes on %s\n", BIT(data, 19) ? "15..8" : "7..0");
}

uint32_t hpc3_device::bbram_r(offs_t offset)
{
	return m_bbram_rd_cb(offset);
}

void hpc3_device::bbram_w(offs_t offset, uint32_t data)
{
	m_bbram_wr_cb(offset, data);
}

void hpc3_device::dump_chain(uint32_t base)
{
	const uint32_t addr = m_gio64_space->read_dword(base);
	const uint32_t ctrl = m_gio64_space->read_dword(base+4);
	const uint32_t next = m_gio64_space->read_dword(base+8);

	LOGMASKED(LOG_CHAIN, "Chain Node:\n");
	LOGMASKED(LOG_CHAIN, "    Addr: %08x\n", addr);
	LOGMASKED(LOG_CHAIN, "    Ctrl: %08x\n", ctrl);
	LOGMASKED(LOG_CHAIN, "    Next: %08x\n", next);

	if (next != 0 && !BIT(ctrl, 31))
	{
		dump_chain(next);
	}
}

void hpc3_device::fetch_chain(int channel)
{
	scsi_dma_t &dma = m_scsi_dma[channel];
	const uint32_t desc_addr = dma.m_nbdp;
	dma.m_cbp = m_gio64_space->read_dword(desc_addr);
	dma.m_bc = m_gio64_space->read_dword(desc_addr+4);
	dma.m_nbdp = m_gio64_space->read_dword(desc_addr+8);
	dma.m_count = dma.m_bc & 0x3fff;

	LOGMASKED(LOG_CHAIN, "Fetching chain from %08x:\n", desc_addr);
	LOGMASKED(LOG_CHAIN, "    Addr: %08x\n", dma.m_cbp);
	LOGMASKED(LOG_CHAIN, "    Ctrl: %08x\n", dma.m_bc);
	LOGMASKED(LOG_CHAIN, "    Next: %08x\n", dma.m_nbdp);
}

void hpc3_device::decrement_chain(int channel)
{
	scsi_dma_t &dma = m_scsi_dma[channel];
	dma.m_count--;
	if (dma.m_count == 0)
		scsi_descriptor_done(channel);
}

// the current descriptor's byte count has run out
void hpc3_device::scsi_descriptor_done(int channel)
{
	scsi_dma_t &dma = m_scsi_dma[channel];
	if (BIT(dma.m_bc, 29))
	{
		LOGMASKED(LOG_SCSI_IRQ, "Raising SCSI %d IRQ\n", channel);
		m_intstat |= 0x100 << channel;
		m_dma_complete_int_cb(1);
	}
	if (BIT(dma.m_bc, 31))
	{
		dma.m_active = false;
		dma.m_ctrl &= ~HPC3_DMACTRL_ENABLE;
		return;
	}
	fetch_chain(channel);
}

// Runs of DMA bytes, with the same result as single-byte DMA cycles: byte i
// of a run starting at addr goes to BYTE4_XOR_BE/LE(addr + i). Within an
// aligned 8-byte group at a that is a + (i ^ x), with x the XOR's constant
// (0 or 3), so the group's big-endian qword is the 8 bytes in order (x = 0)
// or each 4-byte half reversed (x = 3). Aligned groups in RAM go as one
// qword; the rest, and any run not in RAM (a qword access to a device
// would not be the same as 8 byte accesses), byte by byte. A run lies
// within one descriptor, so its start decides.
namespace {
inline uint64_t dma_qword_from_bytes(const uint8_t *b, bool swap4)
{
	return swap4 ? (uint64_t(get_u32le(b)) << 32) | get_u32le(b + 4) : get_u64be(b);
}
inline void dma_bytes_from_qword(uint8_t *b, uint64_t q, bool swap4)
{
	if (swap4)
	{
		put_u32le(b, uint32_t(q >> 32));
		put_u32le(b + 4, uint32_t(q));
	}
	else
		put_u64be(b, q);
}
} // anonymous namespace

void hpc3_device::dma_write_run(uint32_t addr, const uint8_t *src, uint32_t count, bool big_endian)
{
	const uint32_t x = big_endian ? BYTE4_XOR_BE(0) : BYTE4_XOR_LE(0);
	if (!m_gio64_cached)
	{
		for (uint32_t i = 0; i < count; i++)
			m_gio64_space->write_byte((addr + i) ^ x, src[i]);
		return;
	}
	uint32_t i = 0;
	if (count >= 16 && m_gio64_cache.read_ptr(addr))
	{
		for (; (addr + i) & 7; i++)
			m_gio64_cache.write_byte((addr + i) ^ x, src[i]);
		for (; i + 8 <= count; i += 8)
			m_gio64_cache.write_qword(addr + i, dma_qword_from_bytes(src + i, x != 0));
	}
	for (; i < count; i++)
		m_gio64_cache.write_byte((addr + i) ^ x, src[i]);
}

void hpc3_device::dma_read_run(uint32_t addr, uint8_t *dst, uint32_t count, bool big_endian)
{
	const uint32_t x = big_endian ? BYTE4_XOR_BE(0) : BYTE4_XOR_LE(0);
	if (!m_gio64_cached)
	{
		for (uint32_t i = 0; i < count; i++)
			dst[i] = m_gio64_space->read_byte((addr + i) ^ x);
		return;
	}
	uint32_t i = 0;
	if (count >= 16 && m_gio64_cache.read_ptr(addr))
	{
		for (; (addr + i) & 7; i++)
			dst[i] = m_gio64_cache.read_byte((addr + i) ^ x);
		for (; i + 8 <= count; i += 8)
			dma_bytes_from_qword(dst + i, m_gio64_cache.read_qword(addr + i), x != 0);
	}
	for (; i < count; i++)
		dst[i] = m_gio64_cache.read_byte((addr + i) ^ x);
}

uint32_t hpc3_device::scsi_dma_bulk_to_device(int channel, uint8_t *dst, uint32_t count)
{
	scsi_dma_t &dma = m_scsi_dma[channel];
	uint32_t done = 0;
	while (done < count && dma.m_active && dma.m_to_device)
	{
		// as in scsi_dma_bulk_from_device()
		const uint32_t left = dma.m_count ? dma.m_count : 0x10000;
		const uint32_t run = std::min(left, count - done);
		dma_read_run(dma.m_cbp, dst + done, run, dma.m_big_endian);
		dma.m_cbp += run;
		dma.m_count -= run;
		done += run;
		if (dma.m_count == 0)
			scsi_descriptor_done(channel);
	}
	return done;
}

uint32_t hpc3_device::scsi_dma_bulk_from_device(int channel, const uint8_t *src, uint32_t count)
{
	scsi_dma_t &dma = m_scsi_dma[channel];
	uint32_t done = 0;
	while (done < count && dma.m_active && !dma.m_to_device)
	{
		// m_count is 16 bits: a descriptor with a count of 0 runs for 65536
		// bytes, as decrement_chain() wraps it
		const uint32_t left = dma.m_count ? dma.m_count : 0x10000;
		const uint32_t run = std::min(left, count - done);
		dma_write_run(dma.m_cbp, src + done, run, dma.m_big_endian);
		dma.m_cbp += run;
		dma.m_count -= run;
		done += run;
		if (dma.m_count == 0)
			scsi_descriptor_done(channel);
	}
	return done;
}

void hpc3_device::scsi_fifo_flush(int channel)
{
	scsi_dma_t &dma = m_scsi_dma[channel];

	LOGMASKED(LOG_SCSI_DMA, "Flushing SCSI %d FIFO\n", channel);

	if (BIT(dma.m_bc, 29))
	{
		LOGMASKED(LOG_SCSI_IRQ, "Raising SCSI %d IRQ\n", channel);
		m_intstat |= 0x100 << channel;
		m_dma_complete_int_cb(1);
	}

	dma.m_active = false;
	dma.m_ctrl &= ~(HPC3_DMACTRL_ENABLE | HPC3_DMACTRL_FLUSH);
}

void hpc3_device::scsi_drq(bool state, int channel)
{
	scsi_dma_t &dma = m_scsi_dma[channel];
	dma.m_drq = state;

	if (dma.m_drq && dma.m_active)
	{
		do_scsi_dma(channel);
	}
}

void hpc3_device::do_scsi_dma(int channel)
{
	scsi_dma_t &dma = m_scsi_dma[channel];

	const uint32_t addr = dma.m_big_endian ? BYTE4_XOR_BE(dma.m_cbp) : BYTE4_XOR_LE(dma.m_cbp);
	if (dma.m_to_device)
		m_hd_dma_wr_cb[channel](m_gio64_space->read_byte(addr));
	else
		m_gio64_space->write_byte(addr, m_hd_dma_rd_cb[channel]());

	dma.m_cbp++;
	decrement_chain(channel);

	if (!dma.m_active)
	{
		// clear HPC3 DMA active flag
		dma.m_ctrl &= ~HPC3_DMACTRL_ENABLE;
	}
}

void hpc3_device::scsi0_drq(int state)
{
	scsi_drq(state, 0);
}

void hpc3_device::scsi1_drq(int state)
{
	scsi_drq(state, 1);
}

uint32_t hpc3_device::intstat_r()
{
	return m_intstat;
}

uint32_t hpc3_device::misc_r()
{
	return m_misc;
}

void hpc3_device::misc_w(uint32_t data)
{
	LOGMASKED(LOG_PBUS_DMA, "%s: Write miscellaneous register: %08x\n", machine().describe_context(), data);
	LOGMASKED(LOG_PBUS_DMA, "    Real time devices %sabled\n", BIT(data, 0) ? "en" : "dis");
	LOGMASKED(LOG_PBUS_DMA, "    DMA descriptors are %s endian\n", BIT(data, 1) ? "little" : "big");
	m_misc = data & 3;
}

uint32_t hpc3_device::eeprom_r()
{
	uint32_t ret = (m_cpu_aux_ctrl & ~0x10) | (m_eeprom_dati_cb() << 4);
	LOGMASKED(LOG_EEPROM, "%s: HPC Serial EEPROM Read: %08x\n", machine().describe_context(), ret);
	return ret;
}

void hpc3_device::eeprom_w(uint32_t data)
{
	m_cpu_aux_ctrl = data;
	LOGMASKED(LOG_EEPROM, "%s: HPC Serial EEPROM Write: %08x\n", machine().describe_context(), data);
	m_eeprom_pre_cb(BIT(data, 0));
	m_eeprom_dato_cb(BIT(data, 3));
	m_eeprom_cs_cb(BIT(data, 1));
	m_eeprom_clk_cb(BIT(data, 2));
}

void hpc3_device::enet_transmit(int32_t param)
{
	// the channel may have been deactivated since this was scheduled
	if (!(m_enet_tx_ctrl & TXC_CA))
		return;

	// save the first transmit buffer descriptor pointer; the driver reads
	// these as a ping-pong pair holding the current and previous packets
	m_enet_tx_ppfbdp = m_enet_tx_cpfbdp;
	m_enet_tx_cpfbdp = m_enet_tx_nbdp;

	bool done = false;
	while (!done)
	{
		// fetch the current descriptor
		m_enet_tx_cbp = m_gio64_space->read_dword(m_enet_tx_nbdp + 0);
		m_enet_tx_bc = m_gio64_space->read_dword(m_enet_tx_nbdp + 4);
		m_enet_tx_nbdp = m_gio64_space->read_dword(m_enet_tx_nbdp + 8);

		LOGMASKED(LOG_ETHERNET, "enet tx dma chain 0x%08x cbp 0x%08x bc 0x%08x nbdp 0x%08x\n",
			m_enet_tx_cpfbdp, m_enet_tx_cbp, m_enet_tx_bc, m_enet_tx_nbdp);

		// TODO: write inter-packet gap from first descriptor to seeq
		//if (BIT(m_enet_dmacfg, 12))
		//  m_enet->write(2, u8(m_enet_tx_bc >> 16));

		// transfer data from memory to edlc fifo
		unsigned const count = m_enet_tx_bc & BC_BC;
		for (unsigned i = 0; i < count; i++)
			m_enet->fifo_w(m_gio64_space->read_byte(m_enet_tx_cbp + i));

		// check for end of packet
		if (m_enet_tx_bc & BC_EOXP)
		{
			m_enet->txeof_w(1);
			done = true;
		}

		// check for end of chain: the channel stops once this packet has
		// been transmitted (see enet_intr_in_w())
		if (m_enet_tx_bc & BC_EOX)
		{
			m_enet_tx_eox = true;
			done = true;
		}
	}
}

void hpc3_device::enet_misc_w(u32 data)
{
	// channel reset
	bool const entering_reset = bool(data & MISC_RESET);
	m_enet->reset_w(!entering_reset);

	if (entering_reset)
	{
		LOGMASKED(LOG_ETHERNET, "%s: HPC3 Ethernet channel reset (MISC=%08x)\n", machine().describe_context(), data);

		// reset ethernet dma state
		m_enet_rx_cbp = 0;
		m_enet_rx_nbdp = 0;
		m_enet_rx_bc = 0;
		m_enet_rx_ctrl = 0;
		m_enet_rx_gio = 0;
		m_enet_rx_dev = 0;
		m_enet_rx_cbdp = 0;

		m_enet_tx_cbp = 0;
		m_enet_tx_nbdp = 0;
		m_enet_tx_bc = 0;
		m_enet_tx_ctrl = 0;
		m_enet_tx_gio = 0;
		m_enet_tx_dev = 0;
		m_enet_tx_cpfbdp = 0;
		m_enet_tx_ppfbdp = 0;
		m_enet_tx_eox = false;

		// m_enet_tx_timer is deliberately left alone: with TXC_CA cleared,
		// enet_transmit() does nothing if it is still pending, whereas
		// adjust(attotime::never) would leave it marked enabled and block
		// the "!enabled()" checks that start subsequent transmits.
	}

	// clear channel interrupt
	if (data & MISC_INT)
		m_enet_intr_out_cb(0);

	// TODO: loopback

	m_enet_misc = data & ~MISC_INT;
}

bool hpc3_device::enet_rx_bc_dec(unsigned const count)
{
	if ((m_enet_rx_bc & BC_BC) >= count)
	{
		m_enet_rx_bc = (m_enet_rx_bc & ~BC_BC) | (((m_enet_rx_bc & BC_BC) - count) & BC_BC);

		return true;
	}

	// receive buffer overflow
	m_enet_rx_ctrl |= RXC_RBO;
	return false;
}
