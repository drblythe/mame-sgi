// license:BSD-3-Clause
// copyright-holders:Ryan Holtz
/**********************************************************************

    SGI HPC1 "High-performance Peripheral Controller" emulation

**********************************************************************/

#ifndef MAME_SGI_HPC1_H
#define MAME_SGI_HPC1_H

#pragma once

#include "machine/edlc.h"

class hpc1_device
	: public device_t
	, public device_memory_interface
{
public:
	hpc1_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	template <typename T> void set_gio(T &&tag, int spacenum) { m_gio.set_tag(std::forward<T>(tag), spacenum); }
	template <typename T> void set_enet(T &&tag) { m_enet.set_tag(std::forward<T>(tag)); }

	auto int_w() { return m_int_w.bind(); }
	auto scsi_reset_w() { return m_scsi_reset_w.bind(); }
	template <unsigned N> auto dma_r_cb() { return m_dma_r[N].bind(); }
	template <unsigned N> auto dma_w_cb() { return m_dma_w[N].bind(); }
	auto eeprom_dati() { return m_eeprom_dati.bind(); }
	auto eeprom_out() { return m_eeprom_out.bind(); }
	auto dsp_miscsr_w() { return m_dsp_miscsr_w.bind(); }
	auto dsp_irq_w() { return m_dsp_irq_w.bind(); }   // to the DSP (its IRQA)
	auto cpu_dsp_irq_w() { return m_cpu_dsp_irq_w.bind(); } // to the MIPS
	auto dsp_ram_r() { return m_dsp_ram_r.bind(); }   // DSP SRAM, by word
	auto dsp_ram_w() { return m_dsp_ram_w.bind(); }

	template <unsigned N> void write_drq(int state);
	void write_int(int state);

	// SCSI DMA of a run of bytes in one call, for a controller that can
	// hand over more than a byte per request: device to memory (in) and
	// memory to device (out); each returns how many bytes it moved
	u32 scsi_dma_bulk_in(const u8 *src, u32 count);
	u32 scsi_dma_bulk_out(u8 *dst, u32 count);

	void map(address_map &map) ATTR_COLD;

	// DSP side of the DSP interface registers (DSP Y:$FFE0-$FFED)
	static constexpr offs_t DSPIF_COUNTER_OFFSET = 5;
	u32 dsp_r(offs_t offset);
	void dsp_w(offs_t offset, u32 data);

protected:
	// device_t implementation
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;

	// device_memory_interface implementation
	virtual space_config_vector memory_space_config() const override;

	// dma
	void enet_dma();
	void scsi_dma();
	void scsi_chain();

	// ethernet read handlers
	u32 cxbp_r() { return m_enet_cxbp; }
	u32 nxbdp_r() { return m_enet_nxbdp; }
	u32 xbc_r() { return m_enet_xbc; }
	u32 cxbdp_r() { return m_enet_cxbdp; }
	u32 cpfxbdp_r() { return m_enet_cpfxbdp; }
	u32 ppfxbdp_r() { return m_enet_ppfxbdp; }
	u32 intdelay_r() { return m_enet_intdelay; }
	u32 trstat_r();
	u32 rcvstat_r();
	u32 ctl_r() { return m_enet_ctrl; }
	u32 rbc_r() { return m_enet_rbc; }
	u32 crbp_r() { return m_enet_crbp; }
	u32 nrbdp_r() { return m_enet_nrbdp; }
	u32 crbdp_r() { return m_enet_crbdp; }

	// ethernet write handlers
	void cxbp_w(u32 data);
	void nxbdp_w(u32 data);
	void xbc_w(u32 data);
	void cxbdp_w(u32 data);
	void cpfxbdp_w(u32 data);
	void ppfxbdp_w(u32 data);
	void intdelay_w(u32 data);
	void trstat_w(u32 data);
	void rcvstat_w(u32 data);
	void ctl_w(u32 data);
	void rbc_w(u32 data);
	void crbp_w(u32 data);
	void nrbdp_w(u32 data);
	void crbdp_w(u32 data);

	// scsi read handlers
	u32 scsi_bc_r() { return m_scsi_bc; }
	u32 scsi_cbp_r() { return m_scsi_cbp; }
	u32 scsi_nbdp_r() { return m_scsi_nbdp; }
	u32 scsi_ctrl_r() { return m_scsi_ctrl; }

	// scsi write handlers
	void scsi_bc_w(u32 data) { m_scsi_bc = data & 0x1fffU; }
	void scsi_cbp_w(u32 data) { m_scsi_cbp = data & 0x8fff'ffffU; }
	void scsi_nbdp_w(u32 data);
	void scsi_ctrl_w(u32 data);

	// DSP interface registers, shared by the MIPS and DSP sides
	enum dspif_reg : unsigned
	{
		DSPIF_DMAWDCNT = 0,
		DSPIF_GIOADDL,
		DSPIF_GIOADDM,
		DSPIF_PBUSADD,
		DSPIF_DMACTRL,
		DSPIF_COUNTER,
		DSPIF_HANDTX,
		DSPIF_HANDRX,
		DSPIF_CINTSTAT,
		DSPIF_CINTMASK,
		DSPIF_INTCONF,
		DSPIF_INTPOL,
		DSPIF_MISCSR,
		DSPIF_BURSTCTL,
		DSPIF_COUNT
	};
	u32 dspif_r(bool dsp, offs_t offset);
	u32 mips_dspif_r(offs_t offset);
	void mips_dspif_w(offs_t offset, u32 data);
	void dspif_update_irqs();
	TIMER_CALLBACK_MEMBER(dspif_dma_done);
	TIMER_CALLBACK_MEMBER(mips_miscsr_sync);
	void dspif_w(bool dsp, offs_t offset, u32 data);
	void dspif_dma();

	u8 aux_r();
	void aux_w(u8 data);

	void pbus_map(address_map &map) ATTR_COLD;

private:
	address_space_config const m_pbus;
	required_address_space m_gio;

	optional_device<seeq8003_device> m_enet;

	devcb_write_line m_int_w;
	devcb_write_line m_scsi_reset_w;
	devcb_read_line m_eeprom_dati;
	devcb_read8::array<2> m_dma_r;
	devcb_write8::array<2> m_dma_w;
	devcb_write8 m_eeprom_out;
	devcb_write8 m_dsp_miscsr_w;
	devcb_write_line m_dsp_irq_w;
	devcb_write_line m_cpu_dsp_irq_w;
	devcb_read32 m_dsp_ram_r;
	devcb_write32 m_dsp_ram_w;

	u8 m_drq;

	u32 m_enet_cxbp;     // enet current transmit buffer pointer
	u32 m_enet_nxbdp;    // enet current transmit buffer descriptor pointer
	u32 m_enet_xbc;      // enet transmit byte count
	u32 m_enet_cxbdp;    // enet current transmit buffer descriptor pointer
	u32 m_enet_cpfxbdp;  // enet current packet first transmit buffer descriptor pointer
	u32 m_enet_ppfxbdp;  // enet previous packet first transmit buffer descriptor pointer
	u32 m_enet_intdelay; // enet interrupt delay count
	u32 m_enet_txs;      // enet transmit status
	u32 m_enet_rxs;      // enet receive status
	u32 m_enet_ctrl;     // enet interrupt, channel reset, buffer overflow
	u32 m_enet_rbc;      // enet receive byte count
	u32 m_enet_crbp;     // enet current receive buffer pointer
	u32 m_enet_nrbdp;    // enet next receive buffer desriptor pointer
	u32 m_enet_crbdp;    // enet current receive buffer descriptor pointer

	u32 m_scsi_bc;       // scsi byte count
	u32 m_scsi_cbp;      // scsi current buffer pointer
	u32 m_scsi_nbdp;     // scsi next buffer descriptor pointer
	u32 m_scsi_ctrl;     // scsi control

	u32 m_dspif[DSPIF_COUNT];
	u8 m_cpu_intstat;   // interrupt status and mask on the MIPS side
	u8 m_cpu_intmask;
	u32 m_dsp_intstat;  // on the DSP side, and its interrupt configuration
	u32 m_dsp_intmask;
	u32 m_dsp_intconf;
	u32 m_dsp_intpol;
	emu_timer *m_dsp_dma_timer;

	u8 m_aux;
};

DECLARE_DEVICE_TYPE(SGI_HPC1, hpc1_device)

#endif // MAME_SGI_HPC1_H
