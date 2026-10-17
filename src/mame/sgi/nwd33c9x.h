// license:BSD-3-Clause
// copyright-holders:Tyson Smith, Olivier Galibert
/*
 * nwd33c9x.h
 *
 * SGI-only, single-object reimplementation of the WD33C93B initiator
 * (machine/wd33c9x.cpp) plus a fixed, embedded disk target (bus/nscsi/hd.cpp,
 * built on machine/nscsi_hle.cpp's generic SCSI target state machine) that
 * this project's Indy driver actually pairs it with.
 *
 * Profiling found the real per-byte cost of the stock two-object design
 * was not scheduler overhead (already fixed by
 * MAME_SGI_SCSI_COARSE_SCHEDULE in wd33c9x.cpp) but nscsi_full_device's own
 * step() logic (~148ns/byte, measured 2026-09-20) plus
 * nscsi_bus_device::regen_ctrl()'s per-notification vtable-override check
 * and dispatch loop - both structural consequences of the initiator and
 * target living in separate, polymorphic, bus-mediated objects. This device
 * keeps the exact same *behavior* (real REQ/ACK/phase signaling, byte-for-
 * byte identical side effects) but replaces every nscsi_bus_device-mediated
 * call in both FSMs with a direct, non-virtual call between two halves of
 * one C++ object - no bus object, no per-notification device-array scan.
 *
 * This is a fork, not a patch: machine/wd33c9x.cpp, machine/nscsi_hle.cpp,
 * and bus/nscsi/hd.cpp are all left completely untouched (the stock devices
 * keep working and stay available for direct A/B). Most of the initiator
 * half below is copied near-verbatim from wd33c9x_base_device - it was
 * already correct and already validated (including the coarse-schedule
 * fix) - only the bus-touching call sites change. The target half is
 * ported from nscsi_hle.cpp/hd.cpp's logic, trimmed to what a fixed,
 * single, disk-only target actually needs (no arbitrary command-set
 * generality, no multi-device bus, no reselect/disconnect - hd.cpp's own
 * target never uses those).
 *
 * Deliberately dropped rather than carried over from wd33c9x.cpp:
 *   - The WD33C92/93/93A/93B device-type multiplicity: the base file's
 *     register/FSM logic never actually differs between them (only
 *     DEFINE_DEVICE_TYPE's name/description strings do) - one concrete
 *     class covers the same behavior.
 *   - MAME_SGI_SCSI_DELAY_EXPERIMENT (SCALE=500 on arbitration polling) -
 *     documented in wd33c9x.cpp's own header as a throwaway measurement,
 *     never a real fix, and already found to produce no measurable win.
 *     Arbitration polling here uses the real, unscaled delay.
 *   - hd.cpp's optional cylinder-aware seek-timing model (set_seek_timing/
 *     seek_time) - not exercised by this project's default machine config,
 *     can be ported later if a phase needs it.
 *
 * Kept identical to the stock design:
 *   - MAME_SGI_SCSI_COARSE_SCHEDULE and its step_entry() reentrancy-depth
 *     guard. The cross-device reentrancy that guard fixes is a property of
 *     *synchronous bidirectional notification within one call chain*, not
 *     of the initiator and target being separate objects - it reproduces
 *     identically here (the target's reaction to an initiator ctrl change
 *     can itself trigger a target ctrl change that calls back into the
 *     initiator before the outer call returns) and needs the same fix.
 */

#ifndef MAME_SGI_NWD33C9X_H
#define MAME_SGI_NWD33C9X_H

#pragma once

#include "imagedev/harddriv.h"
#include "machine/nscsi_bus.h"

// See MAME_SGI_SCSI_COARSE_SCHEDULE's original definition in
// machine/wd33c9x.h for the full rationale - unchanged here.
#define MAME_SGI_SCSI_COARSE_SCHEDULE

// Scoped 2026-09-29, implemented 2026-09-30, enabled by default
// 2026-10-01 after three real bugs were found and fixed: fast path for
// coarse-eligible BURST DATA_IN/DATA_OUT transfers - skips the per-byte
// REQ/ACK state-machine walk (~7 switch-dispatch hops/byte, each a real
// host function call) for the repetitive MIDDLE section of a transfer.
// The first and last byte of each transfer always go through the real
// dance unchanged, to keep phase-start/transfer-complete sequencing (and
// any mid-transfer register inspection) byte-for-byte identical to the
// unoptimized path. Justified by the BURST-mode visibility diagnostic
// in nwd33c9x.cpp (IRIX's real driver never inspects intermediate REQ/ACK
// state in this mode).
//
// Confirmed LIVE (host gdb backtrace, both directions) before writing
// this: step()'s set_drq()/dma_r() and the DATA_OUT dma_w() call are
// fully SYNCHRONOUS, nested in one call chain from update_step()'s timer
// callback through hpc3_device::scsi0_drq() - not a separately-timed,
// asynchronous host-CPU-driven event the way a naive reading of the
// REQ/ACK dance suggests. This is what makes a tight, local fast-path
// loop safe to reason about at all.
//
// Restricted to SBUF_BLOCK (real disk sector reads/writes) only - the
// task's own original scoping was about disk-read throughput, never
// short command responses like INQUIRY/MODE_SENSE (SBUF_MAIN). PROM's own
// power-on SCSI diagnostic issues a 36-byte INQUIRY in BURST mode, which
// would otherwise also take the fast path; SBUF_BLOCK-only is both the
// originally-intended scope and the fix for that. Revisit the SBUF_MAIN
// case only with its own dedicated investigation, not as a side effect
// of chasing the disk-read win.
//
// Three real bugs found and fixed before this was trustworthy: (1) the fast loop must keep the initiator's own
// per-byte bookkeeping (m_i_state parked at INIT_XFR_WAIT_REQ, its S_REQ
// re-arm) live every iteration, not frozen; (2) t_target_send_buffer_byte()
// (and the t_target_recv_byte() DATA_OUT mirror) must explicitly replicate
// T_TARGET_WAIT_DATA_IN/OUT_BYTE's own m_t_state->T_TARGET_NEXT_CONTROL
// transition before the final byte's real-dance tail call, since the fast
// loop's self-recursion bypasses the t_step() dispatch that normally makes
// that transition; (3) that same T_TARGET_NEXT_CONTROL guard must be
// scoped to m_t_data_buffer_id==SBUF_BLOCK specifically - t_target_recv_byte()
// is also used for CDB/message-byte reception, where m_t_data_buffer_size
// is stale leftover from whatever SBUF_BLOCK transfer came before and has
// no business deciding completion there at all.
//
// A/B validated: boot-to-marker instructions -23.15%, cycles -13.90%,
// branches -25.22% (all p<0.0001, n=5 each, tools/perf_ab_stats.sh); disk_benchmark.sh
// throughput ~3.1x. Boot-to-login wall-clock itself is flat, consistent
// with every other SCSI-path optimization this project has measured -
// boot is dominated by non-disk cost, not disk bandwidth.
//
// MAME_SGI_SCSI_BURST_FASTPATH_COMPARE_DIAG below was planned as a
// standing byte-level validation companion but was never actually
// implemented - see its own comment for why and what was used instead.
#define MAME_SGI_SCSI_BURST_FASTPATH

// Planned companion to MAME_SGI_SCSI_BURST_FASTPATH, matching mips3.h's
// MAME_SGI_FASTRAM_COMPARE_DIAG in spirit (cross-check every fast-pathed
// byte against what the real per-byte dance would have produced,
// fatalerror() on the first mismatch) - declared here when the fast path
// was first designed, but never actually implemented: all three real
// bugs this task found were instead caught with ad hoc, targeted
// osd_printf_info() instrumentation added and removed per-bug (cheaper
// to write for a one-shot question than a standing comparison harness,
// and in bug #3's case the mismatch was a live oscillation a byte-value
// comparison wouldn't have characterized anyway). Left declared, still unimplemented, in case a future change
// to this fast path specifically needs the byte-level comparison this
// was meant to provide. Meaningless without the macro above also
// defined. Disabled by default.
// #define MAME_SGI_SCSI_BURST_FASTPATH_COMPARE_DIAG

// With MAME_SGI_SCSI_BURST_FASTPATH, a disk read in BURST mode hands runs of
// bytes (up to the end of the current sector) to the host's DMA engine in
// one call through the bulk_dma_in delegate, instead of a DRQ/DMA-read
// handshake per byte, and a disk write takes runs from it through
// bulk_dma_out. The host moves what it can and returns the count; the
// per-byte loop takes over from there exactly as before. Unbound, nothing
// changes.
#define MAME_SGI_SCSI_BULK_DMA 1

class nwd33c9x_device;

// Multiple CD-ROMs (2026-09-20): CD-ROM support was never the
// performance target of this device (only the boot disk's per-byte cost
// was ever measured as a problem) and bus/nscsi/cd.cpp's real command set
// is ~834 lines (audio playback, TOC/sub-channel data, several vendor
// quirk variants) - porting it by hand the way hd.cpp was ported would be
// a large amount of new, correctness-sensitive code for a device class
// this project has never touched, for no performance benefit. Instead:
// each configured CD-ROM SCSI ID gets a REAL nscsi_cdrom_sgi_device
// (100% of MAME's existing, correct CD-ROM emulation, completely
// unmodified) living on a small, genuinely real NSCSI_BUS owned by this
// device - this bridge is the one thing standing in for nwd33c9x itself
// on that bus, translating its handful of calls to/from nwd33c9x's own
// initiator half. Disks never touch this at all (see m_t_active_disk) -
// zero added overhead on the fast path this device exists for.
class nwd33c9x_aux_bridge_device : public device_t, public nscsi_device_interface
{
public:
	nwd33c9x_aux_bridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	void set_owner_device(nwd33c9x_device *owner) { m_owner_device = owner; }

	uint32_t bridge_ctrl_r() const { return m_scsi_bus->ctrl_r(); }
	uint32_t bridge_data_r() const { return m_scsi_bus->data_r(); }
	void bridge_ctrl_w(uint32_t lines, uint32_t mask) { m_scsi_bus->ctrl_w(m_scsi_refid, lines, mask); }
	void bridge_data_w(uint32_t lines) { m_scsi_bus->data_w(m_scsi_refid, lines); }
	void bridge_ctrl_wait(uint32_t lines, uint32_t mask) { m_scsi_bus->ctrl_wait(m_scsi_refid, lines, mask); }

	virtual void scsi_ctrl_changed() override;

protected:
	virtual void device_start() override ATTR_COLD;

private:
	nwd33c9x_device *m_owner_device;
};

DECLARE_DEVICE_TYPE(NWD33C9X_AUX_BRIDGE, nwd33c9x_aux_bridge_device)

class nwd33c9x_device : public device_t
{
	friend class nwd33c9x_aux_bridge_device;

public:
	nwd33c9x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock = 0);

	auto irq_cb() { return m_irq_cb.bind(); }
	auto drq_cb() { return m_drq_cb.bind(); }

	// see MAME_SGI_SCSI_BULK_DMA
	using bulk_dma_delegate = device_delegate<uint32_t (const uint8_t *src, uint32_t count)>;
	template <typename... T> void set_bulk_dma_in(T &&... args) { m_bulk_dma_in.set(std::forward<T>(args)...); }
	using bulk_dma_out_delegate = device_delegate<uint32_t (uint8_t *dst, uint32_t count)>;
	template <typename... T> void set_bulk_dma_out(T &&... args) { m_bulk_dma_out.set(std::forward<T>(args)...); }

	// Direct Addressing Interface
	uint8_t dir_r(offs_t offset);
	void dir_w(offs_t offset, uint8_t data);

	// Indirect Addressing Interface
	uint8_t indir_r(offs_t offset);
	void indir_w(offs_t offset, uint8_t data);

	// Alternative Indirect Addressing Interface
	uint8_t indir_addr_r();
	void indir_addr_w(uint8_t data);
	uint8_t indir_reg_r();
	void indir_reg_w(uint8_t data);

	uint8_t status_r();

	// Master Reset (MR) Interface
	void reset_w(int state);

	// DMA Interface (for use with DRQ)
	uint8_t dma_r();
	void dma_w(const uint8_t data);

protected:
	virtual void device_start() override ATTR_COLD;
	virtual void device_reset() override ATTR_COLD;
	virtual void device_add_mconfig(machine_config &config) override ATTR_COLD;

private:
	static const char *const state_names[];
	static const char *const substate_names[];
	static constexpr uint8_t NUM_REGS = 0x20;
	static constexpr uint8_t REGS_MASK = NUM_REGS - 1;
	uint8_t m_addr;
	uint8_t m_regs[NUM_REGS];
	uint8_t m_command_length;
	uint8_t m_last_message;

	void start_command();
	// Looks up m_regs[DESTINATION_ID] against the configured/mounted disk
	// slots and sets m_t_active_disk accordingly - called once per SELECT/
	// SELECT_TRANSFER dispatch, before arbitration begins (see the bus-
	// glue comment below for why this can be decided synchronously up
	// front rather than discovered via a bus broadcast).
	void determine_active_disk();
	TIMER_CALLBACK_MEMBER(update_step);

	// Every genuinely external entry point into the initiator state
	// machine must call step_entry(), never step() directly - see
	// wd33c9x.h's identical comment. Unchanged rationale here.
	void step_entry(bool timeout);
	void step(bool timeout);
	void set_i_state(uint16_t state);
	void set_i_state_sub(uint8_t sub);
	uint16_t m_i_state;
	uint8_t m_mode;
	uint8_t m_xfr_phase;
	int m_i_scsi_id;

	void load_transfer_count();
	bool decrement_transfer_count();
	uint32_t m_transfer_count;

	uint8_t data_fifo_pop();
	void data_fifo_push(const uint8_t data);
	bool data_fifo_empty() const;
	bool data_fifo_full() const;
	void data_fifo_reset();
	static constexpr uint8_t DATA_FIFO_SIZE = 12;
	uint8_t m_data_fifo[DATA_FIFO_SIZE];
	uint8_t m_data_fifo_pos;
	uint8_t m_data_fifo_size;

	uint32_t send_byte(const uint32_t value = 0, const uint32_t mask = 0);

	uint8_t irq_fifo_pop();
	void irq_fifo_push(const uint8_t status);
	bool irq_fifo_empty() const;
	bool irq_fifo_full() const;
	void irq_fifo_reset();
	static constexpr uint8_t IRQ_FIFO_SIZE = 8;
	uint8_t m_irq_fifo[IRQ_FIFO_SIZE];
	uint8_t m_irq_fifo_pos;
	uint8_t m_irq_fifo_size;

	void update_irq();
	devcb_write_line m_irq_cb;

	void set_drq();
	void clear_drq();
	devcb_write_line m_drq_cb;
	bool m_drq_state;
	bulk_dma_delegate m_bulk_dma_in;
	bulk_dma_out_delegate m_bulk_dma_out;
	// whole sectors of a bulk DMA write, written to the disk in one call
	static constexpr uint32_t T_STAGE_BYTES = 65536;
	std::unique_ptr<uint8_t[]> m_t_stage;

	void delay(uint32_t cycles);
	void delay_arb_poll();
	void delay_cycles(uint32_t cycles);
	emu_timer *m_timer;
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	bool coarse_schedule_eligible() const;
	bool m_coarse_continue;
	uint32_t m_coarse_cycle_debt;
	uint32_t m_coarse_iterations;
	// See step_entry()'s own comment and wd33c9x.h's identical field -
	// unchanged rationale, still needed in the flat design (see this
	// file's header comment).
	uint32_t m_step_entry_depth;
#endif

	bool set_command_length(const uint8_t cc);
	uint8_t get_msg_out() const;

	//----------------------------------------------------------------
	// Bus-line glue: replaces nscsi_bus_device's per-device asserted-
	// bits array + regen_ctrl()/regen_data() OR-and-notify loop, fixed
	// to exactly two parties (this object's own initiator half "i" and
	// target half "t") instead of a scanned array of up to 16 devices.
	//
	// Multiple drives/CD-ROMs (2026-09-20): which "other party" a given
	// call routes to is decided once per SELECT/SELECT_TRANSFER command,
	// synchronously, from the already-known DESTINATION_ID register (see
	// start_command()) - m_t_active_disk holds the destination's slot
	// index (0..NUM_DISK_SLOTS-1) if an embedded disk is configured and
	// mounted there, or -1 if not (meaning: forward to the real aux bus
	// instead - a configured CD-ROM will answer there, or nothing will
	// and a real SELECTION_TIMEOUT results, exactly like an empty ID).
	// This is a real routing decision, not a broadcast - unlike a real
	// SCSI wire, there's no need to "discover" who's listening, since
	// the destination ID is already static configuration data known
	// ahead of time. ctrl_r()/data_r() branch the same way so a stale
	// m_i_ctrl/m_t_ctrl from a previous, differently-routed command
	// can never leak into the composite.
	//----------------------------------------------------------------
	uint32_t ctrl_r() const { return (m_t_active_disk >= 0) ? (m_i_ctrl | m_t_ctrl) : m_aux_bridge->bridge_ctrl_r(); }
	uint32_t data_r() const { return (m_t_active_disk >= 0) ? (m_i_data | m_t_data) : m_aux_bridge->bridge_data_r(); }
	void i_ctrl_w(uint32_t lines, uint32_t mask);
	void i_data_w(uint32_t lines);
	void i_ctrl_wait(uint32_t lines, uint32_t mask);
	void t_ctrl_w(uint32_t lines, uint32_t mask);
	void t_data_w(uint32_t lines) { m_t_data = lines; }
	void t_ctrl_wait(uint32_t lines, uint32_t mask) { m_t_wait_ctrl = (m_t_wait_ctrl & ~mask) | (lines & mask); }
	// Called by nwd33c9x_aux_bridge_device::scsi_ctrl_changed() - the
	// real aux bus's own notification that a real attached device (a
	// CD-ROM) changed a line we (the initiator) care about. Mirrors
	// wd33c9x_base_device::scsi_ctrl_changed()'s exact behavior, same as
	// t_ctrl_w()'s embedded-disk notification path below.
	void aux_ctrl_changed();
	uint32_t m_i_ctrl, m_i_data, m_i_wait_ctrl;
	uint32_t m_t_ctrl, m_t_data, m_t_wait_ctrl;

	required_device<nwd33c9x_aux_bridge_device> m_aux_bridge;
	static void cdrom_devices(device_slot_interface &device);

	//----------------------------------------------------------------
	// Target half: ported from nscsi_hle.cpp's generic step() (trimmed
	// to what hd.cpp's command set and phase usage actually exercise -
	// no reselect, no disconnect, no messages beyond receiving a bare
	// IDENTIFY) + hd.cpp's own command dispatch, generalized to
	// NUM_DISK_SLOTS independently addressable embedded disks (index ==
	// SCSI ID) instead of one fixed target - m_t_active_disk (set once
	// per command, see the bus-glue comment above) selects which slot's
	// state m_t_disk[]/m_disk_image[] the code below operates on for the
	// duration of the current command.
	//----------------------------------------------------------------
	void t_step(bool timeout);
	void t_target_recv_byte();
	void t_target_send_byte(uint8_t val);
	void t_target_send_buffer_byte();
	bool t_command_done(uint8_t command, uint8_t length) const;
	void t_scsi_command();
	void t_scsi_message();
	uint8_t t_get_data(int id, int pos);
	uint8_t *t_block_at(int pos);
	void t_put_data(int id, int pos, uint8_t data);
	void t_status_complete(uint8_t st);
	void t_resume_deferred();
	void t_data_in(int buf, int size);
	void t_data_out(int buf, int size);
	void t_sense(bool deferred, uint8_t key, uint8_t asc = 0, uint8_t ascq = 0);
	void t_bad_lun();
	int t_get_lun(int def) const;
	TIMER_CALLBACK_MEMBER(t_update_tick);

	struct t_control {
		int m_action = 0;
		int m_param1 = 0;
		int m_param2 = 0;
	};
	t_control *t_buf_control_push();
	t_control *t_buf_control_pop();
	t_control m_t_buf_control[8];
	int m_t_buf_control_rpos, m_t_buf_control_wpos;

	emu_timer *m_t_timer;
	uint16_t m_t_state;
	int m_t_identify;
	// Sized for the largest response this target's command set actually
	// builds in place (MODE_SENSE_6 page 0x3f's concatenated pages, well
	// under 256 bytes) - not nscsi_full_device's generic 4096, which
	// exists to cover command sets (tape/CD-ROM) this target never uses.
	// Shared scratch, not per-slot: only one disk is ever active per
	// command (see m_t_active_disk), so there's nothing to keep separate.
	uint8_t m_t_cmdbuf[256];
	uint8_t m_t_cmdsize;
	int m_t_data_buffer_id;
	int m_t_data_buffer_size;
	int m_t_data_buffer_pos;

	// Multiple drives (2026-09-20): up to NUM_DISK_SLOTS independently
	// addressable embedded disks, slot index == SCSI ID (0..7, matching
	// real SCSI addressing and this project's existing NSCSI_CONNECTOR
	// convention) - no separate "set the ID" config call needed. Unlike
	// m_t_cmdbuf/m_t_data_buffer_* above, sense_buffer/cur_lba/inquiry_data
	// genuinely are per-target state on real hardware (each target chip
	// remembers its own last error independently) and are kept per-slot
	// here for the same reason, even though this project's own workload
	// never actually interleaves commands to two different disks closely
	// enough to observe the difference.
	static constexpr int NUM_DISK_SLOTS = 8;
	struct t_disk_slot {
		uint8_t block[512]{};
		int lba = 0, cur_lba = -1, blocks = 0;
		int bytes_per_sector = 0;
		int sector_shift = 0;   // log2(bytes_per_sector)
		uint8_t sense_buffer[18]{};
		std::vector<uint8_t> inquiry_data;
	};
	optional_device_array<harddisk_image_device, NUM_DISK_SLOTS> m_disk_image;
	t_disk_slot m_t_disk[NUM_DISK_SLOTS];
	// Which slot the current command addresses (0..NUM_DISK_SLOTS-1), or
	// -1 if the destination ID has no embedded disk configured/mounted
	// (routes through the aux bus instead - see the bus-glue comment).
	// Set once per SELECT/SELECT_TRANSFER command in start_command(),
	// held for that command's entire duration.
	int m_t_active_disk;
};

DECLARE_DEVICE_TYPE(NWD33C9X, nwd33c9x_device)

#endif // MAME_SGI_NWD33C9X_H
