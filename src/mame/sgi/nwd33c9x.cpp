// license:BSD-3-Clause
// copyright-holders:Tyson Smith, Olivier Galibert
/*
 * nwd33c9x.cpp
 *
 * See nwd33c9x.h for the full design rationale. Short version: this is a fork of
 * machine/wd33c9x.cpp's initiator half (kept near-verbatim - only the
 * m_scsi_bus-touching call sites change, to direct calls into the target
 * half below instead of through nscsi_bus_device) plus a hand-ported,
 * trimmed copy of machine/nscsi_hle.cpp's generic target state machine and
 * bus/nscsi/hd.cpp's disk command dispatch, folded into the same object.
 * The three original files are all left completely untouched.
 */

#include "emu.h"
#include "nwd33c9x.h"

#include "bus/nscsi/cd.h"
#include "bus/nscsi/tape.h"
#include "util/multibyte.h"

#include <algorithm>

#define LOG_READS       (1U << 1)
#define LOG_WRITES      (1U << 2)
#define LOG_COMMANDS    (1U << 3)
#define LOG_ERRORS      (1U << 4)
#define LOG_MISC        (1U << 5)
#define LOG_LINES       (1U << 6)
#define LOG_STATE       (1U << 7)
#define LOG_STEP        (1U << 8)
#define LOG_TARGET      (1U << 9)
#define LOG_REGS        (LOG_READS | LOG_WRITES)
#define LOG_ALL         (LOG_REGS | LOG_COMMANDS | LOG_ERRORS | LOG_MISC | LOG_LINES | LOG_STATE | LOG_STEP | LOG_TARGET)

#define VERBOSE         (0)
#include "logmacro.h"

// TEMPORARY diagnostic (2026-10-10): when $MAME_SGI_SCSI_LOG names
// a file, log each command start (with the CDB for select-and-transfer) and
// each interrupt raised or held back, with emulated time, to find a request
// that never completes.
// #define MAME_SGI_SCSI_LOG_DIAG

#ifdef MAME_SGI_SCSI_LOG_DIAG
static FILE *scsi_log_diag_file()
{
	static FILE *s_f = nullptr;
	static bool s_init = false;
	if (!s_init)
	{
		s_init = true;
		if (char const *const p = getenv("MAME_SGI_SCSI_LOG"))
			s_f = fopen(p, "w");
	}
	return s_f;
}
#endif

// Cap on how many bytes' worth of cycles can be accumulated before a real
// timer is forced regardless of eligibility - see MAME_SGI_SCSI_COARSE_
// SCHEDULE's definition in the header. Unchanged from wd33c9x.cpp.
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
static constexpr uint32_t MAME_SGI_SCSI_COARSE_MAX_ITERATIONS = 65536;
#endif

// Required for correctness, not just a boot-time tweak - see delay_arb_
// poll()'s own comment. Matches machine/wd33c9x.cpp's currently-enabled
// MAME_SGI_SCSI_DELAY_SCALE value exactly (that file's own header traces
// SCALE=0's failure mode; this device reproduced it live before this was
// added). Always on here (no #ifdef/off state) since it isn't optional in
// this device - unlike wd33c9x.cpp, nwd33c9x has no separately-proven-safe
// "off" configuration for this constant.
static constexpr uint32_t MAME_SGI_SCSI_DELAY_SCALE = 500;


//**************************************************************************
//  Initiator (WD33C93B) register/chip-level constants - copied verbatim
//  from machine/wd33c9x.cpp, which never varies this by device variant.
//**************************************************************************

enum register_addresses_e : uint8_t {
	OWN_ID               = 0x00,
	CONTROL              = 0x01,
	TIMEOUT_PERIOD       = 0x02,
	CDB_1                = 0x03,
	CDB_2                = 0x04,
	CDB_3                = 0x05,
	CDB_4                = 0x06,
	CDB_5                = 0x07,
	CDB_6                = 0x08,
	CDB_7                = 0x09,
	CDB_8                = 0x0a,
	CDB_9                = 0x0b,
	CDB_10               = 0x0c,
	CDB_11               = 0x0d,
	CDB_12               = 0x0e,
	TARGET_LUN           = 0x0f,
	COMMAND_PHASE        = 0x10,
	SYNCHRONOUS_TRANSFER = 0x11,
	TRANSFER_COUNT_MSB   = 0x12,
	TRANSFER_COUNT       = 0x13,
	TRANSFER_COUNT_LSB   = 0x14,
	DESTINATION_ID       = 0x15,
	SOURCE_ID            = 0x16,
	SCSI_STATUS          = 0x17,
	COMMAND              = 0x18,
	DATA                 = 0x19,
	QUEUE_TAG            = 0x1a,
	INVALID_1B           = 0x1b,
	INVALID_1C           = 0x1c,
	INVALID_1D           = 0x1d,
	INVALID_1E           = 0x1e,
	AUXILIARY_STATUS     = 0x1f
};

enum own_id_e : uint8_t {
	OWN_ID_M_SCSI_ID = 0x07,
	OWN_ID_EAF       = 0x08,
	OWN_ID_EHP       = 0x10,
	OWN_ID_RAF       = 0x20,
	OWN_ID_FS        = 0xc0,
	OWN_ID_FS_2      = 0x00,
	OWN_ID_FS_3      = 0x40,
	OWN_ID_FS_4      = 0x80,
	OWN_ID_CDB_SIZE  = 0x0f
};

enum control_e : uint8_t {
	CONTROL_HSP       = 0x01,
	CONTROL_HA        = 0x02,
	CONTROL_IDI       = 0x04,
	CONTROL_EDI       = 0x08,
	CONTROL_HHP       = 0x10,
	CONTROL_DM        = 0xe0,
	CONTROL_DM_POLLED = 0x00,
	CONTROL_DM_BURST  = 0x20,
	CONTROL_DM_BUS    = 0x40,
	CONTROL_DM_DMA    = 0x80,
};

enum target_lun_e : uint8_t {
	TARGET_LUN_TL  = 0x07,
	TARGET_LUN_TRN = 0x20,
	TARGET_LUN_DOK = 0x40,
	TARGET_LUN_TLV = 0x80,
};

enum command_phase_e : uint8_t {
	COMMAND_PHASE_ZERO                     = 0x00,
	COMMAND_PHASE_SELECTED                 = 0x10,
	COMMAND_PHASE_IDENTIFY_MESSAGE         = 0x20,
	COMMAND_PHASE_TAG_MESSAGE              = 0x21,
	COMMAND_PHASE_QUEUE_TAG                = 0x22,
	COMMAND_PHASE_CP_BYTES_0               = 0x30,
	COMMAND_PHASE_CP_BYTES_1               = 0x31,
	COMMAND_PHASE_CP_BYTES_2               = 0x32,
	COMMAND_PHASE_CP_BYTES_3               = 0x33,
	COMMAND_PHASE_CP_BYTES_4               = 0x34,
	COMMAND_PHASE_CP_BYTES_5               = 0x35,
	COMMAND_PHASE_CP_BYTES_6               = 0x36,
	COMMAND_PHASE_CP_BYTES_7               = 0x37,
	COMMAND_PHASE_CP_BYTES_8               = 0x38,
	COMMAND_PHASE_CP_BYTES_9               = 0x39,
	COMMAND_PHASE_CP_BYTES_A               = 0x3a,
	COMMAND_PHASE_CP_BYTES_B               = 0x3b,
	COMMAND_PHASE_CP_BYTES_C               = 0x3c,
	COMMAND_PHASE_SAVE_DATA_POINTER        = 0x41,
	COMMAND_PHASE_DISCONNECT_MESSAGE       = 0x42,
	COMMAND_PHASE_DISCONNECTED             = 0x43,
	COMMAND_PHASE_RESELECTED               = 0x44,
	COMMAND_PHASE_IDENTIFY_MATCH           = 0x45,
	COMMAND_PHASE_TRANSFER_COUNT           = 0x46,
	COMMAND_PHASE_RECEIVE_STATUS           = 0x47,
	COMMAND_PHASE_STATUS_RECEIVED          = 0x50,
	COMMAND_PHASE_COMMAND_COMPLETE         = 0x60,
	COMMAND_PHASE_LINKED_COMMAND_COMPLETE  = 0x61,
	COMMAND_PHASE_TARGET_LUN               = 0x70,
	COMMAND_PHASE_SIMPLE_QUEUE_TAG_MESSAGE = 0x71,
};

enum destination_id_e : uint8_t {
	DESTINATION_ID_DI     = 0x07,
	DESTINATION_ID_TG     = 0x18,
	DESTINATION_ID_TG_NM  = 0x00,
	DESTINATION_ID_TG_SQT = 0x08,
	DESTINATION_ID_TG_HQT = 0x10,
	DESTINATION_ID_TG_OQT = 0x18,
	DESTINATION_ID_DF     = 0x20,
	DESTINATION_ID_DPD    = 0x40,
	DESTINATION_ID_SCC    = 0x80,
};

enum source_id_e : uint8_t {
	SOURCE_ID_SI  = 0x07,
	SOURCE_ID_SIV = 0x08,
	SOURCE_ID_DSP = 0x20,
	SOURCE_ID_ES  = 0x40,
	SOURCE_ID_ER  = 0x80,
};

enum scsi_status_e : uint8_t {
	SCSI_STATUS_RESET                                 = 0x00,
	SCSI_STATUS_RESET_EAF                             = 0x01,

	SCSI_STATUS_RESELECT_SUCCESS                      = 0x10,
	SCSI_STATUS_SELECT_SUCCESS                        = 0x11,
	SCSI_STATUS_COMMAND_SUCCESS                       = 0x13,
	SCSI_STATUS_COMMAND_ATN_SUCCESS                   = 0x14,
	SCSI_STATUS_TRANSLATE_SUCCESS                     = 0x15,
	SCSI_STATUS_SELECT_TRANSFER_SUCCESS               = 0x16,
	SCSI_STATUS_TRANSFER_SUCCESS                      = 0x18,

	SCSI_STATUS_TRANSFER_INFO_MSG_IN                  = 0x20,
	SCSI_STATUS_SAVE_DATA_POINTERS                    = 0x21,
	SCSI_STATUS_SELECTION_ABORTED                     = 0x22,
	SCSI_STATUS_RECEIVE_SEND_ABORTED                  = 0x23,
	SCSI_STATUS_RECEIVE_SEND_ABORTED_ATN              = 0x24,
	SCSI_STATUS_ABORT_DURING_SELECTION                = 0x25,
	SCSI_STATUS_RESELECTED_DURING_SELECT_AND_TRANSFER = 0x27,
	SCSI_STATUS_TRANSFER_ABORTED                      = 0x28,

	SCSI_STATUS_INVALID_COMMAND                       = 0x40,
	SCSI_STATUS_UNEXPECTED_DISCONNECT                 = 0x41,
	SCSI_STATUS_SELECTION_TIMEOUT                     = 0x42,
	SCSI_STATUS_PARITY_ERROR                          = 0x43,
	SCSI_STATUS_PARITY_ERROR_ATN                      = 0x44,
	SCSI_STATUS_LOGICAL_ADDRESS_TOO_LARGE             = 0x45,
	SCSI_STATUS_RESELECTION_MISMATCH                  = 0x46,
	SCSI_STATUS_INCORRECT_STATUS_BYTE                 = 0x47,
	SCSI_STATUS_UNEXPECTED_PHASE                      = 0x48,

	SCSI_STATUS_RESELECTED                            = 0x80,
	SCSI_STATUS_RESELECTED_EAF                        = 0x81,
	SCSI_STATUS_SELECTED                              = 0x82,
	SCSI_STATUS_SELECTED_ATN                          = 0x83,
	SCSI_STATUS_ATN                                   = 0x84,
	SCSI_STATUS_DISCONNECT                            = 0x85,
	SCSI_STATUS_NEED_COMMAND_SIZE                     = 0x87,
	SCSI_STATUS_REQ                                   = 0x88
};

enum command_e : uint8_t {
	COMMAND_CC                              = 0x7f,
	COMMAND_CC_RESET                        = 0x00,
	COMMAND_CC_ABORT                        = 0x01,
	COMMAND_CC_ASSERT_ATN                   = 0x02,
	COMMAND_CC_NEGATE_ACK                   = 0x03,
	COMMAND_CC_DISCONNECT                   = 0x04,
	COMMAND_CC_RESELECT                     = 0x05,
	COMMAND_CC_SELECT_ATN                   = 0x06,
	COMMAND_CC_SELECT                       = 0x07,
	COMMAND_CC_SELECT_ATN_TRANSFER          = 0x08,
	COMMAND_CC_SELECT_TRANSFER              = 0x09,
	COMMAND_CC_RESELECT_RECEIVE_DATA        = 0x0a,
	COMMAND_CC_RESELECT_SEND_DATA           = 0x0b,
	COMMAND_CC_WAIT_SELECT_RECEIVE_DATA     = 0x0c,
	COMMAND_CC_SEND_STATUS_COMMAND_COMPLETE = 0x0d,
	COMMAND_CC_SEND_DISCONNECT_MESSAGE      = 0x0e,
	COMMAND_CC_SET_IDI                      = 0x0f,
	COMMAND_CC_RECEIVE_COMMAND              = 0x10,
	COMMAND_CC_RECEIVE_DATA                 = 0x11,
	COMMAND_CC_RECEIVE_MESSAGE_OUT          = 0x12,
	COMMAND_CC_RECEIVE_UNSPECIFIED_INFO_OUT = 0x13,
	COMMAND_CC_SEND_STATUS                  = 0x14,
	COMMAND_CC_SEND_DATA                    = 0x15,
	COMMAND_CC_SEND_MESSAGE_IN              = 0x16,
	COMMAND_CC_SEND_UNSPECIFIED_INFO_IN     = 0x17,
	COMMAND_CC_TRANSLATE_ADDRESS            = 0x18,
	COMMAND_CC_TRANSFER_PAD                 = 0x19,
	COMMAND_CC_TRANSFER_INFO                = 0x20,
	COMMAND_SBT                             = 0x80,
};

enum auxiliary_status_e : uint8_t {
	AUXILIARY_STATUS_DBR = 0x01,
	AUXILIARY_STATUS_PE  = 0x02,
	AUXILIARY_STATUS_FFE = 0x04,
	AUXILIARY_STATUS_CIP = 0x10,
	AUXILIARY_STATUS_BSY = 0x20,
	AUXILIARY_STATUS_LCI = 0x40,
	AUXILIARY_STATUS_INT = 0x80
};

enum : uint8_t {
	MODE_D,
	MODE_T,
	MODE_I
};

enum : uint16_t {
	IDLE = 1,
	FINISHED,

	DISC_SEL_ARBITRATION,

	INIT_MSG_WAIT_REQ,
	INIT_XFR,
	INIT_XFR_SEND_PAD_WAIT_REQ,
	INIT_XFR_SEND_PAD,
	INIT_XFR_RECV_PAD_WAIT_REQ,
	INIT_XFR_RECV_PAD,
	INIT_XFR_RECV_BYTE_ACK,
	INIT_XFR_RECV_BYTE_NACK,
	INIT_XFR_FUNCTION_COMPLETE,
	INIT_XFR_BUS_COMPLETE,
	INIT_XFR_WAIT_REQ,
	INIT_CPT_RECV_BYTE_ACK,
	INIT_CPT_RECV_WAIT_REQ,
	INIT_CPT_RECV_BYTE_NACK
};

enum : uint16_t {
	ARB_WAIT_BUS_FREE = 1,
	ARB_CHECK_FREE,
	ARB_EXAMINE_BUS,
	ARB_ASSERT_SEL,
	ARB_SET_DEST,
	ARB_RELEASE_BUSY,
	ARB_TIMEOUT_BUSY,
	ARB_TIMEOUT_ABORT,
	ARB_DESKEW_WAIT,

	SEND_WAIT_SETTLE,
	SEND_WAIT_REQ_0,
	RECV_WAIT_REQ_1,
	RECV_WAIT_SETTLE,
	RECV_WAIT_REQ_0
};

enum : uint16_t {
	STATE_MASK = 0x00ff,
	SUB_SHIFT  = 8,
	SUB_MASK   = 0xff00
};

//**************************************************************************
//  SCSI protocol-level bit/message constants - copied from
//  nscsi_bus.h's nscsi_device_interface enum (pure bit constants, no
//  coupling to nscsi_bus_device itself).
//**************************************************************************

enum : uint32_t {
	S_INP = 0x0001,
	S_CTL = 0x0002,
	S_MSG = 0x0004,
	S_BSY = 0x0008,
	S_SEL = 0x0010,
	S_REQ = 0x0020,
	S_ACK = 0x0040,
	S_ATN = 0x0080,
	S_RST = 0x0100,
	S_ALL = 0x01ff,

	S_PHASE_DATA_OUT = 0,
	S_PHASE_DATA_IN  = S_INP,
	S_PHASE_COMMAND  = S_CTL,
	S_PHASE_STATUS   = S_CTL|S_INP,
	S_PHASE_MSG_OUT  = S_MSG|S_CTL,
	S_PHASE_MSG_IN   = S_MSG|S_CTL|S_INP,
	S_PHASE_MASK     = S_MSG|S_CTL|S_INP
};

enum : uint8_t {
	SM_COMMAND_COMPLETE  = 0x00,
	SM_SAVE_DATA_POINTER = 0x02,
	SM_DISCONNECT        = 0x04,
};

//**************************************************************************
//  Target-side constants: state machine (ported/trimmed from
//  machine/nscsi_hle.cpp) and command opcodes/status/sense codes actually
//  used by bus/nscsi/hd.cpp's command set - not the full ~600-line SCSI-2
//  enum nscsi_hle.h carries for command sets this target never implements.
//**************************************************************************

enum : uint16_t {
	T_IDLE = 1,
	T_TARGET_SELECT_WAIT_BUS_SETTLE,
	T_TARGET_SELECT_WAIT_SEL_0,
	T_RECV_BYTE_T_WAIT_ACK_1,
	T_RECV_BYTE_T_WAIT_ACK_0,
	T_SEND_BYTE_T_WAIT_ACK_1,
	T_SEND_BYTE_T_WAIT_ACK_0,
	T_TARGET_NEXT_CONTROL,
	T_TARGET_WAIT_DATA_IN_BYTE,
	T_TARGET_WAIT_DATA_OUT_BYTE,
	T_TARGET_WAIT_MSG_BYTE,
	T_TARGET_WAIT_CMD_BYTE,
};

enum : uint16_t {
	T_STATE_MASK = 0x00ff,
	T_SUB_SHIFT  = 8,
	T_SUB_MASK   = 0xff00
};

enum {
	T_BC_MSG_OR_COMMAND,
	T_BC_STATUS,
	T_BC_DATA_IN,
	T_BC_DATA_OUT,
	T_BC_MESSAGE_1,
	T_BC_BUS_FREE,
};

enum { SBUF_MAIN = 0, SBUF_SENSE = 1, SBUF_BLOCK = 2 };

enum : uint8_t {
	SC_TEST_UNIT_READY            = 0x00,
	SC_REZERO_UNIT                = 0x01,
	SC_REQUEST_SENSE              = 0x03,
	SC_FORMAT_UNIT                = 0x04,
	SC_READ_6                     = 0x08,
	SC_WRITE_6                    = 0x0a,
	SC_INQUIRY                    = 0x12,
	SC_MODE_SELECT_6              = 0x15,
	SC_MODE_SENSE_6               = 0x1a,
	SC_START_STOP_UNIT            = 0x1b,
	SC_RECEIVE_DIAGNOSTIC_RESULTS = 0x1c,
	SC_SEND_DIAGNOSTIC            = 0x1d,
	SC_READ_CAPACITY              = 0x25,
	SC_READ_10                    = 0x28,
	SC_WRITE_10                   = 0x2a,
	SC_VERIFY_10                  = 0x2f,
	SC_SYNCHRONIZE_CACHE          = 0x35,
};

enum : uint8_t {
	SS_GOOD            = 0x00,
	SS_CHECK_CONDITION = 0x02,
};

enum : uint8_t {
	SK_NO_SENSE       = 0x00,
	SK_ILLEGAL_REQUEST = 0x05,
};

enum : uint8_t {
	SK_ASC_INVALID_FIELD_IN_CDB       = 0x24,
	SK_ASC_LOGICAL_UNIT_NOT_SUPPORTED = 0x25,
};

//**************************************************************************
//  LIVE DEVICE
//**************************************************************************

DEFINE_DEVICE_TYPE(NWD33C9X, nwd33c9x_device, "nwd33c9x", "SGI direct-call WD33C93B + disk target (nwd33c9x)")
DEFINE_DEVICE_TYPE(NWD33C9X_AUX_BRIDGE, nwd33c9x_aux_bridge_device, "nwd33c9x_aux_bridge", "nwd33c9x auxiliary SCSI bus bridge (CD-ROM host)")

nwd33c9x_aux_bridge_device::nwd33c9x_aux_bridge_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NWD33C9X_AUX_BRIDGE, tag, owner, clock)
	, nscsi_device_interface(mconfig, *this)
	, m_owner_device(nullptr)
{
}

void nwd33c9x_aux_bridge_device::device_start()
{
}

void nwd33c9x_aux_bridge_device::scsi_ctrl_changed()
{
	m_owner_device->aux_ctrl_changed();
}

void nwd33c9x_device::cdrom_devices(device_slot_interface &device)
{
	device.option_add("cdrom", NSCSI_CDROM_SGI);
	device.option_add("tape", NSCSI_TAPE_SGI); // DDS-1 DAT drive (SIMH .tap image)
}

nwd33c9x_device::nwd33c9x_device(const machine_config &mconfig, const char *tag, device_t *owner, uint32_t clock)
	: device_t(mconfig, NWD33C9X, tag, owner, clock)
	, m_irq_cb(*this)
	, m_drq_cb(*this)
	, m_bulk_dma_in(*this)
	, m_bulk_dma_out(*this)
	, m_aux_bridge(*this, "auxbridge")
	, m_disk_image(*this, "disk%u", 0U)
{
	m_addr = 0;
	memset(m_regs, 0, sizeof(m_regs));
	m_command_length = 0;
	m_last_message = 0;
	m_i_state = IDLE;
	m_mode = MODE_D;
	m_xfr_phase = 0;
	m_i_scsi_id = -1;
	m_transfer_count = 0;
	memset(m_data_fifo, 0, sizeof(m_data_fifo));
	m_data_fifo_pos = 0;
	m_data_fifo_size = 0;
	memset(m_irq_fifo, 0, sizeof(m_irq_fifo));
	m_irq_fifo_pos = 0;
	m_irq_fifo_size = 0;
	m_drq_state = false;
	m_timer = nullptr;
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	m_coarse_continue = false;
	m_coarse_cycle_debt = 0;
	m_coarse_iterations = 0;
	m_step_entry_depth = 0;
#endif
	m_i_ctrl = m_i_data = m_i_wait_ctrl = 0;
	m_t_ctrl = m_t_data = m_t_wait_ctrl = 0;
	m_t_buf_control_rpos = m_t_buf_control_wpos = 0;
	m_t_timer = nullptr;
	m_t_state = T_IDLE;
	m_t_identify = 0;
	memset(m_t_cmdbuf, 0, sizeof(m_t_cmdbuf));
	m_t_cmdsize = 0;
	m_t_data_buffer_id = 0;
	m_t_data_buffer_size = 0;
	m_t_data_buffer_pos = 0;
	m_t_active_disk = -1;
}

void nwd33c9x_device::device_add_mconfig(machine_config &config)
{
	// MAME's "-hard1"/"-hard2"/... media options are auto-numbered by
	// creation order among same-interface devices, not by slot index/tag
	// - creating slot 1 first keeps "-hard1" meaning what it already
	// means for every existing test tool (tools/boot_test.sh,
	// tools/disk_benchmark.sh) and this project's own IRIX install
	// (SCSI ID 1 is the boot disk, baked into the guest's own PROM/NVRAM
	// hardware inventory - not something the MAME-side config can freely
	// renumber). m_disk_image[i] itself is unaffected by creation order
	// (its tag is fixed to "diskI" by the "disk%u" pattern below,
	// regardless of which order HARDDISK() is actually called in) - only
	// which CLI option number each slot answers to changes here.
	//
	// Every slot after the first also keeps SCSI ID == CLI number
	// directly (-hard2 -> ID 2, ..., -hard7 -> ID 7) - ID 0 is the only
	// one that can't line up this way, since slot 1 has to be created
	// first (above) and nothing else can occupy "-hard1". Put it LAST
	// instead of second (2026-09-28: the previous order's -hard2->ID0/
	// -hard3->ID2 mismatch was found live while attaching an auxiliary
	// test disk at a specific SCSI ID by CLI number alone): "-hard2" is
	// far likelier to get reached for than "-hard8", so this is the
	// smaller-surprise placement for the one ID that has to be the
	// exception.
	static constexpr int creation_order[NUM_DISK_SLOTS] = { 1, 2, 3, 4, 5, 6, 7, 0 };
	for (int i : creation_order)
		HARDDISK(config, m_disk_image[i]).set_interface("scsi_hdd,hdd");

	// Real, unmodified NSCSI_BUS hosting real CD-ROM devices - see this
	// class's own header comment for why CD-ROM isn't ported the way the
	// disk target was. m_aux_bridge occupies refid 0 (our own initiator
	// presence on this bus); connectors 1-7 offer the same "cdrom" option
	// indy_indigo2.cpp's stock scsibus does, defaulting to none except
	// slot 6 (matching this project's existing default boot media layout).
	// Capture NSCSI_BUS's own return reference directly (matching
	// indy_indigo2.cpp's identical "scsibus" pattern) rather than relying
	// on a separately-declared required_device<nscsi_bus_device> finder
	// bound to the same tag - that finder would not be resolved yet at
	// this point in device_add_mconfig() (only a finder passed directly
	// *into* the creating macro, like m_aux_bridge below, is populated
	// immediately).
	auto &auxbus(NSCSI_BUS(config, "auxbus"));
	NWD33C9X_AUX_BRIDGE(config, m_aux_bridge, 0);
	m_aux_bridge->set_owner_device(this);
	auxbus.set_external_device(0, m_aux_bridge);
	NSCSI_CONNECTOR(config, "auxbus:1", cdrom_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "auxbus:2", cdrom_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "auxbus:3", cdrom_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "auxbus:4", cdrom_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "auxbus:5", cdrom_devices, nullptr, false);
	NSCSI_CONNECTOR(config, "auxbus:6", cdrom_devices, "cdrom", false);
	NSCSI_CONNECTOR(config, "auxbus:7", cdrom_devices, nullptr, false);
}

void nwd33c9x_device::device_start()
{
	m_bulk_dma_in.resolve();
	m_bulk_dma_out.resolve();
	m_t_stage = std::make_unique<uint8_t[]>(T_STAGE_BYTES);
	m_timer = timer_alloc(FUNC(nwd33c9x_device::update_step), this);
	m_t_timer = timer_alloc(FUNC(nwd33c9x_device::t_update_tick), this);

	save_item(NAME(m_addr));
	save_item(NAME(m_regs));
	save_item(NAME(m_command_length));
	save_item(NAME(m_last_message));
	save_item(NAME(m_mode));
	save_item(NAME(m_i_state));
	save_item(NAME(m_xfr_phase));
	save_item(NAME(m_i_scsi_id));
	save_item(NAME(m_transfer_count));
	save_item(NAME(m_data_fifo));
	save_item(NAME(m_data_fifo_pos));
	save_item(NAME(m_data_fifo_size));
	save_item(NAME(m_irq_fifo));
	save_item(NAME(m_irq_fifo_pos));
	save_item(NAME(m_irq_fifo_size));
	save_item(NAME(m_drq_state));
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	save_item(NAME(m_coarse_continue));
	save_item(NAME(m_coarse_cycle_debt));
	save_item(NAME(m_coarse_iterations));
	save_item(NAME(m_step_entry_depth));
#endif
	save_item(NAME(m_i_ctrl));
	save_item(NAME(m_i_data));
	save_item(NAME(m_i_wait_ctrl));
	save_item(NAME(m_t_ctrl));
	save_item(NAME(m_t_data));
	save_item(NAME(m_t_wait_ctrl));

	save_item(NAME(m_t_state));
	save_item(NAME(m_t_identify));
	save_item(NAME(m_t_cmdbuf));
	save_item(NAME(m_t_cmdsize));
	save_item(NAME(m_t_data_buffer_id));
	save_item(NAME(m_t_data_buffer_size));
	save_item(NAME(m_t_data_buffer_pos));
	save_item(NAME(m_t_active_disk));
	for (int i = 0; i < NUM_DISK_SLOTS; i++) {
		save_item(NAME(m_t_disk[i].block), i);
		save_item(NAME(m_t_disk[i].lba), i);
		save_item(NAME(m_t_disk[i].cur_lba), i);
		save_item(NAME(m_t_disk[i].blocks), i);
		save_item(NAME(m_t_disk[i].bytes_per_sector), i);
		save_item(NAME(m_t_disk[i].sector_shift), i);
		save_item(NAME(m_t_disk[i].sense_buffer), i);
	}
	for (int i = 0; i < int(std::size(m_t_buf_control)); i++) {
		save_item(NAME(m_t_buf_control[i].m_action), i);
		save_item(NAME(m_t_buf_control[i].m_param1), i);
		save_item(NAME(m_t_buf_control[i].m_param2), i);
	}
	save_item(NAME(m_t_buf_control_rpos));
	save_item(NAME(m_t_buf_control_wpos));
}

void nwd33c9x_device::device_reset()
{
	// Reset before anything else: i_ctrl_w()/i_ctrl_wait() below route by
	// m_t_active_disk, and a stale value left over from whatever this
	// device's last command happened to address (before this reset) must
	// not affect where the reset's own line-clearing lands.
	m_t_active_disk = -1;

	// ---- Initiator half (wd33c9x_base_device::device_reset()) ----
	// Clears both potential routes directly rather than through i_ctrl_w()/
	// i_ctrl_wait() (which, with m_t_active_disk == -1 above, would only
	// reach the aux bus) - a real hardware reset must clear the embedded
	// fast-path side's own m_i_ctrl/m_i_data too.
	m_i_ctrl = 0;
	m_i_data = 0;
	m_i_wait_ctrl = S_SEL|S_BSY|S_RST;
	m_aux_bridge->bridge_ctrl_w(0, S_ALL);
	m_aux_bridge->bridge_data_w(0);
	m_aux_bridge->bridge_ctrl_wait(S_SEL|S_BSY|S_RST, S_ALL);
	m_addr = 0;
	for (uint8_t reg = 0; reg < NUM_REGS; ++reg) {
		m_regs[reg] = (QUEUE_TAG <= reg && reg <= INVALID_1E) ? 0xff : 0;
	}
	m_command_length = 0;
	m_last_message = 0;
	set_i_state(IDLE);
	m_mode = MODE_D;
	m_xfr_phase = 0;
	m_transfer_count = 0;
	data_fifo_reset();
	irq_fifo_reset();
	m_irq_cb(CLEAR_LINE);
	m_drq_cb(CLEAR_LINE);
	m_drq_state = false;
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	m_coarse_continue = false;
	m_coarse_cycle_debt = 0;
	m_coarse_iterations = 0;
	m_step_entry_depth = 0;
#endif
	irq_fifo_push((m_regs[OWN_ID] & OWN_ID_EAF) ? SCSI_STATUS_RESET_EAF : SCSI_STATUS_RESET);
	update_irq();

	// ---- Target half (nscsi_full_device::device_reset() + nscsi_harddisk_device::device_reset()) ----
	m_t_state = T_IDLE;
	m_t_buf_control_rpos = m_t_buf_control_wpos = 0;
	m_t_identify = 0;
	m_t_data_buffer_size = 0;
	m_t_data_buffer_pos = 0;
	t_data_w(0);
	t_ctrl_w(0, S_ALL);
	t_ctrl_wait(S_SEL|S_BSY|S_RST, S_ALL);
	for (int i = 0; i < NUM_DISK_SLOTS; i++) {
		t_disk_slot &d = m_t_disk[i];
		// Matches t_sense(false, SK_NO_SENSE)'s exact byte layout - can't
		// call t_sense() itself here since it operates on whichever slot
		// m_t_active_disk names, and no command (hence no active slot) is
		// in progress during a reset.
		memset(d.sense_buffer, 0, sizeof(d.sense_buffer));
		d.sense_buffer[0] = 0x80 | 0x70;
		d.sense_buffer[7] = 10;
		if (!m_disk_image[i] || !m_disk_image[i]->exists()) {
			d.bytes_per_sector = 0;
			d.sector_shift = 0;
		} else {
			const auto &hdinfo = m_disk_image[i]->get_info();
			d.bytes_per_sector = hdinfo.sectorbytes;
			// t_get_data()/t_put_data() split each byte position into
			// sector and offset with a shift and a mask, and buffer one
			// sector in d.block
			if (d.bytes_per_sector <= 0 || (d.bytes_per_sector & (d.bytes_per_sector - 1)) || d.bytes_per_sector > int(sizeof(d.block)))
				fatalerror("%s: unsupported sector size %d on disk %d (a power of two up to %d is required)\n",
						shortname(), d.bytes_per_sector, i, int(sizeof(d.block)));
			d.sector_shift = 0;
			while ((1 << d.sector_shift) < d.bytes_per_sector)
				d.sector_shift++;
			m_disk_image[i]->get_inquiry_data(d.inquiry_data);
		}
		d.cur_lba = -1;
	}
}

//**************************************************************************
//  MEMORY HANDLERS - copied verbatim from wd33c9x.cpp (none of these touch
//  the SCSI bus directly; that only happens inside start_command()/step()).
//**************************************************************************

uint8_t nwd33c9x_device::dir_r(offs_t offset)
{
	m_addr = offset & REGS_MASK;
	return indir_reg_r();
}

void nwd33c9x_device::dir_w(offs_t offset, uint8_t data)
{
	m_addr = offset & REGS_MASK;
	indir_reg_w(data);
}

uint8_t nwd33c9x_device::indir_r(offs_t offset)
{
	switch (offset) {
	case 0:
		return indir_addr_r();
	case 1:
		return indir_reg_r();
	default:
		LOGMASKED(LOG_READS | LOG_ERRORS, "Read from invalid offset %d\n", offset);
		break;
	}
	return 0;
}

void nwd33c9x_device::indir_w(offs_t offset, uint8_t data)
{
	switch (offset) {
	case 0:
		indir_addr_w(data);
		break;
	case 1:
		indir_reg_w(data);
		break;
	default:
		LOGMASKED(LOG_WRITES | LOG_ERRORS, "Write to invalid offset %d (data=%02x)\n", offset, data);
		break;
	}
}

uint8_t nwd33c9x_device::indir_addr_r()
{
	return m_regs[AUXILIARY_STATUS] & 0x01 ? m_regs[AUXILIARY_STATUS] & 0x7f : m_regs[AUXILIARY_STATUS];
}

uint8_t nwd33c9x_device::status_r()
{
	return m_regs[AUXILIARY_STATUS];
}

void nwd33c9x_device::indir_addr_w(uint8_t data)
{
	m_addr = data & REGS_MASK;
}

uint8_t nwd33c9x_device::indir_reg_r()
{
	uint8_t ret;
	switch (m_addr) {
	case DATA: {
		if (!(m_regs[AUXILIARY_STATUS] & AUXILIARY_STATUS_DBR)) {
			if (!machine().side_effects_disabled())
				fatalerror("%s: The host should never access the data register without DBR set.\n", shortname());
			return 0;
		}
		bool was_full = data_fifo_full();
		ret = data_fifo_pop();
		if (data_fifo_empty())
			m_regs[AUXILIARY_STATUS] &= ~AUXILIARY_STATUS_DBR;
		if (was_full)
			step_entry(false);
		break;
	}

	default:
		if (m_addr == OWN_ID) {
			ret = m_command_length;
		}
		else {
			ret = m_regs[m_addr];
		}


		if (m_addr == SCSI_STATUS) {
			update_irq();
		}

		if (m_addr != COMMAND && m_addr != AUXILIARY_STATUS) {
			m_addr = (m_addr + 1) & REGS_MASK;
		}
		break;
	}

	return ret;
}

void nwd33c9x_device::indir_reg_w(uint8_t data)
{
	switch (m_addr) {
	case SCSI_STATUS:
	case QUEUE_TAG:
	case INVALID_1B:
	case INVALID_1C:
	case INVALID_1D:
	case INVALID_1E:
	case AUXILIARY_STATUS:
		LOGMASKED(LOG_WRITES | LOG_ERRORS, "Write to read-only register address %d (data=%02x)\n", m_addr, data);
		break;

	case COMMAND: {
		if (m_regs[AUXILIARY_STATUS] & (AUXILIARY_STATUS_INT | AUXILIARY_STATUS_CIP)) {
			logerror("%s: The host should never write to the command register when INT or CIP are set.\n", shortname());
		}

		const uint8_t cc = (data & COMMAND_CC);
		if (cc == COMMAND_CC_SET_IDI) {
			m_regs[CONTROL] |= CONTROL_IDI;
			break;
		}

		if (cc > COMMAND_CC_DISCONNECT && (m_regs[AUXILIARY_STATUS] & AUXILIARY_STATUS_BSY)) {
			fatalerror("%s: The host should never issue a Level II command when BSY is set.\n", shortname());
		}

		m_regs[COMMAND] = data;
		start_command();
	} break;

	case DATA:
		if (!(m_regs[AUXILIARY_STATUS] & AUXILIARY_STATUS_DBR)) {
			fatalerror("%s: The host should never write the data register without DBR set.\n", shortname());
		}
		m_regs[AUXILIARY_STATUS] &= ~AUXILIARY_STATUS_DBR;
		data_fifo_push(data);
		decrement_transfer_count();
		step_entry(false);
		break;

	default:
		if (m_addr == OWN_ID) {
			m_command_length = data;
		}
		else {
			m_regs[m_addr] = data;
		}
		m_addr = (m_addr + 1) & REGS_MASK;
		break;
	}
}

void nwd33c9x_device::reset_w(int state)
{
	if (state) {
		LOGMASKED(LOG_LINES, "Reset via MR line\n");
		device_reset();
	}
}

uint8_t nwd33c9x_device::dma_r()
{
	const uint8_t ret = data_fifo_pop();
	decrement_transfer_count();
	clear_drq();
	return ret;
}

void nwd33c9x_device::dma_w(const uint8_t data)
{
	data_fifo_push(data);
	decrement_transfer_count();
	clear_drq();
}

//**************************************************************************
//  Bus-line glue - see nwd33c9x.h's header comment for the design.
//**************************************************************************

void nwd33c9x_device::i_ctrl_w(uint32_t lines, uint32_t mask)
{
	if (m_t_active_disk < 0) {
		// Routed to a configured CD-ROM, or an unconfigured ID (which,
		// with nothing attached there on the real aux bus either, will
		// simply never assert BSY - a real SELECTION_TIMEOUT results,
		// exactly like today's embedded-only behavior for an empty ID).
		m_aux_bridge->bridge_ctrl_w(lines, mask);
		return;
	}
	const uint32_t old_composite = ctrl_r();
	m_i_ctrl = (m_i_ctrl & ~mask) | (lines & mask);
	const uint32_t changed = old_composite ^ ctrl_r();
	if (changed && (m_t_wait_ctrl & changed)) {
		// Direct call replacing nscsi_bus_device::regen_ctrl()'s per-
		// notification vtable-override check + device-array dispatch
		// loop - this is nscsi_full_device::scsi_ctrl_changed()'s exact
		// behavior (no RST short-circuit at this entry point; RST is
		// handled inside t_step() itself, matching the original).
		t_step(false);
	}
}

void nwd33c9x_device::i_data_w(uint32_t lines)
{
	if (m_t_active_disk < 0) {
		m_aux_bridge->bridge_data_w(lines);
		return;
	}
	m_i_data = lines;
}

void nwd33c9x_device::i_ctrl_wait(uint32_t lines, uint32_t mask)
{
	if (m_t_active_disk < 0) {
		m_aux_bridge->bridge_ctrl_wait(lines, mask);
		return;
	}
	m_i_wait_ctrl = (m_i_wait_ctrl & ~mask) | (lines & mask);
}

void nwd33c9x_device::aux_ctrl_changed()
{
	// The real aux nscsi_bus_device already did its own regen_ctrl()/
	// notify-if-waited-on check (using our bridge's own wait_ctrl, set by
	// i_ctrl_wait() above) before calling this - unlike t_ctrl_w()'s
	// embedded-disk path, there's no separate relevance check to redo
	// here. Otherwise this is wd33c9x_base_device::scsi_ctrl_changed()'s
	// exact behavior.
	if (ctrl_r() & S_RST) {
		LOGMASKED(LOG_LINES, "scsi bus reset (aux)\n");
		return;
	}
	step_entry(false);
}

void nwd33c9x_device::t_ctrl_w(uint32_t lines, uint32_t mask)
{
	const uint32_t old_composite = ctrl_r();
	m_t_ctrl = (m_t_ctrl & ~mask) | (lines & mask);
	const uint32_t changed = old_composite ^ ctrl_r();
	if (changed && (m_i_wait_ctrl & changed)) {
		// wd33c9x_base_device::scsi_ctrl_changed()'s exact behavior.
		if (ctrl_r() & S_RST) {
			LOGMASKED(LOG_LINES, "scsi bus reset\n");
			return;
		}
		step_entry(false);
	}
}

//**************************************************************************
//  Initiator half - forked from wd33c9x_base_device. Only the m_scsi_bus-
//  touching call sites differ from the original; everything else (already
//  correct, already validated) is copied as-is.
//**************************************************************************

static const char * select_strings[4] = {
	"Select-w/Atn",
	"Select",
	"Select-w/Atn-and-Transfer",
	"Select-and-Transfer"
};

void nwd33c9x_device::determine_active_disk()
{
	const int destid = m_regs[DESTINATION_ID] & DESTINATION_ID_DI;
	m_t_active_disk = (destid < NUM_DISK_SLOTS && m_disk_image[destid] && m_disk_image[destid]->exists()) ? destid : -1;
}

void nwd33c9x_device::start_command()
{
	const uint8_t cc = m_regs[COMMAND] & COMMAND_CC;
#ifdef MAME_SGI_SCSI_LOG_DIAG
	if (FILE *const f = scsi_log_diag_file())
	{
		fprintf(f, "%.6f %s CMD cc=%02x dest=%02x", machine().time().as_double(), tag(), cc, m_regs[DESTINATION_ID]);
		if (cc == COMMAND_CC_SELECT_TRANSFER || cc == COMMAND_CC_SELECT_ATN_TRANSFER)
			for (int i = 0; i < 12; i++)
				fprintf(f, " %02x", m_regs[CDB_1 + i]);
		fprintf(f, "\n");
		fflush(f);
	}
#endif

	if (cc > COMMAND_CC_DISCONNECT && cc != COMMAND_CC_SET_IDI) {
		m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_BSY;
	}

	switch (cc) {
	case COMMAND_CC_RESET:
		LOGMASKED(LOG_COMMANDS, "Reset Command\n");
		i_ctrl_w(0, S_ALL);
		i_ctrl_wait(S_SEL|S_BSY|S_RST, S_ALL);
		m_regs[OWN_ID] = m_command_length;
		memset(&m_regs[CONTROL], 0, SOURCE_ID - CONTROL);
		m_regs[COMMAND] = 0;
		m_regs[AUXILIARY_STATUS] &= ~AUXILIARY_STATUS_DBR;
		m_mode = MODE_D;
		data_fifo_reset();
		irq_fifo_reset();
		update_irq();
		set_i_state(FINISHED);
		irq_fifo_push((m_regs[OWN_ID] & OWN_ID_EAF) ? SCSI_STATUS_RESET_EAF : SCSI_STATUS_RESET);
		m_i_scsi_id = (m_regs[OWN_ID] & OWN_ID_M_SCSI_ID);
		step_entry(false);
		break;

	case COMMAND_CC_ABORT:
		LOGMASKED(LOG_COMMANDS, "Abort Command\n");
		set_i_state(FINISHED);
		irq_fifo_push((m_regs[OWN_ID] & OWN_ID_EAF) ? SCSI_STATUS_RESET_EAF : SCSI_STATUS_RESET);
		break;

	case COMMAND_CC_ASSERT_ATN:
		LOGMASKED(LOG_COMMANDS, "Assert ATN Command\n");
		if (m_mode != MODE_I) {
			fatalerror("%s: ASSERT_ATN command only valid in the Initiator state.", shortname());
		}
		i_ctrl_w(S_ATN, S_ATN);
		return;

	case COMMAND_CC_NEGATE_ACK:
		LOGMASKED(LOG_COMMANDS, "Negate ACK Command\n");
		if (m_mode != MODE_I) {
			data_fifo_reset();
			irq_fifo_push(SCSI_STATUS_DISCONNECT);
			update_irq();
		}
		i_ctrl_w(0, S_ACK);
		return;

	case COMMAND_CC_DISCONNECT:
		LOGMASKED(LOG_COMMANDS, "Disconnect Command\n");
		i_ctrl_w(0, S_ALL);
		i_ctrl_wait(S_SEL|S_BSY|S_RST, S_ALL);
		m_mode = MODE_D;
		set_i_state(IDLE);
		m_regs[AUXILIARY_STATUS] &= ~(AUXILIARY_STATUS_CIP | AUXILIARY_STATUS_BSY);
		break;

	case COMMAND_CC_SELECT:
	case COMMAND_CC_SELECT_ATN:
		LOGMASKED(LOG_COMMANDS, "%s Command\n", select_strings[cc - COMMAND_CC_SELECT_ATN]);
		if (m_mode != MODE_D) {
			fatalerror("Select commands only valid in the Disconnected state.");
		}
		determine_active_disk();
		set_i_state((ARB_WAIT_BUS_FREE << SUB_SHIFT) | DISC_SEL_ARBITRATION);
		step_entry(false);
		break;

	case COMMAND_CC_SELECT_TRANSFER:
	case COMMAND_CC_SELECT_ATN_TRANSFER:
		LOGMASKED(LOG_COMMANDS, "%s Command\n", select_strings[cc - COMMAND_CC_SELECT_ATN]);
		if (m_mode == MODE_D) {
			determine_active_disk();
			set_i_state((ARB_WAIT_BUS_FREE << SUB_SHIFT) | DISC_SEL_ARBITRATION);
			m_regs[COMMAND_PHASE] = COMMAND_PHASE_ZERO;
		}
		else if (m_mode == MODE_I) {
			set_i_state(INIT_XFR);
		}
		else {
			fatalerror("%s: Select-and-Transfer commands only valid in the Disconnected and Initiator states.", shortname());
		}
		set_command_length(cc);
		load_transfer_count();
		step_entry(false);
		break;

	case COMMAND_CC_TRANSLATE_ADDRESS:
		LOGMASKED(LOG_COMMANDS, "Translate Address Command\n");
		{
			uint8_t total_sectors = m_regs[CDB_1];
			uint8_t total_heads = m_regs[CDB_2];
			uint16_t total_cylinders = m_regs[CDB_3] << 8 | m_regs[CDB_4];
			uint32_t lba = (m_regs[CDB_5] << 24) | (m_regs[CDB_6] << 16) | (m_regs[CDB_7] << 8) | (m_regs[CDB_8] << 0);

			uint16_t cylinder = lba / (total_sectors * total_heads);
			uint8_t head = (lba - (cylinder * total_sectors * total_heads)) / total_sectors;
			uint8_t sector = (lba - (cylinder * total_sectors * total_heads)) % total_sectors;

			m_regs[CDB_9] = sector;
			m_regs[CDB_10] = head;
			m_regs[CDB_11] = cylinder >> 8;
			m_regs[CDB_12] = cylinder >> 0;

			m_regs[AUXILIARY_STATUS] &= ~(AUXILIARY_STATUS_CIP | AUXILIARY_STATUS_BSY);

			if (cylinder >= total_cylinders)
				irq_fifo_push(SCSI_STATUS_LOGICAL_ADDRESS_TOO_LARGE);
			else
				irq_fifo_push(SCSI_STATUS_TRANSLATE_SUCCESS);

			update_irq();
		}
		break;

	case COMMAND_CC_TRANSFER_INFO:
		LOGMASKED(LOG_COMMANDS, "Transfer Info Command\n");
		if (m_mode != MODE_I) {
			fatalerror("%s: TRANSFER_INFO command only valid in the Initiator state.", shortname());
		}
		m_regs[AUXILIARY_STATUS] &= ~AUXILIARY_STATUS_DBR;
		set_i_state(INIT_XFR);
		set_command_length(COMMAND_CC_TRANSFER_INFO);
		load_transfer_count();
		m_xfr_phase = (ctrl_r() & S_PHASE_MASK);
		step_entry(false);
		return;

	default:
		fatalerror("%s: Unimplemented command: 0x%02x", shortname(), cc);
		break;
	}
}

static const char * phase_strings[8] = {
	"DATA_OUT",
	"DATA_IN",
	"COMMAND",
	"STATUS",
	"INVALID_4",
	"INVALID_5",
	"MSG_OUT",
	"MSG_IN",
};

void nwd33c9x_device::step_entry(bool timeout)
{
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	// See wd33c9x.h's identical comment - unchanged rationale. The
	// cross-device reentrancy this guards against still applies in the
	// flat design (see nwd33c9x.h's header comment): t_ctrl_w() calling
	// back into step_entry() synchronously, nested inside this call's
	// own still-running loop, is possible whenever the target's reaction
	// to our own line change itself changes a line we're waiting on.
	if (m_step_entry_depth > 0) {
		if (m_step_entry_depth > 100) {
			fatalerror("%s: step_entry() reentrancy depth exceeded 100 - unexpected nesting shape.\n", shortname());
		}
		++m_step_entry_depth;
		step(timeout);
		--m_step_entry_depth;
		return;
	}

	++m_step_entry_depth;
	do {
		m_coarse_continue = false;
		step(timeout);
		timeout = true;
	} while (m_coarse_continue);
	--m_step_entry_depth;
#else
	step(timeout);
#endif
}

void nwd33c9x_device::step(bool timeout)
{
	const uint8_t cc = (m_regs[COMMAND] & COMMAND_CC);
	const bool sat = (cc == COMMAND_CC_SELECT_TRANSFER || cc == COMMAND_CC_SELECT_ATN_TRANSFER);

	const uint32_t ctrl = ctrl_r();
	const uint32_t data = data_r();

	LOGMASKED(LOG_STEP,
			  "%s: step - PHASE:%s BSY:%x SEL:%x REQ:%x ACK:%x ATN:%x RST:%x DATA:%02x %s\n",
			  shortname(),
			  phase_strings[ctrl & S_PHASE_MASK],
			  (ctrl & S_BSY) ? 1 : 0,
			  (ctrl & S_SEL) ? 1 : 0,
			  (ctrl & S_REQ) ? 1 : 0,
			  (ctrl & S_ACK) ? 1 : 0,
			  (ctrl & S_ATN) ? 1 : 0,
			  (ctrl & S_RST) ? 1 : 0,
			  data,
			  (timeout) ? "timeout" : "change"
			  );

	if (m_mode == MODE_I) {
		if (ctrl & S_BSY) {
			if (ctrl & S_REQ) {
				uint8_t xfr_phase = (ctrl & S_PHASE_MASK);
				switch (m_i_state) {
				case DISC_SEL_ARBITRATION:
					m_xfr_phase = xfr_phase;
					break;

				case INIT_XFR_WAIT_REQ:
					break;

				default:
					if (m_xfr_phase != xfr_phase) {
						fatalerror("%s: Unexpected phase change during state.\n", shortname());
					}
					break;
				}
			}
		} else {
			LOGMASKED(LOG_STATE, "Target disconnected\n");
			if (sat) {
				switch (m_regs[COMMAND_PHASE]) {
				case COMMAND_PHASE_DISCONNECT_MESSAGE:
					set_i_state(FINISHED);
					m_regs[COMMAND_PHASE] = COMMAND_PHASE_DISCONNECTED;
					break;

				case COMMAND_PHASE_COMMAND_COMPLETE:
					if (m_regs[CONTROL] & CONTROL_EDI) {
						set_i_state(FINISHED);
						irq_fifo_push(SCSI_STATUS_SELECT_TRANSFER_SUCCESS);
					} else {
						set_i_state(FINISHED);
						irq_fifo_push(SCSI_STATUS_DISCONNECT);
					}
					break;

				default:
					fatalerror("%s: Unhandled command phase during Select-and-Transfer disconnect.\n", shortname());
					break;
				}
			} else {
				set_i_state(FINISHED);
				irq_fifo_push(SCSI_STATUS_DISCONNECT);
			}
			m_mode = MODE_D;
			i_ctrl_w(0, S_ALL);
			i_ctrl_wait(S_SEL|S_BSY|S_RST, S_ALL);
		}
	}

	switch (m_i_state & SUB_MASK ? m_i_state & SUB_MASK : m_i_state & STATE_MASK) {
	case IDLE:
		break;

	case FINISHED:
		set_i_state(IDLE);
		m_regs[AUXILIARY_STATUS] &= ~(AUXILIARY_STATUS_CIP | AUXILIARY_STATUS_BSY);
		update_irq();
		break;

	case ARB_WAIT_BUS_FREE << SUB_SHIFT:
		if (!(ctrl & (S_BSY | S_SEL))) {
			set_i_state_sub(ARB_CHECK_FREE);
			delay_arb_poll();
		}
		break;

	case ARB_CHECK_FREE << SUB_SHIFT:
		if (ctrl & (S_BSY | S_SEL)) {
			set_i_state_sub(ARB_CHECK_FREE);
			break;
		}
		if (timeout) {
			i_data_w(1 << m_i_scsi_id);
			i_ctrl_w(S_BSY, S_BSY);
			set_i_state_sub(ARB_EXAMINE_BUS);
			delay_arb_poll();
		}
		break;

	case ARB_EXAMINE_BUS << SUB_SHIFT:
		if (timeout) {
			if (ctrl & S_SEL) {
				i_ctrl_w(0, S_BSY);
				i_data_w(0);
				set_i_state_sub(ARB_WAIT_BUS_FREE);
			} else {
				int win;
				for (win = 7; win >=0 && !(data & (1 << win)); win--);
				if (win == m_i_scsi_id) {
					i_ctrl_w(S_SEL, S_SEL);
					set_i_state_sub(ARB_ASSERT_SEL);
					delay_arb_poll();
				} else {
					i_data_w(0);
					i_ctrl_w(0, S_ALL);
					set_i_state_sub(ARB_CHECK_FREE);
				}
			}
		}
		break;

	case ARB_ASSERT_SEL << SUB_SHIFT:
		if (timeout) {
			i_data_w((1 << m_i_scsi_id) | (1 << (m_regs[DESTINATION_ID] & DESTINATION_ID_DI)));
			set_i_state_sub(ARB_SET_DEST);
			delay_arb_poll();
		}
		break;

	case ARB_SET_DEST << SUB_SHIFT:
		if (timeout) {
			i_ctrl_w((cc == COMMAND_CC_SELECT_ATN || cc == COMMAND_CC_SELECT_ATN_TRANSFER) ? S_ATN : 0, S_ATN | S_BSY);
			set_i_state_sub(ARB_RELEASE_BUSY);
			delay_arb_poll();
		}
		break;

	case ARB_RELEASE_BUSY << SUB_SHIFT:
		if (timeout) {
			if (ctrl & S_BSY) {
				set_i_state_sub(ARB_DESKEW_WAIT);
				if (cc == COMMAND_CC_RESELECT) {
					i_ctrl_w(S_BSY, S_BSY);
				}
				delay_arb_poll();
			} else {
				set_i_state_sub(ARB_TIMEOUT_BUSY);
				delay_arb_poll(); // Should be the select timeout...
			}
		}
		break;

	case ARB_DESKEW_WAIT << SUB_SHIFT:
		if (timeout) {
			i_data_w(0);
			i_ctrl_w(0, S_SEL);
			m_mode = (cc == COMMAND_CC_RESELECT) ? MODE_T : MODE_I;
			set_i_state_sub(0);
			step(true);
		}
		break;

	case ARB_TIMEOUT_BUSY << SUB_SHIFT:
		if (timeout) {
			i_data_w(0);
			set_i_state_sub(ARB_TIMEOUT_ABORT);
			delay(1000);
		} else if (ctrl & S_BSY) {
			set_i_state_sub(ARB_DESKEW_WAIT);
			if (cc == COMMAND_CC_RESELECT) {
				i_ctrl_w(S_BSY, S_BSY);
			}
			delay_arb_poll();
		}
		break;

	case ARB_TIMEOUT_ABORT << SUB_SHIFT:
		if (timeout) {
			if (ctrl & S_BSY) {
				set_i_state_sub(ARB_DESKEW_WAIT);
				if (cc == COMMAND_CC_RESELECT) {
					i_ctrl_w(S_BSY, S_BSY);
				}
				delay_arb_poll();
			} else {
				i_ctrl_w(0, S_ALL);
				i_ctrl_wait(S_SEL|S_BSY|S_RST, S_ALL);
				m_regs[AUXILIARY_STATUS] &= ~(AUXILIARY_STATUS_CIP | AUXILIARY_STATUS_BSY);
				m_mode = MODE_D;
				set_i_state(IDLE);
				irq_fifo_push(SCSI_STATUS_SELECTION_TIMEOUT);
				update_irq();
			}
		}
		break;

	case SEND_WAIT_SETTLE << SUB_SHIFT:
		if (timeout) {
			set_i_state_sub(SEND_WAIT_REQ_0);
			step(false);
		}
		break;

	case SEND_WAIT_REQ_0 << SUB_SHIFT:
		if (!(ctrl & S_REQ)) {
			set_i_state_sub(0);
			i_data_w(0);
			i_ctrl_w(0, S_ACK);
			if (sat) {
				switch (m_xfr_phase) {
				case S_PHASE_COMMAND:
					++m_regs[COMMAND_PHASE];
					break;
				}
			}
			step(false);
		}
		break;

	case RECV_WAIT_REQ_1 << SUB_SHIFT:
		if (ctrl & S_REQ) {
			set_i_state_sub(RECV_WAIT_SETTLE);
			delay(1);
		}
		break;

	case RECV_WAIT_SETTLE << SUB_SHIFT:
		if (timeout) {
			if (sat) {
				switch (m_xfr_phase) {
				case S_PHASE_DATA_IN:
					data_fifo_push(data);
					if ((m_regs[CONTROL] & CONTROL_DM) != CONTROL_DM_POLLED) {
						set_drq();
					} else {
						decrement_transfer_count();
						m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
					}
					break;

				case S_PHASE_STATUS:
					m_regs[TARGET_LUN] = data;
					m_regs[COMMAND_PHASE] = COMMAND_PHASE_STATUS_RECEIVED;
					break;

				case S_PHASE_MSG_IN:
					m_last_message = data;
					break;

				default:
					fatalerror("%s: Unexpected phase in RECV_WAIT_SETTLE.\n", shortname());
					break;
				}
			} else {
				data_fifo_push(data);
				if (m_xfr_phase == S_PHASE_DATA_IN && (m_regs[CONTROL] & CONTROL_DM) != CONTROL_DM_POLLED) {
					set_drq();
				} else {
					decrement_transfer_count();
					m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
				}
			}
			set_i_state_sub(RECV_WAIT_REQ_0);
			i_ctrl_w(S_ACK, S_ACK);
			step(false);
		}
		break;

	case RECV_WAIT_REQ_0 << SUB_SHIFT:
		if (!(ctrl & S_REQ)) {
			set_i_state_sub(0);
			step(false);
		}
		break;

	case DISC_SEL_ARBITRATION:
		i_ctrl_wait(S_REQ, S_REQ);
		if (cc == COMMAND_CC_SELECT || cc == COMMAND_CC_SELECT_ATN) {
			set_i_state(FINISHED);
			irq_fifo_push(SCSI_STATUS_SELECT_SUCCESS);
			if (ctrl & S_REQ) {
				irq_fifo_push(SCSI_STATUS_REQ | m_xfr_phase);
			}
		} else {
			if (cc == COMMAND_CC_SELECT_TRANSFER) {
				m_regs[COMMAND_PHASE] = COMMAND_PHASE_CP_BYTES_0;
				for (uint8_t i = 0; i < m_command_length; ++i) {
					data_fifo_push(m_regs[CDB_1 + i]);
				}
				if (VERBOSE & LOG_COMMANDS) {
					std::string cmd;
					for (uint8_t i = 0; i < m_command_length; ++i) {
						cmd += util::string_format(" %02x", m_regs[CDB_1 + i]);
					}
					LOGMASKED(LOG_COMMANDS, "Sending command:%s (%d)\n", cmd, m_transfer_count);
				}
			} else
				m_regs[COMMAND_PHASE] = COMMAND_PHASE_SELECTED;
			set_i_state(INIT_XFR);
		}
		step(false);
		break;

	case INIT_XFR:
		if (ctrl & S_REQ) {
			switch (m_xfr_phase) {
			case S_PHASE_DATA_OUT:
				if ((m_regs[CONTROL] & CONTROL_DM) != CONTROL_DM_POLLED) {
					if (!data_fifo_full() && m_transfer_count > 0)
						set_drq();
				}
				if (!data_fifo_empty()) {
					set_i_state(INIT_XFR_WAIT_REQ);
					delay(send_byte());
				} else if ((m_regs[CONTROL] & CONTROL_DM) == CONTROL_DM_POLLED) {
					m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
				} else {
					delay(1);
				}
				break;

			case S_PHASE_COMMAND:
				if (!data_fifo_empty()) {
					uint32_t mask;
					if (sat) {
						mask = 0;
					} else {
						m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
						mask = (m_transfer_count == 0 && m_data_fifo_size == 1) ? S_ATN : 0;
					}
					set_i_state(INIT_XFR_WAIT_REQ);
					delay(send_byte(0, mask));
				} else if (!sat) {
					m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
				}
				break;

			case S_PHASE_MSG_OUT:
				if (sat) {
					data_fifo_push(get_msg_out());
				}
				if (!data_fifo_empty()) {
					uint32_t mask;
					if (sat) {
						mask = S_ATN;
					} else {
						m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
						mask = (m_transfer_count == 0 && m_data_fifo_size == 1) ? S_ATN : 0;
					}
					set_i_state(INIT_XFR_WAIT_REQ);
					delay(send_byte(0, mask));
				} else if (!sat) {
					m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_DBR;
				}
				break;

			case S_PHASE_DATA_IN:
			case S_PHASE_STATUS:
			case S_PHASE_MSG_IN:
				if (!data_fifo_full()) {
					if (m_drq_state) {
						delay(1);
					} else {
						i_ctrl_wait(S_REQ, S_REQ);
						set_i_state((RECV_WAIT_REQ_1 << SUB_SHIFT) | INIT_XFR_RECV_BYTE_ACK);
						step(false);
					}
				}
				break;

			default:
				fatalerror("%s: Invalid phase during INIT_XFR.\n", shortname());
				break;
			}
		}
		break;

	case INIT_XFR_WAIT_REQ:
		if (ctrl & S_REQ) {
			uint16_t next_state = m_i_state;

			const uint8_t xfr_phase = (ctrl & S_PHASE_MASK);

			switch ((m_xfr_phase << 3) | xfr_phase) {
			case ((S_PHASE_MSG_OUT << 3) | S_PHASE_MSG_OUT):
			case ((S_PHASE_COMMAND << 3) | S_PHASE_COMMAND):
			case ((S_PHASE_MSG_IN  << 3) | S_PHASE_MSG_IN):
				next_state = INIT_XFR;
				break;

			case ((S_PHASE_DATA_IN  << 3) | S_PHASE_DATA_IN):
			case ((S_PHASE_DATA_OUT << 3) | S_PHASE_DATA_OUT):
				if (sat || cc == COMMAND_CC_TRANSFER_INFO) {
					if (m_transfer_count > 0 || (m_xfr_phase == S_PHASE_DATA_OUT && !data_fifo_empty())) {
						next_state = INIT_XFR;
					}
					else {
						next_state = FINISHED;
						uint8_t scsi_status;
						if (sat) {
							m_regs[COMMAND_PHASE] = COMMAND_PHASE_TRANSFER_COUNT;
							scsi_status = SCSI_STATUS_UNEXPECTED_PHASE;
						}
						else {
							scsi_status = SCSI_STATUS_TRANSFER_SUCCESS;
						}
						irq_fifo_push(scsi_status | m_xfr_phase);
					}
				}
				else {
					fatalerror("%s: Unhandled command in data phase.\n", shortname());
					next_state = FINISHED;
				}
				break;

			case ((S_PHASE_MSG_OUT  << 3) | S_PHASE_COMMAND):
			case ((S_PHASE_COMMAND  << 3) | S_PHASE_DATA_OUT):
			case ((S_PHASE_COMMAND  << 3) | S_PHASE_DATA_IN):
			case ((S_PHASE_COMMAND  << 3) | S_PHASE_STATUS):
			case ((S_PHASE_COMMAND  << 3) | S_PHASE_MSG_IN):
			case ((S_PHASE_DATA_OUT << 3) | S_PHASE_STATUS):
			case ((S_PHASE_DATA_IN  << 3) | S_PHASE_STATUS):
			case ((S_PHASE_STATUS   << 3) | S_PHASE_MSG_IN):
				if (!(m_xfr_phase & 1) && !data_fifo_empty()) {
					fatalerror("%s: Data FIFO is not empty on phase transition.\n", shortname());
				}

				if (sat) {
					switch (xfr_phase) {
					case S_PHASE_MSG_OUT:
						next_state = INIT_XFR;
						break;

					case S_PHASE_COMMAND: {
						next_state = INIT_XFR;
						m_regs[COMMAND_PHASE] = COMMAND_PHASE_CP_BYTES_0;
						for (uint8_t i = 0; i < m_command_length; ++i) {
							data_fifo_push(m_regs[CDB_1 + i]);
						}
						if (VERBOSE & LOG_COMMANDS) {
							std::string cmd;
							for (uint8_t i = 0; i < m_command_length; ++i) {
								cmd += util::string_format(" %02x", m_regs[CDB_1 + i]);
							}
							LOGMASKED(LOG_COMMANDS, "Sending command:%s (%d)\n", cmd, m_transfer_count);
						}
						break;
					}

					case S_PHASE_DATA_OUT:
					case S_PHASE_DATA_IN:
						next_state = INIT_XFR;
						break;

					case S_PHASE_STATUS:
						next_state = INIT_XFR;
						m_regs[COMMAND_PHASE] = COMMAND_PHASE_RECEIVE_STATUS;
						break;

					case S_PHASE_MSG_IN:
						next_state = INIT_XFR;
						break;

					default:
						fatalerror("%s: Unhandled phase in Select-w/Atn-and-Transfer.\n", shortname());
						next_state = FINISHED;
						break;
					}
				}
				else if (cc == COMMAND_CC_TRANSFER_INFO) {
					next_state = FINISHED;
					irq_fifo_push(SCSI_STATUS_TRANSFER_SUCCESS | xfr_phase);
				}
				else {
					fatalerror("%s: Unhandled command in data phase.\n", shortname());
					next_state = FINISHED;
				}
				break;

			default:
				fatalerror("%s: Unhandled phase transition in INIT_XFR_WAIT_REQ.\n", shortname());
				next_state = FINISHED;
				break;
			}

			if (next_state != m_i_state) {
				set_i_state(next_state);
				m_xfr_phase = xfr_phase;
				step(false);
			}
		}
		break;

	case INIT_XFR_RECV_BYTE_ACK:
		if (sat && m_xfr_phase == S_PHASE_MSG_IN) {
			if (m_regs[COMMAND_PHASE] <= COMMAND_PHASE_CP_BYTES_C) {
				switch (m_last_message) {
				case SM_SAVE_DATA_POINTER:
					set_i_state(FINISHED);
					irq_fifo_push(SCSI_STATUS_SAVE_DATA_POINTERS);
					m_regs[COMMAND_PHASE] = COMMAND_PHASE_SAVE_DATA_POINTER;
					break;

				case SM_DISCONNECT:
					m_regs[COMMAND_PHASE] = COMMAND_PHASE_DISCONNECT_MESSAGE;
					break;

				default:
					fatalerror("%s: Unhandled MSG_IN %02x.\n", shortname(), m_last_message);
					break;
				}
			} else if (m_regs[COMMAND_PHASE] < COMMAND_PHASE_COMMAND_COMPLETE) {
				switch (m_last_message) {
				case SM_COMMAND_COMPLETE:
					set_i_state(FINISHED);
					irq_fifo_push(SCSI_STATUS_SELECT_TRANSFER_SUCCESS);
					m_regs[COMMAND_PHASE] = COMMAND_PHASE_COMMAND_COMPLETE;
					break;
				default:
					fatalerror("%s: Unhandled MSG_IN %02x.\n", shortname(), m_last_message);
					break;
				}
			}
		} else {
			set_i_state(INIT_XFR_WAIT_REQ);
		}
		i_ctrl_w(0, S_ACK);
		step(false);
		break;

	default:
		fatalerror("%s: Unhandled state in step.\n", shortname());
		break;
	}
}

void nwd33c9x_device::load_transfer_count()
{
	if (m_regs[COMMAND] & COMMAND_SBT) {
		m_transfer_count = 1;
	}
	else {
		m_transfer_count = (
			(uint32_t(m_regs[TRANSFER_COUNT_MSB]) << 16) |
			(uint32_t(m_regs[TRANSFER_COUNT])     <<  8) |
			(uint32_t(m_regs[TRANSFER_COUNT_LSB]) <<  0)
		);
		if (m_transfer_count == 0) {
			m_transfer_count = 1;
		}
	}
	LOGMASKED(LOG_COMMANDS, "Transfer Count %d bytes\n", m_transfer_count);
}

bool nwd33c9x_device::decrement_transfer_count()
{
	if (m_transfer_count == 0) {
		return true;
	}
	--m_transfer_count;
	if (m_transfer_count == 0) {
		m_regs[TRANSFER_COUNT_MSB] = 0;
		m_regs[TRANSFER_COUNT] = 0;
		m_regs[TRANSFER_COUNT_LSB] = 0;
		return true;
	}
	return false;
}

uint8_t nwd33c9x_device::data_fifo_pop()
{
	if (data_fifo_empty()) {
		fatalerror("%s: Data FIFO underflow.\n", shortname());
	}
	--m_data_fifo_size;
	uint8_t ret = m_data_fifo[m_data_fifo_pos];
	m_data_fifo_pos = (m_data_fifo_pos + 1) % DATA_FIFO_SIZE;
	return ret;
}

void nwd33c9x_device::data_fifo_push(const uint8_t data)
{
	if (data_fifo_full()) {
		fatalerror("%s: Data FIFO overflow.\n", shortname());
	}
	m_data_fifo[(m_data_fifo_pos + m_data_fifo_size) % DATA_FIFO_SIZE] = data;
	++m_data_fifo_size;
}

bool nwd33c9x_device::data_fifo_empty() const
{
	return m_data_fifo_size <= 0;
}

bool nwd33c9x_device::data_fifo_full() const
{
	return m_data_fifo_size >= DATA_FIFO_SIZE;
}

void nwd33c9x_device::data_fifo_reset()
{
	memset(m_data_fifo, 0, sizeof(m_data_fifo));
	m_data_fifo_pos = 0;
	m_data_fifo_size = 0;
}

uint32_t nwd33c9x_device::send_byte(const uint32_t value, const uint32_t mask)
{
	set_i_state_sub(SEND_WAIT_SETTLE);
	i_ctrl_wait(S_REQ, S_REQ);
	i_data_w(data_fifo_pop());
	i_ctrl_w(S_ACK | value, S_ACK | mask);
	return 1;
}

void nwd33c9x_device::set_i_state(uint16_t state)
{
	LOGMASKED(LOG_STEP, "SCSI initiator state change: %d.%d to %d.%d\n",
			  m_i_state & STATE_MASK, m_i_state >> SUB_SHIFT,
			  state & STATE_MASK, state >> SUB_SHIFT);
	m_i_state = state;
}

void nwd33c9x_device::set_i_state_sub(uint8_t sub)
{
	set_i_state((m_i_state & STATE_MASK) | (uint16_t(sub) << SUB_SHIFT));
}

uint8_t nwd33c9x_device::irq_fifo_pop()
{
	if (irq_fifo_empty()) {
		fatalerror("%s: IRQ FIFO underflow.\n", shortname());
	}
	--m_irq_fifo_size;
	uint8_t ret = m_irq_fifo[m_irq_fifo_pos];
	m_irq_fifo_pos = (m_irq_fifo_pos + 1) % IRQ_FIFO_SIZE;
	return ret;
}

void nwd33c9x_device::irq_fifo_push(const uint8_t status)
{
	if (irq_fifo_full()) {
		fatalerror("%s: IRQ FIFO overflow.\n", shortname());
	}
	if (m_irq_fifo_size &&
		m_irq_fifo[(m_irq_fifo_pos + m_irq_fifo_size - 1) % IRQ_FIFO_SIZE] == status) {
		return;
	}
	m_irq_fifo[(m_irq_fifo_pos + m_irq_fifo_size) % IRQ_FIFO_SIZE] = status;
	++m_irq_fifo_size;
}

bool nwd33c9x_device::irq_fifo_empty() const
{
	return m_irq_fifo_size <= 0;
}

bool nwd33c9x_device::irq_fifo_full() const
{
	return m_irq_fifo_size >= IRQ_FIFO_SIZE;
}

void nwd33c9x_device::irq_fifo_reset()
{
	memset(m_irq_fifo, 0, sizeof(m_irq_fifo));
	m_irq_fifo_pos = 0;
	m_irq_fifo_size = 0;
}

void nwd33c9x_device::update_irq()
{
	if (m_regs[AUXILIARY_STATUS] & AUXILIARY_STATUS_INT) {
		m_regs[AUXILIARY_STATUS] &= ~AUXILIARY_STATUS_INT;
		LOGMASKED(LOG_LINES, "Clearing IRQ\n");
		m_irq_cb(CLEAR_LINE);
	}
	if (!irq_fifo_empty()) {
		m_regs[SCSI_STATUS] = irq_fifo_pop();
		m_regs[AUXILIARY_STATUS] |= AUXILIARY_STATUS_INT;

		const uint8_t cc = (m_regs[COMMAND] & COMMAND_CC);
		if (cc == COMMAND_CC_SELECT_TRANSFER || cc == COMMAND_CC_SELECT_ATN_TRANSFER) {
			switch (m_regs[SCSI_STATUS]) {
			case SCSI_STATUS_DISCONNECT:
				if (!(m_regs[CONTROL] & CONTROL_IDI)) {
#ifdef MAME_SGI_SCSI_LOG_DIAG
					if (FILE *const f = scsi_log_diag_file()) { fprintf(f, "%.6f %s HOLD status=%02x (IDI clear)\n", machine().time().as_double(), tag(), m_regs[SCSI_STATUS]); fflush(f); }
#endif
					return;
				}
				break;

			case SCSI_STATUS_SELECT_TRANSFER_SUCCESS:
				if ((m_regs[CONTROL] & CONTROL_EDI) && m_mode != MODE_D) {
#ifdef MAME_SGI_SCSI_LOG_DIAG
					if (FILE *const f = scsi_log_diag_file()) { fprintf(f, "%.6f %s HOLD status=%02x (EDI)\n", machine().time().as_double(), tag(), m_regs[SCSI_STATUS]); fflush(f); }
#endif
					return;
				}
				break;
			}
		}

		LOGMASKED(LOG_LINES, "Asserting IRQ - SCSI Status (%02x)\n", m_regs[SCSI_STATUS]);
#ifdef MAME_SGI_SCSI_LOG_DIAG
		if (FILE *const f = scsi_log_diag_file()) { fprintf(f, "%.6f %s IRQ status=%02x\n", machine().time().as_double(), tag(), m_regs[SCSI_STATUS]); fflush(f); }
#endif
		m_irq_cb(ASSERT_LINE);
	}
}

void nwd33c9x_device::set_drq()
{
	if (!m_drq_state) {
		LOGMASKED(LOG_LINES, "Asserting DRQ\n");
		m_drq_state = true;
		m_drq_cb(ASSERT_LINE);
	}
}

void nwd33c9x_device::clear_drq()
{
	if (m_drq_state) {
		LOGMASKED(LOG_LINES, "Clearing DRQ\n");
		m_drq_state = false;
		m_drq_cb(CLEAR_LINE);
	}
}

// CORRECTED 2026-09-20, live-boot finding: the unscaled 1-cycle
// delay_arb_poll() below reproduced machine/wd33c9x.cpp's own documented
// SCALE=0 failure exactly (repeated SCSI_STATUS_SELECTION_TIMEOUT, PROM
// "Unable to boot" loop) - ARB_RELEASE_BUSY's poll interval must be wide
// enough for the target's real 400ns bus-settle delay
// (T_TARGET_SELECT_WAIT_BUS_SETTLE) to actually fire before the
// initiator's own arbitration timeout gives up. This is a correctness
// requirement, not a boot-time optimization - the earlier note here
// conflated it with
// the separate, narrower finding that widening SCALE further (50->500)
// doesn't measurably change boot time once *some* adequate margin exists.
// MAME_SGI_SCSI_DELAY_EXPERIMENT/MAME_SGI_SCSI_DELAY_SCALE restored below,
// matching machine/wd33c9x.cpp's own currently-enabled value (SCALE=500).
TIMER_CALLBACK_MEMBER(nwd33c9x_device::update_step)
{
	step_entry(true);
}

void nwd33c9x_device::delay(uint32_t cycles)
{
#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
	m_coarse_cycle_debt += cycles;
	if (coarse_schedule_eligible() && (++m_coarse_iterations < MAME_SGI_SCSI_COARSE_MAX_ITERATIONS)) {
		m_coarse_continue = true;
		return;
	}
	cycles = m_coarse_cycle_debt;
	m_coarse_cycle_debt = 0;
	m_coarse_iterations = 0;
#endif
	delay_cycles(cycles);
}

#ifdef MAME_SGI_SCSI_COARSE_SCHEDULE
bool nwd33c9x_device::coarse_schedule_eligible() const
{
	if (m_xfr_phase != S_PHASE_DATA_IN && m_xfr_phase != S_PHASE_DATA_OUT)
		return false;
	if ((m_regs[CONTROL] & CONTROL_DM) != CONTROL_DM_BURST)
		return false;
	if (m_transfer_count == 0)
		return false;
	return true;
}
#endif

void nwd33c9x_device::delay_arb_poll()
{
	delay_cycles(MAME_SGI_SCSI_DELAY_SCALE);
}

void nwd33c9x_device::delay_cycles(uint32_t cycles)
{
	m_timer->adjust(clocks_to_attotime(cycles));
}

bool nwd33c9x_device::set_command_length(const uint8_t cc)
{
	const bool eaf = ((m_regs[OWN_ID] & OWN_ID_EAF) != 0);
	bool ret;
	if (eaf && (cc == COMMAND_CC_SELECT_TRANSFER || cc == COMMAND_CC_SELECT_ATN_TRANSFER)) {
		m_command_length &= OWN_ID_CDB_SIZE;
		ret = true;
	} else if (eaf && cc == COMMAND_CC_WAIT_SELECT_RECEIVE_DATA) {
		m_command_length = 6;
		m_regs[COMMAND_PHASE] = COMMAND_PHASE_CP_BYTES_1;
		irq_fifo_push(SCSI_STATUS_NEED_COMMAND_SIZE);
		update_irq();
		ret = false;
	} else {
		switch (m_regs[CDB_1] >> 5) {
		default:
		case 0: m_command_length = 6;  break;
		case 1: m_command_length = 10; break;
		case 5: m_command_length = 12; break;
		}
		ret = true;
	}
	LOGMASKED(LOG_COMMANDS, "SCSI Command Length %d bytes\n", m_command_length);
	return ret;
}

uint8_t nwd33c9x_device::get_msg_out() const
{
	return 0x80 | ((m_regs[SOURCE_ID] & SOURCE_ID_ER) ? 0x40 : 0x00) | (m_regs[TARGET_LUN] & TARGET_LUN_TL);
}

//**************************************************************************
//  Target half - ported/trimmed from machine/nscsi_hle.cpp's generic
//  target state machine (no reselect, no disconnect, no messages beyond
//  receiving a bare IDENTIFY - hd.cpp's target never uses those) and
//  bus/nscsi/hd.cpp's command dispatch.
//**************************************************************************

TIMER_CALLBACK_MEMBER(nwd33c9x_device::t_update_tick)
{
	t_step(param);
}

nwd33c9x_device::t_control *nwd33c9x_device::t_buf_control_push()
{
	if (m_t_buf_control_wpos == int(std::size(m_t_buf_control)))
		throw emu_fatalerror("%s: nwd33c9x target buf_control overflow\n", tag());
	t_control *c = m_t_buf_control + m_t_buf_control_wpos;
	m_t_buf_control_wpos++;
	return c;
}

nwd33c9x_device::t_control *nwd33c9x_device::t_buf_control_pop()
{
	if (m_t_buf_control_rpos == m_t_buf_control_wpos)
		throw emu_fatalerror("%s: nwd33c9x target buf_control underflow\n", tag());
	t_control *c = m_t_buf_control + m_t_buf_control_rpos;
	m_t_buf_control_rpos++;
	if (m_t_buf_control_rpos == m_t_buf_control_wpos)
		m_t_buf_control_rpos = m_t_buf_control_wpos = 0;
	return c;
}

void nwd33c9x_device::t_step(bool timeout)
{
	if (ctrl_r() & S_RST) {
		t_data_w(0);
		t_ctrl_w(0, S_ALL);
		m_t_state = T_IDLE;
		m_t_buf_control_rpos = m_t_buf_control_wpos = 0;
		m_t_identify = 0;
		m_t_data_buffer_size = 0;
		m_t_data_buffer_pos = 0;
		return;
	}

	LOGMASKED(LOG_TARGET, "nwd33c9x target: state=%d.%d %s\n",
		m_t_state & T_STATE_MASK, (m_t_state & T_SUB_MASK) >> T_SUB_SHIFT,
		timeout ? "timeout" : "change");

	switch (m_t_state & T_SUB_MASK ? m_t_state & T_SUB_MASK : m_t_state & T_STATE_MASK) {
	case T_IDLE:
		// Unlike nscsi_hle.cpp's original (which independently checks the
		// data bus for its own configured ID bit, since any of up to 16
		// devices sharing one real bus must self-identify), this doesn't
		// need to re-derive "is this SEL for me" - the initiator already
		// deterministically committed to this exact slot in
		// determine_active_disk() before arbitration even began, and this
		// function only ever runs while m_t_active_disk names it. Only
		// the real electrical condition (SEL alone, BSY not also held by
		// whoever's still arbitrating) gates the transition.
		if ((ctrl_r() & (S_SEL|S_BSY)) == S_SEL) {
			m_t_state = T_TARGET_SELECT_WAIT_BUS_SETTLE;
			// nscsi_full_device::scsi_bus_settle_delay()'s default (400ns);
			// hd.cpp never overrides it.
			m_t_timer->adjust(attotime::from_nsec(400), true);
		}
		break;

	case T_TARGET_SELECT_WAIT_BUS_SETTLE:
		if ((ctrl_r() & (S_SEL|S_BSY)) == S_SEL) {
			m_t_state = T_TARGET_SELECT_WAIT_SEL_0;
			t_ctrl_w(S_BSY, S_BSY);
		} else
			m_t_state = T_IDLE;
		break;

	case T_TARGET_SELECT_WAIT_SEL_0:
		if (ctrl_r() & S_SEL)
			break;
		t_buf_control_push()->m_action = T_BC_MSG_OR_COMMAND;
		m_t_state = T_TARGET_NEXT_CONTROL;
		t_step(false);
		break;

	case T_RECV_BYTE_T_WAIT_ACK_1 << T_SUB_SHIFT:
		if (ctrl_r() & S_ACK) {
			t_put_data(m_t_data_buffer_id, m_t_data_buffer_pos++, data_r());
			m_t_state = (m_t_state & T_STATE_MASK) | (T_RECV_BYTE_T_WAIT_ACK_0 << T_SUB_SHIFT);
			t_ctrl_w(0, S_REQ);
		}
		break;

	case T_RECV_BYTE_T_WAIT_ACK_0 << T_SUB_SHIFT:
		if (!(ctrl_r() & S_ACK)) {
			m_t_state &= T_STATE_MASK;
			t_ctrl_wait(0, S_ACK);
			// hd.cpp never overrides scsi_data_byte_period() (always
			// zero) - always take the immediate path.
			t_step(false);
		}
		break;

	case T_SEND_BYTE_T_WAIT_ACK_1 << T_SUB_SHIFT:
		if (ctrl_r() & S_ACK) {
			m_t_state = (m_t_state & T_STATE_MASK) | (T_SEND_BYTE_T_WAIT_ACK_0 << T_SUB_SHIFT);
			t_data_w(0);
			t_ctrl_w(0, S_REQ);
		}
		break;

	case T_SEND_BYTE_T_WAIT_ACK_0 << T_SUB_SHIFT:
		if (!(ctrl_r() & S_ACK)) {
			m_t_state &= T_STATE_MASK;
			t_ctrl_wait(0, S_ACK);
			t_step(false);
		}
		break;

	case T_TARGET_NEXT_CONTROL: {
		if (m_t_buf_control_rpos == m_t_buf_control_wpos)
			return;
		t_control *ctl = t_buf_control_pop();
		switch (ctl->m_action) {
		case T_BC_MSG_OR_COMMAND:
			m_t_data_buffer_id = SBUF_MAIN;
			m_t_data_buffer_pos = 0;
			if (ctrl_r() & S_ATN) {
				m_t_state = T_TARGET_WAIT_MSG_BYTE;
				t_ctrl_w(S_PHASE_MSG_OUT, S_PHASE_MASK);
			} else {
				m_t_state = T_TARGET_WAIT_CMD_BYTE;
				t_ctrl_w(S_PHASE_COMMAND, S_PHASE_MASK);
			}
			t_target_recv_byte();
			break;

		case T_BC_STATUS:
			t_ctrl_w(S_PHASE_STATUS, S_PHASE_MASK);
			t_target_send_byte(uint8_t(ctl->m_param1));
			break;

		case T_BC_DATA_IN:
			t_ctrl_w(S_PHASE_DATA_IN, S_PHASE_MASK);
			m_t_data_buffer_id = ctl->m_param1;
			m_t_data_buffer_size = ctl->m_param2;
			m_t_data_buffer_pos = 0;
			m_t_state = (m_t_data_buffer_size > 0) ? T_TARGET_WAIT_DATA_IN_BYTE : T_TARGET_NEXT_CONTROL;
			t_step(false);
			break;

		case T_BC_DATA_OUT:
			t_ctrl_w(S_PHASE_DATA_OUT, S_PHASE_MASK);
			m_t_data_buffer_id = ctl->m_param1;
			m_t_data_buffer_size = ctl->m_param2;
			m_t_data_buffer_pos = 0;
			m_t_state = (m_t_data_buffer_size > 0) ? T_TARGET_WAIT_DATA_OUT_BYTE : T_TARGET_NEXT_CONTROL;
			t_step(false);
			break;

		case T_BC_MESSAGE_1:
			t_ctrl_w(S_PHASE_MSG_IN, S_PHASE_MASK);
			t_target_send_byte(uint8_t(ctl->m_param1));
			break;

		case T_BC_BUS_FREE:
			t_data_w(0);
			t_ctrl_wait(S_BSY|S_SEL|S_RST, S_ALL);
			t_ctrl_w(0, S_ALL);
			m_t_state = T_IDLE;
			break;
		}
		break;
	}

	case T_TARGET_WAIT_DATA_IN_BYTE:
		if (m_t_data_buffer_pos == m_t_data_buffer_size - 1)
			m_t_state = T_TARGET_NEXT_CONTROL;
		t_target_send_buffer_byte();
		break;

	case T_TARGET_WAIT_DATA_OUT_BYTE:
		if (m_t_data_buffer_pos == m_t_data_buffer_size - 1)
			m_t_state = T_TARGET_NEXT_CONTROL;
		t_target_recv_byte();
		break;

	case T_TARGET_WAIT_MSG_BYTE:
		if (ctrl_r() & S_SEL)
			return;
		if (!(ctrl_r() & S_ATN)) {
			m_t_cmdsize = uint8_t(m_t_data_buffer_pos);
			t_scsi_message();
			m_t_data_buffer_id = SBUF_MAIN;
			m_t_data_buffer_pos = 0;
			m_t_state = T_TARGET_WAIT_CMD_BYTE;
			t_ctrl_w(S_PHASE_COMMAND, S_PHASE_MASK);
		}
		t_target_recv_byte();
		break;

	case T_TARGET_WAIT_CMD_BYTE:
		if (ctrl_r() & S_SEL)
			return;
		if (ctrl_r() & S_ATN) {
			LOGMASKED(LOG_TARGET, "nwd33c9x target: parity error?\n");
			m_t_state = T_IDLE;
			break;
		}

		if (t_command_done(m_t_cmdbuf[0], uint8_t(m_t_data_buffer_pos))) {
			m_t_cmdsize = uint8_t(m_t_data_buffer_pos);
			t_ctrl_wait(0, S_ACK);
			t_scsi_command();
			m_t_state = T_TARGET_NEXT_CONTROL;
			// hd.cpp's default scsi_data_command_delay() (always zero -
			// the optional seek-timing model isn't ported here; see
			// nwd33c9x.h) - always take the immediate path.
			t_step(false);
		} else
			t_target_recv_byte();
		break;

	default:
		fatalerror("%s: nwd33c9x target: unexpected state %d.%d\n",
			shortname(), m_t_state & T_STATE_MASK, (m_t_state & T_SUB_MASK) >> T_SUB_SHIFT);
		break;
	}
}

void nwd33c9x_device::t_target_recv_byte()
{
#ifdef MAME_SGI_SCSI_BURST_FASTPATH
	// Fast path (DATA_OUT direction) - see nwd33c9x.h's own
	// comment on MAME_SGI_SCSI_BURST_FASTPATH for the full rationale and
	// the live-traced synchronous call chain this relies on. Only takes
	// over once this function has already been reached normally at least
	// once for the current chunk (m_t_data_buffer_pos > 0) - the FIRST
	// byte of every transfer always goes through the real REQ/ACK dance
	// below, same as the last byte (loop condition excludes it).
	if (coarse_schedule_eligible() && m_t_active_disk >= 0 && m_t_data_buffer_id == SBUF_BLOCK && m_t_data_buffer_pos > 0) {
#if MAME_SGI_SCSI_BULK_DMA
		// Runs of bytes straight from the host's DMA engine: the mirror of
		// t_target_send_buffer_byte()'s bulk loop. Each run goes into
		// m_t_stage after the bytes of its first sector already received
		// (in the sector buffer; a transfer's first byte always comes
		// through the handshake), and every sector it completes is written
		// in one disk call (a CHD otherwise rewrites a whole hunk for every
		// sector). The block, cur_lba and the disk end as that many
		// t_put_data() calls would leave them.
		if (!m_bulk_dma_out.isnull()) {
			while (m_t_data_buffer_pos < m_t_data_buffer_size - 1 && m_transfer_count > 1
					&& data_fifo_empty() && !m_drq_state) {
				t_disk_slot &d = m_t_disk[m_t_active_disk];
				const int pos = m_t_data_buffer_pos;
				const uint32_t bps = d.bytes_per_sector;
				const uint32_t pre = pos & (bps - 1);
				const uint32_t run = std::min({ uint32_t(m_t_data_buffer_size - 1 - pos), m_transfer_count - 1, T_STAGE_BYTES - pre });
				if (pre)
					memcpy(m_t_stage.get(), d.block, pre);
				const uint32_t done = m_bulk_dma_out(&m_t_stage[pre], run);
				if (!done)
					break;
				const uint32_t full = (pre + done) >> d.sector_shift;
				const uint32_t rest = (pre + done) & (bps - 1);
				const int first = d.lba + (pos >> d.sector_shift);
				if (full) {
					if (!m_disk_image[m_t_active_disk]->write(first, m_t_stage.get(), full))
						LOGMASKED(LOG_TARGET, "nwd33c9x target: HD WRITE ERROR!\n");
					// the last whole sector stays in the sector buffer
					memcpy(d.block, &m_t_stage[(full - 1) << d.sector_shift], bps);
					d.cur_lba = first + full - 1;
				}
				if (rest) {
					memcpy(d.block, &m_t_stage[full << d.sector_shift], rest);
					d.cur_lba = first + full;
				}
				m_t_data_buffer_pos += done;
				m_transfer_count -= done;
				i_ctrl_wait(S_REQ, S_REQ);
				set_i_state(INIT_XFR_WAIT_REQ);
				if (done < run)
					break;
			}
		}
#endif
		while (m_t_data_buffer_pos < m_t_data_buffer_size - 1 && m_transfer_count > 1) {
			// See t_target_send_buffer_byte()'s identical fix (2026-10-01)
			// for the full rationale - same two bugs, mirrored for this
			// direction: (1) keep the initiator's own per-byte bookkeeping
			// (its INIT_XFR_WAIT_REQ parked state and S_REQ re-arm) live
			// instead of frozen, and (2) explicitly mirror
			// T_TARGET_WAIT_DATA_OUT_BYTE's own m_t_state transition,
			// which this loop's self-recursion bypasses the same way the
			// DATA_IN direction's did. NOT independently boot-verified in
			// this direction yet (the confirmed failure was a DATA_IN
			// disk read) - same mechanism, applied for consistency rather
			// than left asymmetric, but treat with the same caution until
			// a real disk-write path exercises it.
			if (data_fifo_full())
				break;
			// set_drq() synchronously drives dma_w() (confirmed live via
			// host gdb backtrace - hpc3_device::scsi0_drq() calls back
			// into dma_w() before this returns), filling the fifo with
			// the host's next byte and decrementing m_transfer_count.
			set_drq();
			if (data_fifo_empty())
				break; // host wasn't ready - fall through to the real dance for this byte rather than press on incorrectly
			t_put_data(m_t_data_buffer_id, m_t_data_buffer_pos++, data_fifo_pop());
			i_ctrl_wait(S_REQ, S_REQ);
			set_i_state(INIT_XFR_WAIT_REQ);
		}
	}
	// Mirrors T_TARGET_WAIT_DATA_OUT_BYTE's own check - see
	// t_target_send_buffer_byte()'s identical fix for the full rationale.
	//
	// Root-caused 2026-10-01 (side-by-side arbitration/command-phase
	// trace, after the first version of this guard caused a NEW bug):
	// unlike t_target_send_buffer_byte() (only ever reached via T_BC_
	// DATA_IN, where m_t_data_buffer_size is always genuinely set for
	// whichever buffer that action named), this function is ALSO called
	// for CDB and message-byte reception (T_TARGET_WAIT_CMD_BYTE/
	// T_TARGET_WAIT_MSG_BYTE) - contexts that decide their own completion
	// via t_command_done()/S_ATN, never via m_t_data_buffer_size, which
	// sits stale there (left over from whatever SBUF_BLOCK transfer size
	// came before). The unconditional version of this guard fired
	// whenever a CDB byte position happened to coincide with that stale
	// size-1 (observed live: a prior 8-byte transfer left size=8, and
	// the very next command's CDB byte 7 triggered this mid-command),
	// forcing T_TARGET_NEXT_CONTROL partway through CDB reception and
	// corrupting it - visible as an infinite REQ/ACK oscillation between
	// the initiator and target, never reaching the data phase at all.
	// Scoping to SBUF_BLOCK (matching the fast path's own eligibility
	// above, which is the only reason this guard needs to exist at all)
	// fixes it.
	if (m_t_data_buffer_id == SBUF_BLOCK && m_t_data_buffer_pos == m_t_data_buffer_size - 1)
		m_t_state = T_TARGET_NEXT_CONTROL;
#endif
	t_ctrl_wait(S_ACK, S_ACK);
	m_t_state = (m_t_state & T_STATE_MASK) | (T_RECV_BYTE_T_WAIT_ACK_1 << T_SUB_SHIFT);
	t_ctrl_w(S_REQ, S_REQ);
	t_step(false);
}

void nwd33c9x_device::t_target_send_byte(uint8_t val)
{
	t_ctrl_wait(S_ACK, S_ACK);
	m_t_state = (m_t_state & T_STATE_MASK) | (T_SEND_BYTE_T_WAIT_ACK_1 << T_SUB_SHIFT);
	t_data_w(val);
	t_ctrl_w(S_REQ, S_REQ);
	t_step(false);
}

void nwd33c9x_device::t_target_send_buffer_byte()
{
#ifdef MAME_SGI_SCSI_BURST_FASTPATH
	// Fast path (DATA_IN direction) - see nwd33c9x.h's own
	// comment on MAME_SGI_SCSI_BURST_FASTPATH for the full rationale and
	// the live-traced synchronous call chain this relies on. Only takes
	// over once this function has already been reached normally at least
	// once for the current chunk (m_t_data_buffer_pos > 0) - the FIRST
	// byte of every transfer always goes through the real REQ/ACK dance
	// below, same as the last byte (loop condition excludes it).
	if (coarse_schedule_eligible() && m_t_active_disk >= 0 && m_t_data_buffer_id == SBUF_BLOCK && m_t_data_buffer_pos > 0) {
#if MAME_SGI_SCSI_BULK_DMA
		// Runs of bytes straight to the host's DMA engine. Between the
		// per-byte iterations below the FIFO is empty and DRQ is low; a run
		// of n bytes ends in that same state with the position and the
		// transfer count advanced by n, which is what n iterations do. The
		// transfer count stays above 1 (as in the loop below), so its zero
		// transition, and the last byte, are left to the real handshake.
		if (!m_bulk_dma_in.isnull()) {
			while (m_t_data_buffer_pos < m_t_data_buffer_size - 1 && m_transfer_count > 1
					&& data_fifo_empty() && !m_drq_state) {
				const t_disk_slot &d = m_t_disk[m_t_active_disk];
				const uint32_t in_sector = d.bytes_per_sector - (m_t_data_buffer_pos & (d.bytes_per_sector - 1));
				const uint32_t run = std::min({ in_sector, uint32_t(m_t_data_buffer_size - 1 - m_t_data_buffer_pos), m_transfer_count - 1 });
				const uint32_t done = m_bulk_dma_in(t_block_at(m_t_data_buffer_pos), run);
				m_t_data_buffer_pos += done;
				m_transfer_count -= done;
				if (done) {
					i_ctrl_wait(S_REQ, S_REQ);
					set_i_state(INIT_XFR_WAIT_REQ);
				}
				if (done < run)
					break;
			}
		}
#endif
		while (m_t_data_buffer_pos < m_t_data_buffer_size - 1 && m_transfer_count > 1) {
			// Root-caused 2026-10-01 (single-command isolation): the
			// original version
			// of this loop skipped the initiator-side per-byte bookkeeping
			// entirely, not just the timer/scheduler overhead the already-
			// shipped MAME_SGI_SCSI_COARSE_SCHEDULE feature already strips
			// out - m_i_state froze at INIT_XFR_WAIT_REQ for the whole
			// fast-tracked stretch. This version re-derives, directly, the
			// exact net effect of the real per-byte walk's INIT_XFR guard
			// (data_fifo_full()/m_drq_state check) and INIT_XFR_WAIT_REQ's
			// own re-arm (i_ctrl_wait(S_REQ,S_REQ)) and parked destination
			// state - the one piece of real per-byte work that can't be
			// skipped - instead of walking RECV_WAIT_REQ_1/SETTLE/REQ_0 and
			// INIT_XFR_RECV_BYTE_ACK one hop at a time to reach it.
			if (data_fifo_full() || m_drq_state)
				break; // mirrors INIT_XFR's own real guard - fall through to the real dance for this byte rather than press on incorrectly
			data_fifo_push(t_get_data(m_t_data_buffer_id, m_t_data_buffer_pos++));
			// set_drq() synchronously drives dma_r() (confirmed live via
			// host gdb backtrace - hpc3_device::scsi0_drq() calls back
			// into dma_r() before this returns), draining the fifo and
			// decrementing m_transfer_count before the next iteration.
			set_drq();
			i_ctrl_wait(S_REQ, S_REQ);
			set_i_state(INIT_XFR_WAIT_REQ);
		}
	}
	// Root-caused 2026-10-01 (single-command isolation):
	// T_TARGET_WAIT_DATA_IN_BYTE's
	// own dispatch in t_step() is normally the ONLY place that sets
	// m_t_state to T_TARGET_NEXT_CONTROL once m_t_data_buffer_pos reaches
	// the last index, and it does so BEFORE calling this function for the
	// final byte. The fast loop above calls this function's own tail
	// recursively from inside itself, bypassing that t_step() dispatch
	// entirely for every fast-tracked byte - so without this, m_t_state
	// is still T_TARGET_WAIT_DATA_IN_BYTE when the final byte's own real
	// dance completes, t_step() re-enters T_TARGET_WAIT_DATA_IN_BYTE one
	// more time, and sends a bogus extra byte one past the end of the
	// buffer, still claiming S_PHASE_DATA_IN - the second, erroneous REQ
	// pulse that was corrupting the exchange (confirmed directly: this
	// pulse is what the initiator's INIT_XFR_WAIT_REQ decision sees with
	// m_transfer_count already at 0, which is specifically what signals
	// SCSI_STATUS_UNEXPECTED_PHASE). Mirrors T_TARGET_WAIT_DATA_IN_BYTE's
	// own check exactly, run here instead since this function is what's
	// now deciding when the current chunk is really finished.
	if (m_t_data_buffer_pos == m_t_data_buffer_size - 1)
		m_t_state = T_TARGET_NEXT_CONTROL;
#endif
	t_target_send_byte(t_get_data(m_t_data_buffer_id, m_t_data_buffer_pos++));
}

bool nwd33c9x_device::t_command_done(uint8_t command, uint8_t length) const
{
	if (!length)
		return false;
	// command is uint8_t so command>>5 is always 0-7; nscsi_hle.cpp's
	// original switch also has dead cases 8/9 (length==16) that this
	// drops as genuinely unreachable, not a behavior change.
	switch (command >> 5) {
	case 0: return length == 6;
	case 1: return length == 10;
	case 2: return length == 10;
	case 3: return true;
	case 4: return true;
	case 5: return length == 12;
	case 6: return true;
	case 7: return length == 32;
	}
	return true;
}

// the byte at a position in the current SBUF_BLOCK transfer, reading in
// its sector when it isn't the one in the buffer
uint8_t *nwd33c9x_device::t_block_at(int pos)
{
	t_disk_slot &d = m_t_disk[m_t_active_disk];
	int clba = d.lba + (pos >> d.sector_shift);
	if (clba != d.cur_lba) {
		d.cur_lba = clba;
		if (!m_disk_image[m_t_active_disk]->read(d.cur_lba, d.block)) {
			LOGMASKED(LOG_TARGET, "nwd33c9x target: HD READ ERROR!\n");
			memset(d.block, 0, sizeof(d.block));
		}
	}
	return &d.block[pos & (d.bytes_per_sector - 1)];
}

uint8_t nwd33c9x_device::t_get_data(int id, int pos)
{
	if (id == SBUF_BLOCK)
		return *t_block_at(pos);
	if (id == SBUF_SENSE)
		return m_t_disk[m_t_active_disk].sense_buffer[pos];
	return m_t_cmdbuf[pos];
}

void nwd33c9x_device::t_put_data(int id, int pos, uint8_t data)
{
	if (id == SBUF_BLOCK) {
		t_disk_slot &d = m_t_disk[m_t_active_disk];
		const int offset = pos & (d.bytes_per_sector - 1);
		d.block[offset] = data;
		d.cur_lba = d.lba + (pos >> d.sector_shift);
		if (offset == d.bytes_per_sector - 1) {
			if (!m_disk_image[m_t_active_disk]->write(d.cur_lba, d.block))
				LOGMASKED(LOG_TARGET, "nwd33c9x target: HD WRITE ERROR!\n");
		}
		return;
	}
	if (id == SBUF_SENSE) {
		m_t_disk[m_t_active_disk].sense_buffer[pos] = data;
		return;
	}
	m_t_cmdbuf[pos] = data;
}

void nwd33c9x_device::t_status_complete(uint8_t st)
{
	t_control *c;
	c = t_buf_control_push();
	c->m_action = T_BC_STATUS;
	c->m_param1 = st;
	c = t_buf_control_push();
	c->m_action = T_BC_MESSAGE_1;
	c->m_param1 = SM_COMMAND_COMPLETE;
	c = t_buf_control_push();
	c->m_action = T_BC_BUS_FREE;
	t_resume_deferred();
}

void nwd33c9x_device::t_resume_deferred()
{
	if (m_t_state == T_TARGET_NEXT_CONTROL && m_t_buf_control_rpos != m_t_buf_control_wpos)
		t_step(false);
}

void nwd33c9x_device::t_data_in(int buf, int size)
{
	t_control *c = t_buf_control_push();
	c->m_action = T_BC_DATA_IN;
	c->m_param1 = buf;
	c->m_param2 = size;
	t_resume_deferred();
}

void nwd33c9x_device::t_data_out(int buf, int size)
{
	t_control *c = t_buf_control_push();
	c->m_action = T_BC_DATA_OUT;
	c->m_param1 = buf;
	c->m_param2 = size;
	t_resume_deferred();
}

void nwd33c9x_device::t_sense(bool deferred, uint8_t key, uint8_t asc, uint8_t ascq)
{
	uint8_t *buf = m_t_disk[m_t_active_disk].sense_buffer;
	memset(buf, 0, 18);
	buf[0] = 0x80 | (deferred ? 0x71 : 0x70);
	buf[2] = key;
	buf[7] = 10; // additional sense length
	buf[12] = asc;
	buf[13] = ascq;
}

void nwd33c9x_device::t_bad_lun()
{
	t_status_complete(SS_CHECK_CONDITION);
	t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_LOGICAL_UNIT_NOT_SUPPORTED);
}

int nwd33c9x_device::t_get_lun(int def) const
{
	if (m_t_identify & 0x80)
		return m_t_identify & 0x07;
	return def;
}

void nwd33c9x_device::t_scsi_message()
{
	if (m_t_cmdbuf[0] & 0x80) {
		m_t_identify = m_t_cmdbuf[0];
		return;
	}
	LOGMASKED(LOG_TARGET, "nwd33c9x target: unsupported message %02x\n", m_t_cmdbuf[0]);
}

// Ported from bus/nscsi/hd.cpp's scsi_command(), trimmed of the optional
// seek-timing model (set_seek_timing()/seek_time() - not exercised by
// this project's default machine config). Structure and byte-for-byte
// output kept identical to the original, including its MODE_SENSE_6 page-
// length quirks (transcribed line-for-line, not recomputed, to avoid
// introducing a fresh counting bug in a section this fiddly).
void nwd33c9x_device::t_scsi_command()
{
	t_disk_slot &d = m_t_disk[m_t_active_disk];
	harddisk_image_device *image = m_disk_image[m_t_active_disk];

	switch (m_t_cmdbuf[0]) {
	case SC_TEST_UNIT_READY:
		if (!t_get_lun(m_t_cmdbuf[1] >> 5))
			t_status_complete(SS_GOOD);
		else
			t_bad_lun();
		break;

	case SC_READ_6:
		d.lba = get_u24be(&m_t_cmdbuf[1]) & 0x1fffff;
		d.blocks = m_t_cmdbuf[4];
		if (!d.blocks)
			d.blocks = 256;

		if (image->read(d.lba, d.block)) {
			t_data_in(SBUF_BLOCK, d.blocks * d.bytes_per_sector);
			t_status_complete(SS_GOOD);
		} else {
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_INVALID_FIELD_IN_CDB);
		}
		break;

	case SC_WRITE_6:
		d.lba = get_u24be(&m_t_cmdbuf[1]) & 0x1fffff;
		d.blocks = m_t_cmdbuf[4];
		if (!d.blocks)
			d.blocks = 256;

		if (image->write(d.lba, d.block)) {
			t_data_out(SBUF_BLOCK, d.blocks * d.bytes_per_sector);
			t_status_complete(SS_GOOD);
		} else {
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_INVALID_FIELD_IN_CDB);
		}
		break;

	case SC_INQUIRY: {
		const int lun = t_get_lun(m_t_cmdbuf[1] >> 5);
		const int page = m_t_cmdbuf[2];
		int size = m_t_cmdbuf[4];
		switch (page) {
		case 0:
			std::fill_n(m_t_cmdbuf, 148, 0);
			std::fill_n(&m_t_cmdbuf[8], 28, 0x20);

			if (lun != 0)
				m_t_cmdbuf[0] = 0x7f;
			else
				m_t_cmdbuf[0] = 0x00; // direct-access device
			m_t_cmdbuf[1] = 0x00; // media not removable
			m_t_cmdbuf[2] = 0x05; // SPC-3
			m_t_cmdbuf[3] = 0x01; // response data format = CCS
			m_t_cmdbuf[4] = 52;   // additional length
			if (d.inquiry_data.empty()) {
				memset(m_t_cmdbuf + 8, ' ', 28);
				static constexpr char default_model[] =
					" SEAGATE"
					"          ST225N"
					"1.00";
				int len = int(sizeof(default_model)) - 1;
				if (len > 28)
					len = 28;
				memcpy(m_t_cmdbuf + 8, default_model, len);
				m_t_cmdbuf[36] = 0x00;
				m_t_cmdbuf[37] = 0x08;
				m_t_cmdbuf[38] = 0x00;
				m_t_cmdbuf[39] = 0x99;
				m_t_cmdbuf[40] = 0xa0;
				m_t_cmdbuf[41] = 0x27;
				m_t_cmdbuf[42] = 0x34;
				m_t_cmdbuf[43] = 0x01;
				m_t_cmdbuf[44] = 0x04;
				m_t_cmdbuf[45] = 0xa0;
				m_t_cmdbuf[46] = 0x01;
				m_t_cmdbuf[47] = 0x18;
				m_t_cmdbuf[48] = 0x07;
				m_t_cmdbuf[49] = 0x00;
				m_t_cmdbuf[50] = 0xa0;
				m_t_cmdbuf[51] = 0x00;
				m_t_cmdbuf[52] = 0x00;
				m_t_cmdbuf[53] = 0xff;
			}
			else
				std::copy_n(d.inquiry_data.begin(), std::min(d.inquiry_data.size(), size_t(48)), &m_t_cmdbuf[8]);

			if (size > 56)
				size = 56;
			t_data_in(SBUF_MAIN, size);
			break;
		}
		t_status_complete(SS_GOOD);
		break;
	}

	case SC_MODE_SENSE_6: {
		const int lun = t_get_lun(m_t_cmdbuf[1] >> 5);
		if (lun) {
			t_bad_lun();
			break;
		}

		const int page = m_t_cmdbuf[2] & 0x3f;
		int size = m_t_cmdbuf[4];
		int pos = 1;
		m_t_cmdbuf[pos++] = 0x00; // medium type
		m_t_cmdbuf[pos++] = 0x00; // WP, cache

		const auto &info = image->get_info();
		uint32_t dsize = info.cylinders * info.heads * info.sectors - 1;
		m_t_cmdbuf[pos++] = 0x08; // block descriptor length
		m_t_cmdbuf[pos++] = 0x00;
		put_u24be(&m_t_cmdbuf[pos], dsize);
		pos += 3;
		m_t_cmdbuf[pos++] = 0x00;
		put_u24be(&m_t_cmdbuf[pos], info.sectorbytes);
		pos += 3;

		int pmax = page == 0x3f ? 0x3e : page;
		int pmin = page == 0x3f ? 0x00 : page;

		bool fail = false;
		for (int p = pmax; p >= pmin; p--) {
			switch (p) {
			case 0x00: // unit attention parameters page (weird)
				m_t_cmdbuf[pos++] = 0x80;
				m_t_cmdbuf[pos++] = 0x02;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				break;

			case 0x01: // read-write error recovery page
				m_t_cmdbuf[pos++] = 0x01;
				m_t_cmdbuf[pos++] = 0x0a;
				m_t_cmdbuf[pos++] = 0x26;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				break;

			case 0x02: // disconnect-reconnect page
				m_t_cmdbuf[pos++] = 0x02;
				m_t_cmdbuf[pos++] = 0x0e;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				break;

			case 0x03: { // format parameters page
				m_t_cmdbuf[pos++] = 0x83;
				m_t_cmdbuf[pos++] = 0x16;
				put_u16be(&m_t_cmdbuf[pos], info.cylinders * info.heads);
				pos += 2;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				put_u16be(&m_t_cmdbuf[pos], info.sectors);
				pos += 2;
				put_u16be(&m_t_cmdbuf[pos], info.sectorbytes);
				pos += 2;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				break;
			}

			case 0x04: { // rigid drive geometry page
				m_t_cmdbuf[pos++] = 0x84;
				m_t_cmdbuf[pos++] = 0x16;
				put_u24be(&m_t_cmdbuf[pos], info.cylinders);
				pos += 3;
				m_t_cmdbuf[pos++] = uint8_t(info.heads);
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				put_u16be(&m_t_cmdbuf[pos], 10000);
				pos += 2;
				m_t_cmdbuf[pos++] = 0x00;
				m_t_cmdbuf[pos++] = 0x00;
				break;
			}

			case 0x08: // caching page
				m_t_cmdbuf[pos++] = 0x08;
				m_t_cmdbuf[pos++] = 0x0a;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				m_t_cmdbuf[pos++] = 0;
				break;

			case 0x30: { // Apple firmware ID page
				m_t_cmdbuf[pos++] = 0xb0;
				m_t_cmdbuf[pos++] = 0x16;
				static constexpr uint8_t id[22] = {
					'A','P','P','L','E',' ','C','O','M','P','U','T','E','R',',',' ','I','N','C',' ',' ',' '
				};
				for (int i = 0; i < 22; i++)
					m_t_cmdbuf[pos++] = id[i];
				break;
			}

			default:
				if (page != 0x3f) {
					LOGMASKED(LOG_TARGET, "nwd33c9x target: mode sense page %02x unhandled\n", page);
					fail = true;
				}
				break;
			}
		}

		if (!fail) {
			m_t_cmdbuf[0] = uint8_t(pos - 1);
			if (pos > size)
				pos = size;

			t_data_in(SBUF_MAIN, pos);
			t_status_complete(SS_GOOD);
		} else {
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_INVALID_FIELD_IN_CDB);
		}
		break;
	}

	case SC_START_STOP_UNIT:
		t_status_complete(SS_GOOD);
		break;

	case SC_RECEIVE_DIAGNOSTIC_RESULTS: {
		int size = get_u16be(&m_t_cmdbuf[3]);
		int pos = 0;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 6;
		m_t_cmdbuf[pos++] = 0; // ROM OK
		m_t_cmdbuf[pos++] = 0; // RAM OK
		m_t_cmdbuf[pos++] = 0; // data buffer OK
		m_t_cmdbuf[pos++] = 0; // interface OK
		m_t_cmdbuf[pos++] = 0;
		if (size > pos)
			size = pos;
		t_data_in(SBUF_MAIN, size);
		t_status_complete(SS_GOOD);
		break;
	}

	case SC_SEND_DIAGNOSTIC: {
		int size = get_u16be(&m_t_cmdbuf[3]);
		if (m_t_cmdbuf[1] & 4) {
			t_status_complete(SS_GOOD);
			break;
		}
		int pos = 0;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 6;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 0;
		m_t_cmdbuf[pos++] = 0;
		if (size > pos)
			size = pos;
		t_data_in(SBUF_MAIN, size);
		t_status_complete(SS_GOOD);
		break;
	}

	case SC_SYNCHRONIZE_CACHE:
		if (!image->flush())
			LOGMASKED(LOG_TARGET, "nwd33c9x target: HD FLUSH ERROR!\n");
		t_status_complete(SS_GOOD);
		break;

	case SC_READ_CAPACITY: {
		const auto &info = image->get_info();
		uint32_t size = info.cylinders * info.heads * info.sectors - 1;

		put_u32be(&m_t_cmdbuf[0], size);
		put_u32be(&m_t_cmdbuf[4], info.sectorbytes);

		t_data_in(SBUF_MAIN, 8);
		t_status_complete(SS_GOOD);
		break;
	}

	case SC_READ_10:
		d.lba = get_u32be(&m_t_cmdbuf[2]);
		d.blocks = get_u16be(&m_t_cmdbuf[7]);

		if (image->read(d.lba, d.block)) {
			t_data_in(SBUF_BLOCK, d.blocks * d.bytes_per_sector);
			t_status_complete(SS_GOOD);
		}
		else {
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_INVALID_FIELD_IN_CDB);
		}
		break;

	case SC_WRITE_10:
		d.lba = get_u32be(&m_t_cmdbuf[2]);
		d.blocks = get_u16be(&m_t_cmdbuf[7]);

		if (image->write(d.lba, d.block)) {
			t_data_out(SBUF_BLOCK, d.blocks * d.bytes_per_sector);
			t_status_complete(SS_GOOD);
		}
		else {
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_INVALID_FIELD_IN_CDB);
		}
		break;

	case SC_FORMAT_UNIT: {
		const auto &info = image->get_info();
		auto block = std::make_unique<uint8_t[]>(info.sectorbytes);
		for (int cyl = 0; cyl < info.cylinders; cyl++) {
			for (int head = 0; head < info.heads; head++) {
				for (int sector = 0; sector < info.sectors; sector++) {
					image->write(cyl * head * sector, block.get());
				}
			}
		}
		t_status_complete(SS_GOOD);
		break;
	}

	case SC_MODE_SELECT_6:
		if (m_t_cmdbuf[4])
			t_data_out(SBUF_MAIN, m_t_cmdbuf[4]);
		t_status_complete(SS_GOOD);
		break;

	case SC_VERIFY_10:
		if (!(m_t_cmdbuf[1] & 0x02))
			t_status_complete(SS_GOOD);
		else {
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST);
		}
		break;

	default:
		switch (m_t_cmdbuf[0]) {
		case SC_REZERO_UNIT:
			t_status_complete(SS_GOOD);
			break;

		case SC_REQUEST_SENSE: {
			const uint8_t alloc_len = m_t_cmdbuf[4];
			if ((m_t_cmdbuf[1] & 0x1f) || m_t_cmdbuf[2] || m_t_cmdbuf[3]) {
				t_status_complete(SS_CHECK_CONDITION);
				t_sense(false, SK_ILLEGAL_REQUEST, SK_ASC_INVALID_FIELD_IN_CDB);
				break;
			}
			t_data_in(SBUF_SENSE, std::min(18, int(alloc_len)));
			t_status_complete(SS_GOOD);
			break;
		}

		default:
			LOGMASKED(LOG_TARGET, "nwd33c9x target: unsupported command %02x\n", m_t_cmdbuf[0]);
			t_status_complete(SS_CHECK_CONDITION);
			t_sense(false, SK_ILLEGAL_REQUEST);
			break;
		}
		break;
	}
}
