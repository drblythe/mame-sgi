// license:BSD-3-Clause
// copyright-holders:Mietek Bak
#ifndef MAME_UTIL_SIMH_TAPE_FILE_H
#define MAME_UTIL_SIMH_TAPE_FILE_H

#pragma once

#include "tape_file_interface.h"
#include "utilfwd.h"

#include "osdcomm.h"

#include <string>

// Growable tape images: the tape's capacity is nominal, max(file size,
// SIMH_TAPE_NOMINAL_CAPACITY); the file holds only what has been written,
// bytes past its end read as blank tape (0xff, the EOM marker), and every
// write or erase cuts the file off at the new end of data, as writing a real
// tape makes everything after it unreadable.
#define MAME_SGI_SIMH_TAPE_GROWABLE

//////////////////////////////////////////////////////////////////////////////

class simh_tape_file : public tape_file_interface
{
public:
	// construction and destruction
#ifdef MAME_SGI_SIMH_TAPE_GROWABLE
	static constexpr osd::u64 SIMH_TAPE_NOMINAL_CAPACITY = 2'000'000'000; // DDS-1, 90 m tape
	simh_tape_file(util::core_file &file, osd::u64 file_size, bool read_only, bool create = false);
#else
	simh_tape_file(util::random_read_write &file, osd::u64 file_size, bool read_only, bool create = false);
#endif
	virtual ~simh_tape_file();

	// position-preserving operations
	virtual bool is_read_only() const override { return m_read_only; }
	virtual bool is_ew() const override { return m_pos + 32768 >= m_file_size; } // 32KB from EOM; TODO: ANSI says EW should be 10ft from EOM regardless of density
	virtual osd::u8 get_density_code() const override { return 0; } // TODO: SIMH doesn't define density
	virtual std::pair<tape_status, osd::u32> read_position() const override;

	// non-destructive operations
	virtual void rewind(bool eom) override;
	virtual tape_status locate_block(osd::u32 req_block_addr) override;
	virtual tape_status space_eod() override;
	virtual std::pair<tape_status, osd::u32> space_blocks(osd::u32 req_blocks_num) override;
	virtual std::pair<tape_status, osd::u32> space_blocks_reverse(osd::u32 req_blocks_num) override;
	virtual std::pair<tape_status, osd::u32> space_filemarks(osd::u32 req_marks_num, bool setmarks = false, bool sequential = false) override;
	virtual std::pair<tape_status, osd::u32> space_filemarks_reverse(osd::u32 req_marks_num, bool setmarks = false, bool sequential = false) override;
	virtual std::pair<tape_status, osd::u32> read_block(osd::u8 *buf, osd::u32 buf_size) override;

	// destructive operations
	virtual void erase(bool eom) override;
	virtual tape_status write_block(const osd::u8 *buf, osd::u32 len) override;
	virtual tape_status write_filemarks(osd::u32 req_marks_num, bool setmarks = false) override;

protected:
	// internal operations
	void raw_seek(const osd::u64 pos) const;
	void raw_read(osd::u8 *const buf, const osd::u32 len) const;
	void raw_write(const osd::u8 *const buf, const osd::u32 len) const;
	void read_bytes(const osd::u64 pos, osd::u8 *const buf, const osd::u32 len) const;
	osd::u32 read_word(const osd::u64 pos) const;
	void write_bytes(const osd::u64 pos, const osd::u8 *const buf, const osd::u32 len) const;
	void write_byte_repeat(const osd::u64 pos, const osd::u8 data, const osd::u32 len) const;
	void write_word(const osd::u64 pos, const osd::u32 data) const;
#ifdef MAME_SGI_SIMH_TAPE_GROWABLE
	void end_data_at(const osd::u64 pos);
#endif

	// state
#ifdef MAME_SGI_SIMH_TAPE_GROWABLE
	util::core_file &m_file; // tape image file
	osd::u64 m_file_size; // nominal capacity of the tape
	osd::u64 m_data_size; // size of tape image file; the tape is blank from here on
#else
	util::random_read_write &m_file; // tape image file
	osd::u64 m_file_size; // size of tape image file
#endif
	bool m_read_only; // should we disallow destructive operations on tape image
	osd::u64 m_pos; // tape position
};

//////////////////////////////////////////////////////////////////////////////

#endif // MAME_UTIL_SIMH_TAPE_FILE_H
