// license:BSD-3-Clause
// copyright-holders:Aaron Giles
/***************************************************************************

    drcuml.h

    Universal machine language for dynamic recompiling CPU cores.

***************************************************************************/
#ifndef MAME_CPU_DRCUML_H
#define MAME_CPU_DRCUML_H

#pragma once

#include "drccache.h"
#include "uml.h"

#include <iosfwd>
#include <list>
#include <memory>
#include <string>
#include <array>
#include <utility>
#include <vector>


//**************************************************************************
//  CONSTANTS
//**************************************************************************

// these options are passed into drcuml_alloc() and control global behaviors



//**************************************************************************
//  TYPE DEFINITIONS
//**************************************************************************

// opaque structure describing UML generation state
class drcuml_state;


// an integer register, with low/high parts
union drcuml_ireg
{
#ifdef LSB_FIRST
	struct { u32 l, h; }    w;                      // 32-bit low, high parts of the register
#else
	struct { u32 h, l; }    w;                      // 32-bit low, high parts of the register
#endif
	u64                     d;                      // 64-bit full register
};


// a floating-point register, with low/high parts
union drcuml_freg
{
#ifdef LSB_FIRST
	struct { float l, h; }  s;                      // 32-bit low, high parts of the register
#else
	struct { float h, l; }  s;                      // 32-bit low, high parts of the register
#endif
	double                  d;                      // 64-bit full register
};


// the collected machine state of a system
struct drcuml_machine_state
{
	drcuml_ireg             r[uml::REG_I_COUNT];    // integer registers
	drcuml_freg             f[uml::REG_F_COUNT];    // floating-point registers
	u32                     exp;                    // exception parameter register
	u8                      fmod;                   // fmod (floating-point mode) register
	u8                      flags;                  // flags state
};


// hints and information about the back-end
struct drcbe_info
{
	u8                      direct_iregs;           // number of direct-mapped integer registers
	u8                      direct_fregs;           // number of direct-mapped floating point registers
};


// a drcuml_block describes a basic block of instructions
class drcuml_block
{
public:
	// getters
	bool inuse() const { return m_inuse; }
	bool invariant() const { return m_invariant; }
	u32 maxinst() const { return m_maxinst; }

	// code generation
	void end();
	[[noreturn]] void abort();

	// instruction appending
	uml::instruction &append()
	{
		if (UNEXPECTED(m_nextinst >= m_maxinst))
			overrun();
		return m_inst[m_nextinst++]; // get a pointer to the next instruction
	}

	template <typename Format, typename... Params> void append_comment(Format &&fmt, Params &&... args);

	// this class is thrown if abort() is called
	class abort_compilation : public emu_exception
	{
		friend class drcuml_block;
		abort_compilation() { }
	};

protected:
	// construction/destruction
	drcuml_block(drcuml_block const &) = delete;
	drcuml_block &operator=(drcuml_block const &) = delete;
	drcuml_block(drcuml_state &drcuml, u32 maxinst);
	~drcuml_block();

	void begin(bool invariant);

private:
	using instruction_array_ptr = std::unique_ptr<uml::instruction []>;

	// internal helpers
	void optimize();
	void disassemble();
	char const *get_comment_text(uml::instruction const &inst, std::string &comment);
	[[noreturn]] void overrun();

	// internal state
	drcuml_state &          m_drcuml;       // pointer back to the owning UML
	u32 const               m_maxinst;      // maximum number of instructions
	u32                     m_nextinst;     // next instruction to fill in the cache
	instruction_array_ptr   m_inst;         // pointer to the instruction list
	bool                    m_invariant;    // this is an invariant block
	bool                    m_inuse;        // this block is in use
};


// interface structure for a back-end
// MAME_SGI_HASH32: drcuml_state option flag - ask the native backend for a
// hash table with 32-bit L2 entries (x64 backend only; others ignore it)
constexpr u32 DRCUML_OPTION_HASH32 = 0x00000001;

class drcbe_interface
{
public:
	// allow deleting through base pointer
	virtual ~drcbe_interface();

	// required overrides
	virtual void reset() = 0;
	virtual int execute(uml::code_handle &entry) = 0;
	virtual void generate(drcuml_block &block, uml::instruction const *instlist, u32 numinst) = 0;
	virtual bool hash_exists(u32 mode, u32 pc) const noexcept = 0;
	// MAME_SGI_HASH_TABLE_STATS: L2 tables in use per populated mode, plus the
	// number of L1/L2 tables allocated so far (the permanent footprint).
	// Backends without it report nothing.
	struct hash_stats_t { std::vector<std::pair<u32, u32>> modes; std::vector<std::array<u32, 4>> l2s; std::array<u64, 6> slices_by_l2bits{}; std::size_t l1_allocated = 0, l2_allocated = 0, table_bytes = 0; };  // l2s: mode, first pc of the slice, entries, distinct 4KB pages
	virtual void hash_stats(hash_stats_t &out) const noexcept { }
	virtual void hash_invalidate_range(u32 pcstart, u32 pcend) noexcept = 0;
	// MAME_SGI_DRC_GET_CODEPTR_ACCESSOR (permanent, no #ifdef): exposes
	// drc_hash_table's existing internal get_codeptr() lookup (drcbeut.h)
	// through the public backend interface. Currently active consumer:
	// mips3.h's debug_get_codeptr() host-gdb helper (gated by the
	// separate MIPS3_DEBUG_HELPERS, which is defined/active). Originally
	// added alongside mips3drc.cpp's guard-based per-call-site inline
	// cache prototype (see wiki/drc-optimizations.md) - that consumer is
	// currently disabled (MAME_SGI_CALLSITE_IC_PROTOTYPE commented out,
	// confirmed not compiled into the binary), but this accessor stayed
	// since the debug helper still needs it. Implemented in every
	// concrete backend (drcbex64.cpp/drcbec.cpp/drcbearm64.cpp) since
	// this is a pure virtual - all three just forward to their own
	// m_hash.get_codeptr(), matching this method's own name.
	virtual drccodeptr get_codeptr(u32 mode, u32 pc) const noexcept = 0;
	virtual void get_info(drcbe_info &info) const noexcept = 0;
	virtual bool logging() const noexcept { return false; }

protected:
	// base constructor
	drcbe_interface(drcuml_state &drcuml, drc_cache &cache, device_t &device);

	// internal state
	drcuml_state &                  m_drcuml;      // pointer back to our owner
	drc_cache &                     m_cache;       // pointer to the cache
	device_t &                      m_device;      // CPU device we are associated with
	std::vector<address_space *>    m_space;       // pointers to CPU's address space
	drcuml_machine_state &          m_state;       // state of the machine (in near cache)
};


// structure describing UML generation state
class drcuml_state
{
public:
	// construction/destruction
	drcuml_state(device_t &device, drc_cache &cache, u32 flags, int modes, int addrbits, int ignorebits, u32 max_sequence_length);
	~drcuml_state();

	// getters
	device_t &device() const { return m_device; }
	drc_cache &cache() const { return m_cache; }
	u32 max_sequence_length() const { return m_max_sequence_length; }

	// reset the state
	void reset();
	int execute(uml::code_handle &entry) { return m_beintf->execute(entry); }

	// code generation
	drcuml_block &begin_block(u32 maxinst) { return begin_block(maxinst, false); }
	drcuml_block &begin_invariant_block(u32 maxinst) { return begin_block(maxinst, true); }

	// back-end interface
	void get_backend_info(drcbe_info &info) const { m_beintf->get_info(info); }
	bool hash_exists(u32 mode, u32 pc) const { return m_beintf->hash_exists(mode, pc); }
	void hash_stats(drcbe_interface::hash_stats_t &out) const { m_beintf->hash_stats(out); }  // MAME_SGI_HASH_TABLE_STATS
	void hash_invalidate_range(u32 pcstart, u32 pcend) { m_beintf->hash_invalidate_range(pcstart, pcend); }
	drccodeptr get_codeptr(u32 mode, u32 pc) const { return m_beintf->get_codeptr(mode, pc); }  // MAME_SGI_DRC_GET_CODEPTR_ACCESSOR
	void generate(drcuml_block &block, uml::instruction *instructions, u32 count) { m_beintf->generate(block, instructions, count); }

	// handle management
	uml::code_handle *handle_alloc(char const *name);

	// symbol management
	void symbol_add(void *base, u32 length, char const *name);
	char const *symbol_find(void *base, u32 *offset = nullptr);

	// logging
	bool logging() const { return bool(m_umllog); }
	template <typename Format, typename... Params>
	void log_printf(Format &&fmt, Params &&...args)
	{
		log_vprintf(util::make_format_argument_pack(std::forward<Format>(fmt), std::forward<Params>(args)...));
	}
	void log_vprintf(util::format_argument_pack<char> const &args);
	void log_flush() { if (logging()) m_umllog->flush(); }
	bool logging_native() const { return m_beintf->logging(); }

private:
	// symbol class
	class symbol
	{
	public:
		// construction/destruction
		symbol(void *base, u32 length, char const *name)
			: m_base(reinterpret_cast<drccodeptr>(base))
			, m_length(length)
			, m_name(name)
		{ }

		// getters
		bool includes(drccodeptr search) const { return (m_base <= search) && ((m_base + m_length) > search); }
		drccodeptr base() const { return m_base; }
		std::string const &name() const { return m_name; }

	private:
		// internal state
		drccodeptr  m_base;     // base of the symbol
		u32         m_length;   // length of the region covered
		std::string m_name;     // name of the symbol
	};

	class block_impl : public drcuml_block
	{
	public:
		block_impl(drcuml_state &drcuml, u32 maxinst) : drcuml_block(drcuml, maxinst) { }
		~block_impl() = default;

		using drcuml_block::begin;
	};

	drcuml_block &begin_block(u32 maxinst, bool invariant);

	// internal state
	device_t &                              m_device;           // CPU device we are associated with
	drc_cache &                             m_cache;            // pointer to the codegen cache
	u32 const                               m_max_sequence_length; // maximum length in bytes of a generated sequence
	std::unique_ptr<drcbe_interface> const  m_beintf;           // backend interface pointer
	std::unique_ptr<std::ostream> const     m_umllog;           // handle to the UML logfile
	std::list<block_impl>                   m_blocklist;        // list of active blocks
	std::list<uml::code_handle>             m_handlelist;       // list of active handles
	std::list<symbol>                       m_symlist;          // list of symbols
};



//**************************************************************************
//  MEMBER TEMPLATES
//**************************************************************************

//-------------------------------------------------
//  comment - attach a comment to the current
//  output location in the specified block
//-------------------------------------------------

template <typename Format, typename... Params>
inline void drcuml_block::append_comment(Format &&fmt, Params &&... args)
{
	// do the printf
	std::string const temp(util::string_format(std::forward<Format>(fmt), std::forward<Params>(args)...));

	// allocate space in the cache to hold the comment
	char *const comment = reinterpret_cast<char *>(invariant()
			? m_drcuml.cache().alloc_invariant(temp.length() + 1, std::align_val_t(alignof(char)))
			: m_drcuml.cache().alloc_transient(temp.length() + 1, std::align_val_t(alignof(char))));
	if (comment)
	{
		strcpy(comment, temp.c_str());

		// add an instruction with a pointer
		append().comment(comment);
	}
}


#endif // MAME_CPU_DRCUML_H
