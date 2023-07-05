/*
 * Copyright (c) 2023 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_MODULE_H
#define ZEPHYR_MODULE_H

#include <zephyr/sys/slist.h>
#include <zephyr/modules/elf.h>
#include <sys/types.h>

/**
 * @brief A symbol (named memory address)
 */
struct module_symbol {
	elf_addr addr;
	const char *name;
};

/**
 * @brief A symbol table
 *
 * An array of symbols
 */
struct module_symtable {
	elf_word sym_cnt;
	struct module_symbol *syms;
};


/**
 * @brief Enum of module memory regions for lookup tables
 */
enum module_mem {
	MOD_MEM_TEXT,
	MOD_MEM_DATA,
	MOD_MEM_RODATA,
	MOD_MEM_BSS,

	MOD_MEM_COUNT,
};

/**
 * @brief Enum of sections for lookup tables
 */
enum module_section {
	MOD_SECT_TEXT,
	MOD_SECT_DATA,
	MOD_SECT_RODATA,
	MOD_SECT_BSS,

	MOD_SECT_REL_TEXT,
	MOD_SECT_REL_DATA,
	MOD_SECT_REL_RODATA,
	MOD_SECT_REL_BSS,

	MOD_SECT_SYMTAB,
	MOD_SECT_STRTAB,
	MOD_SECT_SHSTRTAB,

	/* Should always be last */
	MOD_SECT_COUNT,
};

/**
 * @brief Loadable code module
 */
struct module {
	sys_snode_t _mod_list;

	/** Name of the module */
	char name[16];

	/** Lookup table of module memory regions */
	void *mem[MOD_MEM_COUNT];

	/** Total size of the module memory usage */
	size_t mem_size;

	/** Exported symbols from the module, may be used in other modules */
	struct module_symtable sym_tab;
};

/**
 * @brief Module loader context
 *
 * A source of an ELF stream/blob
 */
struct module_stream {
	int (*read)(struct module_stream *s, void *buf, size_t len);
	int (*seek)(struct module_stream *s, size_t pos);

	elf_ehdr_t hdr;
	elf_shdr_t sects[MOD_SECT_COUNT];
};

/**
 * @brief Read a length of bytes into the given buffer
 */
int module_read(struct module_stream *ms, void *buf, size_t len);

/**
 * @brief Seek to an absolute location of the module (elf) file
 */
int module_seek(struct module_stream *ms, size_t pos);

/**
 * @brief List head of loaded modules
 */
sys_slist_t *module_list(void);

/**
 * @brief Find a module from a name
 *
 * @param name String name of the module
 * @retval NULL if no module not found
 * @retval module if module found
 */
struct module *module_from_name(const char *name);

/**
 * @brief Load a module
 *
 * Loads an ELF into memory and provides a structure to work with it.
 *
 * What types of ELF streams are loadable depends on Kconfig flags and architecture
 * support of those files. On some architectures it may be possible to load relocatable or
 * exec code which subsequently has op codes transformed to mimic the behavior of a dynamicaly
 * linked elf with position independent code.
 *
 * Internally a module specific heap is used to allocate dynamic structures which are freed when
 * the module is unloaded.
 *
 * @param[in] modstr A byte stream like object that provides a source for loadable code
 * @param[in] name A string identifier for the module
 * @param[out] module A pointer to a statically allocated module struct
 *
 * @retval 0 Success
 * @retval -errno Error
 */
int module_load(struct module_stream *modstr, const char name[16], struct module **module);

/**
 * @brief Unload a module
 */
void module_unload(struct module *module);

/**
 * @brief Find the address for an arbitrary symbol name.
 */
void *module_find_sym(struct module_symtable *sym_table, const char *sym_name);

/**
 * @brief Architecture specific function for relocating
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 * */
void arch_elf_relocate(struct module_stream *ms, struct module *m, elf_rel_t *rel,
			   elf_shdr_t *shdr, elf_sym_t *sym);


#endif /* ELFLOADER_H */
