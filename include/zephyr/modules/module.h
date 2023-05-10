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
 * @brief Loadable code module
 */
struct module {
	sys_snode_t _mod_list;
	char name[16];
	elf_addr virt_start_addr;
	elf_addr load_start_addr;
	elf_word mem_sz;
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
	elf_shdr_t strtab;
	elf_shdr_t shstrtab;
	elf_shdr_t symtab;
	elf_shdr_t text;
	elf_shdr_t rodata;
	elf_shdr_t bss;
	elf_shdr_t data;
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
 * @brief Dynamically link symbols for a module
 */
int module_link(struct module *module, struct module_symtable *syms);

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
void arch_elf_relocate_rel(struct module_stream *ms, struct module *m, elf_rel_t *rel,
			   elf_shdr_t *shdr, elf_sym_t *sym);

/**
 * @brief Architecture specific function for relocating
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 */
void arch_elf_relocate_dyn(struct module_stream *ms, struct module *m, elf_rel_t *rel,
			   elf_shdr_t *shdr, elf_sym_t *sym);


#endif /* ELFLOADER_H */
