#include "zephyr/modules/elf.h"
#include <zephyr/kernel.h>
#include <zephyr/modules/module.h>

/**
 * @brief Architecture specific function for relocating
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 * */
void arch_elf_relocate_rel(struct module *m, elf_rel_t *rel, elf_shdr_t *shdr, elf_sym_t *sym)
{
	uint8_t reloc_type = ELF32_R_TYPE(rel->r_info);
	uint32_t reloc_sym = ELF32_R_SYM(rel->r_info);

	printk("aarch32 rel: relocation info %x, type %d sym %d at offset %x\n", rel->r_info,
	       reloc_type, reloc_sym, rel->r_offset);
}

/**
 * @brief Architecture specific function for relocating
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 */
void arch_elf_relocate_dyn(struct module *m, elf_rel_t *rel, elf_shdr_t *shdr, elf_sym_t *sym)
{
}
