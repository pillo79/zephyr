#include <zephyr/kernel.h>
#include <zephyr/modules/module.h>
#include <zephyr/modules/elf.h>

#define R_ARM_ABS32 2
#define R_ARM_CALL 28

/**
 * @brief Architecture specific function for relocating partially linked (static) elf
 *
 * Elf files contain a series of relocations described in a section. These relocation
 * instructions are architecture specific and each architecture supporting modules
 * must implement this.
 *
 * The relocation codes for arm are well documented
 * https://github.com/ARM-software/abi-aa/blob/main/aaelf32/aaelf32.rst#relocation
 * */
void arch_elf_relocate(struct module_stream *ms, struct module *m, elf_rel_t *rel,
			   elf_shdr_t *shdr, elf_sym_t *sym)
{
	char sym_name[32], sec_name[32];
	elf_word sym_type;
	elf_shdr_t sym_shdr = {0};
	elf_word reloc_type = ELF32_R_TYPE(rel->r_info);
	elf_word reloc_sym = ELF32_R_SYM(rel->r_info);
	size_t pos = 0;

	printk("aarch32 rel: relocation info %x, type %d sym %d offset %d\n", rel->r_info,
	       reloc_type, reloc_sym, rel->r_offset);

	switch(reloc_type) {
		case R_ARM_ABS32:
			/* absolute relocation to a section */
			sym_type = ELF_ST_TYPE(sym->st_info);

			if (sym_type == STT_SECTION) {

				printk("symbol type is section!\n");
				pos = ms->hdr.e_shoff + ms->hdr.e_shentsize*reloc_sym;
				(void)module_seek(ms, pos);
				(void)module_read(ms, &sym_shdr, sizeof(sym_shdr));

				printk("sym shdr offset %d, sh_name %d\n", pos, sym_shdr.sh_name);

				size_t offs = ms->sects[MOD_SECT_SHSTRTAB].sh_offset +
					sym_shdr.sh_name;

				(void)module_seek(ms, offs);
				(void)module_read(ms, sym_name, 32);

			} else {
				size_t offs = ms->sects[MOD_SECT_STRTAB].sh_offset
					+ sym->st_name;

				(void)module_seek(ms, offs);
				(void)module_read(ms, sym_name, 32);
			}

			(void)module_seek(ms, ms->sects[MOD_SECT_STRTAB].sh_offset + shdr->sh_name);
			(void)module_read(ms, sec_name, 32);


			printk("absolute 32bit relocation at offset %d, symbol[%d]"
			       " (st_name %d, sym type %d, shndx %d) %s in section[%d] %s\n",
			       rel->r_offset,
			       reloc_sym,
			       sym->st_name,
			       sym_type,
			       sym->st_shndx,
			       sym_name,
			       shdr->sh_name,
			       sec_name);
			break;
		case R_ARM_CALL:
			/* TODO lookup symbol address, e.g. puts function */
			(void)module_seek(ms,
					  ms->sects[MOD_SECT_STRTAB].sh_offset
					  + sym->st_name);
			(void)module_read(ms, sym_name, 32);

			(void)module_seek(ms,
					  ms->sects[MOD_SECT_SHSTRTAB].sh_offset
					  + shdr->sh_name);
			(void)module_read(ms, sec_name, 32);

			printk("call relocation at offset %d, symbol[%d] %s in section[%d] %s\n",
			       rel->r_offset,
			       reloc_sym,
			       sym_name,
			       shdr->sh_name,
			       sec_name);
			break;
		default:
			printk("Unsupported relocation type %d\n", reloc_type);
			break;

	}
}
