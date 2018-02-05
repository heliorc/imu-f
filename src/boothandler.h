#pragma once


#define BOOT_MAGIC_CODE (*((uint32_t *)0x2000000F))
#define BOOT_MAGIC_ADDRESS (*((uint32_t *)0x20000008))
#define BOOT_MAGIC_COUNTER (*((uint32_t *)0x20000004))
#define BOOT_MAGIC_CHECKSUM (*((uint32_t *)0x20000000))
#define BOOT_MAGIC_WORD 0xC001D00D

extern int boot_to_address(uint32_t address);
extern int boot_handler(void);
extern unsigned int set_boot_checksum(void);
extern unsigned int boot_checksum(void);
extern int clear_boot_magic(void);