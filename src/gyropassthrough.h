#pragma once


#define BOOT_MAGIC_CODE (*((uint32_t *)0x2000000F))
#define BOOT_MAGIC_ADDRESS (*((uint32_t *)0x20000008))
#define BOOT_MAGIC_COUNTER (*((uint32_t *)0x20000004))
#define BOOT_MAGIC_CHECKSUM (*((uint32_t *)0x20000000))
#define BOOT_MAGIC_WORD 0xC001D00D

extern int BootHandler(void);
extern unsigned int SetBootChecksum(void);
extern unsigned int BootChecksum(void);
extern int ClearBootMagic(void);