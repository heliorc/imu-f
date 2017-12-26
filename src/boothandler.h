#pragma once

#define BOOT_MAGIC_WORD         0xC001D00D

extern int BootHandler(void);
extern unsigned int SetBootChecksum(void);
extern unsigned int BootChecksum(void);
extern int ClearBootMagic(void);