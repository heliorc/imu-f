#pragma once

#define BOOT_CODE_RECOVERY      0xB0074EC2
#define BOOT_CODE_BOOTLOADER    0xB007B007
#define BOOT_CODE_APP           0xB007A999
#define BOOT_CODE_DFU           0xB007DFDF
#define BOOT_MAGIC_WORD         0xC001D00D

extern int BootHandler(void);
extern unsigned int SetBootCrc(void);
extern unsigned int BootCrc(void);
extern int ClearBootMagic(void);