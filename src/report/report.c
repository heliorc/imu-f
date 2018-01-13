#include "includes.h"
#include "bootloader_commands.h"

void report_info(VersionInfoTypedef* info, char[]* buff, uint32_t bufferSize)
{
    snprintf(buff, bufferSize, "%i %i %i %i %i %i", 
    info.hardware, info.firmware, info.bootloader, info.uid1, info.uid2, info.uid3);
}
