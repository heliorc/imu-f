#include "includes.h"
#include "version.h"
#include "bootloader.h"

flightVersionInfoTypedef_t flightVerson;

void set_version(void)
{
    flightVerson.hardware   = HARDWARE_VERSION;
    flightVerson.firmware   = FIRMWARE_VERSION;   //in bootloader code this will end up reporting the firmware version known when the bootloader was added
    flightVerson.bootloader = BOOTLOADER_VERSION; //in flight code this will end up reporting the latest known bootloader version
    flightVerson.uid1       = (uint32_t)((*((uint32_t *)(UID_BASE))));
    flightVerson.uid2       = (uint32_t)((*((uint32_t *)(UID_BASE + 4U))));
    flightVerson.uid3       = (uint32_t)((*((uint32_t *)(UID_BASE + 8U))));
}
