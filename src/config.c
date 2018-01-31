#include "includes.h"

flightVersionInfoTypedef_t flightVerson;

#define HARDWARE_VERSION 101
#define FIRMWARE_VERSION 101

void set_version(void)
{
    flightVerson.hardware   = HARDWARE_VERSION;
    flightVerson.firmware   = FIRMWARE_VERSION;
    flightVerson.bootloader = 0; //we're not in a bootloader, no reason to report this, unless we read from the flash
    flightVerson.uid1       = (uint32_t)((*((uint32_t *)(UID_BASE))));
    flightVerson.uid2       = (uint32_t)((*((uint32_t *)(UID_BASE + 4U))));
    flightVerson.uid3       = (uint32_t)((*((uint32_t *)(UID_BASE + 8U))));

}
