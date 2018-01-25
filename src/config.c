#include "includes.h"

flightVersionInfoTypedef_t flightVerson;

#define HARDWARE_VERSION 101
#define FIRMWARE_VERSION 101

void set_version(void)
{
    flightVerson.hardware   = HARDWARE_VERSION;
    flightVerson.firmware   = FIRMWARE_VERSION;
    flightVerson.bootloader = 0; //we're not in a bootloader, no reason to report this, unless we read from the flash
    flightVerson.uid1       = U_ID_0;
    flightVerson.uid2       = U_ID_1;
    flightVerson.uid3       = U_ID_2;

}
