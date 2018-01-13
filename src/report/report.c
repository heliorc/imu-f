#include "includes.h"

VersionInfoTypedef info;

void get_report_info(messageHandle, char* txData, char* rxData)
{
    // ensure that SPI resources are available, but don't block if they are not
    while (HAL_SPI_GetState(messageHandle) != HAL_SPI_STATE_READY);
    snprintf(txData, 256, "%i%i%i%i%i%i", info.hardware, info.firmware, info.bootloader, info.uid1, info.uid2, info.uid3);
}