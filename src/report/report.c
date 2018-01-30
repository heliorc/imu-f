#include "includes.h"
#include "report.h"

VersionInfoTypedef_t info;

//not using this
void get_report_info(SPI_HandleTypeDef* messageHandle, uint8_t* rxData, uint8_t* txData)
{
    // ensure that SPI resources are available, but don't block if they are not
    while (HAL_SPI_GetState(messageHandle) != HAL_SPI_STATE_READY);
    snprintf((char *)txData, 256, "%lu%lu%lu%lu%lu%lu", info.hardware, info.firmware, info.bootloader, info.uid1, info.uid2, info.uid3);
}