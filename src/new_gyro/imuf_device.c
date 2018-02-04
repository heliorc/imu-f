#include "includes.h"

void imuf_write_data(filteredData_t* data) {
    
}

void imuf_write_quaternion_data(quaternion_record_t* data) {

}

void write_data(SPI_HandleTypeDef *hspi)
{
    if (HAL_SPI_GetState(&boardCommSPIHandle) == HAL_SPI_STATE_READY)
    {
        if ( imufCommandRx.command == 0x7f7f7f7F)
        {
            BootToAddress(THIS_ADDRESS);
        }
        if ( imufCommandRx.command == 0x63636363)
        {
            calibratingGyro = 1;
        }
        imufCommandRx.command = BC_NONE; //no command
        //transmit from the memptr to save CPU cycles, receive into the command rx struct, saves about 2us of time
        HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, memptr, (uint8_t *)&imufCommandRx, boardCommState.commMode); //profile: this takes 2.14us to run with O3 optimization
        HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); // a quick spike for EXTI
        HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); // a quick spike for EXTI
    }
}