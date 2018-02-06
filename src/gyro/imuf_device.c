#include "includes.h"
#include "gyro.h"
#include "quaternions.h"
#include "filter.h"
#include "board_comm.h"

void imuf_init(void)
{

    //  // setup board_comm spi mappings and gpio init
    // single_gpio_init(BOARD_COMM_MISO_PORT, BOARD_COMM_MISO_PIN_SRC, BOARD_COMM_MISO_PIN, BOARD_COMM_MISO_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    // single_gpio_init(BOARD_COMM_MOSI_PORT, BOARD_COMM_MOSI_PIN_SRC, BOARD_COMM_MOSI_PIN, BOARD_COMM_MOSI_ALTERNATE, GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    // single_gpio_init(BOARD_COMM_SCK_PORT,  BOARD_COMM_SCK_PIN_SRC,  BOARD_COMM_SCK_PIN,  BOARD_COMM_SCK_ALTERNATE,  GPIO_Mode_AF, GPIO_OType_PP, GPIO_PuPd_NOPULL);

    // //setup NSS GPIO if need be then init SPI and DMA for the SPI based on NSS type
    // #ifndef BOARD_COMM_CS_TYPE
    //     //exti is used for NSS
    //     gpio_exti_init(BOARD_COMM_EXTI_PORT, BOARD_COMM_EXTI_PORT_SRC, BOARD_COMM_EXTI_PIN, BOARD_COMM_EXTI_PIN_SRC, BOARD_COMM_EXTI_LINE, EXTI_Trigger_Rising, BOARD_COMM_EXTI_IRQn, BOARD_COMM_EXTI_ISR_PRE_PRI, BOARD_COMM_EXTI_ISR_SUB_PRI);
    //     spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, SPI_NSS_Soft, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_BaudRatePrescaler_2); 
    // #else
    //     single_gpio_init(BOARD_COMM_CS_PORT, BOARD_COMM_CS_PIN_SRC, BOARD_COMM_CS_PIN, BOARD_COMM_CS_ALTERNATE, BOARD_COMM_CS_TYPE, GPIO_OType_PP, GPIO_PuPd_NOPULL);
    //     spi_init(&boardCommSpiInitStruct, &boardCommDmaInitStruct, BOARD_COMM_SPI, SPI_Mode_Slave, BOARD_COMM_CS_TYPE, SPI_CPOL_Low, SPI_CPHA_1Edge, SPI_BaudRatePrescaler_2);
    // #endif

    // single_gpio_init(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN_SRC, BOARD_COMM_DATA_RDY_PIN, 0, GPIO_Mode_OUT, GPIO_OType_PP, GPIO_PuPd_NOPULL);


    //setup DMA
    // if (HAL_SPI_GetState(&boardCommSPIHandle) == HAL_SPI_STATE_READY)
    // {
    //     if ( bcRx.command == 0x7f7f7f7F)
    //     {
    //         BootToAddress(THIS_ADDRESS);
    //     }
    //     if ( bcRx.command == 0x63636363)
    //     {
    //         calibratingGyro = 1;
    //     }
    //     bcRx.command = BC_NONE; //no command
    //     //transmit from the memptr to save CPU cycles, receive into the command rx struct, saves about 2us of time
    //     HAL_SPI_TransmitReceive_DMA(&boardCommSPIHandle, memptr, (uint8_t *)&bcRx, boardCommState.commMode); //profile: this takes 2.14us to run with O3 optimization
    //     HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 1); // a quick spike for EXTI
    //     HAL_GPIO_WritePin(BOARD_COMM_DATA_RDY_PORT, BOARD_COMM_DATA_RDY_PIN, 0); // a quick spike for EXTI
    // }
}