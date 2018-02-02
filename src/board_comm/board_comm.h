#pragma once
#include "includes.h"

#define COM_BUFFER_SIZE 52


extern SPI_HandleTypeDef boardCommSPIHandle;
extern DMA_HandleTypeDef hdmaBoardCommSPIRx;
extern DMA_HandleTypeDef hdmaBoardCommSPITx;
extern volatile imufCommand_t imufCommandRx;

extern volatile boardCommState_t boardCommState;
extern volatile uint32_t timeBoardCommSetupIsr;

extern void board_comm_init(void);
extern void board_comm_callback_function(SPI_HandleTypeDef *hspi);
extern int  parse_imuf_command(volatile imufCommand_t* newCommand);
extern void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
extern void rewind_board_comm_spi(void);
extern void board_comm_spi_irq_callback(void);
extern void check_board_comm_setup_timeout(void);