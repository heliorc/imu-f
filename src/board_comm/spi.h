#pragma once

#include "includes.h"

extern SPI_HandleTypeDef gyroSPIHandle;
extern DMA_HandleTypeDef hdmaGyroSPIRx;
extern DMA_HandleTypeDef hdmaGyroSPITx;
extern char gyroSpiRxBuffer[];
extern char gyroSpiTxBuffer[];

extern SPI_HandleTypeDef boardCommSPIHandle;
extern DMA_HandleTypeDef hdmaBoardCommSPIRx;
extern DMA_HandleTypeDef hdmaBoardCommSPITx;
extern char boardCommSpiRxBuffer[];
extern char boardCommSpiTxBuffer[];

typedef void (*spi_callback_function_pointer)(SPI_HandleTypeDef *hspi);
extern volatile spi_callback_function_pointer spiCallbackFunctionArray[];

typedef void (*spi_irq_callback_function_pointer)(void);
extern volatile spi_irq_callback_function_pointer spiIrqCallbackFunctionArray[];

extern void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
extern void spi_init(SPI_HandleTypeDef* spiHandle, SPI_TypeDef* instance, uint32_t baudscaler, uint32_t spi_mode, uint32_t irqn, uint32_t irqp, uint32_t irqsp);
extern void spi_dma_init(SPI_HandleTypeDef* spiHandle, DMA_HandleTypeDef* hdma_spi_rx, DMA_HandleTypeDef* hdma_spi_tx, DMA_Channel_TypeDef* rxDmaChannel, DMA_Channel_TypeDef* txDmaChannel);
extern void board_comm_spi_irq_callback(void);
extern void gyro_spi_irq_callback(void);