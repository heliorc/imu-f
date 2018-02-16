#pragma once

#include "includes.h"

typedef void (*spi_tx_done_callback)(void);
extern volatile spi_tx_done_callback spiCallbackFunctionArray[];

//typedef void (*spi_callback_function_pointer)(SPI_HandleTypeDef *hspi);
//extern volatile spi_callback_function_pointer spiCallbackFunctionArray[];

//typedef void (*spi_irq_callback_function_pointer)(void);
//extern volatile spi_irq_callback_function_pointer spiIrqCallbackFunctionArray[];

extern void spi_init(SPI_InitTypeDef *spiInitStructure, DMA_InitTypeDef *dmaInitStructure, SPI_TypeDef *spi, uint16_t mode, uint16_t nss, uint16_t cpol, uint16_t cpha, uint16_t BaudRatePrescaler);
extern void spi_fire_dma(SPI_TypeDef *spi, DMA_Channel_TypeDef *txDma, DMA_Channel_TypeDef *rxDma, DMA_InitTypeDef *dmaInitStructure, uint32_t *size, volatile uint8_t *txBuff, volatile uint8_t *rxBuff);
extern void cleanup_spi(SPI_TypeDef *spi, DMA_Channel_TypeDef *txDma, DMA_Channel_TypeDef *rxDma, uint32_t resetMask);
extern int spi_transfer_blocking(SPI_TypeDef* spi, const uint8_t* txBuff, uint8_t* rxBuff, int len);