
#pragma once
#include "includes.h"

extern void crc_config(void);
extern void append_crc_to_data_v(volatile uint32_t* data, uint32_t size);
extern uint32_t get_crc(volatile uint32_t* data, uint32_t size);