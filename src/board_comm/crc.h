
#pragma once
#include "includes.h"

extern void crc_config(void);
extern void append_crc_to_data(uint32_t* data, uint32_t size);
extern uint32_t get_crc(uint32_t* data, uint32_t size);