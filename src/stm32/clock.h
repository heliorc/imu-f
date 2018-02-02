#pragma once

extern volatile uint32_t systemUsTicks;

extern uint32_t millis(void);
extern void clock_config(void);
extern void update_millis_clock(void);