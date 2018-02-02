#pragma once

extern volatile uint32_t systemUsTicks;

extern void clock_config(void);
extern void update_millis_clock(void);
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);
extern uint32_t millis(void);
extern uint32_t micros(void);