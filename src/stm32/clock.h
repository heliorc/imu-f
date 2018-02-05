#pragma once

#define simpleDelay_ASM(us, clockspeed) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (clockspeed*us) : "memory"\
		      );\
} while(0)

extern volatile uint32_t systemUsTicks;

extern void clock_config(void);
extern void update_millis_clock(void);
extern void delay_ms(uint32_t ms);
extern void delay_us(uint32_t us);
extern uint32_t millis(void);
extern uint32_t micros(void);