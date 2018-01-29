#pragma once

extern volatile uint32_t systemUsTicks;

extern void SystemClock_Config(void);
extern void UpdateMillisClock(void);