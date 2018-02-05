#include "includes.h"

void reset_mcu(void)
{
    //NVIC_SystemReset();
    __DSB();
    SCB->AIRCR  = ((0x5FA << SCB_AIRCR_VECTKEY_Pos) | (SCB->AIRCR & SCB_AIRCR_PRIGROUP_Msk) | SCB_AIRCR_SYSRESETREQ_Msk);
    __DSB();
    while(1);
}