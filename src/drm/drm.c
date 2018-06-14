#include "includes.h"
#include "drm.h"

#define CHECK_ARRAY_SIZE 4

#define CHECKL1 *(uint32_t *)0x20000010
#define CHECK1 0x00C001D0
#define CHECKL2 *(uint32_t *)0x20000014
#define CHECK2 0x00000086
#define CHECKL3 *(uint32_t *)0x20000018
#define CHECK3 0x000000B2
#define CHECKL4 *(uint32_t *)0x2000001C
#define CHECK4 0x00000070

volatile uint32_t checkArray[CHECK_ARRAY_SIZE];

void prerun_check(void)
{

#ifdef C3PUBL
    return; //don't do this on bl
#endif

    checkArray[0] = CHECKL1;
    checkArray[1] = CHECKL2;
    checkArray[2] = CHECKL3;
    checkArray[3] = CHECKL4;

    /*
    if(CHECKL1 != CHECK1)
        while(1);
    if(CHECKL2 != CHECK2)
        while(1);
    if(CHECKL3 != CHECK3)
        while(1);
    if(CHECKL4 != CHECK4)
        while(1);
    if(CHECKL5 != CHECK5)
        while(1);
    if(CHECKL6 != CHECK6)
        while(1);
    if(CHECKL7 != CHECK7)
        while(1);
    if(CHECKL8 != CHECK8)
        while(1);
    */
}


int check_me(void)
{

    static int counter = 0;
    switch(counter)
    {
        case 0:
            if(checkArray[counter++] != CHECK1) {
                return 0;
            }
        break;
        case 1:
            if(checkArray[counter++] != CHECK2) {
                return 0;
            }
        break;
        case 2:
            if(checkArray[counter++] != CHECK3) {
                return 0;
            }
        break;
        case 3:
            if(checkArray[counter++] != CHECK4) {
                return 0;
            }
            counter = 0;
        break;
        case 8:
        default:
        counter = 0;

    }
    return 1;

}