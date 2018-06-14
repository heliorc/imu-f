#include "includes.h"
#include "drm.h"

#define CHECK_ARRAY_SIZE 8

#define CHECKL1 *(uint32_t *)0x20000050
#define CHECK1 0xC88A711C
#define CHECKL2 *(uint32_t *)0x20000054
#define CHECK2 0xD2889D88
#define CHECKL3 *(uint32_t *)0x20000058
#define CHECK3 0x845EB36C
#define CHECKL4 *(uint32_t *)0x2000005C
#define CHECK4 0x0DA0C837
#define CHECKL5 *(uint32_t *)0x20000060
#define CHECK5 0x0B3CDFA3
#define CHECKL6 *(uint32_t *)0x20000064
#define CHECK6 0x05E68DA7
#define CHECKL7 *(uint32_t *)0x20000068
#define CHECK7 0x5A45BEC2
#define CHECKL8 *(uint32_t *)0x2000006C
#define CHECK8 0xC078A797

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
    checkArray[4] = CHECKL5;
    checkArray[5] = CHECKL6;
    checkArray[6] = CHECKL7;
    checkArray[7] = CHECKL8;
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
        break;
        case 4:
            if(checkArray[counter++] != CHECK5) {
                return 0;
            }
        break;
        case 5:
            if(checkArray[counter++] != CHECK6) {
                return 0;
            }
        break;
        case 6:
            if(checkArray[counter++] != CHECK7) {
                return 0;
            }
        break;
        case 7:
            if(checkArray[counter++] != CHECK8) {
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