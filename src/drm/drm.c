#include "includes.h"
#include "drm.h"

#define CHECK_ARRAY_SIZE 8

//#pragma GCC push_options
//#pragma GCC optimize ("O0")



#define CHECK1 1108426010
#define CHECK2 553648128
#define CHECK3 536887272
#define CHECK4 553648128
#define CHECK5 134233851
#define CHECK6 553648128
#define CHECK7 1159026037
#define CHECK8 134231043

void prerun_check(void)
{

#ifdef C3PUBL
    return;
#endif

volatile uint32_t* checkme1 = (uint32_t *)(0x20000000 + 16284);
volatile uint32_t* checkme2 = (uint32_t *)(0x20000000 + 16180);
volatile uint32_t* checkme3 = (uint32_t *)(0x20000000 + 16376);
volatile uint32_t* checkme4 = (uint32_t *)(0x20000000 + 16172);
volatile uint32_t* checkme5 = (uint32_t *)(0x20000000 + 16068);
volatile uint32_t* checkme6 = (uint32_t *)(0x20000000 + 16164);
volatile uint32_t* checkme7 = (uint32_t *)(0x20000000 + 15160);
volatile uint32_t* checkme8 = (uint32_t *)(0x20000000 + 16156);

    if(checkme1[0] != CHECK1)
        while(1);

    if(checkme2[0] != CHECK2)
        while(1);

    if(checkme3[0] != CHECK3)
        while(1);

    if(checkme4[0] != CHECK4)
        while(1);

    if(checkme5[0] != CHECK5)
        while(1);

    if(checkme6[0] != CHECK6)
        while(1);

    if(checkme7[0] != CHECK7)
        while(1);

    if(checkme8[0] != CHECK8)
        while(1);
}


int check_me(void)
{
    return 1;
    /*
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
    */
}
//#pragma GCC pop_options