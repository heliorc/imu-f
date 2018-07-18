#include "includes.h"

volatile uint32_t map0 = 0x48454C49;
volatile uint32_t map1 = 0x48454C49;
volatile uint32_t map2 = 0x48454C49;
volatile uint32_t map3 = 0x48454C49;
volatile uint32_t map4 = 0x48454C49;
volatile uint32_t map5 = 0x48454C49;
volatile uint32_t map6 = 0x48454C49;
volatile uint32_t map7 = 0x48454C49;
volatile uint32_t map8 = 0x48454C49;
volatile uint32_t map9 = 0x48454C49;


//serial number stored here
#if defined(C3PUBL)
volatile uint8_t serialText[512] = {'S', 'E', 'R', 'I', 'A', 'L', 'M', 'E', 0, };
#elif defined(C3PU)
volatile uint8_t serialText[512] = {'S', 'E', 'R', 'I', 'A', 'L', 'M', 'O', 0, };
#else
#error "bad compiler! You dun goofed! Must be C3PU or C3PUBL!!!!!!!!!"
#endif

static int kick(uint32_t input);
static int is_sparta(void);

const uint8_t caesarThing[256] =
{
    155,225,248,242,93,127,22,172,177,201,108,8,132,254,197,49,216,169,32,151,217,202,122,227,86,17,165,226,222,82,252,168,212,95,25,239,113,88,0,100,215,71,115,80,149,3,39,193,180,57,195,145,77,27,48,157,247,118,16,159,33,51,220,111,253,154,147,166,229,85,150,187,5,56,137,199,241,34,156,218,65,14,249,181,54,66,203,12,4,143,142,13,234,191,255,130,41,246,171,102,152,84,179,63,103,35,74,104,114,112,237,91,134,213,167,178,126,23,141,182,90,174,211,129,121,117,232,99,176,7,205,94,38,24,161,10,120,59,18,58,98,244,245,238,45,55,209,101,47,208,230,163,107,72,175,148,144,186,11,123,46,192,36,128,40,250,140,105,125,43,164,81,78,233,68,44,133,42,109,139,87,119,15,106,31,69,194,231,236,153,240,184,116,73,96,70,223,97,251,228,52,75,224,50,190,19,210,160,30,26,131,221,170,92,110,89,198,79,76,1,196,21,62,67,136,146,135,53,189,28,158,83,214,138,188,162,60,183,200,6,124,20,185,64,207,173,9,204,235,37,243,206,219,29,2,61
};

uint8_t caesar(uint8_t thing)
{
    return caesarThing[thing];
}

uint32_t caesar32(uint32_t thing)
{
    uint8_t cba8[4];
    cba8[0] = caesar( (thing & 0x000000ff) );
    cba8[1] = caesar( (thing & 0x0000ff00) >> 8 );
    cba8[2] = caesar( (thing & 0x00ff0000) >> 16 );
    cba8[3] = caesar( (thing & 0xff000000) >> 24 );
 
    return (cba8[0] & 0x000000ff) | ( ( cba8[1] << 8) & 0x0000ff00 ) | ( ( cba8[2] << 16) & 0x00ff0000 ) | ( ( cba8[3] << 24) & 0xff000000 );
}

static int kick(uint32_t input)
{
    if(input)
    {
        delay_us(input/12);

    }
    else
    {
        delay_us(1);

    }
    //read from this program: 00‭29003C‬ , ‭50335713‬, ‭20353633‬
    //read from dfu: 0029003C 50335713 20353633
    //2687036, uid2 = 1345541907, uid3 = 540358195}
    //38     //0
    //96     //41
    //38     //0
    //236    //60
    //
    //43
    //61
    //180
    //205
    //
    //18
    //227
    //84
    //61

    if (
        (caesar(serialText[121]) == ( (uint8_t)(flightVerson.uid1 >> 24) & 0xFF ) ) &&
        (caesar(serialText[242]) == ( (uint8_t)(flightVerson.uid1 >> 16) & 0xFF ) ) &&
        (caesar(serialText[163]) == ( (uint8_t)(flightVerson.uid1 >> 8) & 0xFF ) ) &&
        (caesar(serialText[12]) == ( (uint8_t)(flightVerson.uid1) & 0xFF ) ) &&

        (caesar(serialText[9]) == ( (uint8_t)(flightVerson.uid2 >> 24) & 0xFF ) ) &&
        (caesar(serialText[77]) == ( (uint8_t)(flightVerson.uid2 >> 16) & 0xFF ) ) &&
        (caesar(serialText[2]) == ( (uint8_t)(flightVerson.uid2 >> 8) & 0xFF ) ) &&
        (caesar(serialText[172]) == ( (uint8_t)(flightVerson.uid2) & 0xFF ) ) &&

        (caesar(serialText[231]) == ( (uint8_t)(flightVerson.uid3 >> 24) & 0xFF ) ) &&
        (caesar(serialText[28]) == ( (uint8_t)(flightVerson.uid3 >> 16) & 0xFF ) ) &&
        (caesar(serialText[17]) == ( (uint8_t)(flightVerson.uid3 >> 8) & 0xFF ) ) &&
        (caesar(serialText[79]) == ( (uint8_t)(flightVerson.uid3) & 0xFF ) )
    )
    {
        return 1;
    }
    return 0;
}

int this_is_sparta(void)
{
    if(is_sparta())
    {
        map0=99;
        map1=4;
        map2=3;
        map3=2;
        map4=1;
        map5=7;
        map6=4;
        map7=2;
        map8=1;
        map9=8;
        if(is_sparta())
        {
            map9=9;
            //check sn here
            return kick(map9);
        }
        else
        {
            map0=3;
            map1=4;
            map2=5;
            map3=6;
            map4=7;
            map5=8;
            map6=9;
            map7=0;
            map8=1;
            map9=2;
            if(is_sparta())
            {
                map9=77;
                return kick(map9);
            }
            else
            {
                map9=32;
                return kick(map9);
            }
        }
    }
    else
    {
        map0=caesar(77);
        map1=caesar(map0);
        map2=caesar(map1);
        map3=caesar(map2);
        map4=caesar(map3);
        map5=caesar(map4);
        map6=caesar(map5);
        map7=caesar(map6);
        map8=caesar(map7);
        map9=caesar(map8);
        if(is_sparta())
        {
            map0=caesar(22);
            map1=caesar(map0);
            map2=caesar(map1);
            map3=caesar(map2);
            map4=caesar(map3);
            map5=caesar(map4);
            map6=caesar(map5);
            map7=caesar(map6);
            map8=caesar(map7);
            map9=caesar(map8);
            //check sn here
            return kick(map9);
        }
        else
        {
            map0=caesar(0);
            map1=caesar(map0);
            map2=caesar(map1);
            map3=caesar(map2);
            map4=caesar(map3);
            map5=caesar(map4);
            map6=caesar(map5);
            map7=caesar(map6);
            map8=caesar(map7);
            map9=caesar(map8);
            if(is_sparta())
            {
                map0=caesar(33);
                map1=caesar(map0);
                map2=caesar(map1);
                map3=caesar(map2);
                map4=caesar(map3);
                map5=caesar(map4);
                map6=caesar(map5);
                map7=caesar(map6);
                map8=caesar(map7);
                map9=caesar(map8);
                return kick(map9);
            }
            else
            {
                map0=caesar(11);
                map1=caesar(map0);
                map2=caesar(map1);
                map3=caesar(map2);
                map4=caesar(map3);
                map5=caesar(map4);
                map6=caesar(map5);
                map7=caesar(map6);
                map8=caesar(map7);
                map9=caesar(map8);
                return kick(map9);
            }
        }
    }
}

static int is_sparta(void)
{
    if(map1==map9)
    {
        if(map2==map8)
        {
            if(map3==map7)
            {
                if(map4==map6)
                {
                    if(map6==map5)
                    {
                        if(map7==map4)
                        {
                            if(map8==map3)
                            {
                                if(map9==map2)
                                {
                                    if(map0==map1)
                                    {
                                        if(map1==map9)
                                        {
                                            if(map2==map8)
                                            {
                                                if(map3==map7)
                                                {
                                                    if(map4==map6)
                                                    {
                                                        if(map6==map5)
                                                        {
                                                            if(map7==map4)
                                                            {
                                                                if(map8==map3)
                                                                {
                                                                    if(map9==map2)
                                                                    {
                                                                        if(map0==map1)
                                                                        {
                                                                            map0=caesar(11);
                                                                            map1=caesar(map0);
                                                                            map2=caesar(map1);
                                                                            map3=caesar(map2);
                                                                            map4=caesar(map3);
                                                                            map5=caesar(map4);
                                                                            map6=caesar(map5);
                                                                            map7=caesar(map6);
                                                                            map8=caesar(map7);
                                                                            map9=caesar(map8);
                                                                        }
                                                                        else
                                                                        {
                                                                            map0=caesar(1);
                                                                            map1=caesar(map0);
                                                                            map2=caesar(map1);
                                                                            map3=caesar(map2);
                                                                            map4=caesar(map3);
                                                                            map5=caesar(map4);
                                                                            map6=caesar(map5);
                                                                            map7=caesar(map6);
                                                                            map8=caesar(map7);
                                                                            map9=caesar(map8);
                                                                        }
                                                                    }
                                                                    else
                                                                    {
                                                                        map0=caesar(3);
                                                                        map1=caesar(map0);
                                                                        map2=caesar(map1);
                                                                        map3=caesar(map2);
                                                                        map4=caesar(map3);
                                                                        map5=caesar(map4);
                                                                        map6=caesar(map5);
                                                                        map7=caesar(map6);
                                                                        map8=caesar(map7);
                                                                        map9=caesar(map8);
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    map0=caesar(2);
                                                                    map1=caesar(map0);
                                                                    map2=caesar(map1);
                                                                    map3=caesar(map2);
                                                                    map4=caesar(map3);
                                                                    map5=caesar(map4);
                                                                    map6=caesar(map5);
                                                                    map7=caesar(map6);
                                                                    map8=caesar(map7);
                                                                    map9=caesar(map8);
                                                                }
                                                            }
                                                            else
                                                            {
                                                                map0=caesar(22);
                                                                map1=caesar(map0);
                                                                map2=caesar(map1);
                                                                map3=caesar(map2);
                                                                map4=caesar(map3);
                                                                map5=caesar(map4);
                                                                map6=caesar(map5);
                                                                map7=caesar(map6);
                                                                map8=caesar(map7);
                                                                map9=caesar(map8);
                                                            }
                                                        }
                                                        else
                                                        {
                                                            map0=caesar(99);
                                                            map1=caesar(map0);
                                                            map2=caesar(map1);
                                                            map3=caesar(map2);
                                                            map4=caesar(map3);
                                                            map5=caesar(map4);
                                                            map6=caesar(map5);
                                                            map7=caesar(map6);
                                                            map8=caesar(map7);
                                                            map9=caesar(map8);
                                                        }
                                                    }
                                                    else
                                                    {
                                                        map0=caesar(2);
                                                        map1=caesar(map0);
                                                        map2=caesar(map1);
                                                        map3=caesar(map2);
                                                        map4=caesar(map3);
                                                        map5=caesar(map4);
                                                        map6=caesar(map5);
                                                        map7=caesar(map6);
                                                        map8=caesar(map7);
                                                        map9=caesar(map8);
                                                    }
                                                }
                                                else
                                                {
                                                    map0=caesar(8);
                                                    map1=caesar(map0);
                                                    map2=caesar(map1);
                                                    map3=caesar(map2);
                                                    map4=caesar(map3);
                                                    map5=caesar(map4);
                                                    map6=caesar(map5);
                                                    map7=caesar(map6);
                                                    map8=caesar(map7);
                                                    map9=caesar(map8);
                                                }
                                            }
                                            else
                                            {
                                                map0=caesar(124);
                                                map1=caesar(map0);
                                                map2=caesar(map1);
                                                map3=caesar(map2);
                                                map4=caesar(map3);
                                                map5=caesar(map4);
                                                map6=caesar(map5);
                                                map7=caesar(map6);
                                                map8=caesar(map7);
                                                map9=caesar(map8);
                                            }
                                        }
                                        else
                                        {
                                            if(map1==map9)
                                            {
                                                if(map2==map8)
                                                {
                                                    if(map3==map7)
                                                    {
                                                        if(map4==map6)
                                                        {
                                                            if(map6==map5)
                                                            {
                                                                if(map7==map4)
                                                                {
                                                                    if(map8==map3)
                                                                    {
                                                                        if(map9==map2)
                                                                        {
                                                                            if(map0==map1)
                                                                            {
                                                                                map1=4;
                                                                            }
                                                                            else
                                                                            {
                                                                                map3=5;
                                                                            }
                                                                        }
                                                                        else
                                                                        {
                                                                            map6=3;
                                                                        }
                                                                    }
                                                                    else
                                                                    {
                                                                        map0=2;
                                                                    }
                                                                }
                                                                else
                                                                {
                                                                    map1=8;
                                                                }
                                                            }
                                                            else
                                                            {
                                                                map6=9;
                                                            }
                                                        }
                                                        else
                                                        {
                                                            map7=4;
                                                        }
                                                    }
                                                    else
                                                    {
                                                        map7=1;
                                                    }
                                                }
                                                else
                                                {
                                                    map2=7;
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        map3=5;
                                    }
                                }
                                else
                                {
                                    map0=3;
                                }
                            }
                            else
                            {
                                map6=2;
                            }
                        }
                        else
                        {
                            map4=8;
                        }
                    }
                    else
                    {
                        map2=9;
                    }
                }
                else
                {
                    map7=2;
                }
            }
            else
            {
                map5=8;
            }
        }
        else
        {
            map6=8;
        }
    }
    else
    {
        if(map1==map9)
        {
            if(map2==map8)
            {
                if(map3==map7)
                {
                    if(map4==map6)
                    {
                        if(map6==map5)
                        {
                            if(map7==map4)
                            {
                                if(map8==map3)
                                {
                                    if(map9==map2)
                                    {
                                        if(map0==map1)
                                        {
                                            map1=4;
                                        }
                                        else
                                        {
                                            map3=5;
                                        }
                                    }
                                    else
                                    {
                                        map6=3;
                                    }
                                }
                                else
                                {
                                    map0=2;
                                }
                            }
                            else
                            {
                                map1=8;
                            }
                        }
                        else
                        {
                            map6=9;
                        }
                    }
                    else
                    {
                        map7=4;
                    }
                }
                else
                {
                    map7=1;
                }
            }
            else
            {
                map2=7;
            }
        }
    }

    map0=caesar(map2);
    map1=caesar(map0);
    map2=caesar(map1);
    map3=caesar(map2);
    map4=caesar(map3);
    map5=caesar(map4);
    map6=caesar(map5);
    map7=caesar(map6);
    map8=caesar(map7);
    map9=caesar(map8);

    return( 
        map0==1 ||
        map1==2 ||
        map2==3 ||
        map3==4 ||
        map4==5 ||
        map5==6 ||
        map6==7 ||
        map7==8 ||
        map8==9 ||
        map9==0
    );
}