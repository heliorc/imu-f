#include "includes.h"

typedef void (*pFunction)(void);

/*
Boot modes:
RECOVERY_rfbl_app 1 //This is the recovery loader and thewre exists rfbl and an app
RECOVERY_app      2 //This is the only bootloader and there is an app
recovery_RFBL_app 3 //This is the second bootloader and there is a recovery loader and an app
APP               4 //This is an app and there's no bl (DFU updates only)
recovery_APP      5 //This is an app and there is only one BL
recovery_rfbl_APP 6 //This is an app and there are two boot loaders


#define BOOT_CODE_RECOVERY      0XB0074EC2
#define BOOT_CODE_BOOTLOADER    0XB007B007
#define BOOT_CODE_APP           0XB007A999


*/

static int CheckBootCrc(void);

static int CheckBootCrc(void)
{
    return(BOOT_MAGIC_CRC == BootCrc());
}

int BootToAddress(uint32_t address)
{
    DisableIrq();
    BOOT_MAGIC_CODE = BOOT_MAGIC_WORD;
    BOOT_MAGIC_ADDRESS = address;
    BOOT_MAGIC_COUNTER = 0;
    SetBootCrc();
    SystemReset();
    return(0);
}

int ClearBootMagic(void)
{
    BOOT_MAGIC_CODE = 0;
    BOOT_MAGIC_ADDRESS = 0;
    BOOT_MAGIC_COUNTER = 0;
    BOOT_MAGIC_CRC = 0;
    return(0);
}

unsigned int SetBootCrc(void)
{
    return(BOOT_MAGIC_CRC = BootCrc());
}

unsigned int BootCrc(void)
{
    return(BOOT_MAGIC_CODE + BOOT_MAGIC_ADDRESS + BOOT_MAGIC_COUNTER);
}

int BootHandler(void)
{

    pFunction JumpToApplication;
    uint32_t jumpAddress, address, bootMagic;

    //STEP 1, Check CRC
    if(!CheckBootCrc())
    {
        //crc check failed, clear boot magic and continue boot
        ClearBootMagic();
        return(0);
    }
    else if (THIS_ADDRESS == ADDR_FLASH_SECTOR_0)
    {
        //crc passed and we're in the recovery loader
        BOOT_MAGIC_COUNTER++; //increment boot counter and set boot CRC
        SetBootCrc(); //reset the boot crc

        if(BOOT_MAGIC_COUNTER > 3)
        {
            //we're boot looping, clear ram and continue boot as wer're in recovery already
            ClearBootMagic();
            return(0);
        }
    }


    //crc matches, check mem location, if the magic word is set we check if the boot location matches the code location
    if(BOOT_MAGIC_CODE == BOOT_MAGIC_WORD)
    {
        if(BOOT_MAGIC_ADDRESS == DFU_ADDRESS)
        {
            //boot to DFU requested, set DFU address, clear RAM and continue app
            address = DFU_ADDRESS;
            ClearBootMagic();
        }
        else if(CUSTOM_LOC_ADDRESS_DATA == THIS_ADDRESS)
        {
            //we're in the right spot, return 0 to continue boot, boot magic is to be cleared in the app
            return(0);
        }
        else
        {
            //wrong spot, let's reboot to new address, boot magic will clear in app
            address = BOOT_MAGIC_ADDRESS;
        }
    }

    //we got here so we need to jump
    jumpAddress = *(__IO uint32_t*)(address + 4);
    JumpToApplication = (pFunction)jumpAddress;

    // Initialize user application's Stack Pointer
    __set_MSP(*(__IO uint32_t*)address);
    JumpToApplication();

    return(0);  
}