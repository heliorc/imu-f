#include "includes.h"



typedef void (*pFunction)(void);

static int CheckBootChecksum(void)
{
    return(BOOT_MAGIC_CHECKSUM == BootChecksum());
}

int BootToAddress(uint32_t address)
{
    __disable_irq();
    BOOT_MAGIC_CODE = BOOT_MAGIC_WORD;
    BOOT_MAGIC_ADDRESS = address;
    BOOT_MAGIC_COUNTER = 0;
    SetBootChecksum();
    NVIC_SystemReset();
    return(0);
}

int ClearBootMagic(void)
{
    BOOT_MAGIC_CODE = 0;
    BOOT_MAGIC_ADDRESS = 0;
    BOOT_MAGIC_COUNTER = 0;
    BOOT_MAGIC_CHECKSUM = 0;
    return(0);
}

unsigned int SetBootChecksum(void)
{
    return(BOOT_MAGIC_CHECKSUM = BootChecksum());
}

unsigned int BootChecksum(void)
{
    return(BOOT_MAGIC_CODE + BOOT_MAGIC_ADDRESS + BOOT_MAGIC_COUNTER);
}

int BootHandler(void)
{

    pFunction JumpToApplication;
    uint32_t jumpAddress, address, bootMagic;

    //STEP 1, Check Checksum
    if(!CheckBootChecksum())
    {
        //Checksum check failed, clear boot magic and continue boot
        ClearBootMagic();
        return(0);
    }
    else if (THIS_ADDRESS == (0x08000000))
    {
        //Checksum passed and we're in the recovery loader
        BOOT_MAGIC_COUNTER++; //increment boot counter and set boot Checksum
        SetBootChecksum(); //reset the boot Checksum

        if(BOOT_MAGIC_COUNTER > 3)
        {
            //we're boot looping, clear ram and continue boot as wer're in recovery already
            ClearBootMagic();
            return(0);
        }
    }


    //Checksum matches, check mem location, if the magic word is set we check if the boot location matches the code location
    if(BOOT_MAGIC_CODE == BOOT_MAGIC_WORD)
    {
        if(BOOT_MAGIC_ADDRESS == DFU_ADDRESS)
        {
            //boot to DFU requested, set DFU address, clear RAM and continue app
            address = DFU_ADDRESS;
            ClearBootMagic();
        }
        else if ( (BOOT_MAGIC_ADDRESS == 0) || (BOOT_MAGIC_ADDRESS == THIS_ADDRESS) )
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