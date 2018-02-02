#include "includes.h"
#include "boothandler.h"

typedef void (*pFunction)(void);

static int check_boot_checksum(void)
{
    return(BOOT_MAGIC_CHECKSUM == boot_checksum());
}

int boot_to_address(uint32_t address)
{
    //do an actual reboot to the right address
    __disable_irq(); //CMSIS define is okay here
    BOOT_MAGIC_CODE = BOOT_MAGIC_WORD;
    BOOT_MAGIC_ADDRESS = address;
    BOOT_MAGIC_COUNTER = 0;
    set_boot_checksum();
    reset_mcu(); //points to a std per driver functions, wrapped in reset.c
    return(0);
}

int clear_boot_magic(void)
{
    BOOT_MAGIC_CODE = 0;
    BOOT_MAGIC_ADDRESS = 0;
    BOOT_MAGIC_COUNTER = 0;
    BOOT_MAGIC_CHECKSUM = 0;
    return(0);
}

unsigned int set_boot_checksum(void)
{
    return(BOOT_MAGIC_CHECKSUM = boot_checksum());
}

unsigned int boot_checksum(void)
{
    return(BOOT_MAGIC_CODE + BOOT_MAGIC_ADDRESS + BOOT_MAGIC_COUNTER);
}

int boot_handler(void)
{

    pFunction jumpToApplication;
    uint32_t jumpAddress, address, bootMagic;

    volatile uint32_t watchMe = THIS_ADDRESS;
    //STEP 1, Check Checksum
    if(!check_boot_checksum())
    {
        //Checksum check failed, clear boot magic and continue boot
        clear_boot_magic();
        return(0);
    }
    else if (THIS_ADDRESS == (BL_ADDRESS))
    {
        //Checksum passed and we're in the bootloader
        BOOT_MAGIC_COUNTER++; //increment boot counter and set boot Checksum, we only do this when THIS_ADDRESS is BL_ADDRESS, so it doesn't happen multiple times in a single boot
        set_boot_checksum(); //reset the boot Checksum

        if(BOOT_MAGIC_COUNTER > 3)
        {
            //we're boot looping, clear ram and continue boot as we're in recovery already, set BOOT_MAGIC_ADDRESS to BL_ADDRESS to keep bootloader in recovery mode
            clear_boot_magic();
            BOOT_MAGIC_ADDRESS = BL_ADDRESS; //the boot loader will check this address. If (BOOT_MAGIC_ADDRESS == THIS_ADDRESS), then the recovery loader doesn't continue the boot
            set_boot_checksum();
            return(0);
        }
    }

    //Checksum matches, check mem location, if the magic word is set we check if the boot location matches the code location
    if(BOOT_MAGIC_CODE == BOOT_MAGIC_WORD)
    {
        if(BOOT_MAGIC_ADDRESS == DFU_ADDRESS)
        {
            address = DFU_ADDRESS;
            clear_boot_magic();
        }
        else if ( (BOOT_MAGIC_ADDRESS == 0) || (BOOT_MAGIC_ADDRESS == THIS_ADDRESS) )
        {
            //we're in the right spot, return 0 to continue boot, boot magic is to be cleared in the app
            //0 is a norma boot to app, which is handled by the bootloader. BOOT_MAGIC_ADDRESS == THIS_ADDRESS means we're in the right spot
            return(0);
        }
        else
        {
            //wrong spot, let's reboot to new address, boot magic will clear in app
            address = BOOT_MAGIC_ADDRESS;
        }
    }
    else
    {
        //checksum is right, but boot magic word is not set, clear boot magic and continue to boot
        clear_boot_magic();
        return(0);
    }

    //we got here so we need to jump
    jumpAddress = *(__IO uint32_t*)(address + 4);
    jumpToApplication = (pFunction)jumpAddress;

    // Initialize user application's Stack Pointer
    __set_MSP(*(__IO uint32_t*)address); //CMSIS defined function
    jumpToApplication();

    return(0);
}