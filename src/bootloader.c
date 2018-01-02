#include "includes.h"
#include "shifty_kalman.h"
#include "fix16.h"


static void bootloader_wait(void);
static void bootloader_main(void);

shifty_kalman_t shiftyKalman[3];

fastKalmanF_t floatKalman[3];


static void bootloader_main(void)
{
    //bootloader code
}

void bootloader_start(void)
{
    //PB5, LED on test board
    init_gpio(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);
    //init_gpio_input(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN, 1);

    inline_digital_hi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_lo(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_hi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_lo(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_hi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_lo(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_hi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);
    inline_digital_lo(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    HAL_Delay(500);


    shifty_kalman_init(&shiftyKalman[0], 1000, 1000, 1000);
    shifty_kalman_init(&shiftyKalman[1], 1000, 1000, 1000);
    shifty_kalman_init(&shiftyKalman[2], 1000, 1000, 1000);

    fastKalmanInitF(&floatKalman[0], 1, 1, 1, 0);
    fastKalmanInitF(&floatKalman[1], 1, 1, 1, 0);
    fastKalmanInitF(&floatKalman[2], 1, 1, 1, 0);

    int16_t fakeGyroData = 0;

    while (1)
    {
        HAL_Delay(2);
        inline_digital_hi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );
        
        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );

        shifty_kalman_update(&shiftyKalman[0], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[1], fix16_from_int(fakeGyroData++) );
        shifty_kalman_update(&shiftyKalman[2], fix16_from_int(fakeGyroData++) );


        inline_digital_lo(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);

        inline_digital_hi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);


        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        fastKalmanUpdateF(&floatKalman[0], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[1], (fakeGyroData++));
        fastKalmanUpdateF(&floatKalman[2], (fakeGyroData++));

        inline_digital_lo(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN);
    
    }

    bootloader_wait();
    if (InlineIsPinStatusHi(BOOTLOADER_CHECK_PORT, BOOTLOADER_CHECK_PIN) == 1)
    {
        bootloader_main();
    }
    else 
    {
        BootToAddress(APP_ADDRESS);
    }
}

static void bootloader_wait(void)
{
    HAL_Delay(500);
}