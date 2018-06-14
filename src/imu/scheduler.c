#include "includes.h"
#include "scheduler.h"
#include "board_comm.h"
#include "quaternions.h"
#include "gyro.h"
#include "filter.h"
#include "drm.h"
#include "gyro_device.h" //where gyroRxFrame lives

inline void scheduler_run(void)
{
    while(!gyroDataReadDone); //gyro read imminent, sit and spin
    //once the gyroReadDone is set to 1, we have 31.25 us to do whatever we need to
    gyro_int_to_float(&gyroRxFrame); //collect data to float
    run_gyro_filters();
    if (bcRx.command == BC_IMUF_CALIBRATE)
    {
        //check CRC!
        //if (parse_imuf_command(&bcRx))
        //{
            start_calibration();
        //}
        bcRx.command = 0;
    }    
    //0 is 32 KHz which disabled quaternions
    increment_acc_tracker();
    update_quaternions();
    gyroDataReadDone = 0; //reset read flag to prepare for next read
    fire_spi_send_ready();
    if (!check_me())
    {
        delay_ms(5);
    }
}