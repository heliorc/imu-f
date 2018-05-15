#include "includes.h"
#include "gyro.h"
#include "kalman.h"
#include "filter.h"

variance_t varStruct;
kalman_t kalmanFilterStateRate[3];


void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;      //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
}

void kalman_init(void)
{
    memset(&varStruct, 0, sizeof(varStruct));
    init_kalman(&kalmanFilterStateRate[ROLL], filterConfig.roll_q);
    init_kalman(&kalmanFilterStateRate[PITCH], filterConfig.pitch_q);
    init_kalman(&kalmanFilterStateRate[YAW], filterConfig.yaw_q);
    varStruct.inverseN = 1.0f/filterConfig.w;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
static void update_kalman_variance(volatile axisData_t *gyroRateData)
{
     varStruct.xWindow[ varStruct.windex] = gyroRateData->x;
     varStruct.yWindow[ varStruct.windex] = gyroRateData->y;
     varStruct.zWindow[ varStruct.windex] = gyroRateData->z;

     varStruct.xSumMean +=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean +=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean +=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar + ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.windex++;
    if ( varStruct.windex >= filterConfig.w)
    {
         varStruct.windex = 0;
    }
     varStruct.xSumMean -=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean -=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean -=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar - ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);

     varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
     varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
     varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;

     varStruct.xVar =  varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean);
     varStruct.yVar =  varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean);
     varStruct.zVar =  varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean);

    kalmanFilterStateRate[ROLL].r = varStruct.xVar * VARIANCE_SCALE;
    kalmanFilterStateRate[PITCH].r = varStruct.yVar * VARIANCE_SCALE;
    kalmanFilterStateRate[YAW].r = varStruct.zVar * VARIANCE_SCALE; 
}

inline float kalman_process(kalman_t* kalmanState, volatile float input) {
    //project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX);

    //update last state
    kalmanState->lastX = kalmanState->x;

    //prediction update
    kalmanState->p = kalmanState->p + kalmanState->q;

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
    return kalmanState->x;
}

void kalman_update(volatile axisData_t* input, filteredData_t* output)
{
    update_kalman_variance(input);
    output->rateData.x = kalman_process(&kalmanFilterStateRate[ROLL], input->x);
    output->rateData.y = kalman_process(&kalmanFilterStateRate[PITCH], input->y);
    output->rateData.z = kalman_process(&kalmanFilterStateRate[YAW], input->z);
}
#pragma GCC pop_options
