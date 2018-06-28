#include "includes.h"
#include "gyro.h"
#include "kalman.h"
#include "filter.h"

variance_t varStruct;
volatile uint32_t setPointNew;
volatile axisDataInt_t setPointInt;
volatile axisData_t setPoint;
kalman_t kalmanFilterStateRate[3];

#define DT 1 / 32000

void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    setPointNew = 0;
    filter->q = q * 0.001f;      //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
    filter->x = 0.0f;         //set intial value, can be zero if unknown
    filter->lastX = 0.0f;     //set intial value, can be zero if unknown
    filter->e = 1.0f;
    filter->acc = 0.0f;       //set intial value, can be zero if unknown
}

void kalman_init(void)
{
    setPointNew = 0;
    memset(&varStruct, 0, sizeof(varStruct));
    memset((uint32_t *)&setPoint, 0, sizeof(axisData_t));
    memset((uint32_t *)&setPointInt, 0, sizeof(axisDataInt_t));
    init_kalman(&kalmanFilterStateRate[ROLL], filterConfig.roll_q);
    init_kalman(&kalmanFilterStateRate[PITCH], filterConfig.pitch_q);
    init_kalman(&kalmanFilterStateRate[YAW], filterConfig.yaw_q);
    varStruct.inverseN = 1.0f/filterConfig.w;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
void update_kalman_covariance(volatile axisData_t *gyroRateData)
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

     varStruct.xVar =  ABS(varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean));
     varStruct.yVar =  ABS(varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean));
     varStruct.zVar =  ABS(varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean));

    float squirt;
    arm_sqrt_f32(varStruct.xVar, &squirt);
    if (setPoint.y != 0.0f && setPoint.z != 0.0f && kalmanFilterStateRate[PITCH].lastX != 0.0f && kalmanFilterStateRate[YAW].lastX != 0.0f){ 
        squirt = squirt * ABS(((setPoint.y/kalmanFilterStateRate[PITCH].lastX) + (setPoint.z/kalmanFilterStateRate[YAW].lastX)) * VARIANCE_SCALE);
    }
    kalmanFilterStateRate[ROLL].r = squirt;
    arm_sqrt_f32(varStruct.yVar, &squirt);
    if (setPoint.x != 0.0f && setPoint.z != 0.0f && kalmanFilterStateRate[ROLL].lastX != 0.0f && kalmanFilterStateRate[YAW].lastX != 0.0f){ 
        squirt = squirt * ABS(((setPoint.x/kalmanFilterStateRate[ROLL].lastX) + (setPoint.z/kalmanFilterStateRate[YAW].lastX)) * VARIANCE_SCALE);
    }
    kalmanFilterStateRate[PITCH].r = squirt;
    arm_sqrt_f32(varStruct.zVar, &squirt);
    if (setPoint.x != 0.0f && setPoint.y != 0.0f && kalmanFilterStateRate[ROLL].lastX != 0.0f && kalmanFilterStateRate[PITCH].lastX != 0.0f){ 
        squirt = squirt * ABS(((setPoint.x/kalmanFilterStateRate[ROLL].lastX) + (setPoint.y/kalmanFilterStateRate[PITCH].lastX)) * VARIANCE_SCALE);
    }
    kalmanFilterStateRate[YAW].r = squirt;
}

inline float kalman_process(kalman_t* kalmanState, volatile float input, volatile float target) {
    //project the state ahead using acceleration
    kalmanState->x = kalmanState->lastX + (kalmanState->acc * DT);

    //update last state
    kalmanState->lastX = kalmanState->x;

    if (target != 0.0f) {
        kalmanState->e = ABS(1.0f - (target/kalmanState->lastX));
    } else {
        kalmanState->e = 1.0f;
    }
    
    //prediction update
    kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
    
    kalmanState->acc = (kalmanState->x - kalmanState->lastX) * DT;
    kalmanState->lastX = kalmanState->x;

    return kalmanState->x;
}

void kalman_update(volatile axisData_t* input, filteredData_t* output)
{
    if (setPointNew) 
    {
        setPointNew = 0;
        memcpy((uint32_t *)&setPoint, (uint32_t *)&setPointInt, sizeof(axisData_t));
    }
    update_kalman_covariance(input);
    output->rateData.x = kalman_process(&kalmanFilterStateRate[ROLL], input->x, setPoint.x);
    output->rateData.y = kalman_process(&kalmanFilterStateRate[PITCH], input->y, setPoint.y);
    output->rateData.z = kalman_process(&kalmanFilterStateRate[YAW], input->z, setPoint.z);
}
#pragma GCC pop_options
