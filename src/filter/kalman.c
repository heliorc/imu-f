#include "includes.h"
#include "gyro.h"
#include "kalman.h"
#include "filter.h"

variance_t varStruct;
kalman_t   kalmanFilterStateRate[3];
float      r_filter_weight = 1.0f;


void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.0001f;     //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
    filter->e = 1.0f;
}

void kalman_init(void)
{
    setPointNew = 0;
    memset(&varStruct, 0, sizeof(varStruct));
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
     varStruct.xySumCoVar =  varStruct.xySumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
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
     varStruct.xySumCoVar =  varStruct.xySumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumCoVar =  varStruct.xzSumCoVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumCoVar =  varStruct.yzSumCoVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);

     varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
     varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
     varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;

     varStruct.xVar =  ABS(varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean));
     varStruct.yVar =  ABS(varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean));
     varStruct.zVar =  ABS(varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean));
     varStruct.xyCoVar =  ABS(varStruct.xySumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.yMean));
     varStruct.xzCoVar =  ABS(varStruct.xzSumCoVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.zMean));
     varStruct.yzCoVar =  ABS(varStruct.yzSumCoVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.zMean));

    float squirt;
    arm_sqrt_f32(varStruct.xVar +  varStruct.xyCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[ROLL].r = squirt * r_filter_weight;
    arm_sqrt_f32(varStruct.yVar +  varStruct.xyCoVar +  varStruct.yzCoVar, &squirt);
    kalmanFilterStateRate[PITCH].r = squirt * r_filter_weight;
    arm_sqrt_f32(varStruct.zVar +  varStruct.yzCoVar +  varStruct.xzCoVar, &squirt);
    kalmanFilterStateRate[YAW].r = squirt * r_filter_weight;
}

inline float kalman_process(kalman_t* kalmanState, volatile float input, volatile float target)
{
	//project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX);
    
    //figure out how much to boost or reduce our error in the estimate based on setpoint target.
    //this should be close to 0 as we approach the setpoint and really high the further away we are from the setpoint.
    //update last state
    kalmanState->lastX = kalmanState->x;

    if (target != 0.0f && input  != 0.0f)
    {
        kalmanState->e = ABS(1.0f - target/input);
    }
    else
    {
        kalmanState->e = 1.0f;
    }
    
    //prediction update
    kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
    return kalmanState->x;
}

void kalman_update(volatile axisData_t* input, filteredData_t* output)
{
    update_kalman_covariance(input);
    output->rateData.x = kalman_process(&kalmanFilterStateRate[ROLL], input->x, setPoint.x);
    output->rateData.y = kalman_process(&kalmanFilterStateRate[PITCH], input->y, setPoint.y);
    output->rateData.z = kalman_process(&kalmanFilterStateRate[YAW], input->z, setPoint.z);
}
#pragma GCC pop_options
