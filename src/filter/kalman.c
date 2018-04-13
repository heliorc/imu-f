#include "includes.h"
#include "gyro.h"
#include "kalman.h"

variance_t varStruct;
kalman_t kalmanFilterStateRate[3];

volatile filter_config_t filterConfig;

void init_kalman(kalman_t *filter, float q, float r, float p, float intialValue)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;      //add multiplier to make tuning easier
    filter->r = r;               //add multiplier to make tuning easier
    filter->p = p;               //add multiplier to make tuning easier
    filter->x = intialValue;     //set intial value, can be zero if unknown
    filter->lastX = intialValue; //set intial value, can be zero if unknown
}

void kalman_init(void)
{
    memset(&varStruct, 0, sizeof(varStruct));

    init_kalman(&kalmanFilterStateRate[ROLL], filterConfig.pitch_q, 88.0f, filterConfig.pitch_q, 0.0f);
    init_kalman(&kalmanFilterStateRate[PITCH], filterConfig.roll_q, 88.0f, filterConfig.roll_q, 0.0f);
    init_kalman(&kalmanFilterStateRate[YAW], filterConfig.yaw_q, 88.0f, filterConfig.yaw_q, 0.0f);
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
     varStruct.xySumVar =  varStruct.xySumVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumVar =  varStruct.xzSumVar + ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumVar =  varStruct.yzSumVar + ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.windex++;
    if ( varStruct.windex >= WINDOW_SIZE)
    {
         varStruct.windex = 0;
    }
     varStruct.xSumMean -=  varStruct.xWindow[ varStruct.windex];
     varStruct.ySumMean -=  varStruct.yWindow[ varStruct.windex];
     varStruct.zSumMean -=  varStruct.zWindow[ varStruct.windex];
     varStruct.xSumVar =  varStruct.xSumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.xWindow[ varStruct.windex]);
     varStruct.ySumVar =  varStruct.ySumVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.zSumVar =  varStruct.zSumVar - ( varStruct.zWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.xySumVar =  varStruct.xySumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.yWindow[ varStruct.windex]);
     varStruct.xzSumVar =  varStruct.xzSumVar - ( varStruct.xWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);
     varStruct.yzSumVar =  varStruct.yzSumVar - ( varStruct.yWindow[ varStruct.windex] *  varStruct.zWindow[ varStruct.windex]);

     varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
     varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
     varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;

     varStruct.xVar =  varStruct.xSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.xMean);
     varStruct.yVar =  varStruct.ySumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.yMean);
     varStruct.zVar =  varStruct.zSumVar *  varStruct.inverseN - ( varStruct.zMean *  varStruct.zMean);
     varStruct.xyCoVar =  varStruct.xySumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.yMean);
     varStruct.xzCoVar =  varStruct.xzSumVar *  varStruct.inverseN - ( varStruct.xMean *  varStruct.zMean);
     varStruct.yzCoVar =  varStruct.yzSumVar *  varStruct.inverseN - ( varStruct.yMean *  varStruct.zMean);

    kalmanFilterStateRate[0].r = ( varStruct.xVar +  varStruct.xyCoVar +  varStruct.xzCoVar) * VARIANCE_SCALE;
    kalmanFilterStateRate[1].r = ( varStruct.yVar +  varStruct.xyCoVar +  varStruct.yzCoVar) * VARIANCE_SCALE;
    kalmanFilterStateRate[2].r = ( varStruct.zVar +  varStruct.yzCoVar +  varStruct.xzCoVar) * VARIANCE_SCALE; 
}

void kalman_update(volatile axisData_t* input, filteredData_t* output)
{
    update_kalman_covariance(input);
    float axisValues[3];
    axisValues[0] = input->x;
    axisValues[1] = input->y;
    axisValues[2] = input->z;
    for (int axis = 2; 0 <= axis; axis--){
        //project the state ahead using acceleration
        kalmanFilterStateRate[axis].x += (kalmanFilterStateRate[axis].x - kalmanFilterStateRate[axis].lastX);

        //update last state
        kalmanFilterStateRate[axis].lastX = kalmanFilterStateRate[axis].x;

        //prediction update
        kalmanFilterStateRate[axis].p = kalmanFilterStateRate[axis].p + kalmanFilterStateRate[axis].q;

        //measurement update
        kalmanFilterStateRate[axis].k = kalmanFilterStateRate[axis].p / (kalmanFilterStateRate[axis].p + kalmanFilterStateRate[axis].r);
        kalmanFilterStateRate[axis].x += kalmanFilterStateRate[axis].k * (axisValues[axis] - kalmanFilterStateRate[axis].x);
        kalmanFilterStateRate[axis].p = (1.0f - kalmanFilterStateRate[axis].k) * kalmanFilterStateRate[axis].p;
        axisValues[axis] = kalmanFilterStateRate[axis].x;
    }
    output->rateData.x = axisValues[0];
    output->rateData.y = axisValues[1];
    output->rateData.z = axisValues[2];
}
#pragma GCC pop_options