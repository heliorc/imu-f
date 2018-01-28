#include "quaternions.h"
#include "includes.h"
#include "gyro.h"

void generate_quaterions(axisData_t* gyroRateData,axisData_t* gyroAccData,filteredData_t* filteredData)
{
    filteredData->quaternion[0] = 1.0f;
    filteredData->quaternion[1] = 0.0f;
    filteredData->quaternion[2] = 0.0f;
    filteredData->quaternion[3] = 0.0f;
}