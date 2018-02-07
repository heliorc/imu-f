#pragma once
#include "includes.h"
#include "gyro.h"
#include "quaternions.h"
#include "filter.h"
extern void imuf_write_data(uint32_t* data);
extern void imuf_write_quaternion_data(quaternion_record_t* data);