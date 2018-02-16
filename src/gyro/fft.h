#pragma once

#include "filter.h"
#include "arm_math.h"

#define FFT_SIZE              32
#define FFT_MIN_HZ            100 
#define FFT_MAX_HZ            500 
#define NOTCH_WIDTH           100 
#define NOTCH_MIN             120 
#define NOTCH_MAX             200 
#define AXIS_AMOUNT           3
#define FFT_BUFFS             2
#define FFT_DATA_COLLECT_SIZE 42 //allow overflow room since FFT calcs aren't realtime
#define NOTCH_APROX(cen, max) ((float)(max * cen) / ((float)(cen - max) * (float)(cen + max)))

extern void arm_bitreversal_32(uint32_t * pSrc, const uint16_t bitRevLen, const uint16_t * pBitRevTable);
extern void stage_rfft_f32(arm_rfft_fast_instance_f32 * S, float32_t * p, float32_t * pOut);
extern void arm_cfft_radix8by2_f32( arm_cfft_instance_f32 * S, float32_t * p1);
extern void arm_cfft_radix8by4_f32( arm_cfft_instance_f32 * S, float32_t * p1);
extern void arm_radix8_butterfly_f32(float32_t * pSrc, uint16_t fftLen, const float32_t * pCoef, uint16_t twidCoefModifier);
extern void arm_cmplx_mag_f32(float32_t * pSrc, float32_t * pDst, uint32_t numSamples);

void init_fft(void);
void update_fft(void);