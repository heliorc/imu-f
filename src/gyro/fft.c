#include "includes.h"
#include "fft.h"
#include "biquad.h"
#include "filter.h"
#include "imu.h"

volatile float fftGyroData[FFT_BUFFS][AXIS_AMOUNT][FFT_DATA_COLLECT_SIZE];
volatile float fftGyroDataPtr;
volatile float fftGyroDataInUse;

typedef struct fft_data {
    float max;
    uint16_t cen;
} fft_data_t;

static const uint8_t fftBinCount = ((FFT_SIZE / 2 - 1) * FFT_MAX_HZ) / (FFT_MAX_HZ) + 1;
static const float fftResolution = 1000 / FFT_SIZE;
static float gyroData[3][FFT_SIZE];
static arm_rfft_fast_instance_f32 fftInstance;
static float fftData[FFT_SIZE];
static float rfftData[FFT_SIZE];
static fft_data_t fftResult[3];
static uint16_t fftIdx = 0;

static axisData_t fftBuffer = {0,0,0};
static int fftBufferCount = 0;
static int fftSamplingScale = 1;

static biquad_axis_state_t fftGyroFilter[3];
static biquad_axis_state_t fftFreqFilter[3];

static float hwin[FFT_SIZE];





/// 32 khz, every 8 is ACC, so do update after every ACC





//init fft anytime the filters are init. this happens in filter.c
void init_fft(void)
{
    //zero the data buffer, volatile struct must be done in for loops
    int x,y,z;
    for(x=0;x<FFT_BUFFS;x++)
    {
        for(y=0;y<AXIS_AMOUNT;y++)
        {
            for(z=0;z<FFT_DATA_COLLECT_SIZE;z++)
            {
                fftGyroData[x][y][z] = 0.0f;
            }
        }
    }
    fftGyroDataPtr = 0;
    fftGyroDataInUse = 0;

    //first init of notch biquads
    biquad_init(240, &(dynNotchStateRate[0].x), REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
    biquad_init(240, &(dynNotchStateRate[0].y), REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
    biquad_init(240, &(dynNotchStateRate[0].z), REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
    biquad_init(240, &(dynNotchStateRate[1].x), REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
    biquad_init(240, &(dynNotchStateRate[1].y), REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);
    biquad_init(240, &(dynNotchStateRate[1].z), REFRESH_RATE, FILTER_TYPE_NOTCH, BIQUAD_BANDWIDTH);

    arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);    
    for (int i = 0; i < FFT_SIZE; i++) {
        hwin[i] = (0.5 - 0.5 * arm_cos_f32(2 * M_PI_FLOAT * i / (FFT_SIZE - 1)));
    }
}

void calculate_fft(biquad_state_t *state)
{
    static int axis = 0;

    arm_cfft_instance_f32 * Sint = &(fftInstance.Sint);
    switch (FFT_SIZE / 2) {
        case 16:
            arm_cfft_radix8by2_f32(Sint, fftData);
            break;
        case 32:
            arm_cfft_radix8by4_f32(Sint, fftData);
            break;
        case 64:
            arm_radix8_butterfly_f32(fftData, FFT_SIZE / 2, Sint->pTwiddle, 1);
            break;
    }

    arm_bitreversal_32((uint32_t*) fftData, Sint->bitRevLength, Sint->pBitRevTable);

    stage_rfft_f32(&fftInstance, fftData, rfftData);

    arm_cmplx_mag_f32(rfftData, fftData, fftBinCount);

    float fftSum = 0;
    float fftWeightedSum = 0;

    fftResult[axis].max = 0;
    float squaredData;
    for (int i = 0; i < fftBinCount; i++) {
        squaredData = fftData[i] * fftData[i];
        fftResult[axis].max = MAX(fftResult[axis].max, squaredData);
        fftSum += squaredData;
        fftWeightedSum += squaredData * (i + 1);
    }

    if (fftSum > 0) {
        float fftMeanIndex = (fftWeightedSum / fftSum) - 1;
        float newFreq;
        newFreq = CONSTRAIN(fftMeanIndex * fftResolution, NOTCH_MIN + 10, FFT_MAX_HZ);
        biquad_update(newFreq, &fftFreqFilter[axis]);
        newFreq = CONSTRAIN(newFreq, NOTCH_MIN + 10, FFT_MAX_HZ);
        fftResult[axis].cen = newFreq;
    }

    float cutoffFreq = CONSTRAIN(fftResult[axis].cen - NOTCH_WIDTH, NOTCH_MIN, NOTCH_MAX);
    float notchQ = NOTCH_APROX(fftResult[axis].cen, cutoffFreq);
    biquad_axis_state_t* axisState;
    switch (axis) {
        case 1:
            axisState = &(state->y);
            break;
        case 2:
            axisState = &(state->z);
            break;
        default:
            axisState = &(state->x);
            break;
    }
    biquad_init(cutoffFreq, axisState, REFRESH_RATE, FILTER_TYPE_NOTCH, notchQ);

    axis++;
    if(axis>2)
    {
        axis=0;
    }

    uint8_t ringBufIdx = FFT_SIZE - fftIdx;
    arm_mult_f32(&gyroData[axis][fftIdx], &hwin[0], &fftData[0], ringBufIdx);
    if (fftIdx > 0)
    {
        arm_mult_f32(&gyroData[axis][0], &hwin[ringBufIdx], &fftData[ringBufIdx], fftIdx);
    }

}

//runs after quaternions have their chace at running
void update_fft(void)
{
    if(fftGyroDataPtr >= 23) //calc yaw
    {
        //calculate_fft(biquad_state_t *state)
    }
    else if(fftGyroDataPtr >= 15) //calc pitch
    {

    }
    else if(fftGyroDataPtr >= 7) //calc roll
    {

    }

    fftBuffer.x += axisData->x;
    fftBuffer.y += axisData->y;
    fftBuffer.z += axisData->z;

    fftBufferCount++;
    if (fftBufferCount == fftSamplingScale) {
        fftBufferCount = 0;
        float sample = fftBuffer.x / fftSamplingScale;
        sample = biquad_update(sample, &fftGyroFilter[0]);  
        gyroData[0][fftIdx] = sample;
        sample = biquad_update(sample, &fftGyroFilter[1]);  
        gyroData[1][fftIdx] = sample;
        sample = biquad_update(sample, &fftGyroFilter[2]);  
        gyroData[2][fftIdx] = sample;
        fftIdx++;
        if(fftIdx >= FFT_SIZE)
        {
            FFT_SIZE = 0;
        }
    }
    calculate_fft(state);
}
