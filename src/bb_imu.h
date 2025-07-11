// bb_imu.h
// A universal IMU (Inertial Measurement Unit) libraray
// Written by Larry Bank
//
// Copyright (c) 2023 - 2025 BitBank Software, Inc.
// All rights reserved.
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <Arduino.h>
#include <BitBang_I2C.h>

#ifndef __BB_IMU__
#define __BB_IMU__

// Capabilities
#define IMU_CAP_ACCELEROMETER 1
#define IMU_CAP_GYROSCOPE 2
#define IMU_CAP_MAGNETOMETER 4
#define IMU_CAP_3DPOS 8
#define IMU_CAP_FIFO 16
#define IMU_CAP_TEMPERATURE 32 
#define IMU_CAP_PEDOMETER 64

typedef struct _tagsample
{
   int16_t accel[3];
   int16_t gyro[3];
   int temperature;
   int steps;
} IMU_SAMPLE;

//
// Currently supported devices
//
enum {
   IMU_TYPE_UNDEFINED=0,
   IMU_TYPE_ADXL345,
   IMU_TYPE_MPU6050,
   IMU_TYPE_LSM9DS1,
   IMU_TYPE_LSM6DS3,
   IMU_TYPE_BMI160,
   IMU_TYPE_LIS3DH,
   IMU_TYPE_LIS3DSH,
   IMU_TYPE_MPU6886,
   IMU_TYPE_BNO055,
   IMU_TYPE_BMI270,
   IMU_TYPE_QMI8658,
   IMU_TYPE_MPU6500,
   TYPE_COUNT
};

// Accelerometer scale
enum {
   ACCEL_SCALE_2G=0,
   ACCEL_SCALE_4G,
   ACCEL_SCALE_8G,
   ACCEL_SCALE_16G
};

#define MODE_ACCEL 1
#define MODE_GYRO  2
#define MODE_TEMP  4
#define MODE_FIFO  8
#define MODE_3DPOS 16
#define MODE_STEP  32

#define IMU_SUCCESS 0
#define IMU_ERROR -1

#define IMU_LSM9DS1_ADDR 0x6a
#define IMU_ADXL345_ADDR 0x53
#define IMU_BMI160_ADDR 0x68
#define IMU_MPU6050_ADDR 0x68
#define IMU_MPU6886_ADDR 0x68
#define IMU_LSM6DS3_ADDR 0x6a
#define IMU_LIS3DH_ADDR 0x18
#define IMU_LIS3DSH_ADDR 0x1c
#define IMU_BNO055_ADDR 0x28
#define IMU_BMI270_ADDR 0x68
#define IMU_QMI8658_ADDR 0x6a

// bits to enable specific channels
#define IMU_CHANNEL_ACC_X 1
#define IMU_CHANNEL_ACC_Y 2
#define IMU_CHANNEL_ACC_Z 4
#define IMU_CHANNEL_GYR_X 8
#define IMU_CHANNEL_GYR_Y 16
#define IMU_CHANNEL_GYR_Z 32
#define IMU_CHANNEL_MAG_X 64
#define IMU_CHANNEL_MAG_Y 128
#define IMU_CHANNEL_MAG_Z 256

class BBIMU
{
public:
    BBIMU() {_iType = IMU_TYPE_UNDEFINED; _iAccRate = _iGyroRate = 200; }
    ~BBIMU() {}

    int init(int iSDA = -1, int iSCL = -1, bool bBitBang = false, uint32_t u32Speed=400000);
    int start(int iSampleRate = 200, int iMode = MODE_ACCEL | MODE_GYRO);
    int stop(void);
    int reset(void);
    int configFIFO(void);
    int configIRQ(bool bOn);
    int getQueuedSamples(int16_t *pSamples, int *iNumSamples, int iMaxSamples);
    void setAccScale(int iScale);
    void setGyroScale(int iScale);
    void setAccRate(int iRate);
    void setGyroRate(int iRate);
    int getAccScale(void);
    int getGyroScale(void);
    int getAccRate(void);
    int getGyroRate(void);
    int16_t getOneChannel(uint32_t u32Channel);
    uint8_t getStatus(void);
    uint32_t caps(void);
    int type(void);
    BBI2C *getBB(void);
    int getSample(IMU_SAMPLE *pSample);
 
private:
    BBI2C _bbi2c;
    int _iAddr;
    int _iType;
    int _iMode;
    int _iStatus, _iMagStart, _iAccStart, _iGyroStart, _iTempStart; // starting registers
    int _iAccRate, _iGyroRate; // sample rates
    int _iAccScale, _iGyroScale; // gravity scale
    int _iStepStart;
    int _iTempLen; // length of temp info in bytes
    bool _bBigEndian;
    uint32_t _u32Caps;
    int16_t get16Bits(uint8_t *s);
    int matchRate(int value, int16_t *pList);
}; // class BBIMU
#endif // __BB_IMU__
