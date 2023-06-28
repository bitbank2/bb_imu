//
// BitBank Inertial Measurement Unit (IMU) Library
// written by Larry Bank
//
// Copyright 2023 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================

#include <Arduino.h>
#include <BitBang_I2C.h>

#ifndef __BB_IMU__
#define __BB_IMU__

// Capabilities
#define IMU_CAP_ACCELEROMETER 1
#define IMU_CAP_GYROSCOPE 2
#define IMU_CAP_MAGNETOMETER 4
#define IMU_CAP_FIFO 8
#define IMU_CAP_TEMPERATURE 16
#define IMU_CAP_PEDOMETER 32

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
   TYPE_COUNT
};

#define MODE_ACCEL 1
#define MODE_GYRO  2
#define MODE_TEMP  4
#define MODE_FIFO  8
#define MODE_STEP  16

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

class BBIMU
{
public:
    BBIMU() {_iType = IMU_TYPE_UNDEFINED; _iAccRate = iGyroRate = 200; }
    ~BBIMU() {}

    int init(int iSDA = -1, int iSCL = -1, bool bBitBang = false, uint32_t u32Speed=400000);
    int start(int iSampleRate = 200, int iMode = MODE_ACCEL | MODE_GYRO);
    int stop(void);
    void setAccRate(int iRate);
    void setGyroRate(int iRate);
    int getAccRate(void);
    int getGyroRate(void);
    uint32_t caps(void);
    int type(void);
    BBI2C *getBB(void);
    int getSample(IMU_SAMPLE *pSample);
 
private:
    BBI2C _bbi2c;
    int _iAddr;
    int _iType;
    int _iMode;
    int _iAccStart, _iGyroStart, _iTempStart; // starting registers
    int _iAccRate, _iGyroRate; // sample rates
    int _iStepStart;
    int _iTempLen; // length of temp info in bytes
    bool _bBigEndian;
    uint32_t _u32Caps;
    int16_t get16Bits(uint8_t *s);
    int matchRate(int value, int *pList);
}; // class BBIMU
#endif // __BB_IMU__
