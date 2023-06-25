//
// BitBank Inertial Measurement Unit (IMU) Library
// Written by Larry Bank
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
#include "bb_imu.h"
//
// Initialize the library
// It only needs to initialize the I2C interface; the chip is ready
//
int BBIMU::init(int iSDA, int iSCL, bool bBitBang, uint32_t u32Speed)
{
uint8_t ucTemp[4];

    _bbi2c.iSDA = iSDA;
    _bbi2c.iSCL = iSCL;
    _bbi2c.bWire = !bBitBang;
    I2CInit(&_bbi2c, u32Speed);

    // probe the I2C bus for devices
    if (I2CTest(IMU_LSM9DS1_ADDR)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LSM9DS1_ADDR, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x68) {
           _iType = IMU_TYPE_LSM9DS1;
           _iAddr = IMU_LSM9DS1_ADDR;
           _bBigEndian = false;
           _iAccStart = 0x18;
           _iGyroStart = 0x28;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(IMU_LSM6DS3_ADDR)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LSM6DS3_ADDR, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x69) {
           _iType = IMU_TYPE_LSM6DS3;
           _iAddr = IMU_LSM6DS3_ADDR;
           _bBigEndian = false;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(IMU_LIS3DH_ADDR)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LIS3DH_ADDR, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x33) {
           _iType = IMU_TYPE_LIS3DH;
           _iAddr = IMU_LIS3DH_ADDR;
           _bBigEndian = false;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(IMU_ADXL345_ADDR)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_ADXL345_ADDR, 0x0, ucTemp, 1); // get ID
       if (ucTemp[0] == 0xe5) {
          _iType = IMU_TYPE_ADXL345;
          _iAddr = IMU_ADXL345_ADDR;
          _bBigEndian = false;
          _iAccStart = 0x32;
          return IMU_SUCCESS;
       }
    }
    if (I2CTest(IMU_BMI160_ADDR) || I2CTest(IMU_BMI160_ADDR+1)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_BMI160_ADDR, 0x0, ucTemp, 1); // get ID
       if (ucTemp[0] == 0xd1) {
          if (I2CTest(IMU_BMI160_ADDR)
             _iAddr = IMU_BMI160_ADDR;
          else
             _iAddr = IMU_BMI160_ADDR+1;
          _iType = IMU_TYPE_BMI160;
          _bBigEndian = false;
          _iAccStart = 0xc;
          _iGyroStart = 0x12;
          return IMU_SUCCESS;
       }
    }
    if (I2CTest(IMU_MPU6050_ADDR)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_MPU6050_ADDR, 0x75, ucTemp, 1); // get ID
       if (ucTemp[0] == 0x68) {
          _iType = IMU_TYPE_MPU6050;
          _iAddr = IMU_MPU6050_ADDR;
          _bBigEndian = true;
          _iAccStart = 0x43;
          _iGyroStart = 0x3b;
          return IMU_SUCCESS;
       }
    }
    if (I2CTest(IMU_MPU6886_ADDR)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_MPU6886_ADDR, 0x75, ucTemp, 1); // get ID
       if (ucTemp[0] == 0x19) {
          _iType = IMU_TYPE_MPU6886;
          _iAddr = IMU_MPU6886_ADDR;
          _bBigEndian = true;
          _iAccStart = 0x3b;
          _iGyroStart = 0x43
          return IMU_SUCCESS;
       }
    }
    return IMU_ERROR;
} /* init() */
int BBIMU::start(int iSampleRate, int iMode)
{
uint8_t ucTemp[4];

   _iMode = iMode;
   switch (_iType) {
      case IMU_TYPE_MPU6050:
// pwr mgmt 1 register
// bits: 7=reset, 6=sleep, 5=cycle, 4=n/a, 3=temp_disable, 2-0=clock select
         ucTemp[0] = 0x6b; // power management 1 register
         ucTemp[1] = 0x00; // disable sleep mode
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break; // MPU6050
      case IMU_TYPE_ADXL345:
         ucTemp[0] = 0x2c; // bandwidth/rate mode
         ucTemp[1] = 0x06; // 6.125hz sampling (lowest power)
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         ucTemp[0] = 0x2d; // power control
         ucTemp[1] = 0x08; // set simplest sampling mode (only measure bit)
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         ucTemp[0] = 0x31; // data format
         ucTemp[1] = 0x00; // set +/-2g range and right justified mode
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break; // ADXL345
      case IMU_TYPE_MPU6886:
            ucTemp[0] = 0x6b; // PWR_MGMT_1
            ucTemp[1] = 0x80; // reset chip
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(10);
            ucTemp[1] = 1; // select the best available oscillator
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(10);
            ucTemp[0] = 0x1c; // ACCEL_CONFIG
            ucTemp[1] = 0x00; // full scale = 2G, all axes enabled
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x1b; // GYRO_CONFIG
            ucTemp[1] = 0x18; // +/- 2000 degrees per second
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x1a; // CONFIG
            ucTemp[1] = 1; // 176 filtered samples per sec (1k sampling rate)
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x19; // SMPLRT_DIV
            ucTemp[1] = 0x05; // sample rate divider (1000 / (1+this_val))
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x38; // INT_ENABLE
            ucTemp[1] = 0x00; // disable interrupts
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x1d; // ACCEL_CONFIG2
            ucTemp[1] = 0x00; // avg 4 samples
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x6a; // USER_CTRL
            ucTemp[1] = 0x00; // disable FIFO
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x23; // FIFO_EN
            ucTemp[1] = 0x00; // disable
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x37; // INT_PIN_CFG
            ucTemp[1] = 0x22; // latch int enable
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(1);
            ucTemp[0] = 0x38; // INT_ENABLE
            ucTemp[1] = 0x01; // enable interrupt on data ready
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break; // MPU6886
      case IMU_TYPE_BMI160:
         ucTemp[0] = 0x7e; // send command
         ucTemp[1] = 0x11; // set accelerometer to normal mode
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         delay(4); // give it 4ms to occur
         ucTemp[0] = 0x7e; // command
         ucTemp[1] = 0x15; // set gyroscope to normal power mode
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break; // BMI160
   } // switch
} /* start() */

int16_t BBIMU::get16Bits(uint8_t *s)
{
int16_t i;
   if (_bBigEndian) {
       i = (int16_t)s[0] << 8;
       i |= s[1];
   } else {
       i = (int16_t)s[1] << 8;
       i |= s[0];
   }
   return i;
} /* get16Bits() */

int BBIMU::getSample(IMU_SAMPLE *pSample)
{
uint8_t ucTemp[16];
int i;

     if (_iMode & MODE_ACCEL) { // read accelerometer info
        I2CReadRegister(&_bbi2c, _iAddr, _iAccStart, ucTemp, 6);
        for (i=0; i<3; i++) { 
           pSample->accel[i] = get16Bits(&ucTemp[i*2]);
        }
     }
     if (_iMode & MODE_GYRO) { // read gyroscope info
        I2CReadRegister(&_bbi2c, _iAddr, _iGyroStart, ucTemp, 6);
        for (i=0; i<3; i++) {
           pSample->gyro[i] = get16Bits(&ucTemp[i*2]);
        }
     }
} /* getSample() */

int BBIMU::stop(void)
{
} /* stop() */

uint32_t BBIMU::caps(void)
{
  return _u32Caps;
} /* caps() */

int BBIMU::type(void)
{
  return _iType;
} /* type() */

