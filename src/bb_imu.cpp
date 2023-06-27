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

BBI2C * BBIMU::getBB(void)
{
   return &_bbi2c;
} /* getBB() */
//
// Initialize the I2C interface and detect the chip type
//
int BBIMU::init(int iSDA, int iSCL, bool bBitBang, uint32_t u32Speed)
{
uint8_t ucTemp[4];
int iOffset;

    _bbi2c.iSDA = iSDA;
    _bbi2c.iSCL = iSCL;
    _bbi2c.bWire = !bBitBang;
    I2CInit(&_bbi2c, u32Speed);

    for (iOffset = 0; iOffset<2; iOffset++) { // try both addresses of each device
    // probe the I2C bus for devices
    if (I2CTest(&_bbi2c, IMU_LSM9DS1_ADDR+iOffset)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LSM9DS1_ADDR + iOffset, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x68) {
           _iType = IMU_TYPE_LSM9DS1;
           _iAddr = IMU_LSM9DS1_ADDR + iOffset;
           _bBigEndian = false;
           _iAccStart = 0x18;
           _iGyroStart = 0x28;
           _iTempStart = 0x15;
           _iTempLen = 2;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_MAGNETOMETER | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_LSM6DS3_ADDR + iOffset)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LSM6DS3_ADDR + iOffset, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x69) {
           _iType = IMU_TYPE_LSM6DS3;
           _iAddr = IMU_LSM6DS3_ADDR + iOffset;
           _bBigEndian = false;
           _iAccStart = 0x28;
           _iGyroStart = 0x22;
           _iTempStart = 0x20;
           _iTempLen = 2;
           _iStepStart = 0x4b;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE | IMU_CAP_PEDOMETER;
           return IMU_SUCCESS;
       }
    }
    
    if (I2CTest(&_bbi2c, IMU_LIS3DH_ADDR+iOffset)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LIS3DH_ADDR+iOffset, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x33) {
           _iType = IMU_TYPE_LIS3DH;
           _iAddr = IMU_LIS3DH_ADDR+iOffset;
           _iTempStart = 0xc;
           _iTempLen = 1;
           _iAccStart = 0x28;
           _bBigEndian = false;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_LIS3DSH_ADDR+iOffset)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LIS3DSH_ADDR+iOffset, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x3F) {
           _iType = IMU_TYPE_LIS3DSH;
           _iAddr = IMU_LIS3DSH_ADDR+iOffset;
           _iTempStart = 0xc;
           _iTempLen = 1;
           _iAccStart = 0x28;
           _bBigEndian = false;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_ADXL345_ADDR+iOffset)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_ADXL345_ADDR+iOffset, 0x0, ucTemp, 1); // get ID
       if (ucTemp[0] == 0xe5) {
           _iType = IMU_TYPE_ADXL345;
           _iAddr = IMU_ADXL345_ADDR+iOffset;
           _bBigEndian = false;
           _iAccStart = 0x32;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_FIFO;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_BMI160_ADDR+iOffset)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_BMI160_ADDR+iOffset, 0x0, ucTemp, 1); // get ID
       if (ucTemp[0] == 0xd1) {
          _iAddr = IMU_BMI160_ADDR+iOffset;
          _iType = IMU_TYPE_BMI160;
          _bBigEndian = false;
          _iAccStart = 0x12;
          _iGyroStart = 0xc;
          _iTempStart = 0x20;
          _iTempLen = 2;
          _iStepStart = 0x78;
          _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE | IMU_CAP_PEDOMETER;
          return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_MPU6050_ADDR+iOffset)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_MPU6050_ADDR+iOffset, 0x75, ucTemp, 1); // get ID
       if (ucTemp[0] == 0x68) {
          _iType = IMU_TYPE_MPU6050;
          _iAddr = IMU_MPU6050_ADDR+iOffset;
          _bBigEndian = true;
          _iAccStart = 0x43;
          _iGyroStart = 0x3b;
          _iTempStart = 0x41;
          _iTempLen = 2;
          _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
          return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_MPU6886_ADDR+iOffset)) {
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_MPU6886_ADDR+iOffset, 0x75, ucTemp, 1); // get ID
       if (ucTemp[0] == 0x19) {
          _iType = IMU_TYPE_MPU6886;
          _iAddr = IMU_MPU6886_ADDR+iOffset;
          _bBigEndian = true;
          _iAccStart = 0x3b;
          _iGyroStart = 0x43;
          return IMU_SUCCESS;
       }
    }
    } // for each address offset
    return IMU_ERROR;
} /* init() */
//
// Start the accelerometer, gyroscope or both
// with the given sample rate
//
int BBIMU::start(int iSampleRate, int iMode)
{
uint8_t ucTemp[4];

   _iMode = iMode;
   switch (_iType) {
      case IMU_TYPE_LSM6DS3:
         // If accelerometer enabled
         if (_iMode & MODE_ACCEL) {
            ucTemp[0] = 0x10; // CTRL1_XL
            ucTemp[1] = (5<<4); // 208hz iODR << 4;
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         } // accelerometer enabled
         // if gyroscope enabled
         if (_iMode & MODE_GYRO) {
            ucTemp[0] = 0x11; // CTRL2_G
            //if (iODR > 8) iODR = 8; // Gyro max rate = 1660hz
            ucTemp[1] = (5<<4); //208Hz iODR << 4; // gyroscope data rate
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         } // gyroscope enable
         if (_iMode & MODE_STEP) {
            ucTemp[0] = 0x19; // CTRL10_C
            ucTemp[1] = (_iMode & MODE_GYRO) ? 0x3e : 0x4; // check 3 axis of gyro are enabled (on by default)
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2); 
            ucTemp[0] = 0x58; // enable step counter in TAP_CFG register
            ucTemp[1] = 0x40;
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         }
         ucTemp[0] = 0x16; // CTR7_G - power mode
         //if (_u32Rate <= 52)
         //ucTemp[1] = 0x80; // Enable low power mode
         // else
         ucTemp[1] = 0x40; // Disable low power mode, enable high pass filter
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break;
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
         if (_iMode & MODE_STEP) {
             ucTemp[0] = 0x7a; // STEP_CONF
             ucTemp[1] = 0x15;
             ucTemp[2] = 0x03; // normal mode + enabled
             I2CWrite(&_bbi2c, _iAddr, ucTemp, 3);
             ucTemp[0] = 0x7b; // enable in separate step?
             ucTemp[1] = 0x0b;
             I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         }
         break; // BMI160
      case IMU_TYPE_LIS3DH:
      case IMU_TYPE_LIS3DSH:
         if (_iMode & MODE_ACCEL) {
            ucTemp[0] = 0x20; // CTRL_REG1
            ucTemp[1] = (6 << 4); // 100Hz iODR << 4;
            // Enable only the requested channels
            ucTemp[1] |= (1 | 2 | 4); // activate all channels
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         } // accelerometer enabled
         ucTemp[0] = 0x23; // CTRL_REG4
         ucTemp[1] = 0x88; // BDU & high res mode enabled
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break; // LIS3DH / LIS3DSH
      default:
         return IMU_ERROR;
   } // switch
   return IMU_SUCCESS;
} /* start() */
//
// Read a 16-bit signed integer value from 2 bytes stored at the given pointer addr
// The member variable _bBigEndian determines the byte order
//
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
//
// Read an accel, gyro, and temp sample depending on the operating mode
//
int BBIMU::getSample(IMU_SAMPLE *pSample)
{
uint8_t ucTemp[16];
int i;

     if (_iMode & MODE_ACCEL && _u32Caps & IMU_CAP_ACCELEROMETER) { // read accelerometer info
        I2CReadRegister(&_bbi2c, _iAddr, _iAccStart, ucTemp, 6);
        for (i=0; i<3; i++) { 
           pSample->accel[i] = get16Bits(&ucTemp[i*2]);
        }
     }
     if (_iMode & MODE_GYRO && _u32Caps & IMU_CAP_GYROSCOPE) { // read gyroscope info
        I2CReadRegister(&_bbi2c, _iAddr, _iGyroStart, ucTemp, 6);
        for (i=0; i<3; i++) {
           pSample->gyro[i] = get16Bits(&ucTemp[i*2]);
        }
     }
     if (_iMode & MODE_TEMP && _u32Caps & IMU_CAP_TEMPERATURE) { // read the temperature
        I2CReadRegister(&_bbi2c, _iAddr, _iTempStart, ucTemp, _iTempLen);
        if (_iTempLen == 1) {
           pSample->temperature = (int)((int8_t)ucTemp[0]) * 10;
        } else { // two byte temperature value
           i = get16Bits(ucTemp);
           if (_iType == IMU_TYPE_LSM6DS3)
              pSample->temperature = 250 + ((i * 160)/16);
           else if (_iType == IMU_TYPE_MPU6050)
              pSample->temperature = (i/34) + 365;
           else if (_iType == IMU_TYPE_BMI160)
              pSample->temperature = 230 + ((i*10)/512);
        }
     }
     if (_iMode & MODE_STEP && _u32Caps & IMU_CAP_PEDOMETER) { // read step count
        I2CReadRegister(&_bbi2c, _iAddr, _iStepStart, ucTemp, 2);
        pSample->steps = get16Bits(ucTemp);
     }
     return IMU_SUCCESS;
} /* getSample() */
//
// Stop all activity on the IMU
//
int BBIMU::stop(void)
{
} /* stop() */
//
// Return the capability bits of the current device
//
uint32_t BBIMU::caps(void)
{
  return _u32Caps;
} /* caps() */
//
// Return the enumerated type of the current device
//
int BBIMU::type(void)
{
  return _iType;
} /* type() */

