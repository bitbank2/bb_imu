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
#include "BMI270_config.inl"

int16_t bmi270_rates[] = {0, 1, 2, 3, 6, 12, 25, 50, 100, 200, 400, 800, 1600, 3200, 6400, 12800, -1};
int16_t lis3dsh_rates[] = {0, 3, 6, 12, 25, 50, 100, 400, 800, 1600, -1};
int16_t lsm6ds3_rates[] = {0, 12, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660, -1};
int16_t lsm9ds1_accel_rates[] = {0, 10, 50, 119, 238, 476, 952, -1};
int16_t lsm9ds1_gyro_rates[] = {0, 15, 60, 119, 238, 476, 952, -1};
const int16_t mpu6050_rates[] = {0, 3, 7, 15, 31, 62, 125, 250, 500, 1000, 2000, 4000, 8000, -1}; 
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
    if (I2CTest(&_bbi2c, IMU_BNO055_ADDR+iOffset)) {
       // try to read the "CHIP_ID" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_BNO055_ADDR + iOffset, 0x0, ucTemp, 1);
       if (ucTemp[0] == 0xa0) {
           _iType = IMU_TYPE_BNO055;
           _iAddr = IMU_BNO055_ADDR + iOffset;
           _bBigEndian = false;
           _iMagStart = 0xe;
           _iAccStart = 0x8;
           _iGyroStart = 0x14;
           _iTempStart = 0x34;
           _iTempLen = 1;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_MAGNETOMETER | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE | IMU_CAP_3DPOS;
           return IMU_SUCCESS;
       }
    }
    if (I2CTest(&_bbi2c, IMU_BMI270_ADDR+iOffset)) {
       // try to read the CHIP_ID register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_BMI270_ADDR+iOffset, 0x0, ucTemp, 1);
       if (ucTemp[0] == 0x24) {
           _iType = IMU_TYPE_BMI270;
           _iAddr = IMU_BMI270_ADDR + iOffset;
           _bBigEndian = false;
           _iAccStart = 0xc;
           _iGyroStart = 0x12;
           _iTempStart = 0x22;
           _iTempLen = 2;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
           return IMU_SUCCESS;
       } 
    }
    if (I2CTest(&_bbi2c, IMU_LSM9DS1_ADDR+iOffset)) {
       // try to read the "WHO_AM_I" register
       ucTemp[0] = 0;
       I2CReadRegister(&_bbi2c, IMU_LSM9DS1_ADDR + iOffset, 0x0f, ucTemp, 1);
       if (ucTemp[0] == 0x68) {
           _iType = IMU_TYPE_LSM9DS1;
           _iAddr = IMU_LSM9DS1_ADDR + iOffset;
           _bBigEndian = false;
           _iAccStart = 0x28;
           _iGyroStart = 0x18;
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
       if (ucTemp[0] == 0x69 || ucTemp[0] == 0x6a) { // normal or "C" variant
           _iType = IMU_TYPE_LSM6DS3;
           _iAddr = IMU_LSM6DS3_ADDR + iOffset;
           _bBigEndian = false;
           _iStatus = 0x1e; // status register
           _iAccStart = 0x28;
           _iGyroStart = 0x22;
           _iTempStart = 0x20;
           _iTempLen = 2;
           _iStepStart = 0x4b;
           _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
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
          _iAccStart = 0x3b;
          _iGyroStart = 0x43;
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
          _u32Caps = IMU_CAP_ACCELEROMETER | IMU_CAP_GYROSCOPE | IMU_CAP_FIFO | IMU_CAP_TEMPERATURE;
          return IMU_SUCCESS;
       }
    }
    } // for each address offset
    return IMU_ERROR;
} /* init() */
int BBIMU::getQueuedSamples(int16_t *pSamples, int *iNumSamples, int iMaxSamples)
{
int16_t *d = (int16_t *)pSamples;
uint8_t ucTemp[4];
int i, iNum, iCount;

    if (_iType == IMU_TYPE_LSM6DS3) {
        // read the FIFO status
        if (!I2CReadRegister(&_bbi2c, _iAddr, 0x3a, ucTemp, 4))
        {
            return IMU_ERROR;
        }
//        if (ucTemp[1] & 0x10) { // FIFO is empty
//            *iNumSamples = 0;
//            return MT_SUCCESS;
//        }
        iNum = ucTemp[0] + ((ucTemp[1] & 0xf) << 8); // number of unread 16-bit axis in FIFO (12 bits)
        if (iNum == 0) {
            *iNumSamples = 0;
            return IMU_SUCCESS;
        }
        iCount = 0;
        if (_iMode & MODE_ACCEL) iCount += 3;
        if (_iMode & MODE_GYRO) iCount += 3;
        if ((iNum / iCount) > iMaxSamples) {
            iNum = iCount * iMaxSamples;
        }
        for (i=0; i<iNum; i++) { // read an even number of samples
            if (!I2CReadRegister(&_bbi2c, _iAddr, 0x3e, ucTemp, 2))
            {
                return IMU_ERROR;
            }
            *d++ = (int16_t)(ucTemp[0] | (ucTemp[1]<<8));
        }
        *iNumSamples = iNum / iCount;
    }
    return IMU_SUCCESS;
} /* getQueuedSamples() */

//
// Configure the channels used for the FIFO and activate that mode
//
int BBIMU::configFIFO(void)
{
    uint8_t ucEnable, ucTemp[4];
    int iODR;

        if (_iType == IMU_TYPE_LSM6DS3) {
            // calculate the ODR (output data rate)
            iODR = 0;
            if (_iAccRate == 12) {
                iODR = 1; // 12.5hz low power
            } else if (_iAccRate == 26) {
                iODR = 2; // 26hz
            } else if (_iAccRate == 52) {
                iODR = 3; // 52hz
            } else if (_iAccRate == 104) {
                iODR = 4; // 104hz
            } else if (_iAccRate == 208) {
                iODR = 5; // 208hz
            } else if (_iAccRate == 416) {
                iODR = 6; // 416hz
            } else if (_iAccRate == 833) {
                iODR = 7; // 833hz
            } else if (_iAccRate == 1660) {
                iODR = 8; // 1660hz
            } else if (_iAccRate == 3330) {
                iODR = 9; // 3330hz
            } else if (_iAccRate == 6660) {
                iODR = 10; // 6660hz
            }
            // set bypass mode first to reset the FIFO
            ucTemp[0] = 0x0a; // FIFO_CTRL5
            ucTemp[1] = 0; // bypass mode (FIFO_MODE [2:0] = 000)
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            // set the FIFO threshold value
            ucTemp[0] = 6; // FIFO_CTRL1 & FIFO_CTRL2
            ucTemp[1] = 0; // low byte
            ucTemp[2] = 8; // high byte (2048 samples)
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 3);

            // Enable the accelerometer, gyro or both
            ucEnable = 0;
            if (_iMode & MODE_ACCEL) {
                // enable only the requested channels
//                ucTemp[0] = 0x18; // CTRL9_XL select channels for accelerometer
//                ucTemp[1] = 0;
//                if (u32Channels & IMU_CHANNEL_ACC_X) ucTemp[1] |= 8;
//                if (u32Channels & IMU_CHANNEL_ACC_Y) ucTemp[1] |= 16;
//                if (u32Channels & IMU_CHANNEL_ACC_Z) ucTemp[1] |= 32;
//                I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
                ucEnable |= 0x01; // enable accelerometer with no decimation
            }
            if (_iMode & MODE_GYRO) {
                // enable only the requested channels
//                ucTemp[0] = 0x19; // CTRL10_C select channels for gyroscope
//                ucTemp[1] = 0;
//                if (u32Channels & IMU_CHANNEL_GYR_X) ucTemp[1] |= 8;
//                if (u32Channels & IMU_CHANNEL_GYR_Y) ucTemp[1] |= 16;
//                if (u32Channels & IMU_CHANNEL_GYR_Z) ucTemp[1] |= 32;
//                I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
                ucEnable |= 0x8; // enable gyroscope with no decimation
            }
            ucTemp[0] = 0x8; // FIFO_CTRL3
            ucTemp[1] = ucEnable; // enables acc, gyr or both
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            // turn on the FIFO
            ucTemp[0] = 0x0a; // FIFO_CTRL5
            ucTemp[1] = (iODR << 3); // FIFO mode enabled
            ucTemp[1] |= 0x06; // continuous update - old data is tossed as new arrives
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
    //        ucTemp[0] = 0x1a; // MASTR_CONFIG
    //        ucTemp[1] = 0x00; // start?
    //        I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
        }
        return IMU_SUCCESS;

} /* configFIFO() */

//
// Read the status register
// This is usually needed to clear the last interrupt event
//
uint8_t BBIMU::getStatus(void)
{
    uint8_t uc;
    I2CReadRegister(&_bbi2c, _iAddr, _iStatus, &uc, 1);
    return uc;
} /* getStatus() */

int BBIMU::configIRQ(bool bOn)
{
uint8_t ucTemp[4];
    
    switch (_iType) {
        case IMU_TYPE_BMI270:
            break;
        case IMU_TYPE_LSM6DS3:
            ucTemp[0] = 0x0d; // INT1_CTRL
            ucTemp[1] = (bOn) ? 0x03 : 0x00; // INT1_DRDY_G | INT1_DRDY_XL; // data ready acc+gyr
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            break;
        case IMU_TYPE_MPU6050:
            break;
        case IMU_TYPE_ADXL345:
            break;
        case IMU_TYPE_MPU6886:
            break;
        case IMU_TYPE_BMI160:
            break;
        case IMU_TYPE_LIS3DH:
        case IMU_TYPE_LIS3DSH:
           break; // LIS3DH / LIS3DSH
        case IMU_TYPE_LSM9DS1:
           break; // LSM9DS1
        default:
           return IMU_ERROR;
    } // switch on type
    return IMU_SUCCESS;
} /* configIRQ() */

//
// Start the accelerometer, gyroscope or both
// with the given sample rate
//
int BBIMU::start(int iSampleRate, int iMode)
{
uint8_t ucTemp[4];
int iRate;

   _iMode = iMode;
   switch (_iType) {
      case IMU_TYPE_BMI270:
         ucTemp[0] = 0x7e; // CMD_REG_ADDR
         ucTemp[1] = 0xb6; // SOFT_RESET_CMD
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         delay(100);
         ucTemp[0] = 0x7c; // power configuration
         ucTemp[1] = 0; // pwr save disabled
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         delay(4);
 //        ucTemp[0] = 0x5b; // INIT_ADDR_0
 //        ucTemp[1] = 0x00;
 //        ucTemp[2] = 0x00;
 //        I2CWrite(&_bbi2c, _iAddr, ucTemp, 3);
         I2CWrite(&_bbi2c, _iAddr, (uint8_t *)bmi270_config_file, sizeof(bmi270_config_file));
         ucTemp[0] = 0x59; // INIT_CTRL
         ucTemp[1] = 1; // start initialization
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
//         ucTemp[0] = 0x58; // INT_MAP_DATA_ADDR
//         ucTemp[1] = 0xff;
//         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         delay(100);
         // set rate and range
         _iAccRate = iSampleRate;
         iRate = 1+matchRate(_iAccRate, &bmi270_rates[0]); 
         _iAccRate = bmi270_rates[iRate]; // get the quantized value
         ucTemp[0] = 0x40; // accel rate (0x41 = range)
         ucTemp[1] = iRate;
         ucTemp[2] = 0x00; // +/- 2g range
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 3);
         // set the same rate for the gyroscope
         ucTemp[0] = 0x42; // gyro rate (0x43 = range)
         ucTemp[1] = iRate;
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
// enable requested sensors
         ucTemp[0] = 0x7d; // power control
         ucTemp[1] = 8; // enable temperature register
         if (_iMode & MODE_ACCEL) {
            ucTemp[1] |= 4; // enable accelerometer
         }
         if (_iMode & MODE_GYRO) {
            ucTemp[1] |= 2; // enable gyroscope
         }
         I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         break; // BMI270

      case IMU_TYPE_LSM6DS3:
         // If accelerometer enabled
         if (_iMode & MODE_ACCEL) {
            _iAccRate = iSampleRate;
            iRate = 1 + matchRate(_iAccRate, &lsm6ds3_rates[0]); 
            _iAccRate = lsm6ds3_rates[iRate]; // get the quantized value
            ucTemp[0] = 0x10; // CTRL1_XL
            ucTemp[1] = (iRate<<4); // iODR << 4;
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         } // accelerometer enabled
         // if gyroscope enabled
         if (_iMode & MODE_GYRO) {
            _iGyroRate = iSampleRate;
            iRate = 1 + matchRate(_iGyroRate, &lsm6ds3_rates[0]);
            ucTemp[0] = 0x11; // CTRL2_G
            if (iRate > 8) iRate = 8; // Gyro max rate = 1660hz
            _iGyroRate = lsm6ds3_rates[iRate]; // get the quantized value
            ucTemp[1] = (iRate<<4); // gyroscope data rate
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
            ucTemp[1] = 0x00;
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(10);

            ucTemp[1] = 0x80; // reset chip
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(10);
            ucTemp[1] = 1; // select the best available oscillator
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
            delay(10);
            ucTemp[0] = 0x1c; // ACCEL_CONFIG
            ucTemp[1] = 0x10; // full scale = 2G, all axes enabled
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
//            delay(1);
//            ucTemp[0] = 0x38; // INT_ENABLE
//            ucTemp[1] = 0x01; // enable interrupt on data ready
//            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
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
      case IMU_TYPE_LSM9DS1:
         if (_iMode & MODE_ACCEL) {
            ucTemp[0] = 0x20; // CTRL_REG6_XL (accelerometer control) 
            ucTemp[1] = 0x80; // output rate 238Hz
            I2CWrite(&_bbi2c, _iAddr, ucTemp, 2);
         }
         break; // LSM9DS1
      default:
         return IMU_ERROR;
   } // switch
   return IMU_SUCCESS;
} /* start() */
int BBIMU::reset(void)
{
    return IMU_SUCCESS;
} /* reset() */

void BBIMU::setAccScale(int iScale)
{
   _iAccScale = iScale;
} /* setAccScale() */

void BBIMU::setGyroScale(int iScale)
{
   _iGyroScale = iScale;
} /* setGyroScale() */

int BBIMU::getAccScale(void)
{
   return _iAccScale;
} /* getAccScale() */

int BBIMU::getGyroScale(void)
{
   return _iGyroScale;
} /* getGyroScale() */


//
// Set the accelerometer sampling rate
// not all rates are possible, so the closest
// valid value will be chosen. Use getAccRate() to
// see the actual value used
//
void BBIMU::setAccRate(int iRate)
{
   _iAccRate = iRate;
} /* setAccRate() */

int BBIMU::getAccRate(void)
{
   return _iAccRate;
} /* getAccRate() */

//
// Set the gyroscope sampling rate
// not all rates are possible, so the closest
// valid value will be chosen. Use getGyriRate() to 
// see the actual value used
//
void BBIMU::setGyroRate(int iRate)
{
   _iGyroRate = iRate;
} /* setGyroRate() */
int BBIMU::getGyroRate(void)
{
   return _iGyroRate;
} /* getGyroRate() */

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
// Get a single channel's accelerometer or gyroscope sample
// This can speed up access if you just need to read 1 channel
//
int16_t BBIMU::getOneChannel(uint32_t u32Channel)
{
    int iOff = 0;
    uint8_t ucTemp[4] = {0};
    
    if ((_iMode & MODE_ACCEL) && (_u32Caps & IMU_CAP_ACCELEROMETER) && (u32Channel & (IMU_CHANNEL_ACC_X | IMU_CHANNEL_ACC_Y | IMU_CHANNEL_ACC_Z))) { // read accelerometer info
        iOff = _iAccStart;
        if (u32Channel & IMU_CHANNEL_ACC_Y) iOff += 2;
        else if (u32Channel & IMU_CHANNEL_ACC_Z) iOff += 4;
    }
    if ((_iMode & MODE_GYRO) && (_u32Caps & IMU_CAP_ACCELEROMETER) && (u32Channel & (IMU_CHANNEL_ACC_X | IMU_CHANNEL_ACC_Y | IMU_CHANNEL_ACC_Z))) { // read accelerometer info
        iOff = _iGyroStart;
        if (u32Channel & IMU_CHANNEL_GYR_Y) iOff += 2;
        else if (u32Channel & IMU_CHANNEL_GYR_Z) iOff += 4;
    }
    
    I2CReadRegister(&_bbi2c, _iAddr, iOff, ucTemp, 2);
    return get16Bits(ucTemp);
} /* getOneChannel() */

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
           else if (_iType == IMU_TYPE_BMI160 || _iType == IMU_TYPE_BMI270)
              pSample->temperature = 230 + ((i*10)/512);
           else if (_iType == IMU_TYPE_LSM9DS1)
              pSample->temperature = 250 + ((i * 10)/16);
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
    return IMU_SUCCESS;
} /* stop() */
//
// Match the requested rate to the closest supported value
//
int BBIMU::matchRate(int value, int16_t *pList)
{
int index = 0;

   while (pList[index] != -1) {
     if (value > pList[index] && value <= pList[index+1]) {
         break;
     }
     index++;
   }
   if (pList[index] == -1)
      index--; // return the max value

   return index;
} /* matchRate() */

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

