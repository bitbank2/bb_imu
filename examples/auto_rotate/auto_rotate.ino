//
// bb_imu auto-rotation example
// This program shows how to use an accelerometer to automatically rotate
// display contents to correspond with gravity
//
// This example assumes that you will hang the PCB on a wall, so te Z-axis is through the PCB while
// Y is vertical and X is horizontal
// The 4 rotated orientations will result in these values:
// upright: Y = Gravity, X/Z = 0
// 180 degrees: Y = -Gravity, X/Z = 0
// 90 degrees clockwise: X = Gravity, Y/Z = 0
// 90 degrees counterclockwise: X = -Gravity, Y/Z = 0
#include <bb_imu.h>

BBIMU imu;
#define IMU_SAMPLE_RATE 50
// Change these depending on your hardware
#define SDA_PIN 39
#define SCL_PIN 40

void setup()
{
int rc;
  Serial.begin(115200);
  delay(3000); // allow time for CDC-Serial to start
  Serial.println("Starting");
  // If your Arduino board has default I2C pins defined and your sensor
  // is attached to them, you can call init() with no parameters
  rc = imu.init(SDA_PIN, SCL_PIN);
  if (rc == IMU_SUCCESS) {
    Serial.printf("IMU success!, type = %d\n", imu.type());
  } else {
    Serial.println("IMU init failed");
    while (1) {}
  }
  imu.start(IMU_SAMPLE_RATE, MODE_ACCEL); // enable the accelerometer
}

void loop()
{
IMU_SAMPLE is;
int iOrient = -2; // force an update the first time through
int iNewOrient = 0;

  while (1) {
    imu.getSample(&is);
    iNewOrient = -1; // assume invalid until proven otherwise
    if (is.accel[1] > 13000 && is.accel[0] < 2000 && is.accel[2] < 2000) { // upright
        iNewOrient = 0;
    } else if (is.accel[1] < -13000 && is.accel[0] < 2000 && is.accel[2] < 2000) { // 180 rotated
        iNewOrient = 180;
    } else if (is.accel[0] > 13000 && is.accel[1] < 2000 && is.accel[2] < 2000) { // 90 rotated
        iNewOrient = 90;
    } else if (is.accel[0] < -13000 && is.accel[1] < 2000 && is.accel[2] < 2000) { // 270 rotated
        iNewOrient = 270;
    }
    if (iNewOrient != -1 && iNewOrient != iOrient) { // changed, update the display
        Serial.printf("Orientation changed to %d\n", iNewOrient);
        iOrient = iNewOrient;
    }
    delay(20); // check 50 times per second for orientation changes
  }
}

