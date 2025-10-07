//
// Magnetometer sample gathering
//
#include <bb_imu.h>
BBIMU imu;

void setup()
{
   Serial.begin(115200);
   delay(3000); // allow time for CDC-Serial to start
   Serial.println("Starting");

   imu.init();
   imu.start();
}

void loop()
{
  IMU_SAMPLE is;
  while (1) {
    imu.getSample(&is);
    Serial.printf("Mag: %d, %d, %d\n", is.mag[0], is.mag[1], is.mag[2]);
    delay(100);
  }
}

