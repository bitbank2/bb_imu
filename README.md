## BitBank Inertial Measurement Unit (IMU) Library

Copyright (c) 2025 BitBank Software, Inc.<br>
Written by Larry Bank<br>
email: bitbank@pobox.com<br>
<br>
This library aims to make it easier to work with IMUs (Inertial Measurement Units). There are a variety of vendors and products which all work in a very similar way. This library aims to simplify their use by providing a single API to work with the most popular products and auto-detect them. The end result is that you'll be able to write your project code and be able to freely change your component choices without having to change a single line of your code.<br>

<b>Getting Started</b><br>
If your sensor is connected to the default I2C bus of your Arduino, this is the easiest scenario to use. Simply call init() with no parameters, then start() with the sensor mode you want to use:<br>
rc = imu.init();
if (rc == IMU_SUCCESS) {<br>
   imu.start(MODE_ACCEL); // start the accelerometer<br>
}<br>
Now you can start to receive accelerometer samples:<br>
while (1) {<br>
    IMU_SAMPLE samp;<br>
    imu.getSample(&samp);<br>
    Serial.printf("X: %d, Y: %d, Z: %d\n", samp.accel[0], samp.accel[1], samp.accel[2]);<br>
}<br>
For more info about the full API, please consult the WiKi<br>

If you find this code useful, please consider becoming a sponsor or sending a donation.

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=SR4F44J2UR8S4)


