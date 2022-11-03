#include <Arduino.h>
#include <Wire.h>

#include "bmi270.h"

BMI270 imu = BMI270(&Wire);
bmi270_accelerometer_data xl_data;

void setup() {
    Serial.begin(115200);

    Wire.begin();

    delay(1);

    if (imu.begin()) {
        Serial.print("Connection established with BMI270\n");
    }

    imu.soft_reset();
    delay(200);

    bool result = imu.sensor_init();
    if (result == true) {
        Serial.print("Sensor init success\n");
    }

    imu.normal_mode();

    delay(1000);  // Just a start up delay not needed though
}

void loop() {
    imu.read(xl_data);

    Serial.print("acc_X : ");
    Serial.print(xl_data.x);

    Serial.print("acc_y : ");
    Serial.print(xl_data.y);

    Serial.print("acc_z : ");
    Serial.print(xl_data.z);
    Serial.print("\n");

    delay(100);
}