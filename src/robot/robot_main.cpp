#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_drive.h"

void setup() {
    setupIMU();
    setupDrive();
}

void loop() {
    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {

    }  
}