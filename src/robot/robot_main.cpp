#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"

int previousCommand = 0;
float currentYaw = 0;
float initialYaw = 0;
extern struct euler_t {
    float yaw;
    float pitch;
    float roll;
} ypr;

 
void setup() {
    
    setupIMU();
    setupDrive();
    readIMU(false);
    Serial1.begin(115200, SERIAL_8N1, 17, 18); // RX=17 TX=18 – change if needed
    Serial.println("ESP32-S3 ready");

}

void loop() {
    readIMU(false);
    currentYaw = ypr.yaw;
    jetsonOutput output = jetsonComms();
    if (previousCommand != output.COMMAND) initialYaw = ypr.yaw;
    switch (output.COMMAND)
    {
        case 0:
        break;
        case 1:
        rotate(initialYaw, currentYaw, output.INPUT_VAL);
        break;
        case 2:
        break;
        default:
    }

    // Update PID at 1Hz
    EVERY_N_MILLIS(1000) {
       updatePIDs();
    }
}