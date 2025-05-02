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
        case SETUP:
        break;
        case ALIGN:
        break;
        case ROTATE_CW:
        rotate(initialYaw, currentYaw, output.INPUT_VAL);
        break;
        case ROTATE_CCW:
        rotate(initialYaw, currentYaw, output.INPUT_VAL);
        break;
        case FINE_ALIGN:
        break;
        case APPROACH_PICKUP_POSE:
        break;
        case GRAB_BIN:
        grabBin();  
        break;
        case DEPOSIT_BIN:
        depositBin();  
        break;
        case FIXED_BACKUP:
        break;
        case BACKUP:
        break;
        case S_MANEUVER_ALIGN:
        break;
        case DRIVE_UP_RAMP:
        break;
        case DRIVE_DOWN_RAMP:
        break;
        case COLOR_DETECT_STORE:
        break;
        case DRIVE_TO_PICKUP:
        break;
        default:
    }

    // Update PID at 1Hz
    EVERY_N_MILLIS(1000) {
       updatePIDs();
    }
}