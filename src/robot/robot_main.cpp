#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"

HardwareSerial mySerial(1); // Use UART1

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

    mySerial.begin(115200, SERIAL_8N1, 44, 43); // RX=44, TX=43
    Serial.begin(115200); // Optional: USB debug output
    Serial.println("Debug serial ready");
    mySerial.println("ESP32-S3 UART1 ready");
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
        break;
        case FIXED_BACKUP:
        break;
        case DEPOSIT_BIN:
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
        // default:
    }

    // Update PID at 1Hz
    EVERY_N_MILLIS(1000) {
       updatePIDs();
    }

    delay(10); // Avoid flooding CPU, only run loop every 10 ms
}