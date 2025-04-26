#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "imu_test.cpp"
#include <Adafruit_BNO08x.h>

// Create an instance of the MotorDriver class
MotorDriver driveMotors[2] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1} };
MotorDriver flywheels[2] = { {B_DIR1, B_PWM1, 0}, {B_DIR2, B_PWM2, 1} };


// Rotate Variables
float initialYaw = 9999999;
float yawDiff = 0;
float rotateSpeedR = 1;
float rotateSpeedL = -1;

// Align Variables

// Box Collection Variables
float distToPickUp;

// Box Drop Off Variables
float distToDropOff;

void setup() {
    Serial.begin();

    // Setup the motor drivers
    for (uint8_t i = 0; i < 2; i++)
        driveMotors[i].setup();
}

void rotate(int dir) { // 1 = right (clockwise), -1 = left (counterclockwise)
    double speeds[2] = {rotateSpeedR, rotateSpeedL};
    if (dir == -1)
        speeds[0] = rotateSpeedR * -1, speeds[1] = rotateSpeedL * -1;

    if (foundIMU) {
        if (initialYaw == 9999999) {         // Set the motors to rotate
            initialYaw = ypr.yaw;
            driveMotors[0].drive(speeds[0]);
            driveMotors[1].drive(speeds[1]);
        }
        
        yawDiff = abs(initialYaw - ypr.yaw);
        if (yawDiff >= 90) {                 // Stop both motors
            driveMotors[0].drive(0);
            driveMotors[1].drive(0);
        }
    
        Serial.println(initialYaw);           Serial.print("\t");
        Serial.print(ypr.yaw);                Serial.print("\t");
        Serial.print(yawDiff);                Serial.print("\t");
    }
}

void align() {

}

void pickUpBox() {

}

void dropOffBox() {

}