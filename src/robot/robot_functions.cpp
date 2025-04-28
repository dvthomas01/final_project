#include <Arduino.h>
#include "robot_pinout.h"
#include "robot_drive.h"
#include "MotorDriver.h"
#include <Adafruit_BNO08x.h>
#include "imu.h"

// Create an instance of the MotorDriver class
MotorDriver driveMotors[2] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1} };
MotorDriver flywheels[2] = { {B_DIR1, B_PWM1, 0}, {B_DIR2, B_PWM2, 1} };

// Rotate Variables
float initialYaw = 9999999;
float rotateSpeedR = -1; //Default set to clockwise rotation
float rotateSpeedL = 1;

// Align Variables
float camToBackDist = 7543789543975943; // MEASURE

// Box Collection Variables
float distToPickUp = 46382537298543432; // MEASURE
float driveUpSpeed = 1;

// Box Drop Off Variables
float distToDropOff = 35854392574232102; // MEASURE

// IMU Reference
extern struct euler_t {
    float yaw;
    float pitch;
    float roll;
 } ypr;


void setupDrive() {
    // Setup the motor drivers
    for (uint8_t i = 0; i < 2; i++) {
        driveMotors[i].setup();
        flywheels[i].setup();
    }

    initialYaw = ypr.yaw;
}

void setupMotors() 
{
    for (uint8_t i = 0; i < 2; i++)
        driveMotors[i].setup();
    for (uint8_t i = 0; i < 2; i++)
        flywheels[i].setup();
}

void rotate(int dir) { // 1 = right (clockwise), -1 = left (counterclockwise)
    double speeds[2] = {rotateSpeedR, rotateSpeedL};
    float rotationSetPoint = initialYaw - 90;

    if (dir == -1) {
        // Flip the speeds and set point around
        speeds[0] = rotateSpeedR * -1; speeds[1] = rotateSpeedL * -1;
        rotationSetPoint = initialYaw + 180;
    }

    // Set the motors to rotate
    driveMotors[0].drive(speeds[0]);
    driveMotors[1].drive(speeds[1]);
    
    float yawDiff = 0;
    while (yawDiff < 90)
    {
        yawDiff = abs(initialYaw - ypr.yaw);
        if (yawDiff >= 90) {                 
            // Stop both motors
            driveMotors[0].drive(0);
            driveMotors[1].drive(0);
        }
        
        Serial.println(initialYaw);           Serial.print("\t");
        Serial.print(ypr.yaw);                Serial.print("\t");
        Serial.print(yawDiff);                Serial.print("\t");
        delay(10);
    }

    initialYaw = rotationSetPoint; //Reset the initialYaw for the next rotation
}

void align() {

}

void pickUpBox() {
    double speeds[2] = {driveUpSpeed, driveUpSpeed};


}

void dropOffBox() {

}