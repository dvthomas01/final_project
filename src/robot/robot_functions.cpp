#include <Arduino.h>
#include "robot_pinout.h"
#include "robot_drive.h"
#include "MotorDriver.h"
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"

// Create an instance of the MotorDriver class
/*
MotorDriver driveMotors[2] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1} };
MotorDriver flywheels[2] = { {B_DIR1, B_PWM1, 0}, {B_DIR2, B_PWM2, 1} };
*/

/*
EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_A_PIN, ENCODER1_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER2_A_PIN, ENCODER2_B_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER3_A_PIN, ENCODER3_B_PIN, CPR_312_RPM, 0.2}, 
                                         {ENCODER4_A_PIN, ENCODER4_B_PIN, CPR_312_RPM, 0.2} };
*/

// Rotate Variables
// float initialYaw = 0;
float rotateSpeedR = -1; //Default set to clockwise rotation
float rotateSpeedL = 1;


// Align Variables
float camToBackDist = 7543789543975943; // MEASURE
float alignSpeed = 1;

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


jetsonOutput jetsonComms() {
    jetsonOutput output;

    // Read state and input, echo state
    if (Serial1.available()) {          // data from Jetson
        String line = Serial1.readStringUntil('\n');
        Serial.print  ("RX: "); Serial.println(line);
        Serial1.print ("ACK ");           // bounce something back
        Serial1.println(line);            // → Jetson reads "ACK HELLO"


        output.COMMAND = SETUP;
        return output;
    }

    output.COMMAND = NO_STATE_DETECTED;
    return output;
}

void straightline() {
    trajectoryMode trajectory = FORWARD;
    followTrajectory(trajectory);
}


void rotate(int dir) {
    // Update Initial IMU readings
    readIMU(false);
    float statingYaw = ypr.yaw;
    // double speeds[2] = {rotateSpeedR, rotateSpeedL};
    // float rotationSetPoint = initialYaw - 90;

    // if (dir == -1) {
    //     // Flip the speeds and set point around
    //     speeds[0] = rotateSpeedR * -1; speeds[1] = rotateSpeedL * -1;
    //     rotationSetPoint = initialYaw + 180;
    // }

    // Set rotation and goal angle based on direction input
    trajectoryMode trajectory;


    if(dir == 1) {
        trajectory = CW;
    } else {
        trajectory = CCW;   
    }

    followTrajectory(trajectory);


    // Wait until we reach yawGoal degrees rotaton in specified direction
    updatePIDs();
    float yawDiff = 0;
    float yawGoal = 90; // Define out here so we only need to change it once if needed
    while (abs(yawDiff) < yawGoal)
    {
        yawDiff = abs(statingYaw - ypr.yaw);
        readIMU(false);
        if (yawDiff >= yawGoal) {                 
           // Stop both motors
           trajectory = STOP;
           followTrajectory(trajectory);
           updatePIDs();
        }
        
        Serial.print(statingYaw);           Serial.print("\t");
        Serial.print(ypr.yaw);                Serial.print("\t");
        Serial.println(yawDiff);                Serial.print("\t");
        delay(10);
    }
}

void grabBin() {
    double speeds[2] = {driveUpSpeed, driveUpSpeed};
    

}

void depositBin() {

}