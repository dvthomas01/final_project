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

// Wrapper function to initialize all motors
void setupMotors(MotorDriver *motors) 
{
    for (uint8_t i = 0; i < NUM_MOTORS; i++)
        motors[i].setup();
}

// Robot motion hardware setup process
// Input: Array of motorDriver objects representing each motor on the robot
// Output: Yaw (float) at startup
float setupDrive(MotorDriver *motors) {
    // Setup the motor drivers
    setupMotors(motors);

    float initialYaw = ypr.yaw;
    return initialYaw;
}

/*
void straight() {
    double speeds = 1;
    motors[0].drive(speeds);
    motors[1].drive(speeds);
    motors[2].drive(0);
    motors[3].drive(0);
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
    motors[0].drive(speeds[0]);
    motors[1].drive(speeds[1]);
    
    float yawDiff = 0;
    while (yawDiff < 90)
    {
        yawDiff = abs(initialYaw - ypr.yaw);
        if (yawDiff >= 90) {                 
            // Stop both motors
            motors[0].drive(0);
            motors[1].drive(0);
        }
        
        Serial.println(initialYaw);           Serial.print("\t");
        Serial.print(ypr.yaw);                Serial.print("\t");
        Serial.print(yawDiff);                Serial.print("\t");
        delay(10);
    }

    initialYaw = rotationSetPoint; //Reset the initialYaw for the next rotation
}
    */
/*
void align() {
    double speed = alignSpeed;
    if (t[0] > 0) {
        speed = -alignSpeed;
    }

    if (abs(t[0]) > 0.1) {

        // Set the motors to rotate
        driveMotors[0].drive(speed);
        driveMotors[1].drive(speed);
        
        while (abs(t[0] > 0.5))
        {
            if (abs(t[0]) <= 0.5) {
            driveMotors[0].drive(0);
            driveMotors[0].drive(0);
            }
            Serial.println(t[0]);
        }
    }

}
*/

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