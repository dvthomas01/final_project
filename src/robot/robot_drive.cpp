#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"


MotorDriver motors[NUM_MOTORS] = { {A_DIR1, A_PWM1, 0}, {A_DIR2, A_PWM2, 1},
                                   {B_DIR1, B_PWM1, 2}, {B_DIR2, B_PWM2, 3} };


// NUM_MOTORS != NUM_ENCODERS, See comment in robot_drive.h
EncoderVelocity encoders[NUM_ENCODERS] = { {ENCODER1_B_PIN, ENCODER1_A_PIN, CPR_312_RPM, 0.2}, 
                                         {ENCODER2_B_PIN, ENCODER2_A_PIN, CPR_312_RPM, 0.2},  
                                         };

PID pids[NUM_ENCODERS] = { {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false} };


double setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_ENCODERS] = {0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};

void setupDrive() 
{
    for (uint8_t i = 0; i < 4; i++)
        motors[i].setup();
}

void updateDriveSetpoints(double left, double right) {
    setpoints[2] = left; // setpoints[0]; //left flywheel
    setpoints[3] = right; //setpoints[1]; //right flywheel
    // Serial.print(left); Serial.print(" "); Serial.println(right);
//    setpoints[2] = left; //drive
//    setpoints[3] = right; //drive
}

void updateFlywheelSetpoints(double left, double right) {
    setpoints[0] = left; //left flywheel
    setpoints[1] = right; //right flywheel
    // setpoints[2] = setpoints[2]; //drive
    // setpoints[3] = setpoints[3]; //drive
}

void updatePIDs() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        // Drive motors: 2 & 3
        // Flywheels: 0 & 1
        // There are only encoders for 2 & 3
        if (i > 1) {
            velocities[i - 2] = pow(-1, i - 2) * encoders[i - 2].getVelocity();
            controlEfforts[i] = pids[   i - 2].calculateParallel(velocities[i - 2], setpoints[i]);
        } else {
            controlEfforts[i] = setpoints[i];
        }

        motors[i].drive(controlEfforts[i]);
    }
}