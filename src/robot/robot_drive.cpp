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


double setpoints[NUM_ENCODERS] = {0, 0};
double velocities[NUM_ENCODERS] = {0, 0};
double controlEfforts[NUM_ENCODERS] = {0, 0};

void setupDrive() 
{
    for (uint8_t i = 0; i < 4; i++)
        motors[i].setup();
}

void updateDriveSetpoints(double left, double right) {
    setpoints[0] = setpoints[0]; //left flywheel
    setpoints[1] = setpoints[1]; //right flywheel
    setpoints[2] = left; //drive
    setpoints[3] = right; //drive
}

void updateFlywheelSetpoints(double left, double right) {
    setpoints[0] = left; //left flywheel
    setpoints[1] = right; //right flywheel
    setpoints[2] = setpoints[2]; //drive
    setpoints[3] = setpoints[3]; //drive
}

void updatePIDs() {
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity();
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
        motors[i+2].drive(controlEfforts[i]);
    }
}