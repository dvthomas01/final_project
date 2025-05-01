#include <Arduino.h>
#include "robot_pinout.h"
#include "MotorDriver.h"
#include "PID.h"
#include "EncoderVelocity.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"


MotorDriver motors[NUM_MOTORS] = { {B_DIR1, B_PWM1, 0}, {B_DIR2, B_PWM2, 1},
                                   {A_DIR1, A_PWM1, 2}, {A_DIR2, A_PWM2, 3} };


EncoderVelocity encoders[NUM_MOTORS] = { {ENCODER1_B_PIN, ENCODER1_A_PIN, CPR_312_RPM, 0.2}, 
                                        {ENCODER2_B_PIN, ENCODER2_A_PIN, CPR_312_RPM, 0.2},  
                                        {ENCODER3_B_PIN, ENCODER3_A_PIN, CPR_312_RPM, 0.2},
                                         {ENCODER4_B_PIN, ENCODER4_A_PIN, CPR_312_RPM, 0.2},
                                         };

PID pids[NUM_MOTORS] = { {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false}, 
                         {Kp, Ki, Kd, 0, pidTau, false}, {Kp, Ki, Kd, 0, pidTau, false} };


double setpoints[NUM_MOTORS] = {0, 0, 0, 0};
double velocities[NUM_MOTORS] = {0, 0, 0, 0};
double controlEfforts[NUM_MOTORS] = {0, 0, 0, 0};

void setupDrive() 
{
    for (uint8_t i = 0; i < 4; i++)
        motors[i].setup();
}

void updateSetpoints(double left, double right) {
    setpoints[0] = left;
    setpoints[1] = right;
    setpoints[2] = 0;
    setpoints[3] = 0;
}

void updatePIDs() {
    for (uint8_t i = 0; i < 4; i++) {
        velocities[i] = pow(-1, i) * encoders[i].getVelocity();
        controlEfforts[i] = pids[i].calculateParallel(velocities[i], setpoints[i]);
        motors[i].drive(controlEfforts[i]);
    }
}