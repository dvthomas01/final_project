#ifndef ROBOT_DRIVE_H
#define ROBOT_DRIVE_H

#include "MotorDriver.h"
#define NUM_MOTORS 4

extern MotorDriver motors[NUM_MOTORS];

// 1 Encoder per side of robot (per set of motors in parallel), no encoders on flywheels
// This means we only PID control the rear wheels, and the front wheels just echo it
// This is fune, because the front wheels are omni wheels and will always slip first
#define NUM_ENCODERS 2 

#define Kp 0.25
#define Ki 0.01
#define Kd 0
#define pidTau 0.1

#define MAX_FORWARD 6
#define MAX_TURN 3

void setupDrive();
void updateSetpoints(double left, double right);
void updatePIDs();

#endif // ROBOT_DRIVE_H