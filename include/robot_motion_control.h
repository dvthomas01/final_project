#ifndef ROBOT_MOTION_CONTROL_H
#define ROBOT_MOTION_CONTROL_H

// wheel radius in meters
//#define 
const float r = 0.06;
// distance from back wheel to center in meters
//#define 
const float b = 0.2;

// Define enumeration of vehicle trajectories for followTrajectory
enum trajectoryMode {
    // STOP,
    FORWARD,
    BACKWARD,
    CW,
    CCW
};  

// Define enum of commands for JetsonComms
enum commands {
    SETUP,
    ALIGN_F,
    ALIGN_B,
    ROTATE_CW,
    ROTATE_CCW,
    FINE_ALIGN,
    APPROACH_PICKUP_POSE,
    GRAB_BIN,
    FIXED_BACKUP,
    DEPOSIT_BIN,
    BACKUP,
    S_MANEUVER_ALIGN,
    DRIVE_UP_RAMP,
    DRIVE_DOWN_RAMP,
    COLOR_DETECT_STORE,
    DRIVE_TO_PICKUP,
    FINISH,
    STOP_DRIVE,
    STOP_DEPOSIT,
    NO_STATE_DETECTED
};

// Structure to package JetsonComms output
struct jetsonOutput {
    commands COMMAND;

    // If INPUT_VAL = 999, COMMAND has no input
    int INPUT_VAL = 999;
}; 


// Create templates for functions required for state machine


// LOW-LEVEL FUNCTIONS


// Command robot to rotate 90 degrees CW or CCW
// 1 = CW, -1 = CCW
void rotate(float initialYaw, float currentYaw, int dir);

// Drive in straight line 
// 1 = Forward
// -1 = Backward
void driveStraight(int dir);

// Draw bin into center cavity
// If limit switch activated, (async) sets motors to brake
// If limit switch deactivated, (async) spins flywheels until it is
// Called in main control loop to guarantee bin held while in motion
void grabBin();

// Spit bin out from center cavity
// Just spins flywheels such that bin is ejected for sufficient time (1 sec?)
void depositBin();

// Check if the override button on the controller has been pressed
// If so, enter joystick mode, tell the jetson we're ignoring it
bool checkJoystickInterrupt();


// Now that we're in joystick mode, do the joystick code
void readJoystick();

// Setup function, determines what position (assoc. AprilTag ID) bins start in
// Returns bin color order from left (index 0) to right (index 2) as char array
// b - blue, y - yellow, r - red
// char colorDetectStore();


// HIGH-LEVEL FUNCTIONS


// Perform sequence of commands to go from ramp start to ramp top platform
// void drive_up_ramp();

// Perform sequence of commands to go from ramp top platform to ramp end
// void drive_down_ramp();

// From current position, go to reference starting position for bin pickup
// based on AprilTag waypoint(s)
// void drive_to_pickup();

// Find AprilTag of ID tagID, get in rough alignment to it
// void align(int tagID);

// Ensure the center x,y position of in-frame AprilTag is correct based on specified tag
// Gets robot aligned with bin start/end spot for bin pickup/dropoff
// void fine_align(int tagID);

// Drives forward while keeping AprilTag x,y positon fixed while driving to per-tag predefined
// z-distance for picking up bin
// void approach_pickup_pose(int tagID);


// TODO: Change following if we decide to have unique backup procedure per AprilTag

// Back up a fixed amount after depositing bin
// void backUp();

// Back up a fixed amount after depositing bin
// Distance is defined per-tag
// void backUp(int tagID);

// TODO: Following may not be necessary, especiallyu= if we only drive in straight lines for robustness

// Drive in S-shape to avoid the ramp and align with AprilTag 8
// void s_maneuver_align(int tagID);

// Get current state from Jetson, return jetsonOutput struct with current state and function input
jetsonOutput jetsonComms();


// HELPER FUNCTIONS

// Follow the specified trajectory
// void followTrajectory(trajectoryMode Trajectory);
void followTrajectory();

// Updates Odometry 
// For joystick control
void updateOdometry();


#endif