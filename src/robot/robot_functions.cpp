#include <Arduino.h>
#include "robot_pinout.h"
#include "robot_drive.h"
#include "MotorDriver.h"
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"
#include "wireless.h"
#include "util.h"

// Rotate Variables

// float initialYaw = 0;
// float rotateSpeedR = -1; //Default set to clockwise rotation
// float rotateSpeedL = 1;

// Align Variables
float camToBackDist = 7543789543975943; // MEASURE
float alignSpeed = 1;

// Box Collection Variables
float distToPickUp = 46382537298543432; // MEASURE
float driveUpSpeed = 1;
double flywheelSpeed = .5;

// Box Drop Off Variables
float distToDropOff = 35854392574232102; // MEASURE

// IMU Reference
extern struct euler_t {
    float yaw;
    float pitch;
    float roll;
 } ypr;

 // External UART Connection declaration
 extern HardwareSerial mySerial; // Use UART1

// External Wireless Comms
extern bool freshWirelessData;
extern ControllerMessage controllerMessage;
extern RobotMessage robotMessage;

// External state machine state
extern enum { WAITING, ACTIVE, FINISHED, JOYSTICK_INTERRUPT } state;

jetsonOutput jetsonComms() {
    // declare output struct
    jetsonOutput output;

    // Read Jetson message from serial
    String input = mySerial.readStringUntil('\n');
    input.trim(); // Remove whitespace/newline
    Serial.println("Received on UART1: " + input); // USB debug

    if (input == "HELLO") {
        mySerial.println("ACK HELLO");
        Serial.println("HELLO from robot functions");
        output.COMMAND = NO_STATE_DETECTED;
        return output;
    }

    // Parse Jetson comma-seperated message

    // Create char array to fill with chars of input
    char inputChar[input.length()];
    // copy input chars to inputChar
    strcpy(inputChar, input.c_str());
    // get tokens for each part of Jetson msg
    char* input_command = strtok(inputChar, ",");
    char* input_funcValChar = strtok(NULL, ",");

    // convert tokens to strings
    String input_parsed[2];
    input_parsed[0] = input_command;
    input_parsed[1] = input_funcValChar;

    // convert function arg to int if it exists
    int input_val_int;
    if(input_parsed[1] != NULL) {
        input_val_int = input_parsed[1].toInt();
    }

    // Decode Jetson command from string to enum
    input_parsed[0].toUpperCase(); // Ensure command is same case
    if(input_parsed[0] == "SETUP") {
        output.COMMAND = SETUP;
    } else if(input_parsed[0] == "ROTATE" && input_val_int==-1){
        output.COMMAND = ROTATE_CCW;
    } else if(input_parsed[0] == "ROTATE" && input_val_int==1){
        output.COMMAND = ROTATE_CW;
    } else if(input_parsed[0] == "ALIGN_F"){
        output.COMMAND = ALIGN_F;
    } else if(input_parsed[0] == "ALIGN_B"){
        output.COMMAND = ALIGN_B;
    } else if(input_parsed[0] == "A_PICKUP"){
        output.COMMAND = APPROACH_PICKUP_POSE;
    } else if(input_parsed[0] == "G_BIN"){
        output.COMMAND = GRAB_BIN;
    } else if(input_parsed[0] == "B_TAG"){
        output.COMMAND = FIXED_BACKUP;
    } else if(input_parsed[0] == "D_BIN"){
        output.COMMAND = DEPOSIT_BIN;
    } else if(input_parsed[0] == "B_UP"){
        output.COMMAND = BACKUP;
    } else if(input_parsed[0] == "S_ALIGN"){
        output.COMMAND = S_MANEUVER_ALIGN;
    } else if(input_parsed[0] == "DRIVE_UP_RAMP"){
        output.COMMAND = DRIVE_UP_RAMP;
    } else if(input_parsed[0] == "DRIVE_DOWN_RAMP"){
        output.COMMAND = DRIVE_DOWN_RAMP;
    } else if(input_parsed[0] == "COLOR_DETECT"){
        output.COMMAND = COLOR_DETECT_STORE;
    } else if(input_parsed[0] == "DRIVE_TO_PICKUP"){
        output.COMMAND = DRIVE_TO_PICKUP;
    } else if(input_parsed[0] == "FINISH") {
        output.COMMAND = FINISH;
    } else if(input_parsed[0] == "STOP") {
        output.COMMAND = STOP_DRIVE;
    }

    else {
        Serial.println("Unknown Command: " + input_parsed[0]);
        output.COMMAND = NO_STATE_DETECTED;
    }

    //Serial.println("hello from jestson comms");
    //Serial.print(input_parsed[0]); Serial.println(input_parsed[1]);
    // If function arg exists, overwrite default
    if(input_parsed[1] != NULL){
        output.INPUT_VAL = input_val_int;
    }

    return output;
}

bool checkJoystickInterrupt() {
    if (freshWirelessData && (controllerMessage.debouncedInterrupt == 0)) {
        Serial.println("INTERRUPT TRIGGERED");
        return true;
    }else{
        return false;
    }

}

// Joystick helper function
void followJoystickTrajectory() {
    if (freshWirelessData) {
        double forward = abs(controllerMessage.joystick1.y) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.y, -1, 1, -MAX_FORWARD/3, MAX_FORWARD/3);
        double turn = abs(controllerMessage.joystick1.x) < 0.1 ? 0 : mapDouble(controllerMessage.joystick1.x, -1, 1, -MAX_TURN, MAX_TURN);
        updateDriveSetpoints(.75*(forward + turn), .75*(forward - turn));
        if (Serial) Serial.println("Driving");

    } else {
        Serial.println("No Wireless Data");
    }
}

void readJoystick() {
    EVERY_N_MILLIS(10) {
        followJoystickTrajectory();
        // we want to say if buttonF = 0 update setpoints for both to turn forwards
        // if button R = 0 udpate setpoints for both to turn backwards
        //otherwise setpoints stay 0
        if(controllerMessage.debouncedInputF == 0){
            updateFlywheelSetpoints(-2,-2);
            if (Serial) Serial.println("Grabbing");
        }
        else if(controllerMessage.debouncedInputR == 0){
            updateFlywheelSetpoints(2,2);
            if (Serial) Serial.println("Spitting");
        }
        else{
            updateFlywheelSetpoints(0,0);
            if (Serial) Serial.println("Holding");
        }
    }

    // Update PID at 200Hz
    // EVERY_N_MILLIS(5) {
    //     updatePIDs();
    // }

    // Send and print robot values at 50Hz
    EVERY_N_MILLIS(20) {
        updateOdometry();
        sendRobotData();

        Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
                    robotMessage.x, robotMessage.y, robotMessage.theta);
    }
}


void rotate(float initialYaw, float currentYaw, int dir)
/* dir =  1  → clockwise  (ROTATE_CW)
 * dir = -1  → counter‑clockwise (ROTATE_CCW)
 * Turns ~90 ° then prints "ROTATE_DONE"*/

{   //Serial.println("rotate");
    //Serial.println(initialYaw);
    //Serial.println(currentYaw);
    // 1.  How much have we turned so far?
    float diff = fabsf(currentYaw - initialYaw);
    Serial.println(diff);
    if (diff > 180) {
        diff = 360.0 - diff;     // handle wrap‑around
        Serial.println("Wrap-around");
    }

    // 2.  If we’re within ±5 ° of 90 °, stop and acknowledge
    if (diff >= 85.0) {
        updateDriveSetpoints(0, 0);
        mySerial.println("ROTATE_DONE");      // ← Jetson is listening for this specific String
        //Serial.println("Rotation Finished \n");
        return;
    }

    // 3.  Otherwise keep spinning
    const float SPEED = .75;
    double left  =  SPEED * dir;
    double right = -left;
    //Serial.println(left);
    //Serial.println();

    updateDriveSetpoints(left, right);
}

void driveStraight(int dir) {
    Serial.println("driveStraight");
    double driveSpeed = .6;
    driveSpeed *= dir; // If driving backwards, flip drive vel

    // Update drive setpoints
    updateDriveSetpoints(driveSpeed, driveSpeed);
    mySerial.println("DRIVING");

}

void grabBin() {
    double speeds[2] = {-flywheelSpeed, -flywheelSpeed};

    // Behavior changes if limit switch depressed
    // 0 - Depressed -> False
    // 1 - Open -> True
    if(digitalRead(LIMIT_SWITCH_PIN) == HIGH) {
        // Switch Open
        updateFlywheelSetpoints(speeds[0], speeds[1]);
        return;
    }

    // Switch Closed
    updateFlywheelSetpoints(0, 0);
    mySerial.println("BIN_GRABBED");

}

void depositBin() {
    // Jetson will send stop signal, just get spinin
    double speeds[2] = {flywheelSpeed, flywheelSpeed};
    updateFlywheelSetpoints(speeds[0], speeds[1]);
}
