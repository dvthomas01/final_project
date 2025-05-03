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

 // External UART Connection declaration
 extern HardwareSerial mySerial; // Use UART1

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
    } else if(input_parsed[0] == "ALIGN"){
        output.COMMAND = ALIGN;
    } else if(input_parsed[0] == "ROTATE" && input_val_int==-1){
        output.COMMAND = ROTATE_CCW;
    } else if(input_parsed[0] == "ROTATE" && input_val_int==1){
        output.COMMAND = ROTATE_CW;
    } else if(input_parsed[0] == "F_ALIGN"){
        output.COMMAND = FINE_ALIGN;
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
    } else {
        Serial.println("Unknown Command: " + input_parsed[0]);
        output.COMMAND = NO_STATE_DETECTED;
    }
    
    Serial.println("hello from jestson comms");
    // If function arg exists, overwrite default
    if(input_parsed[1] != NULL){
        output.INPUT_VAL = input_val_int;
    }

    return output;
}
/*
void straightline() {
    trajectoryMode trajectory = FORWARD;
    followTrajectory(trajectory);
}*/



void rotate(float initialYaw, float currentYaw, int dir)
/* dir =  1  → clockwise  (ROTATE_CW)
 * dir = -1  → counter‑clockwise (ROTATE_CCW)
 * Turns ~90 ° then prints "ROTATE_DONE"
 */
{
    // 1.  How much have we turned so far?
    float diff = fabsf(currentYaw - initialYaw);
    if (diff > 180) diff = 360.0f - diff;     // handle wrap‑around

    // 2.  If we’re within ±5 ° of 90 °, stop and acknowledge
    if (diff >= 85.0f) {
        updateDriveSetpoints(0, 0);                // brakes on both wheels
        updatePIDs();
        mySerial.println("ROTATE_DONE");      // ← Jetson is listening for this
        return;                               // no further motor commands
    }

    // 3.  Otherwise keep spinning
    const float SPEED = 0.75f;                // tweak to taste
    double left  =  SPEED * dir;              // CW:  +0.75  CCW: –0.75
    double right = -left;                     // opposite wheel
    updateDriveSetpoints(left, right);
    updatePIDs();
}

/*void rotate(float initialYaw, float currentYaw, int dir) { // 1 = right (clockwise), -1 = left (counterclockwise), ccw is positive
    double left = 0;
    double right = 0;    
    
    if ((initialYaw > -86.5) && (dir == 1))
    {
        float yawSetpoint = initialYaw - 86.5;
        if (currentYaw > yawSetpoint){
            left = 0.75;
            right = -left;
        }
        Serial.println("Hi4");
    }    
    
    if ((initialYaw < -86.5) && (dir == 1)) {
        float yawSetpoint = initialYaw + 273.5;
        float yawDiff = abs(yawSetpoint - currentYaw);
        if (((currentYaw > yawSetpoint) && (yawDiff < 90)) || ((currentYaw < yawSetpoint) && (yawDiff > 180))) {
            left = -0.75;
            right = -left;
            Serial.println("Hi");
        }
    }    
    
    if ((initialYaw < 86.5) && (dir == -1))
    {
        float yawSetpoint = initialYaw + 86.5;
        if (currentYaw > yawSetpoint) {
            left = -0.75;
            right = - left;
        }
        Serial.println("Hi2");
    }    
    
    if ((initialYaw > 86.5) && (dir == -1)) {
        float yawSetpoint = initialYaw - 273.5;
        float yawDiff = abs(yawSetpoint - currentYaw);
        if (((currentYaw < yawSetpoint) && (yawDiff < 90)) || ((currentYaw > yawSetpoint) && (yawDiff > 180))) {
            left = -0.75;
            right = -left;
        }
        Serial.println("Hi3");
    }    
    Serial.println(left);
    updateSetpoints(left, right);
    updatePIDs();
}*/

void grabBin() {
    double speeds[2] = {driveUpSpeed, driveUpSpeed};
    

}

void depositBin() {

}