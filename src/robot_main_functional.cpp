#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"

HardwareSerial mySerial(1); // UART1: RX = 44, TX = 43

// Track current command and yaw state
int currentCommand = NO_STATE_DETECTED;
int currentCommandInputVal = 0;
float initialYaw = 0;
float currentYaw = 0;
bool commandComplete = true;

// Shared IMU data
extern struct euler_t {
    float yaw;
    float pitch;
    float roll;
} ypr;

void setup() {
    setupIMU();
    setupDrive();
    readIMU(false);

    mySerial.begin(115200, SERIAL_8N1, 44, 43); // Jetson UART
    Serial.begin(115200);                      // USB debug
    Serial.println("ESP32-S3 ready");
    mySerial.println("ESP32-S3 UART1 ready");
}

void loop() {
    // 1. Always update IMU
    readIMU(false);
    currentYaw = ypr.yaw;

    // 2. If no command running, check for new input
    static bool isFinished = false;
    if (!isFinished) {
        if (commandComplete) {
            jetsonOutput output = jetsonComms();  // blocks ~5 ms max

            if (output.COMMAND == FINISH) {
                Serial.println("Received FINISH — stopping robot.");
                updateDriveSetpoints(0, 0);
                updatePIDs();
                isFinished = true;
                return; // Skip everything else
            }


            if (output.COMMAND != NO_STATE_DETECTED) {
                currentCommand = output.COMMAND;
                currentCommandInputVal = output.INPUT_VAL;
                initialYaw = currentYaw;
                commandComplete = false;
                Serial.print("Starting command: ");
                Serial.println(currentCommand);
            }

        }



        // 3. Handle the active command
        switch (currentCommand) {
            case ROTATE_CW:
                rotate(initialYaw, currentYaw, currentCommandInputVal);
                // ROTATE_DONE gets printed inside rotate() when done
            break;
            case ROTATE_CCW:
            case SETUP:
            case ALIGN:
            case FINE_ALIGN:
            case APPROACH_PICKUP_POSE:
            case GRAB_BIN:
            case FIXED_BACKUP:
            case DEPOSIT_BIN:
            case BACKUP:
            case S_MANEUVER_ALIGN:
            case DRIVE_UP_RAMP:
            case DRIVE_DOWN_RAMP:
            case COLOR_DETECT_STORE:
            case DRIVE_TO_PICKUP:
                // Add other commands here later
                break;

            default:
                break;
        }

        // 4. Check if rotate is done (or other future conditions)
        if (currentCommand == ROTATE_CW || currentCommand == ROTATE_CCW) {
            // ROTATE_DONE printed = signal from rotate() means we’re done
            // We’ll reset command on next loop after it's printed
            static bool lastLoopSawRotateDone = false;
            if (lastLoopSawRotateDone) {
                currentCommand = NO_STATE_DETECTED;
                commandComplete = true;
                lastLoopSawRotateDone = false;
            } else if (Serial.available()) {
                String line = Serial.readStringUntil('\n');
                if (line.indexOf("ROTATE_DONE") >= 0) {
                    lastLoopSawRotateDone = true;
                }
            }
        }
    } else { //already finished stay stopped
        updateDriveSetpoints(0, 0);
        updatePIDs();
        delay(10);
        return;
    }
    // 5. PID update
    EVERY_N_MILLIS(1000) {
        updatePIDs();
    }

    delay(10); // prevent tight loop burn
}
