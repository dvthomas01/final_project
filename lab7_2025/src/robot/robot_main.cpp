#include <Arduino.h>
#include "robot_drive.h"
#include "wireless.h"
#include "util.h"
#include "robot_motion_control.h"

void setup() {
    Serial.begin(115200);
    setupDrive();
    setupWireless();
}

void loop() {
    // Update velocity setpoints based on trajectory at 50Hz
    EVERY_N_MILLIS(20) {
        followTrajectory();
        // we want to say if buttonF = 0 update setpoints for both to turn forwards
        // if button R = 0 udpate setpoints for both to turn backwards
        //otherwise setpoints stay 0
        if(controllerMessage.debouncedInputF == 0){
            updateFlywheelSetpoints(-2,-2);
        }
        else if(controllerMessage.debouncedInputR == 0){
            updateFlywheelSetpoints(2,2);
        }
        else{
            updateFlywheelSetpoints(0,0);
        }
    }

    // Update PID at 200Hz
    EVERY_N_MILLIS(5) {
        updatePIDs();
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {
        updateOdometry();
        sendRobotData();

        Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
                    robotMessage.x, robotMessage.y, robotMessage.theta);
            }
  
}