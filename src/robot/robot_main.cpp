#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"

void setup() {
    
    setupIMU();
    setupDrive();

    rotate(1);
}

void loop() {
    // Send and print robot values at 20Hz

    /*
    EVERY_N_MILLIS(10000) {
        straightline();
        Serial.println("Hi");
    }
    */

    // EVERY_N_MILLIS(10000) {
    //     straightline();
    //     Serial.println("Hi");
    // }

    // Update PID at 200Hz
    EVERY_N_MILLIS(1000) {
       updatePIDs();
    }

    // Send and print robot values at 20Hz
    EVERY_N_MILLIS(50) {
        /* updateOdometry();
    /*
        sendRobotData();

        Serial.printf("x: %.2f, y: %.2f, theta: %.2f\n",
                    robotMessage.x, robotMessage.y, robotMessage.theta);
    */
    }
}