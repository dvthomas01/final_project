#include <Arduino.h>
#include <Adafruit_BNO08x.h>
#include "imu.h"
#include "robot_drive.h"
#include "robot_motion_control.h"
#include "RobotFunctions.h"

HardwareSerial mySerial(1); // Use UART1

void setup() {
  mySerial.begin(115200, SERIAL_8N1, 44, 43); // RX=44, TX=43
  Serial.begin(115200); // Optional: USB debug output
  Serial.println("Debug serial ready");
  mySerial.println("ESP32-S3 UART1 ready");
}

void loop() {
    String input = mySerial.readStringUntil('\n');
    input.trim(); // Remove whitespace/newline
    Serial.println("Received on UART1: " + input); // USB debug

    if (input == "HELLO") {
      mySerial.println("ACK HELLO");
    } else {
      mySerial.println("UNKNOWN CMD");
    }
    delay(10); // Avoid flooding CPU
}
