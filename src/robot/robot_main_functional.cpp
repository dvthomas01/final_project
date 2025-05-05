/*
 * robot_main.cpp – streamlined version (fixed)
 * Keeps original behaviour while clarifying control flow and eliminating
 * duplicated flags.  Adds a proper CommandCtx constructor so the compiler
 * is happy with CommandCtx(...).
 */

 #include <Arduino.h>
 #include <Adafruit_BNO08x.h>
 #include "imu.h"
 #include "robot_drive.h"
 #include "robot_motion_control.h"
 #include "RobotFunctions.h"   // brings in the `commands` enum
 #include "robot_pinout.h"
 #include "wireless.h"
 #include "util.h"
 
 HardwareSerial mySerial(1);               // UART1: RX = 44, TX = 43
 
 // ──────────────────────────────────────────────────────────────
 //  Small helper that tracks everything we need while a command
 //  is executing. We give it an explicit constructor so the line
 //  `CommandCtx(out.COMMAND, out.INPUT_VAL, ypr.yaw)` compiles.
 // ──────────────────────────────────────────────────────────────
 struct CommandCtx {
    commands cmd;      // what we’re doing (ROTATE_CW, ALIGN, …)
    int       arg;     // extra integer argument from Jetson
    float     yaw0;    // yaw where the command started
    bool      done;    // set true once the command completes

    CommandCtx() : cmd(NO_STATE_DETECTED), arg(0), yaw0(0), done(true) {}
    CommandCtx(commands c, int a, float y) : cmd(c), arg(a), yaw0(y), done(false) {}
 };
 
 static CommandCtx ctx;           // current command context
 
 // Shared IMU data from imu_reader.cpp
 extern struct euler_t { float yaw, pitch, roll; } ypr;

// Wireless Comms Variables
extern bool freshWirelessData;
extern ControllerMessage controllerMessage;
extern RobotMessage robotMessage;
 
 // ──────────────────────────────────────────────────────────────
 //  Helper‑functions – keep main loop skinny
 // ──────────────────────────────────────────────────────────────

 static void holdStill() {
    updateDriveSetpoints(0, 0);
    updatePIDs();
 }
 
 static void fetchJetsonCommand() {
    jetsonOutput out = jetsonComms();  // blocks ~5 ms
    if (out.COMMAND == FINISH) {
        ctx = CommandCtx(FINISH, 0, ypr.yaw);
        return;
    }
    if (out.COMMAND != NO_STATE_DETECTED) {
        ctx = CommandCtx(out.COMMAND, out.INPUT_VAL, ypr.yaw);
    }
 }
 
 static void executeCommand() {
    switch (ctx.cmd) {
        case ROTATE_CCW:
            //Serial.println("CCW Running");
            rotate(ctx.yaw0, ypr.yaw, ctx.arg);
            break;
        case ROTATE_CW:
            Serial.println("CW Running");
            // rotate(ctx.yaw0, ypr.yaw, ctx.arg);
            break;
        case APPROACH_PICKUP_POSE: 
            Serial.println("Driving Straight");
            driveStraight(1);
            break;
        case ALIGN_F: 
            Serial.println("Driving Straight");
            driveStraight(1);
            break;
        case ALIGN_B: 
            Serial.println("Driving Back");
            driveStraight(-1);
            break;
        case GRAB_BIN: 
            Serial.println("Grabbing Bin");
            grabBin(); 
        case GRAB_BIN: 
            Serial.println("Grabbing Bin");
            grabBin(); 
        /* TODO: plug in other command handlers here */
        case STOP:
            updateDriveSetpoints(0, 0);
            break;
        default:
            break;
     }
 }
 
 static bool commandComplete() {
    // Currently the only long‑running command is ROTATE_*, which tells us it’s
    // done by printing "ROTATE_DONE" on USB Serial.

    String line = mySerial.readStringUntil('\n');
    Serial.println("commandComplete line: " +line);
    if (line.indexOf("STOP") >= 0) {
        mySerial.println("STOP"); 
    }

    return line.indexOf("STOP") >= 0;
    /*
    if (ctx.cmd == ROTATE_CW || ctx.cmd == ROTATE_CCW) {
        String line = mySerial.readStringUntil('\n');
        if (line.indexOf("STOP") >= 0) Serial.println("commandComplete: " + line     );
        return line.indexOf("STOP") >= 0;
    }
    if (ctx.cmd == APPROACH_PICKUP_POSE) {
        String line = mySerial.readStringUntil('\n');
        return line.indexOf("STRAIGHT_DONE") >= 0; 
    }
    return false;*/
 }
 
 // ──────────────────────────────────────────────────────────────
 //  Arduino lifecycle
 // ──────────────────────────────────────────────────────────────
 void setup() {
     setupIMU();
     setupDrive();
     readIMU(false);
     setupWireless();
 
     mySerial.begin(115200, SERIAL_8N1, 44, 43); // Jetson UART
     mySerial.setTimeout(10);  // 10 ms timeout
     Serial.begin(115200);                      // USB debug
     Serial.println("ESP32‑S3 ready");
     pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP); // Set limit switch pin to pullup mode
 }
 
 void loop() {
    readIMU(false);                // Always keep yaw fresh

    static enum { WAITING, ACTIVE, FINISHED, JOYSTICK_INTERRUPT } state = WAITING;
    if(checkJoystickInterrupt()){
        state = JOYSTICK_INTERRUPT;
    }

    switch (state) {
        case WAITING:
            fetchJetsonCommand();
            if (!ctx.done && ctx.cmd != NO_STATE_DETECTED) {
                state = ACTIVE;
                Serial.println("In Waiting to Active transition");
            } 
            break;

        case ACTIVE:
            executeCommand();
            if (commandComplete()) {
                ctx = CommandCtx();       // reset to default (done=true)
                state = WAITING;
            }
            if (ctx.cmd == FINISH) {
                holdStill();
                state = FINISHED;
            }
            break;

        case FINISHED:
            holdStill();
            break;

        case JOYSTICK_INTERRUPT:
            // Joystick activated, swap to joystick mode
            readJoystick();
            break;
    }

    EVERY_N_MILLIS(5) { updatePIDs(); }
    delay(10);   // relieve the watchdog
 }
 