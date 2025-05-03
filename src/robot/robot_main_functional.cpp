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
         Serial.print("Starting command: ");
         Serial.println(ctx.cmd);
     }
 }
 
 static void executeCommand() {
     switch (ctx.cmd) {
         case ROTATE_CCW:
            break;
         case ROTATE_CW:
            Serial.println("CW Running");
            rotate(ctx.yaw0, ypr.yaw, ctx.arg);
            break;
         /* TODO: plug in other command handlers here */
         case APPROACH_PICKUP_POSE: 
            driveStraight(1);
            break;
         default:
             break;
     }
 }
 
 static bool commandComplete() {
     // Currently the only long‑running command is ROTATE_*, which tells us it’s
     // done by printing "ROTATE_DONE" on USB Serial.
     if (ctx.cmd == ROTATE_CW || ctx.cmd == ROTATE_CCW) {
        String line = mySerial.readStringUntil('\n');
        return line.indexOf("ROTATE_DONE") >= 0;
     }
     if (ctx.cmd == APPROACH_PICKUP_POSE) {
        String line = mySerial.readStringUntil('\n');
        return line.indexOf("STOP") >=0; 
     }
     return false;
 }
 
 // ──────────────────────────────────────────────────────────────
 //  Arduino lifecycle
 // ──────────────────────────────────────────────────────────────
 void setup() {
     setupIMU();
     setupDrive();
     readIMU(false);
 
     mySerial.begin(115200, SERIAL_8N1, 44, 43); // Jetson UART
     Serial.begin(115200);                      // USB debug
     Serial.println("ESP32‑S3 ready");
 }
 
 void loop() {
     readIMU(false);                // Always keep yaw fresh
 
     static enum { WAITING, ACTIVE, FINISHED } state = WAITING;
 
     switch (state) {
         case WAITING:
             fetchJetsonCommand();
             if (!ctx.done && ctx.cmd != NO_STATE_DETECTED) state = ACTIVE;
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
     }
 
     EVERY_N_MILLIS(5) { updatePIDs(); }
     delay(10);   // relieve the watchdog
 }
 