#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

#define BOUNCE_PIN1 8
#define BOUNCE_PIN2 6
#define BOUNCE_PIN3 12

Bounce bounceF = Bounce(); //button to spin flywheels forward
Bounce bounceR = Bounce(); //button to spin flywheels reverse
Bounce bounceI = Bounce(); //button to interrupt autonomy with joystick control

int dif = 1; //forward
int dir = 1; //reverse
int dii = 1; //interrupt


ControllerMessage prevControllerMessage;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
//Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

void setup() {
    Serial.begin(115200);

    setupWireless();

    joystick1.setup();

    bounceF.attach(BOUNCE_PIN1, INPUT_PULLUP);
    bounceR.attach(BOUNCE_PIN2, INPUT_PULLUP);
    bounceI.attach(BOUNCE_PIN3, INPUT_PULLUP);

    Serial.println("Setup complete.");
}

void loop() {
    // Read and send controller sensors
    EVERY_N_MILLIS(20) {

        bounceF.update();
        bounceR.update();
        bounceI.update();

        dif = bounceF.read();
        dir = bounceR.read();
        dii = bounceI.read();

        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read();
        controllerMessage.debouncedInputF = dif;
        controllerMessage.debouncedInputR = dir;
        controllerMessage.debouncedInterrupt = dii;
        
        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}