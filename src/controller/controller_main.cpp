#include <Bounce2.h>
#include "wireless.h"
#include "util.h"
#include "joystick.h"
#include "dpad.h"
#include "display.h"
#include "controller_pinout.h"

#define BOUNCE_PIN1 8
#define BOUNCE_PIN2 6

Bounce bounceF = Bounce();
Bounce bounceR = Bounce();

int dif = 1;
int dir = 1;


ControllerMessage prevControllerMessage;

Joystick joystick1(JOYSTICK1_X_PIN, JOYSTICK1_Y_PIN);
//Joystick joystick2(JOYSTICK2_X_PIN, JOYSTICK2_Y_PIN);

void setup() {
    Serial.begin(115200);

    setupWireless();

    joystick1.setup();

    bounceF.attach(BOUNCE_PIN1, INPUT_PULLUP);
    bounceR.attach(BOUNCE_PIN2, INPUT_PULLUP);

    Serial.println("Setup complete.");
}

void loop() {
    // Read and send controller sensors
    EVERY_N_MILLIS(20) {

        bounceF.update();
        bounceR.update();

        dif = bounceF.read();
        dir = bounceR.read();

        controllerMessage.millis = millis();
        controllerMessage.joystick1 = joystick1.read();
        controllerMessage.debouncedInputF = dif;
        controllerMessage.debouncedInputR = dir;
        
        if (!(prevControllerMessage == controllerMessage)) {
            sendControllerData();
            prevControllerMessage = controllerMessage;
        }
    }
}