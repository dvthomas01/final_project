#ifndef WIRELESS_H
#define WIRELESS_H

#include <esp_now.h>
#include "joystick.h"
#include "dpad.h" 
#include "display.h"

const uint8_t controllerAddr[] = {0xF4, 0x12, 0xFA, 0x40, 0x9C, 0x54};
const uint8_t robotAddr[] = {0xEC, 0xDA, 0x3B, 0x5C, 0x89, 0xD8}; 


struct ControllerMessage { //This struct is defined for a complex controller, but in lab 7 we only use a single joystick, so values are only written to joystick1.
    unsigned long millis;
    JoystickReading joystick1;
    /*JoystickReading joystick2;
    DPadReading dPad;
    bool buttonL;
    bool buttonR;
    TouchReading touchPoint;*/
    int debouncedInputF = 1; //open = 1, closed = 0
    int debouncedInputR = 1;
    int debouncedInterrupt = 1;

    void print();
    bool operator==(const ControllerMessage& other);
} ;

struct RobotMessage {
    unsigned long millis;
    float x;
    float y;
    float theta;

    void print();
    bool operator==(const RobotMessage& other);
} ;

void onSendData(const uint8_t * mac, esp_now_send_status_t status);
void onRecvData(const uint8_t * mac, const uint8_t *data, int len);
void setupWireless();
bool sendControllerData();
bool sendRobotData();

extern const uint8_t * peerAddr;
extern esp_now_peer_info_t peerInfo;

extern bool freshWirelessData;
extern ControllerMessage controllerMessage;
extern RobotMessage robotMessage;

#endif // WIRELESS_H
