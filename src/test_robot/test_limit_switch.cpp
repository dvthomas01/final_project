#include <Adafruit_MCP23X17.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

//jetson nano pw admin

//#define LED_PIN 0     // MCP23XXX pin LED is attached to
//#define BUTTON_PIN 2  // MCP23XXX pin button is attached to


Adafruit_MCP23X17 mcp;

void setup() {
  Serial.begin(9600);
  //while (!Serial);
  Serial.println("MCP23017 Test!");

  // uncomment appropriate mcp.begin
  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }

  // configure LED pin for output
  //mcp.pinMode(LED_PIN, OUTPUT);

  // configure button pin for input with pull up
  //mcp.pinMode(BUTTON_PIN, INPUT);
  pinMode(5,INPUT_PULLUP);

  Serial.println("Looping...");
}

void loop() {
  //mcp.digitalWrite(LED_PIN, !digitalRead(5));
  Serial.println(digitalRead(5));
  delay(100);
}
