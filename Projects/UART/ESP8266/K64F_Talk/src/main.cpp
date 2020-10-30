#include  <Arduino.h>

#include <stdint.h>

/* CONSTANT AND DEFINE VALUES */
#define  PIN_LED   D0

/* STATIC AND GLOBAL VARIABLES */
uint8_t   counter;
String    message;

void setup() {
  // Initialization of pin led
  pinMode(PIN_LED, OUTPUT);

  // Initialization of serial port
  Serial.begin(115200);
}

void loop() {
  if ( Serial.available() > 0)
  {
    // Copy the response
    message = Serial.readString();

    if (message.compareTo("ECHO"))
    {
      digitalWrite(PIN_LED, HIGH);
      delay(300);
      digitalWrite(PIN_LED, LOW);
      
      // Send the response to the Debug Monitor
      Serial.write("ACK");
    }
  }
}