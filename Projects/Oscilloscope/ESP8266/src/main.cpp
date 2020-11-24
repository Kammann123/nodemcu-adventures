#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

/* Constant parameters of the program */
#define   BAUD_RATE               921600

/* Declaring the used GPIO on the board */
#define   ANALOG_CHANNEL_0        D0      // Channel 1 for digital measuring

void setup() {
  // Starts the Serial Port
  Serial.begin(BAUD_RATE);
}

void loop() {
  // Sample the analog input
  int sample = analogRead(ANALOG_CHANNEL_0);

  // Send the sample via UART to the PC                                          
  Serial.write( 0xff );                                                         
  Serial.write( (sample >> 8) & 0xff );                                            
  Serial.write( sample & 0xff );
}