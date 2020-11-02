#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

/* Constant parameters of the program */
#define   BAUD_RATE               115200
#define   SAMPLING_TIME_MICROS    (int)(1000000.0/BAUD_RATE)

/* Declaring the used GPIO on the board */
#define   DIGITAL_CHANNEL_1        D0      // Channel 1 for digital measuring

/* Static variables */
uint64_t  lastTime;

uint8_t   currentBuffer;    // The byte buffer has eight samples
uint8_t   currentSample;    // The current sample in the eight sample buffer

uint8_t   sendBuffer;
bool      hasBuffer;

void setup() {
  // Starts the Serial Port on 115200 baudrate
  Serial.begin(BAUD_RATE);

  // Configures the channel gpio
  pinMode(DIGITAL_CHANNEL_1, INPUT);

  // Initialize variables
  lastTime = micros();
  currentBuffer = 0x00;
  currentSample = 0x00;
  hasBuffer = false;
}

void loop() {
  // Every loop verify if the sampling rate has reached
  if ((micros() - lastTime) >= SAMPLING_TIME_MICROS)
  {
    // Update the last time
    lastTime = micros();

    // Sample the digital input
    currentBuffer = currentBuffer << 1;
    currentBuffer = currentBuffer | (digitalRead(DIGITAL_CHANNEL_1) == HIGH ? 0x01 : 0x00);
    currentSample++;

    // When completed, load the send buffer
    if (currentSample == 8)
    {
      sendBuffer = currentBuffer;
      currentBuffer = 0x00;
      currentSample = 0x00;
      hasBuffer = true;
    }
  }

  // Every loop verify if there is a pending byte to send
  if (hasBuffer && Serial.availableForWrite())
  {
    hasBuffer = false;
    Serial.write(sendBuffer);
  }
}