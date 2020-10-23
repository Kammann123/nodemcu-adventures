#include <Arduino.h>

#define   PIN_LED     D2

static uint32_t count = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_LED, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PIN_LED, count);
  delay(10);
  count += 10;
  if (count >= 1023)
  {
    count = 0;
  }
}