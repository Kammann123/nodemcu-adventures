#include <Arduino.h>

#include "ESP8266WiFi.h"

/*******************************
 * CONSTANT VALUES AND DEFINES *
 ******************************/

#define SERIAL_BAUD_RATE    9600

/*******************************
 * STATIC FILE SCOPE VARIABLES *
 ******************************/

const char* ssid = "Fibertel WiFi568 2.4GHz";
const char* password = "0103052182";

void setup() 
{
  // Initialization of the serial communication driver
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialization of the WiFi driver
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(100);
  }
  
  // Log a message in the Serial monitor
  Serial.println("WiFi connection established!");
  Serial.println(WiFi.localIP());
}

void loop() 
{
  // put your main code here, to run repeatedly:
}