#include <Arduino.h>

#include "ESP8266WiFi.h"

#include <stdint.h>
#include <string.h>

/*******************************
 * CONSTANT VALUES AND DEFINES *
 ******************************/

#define SERIAL_BAUD_RATE      9600
#define HARDWARE_INIT_DELAY   100
#define SCAN_PERIOD_MS        5000

/**********************************
 * STATIC VARIABLES AT FILE SCOPE *
 *********************************/

uint32_t  lastMillis;
uint32_t  currentMillis;
int32_t   networkCount;

void setup()
{
  // Initialize the serial communication driver
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize the wifi communication driver as Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  // Initial delay to wait for correct initialization of hardware
  delay(HARDWARE_INIT_DELAY);
}

void loop() 
{
  // Every SCAN_PERIOD_MS starts a process of scanning the WiFi networks
  currentMillis = millis();
  if (currentMillis - lastMillis > SCAN_PERIOD_MS)
  {
    lastMillis = currentMillis;
    WiFi.scanNetworks(true);
    Serial.println("<WiFi Scanner> Scan starting...");
  }

  // When the network scan has been completed, show them via the serial monitor
  networkCount = WiFi.scanComplete();
  if (networkCount >= 0)
  {
    Serial.println("<WiFi Scanner> Networks found!");
    for (uint8_t i = 0 ; i < networkCount ; i++)
    {
      Serial.print("<WiFi Scanner> WiFi SSID ");
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.println(WiFi.SSID(i));
    }
    WiFi.scanDelete();
  }
}