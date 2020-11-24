#include <Arduino.h>

#include "ESP8266WiFi.h"
#include "PubSubClient.h"

/*******************************
 * CONSTANT VALUES AND DEFINES * 
 ******************************/

#define SERIAL_BAUD_RATE        9600
#define SCAN_PERIOD_MS          1000

/***********************************
 * FUNCTION PROPOTYPES DECLARATION * 
 **********************************/

void messageCallback(char* topic, byte* payload, unsigned int length);

/************************
 * FILE SCOPE VARIABLES * 
 ***********************/

const char*   wifiSsid = "Fibertel WiFi568 2.4GHz";
const char*   wifiPassword = "0103052182";
IPAddress     ip(192, 168, 0, 223);         // IP Address of the MQTT Server
uint16_t       port = 1883;                 // Port of the MQTT Server

const char*   mqttUser = "lucaskammann";    // User for the MQTT connection
const char*   mqttPassword = "password";    // Password for the MQTT connection
const char*   mqttClientID = "someid";      // ID to identify as unique user

PubSubClient  mqttClient;                   // Instance of the MQTT Client
WiFiClient    wifiClient;                   // Instance of the WiFi Client

uint32_t      lastMillis;
uint32_t      currentMillis;

void setup() 
{
  // Initialize the serial communication driver
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize the MQTT client
  mqttClient.setServer(ip, port);
  mqttClient.setCallback(messageCallback);
  mqttClient.setClient(wifiClient);

  // Initialize the connection to WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  WiFi.begin(wifiSsid, wifiPassword);
  Serial.println("< MQTT Client > Trying to connect to WiFi network...");
  while ( WiFi.status() != WL_CONNECTED )
  {
    delay(100);
  }
  Serial.println("< MQTT Client > WiFi connection established!");
}

void loop() 
{
  // When the MQTT Client is not connected, try to connect using the
  // client id, and user credentials corresponding...
  while ( !mqttClient.connected() )
  {
    Serial.println("< MQTT Client > Trying to connect to MQTT Broker");
    if (mqttClient.connect(mqttClientID, mqttUser, mqttPassword))
    {
      Serial.println("< MQTT Client > Connection established to the MQTT Broker");
      mqttClient.subscribe("esp/something");
    }
  }

  // Periodically publish a message to the MQTT broker
  currentMillis = millis();
  if (currentMillis - lastMillis >= SCAN_PERIOD_MS)
  {
    lastMillis = currentMillis;
    mqttClient.publish("esp/hello", "Hello World!");
  }

  // Run the loop of the MQTT client
  mqttClient.loop();
}

/***********************
 * FUNCTION DEFINITION * 
 **********************/

void messageCallback(char* topic, byte* payload, unsigned int length)
{
  // Set the terminator of the payload
  payload[length] = 0;
  Serial.printf("< MQTT Client > Topic: %s Payload: %s Length: %d", topic, payload, length);
}