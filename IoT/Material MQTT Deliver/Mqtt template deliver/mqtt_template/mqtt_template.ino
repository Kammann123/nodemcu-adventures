/*
  Box1

  modified  August 2017
  by Jacoby Daniel
  
*/

// Board NodeMCU 0.9 (ESP-12 Module) 

#include <ESP8266WiFi.h> 
#include <PubSubClient.h>   
/***************************************
 * PubSubClient Documentation
 * https://pubsubclient.knolleary.net/
 **************************************/ 
 						
// Mqtt Stuff

// ****************************** Cloud *********************************************
#define  CLOUD_MQTT  0         //https://www.cloudmqtt.com/ (Amazon Web Services)
#define  CLOUD_DIOTY 1         //http://www.dioty.co/       (colombian Server)
#define  CLOUD_DANY  2         //At home Local network      (At home)
//==========Select cloud======================

#define CLOUD CLOUD_DANY

//============================================
#if CLOUD==CLOUD_DANY

IPAddress MqttServer(192,168,0,126);				        // MQTT server URL or IP
const unsigned int MqttPort=1883; 					        // MQTT port (default 1883)
const char MqttUser[]="itba.jacoby@gmail.com";		  // user name 
const char MqttPassword[]="password";				        // user password
const char MqttClientID[]="aname";					        // Device ID (warning: must be unique) 

#elif CLOUD==CLOUD_DIOTY

const char MqttServer[]="mqtt.dioty.co";
const unsigned int MqttPort=1883; 
const char MqttUser[]="itba.jacoby@gmail.com";
const char MqttPassword[]="password";
const char MqttClientID[]="aname";

#elif CLOUD==CLOUD_MQTT

const char MqttServer[]="m12.cloudmqtt.com";
const unsigned int MqttPort=16111; 
const char MqttUser[]="user";
const char MqttPassword[]="password";
const char MqttClientID[]="aname";

#endif

// Create Cloud sockets
WiFiClient wclient;
PubSubClient mqtt_client(wclient);
//=====================================================================
// WiFi credentials

//char ssid[] = "my_router_ssid";
//char password[] = "my_router password";   // Set password to "" for open networks


//char ssid[] = "NODE_RED";
//char password[] = "GEDA2016";   // Set password to "" for open networks.
//=====================================================================

//Board defifinitions 
 
//======================
#define HARD AMICA              // Select Board Type or External pin
//======================

#define AMICA 0
#define LOLIN 1
#define EXTERN_PIN 2
  
// NODEMCU Boards & pin defs
#define AMICA_OnBoard_LED D0    // AMICA NodeMCU OnBoadrd LED  (on board blue led same as D0 but inverted)
#define LOLIN_OnBoard_LED D4    // LOLIN NodeMCU V3 OnBoadrd LED  (on board blue led same as D4 but inverted)
#define External_LED D1         // External led on D1

// **** Pin polarity **** 
// Board
#define Board_LED_OFF  1
#define Board_LED_ON   0
// External
#define External_LED_OFF  0
#define External_LED_ON   1
// **********************//
// Select here the LED pin to be used: Board Build In led or External Pin//
#if HARD == AMICA
#define MYLED   AMICA_OnBoard_LED        // User LED
#define LED_ON  Board_LED_ON
#define LED_OFF Board_LED_OFF

#elif HARD == LOLIN
#define MYLED   LOLIN_OnBoard_LED        // User LED
#define LED_ON  Board_LED_ON
#define LED_OFF Board_LED_OFF

#elif HARD ==  EXTERN_PIN
#define MYLED   External_LED        // User LED
#define LED_ON  External_LED_ON
#define LED_OFF External_LED_OFF
#endif



/////////////////////////////////////////////
#define OFF  LOW
#define ON HIGH


#define MQTT_STS  D6 
#define STATUS_LED MYLED //Wifi - broker connection status indicator

// Status Led (blue)
// If it blinks fast: Unable to connect to Wifi network
// If it blinks every 2 seconds: Unable to find Broker 
// If it Stays on device ready to operate  


// *************************************************
//
// 				Print debug messages macro
//
// *************************************************
#define DEBUG_OFF  0
#define DEBUG_ON   1

#define debug DEBUG_ON

#define debug_message(fmt,...)          \
  do {              \
    if (debug)          \
       Serial.printf (fmt, ##__VA_ARGS__);     \
  } while(0)

//==================================================

void setup_wifi(void);    
void setup_mqtt(void);


long TimeSinceLastMeasure = 0;
uint32_t Ldr_delayMS=1000;  


unsigned char ldr_enable=true;
unsigned char count_ldr=0;


// the setup function runs once when you press reset or power the board
void setup() {

  Serial.begin(9600, SERIAL_8N1);  // Serial Port Parameters

// Setup Digital Pins
  pinMode(STATUS_LED , OUTPUT);   //Wifi & MQTT server Status led 
       
 
// Pins initial values
  
  digitalWrite(STATUS_LED,LED_OFF);
  

// Setup Wifi and MQTT  
  delay(1000);
  setup_wifi();          // initialize WIFI an connect to network
  setup_mqtt();          // initialize mqtt server 

  
}

// the loop function runs over and over again forever
void loop() {

// Check MQTT conection is still active

  if (!mqtt_client.connected()) {
      reconnect();
      
 }

// This should be called regularly to allow the client 
// to process incoming messages and maintain
// its connection to the server

  mqtt_client.loop();  
  


// ADC Sample time.
 
  if (millis() - TimeSinceLastMeasure > Ldr_delayMS) {  
         
      debug_message("ADC %d\n",analogRead(A0));

      if(ldr_enable==true)   
     // mqtt_client.publish("light_box/ldrvalue",String(analogRead(A0)).c_str(),false); //(Publish ADC)
        mqtt_client.publish("light_box/ldrvalue",String(count_ldr++).c_str(),false);       //(Simulate ADC)
      TimeSinceLastMeasure = millis();
  }
  
 
} // End of main Loop

//=======================SET UP WiFi==================================================================
void setup_wifi(void) {

// Start connecting to a WiFi network
 
  debug_message("\n\nConnecting to %s\n",ssid);
  
  WiFi.softAPdisconnect (true);   // Shut Down SoftAP
  WiFi.disconnect();    		  // If warm reset (not power down esp) ensure to signal router that old conection has finished
  WiFi.begin(ssid, password);     // Connect to WiFi network
  
  // Wait until WiFi conection is done
  while (WiFi.status() != WL_CONNECTED) { 
    
    digitalWrite(STATUS_LED,LED_OFF);
    delay(250);
    debug_message(".");
    digitalWrite(STATUS_LED,LED_ON);
    delay(250);
  }

  digitalWrite(STATUS_LED,LED_OFF);

  debug_message("\nWiFi Connected!!  IP Address:%s \n\n",WiFi.localIP().toString().c_str() );

  
}

//=======================SET UP MQTT==================================================================
void setup_mqtt(void) {
  
//http://pubsubclient.knolleary.net/api.html   (Arduino mqtt API)

 mqtt_client.setServer(MqttServer, MqttPort);
 mqtt_client.setCallback(callback);
}


void callback(char* topic, byte* payload, unsigned int length) {

 payload[length]=0; // Set terminator
  
 debug_message("Message arrived [ Topic:%s Length:%d Payload: %s ] \n",topic,length,payload);

 ParseTopic(topic,payload,length);

}



// ============================== MQTT PARSER ==================================================

void ParseTopic(char* topic, byte* payload, unsigned int length)
{

  if(!strcmp(topic,"light_box/ldr_enable"))  
  {
      if(payload[0]=='0')
      {
            ldr_enable=false;
      }
      
      
      if(payload[0]=='1')
      {
            ldr_enable=true;
      }
  
  }

  if(!strcmp(topic,"light_box/Set_ldr_Delay"))  
  {

          Ldr_delayMS=atoi((char*)payload);
          debug_message("Reading interval: %d\n",Ldr_delayMS);

  }

        
}

void reconnect() {

  while (!mqtt_client.connected())      // Loop until we're reconnected
  {
      debug_message("Attempting MQTT connection...");
   
	    // Try to connect to broker
      if (mqtt_client.connect(MqttClientID,MqttUser,MqttPassword))
      {
            debug_message("Connected \r\n"); 
			      digitalWrite(STATUS_LED,LED_ON);
  
            // ... and subscribe to topics
           
            mqtt_client.subscribe("light_box/Set_ldr_Delay");
            mqtt_client.subscribe("light_box/ldr_enable");
      }
      else
      {
		   // If conection is not established just make a short blink
      
            debug_message("failed, rc=");       //For rc codes see https://pubsubclient.knolleary.net/api.html#state
            debug_message("%d",mqtt_client.state());
            debug_message(" try again in 2 seconds \r\n");
        
            delay(2000);  //Wait 2 seconds before retrying
            digitalWrite(STATUS_LED,LED_ON);
            delay(200);  //Short 200 ms blink
            digitalWrite(STATUS_LED,LED_OFF);
        
      }
      
  } // end of while
 
}// End reconnect





