#include <Arduino.h>
#include <Wire.h>

#include <stdint.h>

/***************************************/
/* DEFINE, MACROS, AND CONSTANT VALUES */
/***************************************/

#define SERIAL_BAUD_RATE        9600    // Baud rate used to log on PC monitor
#define I2C_SLAVE_ADDRESS       0x0A    // I2C address of this device

/***********************/
/* FUNCTION PROTOTYPES */
/***********************/

/**
 * @brief Callback registered to be called when the I2C driver receives data from Master
 * @param bytesReceived   Amount of bytes received from Master
 */
static void I2CReceiveHandler(int bytesReceived);

/***********************/
/* MAIN SETUP AND LOOP */
/***********************/

void setup() 
{
  // Initialize the serial peripheral
  Serial.begin(SERIAL_BAUD_RATE);

  // Initialize the I2C peripheral as slave on the given address
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(I2CReceiveHandler);

  // Starting up message...
  Serial.println("< I2C Slave Logger > Starting device... OK!");
}

void loop() 
{
  // Delay on each main loop, not doing anything
  delay(100);
}

/************************/
/* FUNCTION DEFINITIONS */
/************************/

static void I2CReceiveHandler(int bytesReceived)
{
  char byte;

  // Print in the monitor a message notifying the amount of bytes received
  Serial.print("< I2C Slave Logger > Detected ");
  Serial.print(bytesReceived);
  Serial.println(" bytes received from Master");

  // Fetch each byte from the I2C driver, and log in the monitor its content
  while (bytesReceived--)
  {
    byte = Wire.read();
    Serial.print("< I2C Slave Logger > Byte received ");
    Serial.println(byte);
  }
}