/*******************************************************************************
  @file     uart.h
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature.
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef MCAL_UART_UART_H_
#define MCAL_UART_UART_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the callback type
typedef	void	(*uart_tx_callback)(void);
typedef	void	(*uart_rx_callback)(void);

// Declaring UART instance id
typedef enum {
	UART_INSTANCE_0,
	UART_INSTANCE_1,
	UART_INSTANCE_2,
	UART_INSTANCE_3,
	UART_INSTANCE_4,
	UART_AMOUNT
} uart_id_t;

// Declaring UART baud rate standards
typedef enum {
	UART_BAUD_RATE_110 		= 110,
	UART_BAUD_RATE_300 		= 300,
	UART_BAUD_RATE_600 		= 600,
	UART_BAUD_RATE_1200 	= 1200,
	UART_BAUD_RATE_2400 	= 2400,
	UART_BAUD_RATE_4800 	= 4800,
	UART_BAUD_RATE_9600 	= 9600,
	UART_BAUD_RATE_14400 	= 14400,
	UART_BAUD_RATE_19200 	= 19200,
	UART_BAUD_RATE_38400 	= 38400,
	UART_BAUD_RATE_57600 	= 57600,
	UART_BAUD_RATE_115200 	= 115200,
	UART_BAUD_RATE_128000 	= 128000,
	UART_BAUD_RATE_256000 	= 256000
} uart_baudrate_t;

// Declaring UART parity modes
typedef enum {
	UART_PARITY_EVEN,
	UART_PARITY_ODD
} uart_parity_mode_t;

// Declaring UART stop modes
typedef enum {
	UART_STOP_SINGLE_BIT,
	UART_STOP_TWO_BITS
} uart_stop_mode_t;

// Declaring UART length available
typedef enum {
	UART_DATA_8_BITS,
	UART_DATA_9_BITS
} uart_length_t;

// Declaring UART configuration
typedef struct {
	uart_baudrate_t		baudRate;
	uint8_t				parityEnable : 1;	// 0 for parity disabled
	uint8_t				parityMode   : 1;	// 0 for even parity
	uint8_t				stopMode	 : 1;	// 0 for 1 stop bit
	uint8_t				length		 : 1;	// 0 for 8 data bits
} uart_cfg_t;

// Declaring the size of the words
typedef uint8_t word_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize UART driver
 * @param id 		UART's number
 * @param config 	UART's configuration (baud rate, parity, etc.)
*/
void uartInit (uart_id_t id, uart_cfg_t config);

/**
 * @brief Subscribes a callback to be called when the transmission is completed.
 * @param callback	Callback to be called when the event is triggered
 */
bool uartSubscribeTxMsgComplete(uart_id_t id, uart_tx_callback callback);

/**
 * @brief Subscribes a callback to be called when a frame of the given size is received
 * @param callback	Callback to be called when the event is triggered
 * @param frameSize	Size in bytes of the frame to trigger the event
 */
bool uartSubscribeRxMsg(uart_id_t id, uart_rx_callback callback, uint8_t frameSize);

/**
 * @brief Check if a new word was received
 * @param id 		UART's number
 * @return true if received a message or false otherwise
*/
bool uartHasRxMsg(uart_id_t id);

/**
 * @brief Check if all bytes were transfered
 * @param id 		UART's number
 * @return All bytes were transfered
*/
bool uartIsTxMsgComplete(uart_id_t id);

/**
 * @brief Check how many words were received
 * @param id 		UART's number
 * @return Quantity of received words
*/
uint8_t uartGetRxMsgLength(uart_id_t id);

/**
 * @brief Read a received message. Non-Blocking
 * @param id		UART's number
 * @param msg 		Buffer to paste the received bytes
 * @param length 	Desired quantity of bytes to be pasted
 * @return Real quantity of pasted bytes
*/
uint8_t uartReadMsg(uart_id_t id, word_t* msg, uint8_t length);

/**
 * @brief Write a message to be transmitted. Non-Blocking
 * @param id 		UART's number
 * @param msg 		Buffer with the bytes to be transfered
 * @param length 	Desired quantity of bytes to be transfered
 * @return Real quantity of bytes to be transfered
*/
uint8_t uartWriteMsg(uart_id_t id, const word_t* msg, uint8_t length);

/*******************************************************************************
 ******************************************************************************/


#endif /* MCAL_UART_UART_H_ */
