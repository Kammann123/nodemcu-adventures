/********************************************************************************
  @file     App.c
  @brief    Application functions
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 *******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "drivers/MCAL/uart/uart.h"
#include "drivers/HAL/button/button.h"
#include "drivers/HAL/led/led.h"
#include "board.h"

#include <string.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// #define SOME_CONSTANT    20
// #define MACRO(x)         (x)

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void onSW3Pressed(void);

/*******************************************************************************
 * VARIABLES TYPES DEFINITIONS
 ******************************************************************************/

// typedef int  my_int_t;

/*******************************************************************************
 * PRIVATE VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const char*		request = "ECHO";
static const char* 		response = "ACK";
static word_t			buffer[100];

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

/* Called once at the beginning of the program */
void appInit (void)
{
	// Board initialization
	boardInit();

	// Initialization of the UART driver
	uart_cfg_t uartConfiguration = {
			.baudRate 		= UART_BAUD_RATE_115200,
			.parityEnable 	= 0,
			.parityMode		= 0,
			.stopMode		= 0,
			.length			= 0
	};
	uartInit(UART_INSTANCE_3, uartConfiguration);

	// Initialization of the Led driver
	ledInit();

	// Initialization of the Button driver
	buttonInit();
	buttonSubscribe(BUTTON_1, BUTTON_PRESS, onSW3Pressed);
}

/* Called repeatedly in an infinite loop */
void appRun (void)
{
	if (uartHasRxMsg(UART_INSTANCE_3))
	{
		uint8_t length = uartGetRxMsgLength(UART_INSTANCE_3);
		uartReadMsg(UART_INSTANCE_3, buffer, length);
		if (strcmp(buffer, response))
		{
			ledBlinkSome(LED_GREEN, LED_MS2TICK(200), 10);
		}
	}
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void onSW3Pressed(void)
{
	uartWriteMsg(UART_INSTANCE_3, (const word_t*)request, strlen(request));
}


/*******************************************************************************
 ******************************************************************************/
