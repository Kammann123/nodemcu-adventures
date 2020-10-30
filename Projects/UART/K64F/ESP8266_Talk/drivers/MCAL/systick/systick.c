/*******************************************************************************
  @file     systick.h
  @brief    Systick simple timer driver
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "SysTick.h"
#include "MK64F12.h"
#include "core_cm4.h"
#include "hardware.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define MAX_AMOUT_OF_SUBSCRIBERS	5

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

__ISR__ SysTick_Handler(void);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static void 	(*drivers[MAX_AMOUT_OF_SUBSCRIBERS])(void);	// Each driver function added to receive systick ticks
static uint8_t 	subscribers = 0;							// Amount of functions subscribed to ticks

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/


/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

bool systickInit (void (*funcallback)(void))
{
	// Computing the tick amount with the frequency
	uint32_t ticks = CPU_FREQUENCY_HZ / SYSTICK_ISR_FREQUENCY_HZ;
	bool succeed = true;

	static bool alreadyInit = false;

	// Verifying if there are subscribers available
	if (subscribers < MAX_AMOUT_OF_SUBSCRIBERS)
	{
		// Setting up the SysTick peripheral
		if (!alreadyInit)
		{
			SysTick->CTRL = 0x00;
			NVIC_EnableIRQ(SysTick_IRQn);
			succeed = (bool)SysTick_Config(ticks);
			alreadyInit = true;
		}

		// Adding the driver callback
		drivers[subscribers++] = funcallback;
	}
	else
	{
		succeed = false;
	}

	// Return status of the initialization
	return succeed;
}

/*******************************************************************************
 *******************************************************************************
						INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

__ISR__ SysTick_Handler(void)
{
	uint8_t index;
	for (index = 0 ; index < subscribers ; index++)
	{
		if (drivers[index])
		{
			drivers[index]();
		}
	}
}

/******************************************************************************/