/*******************************************************************************
  @file     led.c
  @brief    LED Driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "led.h"
#include "board.h"

#include "drivers/MCAL/gpio/gpio.h"
#include "drivers/MCAL/systick/systick.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// ¡IMPORTANT USE WARNING!
// The led driver supports two operation modes, user may define which
// will be used by applying the corresponding define.
//
// * LED_DRIVER_LIGHT_MODE only allows simple Set, Clear, Toggle and Status
// control services, and disables all other complex functionalities
// that use periodic interruption services. Use this mode when you don't need
// blinking, burst, one shot functionalities and want to avoid having the led driver
// PISR CPU load.
// * LED_DRIVER_ADVANCED_MODE deploys complete functionalities of the led driver,
// using the PISR.
//
// #define LED_DRIVER_LIGHT_MODE
#define LED_DRIVER_ADVANCED_MODE

#if !defined(LED_DRIVER_LIGHT_MODE) && !defined(LED_DRIVER_ADVANCED_MODE)
	#error	Need to define the operation mode of the driver.
#endif

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaration of led modes, used to control the current data context
typedef enum {
	STATIC,			// Led is on or off, does not need continuous control of the ISR
	BLINK,			// Blinking mode
	BLINK_SOME,		// Blinking mode during some specified amount of blink periods
	BURST,			// Burst mode
	ONE_SHOT,		// One shot mode

	LED_MODE_COUNT
} led_mode_t;

// Declaration of variables needed to run blinking mode
typedef struct {
	uint32_t	period;			// Blinking period
	uint32_t	counter;		// Blinking current count value
} blink_context_t;

// Declaration of variables needed to run blinking some mode
typedef struct {
	uint32_t	period;			// Blinking period
	uint32_t	counter;		// Blinking current count value
	uint8_t		changes;		// Amount of changes before finish (changes = 2 * blinks)
} blink_some_context_t;

// Declaration of variables needed to run burst mode
typedef struct {
	uint32_t	burstPeriod;	// Period of the burst
	uint32_t	blinkPeriod;	// Period of the high speed blinking
	uint32_t	burstCounter;	// Burst current count value
	uint32_t	blinkCounter;	// Blink current count value
	uint8_t		blinks;			// Amount of blinks for each burst period
	uint8_t		changes;		// Remaining changes ( changes = 2 * blinks )
} burst_context_t;

// Declaration of variables needed to run one shot mode
typedef struct {
	uint32_t	counter;		// Current value of remaining ticks before finishing
} one_shot_context_t;

// Declaration of context data structure, use the same memory for all context types
typedef union {
	blink_context_t			blink;		// Blink mode context
	blink_some_context_t	blinkSome;	// BlinkSome mode context
	burst_context_t			burst;		// Burst mode context
	one_shot_context_t		oneShot;	// OneShot mode context
} context_t;

// Declaration of the led data structure
typedef struct {
	pin_t		pin;		// The pin assigned to the led
	bool		active;		// ¿The led is active on high?
	led_mode_t	mode;		// The current led mode
	bool		enabled;	// Stops interruption control while settings are being changed by user
	context_t	context;	// Data context to run each led mode
} led_t;

// Declaration of the led function process control
typedef void	(*process)(led_t* led);

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Driver periodic service routine used to control status of all the
 * 	      led registered, should run periodically over the specified base time
 * 	      in the .h file.
 */
static void ledISR(void);

/*******************************************************************************
 * LED STRUCTURE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

/**
 * @brief Initializes the led structure.
 * @param led	Led instance pointer
 */
static void ledInstanceInit(led_t* led);

/**
 * @brief Locks the control process of the led, so the interruption service routine
 * 		  will not try to control the led, this must be used whenever the led changes
 * 		  its current settings.
 * @param led	Led instance pointer
 */
static void ledInstanceLock(led_t* led);

/**
 * @brief Unlocks the led, enables the ISR control process.
 * @param led	Led instance pointer
 */
static void ledInstanceUnlock(led_t* led);

/**
 * @brief Runs the process of control every tick for each function
 * @param led	Led instance pointer
 */
static void ledInstanceBlinkProcess(led_t* led);
static void ledInstanceBlinkSomeProcess(led_t* led);
static void ledInstanceBurstProcess(led_t* led);
static void ledInstanceOneShotProcess(led_t* led);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static const process	processDispatch[LED_MODE_COUNT] = {
		NULL,							// mode = STATIC
		ledInstanceBlinkProcess,		// mode = BLINK
		ledInstanceBlinkSomeProcess,	// mode = BLINK_SOME
		ledInstanceBurstProcess,		// mode = BURST
		ledInstanceOneShotProcess		// mode = ONE_SHOT
};

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// Definition of each led instance
static led_t 			leds[LED_TOTAL_COUNT] = {
		//  LED PIN			LED ACTIVE STATUS
		{	PIN_LED_RED,	LED_ACTIVE	},
		{	PIN_LED_BLUE, 	LED_ACTIVE	},
		{	PIN_LED_GREEN, 	LED_ACTIVE	}
};

/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void ledInit(void)
{
	static bool init = false;
	if (!init)
	{
		// If not initialized, clear the flag to avoid multiple initializations
		init = true;

		// Initialize each led instance structure
		for (uint8_t ledIndex = 0; ledIndex < LED_TOTAL_COUNT; ledIndex++)
		{
			ledInstanceInit(&leds[ledIndex]);
		}

#ifdef LED_DRIVER_ADVANCED_MODE
		// Initialize the base time used for the driver ISR
		systickInit(ledISR);
#endif
	}
}

void ledSet(led_id_t id)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = STATIC;
		gpioWrite(led->pin, led->active);

		ledInstanceUnlock(led);
	}
}

void ledClear(led_id_t id)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = STATIC;
		gpioWrite(led->pin, !led->active);

		ledInstanceUnlock(led);
	}
}

void ledToggle(led_id_t id)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = STATIC;
		gpioToggle(led->pin);

		ledInstanceUnlock(led);
	}
}

void ledStatus(led_id_t id, bool active)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = STATIC;
		gpioWrite(led->pin, active ? led->active : !led->active);

		ledInstanceUnlock(led);
	}
}

void ledBlink(led_id_t id, uint32_t period)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
#ifdef LED_DRIVER_ADVANCED_MODE
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = BLINK;
		led->context.blink.period = period;
		led->context.blink.counter = period / 2;
		gpioWrite(led->pin, led->active);

		ledInstanceUnlock(led);
#endif
	}
}

void ledBlinkSome(led_id_t id, uint32_t period, uint8_t blinks)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
#ifdef LED_DRIVER_ADVANCED_MODE
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = BLINK_SOME;
		led->context.blinkSome.period = period;
		led->context.blinkSome.counter = period / 2;
		led->context.blinkSome.changes = 2 * blinks - 1;
		gpioWrite(led->pin, led->active);

		ledInstanceUnlock(led);
#endif
	}
}

void ledBurst(led_id_t id, uint32_t burstPeriod, uint32_t blinkPeriod, uint8_t blinks)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
#ifdef LED_DRIVER_ADVANCED_MODE
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = BURST;
		led->context.burst.burstPeriod = burstPeriod;
		led->context.burst.blinkPeriod = blinkPeriod;
		led->context.burst.burstCounter = burstPeriod;
		led->context.burst.blinkCounter = blinkPeriod / 2;
		led->context.burst.blinks = blinks;
		led->context.burst.changes = 2 * blinks - 1;
		gpioWrite(led->pin, led->active);

		ledInstanceUnlock(led);
#endif
	}
}

void ledOneShot(led_id_t id, uint32_t duration)
{
#ifdef LED_DEVELOPMENT_MODE
	if (id >= 0 && id < LED_TOTAL_COUNT)
#endif
	{
#ifdef LED_DRIVER_ADVANCED_MODE
		led_t* led = &leds[id];
		ledInstanceLock(led);

		led->mode = ONE_SHOT;
		led->context.oneShot.counter = duration;
		gpioWrite(led->pin, led->active);

		ledInstanceUnlock(led);
#endif
	}
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void ledISR(void)
{
	for (uint8_t ledIndex = 0; ledIndex < LED_TOTAL_COUNT; ledIndex++)
	{
		led_t* led = &leds[ledIndex];
		if (led->enabled)
		{
			process modeProcess = processDispatch[led->mode];
			if (modeProcess)
			{
				modeProcess(led);
			}
		}
	}
}

/*******************************************************************************
 *******************************************************************************
                        LED STRUCTURE FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

static void ledInstanceInit(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		// Initialization of some structure variables
		led->enabled = true;
		led->mode = STATIC;

		// Initialization of GPIO associated to the led structure
		gpioWrite(led->pin, !led->active);
		gpioMode(led->pin, OUTPUT);
	}
}

static void ledInstanceLock(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		led->enabled = false;
	}
}

static void ledInstanceUnlock(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		led->enabled = true;
	}
}

static void ledInstanceBlinkProcess(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		blink_context_t* context = &led->context.blink;
		if (--context->counter == 0)
		{
			gpioToggle(led->pin);
			context->counter = context->period / 2;
		}
	}
}

static void ledInstanceBlinkSomeProcess(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		blink_some_context_t* context = &led->context.blinkSome;
		if (--context->counter == 0)
		{
			gpioToggle(led->pin);
			if (--context->changes == 0)
			{
				led->mode = STATIC;
			}
			else
			{
				context->counter = context->period / 2;
			}
		}
	}
}

static void ledInstanceBurstProcess(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		burst_context_t* context = &led->context.burst;
		if (--context->burstCounter == 0)
		{
			gpioWrite(led->pin, led->active);
			context->burstCounter = context->burstPeriod;
			context->blinkCounter = context->blinkPeriod / 2;
			context->changes = context->blinks * 2 - 1;
		}
		else
		{
			if (context->changes)
			{
				if (--context->blinkCounter == 0)
				{
					gpioToggle(led->pin);
					context->blinkCounter = context->blinkPeriod / 2;
					context->changes--;
				}
			}
		}
	}
}

static void ledInstanceOneShotProcess(led_t* led)
{
#ifdef LED_DEVELOPMENT_MODE
	if (led)
#endif
	{
		one_shot_context_t* context = &led->context.oneShot;
		if (--context->counter == 0)
		{
			gpioWrite(led->pin, !led->active);
			led->mode = STATIC;
		}
	}
}


/*******************************************************************************
 *******************************************************************************
						 INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

/******************************************************************************/
