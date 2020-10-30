/***************************************************************************//**
  @file     button.c
  @brief    Buttons driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/


/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "../../MCAL/systick/systick.h"
#include "../../MCAL/gpio/gpio.h"
#include "../../../board/board.h"
#include "button.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define	SYSTICK_MS				( 1.0 / SYSTICK_ISR_FREQUENCY_HZ * 1000.0 )
#define MS2TICKS(ms)  			( (ms) / SYSTICK_MS )

#define	LKP_MS					3000
#define TYPEMATIC_MS			500 // 500ms for debug

#define	BUTTON_DEVELOPMENT_MODE 1

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// FSM State IDs
typedef enum {
	RELEASED,
	PRESSED,
	TYPEMATIC
} buttonFsmState_t;

// FSM Event IDs
typedef enum {
	NO_EVENT,
	PRESS,
	RELEASE,
	TIMEOUT
} buttonFsmEv_t;

// Button structure
typedef struct {
	// Button static properties
	const pin_t 	pinNumber;			// Board pin assigned to button
	const bool 		activeState;		// Whether the button is active high or low
	const bool 		pullEnable;			// Enable/Disable pullup/down
	const uint32_t 	debounceMillis;		// Milliseconds to wait for debounce

	// External event emitters, call this functions on each event
	void 			(*evCallbacks[BUTTON_EV_COUNT]) (void);

	// FSM state
	buttonFsmState_t fsmState;

	// Counters for debounce and LKP/TYPEMATIC timeouts
	uint32_t debounceTicks;
	uint32_t timeoutCount;
} button_t;

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

static buttonFsmState_t buttonFSMCycle(buttonFsmState_t prevState, buttonFsmEv_t ev, button_t *button);
static void handleCount(void);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/


/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

static button_t buttons[BUTTON_COUNT] = {
	//	  Pin	  	ActiveState PE		MsBounce
		{ PIN_SW3,	SW3_ACTIVE, true,	100 	},
		{ PIN_SW2,	SW2_ACTIVE, true,	100  	}
};


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void buttonInit(void)
{
	// Configure periodic interrupts
	systickInit(handleCount);

	// Construct each button instance
	uint8_t i;
	uint32_t pull = 0;
	for( i = 0 ; i < BUTTON_COUNT ; i++)
	{
		// Initial FSM state
		buttons[i].fsmState = RELEASED;

		// Check if pullup/down was requested and configure pins for GPIO input
		if (buttons[i].pullEnable)
		{
			pull = buttons[i].activeState ? PULLDOWN : PULLUP;
		}
		gpioMode(buttons[i].pinNumber, INPUT | pull );
		pull = 0;
	}
}

void buttonSubscribe(button_id_t id, button_events_t ev, void (*callback) (void))
{
#ifdef BUTTON_DEVELOPMENT_MODE
	if ((id < BUTTON_COUNT) && (ev < BUTTON_EV_COUNT))
#endif // BUTTON_DEVELOPMENT_MODE
	{
		buttons[id].evCallbacks[ev] = callback;
	}
}

bool isButtonPressed(button_id_t id)
{
	bool ret = false;
#ifdef BUTTON_DEVELOPMENT_MODE
	if (id < BUTTON_COUNT)
#endif // BUTTON_DEVELOPMENT_MODE
	{
		ret = (gpioRead(buttons[id].pinNumber) == buttons[id].activeState);
	}
	return ret;
}
/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void handleCount(void)
{
	uint8_t i;
	bool active;
	buttonFsmEv_t ev;
	buttonFsmState_t state;

	for( i = 0 ; i < BUTTON_COUNT ; i++)
	{
		// If we are not waiting for debounce
		if( buttons[i].debounceTicks == 0 )
		{
			// No event by default
			ev = NO_EVENT;

			// Check whether the button is being pressed of not
			active = (gpioRead(buttons[i].pinNumber) == buttons[i].activeState);
			state = buttons[i].fsmState;

			// If there was a change on the button state, emit respective event
			if ( active &&  (state == RELEASED) )
			{
				ev = PRESS;
			}
			else if ( !active && (state != RELEASED) )
			{
				ev = RELEASE;
			}
			else if (buttons[i].timeoutCount > 0)
			{
				if(--buttons[i].timeoutCount == 0)
				{
					ev = TIMEOUT;
				}
			}

			// Check if something happened
			if( ev != NO_EVENT )
			{
				// Cycle State Machine
				buttons[i].fsmState = buttonFSMCycle(state, ev, &(buttons[i]) );
			}

			// Configure debounce if button pressed or released
			if ((ev == PRESS) || (ev == RELEASE))
			{
				buttons[i].debounceTicks = MS2TICKS(buttons[i].debounceMillis);
			}
		}
		else
		{
			buttons[i].debounceTicks--;
		}
	}
}


buttonFsmState_t buttonFSMCycle(buttonFsmState_t prevState, buttonFsmEv_t ev, button_t *button)
{
	buttonFsmState_t nextState = prevState;
	button_events_t subscribedEv = BUTTON_EV_COUNT;

	switch(prevState)
	{
	case RELEASED:
		if (ev == PRESS)
		{
			nextState = PRESSED;
			button->timeoutCount = MS2TICKS(LKP_MS);
			subscribedEv = BUTTON_PRESS;
		}
		break;
	case PRESSED:
		if (ev == RELEASE)
		{
			nextState = RELEASED;
			button->timeoutCount = 0;
			subscribedEv = BUTTON_RELEASE;
		}
		else if (ev == TIMEOUT)
		{
			nextState = TYPEMATIC;
			button->timeoutCount = MS2TICKS(TYPEMATIC_MS);
			subscribedEv = BUTTON_LKP;
		}
		break;
	case TYPEMATIC:
		if (ev == RELEASE)
		{
			nextState = RELEASED;
			button->timeoutCount = 0;
			subscribedEv = BUTTON_RELEASE;
		}
		else if (ev == TIMEOUT)
		{
			nextState = TYPEMATIC;
			button->timeoutCount = MS2TICKS(TYPEMATIC_MS);
			subscribedEv = BUTTON_TYPEMATIC;
		}
		break;
	}

	// If there is an event to emit
	if(subscribedEv < BUTTON_EV_COUNT)
	{
		// Emit external event if somebody is subscribed
		if(button->evCallbacks[subscribedEv])
		{
			(button->evCallbacks[subscribedEv])();
		}
	}

	return nextState;
}

/******************************************************************************/
