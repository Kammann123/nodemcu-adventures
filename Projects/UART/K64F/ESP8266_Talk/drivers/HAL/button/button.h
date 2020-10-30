/***************************************************************************//**
  @file     button.h
  @brief    Buttons driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef BUTTON_BUTTON_H_
#define BUTTON_BUTTON_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef enum {
	BUTTON_RELEASE,		// Button has been released
	BUTTON_PRESS,		// Button has been pressed
	BUTTON_LKP,			// Button has been pressed a long time
	BUTTON_TYPEMATIC,	// If the button is kept pressed some time, this event is emitted periodically
	BUTTON_EV_COUNT
} button_events_t;

// Declare the ID of buttons
typedef enum {
	BUTTON_1,
	BUTTON_2,
	BUTTON_COUNT
} button_id_t;

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Initialize button and corresponding peripheral
 */
void buttonInit(void);

// Non-Blocking services //

/**
 * @brief Suscribe to button event
 */
void buttonSubscribe(button_id_t id, button_events_t event, void (*callback) (void));

/**
 * @brief Check if button is pressed
 * @param id ID of the button
 */
bool isButtonPressed(button_id_t id);

#endif /* BUTTON_BUTTON_H_ */
