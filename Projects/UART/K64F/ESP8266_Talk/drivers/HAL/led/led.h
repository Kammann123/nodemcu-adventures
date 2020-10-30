/*******************************************************************************
  @file     led.h
  @brief    LED Driver
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef HAL_LED_LED_H_
#define HAL_LED_LED_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define LED_TICK_FREQUENCY	(1000U)					// Tick frequency for the timing of LED's
#define LED_TICK_MS			1						// Equivalent amount of ticks of 1mS
#define LED_MS2TICK(ms)		((ms) / LED_TICK_MS)	// Converts milliseconds to amount of ticks

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring LED identification tags, definition of hardware related
// specifications is taken care in driver implementation file.
typedef enum {
	LED_RED,
	LED_BLUE,
	LED_GREEN,

	LED_TOTAL_COUNT
} led_id_t;

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/************************
 *                      *
 * LED GENERAL SERVICES *
 *                      *
 ***********************/

/**
 * @brief Initializes the LED driver.
 */
void ledInit(void);

/************************
 *                      *
 * LED CONTROL SERVICES *
 *                      *
 ***********************/

/**
 * @brief Turns on the specified LED.
 * @param id	LED identification tag
 */
void ledSet(led_id_t id);

/**
 * @brief Turns off the specified LED.
 * @param id	LED identification tag
 */
void ledClear(led_id_t id);

/**
 * @brief Toggles the current status of the specified LED.
 * @param id	LED identification tag
 */
void ledToggle(led_id_t id);

/**
 * @brief Sets the required status for the specified LED.
 * @param id		LED identification tag
 * @param active	Whether the LED is to set active or inactive
 */
void ledStatus(led_id_t id, bool active);

/*************************
 *                       *
 * LED FUNCTION SERVICES *
 *                       *
 ************************/

/**
 * @brief Starts the blinking mode of the specified LED.
 *        The blinking mode turns on the LED 50% of the period time, and then turns off
 *        the LED the rest of the period time, the process continues periodically.
 * @param id		ID of the led to start blinking
 * @param period	Period of the blinking
 */
void ledBlink(led_id_t id, uint32_t period);

/**
 * @brief Starts the blinking mode of the specified LED, during a given amount of blink periods.
 * 		  This blinking mode blinks the LED a given amount of time and then stops working.
 * @param id		ID of the led to start blinking
 * @param period	Period of the blinking
 * @param blinks	Amount of blinking periods
 */
void ledBlinkSome(led_id_t id, uint32_t period, uint8_t blinks);

/**
 * @brief Starts the burst mode of the specified LED.
 * 	      The burst mode generates an established amount of high speed blinks
 * 	      periodically.
 * @param id			ID of the led to start burst
 * @param burstPeriod	Period of the burst
 * @param blinkPeriod	Period of the blink
 * @param blinks		Amount of blinks at the beginning of each period
 */
void ledBurst(led_id_t id, uint32_t burstPeriod, uint32_t blinkPeriod, uint8_t blinks);

/**
 * @brief Starts the one shot mode of the specified LED.
 * 		  The one shot mode turns on the LED during a given interval of time in ticks.
 * @param id			ID of the led to start one shot
 * @param duration		Duration of the led turned on
 */
void ledOneShot(led_id_t id, uint32_t duration);

/*******************************************************************************
 ******************************************************************************/

#endif
