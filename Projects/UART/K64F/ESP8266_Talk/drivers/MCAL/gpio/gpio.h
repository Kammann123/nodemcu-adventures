/*******************************************************************************
  @file     gpio.h
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

#ifndef _GPIO_H_
#define _GPIO_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

// Ports declaration
enum { PA, PB, PC, PD, PE };

// Convert port and number into pin ID
// Ex: PTB5  -> PORTNUM2PIN(PB,5)  -> 0x25
//     PTC22 -> PORTNUM2PIN(PC,22) -> 0x56
#define PORTNUM2PIN(p,n)    (((p)<<5) + (n))
#define PIN2PORT(p)         (((p)>>5) & 0x07)
#define PIN2NUM(p)          ((p) & 0x1F)

// Modes of configuration for each pin, optional settings can be appendeable 
// using INPUT | PULLDOWN | LOCK, for example...
#define INPUT             0x001
#define OUTPUT            0x002
#define PULLDOWN			    0x004
#define PULLUP				    0x008
#define SLEWRATE 			    0x010
#define OPENDRAIN			    0x020
#define LOCK				      0x040
#define GPIO_FILTER				0x080
#define DRIVESTRENGTH		  0x100

// Backwards Compatibility
#define INPUT_PULLUP        ( INPUT | PULLUP )
#define INPUT_PULLDOWN      ( INPUT | PULLDOWN )

// Digital values
#define LOW     0
#define HIGH    1

// IRQ modes
enum {
    GPIO_IRQ_MODE_DISABLE,
    GPIO_IRQ_MODE_DMA_RISING_EDGE,
    GPIO_IRQ_MODE_DMA_FALLING_EDGE,
    GPIO_IRQ_MODE_DMA_BOTH_EDGES,


	GPIO_IRQ_MODE_INTERRUPT_LOGIC_0 = 0x8,
	GPIO_IRQ_MODE_INTERRUPT_RISING_EDGE,
	GPIO_IRQ_MODE_INTERRUPT_FALLING_EDGE,
	GPIO_IRQ_MODE_INTERRUPT_BOTH_EDGES,
	GPIO_IRQ_MODE_INTERRUPT_LOGIC_1,
};

#define GPIO_IRQ_CANT_MODES 9


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

typedef uint8_t pin_t;

typedef void (*pinIrqFun_t)(void);

/*******************************************************************************
 * VARIABLE PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES WITH GLOBAL SCOPE
 ******************************************************************************/

/**
 * @brief Configures the specified pin to behave either as an input or an output
 * @param pin the pin whose mode you wish to set (according PORTNUM2PIN)
 * @param mode INPUT, OUTPUT, INPUT_PULLUP or INPUT_PULLDOWN.
 */
void gpioMode (pin_t pin, uint32_t mode);

/**
 * @brief Configures how the pin reacts when an IRQ event ocurrs
 * @param pin the pin whose IRQ mode you wish to set (according PORTNUM2PIN)
 * @param irqMode disable, risingEdge, fallingEdge or bothEdges
 * @param irqFun function to call on pin event
 * @return Registration succeed
 */
bool gpioIRQ (pin_t pin, uint8_t irqMode, pinIrqFun_t irqFun);

/**
 * @brief Write a HIGH or a LOW value to a digital pin
 * @param pin the pin to write (according PORTNUM2PIN)
 * @param val Desired value (HIGH or LOW)
 */
void gpioWrite (pin_t pin, bool value);

/**
 * @brief Toggle the value of a digital pin (HIGH<->LOW)
 * @param pin the pin to toggle (according PORTNUM2PIN)
 */
void gpioToggle (pin_t pin);

/**
 * @brief Reads the value from a specified digital pin, either HIGH or LOW.
 * @param pin the pin to read (according PORTNUM2PIN)
 * @return HIGH or LOW
 */
bool gpioRead (pin_t pin);


/*******************************************************************************
 ******************************************************************************/

#endif // _GPIO_H_
