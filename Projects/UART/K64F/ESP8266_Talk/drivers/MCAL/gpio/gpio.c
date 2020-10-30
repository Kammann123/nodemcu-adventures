/*******************************************************************************
  @file     gpio.c
  @brief    Simple GPIO Pin services, similar to Arduino
  @author   N. Magliola, G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "MK64F12.h"
#include "gpio.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define SETTING(x,y) (((x) & (y)) ? y##_K64F : 0)

#define TYPE_MASK					0x0080 // Unused bit on PCR
#define GPIO_MASK					(1 << PORT_PCR_MUX_SHIFT)

#define INPUT_K64F             		0
#define OUTPUT_K64F            		TYPE_MASK
#define PULLDOWN_K64F			    PORT_PCR_PE_MASK
#define PULLUP_K64F				    ( PORT_PCR_PE_MASK | PORT_PCR_PS_MASK )
#define SLEWRATE_K64F 			    PORT_PCR_SRE_MASK
#define OPENDRAIN_K64F			    PORT_PCR_ODE_MASK
#define LOCK_K64F				    PORT_PCR_LK_MASK
#define GPIO_FILTER_K64F			PORT_PCR_PFE_MASK
#define DRIVESTRENGTH_K64F		  	PORT_PCR_DSE_MASK


/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/


/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

static void (*(drivers[5][32]))(void);

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

// ISR handler for each port
void PORTA_IRQHandler(void);
void PORTB_IRQHandler(void);
void PORTC_IRQHandler(void);
void PORTD_IRQHandler(void);
void PORTE_IRQHandler(void);

// Enable the gate for the clock of each pin
static void clockGateEnable(pin_t pin);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/



/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/



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

static void clockGateEnable(pin_t pin)
{
	// Checks if port is enabled and if not, enables it.
	switch (PIN2PORT(pin))
	{
		case PA:
		{
			if (!(SIM->SCGC5 & SIM_SCGC5_PORTA_MASK))
			{
				SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
			}
			break;
		}
		case PB:
		{
			if (!(SIM->SCGC5 & SIM_SCGC5_PORTB_MASK))
			{
				SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
			}
			break;

		}
		case PC:
		{
			if (!(SIM->SCGC5 & SIM_SCGC5_PORTC_MASK))
			{
				SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
			}
			break;
		}
		case PD:
		{
			if (!(SIM->SCGC5 & SIM_SCGC5_PORTD_MASK))
			{
				SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
			}
			break;
		}
		case PE:
		{
			if (!(SIM->SCGC5 & SIM_SCGC5_PORTE_MASK))
			{
				SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;
			}
			break;
		}
		default: break;
	}
}

void gpioMode (pin_t pin, uint32_t mode)
{
	// Handy use of port and gpio pointers
	PORT_Type* ports[] = PORT_BASE_PTRS;
	GPIO_Type* gpios[] = GPIO_BASE_PTRS;
	uint32_t setting = mode;

	// Enabling the gate for the clock of each pin
	clockGateEnable(pin);

	// Save the current status of gpioIRQ
	mode = ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & PORT_PCR_IRQC_MASK;

	// K64F converting mode flags
	mode |= SETTING(setting, INPUT) 	| SETTING(setting, OUTPUT) 		| SETTING(setting, PULLDOWN) 	|  SETTING(setting, PULLUP);
	mode |= SETTING(setting, SLEWRATE) 	| SETTING(setting, OPENDRAIN) 	| SETTING(setting, LOCK) 		|  SETTING(setting, GPIO_FILTER);

	// Setting MUX to GPIO alternative.
	ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = ( GPIO_MASK | ( mode & ~TYPE_MASK ) );

	// Setting GPIOs to read/write. If mode is OUTPUT, mode=1, otherwise it's an input.
	gpios[PIN2PORT(pin)]->PDDR |= ( (mode & TYPE_MASK) == OUTPUT_K64F) << PIN2NUM(pin);
}

void gpioToggle (pin_t pin)
{
	// Port pointer, back up interrupt status flag
	GPIO_Type* gpios[] = GPIO_BASE_PTRS;

	// Toggling pin.
	gpios[PIN2PORT(pin)]->PTOR |= 1 << PIN2NUM(pin);
}

void gpioWrite (pin_t pin, bool value)
{
	// Port pointer, back up interrupt status flag
	GPIO_Type* gpios[] = GPIO_BASE_PTRS;
	uint32_t mask = 0x1 << PIN2NUM(pin);

	if (value == HIGH)
	{
		gpios[PIN2PORT(pin)]->PDOR |= mask;
	}
	else
	{
		gpios[PIN2PORT(pin)]->PDOR &= (~mask);
	}
}

bool gpioRead (pin_t pin)
{
	// Port pointer, back up interrupt status flag
	GPIO_Type* gpios[] = GPIO_BASE_PTRS;

	return (gpios[PIN2PORT(pin)]->PDIR & (1 << PIN2NUM(pin)) ) != LOW;
}

bool gpioIRQ (pin_t pin, uint8_t irqMode, pinIrqFun_t irqFun)
{
	PORT_Type* ports[] = PORT_BASE_PTRS;
	IRQn_Type irqs[] = PORT_IRQS;

	if (irqFun || irqMode == GPIO_IRQ_MODE_DISABLE)
	{
		// PCR interruption mode configuration
		ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] |= PORT_PCR_IRQC(irqMode);
		if (irqMode != GPIO_IRQ_MODE_DISABLE)
		{
			// Setting the driver's routine
			drivers[PIN2PORT(pin)][PIN2NUM(pin)] = irqFun;

			// NVIC local enable for the interrupt
			NVIC_EnableIRQ(irqs[PIN2PORT(pin)]);
		}
	}

	return !(irqFun);
}

void portHandler(uint8_t port)
{
	// Port pointer, back up interrupt status flag
	PORT_Type* ports[] = PORT_BASE_PTRS;
	uint32_t isfr = ports[port]->ISFR;

	// Clear interrupt status flag for every pin in the port
	ports[port]->ISFR = 0xFFFFFFFF;

	// Verify which pin was interrupted, dispatches
	uint8_t i = 0;
	for (i = 0; i < 32 && isfr; i++)
	{
		if (isfr & 1)
		{
			if (drivers[port][i])
			{
				drivers[port][i]();
			}
		}

		isfr = isfr >> 1;
	}
}

/*******************************************************************************
 *******************************************************************************
						INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

void PORTA_IRQHandler(void)
{
	portHandler(PA);
}

void PORTB_IRQHandler(void)
{
	portHandler(PB);
}

void PORTC_IRQHandler(void)
{
	portHandler(PC);
}

void PORTD_IRQHandler(void)
{
	portHandler(PD);
}

void PORTE_IRQHandler(void)
{
	portHandler(PE);
}

/******************************************************************************/
