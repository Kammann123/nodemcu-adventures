/*******************************************************************************
  @file     uart.c
  @brief    UART Driver for K64F. Non-Blocking and using FIFO feature.
  @author   G. Davidov, F. Farall, J. Gaytán, L. Kammann, N. Trozzo
 ******************************************************************************/

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include "CMSIS/MK64F12.h"

#include <stdint.h>
#include <string.h>

#include "../../../startup/hardware.h"
#include "../../../lib/queue/queue.h"
#include "../gpio/gpio.h"
#include "uart.h"

/*******************************************************************************
 * CONSTANT AND MACRO DEFINITIONS USING #DEFINE
 ******************************************************************************/

#define RX_BUFFER_SIZE       	100
#define TX_BUFFER_SIZE       	100

#define SYSTEM_CLOCK 	     	  ((uint32_t)100000000U)
#define BUS_CLOCK            	(SYSTEM_CLOCK / 2)

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

// Declaring the instance structure
typedef struct {
  // MCU Peripheral data
  uint8_t           txFifoWatermark;  // Water mark of the transmitter FIFO
  uint8_t           rxFifoWatermark;  // Water mark of the receiver FIFO
  uint8_t           txFifoSize;       // Size of the transmitter peripheral FIFO
  uint8_t           rxFifoSize;       // Size of the receiver peripheral FIFO

  // Event-oriented interface
  uart_tx_callback  txCallback;   // Callback registered for the transmitter
  uart_rx_callback  rxCallback;   // Callback registered for the receiver
  uint8_t           rxSize;       // Size of the frame to be notified
  
  // Buffering interface
  word_t            rxBuffer[RX_BUFFER_SIZE]; 	// Data buffer for Rx
  word_t            txBuffer[TX_BUFFER_SIZE];  	// Data buffer for Tx
  queue_t           txQueue;      				// Queue for Tx (must be initialized with txBuffer)
  queue_t           rxQueue;      				// Queue for Rx (must be initialized with rxBuffer)
  
  // Flags
  bool              txCompleted;   // asserts if transmission completed

  // Configuration of the UART protocol
  uart_cfg_t        cfg;

} uart_instance_t;

// Declaring UART protocol simple pin out
enum {
  UART_PIN_RX,
  UART_PIN_TX,
  UART_PIN_RTS,
  UART_PIN_CTS,
  UART_PIN_COUNT
};

/*******************************************************************************
 * VARIABLES WITH GLOBAL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * FUNCTION PROTOTYPES FOR PRIVATE FUNCTIONS WITH FILE LEVEL SCOPE
 ******************************************************************************/

// Declaring the IRQ Handlers for each receiver and transmitter UART instance,
// and the general dispatcher functions used to dispatch a transmission and reception.
__ISR__ UART0_RX_TX_IRQHandler(void);
__ISR__ UART1_RX_TX_IRQHandler(void);
__ISR__ UART2_RX_TX_IRQHandler(void);
__ISR__ UART3_RX_TX_IRQHandler(void);
__ISR__ UART4_RX_TX_IRQHandler(void);
static void UART_IRQDispatcher(uart_id_t id);
static void UART_TxDispatcher(uart_id_t id);
static void UART_RxDispatcher(uart_id_t id);
static void readFifo(uart_id_t id);

/*******************************************************************************
 * ROM CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

/*******************************************************************************
 * STATIC VARIABLES AND CONST VARIABLES WITH FILE LEVEL SCOPE
 ******************************************************************************/

// Memory context for each UART configuration and variables
static uart_instance_t  uartInstances[UART_AMOUNT] = {
  //	FIFO_TX_WATERMARK	FIFO_RX_WATERMARK
  { 	3, 					6		},
  { 	3, 					6		},
  { 	3, 					6		},
  { 	1, 					1		},
  { 	3, 					6		}
};

// Look up table for selecting the pin corresponding to each UART
static const pin_t    uartPins[UART_AMOUNT][UART_PIN_COUNT] = {
  //      RX                   TX                RTS                    CTS
  { PORTNUM2PIN(PB, 16),PORTNUM2PIN(PB, 17),PORTNUM2PIN(PB, 2), PORTNUM2PIN(PB, 3) }, // UART0
  { PORTNUM2PIN(PC, 3), PORTNUM2PIN(PC, 4), PORTNUM2PIN(PC, 1), PORTNUM2PIN(PC, 2) }, // UART1
  { PORTNUM2PIN(PD, 2), PORTNUM2PIN(PD, 3), PORTNUM2PIN(PD, 0), PORTNUM2PIN(PD, 1) }, // UART2
  { PORTNUM2PIN(PC, 16),PORTNUM2PIN(PC, 17),PORTNUM2PIN(PC, 18),PORTNUM2PIN(PB, 9) }, // UART3
  { PORTNUM2PIN(PE, 25),PORTNUM2PIN(PE, 24),PORTNUM2PIN(PC, 12),PORTNUM2PIN(PE, 26)}, // UART4
};

// Look up table to determine the alternative in each pin corresponding to the UART
static const uint8_t  uartPinAlts[UART_AMOUNT][UART_PIN_COUNT] = {
  //  RX  TX  RTS CTS
  {   3,  3,  3,  3 }, // UART0
  {   3,  3,  3,  3 }, // UART1
  {   3,  3,  3,  3 }, // UART2
  {   3,  3,  3,  3 }, // UART3
  {   3,  3,  3,  3 }, // UART4
};

// Mapping the UART memory position and IRQ number in the NVIC
static UART_Type*    	uartPointers[] = UART_BASE_PTRS;
static const uint8_t 	uartRxTxIrqs[] = UART_RX_TX_IRQS;


/*******************************************************************************
 *******************************************************************************
                        GLOBAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void uartInit (uint8_t id, uart_cfg_t config)
{
  UART_Type* uartInstance = uartPointers[id];
  PORT_Type* ports[]      = PORT_BASE_PTRS;

  // Enable clock gating
  switch(id)
  {
    case UART_INSTANCE_0:
      SIM->SCGC4 |= SIM_SCGC4_UART0_MASK;
      break;
      
    case UART_INSTANCE_1:
      SIM->SCGC4 |= SIM_SCGC4_UART1_MASK;
      break;
      
    case UART_INSTANCE_2:
      SIM->SCGC4 |= SIM_SCGC4_UART2_MASK;
      break;
      
    case UART_INSTANCE_3:
      SIM->SCGC4 |= SIM_SCGC4_UART3_MASK;
      break;
      
    case UART_INSTANCE_4:
      SIM->SCGC1 |= SIM_SCGC1_UART4_MASK;
      break;

    default:
      break;
  }

  // Port Clock Gating
  SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTD_MASK;
  SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

  // Number of stop bits
  uartInstance->BDH = UART_BDH_SBNS(config.stopMode);

  // Number of data bits and parity configuration
  uartInstance->C1 = UART_C1_M(config.length) | UART_C1_PE(config.parityEnable) | UART_C1_PT(config.parityMode);

  // Baud-Rate configuration
  uint32_t clk = ((id == UART_INSTANCE_0) || (id == UART_INSTANCE_1)) ? SYSTEM_CLOCK : BUS_CLOCK;
  uint16_t sbr, brfa;
  
  // SBR: coarse adjustment, BRFA: fine adjustment
  sbr = clk / (config.baudRate << 4);             
  brfa = ((clk << 2) / config.baudRate) - (sbr << 5);

  uartInstance->BDH = (uartInstance->BDH & ~UART_BDH_SBR_MASK) | UART_BDH_SBR(sbr >> 8);
  uartInstance->BDL = UART_BDL_SBR((uint8_t)(sbr & 0xFF));
  uartInstance->C4 = (uartInstance->C4 & ~UART_C4_BRFA_MASK) | UART_C4_BRFA(brfa);

  // Enable RX and TX FIFOs
  uartInstance->PFIFO |= UART_PFIFO_TXFE(1) | UART_PFIFO_RXFE(1);

  // Configure RX and TX FIFO water marks
  uartInstance->TWFIFO = uartInstances[id].txFifoWatermark;
  uartInstance->RWFIFO = uartInstances[id].rxFifoWatermark;

  // Save FIFO sizes
  uartInstances[id].txFifoSize = 2 << ((uartInstance->PFIFO & UART_PFIFO_TXFIFOSIZE_MASK) >> UART_PFIFO_TXFIFOSIZE_SHIFT);
  uartInstances[id].rxFifoSize = 2 << ((uartInstance->PFIFO & UART_PFIFO_RXFIFOSIZE_MASK) >> UART_PFIFO_RXFIFOSIZE_SHIFT);

  // Configure pins on UART alternative
  for (uint8_t i = 0 ; i < UART_PIN_COUNT ; i++ )
  {
    pin_t pin = uartPins[id][i];
    ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] = (ports[PIN2PORT(pin)]->PCR[PIN2NUM(pin)] & ~PORT_PCR_MUX_MASK) | PORT_PCR_MUX(uartPinAlts[id][i]);
  }

  // Enable NVIC
  NVIC_EnableIRQ(uartRxTxIrqs[id]);

  // UART IRQs initialization
  uartInstance->C2 |= UART_C2_RIE(1);

  // Receiver and transmitter queue initialization
  for (uint8_t i = 0 ; i < UART_AMOUNT ; i++)
  {
    uartInstances[i].rxQueue = createQueue(uartInstances[i].rxBuffer, RX_BUFFER_SIZE, sizeof(word_t)); 
    uartInstances[i].txQueue = createQueue(uartInstances[i].txBuffer, TX_BUFFER_SIZE, sizeof(word_t));
  }
  
  // Clearing the flags before starting
  uartInstance->S1;

  // Enable Transmitter and Receiver
  uartInstance->C2 |= UART_C2_RE(1);
}

bool uartSubscribeTxMsgComplete(uart_id_t id, uart_tx_callback callback)
{
  if (callback)
  {
    uartInstances[id].txCallback = callback;
    return true;
  }
  else
  {
    return false;
  }
}

bool uartSubscribeRxMsg(uart_id_t id, uart_rx_callback callback, uint8_t frameSize)
{
  if (callback)
  {
    uartInstances[id].rxCallback = callback;
    uartInstances[id].rxSize = frameSize;
    return true;
  }
  else
  {
    return false;
  }
}

bool uartHasRxMsg(uint8_t id)
{
  return !isEmpty(&(uartInstances[id].rxQueue));
}

uint8_t uartGetRxMsgLength(uint8_t id)
{
  return size(&uartInstances[id].rxQueue);
}

uint8_t uartReadMsg(uint8_t id, word_t* msg, uint8_t length)
{
  queue_t *rxQueue = &uartInstances[id].rxQueue;
  uint8_t rxWords = length <= size(rxQueue) ? length : size(rxQueue);
  popMany(rxQueue, msg, rxWords);
  return rxWords;
}

uint8_t uartWriteMsg(uint8_t id, const word_t* msg, uint8_t length)
{
  queue_t *txQueue = &(uartInstances[id].txQueue);
  uint8_t txWords = length <= emptySize(txQueue) ? length : emptySize(txQueue);
  uint8_t i = 0;

  if (txWords)
  {
    // Reset "completed transmission" flag if there is data to send
    uartInstances[id].txCompleted = false;

    // Fill the software FIFO with the message
    for ( i = 0 ; i < txWords ; i++ )
    {
      push(txQueue, (void*)&msg[i]);
    }

	// Fill hardware FIFO, if empty, to start the process
	if (uartPointers[id]->S1 & UART_S1_TDRE_MASK)
	{
		for ( i = 0 ; (i < txWords) && (i < (uartInstances[id].txFifoSize - uartPointers[id]->TCFIFO)) ; i++ )
		{
			// Clear the flag
			uartPointers[id]->S1;

			// Pop the next element from the Queue and transmit it
			word_t word = *(word_t*)pop(txQueue);
		    if (uartInstances[id].cfg.length == UART_DATA_9_BITS && !uartInstances[id].cfg.parityEnable)
		    {
		      uartPointers[id]->C3 = (uartPointers[id]->C3 & ~UART_C3_T8_MASK) | UART_C3_T8((word & 0x100) >> 8);
		    }
		    uartPointers[id]->D = word;
		}
	}

	// Enable Transmitter
	uartPointers[id]->C2 = (uartPointers[id]->C2 & ~UART_C2_TE_MASK & ~UART_C2_TIE_MASK & ~UART_C2_TCIE_MASK) | UART_C2_TE(1) | UART_C2_TIE(1) | UART_C2_TCIE(1);
  }

  return txWords;
}

bool uartIsTxMsgComplete(uint8_t id)
{
  bool prevFlag = uartInstances[id].txCompleted;
  uartInstances[id].txCompleted = false;
  return prevFlag;
}

/*******************************************************************************
 *******************************************************************************
                        LOCAL FUNCTION DEFINITIONS
 *******************************************************************************
 ******************************************************************************/

void readFifo(uart_id_t id)
{
  uart_instance_t*  uartInstance  = &uartInstances[id];
  UART_Type*        uart          = uartPointers[id];
  uint8_t           bufferCount;
  uint16_t          word;

  // Get the buffer count, the current amount of elements in the receiver FIFO,
  // and push every element from the FIFO to the driver receiver queue
  bufferCount = uart->RCFIFO;
  if (emptySize(&uartInstance->rxQueue) >= bufferCount)
  {
    while (bufferCount)
    {
      if (uartInstance->cfg.length == UART_DATA_9_BITS && !uartInstance->cfg.parityEnable)
      {
        word = (uart->C3 & UART_C3_R8_MASK) << 1;
      }
      else
      {
        word = 0x0000;
      }
      word = (word & 0x0100) | uart->D;
      push(&uartInstance->rxQueue, &word);
      bufferCount--;
    }
  }
  else 
  {
    // Read D to clear RDRF flag
    uart->D;
  }
}

static void UART_IRQDispatcher(uart_id_t id)
{
  UART_Type* uart = uartPointers[id];
  uint8_t	 s1 = uart->S1;

  // If the UART transmitter is enabled, check the flags
  if (uart->C2 & UART_C2_TE_MASK)
  {
	  if (s1 & UART_S1_TDRE_MASK)
	  {
	    UART_TxDispatcher(id);
	  }
	  if (s1 & UART_S1_TC_MASK)
	  {
	    if (isEmpty(&uartInstances[id].txQueue))
	    {
	      // Disable Transmitter
	  	  uartPointers[id]->C2 = uartPointers[id]->C2 & ~UART_C2_TE_MASK & ~UART_C2_TIE_MASK & ~UART_C2_TCIE_MASK;

	      // Callback or flag for polling service
		  if (uartInstances[id].txCallback)
	      {
	        uartInstances[id].txCallback();
	      }
	      else
	      {
	        uartInstances[id].txCompleted = true;
	      }
	    }
	  }
  }

  // If the UART receiver is enabled, check the flag
  if (uart->C2 & UART_C2_RE_MASK)
  {
	  if (s1 & UART_S1_RDRF_MASK)
	  {
	    UART_RxDispatcher(id);
	  }
  }
}

static void UART_TxDispatcher(uart_id_t id)
{
  uart_instance_t*  uartInstance  = &uartInstances[id];
  UART_Type*        uart          = uartPointers[id];
  uint8_t			queueSize     = size(&uartInstance->txQueue);
  uint8_t			fifoSize      = uartInstance->txFifoSize - uart->TCFIFO;
  uint8_t			length        = queueSize < fifoSize ? queueSize : fifoSize;

  for (uint8_t i = 0 ; i < length; i++)
  {
    // Pop an element from the transmission queue, and send it via the
    // UART D register to the FIFO, verifying when the 9 bits is enabled
    word_t word = *(word_t*)pop(&uartInstance->txQueue);
    if (uartInstance->cfg.length == UART_DATA_9_BITS && !uartInstance->cfg.parityEnable)
    {
      uart->C3 = (uart->C3 & ~UART_C3_T8_MASK) | UART_C3_T8((word & 0x100) >> 8);
    }
    uart->D = word;
  }
}

static void UART_RxDispatcher(uart_id_t id)
{
  uart_instance_t*  uartInstance  = &uartInstances[id];

  // Read the FIFO and push the elements into the software queue
  readFifo(id);

  // When the current size of the receiver queue is higher than the frame size
  // configured for interruption, the user is notified via the given callback
  if (size(&uartInstance->rxQueue) >= uartInstance->rxSize)
  {
    if (uartInstance->rxCallback)
    {
      uartInstance->rxCallback();
    }
  }
}

/*******************************************************************************
 *******************************************************************************
						            INTERRUPT SERVICE ROUTINES
 *******************************************************************************
 ******************************************************************************/

__ISR__ UART0_RX_TX_IRQHandler(void)
{
  UART_IRQDispatcher(UART_INSTANCE_0);
}

__ISR__ UART1_RX_TX_IRQHandler(void)
{
  UART_IRQDispatcher(UART_INSTANCE_1);
}

__ISR__ UART2_RX_TX_IRQHandler(void)
{
  UART_IRQDispatcher(UART_INSTANCE_2);
}

__ISR__ UART3_RX_TX_IRQHandler(void)
{
  UART_IRQDispatcher(UART_INSTANCE_3);
}

__ISR__ UART4_RX_TX_IRQHandler(void)
{
  UART_IRQDispatcher(UART_INSTANCE_4);
}

/******************************************************************************/
