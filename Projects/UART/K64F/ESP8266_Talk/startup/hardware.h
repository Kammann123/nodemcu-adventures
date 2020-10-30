/*******************************************************************************
  @file     hardware.h
  @brief
  @author   Nicolás Magliola, Lucas A. Kammann (modifications)
 ******************************************************************************/

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

/*******************************************************************************
 * INCLUDE HEADER FILES
 ******************************************************************************/

#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * ENUMERATIONS AND STRUCTURES AND TYPEDEFS
 ******************************************************************************/

#include "fsl_device_registers.h"
#include "core_cm4.h"
#include <stdbool.h>
#include <stdint.h>

#define __CORE_CLOCK__  100000000U
#define __FOREVER__     for(;;)
#define __ISR__         void __attribute__ ((interrupt))

void hardwareInit (void);
void hardwareEnableInterrupts (void);
void hardwareDisableInterrupts (void);

/* See IRQn_Type for IRQn definitions
 * Example: NVIC_EnableIRQ(SysTick_IRQn);
 * Example: NVIC_DisableIRQ(SysTick_IRQn);
 */

/* See this list for ISR definitions (ISR ~ Handler/IRQHandler)
 * Example: __ISR__ SysTick_Handler (void);
 */
/*
 *    NMI_Handler
 *    HardFault_Handler
 *    MemManage_Handler
 *    BusFault_Handler
 *    UsageFault_Handler
 *    SVC_Handler
 *    DebugMon_Handler
 *    PendSV_Handler
 *    SysTick_Handler
 *    DMA0_IRQHandler
 *    DMA1_IRQHandler
 *    DMA2_IRQHandler
 *    DMA3_IRQHandler
 *    DMA4_IRQHandler
 *    DMA5_IRQHandler
 *    DMA6_IRQHandler
 *    DMA7_IRQHandler
 *    DMA8_IRQHandler
 *    DMA9_IRQHandler
 *    DMA10_IRQHandler
 *    DMA11_IRQHandler
 *    DMA12_IRQHandler
 *    DMA13_IRQHandler
 *    DMA14_IRQHandler
 *    DMA15_IRQHandler
 *    DMA_Error_IRQHandler
 *    MCM_IRQHandler
 *    FTFE_IRQHandler
 *    Read_Collision_IRQHandler
 *    LVD_LVW_IRQHandler
 *    LLWU_IRQHandler
 *    WDOG_EWM_IRQHandler
 *    RNG_IRQHandler
 *    I2C0_IRQHandler
 *    I2C1_IRQHandler
 *    SPI0_IRQHandler
 *    SPI1_IRQHandler
 *    I2S0_Tx_IRQHandler
 *    I2S0_Rx_IRQHandler
 *    UART0_LON_IRQHandler
 *    UART0_RX_TX_IRQHandler
 *    UART0_ERR_IRQHandler
 *    UART1_RX_TX_IRQHandler
 *    UART1_ERR_IRQHandler
 *    UART2_RX_TX_IRQHandler
 *    UART2_ERR_IRQHandler
 *    UART3_RX_TX_IRQHandler
 *    UART3_ERR_IRQHandler
 *    ADC0_IRQHandler
 *    CMP0_IRQHandler
 *    CMP1_IRQHandler
 *    FTM0_IRQHandler
 *    FTM1_IRQHandler
 *    FTM2_IRQHandler
 *    CMT_IRQHandler
 *    RTC_IRQHandler
 *    RTC_Seconds_IRQHandler
 *    PIT0_IRQHandler
 *    PIT1_IRQHandler
 *    PIT2_IRQHandler
 *    PIT3_IRQHandler
 *    PDB0_IRQHandler
 *    USB0_IRQHandler
 *    USBDCD_IRQHandler
 *    Reserved71_IRQHandler
 *    DAC0_IRQHandler
 *    MCG_IRQHandler
 *    LPTMR0_IRQHandler
 *    PORTA_IRQHandler
 *    PORTB_IRQHandler
 *    PORTC_IRQHandler
 *    PORTD_IRQHandler
 *    PORTE_IRQHandler
 *    SWI_IRQHandler
 *    SPI2_IRQHandler
 *    UART4_RX_TX_IRQHandler
 *    UART4_ERR_IRQHandler
 *    UART5_RX_TX_IRQHandler
 *    UART5_ERR_IRQHandler
 *    CMP2_IRQHandler
 *    FTM3_IRQHandler
 *    DAC1_IRQHandler
 *    ADC1_IRQHandler
 *    I2C2_IRQHandler
 *    CAN0_ORed_Message_buffer_IRQHandler
 *    CAN0_Bus_Off_IRQHandler
 *    CAN0_Error_IRQHandler
 *    CAN0_Tx_Warning_IRQHandler
 *    CAN0_Rx_Warning_IRQHandler
 *    CAN0_Wake_Up_IRQHandler
 *    SDHC_IRQHandler
 *    ENET_1588_Timer_IRQHandler
 *    ENET_Transmit_IRQHandler
 *    ENET_Receive_IRQHandler
 *    ENET_Error_IRQHandler
 */

#endif /* _HARDWARE_H_ */
