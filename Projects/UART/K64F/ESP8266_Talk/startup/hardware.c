/*******************************************************************************
  @file     hardware.h
  @brief
  @author   Nicolás Magliola, Lucas A. Kammann (modifications)
 ******************************************************************************/

#include "hardware.h"

static uint32_t __LDM_interruptDisableCount = 0;

void hardwareInit (void)
{
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));      /* set CP10, CP11 for Full Access to the FPU*/

    WDOG->UNLOCK  = WDOG_UNLOCK_WDOGUNLOCK(0xC520);     /* Key 1 */
    WDOG->UNLOCK  = WDOG_UNLOCK_WDOGUNLOCK(0xD928);     /* Key 2 */
    WDOG->STCTRLH = WDOG_STCTRLH_ALLOWUPDATE_MASK | WDOG_STCTRLH_CLKSRC_MASK | 0x0100U; /* Disable WDOG */

    PMC->REGSC |= PMC_REGSC_ACKISO_MASK;                /* Release hold with ACKISO:  Only has an effect if recovering from VLLSx.*/

    SIM->CLKDIV1 = SIM_CLKDIV1_OUTDIV1(0x00) | SIM_CLKDIV1_OUTDIV2(0x01) | SIM_CLKDIV1_OUTDIV3(0x01) | SIM_CLKDIV1_OUTDIV4(0x03); /* Core-System = 100MHz, Bus = 50MHz, FlexBus = 50MHz, Flash = 25MHz */
    SIM->SOPT1 |= SIM_SOPT1_OSC32KSEL(0x03);            /* Set 32 kHz clock source (ERCLK32K) */
    SIM->SOPT2 = SIM_SOPT2_PLLFLLSEL_MASK;              /* Set high frequency clock source (PLL) */
    
    MCG->SC = MCG_SC_FCRDIV(0x02);                      /* Fast clock internal reference divider */
    MCG->C2 = MCG_C2_RANGE(0x02);                       /* High frequency range external reference selection */

    OSC->CR = OSC_CR_ERCLKEN_MASK;                      /* Set external reference clock (OSCERCLK) */

    MCG->C7 = MCG_C7_OSCSEL(0x00);                      /* Set FLL external reference clock (OSCCLK0) */
    MCG->C1 = MCG_C1_CLKS(0x02) | MCG_C1_FRDIV(0x07);   /* Set external reference as source, FLL external reference divider (PBE mode) */
    while((MCG->S & MCG_S_IREFST_MASK) != 0x00U);       /* Check external reference validation */
    MCG->C5 = MCG_C5_PRDIV0(0x0F);                      /* Set PLL divider while PLL turned off */
    MCG->C6 = MCG_C6_PLLS_MASK | MCG_C6_VDIV0(0x08);    /* Set PLL multiplier and PLL select */
    while((MCG->S & MCG_S_LOCK0_MASK) == 0x00U);        /* Wait until PLL is locked*/
    MCG->C1 &= ~MCG_C1_CLKS_MASK;
    while((MCG->S & MCG_S_CLKST_MASK) != 0x0CU);        /* Wait until output of the PLL is selected */
}

void hardwareEnableInterrupts (void)
{
    if (__LDM_interruptDisableCount > 0)
    {
        __LDM_interruptDisableCount--;
        if (__LDM_interruptDisableCount == 0)
            __enable_irq();
    }
}

void hardwareDisableInterrupts (void)
{
    __disable_irq();
    __LDM_interruptDisableCount++;
}
