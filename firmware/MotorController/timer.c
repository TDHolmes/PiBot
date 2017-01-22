/*
 * Timer setup and run functions
 *
 * Author: Tyler Holmes (tyler@holmesengineering.com)
 * Date: 1/21/2016
 */

#include <stdint.h>


#include "MKL03Z4.h"
#include "hardware.h"

#include "fsl_lptmr.h"
#include "fsl_gpio.h"
#include "fsl_common.h"   // has USEC_TO_COUNT macro

#include "pin_mux.h"
#include "clock_config.h"


// Get source clock for LPTMR driver
#define LPTMR_SOURCE_CLOCK CLOCK_GetFreq(kCLOCK_LpoClk)

// Define LPTMR microseconds counts value
#define LPTMR_USEC_COUNT 1000U
#define TIMER_TICK_IRQ LPTMR0_IRQHandler


typedef struct {
    volatile uint32_t count;
    uint16_t flag_threshold;
    uint8_t  *flag_ptr;
} timer_admin_t;


timer_admin_t tmr_admin;


void timer_init(uint8_t * flag_ptr, uint16_t flag_threshold)
{
    lptmr_config_t lptmrConfig;
    /* Configure LPTMR */
    /*
     * lptmrConfig.timerMode = kLPTMR_TimerModeTimeCounter;
     * lptmrConfig.pinSelect = kLPTMR_PinSelectInput_0;
     * lptmrConfig.pinPolarity = kLPTMR_PinPolarityActiveHigh;
     * lptmrConfig.enableFreeRunning = false;
     * lptmrConfig.bypassPrescaler = true;
     * lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1;
     * lptmrConfig.value = kLPTMR_Prescale_Glitch_0;
     */
    LPTMR_GetDefaultConfig(&lptmrConfig);

    /* Initialize the LPTMR */
    LPTMR_Init(LPTMR0, &lptmrConfig);

    /* Set timer period */
    LPTMR_SetTimerPeriod(LPTMR0, USEC_TO_COUNT(LPTMR_USEC_COUNT, LPTMR_SOURCE_CLOCK));

    /* Enable timer interrupt */
    LPTMR_EnableInterrupts(LPTMR0, kLPTMR_TimerInterruptEnable);

    // setting up our struct...
    tmr_admin.count = 0;
    tmr_admin.flag_threshold = flag_threshold;
    tmr_admin.flag_ptr = flag_ptr;

    /* Enable at the NVIC */
    EnableIRQ(LPTMR0_IRQn);
    /* Start counting */
    LPTMR_StartTimer(LPTMR0);
}

//! Timer IRQ that does the ticking.

void TIMER_TICK_IRQ(void)
{
    LPTMR_ClearStatusFlags(LPTMR0, kLPTMR_TimerCompareFlag);
    tmr_admin.count += 1;
    // check if we should raise the flag!
    if (tmr_admin.count % tmr_admin.flag_threshold == 0) {
        *tmr_admin.flag_ptr = 1;
    }
}

//! Helper accesser functions that can clear / read out the current tick

inline void timer_clear(void)
{
    tmr_admin.count = 0;
}

inline uint32_t timer_get(void)
{
    return tmr_admin.count;
}
