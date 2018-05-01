#include "scheduler.h"

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"


/************************************************************************************
 * Function: schedulerInit
 * configure ISR for motion control update
 * configured as highest priory task

 * argument: control loop frequency, Hz
 * return: void
 * Author: Hardy Nelson
 * Date:
 * Revision:
 *************************************************************************************/

void schedulerInit(uint32_t freq)
{
    uint32_t ui32Period;

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    ui32Period = (SysCtlClockGet() / freq);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);

    IntPrioritySet(INT_TIMER0A, 0);

    IntEnable(INT_TIMER0A);

    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    TimerEnable(TIMER0_BASE, TIMER_A);
}
