

/**
 * main.c
 */

#include "tm4c123g.h"
#include "mtrDrv.h"
#include "qei.h"
#include "uart.h"
#include "euler.h"
#include "scheduler.h"
#include "pid.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "uartstdio.h"

#define SAMPLERATE 100 //Hz

double euler_h, euler_r, euler_p;

int main(void) {

    volatile uint32_t leftMotorPos, rightMotorPos;

    //volatile double euler_h, euler_r, euler_p;

    //volatile double test;
//test comment

    tm4c123gInit(5);  //200Mhz / 5 = 40MHz system clock
    InitIMUEuler();
    uartInit();
    schedulerInit(SAMPLERATE); //hz
    mtrDrvInit(64);  //PWM clock, divide system clock by 64, 2 -> 64, two's compliment
    qeiInit();


    IntMasterEnable(); //enable processor interrupts

    while (1);
    return 0;
}


/*
 * Timer ISR
 */
void Timer0IntHandler(void)
{

    static int8_t output;
    static double currAngle;
    // Clear the timer interrupt


    // Do the thing
    GetEulerAngles( (double*)&euler_h, (double*)&euler_r, (double*)&euler_p);

    //UartGetK(); //polling for now until interrupt is made

    output = pid(0.1, euler_p, kp, ki, kd, kc, 45.0, 35.0, SAMPLERATE, 1.0);  //setpoint deg., measurement, kp, ki, kd,
                                                                          //give-up angle deg., full-power angle deg.,
                                                                          //sample-time s, minimum output

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

}

