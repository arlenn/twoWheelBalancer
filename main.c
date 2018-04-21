

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

#define sampleRate 100 //Hz


int main(void) {

    volatile uint32_t leftMotorPos, rightMotorPos;

    //volatile double euler_h, euler_r, euler_p;

    //volatile double test;

    tm4c123gInit(5);  //200Mhz / 5 = 40MHz system clock
    mtrDrvInit(64);  //PWM clock, divide system clock by 64, 2 -> 64, two's compliment
    qeiInit();
    uartInit();
    InitIMUEuler();
    schedulerInit(sampleRate); //hz


    /*
     * make a start button with these
     */
    mtrDrvEnable(MOTOR_LEFT, true);
    mtrDrvEnable(MOTOR_RIGHT, true);

    //mtrDrvSpeed(MOTOR_LEFT, FORWARD, 50);
    //mtrDrvSpeed(MOTOR_RIGHT, REVERSE, 50);



    while (1);

    return 0;
}


/*
 * Timer ISR
 */
void Timer0IntHandler(void)
{
    static double euler_h, euler_r, euler_p;
    static int8_t output;
    static double currAngle;

    // Clear the timer interrupt
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Do the thing
    GetEulerAngles( (double*)&euler_h, (double*)&euler_r, (double*)&euler_p);
    output = pid(0.0, euler_p, 5.0, 1.0, 1.0, 45.0, 35.0, sampleTime, 3.0);  //setpoint deg., measurement, kp, ki, kd,
                                                                             //give-up angle deg., full-power angle deg.,
                                                                             //sample-time s, minimum output %


    //currAngle = euler_p;

    //if (currAngle > 0) currAngle = currAngle - 180.0;
    //else currAngle = currAngle + 180.0;

    //UARTprintf("euler_p= %i, output= %i\n", (int)currAngle, output);

}

