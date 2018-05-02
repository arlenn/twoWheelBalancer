

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
double leftMotorPos, rightMotorPos;
int32_t outputSAT;

int main(void) {

    volatile uint32_t leftMotorPos, rightMotorPos;

    tm4c123gInit(5);  //200Mhz / 5 = 40MHz system clock
    uartInit();
    InitIMUEuler();

    schedulerInit(SAMPLERATE); //hz
    mtrDrvInit(64);  //PWM clock, divide system clock by 64, 2 -> 64, two's compliment
    qeiInit();

    IntMasterEnable(); //enable processor interrupts

    //initial conditions
    kp = 4.100;
    ki = 0.017;
    kd = 0.0065;  //0.0069
    kc = 0.016;  //0.0165

    while (1){
       UARTprintf("currAngle= %4i, outputSAT= %4i\r", (int)euler_p, (int)outputSAT);
    }

    return 0;
}


/************************************************************************************
 * Function: Timer0IntHandler
 * get Euler angles from IMU,
 * read values from encoders on MCU
 * call pid algorithm

 * argument: void
 * return: void
 * Author: Hardy Nelson & Kushant Gounder
 * Date: March.18/2018
 * Revision:
 *************************************************************************************/
void Timer0IntHandler(void)
{

    // Do the thing
    GetEulerAngles(&euler_h, &euler_r, &euler_p);  //read values from the IMU, unit: degree
    qeiGetPos(&leftMotorPos, &rightMotorPos);  //read values from the encoders, unit: mm

     outputSAT = pid(0.0, euler_p, kp, ki, kd, kc, 85.0, SAMPLERATE); //setpoint deg., measurement, kp, ki, kd,
                                                                                 //give-up angle deg., full-power angle deg.,
                                                                                 //sample-time



    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

}

