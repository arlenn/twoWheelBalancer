

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
uint32_t leftMotorPos, rightMotorPos;
int32_t outputSAT;
int32_t lmOutputSAT;
int32_t rmOutputSAT;

int main(void) {

    tm4c123gInit(5);  //200Mhz / 5 = 40MHz system clock
    uartInit();
    InitIMUEuler();
    schedulerInit(SAMPLERATE); //hz
    mtrDrvInit(64);  //PWM clock, divide system clock by 64, 2 -> 64, two's compliment
    qeiInit();
    qeiResetPos();  //reset encoder counts

    IntMasterEnable(); //enable processor interrupts

    calibIMU();

    //initial conditions
    kp = 4.100;
    ki = 0.017;
    kd = 0.0065;  //0.0069
    kc = 0.016;  //0.0165
    re = -4.000;  //vertical offset

    lmkp = 1.00;
    lmki = 0.01;
    lmkd = 0.01;
    lmkc = 0.01;

    rmkp = 1.00;
    rmki = 0.01;
    rmkd = 0.01;
    rmkc = 0.01;


    while (1){
       UARTprintf("currAngle= %4i, outputSAT= %4i, left:= %4u, right:= %4u\r", (int)euler_p, (int)outputSAT, leftMotorPos, rightMotorPos);
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
    static int32_t leftOutput;
    static int32_t rightOutput;
    static uint32_t lDir;
    static uint32_t rDir;


    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Do the thing
    GetEulerAngles(&euler_h, &euler_r, &euler_p);  //read values from the IMU, unit: degree
    qeiGetPos(&leftMotorPos, &rightMotorPos);  //read values from the encoders, unit: mm

    outputSAT = pid(re, euler_p, kp, ki, kd, kc); //setpoint deg., measurement, kp, ki, kd,
                                                                                 //give-up angle deg., full-power angle deg.,
                                                                                 //sample-time

    lmOutputSAT = motorPid(UINT32_MAX / 2, leftMotorPos, lmkp, lmki, lmkd, lmkc);
    rmOutputSAT = motorPid(UINT32_MAX / 2, rightMotorPos, rmkp, rmki, rmkd, rmkc);

    leftOutput = lmOutputSAT + outputSAT;
    rightOutput = rmOutputSAT + outputSAT;

    // determine direction
    if (leftOutput < 0) lDir = REVERSE;
    else lDir = FORWARD;

    // determine direction
    if (rightOutput < 0) rDir = REVERSE;
    else rDir = FORWARD;

    // system has failed? turn off motors
    if (fabs(euler_p) > 80.0) outputSAT = 0.0;

    // command motors
    mtrDrvSpeed(MOTOR_LEFT, lDir, abs(leftOutput) );
    mtrDrvSpeed(MOTOR_RIGHT, rDir, abs(rightOutput) );




}

