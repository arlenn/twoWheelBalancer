

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
double disturbance;
uint32_t leftMotorPos, rightMotorPos;
int32_t leftMotorVel, rightMotorVel;
double LRPM, RRPM;
int32_t outputSAT;
int32_t lmOutputSAT;
int32_t rmOutputSAT;

double disturbancePercent;

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
    kd = 0.0065;
    kc = 0.016;
    re = -4.000;  //vertical offset

    pkp = 0.015;
    pki = 0.000;
    pkd = 1.070;
    pkc = 0.000;
    pa = 3.0;

    //rmkp = 1.00;
    //rmki = 0.01;
    //rmkd = 0.01;
    //rmkc = 0.01;


    while (1){
       //UARTprintf("currAngle= %4i, outputSAT= %4i, distrb:= %4i, left:= %4i, right:= %4i    End\r", (int)euler_p, (int)outputSAT, (int)disturbance, leftMotorPos - (UINT32_MAX / 2), rightMotorPos - (UINT32_MAX / 2));
        UARTprintf("LRPM = %4i, RRPM = %4i, leftVelocity = %4i, rightVelocity = %4i \r",(int)LRPM, (int)RRPM, leftMotorVel, rightMotorVel);
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
    //static double disturbancePercent;
    //static int32_t rightOutput;
    static uint32_t dir;
    //static double disturbance;

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    //test velocity encoders
    qeiGetVelocity(&leftMotorVel, &rightMotorVel);

    LRPM = (SysCtlClockGet()*leftMotorVel*60) / (2500*480*4);
    RRPM = (SysCtlClockGet()*rightMotorVel*60) / (2500*480*4);

    //rpm = (clock * (2 ^ VELDIV) * SPEED * 60) ÷ (LOAD * ppr * edges)
    //////////////////////////////////


    // Do the thing
    GetEulerAngles(&euler_h, &euler_r, &euler_p);  //read values from the IMU, unit: degree
    qeiGetPos(&leftMotorPos, &rightMotorPos);  //read values from the encoders, unit: mm


    disturbancePercent = (double) positionPid(UINT32_MAX / 2, rightMotorPos, pkp, pki, pkd, pkc);

    disturbance = pa * disturbancePercent / 100.0;

//    //bind pitch disturbance
//    if(disturbance > 0) disturbance = pa;
//    if(disturbance < 0) disturbance = -pa;

    outputSAT = pid( (re + disturbance), euler_p, kp, ki, kd, kc ); //setpoint deg., measurement, kp, ki, kd,
                                                                  //give-up angle deg., full-power angle deg.,
                                                                  //sample-time
    // determine direction
    if (outputSAT < 0) dir = REVERSE;
    else dir = FORWARD;

    // system has failed? turn off motors
    if (fabs(euler_p) > 80.0) outputSAT = 0.0;

    // command motors
    mtrDrvSpeed(MOTOR_LEFT, dir, abs(outputSAT) );
    mtrDrvSpeed(MOTOR_RIGHT, dir, abs(outputSAT) );




}

