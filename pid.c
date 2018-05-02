/*
 * pid.c
 *
 *  Created on: Apr 20, 2018
 *      Author: hardyn
 */

#include "mtrDrv.h"
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include "uartstdio.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"

/************************************************************************************
 * Function: pid
 * calculate duty cycle for  PWM
 * saturate duty cyle to be less than or equal to 100%
 * determine direction motors should move
 * check if robot has moved beyond fail angle
 * output PWM to motors

 * argument:
 * - setPoint: angle of which the robot is trying to stay (in degrees)
 * - currAngle: Euler angle of current angle of robot (in degrees)
 * - kp: gain for proportional error
 * - ki: gain for integral error
 * - kd: gain for derivative error
 * - kc: gain for integral anti-wind-up correction
 * - failAngle: angle at which the motors on the robot will turn off
 * - scanFreq: frequency of refresh for the IMU
 *
 * return: OutputSat: duty cycle of PWM
 * Author: Hardy Nelson & Kushant Gounder
 * Date: April.19/2018
 * Revision:
 *************************************************************************************/

int32_t pid (double setPoint, double currAngle, double kp, double ki, double kd, double kc, double failAngle, double scanFreq)
{
    volatile static double error;
    volatile static double lastError;
    volatile static double integral;
    volatile static double derivative;
    //volatile static double deltaTime;
    volatile static int32_t output;
    volatile static int32_t outputSAT;
    volatile static uint32_t dir;
    volatile static double MAXoutput = 100;

    //deltaTime = 1.0/scanFreq;

    //correction
    //currAngle = currAngle + 4.0;


    /*
    ///////////////////////////////////////////////////// first try for error calc
    error = setPoint - currAngle;  //proportionals
    integral = integral + (error * deltaTime);  //integral
    derivative = (error - lastError) / deltaTime;  //derivative
    lastError = error;

    output = (error * kp) + (integral * ki) + (derivative * kd);  //in percent
    /////////////////////////////////////////////////////
    */

    ///////////////////////////////////////////////////// C2000 math for error calc
    error = setPoint - currAngle;
    integral = integral + ki*kp*error + kc*(outputSAT - output);
    derivative = kd*kp*(error - lastError);
    lastError = error;

    output = (int32_t) (kp * error + integral + derivative);
    /////////////////////////////////////////////////////

    // Bind output 0 - 100%
    if(abs(output) > MAXoutput) outputSAT = 100;
    else outputSAT = output;

    // Add -2/+2 deg. dead-band
    //if(fabs(currAngle) < 0.5 ) outputSAT = 0.0;

    // determine direction
    if (outputSAT < 0) dir = REVERSE;
    else dir = FORWARD;

    // system has failed? turn off motors
    if (fabs(currAngle) > failAngle) outputSAT = 0.0;

    // command motors
    mtrDrvSpeed(MOTOR_LEFT, dir, (abs(outputSAT)) );
    mtrDrvSpeed(MOTOR_RIGHT, dir, (abs(outputSAT)) );

    return outputSAT;

}

