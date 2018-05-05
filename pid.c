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

int32_t pid (double setPoint, double currAngle, double kp, double ki, double kd, double kc, double failAngle)
{
    volatile static double error;
    volatile static double lastError;
    volatile static double integral;
    volatile static double derivative;
    volatile static int32_t output;
    volatile static int32_t outputSAT;
    volatile static int32_t MAXoutput = 100;

    // C2000 math for error calc
    error = setPoint - currAngle;
    integral = integral + ki*kp*error + kc*(outputSAT - output);
    derivative = kd*kp*(error - lastError);
    lastError = error;

    output = (int32_t) (kp * error + integral + derivative);

    // Bind output -100% +/- 100%
    if(output > MAXoutput) outputSAT = MAXoutput;
    else if(output < -MAXoutput) outputSAT = -MAXoutput;
    else outputSAT = output;

    return outputSAT;

}

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

int32_t positionPid (double setPoint, double currDisp, double kp, double ki, double kd, double kc)
{
    volatile static double error;
    volatile static double lastError;
    volatile static double integral;
    volatile static double derivative;
    volatile static int32_t output;
    volatile static int32_t outputSAT;
    volatile static int32_t MAXoutput = 100;

    // C2000 math for error calc
    error = setPoint - currDisp;
    integral = integral + ki*kp*error + kc*(outputSAT - output);
    derivative = kd*kp*(error - lastError);
    lastError = error;

    output = (int32_t) (kp * error + integral + derivative);

    // Bind output -100% - 100%
    if(output > MAXoutput) outputSAT = MAXoutput;
    else if(output < -MAXoutput) outputSAT = -MAXoutput;
    else outputSAT = output;

    return outputSAT;

}
