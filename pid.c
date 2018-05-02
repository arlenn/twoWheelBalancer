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


int32_t pid (double setPoint, double currAngle, double kp, double ki, double kd, double kc, double failAngle, double maxAng, double scanFreq, double minPercent)
{
    volatile static double error;
    volatile static double lastError;
    volatile static double integral;
    volatile static double derivative;
    volatile static double deltaTime;
    volatile static int32_t output;
    volatile static int32_t outputSAT;
    volatile static uint32_t dir;
    volatile static double MAXoutput = 100;

    /*
     * correct orientation
     */
    //if (currAngle >= 0.0) currAngle = currAngle - 180.0;
    //else currAngle = currAngle + 180.0;

    deltaTime = 1.0/scanFreq;

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
    //////////////////////////////////////////////////////

    if(abs(output) > MAXoutput){
        outputSAT = 100;
    }
    else{
       outputSAT = output;
    }

    /////////////////////////////////////////////////////

    /*
     * determine direction
     */
    if (outputSAT < 0) dir = REVERSE;
    else dir = FORWARD;

    /*
     * bound PWM duty
     */
   // if (fabs(outputSAT) > 100.0 ) outputSAT = 100.0;
   // else if (fabs(outputSAT) < minPercent ) outputSAT = minPercent;  //this min thing might not be a good idea?

    if (fabs(currAngle) > failAngle) outputSAT = 0.0;

    mtrDrvSpeed(MOTOR_LEFT, dir, (abs(outputSAT)) );
    mtrDrvSpeed(MOTOR_RIGHT, dir, (abs(outputSAT)) );

    return outputSAT;

}

