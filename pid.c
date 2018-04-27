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
    volatile static double output;
    volatile static double outputSAT;
    volatile static uint32_t dir;
    volatile static double MAXoutput = 100;

    /*
     * correct orientation
     */
    //if (currAngle >= 0.0) currAngle = currAngle - 180.0;
    //else currAngle = currAngle + 180.0;

    deltaTime = 1.0/scanFreq;


    ///////////////////////////////////////////////////// first try for error calc
//    error = setPoint - currAngle;  //proportionals
//    integral = integral + (error * deltaTime);  //integral
//    derivative = (error - lastError) / deltaTime;  //derivative
//    lastError = error;
//
//    output = (error * kp) + (integral * ki) + (derivative * kd);  //in percent
    /////////////////////////////////////////////////////


    ///////////////////////////////////////////////////// C2000 math for error calc
    error = setPoint - currAngle;  //proportionals
    integral = integral + ki*kp*error*deltaTime + kc*(outputSAT - output);
    derivative = kd*kp*(error - lastError)/deltaTime;
    lastError = error;

    output = error + integral + derivative;
    if(fabs(output) > MAXoutput){
        outputSAT = 100;
    }
    else{
       outputSAT = output;
    }

    /////////////////////////////////////////////////////

    UARTprintf("currAngle= %4i, kp= %2i, ki= %2i, kd= %2i, kc= %2i, outputSAT= %4i\r", (int)currAngle, (int)kp, (int)ki, (int)kd, (int)kc, (int)outputSAT);

    /*
     * determine direction
     */
    if (outputSAT < 0.0) dir = REVERSE;
    else dir = FORWARD;

    /*
     * bound PWM duty
     */
   // if (fabs(outputSAT) > 100.0 ) outputSAT = 100.0;
   // else if (fabs(outputSAT) < minPercent ) outputSAT = minPercent;  //this min thing might not be a good idea?

    if (fabs(currAngle) > failAngle) outputSAT = 0.0;

    mtrDrvSpeed(MOTOR_LEFT, dir, (uint32_t) (fabs(outputSAT)) );
    mtrDrvSpeed(MOTOR_RIGHT, dir, (uint32_t) (fabs(outputSAT)) );

    return dir;
}
