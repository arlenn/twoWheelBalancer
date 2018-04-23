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




int32_t pid (double setPoint, double currAngle, double kp, double ki, double kd, double failAngle, double maxAng, double scanFreq, double minPercent)
{
    volatile static double error;
    volatile static double lastError;
    volatile static double integral;
    volatile static double derivative;
    volatile static double deltaTime;
    volatile static float output;
    volatile static uint32_t dir;

    /*
     * corrent orientation
     */
    if (currAngle > 0) currAngle = currAngle - 180.0;
    else currAngle = currAngle + 180.0;

    deltaTime = 1.0/scanFreq;

    //UARTprintf("currAngle= %i\n", (int)currAngle);

    error = setPoint - currAngle; //the P

    //UARTprintf("error= %i\n", (int)error);

    integral = integral + (error * deltaTime);

    //UARTprintf("integral= %i\n", (int)integral);

    derivative = (error - lastError) / deltaTime;

    //UARTprintf("derivative= %i\n", (int)derivative);

    output = (error * kp) + (integral * ki) + (derivative * kd);  //in percent

    //UARTprintf("output= %i\n", (int)output);

    lastError = error;

    UARTprintf("currAngle= %3i, error= %3i, integral= %3i, derivative= %3i, output= %3i, lastError= %3i\r", (int)currAngle, (int)error, (int)integral, (int)derivative, (int)output, (int)lastError);

    /*
     * determine direction
     */
    if (output < 0) dir = FORWARD;
    else dir = REVERSE;

    //UARTprintf("dir= %i\n", (int)dir);

    if (abs(output) > 100 ) output = 100;
    else if (abs(output) < minPercent ) output = minPercent;

    if (abs(currAngle) > failAngle) output = 0;

    mtrDrvSpeed(MOTOR_LEFT, dir, abs(output));
    mtrDrvSpeed(MOTOR_RIGHT, dir, abs(output));

    return dir;
}


