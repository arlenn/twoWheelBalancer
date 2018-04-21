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
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"




int8_t pid (double setPoint, double currAngle, double kp, double ki, double kd, double failAngle, double maxAng, double deltaTime, double minPercent)
{
    static double error;
    static int8_t output;
    static uint8_t dir;

    /*
     * corrent orientation
     */
    if (currAngle > 0) currAngle = currAngle - 180.0;
    else currAngle = currAngle + 180.0;

    error = setPoint - currAngle;

    output = (int8_t) (error * kp);  //in percent

    /*
     * determine direction
     */
    if (output < 0) dir = FORWARD;
    else dir = REVERSE;


    if (abs(output) > 100 ) output = 100;
    else if (abs(output) < minPercent ) output = minPercent;

    if (abs(currAngle) > failAngle) output = 0;

    mtrDrvSpeed(MOTOR_LEFT, dir, abs(output));
    mtrDrvSpeed(MOTOR_RIGHT, dir, abs(output));

    return dir;
}


