

/**
 * main.c
 */

#include "tm4c123g.h"
#include "mtrDrv.h"
#include "qei.h"
#include "uart.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"

#include "uartstdio.h"

int main(void) {

    volatile uint32_t leftMotorPos, rightMotorPos;

    tm4c123gInit(5);  //200Mhz / 5 = 40MHz system clock

    mtrDrvInit(64);  //PWM clock, divide system clock by 64, 2 -> 64, two's compliment

    qeiInit();

    uartInit();

    mtrDrvEnable(MOTOR_LEFT, true);
    mtrDrvEnable(MOTOR_RIGHT, true);

    mtrDrvSpeed(MOTOR_LEFT, FORWARD, 50);
    mtrDrvSpeed(MOTOR_RIGHT, FORWARD, 50);

    while (1) {
        leftMotorPos = qeiGetPos(MOTOR_LEFT);
        UARTprintf("left motor= %d ", leftMotorPos);
        ROM_SysCtlDelay(100000);
        rightMotorPos = qeiGetPos(MOTOR_RIGHT);
        UARTprintf("right motor= %d\n", rightMotorPos);
        ROM_SysCtlDelay(100000);
    }

    return 0;
}
