/*
 * mtrDrv.c
 *
 *  Created on: Mar 22, 2018
 *      Author: hardyn
 *
 *  Based upon the TM4C123G LaunchPad Workshop Workbook
 *  http://software-dl.ti.com/trainingTTO/trainingTTO_public_sw/GSW-TM4C123G-LaunchPad/TM4C123G_LaunchPad_Workshop_Workbook.pdf
 */

/*
 * header files
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


/************************************************************************************
 * Function: mtrDrvInit
 * initialize GPIO for PWM generation

 * argument: main-clock division
 * return: success/fail
 * Author: Hardy Nelson
 * Date:
 * Revision:
 *************************************************************************************/

uint8_t mtrDrvInit(uint8_t pwmDiv) {

    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;

    /*
     * The PWM module is clocked by the system clock through a divider,
     * and that divider has a range of 2 to 64.
     */
    switch (pwmDiv) {
    case 64:  //64, system default
        SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
        break;
    default:
        return EXIT_FAILURE;
    }

    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //Enable motion control hardware PWM1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //GPIO as PWM pins

    /*
     * Left motor
     */
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    GPIOPinConfigure(GPIO_PD0_M1PWM0);

    /*
     * right motor
     */
    GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    GPIOPinConfigure(GPIO_PD1_M1PWM1);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);  //count-up, reset, count-up timer /|_/|_

    /*
     * Enable GPIO for motor direction
     */
    /*
     * Left motor
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  //GPIO as Dir pin
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    /*
     * Right motor
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  //GPIO as Dir pin
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);

    /*
     * Set PWM frequency
     */

    /*
     * Period = 1 / PWM frequency * PWMClock
     */

    ui32PWMClock = (uint32_t) SysCtlClockGet() / pwmDiv;  //PWM = clock System clock / pwmDiv
    ui32Load = (ui32PWMClock / ((uint32_t) PWM_FREQUENCY)) - 1;

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);  //unit is ticks

    return EXIT_SUCCESS;
}



/************************************************************************************
 * Function: mtrDrvEnable
 * enable PWM clock

 * argument: MOTOR_LEFT / MOTOR_RIGHT, true / false
 * return: success/fail
 * Author: Hardy Nelson
 * Date:
 * Revision:
 *************************************************************************************/

uint8_t mtrDrvEnable(uint32_t motor, uint8_t enable) {
    /*
     * PWM module 1, generator 0 needs to be enabled as an output and enabled to run.
     */

    switch (enable) {
    case false:  //disable

        if (motor == MOTOR_LEFT) PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);  //set PWM pin low
        if (motor == MOTOR_RIGHT) PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false); //set PWM pin low

        PWMGenDisable(PWM1_BASE, PWM_GEN_0);  //disable PWM pulse-train

        break;

    case true:  //enable

        if (motor == MOTOR_LEFT) {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 1);  //no motor speed
            PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);  //set PWM pin active
        }

        if (motor == MOTOR_RIGHT) {
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 1);  // no motor speed
            PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);  //set PWM pin active
        }

        PWMGenEnable(PWM1_BASE, PWM_GEN_0);  //enable PWM pulse-train
        break;

    default:
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}


/************************************************************************************
 * Function: mtrDrvSpeed
 * adjust PWM duty 1-99%

 * argument: MOTOR_LEFT / MOTOR_RIGHT, FORWARD/REVERSE, duty %
 * return: success/fail
 * Author: Hardy Nelson
 * Date:
 * Revision:
 *************************************************************************************/

uint8_t mtrDrvSpeed (uint32_t motor, uint8_t dir, uint8_t duty) {

    volatile uint32_t pwmPeriod;
    static volatile double u32Duty;
    pwmPeriod = PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0);

    /*
     * one motor will have have it's direction reversed, as the driver output will be in the same direction
     * but the motor is oriented in the opposite direction.... or just do it in the wiring
     */

    //PWMPulseWidthSet(PWM1_BASE, motor, 0);

    switch (motor) {
    case MOTOR_LEFT:
        if (dir == REVERSE) GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
        else GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
        break;

    case MOTOR_RIGHT:
        if (dir == REVERSE) GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
        else GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
        break;

    default:
        return EXIT_FAILURE;

    }

    u32Duty = (uint32_t) (pwmPeriod*duty/100.0);
    if (u32Duty <= 0) u32Duty = 1;
    else if (u32Duty > 99) u32Duty = 99;

    PWMPulseWidthSet(PWM1_BASE, motor, u32Duty);

    return EXIT_SUCCESS;
}




