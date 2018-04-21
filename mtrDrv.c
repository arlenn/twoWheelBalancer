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

/*
 * mtrDrvInit()
 * Initialize the PWM and GPIO components to control the Pololu G2 motor driver
 */

uint8_t mtrDrvInit(uint8_t pwmDiv) {

    volatile uint32_t ui32Load;
    volatile uint32_t ui32PWMClock;

    /*
     * The PWM module is clocked by the system clock through a divider,
     * and that divider has a range of 2 to 64.
     */
    switch (pwmDiv) {
    case 64:  //64, system default
        ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
        break;
    default:
        return EXIT_FAILURE;
    }

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //Enable motion control hardware PWM1
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //GPIO as PWM pins

    /*
     * Left motor
     */
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinConfigure(GPIO_PD0_M1PWM0);

    /*
     * right motor
     */
    ROM_GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
    ROM_GPIOPinConfigure(GPIO_PD1_M1PWM1);

    PWMGenConfigure(PWM1_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);  //count-up, reset, count-up timer /|_/|_

    /*
     * Enable GPIO for motor direction
     */
    /*
     * Left motor
     */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);  //GPIO as Dir pin
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);

    /*
     * Right motor
     */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);  //GPIO as Dir pin
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5);

    /*
     * Set PWM frequency
     */

    /*
     * Period = 1 / PWM frequency * PWMClock
     */

    ui32PWMClock = (uint32_t) SysCtlClockGet() / pwmDiv;  //PWM = clock System clock / pwmDiv
    ui32Load = (ui32PWMClock / ((uint32_t) PWM_FREQUENCY)) - 1;

    ROM_PWMGenPeriodSet(PWM1_BASE, PWM_GEN_0, ui32Load);  //unit is ticks

    return EXIT_SUCCESS;
}

uint8_t mtrDrvEnable(uint32_t motor, uint8_t enable) {
    /*
     * PWM module 1, generator 0 needs to be enabled as an output and enabled to run.
     */

    switch (enable) {
    case false:  //disable

        if (motor == MOTOR_LEFT) ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, false);  //set PWM pin low
        if (motor == MOTOR_RIGHT) ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, false); //set PWM pin low

        ROM_PWMGenDisable(PWM1_BASE, PWM_GEN_0);  //disable PWM pulse-train

        break;

    case true:  //enable

        if (motor == MOTOR_LEFT) {
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, 0);  //no motor speed
            ROM_PWMOutputState(PWM1_BASE, PWM_OUT_0_BIT, true);  //set PWM pin active
        }

        if (motor == MOTOR_RIGHT) {
            ROM_PWMPulseWidthSet(PWM1_BASE, PWM_OUT_1, 0);  // no motor speed
            ROM_PWMOutputState(PWM1_BASE, PWM_OUT_1_BIT, true);  //set PWM pin active
        }

        ROM_PWMGenEnable(PWM1_BASE, PWM_GEN_0);  //enable PWM pulse-train
        break;

    default:
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

uint8_t mtrDrvSpeed (uint32_t motor, uint8_t dir, uint8_t duty) {

    volatile uint32_t pwmPeriod;
    pwmPeriod = ROM_PWMGenPeriodGet(PWM1_BASE, PWM_GEN_0);

    /*
     * one motor will have have it's direction reversed, as the driver output will be in the same direction
     * but the motor is oriented in the opposite direction.... or just do it in the wiring
     */

    //ROM_PWMPulseWidthSet(PWM1_BASE, motor, 0);

    switch (motor) {
    case MOTOR_LEFT:
        if (dir == REVERSE) GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, GPIO_PIN_5);
        else ROM_GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
        break;

    case MOTOR_RIGHT:
        if (dir == REVERSE) GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0);
        else ROM_GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, GPIO_PIN_5);
        break;

    default:
        return EXIT_FAILURE;

    }

    /*
     * ui32Load = 100% duty, ui32Load / 2 = 50%, etc.
     */
    ROM_PWMPulseWidthSet(PWM1_BASE, motor, pwmPeriod*duty/100 );




    return EXIT_SUCCESS;
}




