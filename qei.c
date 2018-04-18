/*
 * qei.c
 *
 *  Created on: Mar 27, 2018
 *      Author: hardyn
 */




//*****************************************************************************
//
// qei.c - Example to demonstrate QEI on Tiva Launchpad
    //This setup uses QEI0 P6/PD7, in my testing an arcade trackball is connected.
    //You can also use QEI1 PC5/PC6 in which case you don't need the PD7 HWREG calls (note: I didn't test this)
//
//
//*****************************************************************************


#include "qei.h"
#include "mtrDrv.h"  //for MOTOR_LEFT and MOTOR_RIGHT macros

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
#include "driverlib/qei.h"


volatile int qeiPosition;


uint8_t qeiInit(void) {

    /*
     * Enable QEI Peripherals
     */
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  //enable GPIO pins
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //enable GPIO pins

    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);  //enable QEI0
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);  //enable QEI1

    /*
     * Unlock GPIO PD7 - Like PF0 its used for NMI
     */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    /*
     * Set Pins to be PHA0 and PHB0
     */
    ROM_GPIOPinConfigure(GPIO_PD6_PHA0);
    ROM_GPIOPinConfigure(GPIO_PD7_PHB0);

    /*
     * Set Pins to be PHA1 and PHB1
     */
    ROM_GPIOPinConfigure(GPIO_PC5_PHA1);
    ROM_GPIOPinConfigure(GPIO_PC6_PHB1);

    /*
     * Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7
     */
    ROM_GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /*
     * Set GPIO pins for QEI. PhA1 -> PE3, PhB1 ->PE4
     */
    ROM_GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    /*
     * Configure quadrature encoder, use an arbitrary top limit of 1000
     */
    ROM_QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), UINT32_MAX);
    ROM_QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), UINT32_MAX);

    /*
     * Enable the quadrature encoder.
     */
    ROM_QEIEnable(QEI0_BASE);
    ROM_QEIEnable(QEI1_BASE);

    //Set position so we can see if things are working
    ROM_QEIPositionSet(QEI0_BASE, 0);
    ROM_QEIPositionSet(QEI1_BASE, 0);

    return EXIT_SUCCESS;
}

uint32_t qeiGetPos(uint32_t motor) {

    // uint32_t posLeft, posRight;

    switch (motor) {
    case MOTOR_LEFT:
        return ROM_QEIPositionGet(QEI0_BASE);
        break;

    case MOTOR_RIGHT:
        return (-1 * ROM_QEIPositionGet(QEI1_BASE));
        break;

    default:
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
