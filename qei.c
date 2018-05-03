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


/************************************************************************************
 * Function: qeiInit
 * initialize encoders

 * argument: void
 * return: void
 * Author: Hardy Nelson & Kushant Gounder
 * Date: March.21/2018
 * Revision:
 *************************************************************************************/

uint8_t qeiInit(void) {

    /*
     * Enable QEI Peripherals
     */
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);  //enable GPIO pins
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);  //enable GPIO pins

    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);  //enable QEI0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);  //enable QEI1

    /*
     * Unlock GPIO PD7 - Like PF0 its used for NMI
     */
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    /*
     * Set Pins to be PHA0 and PHB0
     */
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    /*
     * Set Pins to be PHA1 and PHB1
     */
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    /*
     * Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7
     */
    GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);

    /*
     * Set GPIO pins for QEI. PhA1 -> PE3, PhB1 ->PE4
     */
    GPIOPinTypeQEI(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6);

    /*
     * Configure quadrature encoder, use an arbitrary top limit of 1000
     */
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), UINT32_MAX);
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A_B  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_QUADRATURE | QEI_CONFIG_NO_SWAP), UINT32_MAX);

    /*
     * Enable the quadrature encoder.
     */
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);

    //Set position so we can see if things are working
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);

    return EXIT_SUCCESS;
}

/************************************************************************************
 * Function: qeiGetPos
 * get position from each encoder for each wheel

 * argument:
 * leftmotor: pointer for variable holding left motor position
 * rightmotor: pointer for variable holding right motor position
 *
 * return: 0
 * Author: Hardy Nelson & Kushant Gounder
 * Date: March.21/2018
 * Revision:
 *************************************************************************************/

uint8_t qeiGetPos(uint32_t* leftMotor, uint32_t* rightMotor) {

    *leftMotor = QEIPositionGet(QEI0_BASE);
    *rightMotor = -QEIPositionGet(QEI1_BASE);

    return EXIT_SUCCESS;
}

/************************************************************************************
 * Function: qeiResetPos
 * reset encoder for each motor

 * argument: void
 *
 * return: 0
 * Author: Hardy Nelson & Kushant Gounder
 * Date: March.21/2018
 * Revision:
 *************************************************************************************/

uint8_t qeiResetPos(void) {

    QEIPositionSet(QEI0_BASE, UINT32_MAX / 2);
    QEIPositionSet(QEI1_BASE, UINT32_MAX / 2);

    return EXIT_SUCCESS;
}
