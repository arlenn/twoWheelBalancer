/*
 * uartTest.c
 *
 *  Created on: Mar 31, 2018
 *      Author: hardyn
 */

#include "uart.h"
#include "mtrDrv.h"
#include "euler.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/interrupt.h"

#include "uartstdio.h"

double kp = 0;
double ki = 0;
double kd = 0;
double kc = 0;

#define MAX_STR_LEN 24

/************************************************************************************
 * Function: uartInit
 * configure GPIO as UART for Bluetooth comms.
 * configured as lowest priorty task

 * argument: void
 * return: success/failure
 * Author: Hardy Nelson
 * Date:
 * Revision:
 *************************************************************************************/

uint8_t uartInit(void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC); //16Mhz crystal oscillator
    UARTStdioConfig(1, 38400, 16000000);  //port 1, baud-rate, 16MHz oscillator magic number

    IntPrioritySet(INT_UART1, 0xE0);  //lowest priority

    IntEnable(INT_UART1); //enable the UART interrupt

    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT); //only enable RX and TX interrupts

    return EXIT_SUCCESS;
}

/************************************************************************************
 * Function: UartGetK
 * text parser from UART for robot control

 * argument: void
 * return: void
 * Author: Hardy Nelson & Kushant Gounder
 * Date:
 * Revision:
 *************************************************************************************/

void UartGetK(void){

    static char buffer[MAX_STR_LEN];
    static uint32_t bufflen = MAX_STR_LEN;
    static char *delimeter = { " \n\r" };
    static char *command;
    static char *kpString;
    static char *kiString;
    static char *kdString;
    static char *kcString;

    UARTgets(buffer, bufflen);

    command = strtok(buffer,delimeter);

    if (!(strcmp(command, "st")))
    {
        mtrDrvEnable(MOTOR_LEFT, true);
        mtrDrvEnable(MOTOR_RIGHT, true);
    }

    else if (!(strcmp(command, "sp")))
    {
        mtrDrvEnable(MOTOR_LEFT, false);
        mtrDrvEnable(MOTOR_RIGHT, false);
    }

    else if (!(strcmp(command, "kp")))
    {
        kpString = strtok('\0', delimeter);
        kp = atof(kpString);
    }

    else if (!(strcmp(command, "ki")))
    {
        kiString = strtok('\0', delimeter);
        ki = atof(kiString);
    }

    else if (!(strcmp(command, "kd")))
    {
        kdString = strtok('\0', delimeter);
        kd = atof(kdString);
    }

    else if (!(strcmp(command, "kc")))
    {
        kcString = strtok('\0', delimeter);
        kc = atof(kcString);
    }

    else if (!(strcmp(command, "k")))
    {
        kpString = strtok('\0', delimeter);
        kp = atof(kpString);

        kiString = strtok('\0', delimeter);
        ki = atof(kiString);

        kdString = strtok('\0', delimeter);
        kd = atof(kdString);

        kcString = strtok('\0', delimeter);
        kc = atof(kcString);
    }

    else
    {
        UARTprintf("command not recognized\n");
    }
}

/************************************************************************************
 * Function: UARTIntHandler
 * ISR for UART Rx

 * argument: void
 * return: void
 * Author: Hardy Nelson
 * Date:
 * Revision:
 *************************************************************************************/

void UARTIntHandler(void)
{
    uint32_t ui32Status;
    ui32Status = UARTIntStatus(UART1_BASE, true); //get interrupt status
    UARTIntClear(UART1_BASE, ui32Status); //clear the asserted interrupts

    UartGetK();


}
