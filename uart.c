/*
 * uartTest.c
 *
 *  Created on: Mar 31, 2018
 *      Author: hardyn
 */

#include "uart.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "uartstdio.h"



uint8_t uartInit(void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC); //16Mhz crystal oscillator
    UARTStdioConfig(0, 115200, 16000000);  //port 0, baud-rate, 16MHz oscillator magic number

    return EXIT_SUCCESS;
}
