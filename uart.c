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

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);

    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC); //16Mhz crystal oscillator
    UARTStdioConfig(1, 38400, 16000000);  //port 0, baud-rate, 16MHz oscillator magic number

    return EXIT_SUCCESS;
}
