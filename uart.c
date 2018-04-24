/*
 * uartTest.c
 *
 *  Created on: Mar 31, 2018
 *      Author: hardyn
 */

#include "uart.h"
#include <string.h>
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

double kp = 3.5;
double ki = 0;
double kd = 0;

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


void UartGetK(void){

    char buffer[12];
    uint32_t bufflen = 12;
    char *delimeter = { " " };
    char *kpString;
    char *kiString;
    char *kdString;

    UARTgets(buffer, bufflen); // wait for carriage return from UART

    //parse string in for kp ki and kd then turn then into double

    kpString = strtok(buffer,delimeter);
    kp = atof(kpString);

    kiString = strtok('\0', delimeter);
    ki = atof(kiString);

    kdString = strtok('\0', delimeter);
    kd = atof(kdString);

}
