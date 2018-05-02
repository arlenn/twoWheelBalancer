/*
 * uart.h
 *
 *  Created on: Mar 31, 2018
 *      Author: hardyn
 */

#ifndef UART_H_
#define UART_H_

#include <stdint.h>

uint8_t uartInit(void);
void UartGetK(void);

extern double kp;
extern double ki;
extern double kd;
extern double kc;
extern double re;

#endif /* UART_H_ */
