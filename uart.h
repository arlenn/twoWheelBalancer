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

extern double lmkp;
extern double lmki;
extern double lmkd;
extern double lmkc;
extern double lmre;

extern double rmkp;
extern double rmki;
extern double rmkd;
extern double rmkc;
extern double rmre;


#endif /* UART_H_ */
