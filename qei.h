/*
 * qei.h
 *
 *  Created on: Mar 31, 2018
 *      Author: hardyn
 */

#ifndef QEI_H_
#define QEI_H_

#include <stdint.h>

uint8_t qeiInit(void);

uint8_t qeiGetPos(double* leftMotor, double* rightMotor);

uint8_t qeiResetPos(void);



#endif /* QEI_H_ */
