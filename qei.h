/*
 * qei.h
 *
 *  Created on: Mar 31, 2018
 *      Author: hardyn
 */

#ifndef QEI_H_
#define QEI_H_

#include <stdint.h>

uint8_t qeiInit(uint32_t freq);

uint8_t qeiGetPos(uint32_t* leftMotor, uint32_t* rightMotor);

uint8_t qeiResetPos(void);

uint8_t qeiGetVelocity(int32_t* leftMotor, int32_t* rightMotor);


#endif /* QEI_H_ */
