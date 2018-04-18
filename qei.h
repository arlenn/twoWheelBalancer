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

uint32_t qeiGetPos(uint32_t motor);

#endif /* QEI_H_ */
