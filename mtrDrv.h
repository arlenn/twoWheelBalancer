/*
 * mtrDrv.h
 *
 *  Created on: Mar 22, 2018
 *      Author: hardyn
 */
#include <stdint.h>
#ifndef MTRDRV_H_
#define MTRDRV_H_

/*
 * macros
 */
#define PWM_FREQUENCY 10000 //Hz
#define MOTOR_LEFT PWM_OUT_0
#define MOTOR_RIGHT PWM_OUT_1
#define FORWARD 0
#define REVERSE 1


uint8_t mtrDrvInit (uint8_t pwmDiv);

uint8_t mtrDrvEnable (uint32_t motor, uint8_t enable);  // true / false

uint8_t mtrDrvSpeed (uint32_t motor, uint8_t dir, uint8_t duty);  //MOTOR_LEFT / MOTOR_RIGHT, FORWARD / REVERSE, 0-100%

#endif /* MTRDRV_H_ */
