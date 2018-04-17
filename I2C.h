/*
 * I2C.h
 *
 *  Created on: Apr 13, 2018
 *      Author: Hardy Nelson & Kushant Gounder
 */

#include <bno055.h>


#ifndef I2C_H_
#define I2C_H_

void InitI2C0(void);
void BNO055_I2C_read_BB(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
void BNO055_I2C_write_BB(u8 dev_addr, u8 reg_addr, u8 reg_data);

#endif /* I2C_H_ */
