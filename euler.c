/*
 * euler.c
 * -

 *  Created on: Apr.13, 2018
 *      Author: Hardy Nelson & Kushant Gounder
 */

#include "euler.h"
#include "bno055.h"
#include "I2C.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"


/************************************************************************************
 * Function: InitI2C0
 * initialize BNO055 for reading absolute euler angles

 * argument:
 * return: void.
 * Author: Hardy Nelson & Kushant Gounder
 * Date:
 * Revision:
 *************************************************************************************/

void InitIMUEuler(void){

    InitI2C0(); //initialize master

    SysCtlDelay(13333333);  // 3clocks * 1/40e-6s = ~1.0 sec; IMU requires 600ms to become "alive"

    BNO055_I2C_write_BB(BNO055_I2C_ADDR1,BNO055_AXIS_MAP_SIGN_ADDR,0x05); // change orientation of sensor
    //BNO055_I2C_write_BB(BNO055_I2C_ADDR1,BNO055_AXIS_MAP_CONFIG_ADDR,0x21); // change orientation of sensor

    BNO055_I2C_write_BB(BNO055_I2C_ADDR1,BNO055_OPR_MODE_ADDR,DNOF); //set to DNOF mode
    BNO055_I2C_write_BB(BNO055_I2C_ADDR1,BNO055_UNIT_SEL_ADDR,0x00); //set to degree mode

}

void ResetIMU(void){

    BNO055_I2C_write_BB(BNO055_I2C_ADDR1,BNO055_OPR_MODE_ADDR,DNOF); //set to DNOF mode
    BNO055_I2C_write_BB(BNO055_I2C_ADDR1,BNO055_UNIT_SEL_ADDR,0x00); //set to degree mode
}





/************************************************************************************
 * Function: GetEulerAngles
 * get euler angles from sensor
 * convert raw angles into degrees

 * argument:
 * - euler_h_d: pointer to variable that will hold euler heading in degrees
 * - euler_r_d: pointer to variable that will hold euler roll in degrees
 * - euler_p_d: pointer to variable that will hold euler pitch in degrees
 * return: void.
 * Author: Hardy Nelson & Kushant Gounder
 * Date:
 * Revision:
 *************************************************************************************/
void GetEulerAngles(double *euler_h_d, double *euler_r_d, double *euler_p_d){

    u8 reg_data[7]; //array to store raw data
    s16 heading;
    s8 *headingptr;
    s16 roll;
    s8 *rollptr;
    s16 pitch;
    s8 *pitchptr;

    headingptr = (s8*)&heading;
    rollptr = (s8*)&roll;
    pitchptr = (s8*)&pitch;

    //get 6 bytes needed for euler angles from sensor
    BNO055_I2C_read_BB(BNO055_I2C_ADDR1,BNO055_EULER_H_LSB_ADDR,reg_data,EULER_RAW_DATA);

    //convert raw data from bytes to words

    *headingptr = reg_data[0];
    *(headingptr+1) = reg_data[1];
    *rollptr = reg_data[2];
    *(rollptr+1) = reg_data[3];
    *pitchptr = reg_data[4];
    *(pitchptr+1) = reg_data[5];

    //conversion to turn raw data into degrees

    *euler_h_d = (double)(heading/BNO055_EULER_DIV_DEG);
    *euler_r_d = (double)(roll/BNO055_EULER_DIV_DEG);
    *euler_p_d = (double)(pitch/BNO055_EULER_DIV_DEG);

}
