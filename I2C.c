/*
 * I2C.c
 * -

 *  Created on: Apr.13, 2018
 *      Author: Hardy Nelson & Kushant Gounder
 */

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




/************************************************************************************
 * Function: InitI2C0
 * initialize TIVA as master for I2c communication

 * argument:
 * return: void.
 * Author: Hardy Nelson & Kushant Gounder
 * Date:
 * Revision:
 *************************************************************************************/

void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);
}



/************************************************************************************
 * Function: BNO055_I2C_write_BB
 * write data to register in slave

 * argument:
 * -dev_addr: address of slave
 * -reg_addr: address of register you wish to write too
 * -reg_data: data to want to write to a register on slave
 * return:
 * Author: Hardy Nelson & Kushant Gounder
 * Date: April.13/2018
 * Revision:
 *************************************************************************************/


void BNO055_I2C_write_BB(u8 dev_addr, u8 reg_addr, u8 reg_data)
{

   //specify that we want to communicate to device address with an intended write to bus
   I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);

   //register to be read
   I2CMasterDataPut(I2C0_BASE, reg_addr);

   //send control byte and register address byte to slave device
   I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

   //wait for MCU to finish transaction
   while(I2CMasterBusy(I2C0_BASE));


   //specify data to be written to the above mentioned reg_data
   I2CMasterDataPut(I2C0_BASE, reg_data);

   //wait while checking for MCU to complete the transaction
   I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

   //wait for MCU & device to complete transaction
   while(I2CMasterBusy(I2C0_BASE));    //more magic

}


/************************************************************************************
 * Function: BNO055_I2C_read_BB
 * read data in slave register

 * argument:
 * - dev_addr: address of slave
 * - reg_addr: address of register you wish to read from
 * - reg_data: pointer to array you want to store raw euler data in
 * - cnt: number of bytes wished to be read
 * return:
 * Author:Hardy Nelson & Kushant Gounder
 * Date: April.13/2018
 * Revision:
 *************************************************************************************/
void BNO055_I2C_read_BB(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt)
{
    u32 x; //counter for for loop
    u8 inc = 0; //counter for incrementing pointer that is pointing to array holding raw euler data


   for(x=0; x < cnt ; x++)
        {
           //wait for MCU to finish transaction
           while(I2CMasterBusy(I2C0_BASE));

            //specify that we want to communicate to device address with an intended write to bus
            I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, false);

            //specify register to be read
            I2CMasterDataPut(I2C0_BASE, reg_addr);

            //send control byte and register address byte to slave device
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

            //wait for MCU to finish transaction
            while(I2CMasterBusy(I2C0_BASE));

            I2CMasterSlaveAddrSet(I2C0_BASE, dev_addr, true);

            //send control byte and read from the register from the MCU
            I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

            //wait for MCU to finish transaction
            while(I2CMasterBusy(I2C0_BASE));

            *(reg_data+inc) = I2CMasterDataGet(I2C0_BASE); //load raw data into array

            inc++;
            reg_addr++;

        }

}
