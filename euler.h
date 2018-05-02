/*
 * euler.h
 *
 *  Created on: Apr 13, 2018
 *      Author: Hardy Nelson & Kushant Gounder
 */


#ifndef euler_H_
#define euler_H_

#define DNOF 0x0C //data to send to BNO055 to configure to absolute made
#define EULER_RAW_DATA 6 //heading roll and pitch are all 2 bytes so we need 6 bytes from the IMU

void InitIMUEuler(void);


void GetEulerAngles(double *euler_h_d, double *euler_r_d, double *euler_p_d);


#endif /*euler_H_ */

 /*
  * RESET PIN MUST BE HELD HIGH FOR OPERATION
  */

/*
 * example code on how to use each function to get euler angles
 */

/*

int main(void)
{

    //set up variables to store the euler angles in degrees

double euler_h_d;
double euler_r_d;
double euler_p_d;

    //initialize the TIVA to be a master and the slave to have euler angles in degrees

InitIMUEuler();

    //put the address of the variables declared earlier in the function to get the data

    while(1){
        GetEulerAngles(&euler_h_d, &euler_r_d, &euler_p_d);
    }
}

*/
