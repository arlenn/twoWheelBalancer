/*
 * tm4c123g.h
 *
 *  Created on: Mar 22, 2018
 *      Author: hardyn
 */

#ifndef TM4C123G_H_
#define TM4C123G_H_

#include <stdint.h>

/*
 * 0x0 /1 reserved Clock source frequency/1 SYSCTL_SYSDIV_1
 * 0x1 /2 reserved Clock source frequency/2 SYSCTL_SYSDIV_2
 * 0x2 /3 66.67 MHz Clock source frequency/3 SYSCTL_SYSDIV_3
 * 0x3 /4 50 MHz Clock source frequency/4 SYSCTL_SYSDIV_4
 * 0x4 /5 40 MHz Clock source frequency/5 SYSCTL_SYSDIV_5
 * 0x5 /6 33.33 MHz Clock source frequency/6 SYSCTL_SYSDIV_6
 * 0x6 /7 28.57 MHz Clock source frequency/7 SYSCTL_SYSDIV_7
 * 0x7 /8 25 MHz Clock source frequency/8 SYSCTL_SYSDIV_8
 * 0x8 /9 22.22 MHz Clock source frequency/9 SYSCTL_SYSDIV_9
 * 0x9 /10 20 MHz Clock source frequency/10 SYSCTL_SYSDIV_10
 * 0xA /11 18.18 MHz Clock source frequency/11 SYSCTL_SYSDIV_11
 * 0xB /12 16.67 MHz Clock source frequency/12 SYSCTL_SYSDIV_12
 * 0xC /13 15.38 MHz Clock source frequency/13 SYSCTL_SYSDIV_13
 * 0xD /14 14.29 MHz Clock source frequency/14 SYSCTL_SYSDIV_14
 * 0xE /15 13.33 MHz Clock source frequency/15 SYSCTL_SYSDIV_15
 * 0xF /16 12.5 MHz (default) Clock source frequency/16 SYSCTL_SYSDIV_16
 */
int tm4c123gInit (uint8_t clockSpeed);



#endif /* TM4C123G_H_ */
