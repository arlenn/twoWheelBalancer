/*
 * tm4c123g.c
 *
 *  Created on: Mar 22, 2018
 *      Author: hardyn
 */

/*
 * header files
 */

#include "tm4c123g.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/fpu.h"


int tm4c123gInit (uint8_t clockSpeed) {

    /*
     * the clock speed is evaluated as 200 / division
     * eg. 200MHz/5 = 40MHz
     */

    switch (clockSpeed) {
    case 5:  // 1/5, 40MHz
        ROM_SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);  //this MUST be in one line!
        break;
    default:
        return EXIT_FAILURE;
    }

    /*
     * enable FPU
     */
    ROM_FPULazyStackingEnable();
    ROM_FPUEnable();

    return EXIT_SUCCESS;
}

