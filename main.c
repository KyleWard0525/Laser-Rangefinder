/*
 * main.c
 *
 *  Created on: Apr 27, 2022
 *      Author: user
 */
#include <stdio.h>
#include "lidar.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "uartstdio.h"
#include "TM4cUtils.h"

int main(int argc, char* argv[])
{
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_INT | SYSCTL_XTAL_16MHZ);

    // Enable interrupts
    IntMasterEnable();

    // Setup UART, Lidar, and Stepper motor
    initUART();
    Lidar_InitI2C();

    uint32_t lidar_whoami = Lidar_readRegister(WHOAMI);

    if(lidar_whoami == SLAVE_ADDR)
    {
        UARTprintf("\nTF-Luna device found!");
    }
    else if(lidar_whoami == WHOAMI)
    {
        UARTprintf("\nThis probably shouldn't happen");
    }
    else {
        UARTprintf("\nValue 0x%X is not slave address or whoami address", lidar_whoami);
    }

    Lidar_printConfig();

}
