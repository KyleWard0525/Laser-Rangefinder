#ifndef TM4C_UTILS_H
#define TM4C_UTILS_H
/**
 * Common utility functions for working with the TM4C123GXL LaunchPad
 * development board
 *
 * kward
 */

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_nvic.h"
#include "inc/hw_gpio.h"                    // Defines Macros for GPIO hardware
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "inc/tm4c123gh6pm.h"
#include "utils/uartstdio.h"

#define ARRAY_SIZE(x) sizeof(x)/sizeof(x[0])

#define ADC_RESOLUTION 4096
#define MAX_ADC_SAMPLE_RATE 1000000
#define UART_BAUD_RATE 115200
#define RED_LED_PIN GPIO_PIN_1
#define BLUE_LED_PIN GPIO_PIN_2
#define GREEN_LED_PIN GPIO_PIN_3

/**
 * Clear the terminal screen
 */
void clearTerminal()
{
    UARTprintf("\e[1;1H\e[2J");
}

// Setup UART and its pins. Must be called before UARTprintf()
void initUART()
{
    // Enable UART peripheral port (GPIO Port A)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);


    // Configure UART receiver and transmitter pins
    GPIOPinConfigure(GPIO_PA0_U0RX);    // Receiver pin
    GPIOPinConfigure(GPIO_PA1_U0TX);    // Transmitter pin

    // Enable UART0 port
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Set UART clock to internal 16MHz clock
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);  // Setup in UART mode

    // Setup UART to use console IO at 115200 baud rate
    UARTStdioConfig(0, UART_BAUD_RATE, 16000000);

    const char* msg = "\r\nUART0 Initialized!";

    clearTerminal();

    // Set terminal text color to green and background black
    UARTprintf("\e[32;40m");

    // Print init message
    UARTprintf(msg);
}





/*
 * Delay for a specified number of milliseconds.
 *
 * To calculate how long to delay, we know the following:
 *
 * - Time (in seconds) of 1 clock cycle =
 *
 *          1 / SysCtlClockGet()
 *
 * - Since SysCtlDelay() uses 3 assembly instructions per iteration (ADD, CMP, CBZ),
 *   the time to execute 1 iteration of SysCtlDelay() in seconds is:
 *
 *          3 / SysCtlClockGet()
 *
 * Thus we can compute 1 second of delay to be:
 *
 *          SysCtlClockGet() / 3
 *
 * Meaning 1 millisecond of delay is:
 *
 *          SysCtlClockGet() / 3 / 1000
 *
 *
 * @param dur_ms Duration in milliseconds
 */
void delaySysCtl(uint32_t dur_ms)
{

    SysCtlDelay(dur_ms * (SysCtlClockGet() / 3 / 1000));
}

/**
 * Convert frequency in hertz to time in seconds
 */
float HzToSeconds(long hertz)
{
    return 1.0 / hertz;
}

/**
 * Get number of seconds for 1 clock cycle
 */
uint32_t secondsPerClockCycle()
{
    return 1 / SysCtlClockGet();
}


/**
 * Read the bit at a given position in a number
 */
uint8_t readBit(uint32_t data, uint8_t bitPos)
{
    return data & (1 << bitPos);
}

/**
 * Delay for a specified number of milliseconds
 */
void delay(float delayMs)
{
    // Convert ms to seconds
    float delaySeconds = delayMs / 1000.0;

    // Disable SysTick (so it can be configured/reconfigured)
    NVIC_ST_CTRL_R = 0x0;   //  NVIC SysTick Control Register

    // Set SysTick reload value based on clock frequency
    NVIC_ST_RELOAD_R = (SysCtlClockGet() * delaySeconds) - 1;   //  NVIC SysTick Reload Register

    // Reset current SysTick counter
    NVIC_ST_CURRENT_R = 0x0;

    // Set SysTick to use system clock and enable
    NVIC_ST_CTRL_R |= 0x00000101; // Bit0 = ENABLE, Bit2=CLK_SRC(1=System clock, 0=PLL/4)

    // Wait for SysTick Current counter to tick to 0 and COUNT bit of SysTick CTRL Register to be set
    while((NVIC_ST_CTRL_R & (1 << 16)) == 0)
    {
        ;
    }

}

/**
 * Convert the digital value read from one of the onboard ADCs
 * to the corresponding analog voltage reading
 *
 * @param reading - Digital value read from ADC
 * @param SysVoltage - System voltage being used (usually 3.3v or 5v)
 *
 * @return analog voltage reading
 */
float ADCReadingToAnalogVoltage(uint32_t reading, float SysVoltage)
{
    return (SysVoltage * reading) / ADC_RESOLUTION;
}

#endif
