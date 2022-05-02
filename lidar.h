#ifndef LIDAR_H
#define LIDAR_H

/*
 * lidar.h
 *
 *  API for interfacing with the TF-Luna LiDAR ranging sensor
 *
 *  TF-Luna Datasheet:
 *      https://www.robotshop.com/media/files/content/b/ben/pdf/tf-luna-8m-lidar-distance-sensor-instructions-manual.pdf
 *
 * NOTE: In order to enable I2C mode, pin 5 of the TF-Luna must be connected to GND
 *
 * kward
 */
#include <math.h>
#include "TM4cUtils.h"
#include "driverlib/i2c.h"
#include "inc/hw_i2c.h"
#include "lidar_memmap.h"

#define MIN_DIST 20                 //  Minimum range in cm, aka the "ranging deadzone"
#define MAX_DIST 800                //  Maximum range in cm (8m)
#define MIN_AMP 100                 //  Minimum signal strength indication for reading to be considered valid
#define MAX_AMP 65535               //  Maximum signal strength indication for reading to be considered valid
#define FOV 2                       //  Field of view (2 degrees)
#define FRAME_RATE 100              //  Frame rate in Hz
#define I2C_PORT GPIO_PORTD_BASE    //  GPIO port of the I2C module this device uses
#define I2C_SCL GPIO_PIN_0          //  GPIO pin to use for the I2C module's serial clock
#define I2C_SDA GPIO_PIN_1          //  GPIO pin to use for the I2C module's serial data line


/**
 * Data structure for storing gpio information about the lidar's i2c pins
 */
typedef struct lidar_pins_t {
    uint32_t port;              //  GPIO port
    uint32_t scl_pin;           //  Serial clock pin
    uint32_t sda_pin;           //  Serial data pin
    uint32_t update_port;       //  Data update port
    uint32_t update_pin;        //  Data update pin
} lidar_pins_t;

lidar_pins_t lidar_pins;

int debug_counter = 0;


/**
 * Read register value from the lidar
 */
uint32_t Lidar_readRegister(uint8_t reg)
{
    // Set I2C slave address and configure to write data
    I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false);

    // Specify which register we want to read from
    I2CMasterDataPut(I2C3_BASE, reg);

    // Initiate data transaction
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait for transaction to finish
    while(I2CMasterBusy(I2C3_BASE));

    // Set to read from slave address
    I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, true);

    // Read single byte
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait for transaction to finish
    while(I2CMasterBusy(I2C3_BASE));

    // Get data from register
    uint32_t data = I2CMasterDataGet(I2C3_BASE);

    // Check if there was any error in reading from the device register
    uint32_t err = I2CMasterErr(I2C3_BASE);

    if(err == I2C_MASTER_ERR_NONE)
    {
        return data;
    }
    else if(err == I2C_MASTER_ERR_ADDR_ACK)
    {
        UARTprintf("\nERROR: Failed to read from device due to an ADDR ACK error!\n");
    }
    else if(err == I2C_MASTER_ERR_DATA_ACK)
    {
        UARTprintf("\nERROR: Failed to read from device due to an DATA ACK error!\n");
    }
    else if(err == I2C_MASTER_ERR_ARB_LOST)
    {
        UARTprintf("\nERROR: Failed to read from device due to an ARB LOST error!\n");
    }
    return 0;
}



/**
 * Validate ranging data
 */
bool validateData(uint16_t dist, uint16_t amp)
{
    // Check distance
    if(dist <= MIN_DIST || dist > MAX_DIST)
    {
        return false;
    }
    // Check signal amplitude
    if(amp <= MIN_AMP || amp >= MAX_AMP)
    {
        return false;
    }

    return true;
}

/**
 * Handle data update signal from the LiDAR
 */
void handleDataUpdateSignal(void)
{
    // Clear interrupt source
    I2CMasterIntClear(I2C3_BASE);

    // TODO: Read new data from lidar

   uint16_t distance = Lidar_read16bit(DIST_LOW, DIST_HIGH);
   uint16_t amplitude = Lidar_read16bit(AMP_LOW, AMP_HIGH);

    if(validateData(distance, amplitude))
    {
        UARTprintf("\nDistance = %dcm\tAmplitude = %d", distance, amplitude);
    }

}


/**
 * Setup I2C communication
 */
void Lidar_InitI2C()
{
    // Enable I2C module 3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C3);

    // Enable GPIO Port D and C
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // Reset I2C module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C3);

    // Setup lidar pins
    lidar_pins.port = I2C_PORT;
    lidar_pins.scl_pin = I2C_SCL;
    lidar_pins.sda_pin = I2C_SDA;
    lidar_pins.update_port = GPIO_PORTC_BASE;
    lidar_pins.update_pin = GPIO_PIN_7;     //  PC7

    // Enable GPIO port for the I2C module
    SysCtlPeripheralEnable(lidar_pins.port);

    // Configure the gpio pins for the lidar to use I2C mode
    GPIOPinConfigure(GPIO_PD0_I2C3SCL);
    GPIOPinConfigure(GPIO_PD1_I2C3SDA);

    // Set PD0 to be used as the serial clock line for the I2c periph
    GPIOPinTypeI2CSCL(I2C_PORT, I2C_SCL);

    // Set PD1 to be used as the serial data line for this I2C module
    GPIOPinTypeI2C(I2C_PORT, I2C_SDA);

    // Initialize the I2C Master Block on I2C port 3 and set clock rate to the processor clock
    I2CMasterInitExpClk(I2C3_BASE, SysCtlClockGet(), false);
    I2CMasterIntEnable(I2C3_BASE);
    //I2CIntRegister(I2C3_BASE, handleDataUpdateSignal);

    UARTprintf("\nI2C_3 initialized!");
}



/**
 * Combine low byte and high byte into a 16-bit unsigned number
 */
uint16_t Lidar_read16bit(uint32_t lowAddr, uint32_t highAddr)
{
    // Read lower byte
    uint32_t lower = Lidar_readRegister(lowAddr);
    uint32_t higher = Lidar_readRegister(highAddr);

    /**
     * Left shift high data by 8 bits so it remains the high byte and
     * OR with lower byte to create the 16-bit data
     */
    return (higher << 8) | lower;
}

/**
 * Write a byte to a register on the device
 */
void Lidar_writeRegister(uint32_t reg, uint8_t data)
{
    // Set slave address for master to write to
    I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false);
    // Select register to write to
    I2CMasterDataPut(I2C3_BASE, reg);
    // Initiate data transaction
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_START);
    // Wait for transaction to complete
    while(I2CMasterBusy(I2C3_BASE));


    // Set slave address for master to write to
    // Write data to register
    I2CMasterDataPut(I2C3_BASE, data);
    // Initiate data transaction
    I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
    // Wait for transaction to complete
    while(I2CMasterBusy(I2C3_BASE));

    // Write 0x01 to SAVE register
   I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false);
   I2CMasterDataPut(I2C3_BASE, SAVE);
   I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
   while(I2CMasterBusy(I2C3_BASE));
   I2CMasterDataPut(I2C3_BASE, 0x01);
   I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
   while(I2CMasterBusy(I2C3_BASE));

   // Reboot device by writing 0x02 to the REBOOT register
   I2CMasterSlaveAddrSet(I2C3_BASE, SLAVE_ADDR, false);
   I2CMasterDataPut(I2C3_BASE, SHUTDOWN_REBOOT);
   I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
   while(I2CMasterBusy(I2C3_BASE));
   I2CMasterDataPut(I2C3_BASE, 0x02);
   I2CMasterControl(I2C3_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
   while(I2CMasterBusy(I2C3_BASE));
}



/**
 * Configure device
 */
void Lidar_Configure()
{
    //Lidar_writeRegister(FACTORY_RESET, 0x01);
    // Turn off device
    Lidar_writeRegister(ENABLE, 0x01);

    // Set to continuous (0x01 for Trigger)
    Lidar_writeRegister(MODE, 0x0);

    // Set output frequency to 100Hz
    Lidar_writeRegister(FPS_LOW, 0x64);
    Lidar_writeRegister(FPS_HIGH, 0x0);

    // Set amplitude threshold to 100A
    Lidar_writeRegister(AMP_THR_LOW, 0x64);
    Lidar_writeRegister(AMP_THR_HIGH, 0x01);

    // Set power mode to normal (0x01 for power saving)
    Lidar_writeRegister(POWER_MODE, 0x0);

    // Set minimum distance threshold to 20cm
    Lidar_writeRegister(MIN_DIST_LOW, 0x14);
    Lidar_writeRegister(MIN_DIST_HIGH, 0x0);

    // Set maximum distance threshold to 800cm
    Lidar_writeRegister(MAX_DIST_LOW, 0x20);
    Lidar_writeRegister(MAX_DIST_HIGH, 0x3);
}

/**
 * Print device's current configuration settings
 */
void Lidar_printConfig()
{
    // Read config data from device registers
    uint32_t slv_addr = Lidar_readRegister(WHOAMI);
    uint32_t mode = Lidar_readRegister(MODE);
    uint32_t enable = Lidar_readRegister(ENABLE);
    uint16_t frequency = Lidar_read16bit(FPS_LOW, FPS_HIGH);
    uint32_t powerMode = Lidar_readRegister(POWER_MODE);
    uint16_t ampMin = Lidar_read16bit(AMP_THR_LOW, AMP_THR_HIGH);
    uint16_t minDist = Lidar_read16bit(MIN_DIST_LOW, MIN_DIST_HIGH);
    uint16_t maxDist = Lidar_read16bit(MAX_DIST_LOW, MAX_DIST_HIGH)/10;

    // Strings for printing
    char* str_mode;
    char* str_enabled;
    char* str_powerMode;

    // Check mode
    if(mode == 0x00)
    {
        str_mode = "Continuous";
    }
    else if(mode == 0x01)
    {
        str_mode = "Trigger";
    }
    else {
        str_mode = "ERROR";
    }

    // Check if device is on or off
    if(enable == 0x01)
    {
        str_enabled = "ON";
    }
    else if(enable == 0x00)
    {
        str_enabled = "OFF";
    }
    else {
        str_enabled = "ERROR";
    }

    // Check device's power mode
    if(powerMode == 0x00)
    {
        str_powerMode = "Normal";
    }
    else if(powerMode = 0x01)
    {
        str_powerMode = "Power saving";
    }
    else {
        str_powerMode = "ERROR";
    }

    UARTprintf("\n\nTF-Luna LiDAR Config:\n");
    UARTprintf("-------------------------");
    UARTprintf("\nSlave address: 0x%X", slv_addr);
    UARTprintf("\nDevice mode: %s", str_mode);
    UARTprintf("\nDevice status: %s", str_enabled);
    UARTprintf("\nPower mode: %s", str_powerMode);
    UARTprintf("\nOutput frequency: %dHz", frequency);
    UARTprintf("\nAmplitude minimum threshold: %d", ampMin);
    UARTprintf("\nMinimum measure distance: %dcm", minDist);
    UARTprintf("\nMaximum measure distance: %dcm", maxDist);
}

/**
 * In order for a reading to be reliable, the reflection surface, that being
 * the surface of the object measured, must be at least the diameter of the light
 * spot. The light spot simply meaning the area of light coverage from the lidar at
 * that distance.
 *
 * As a result, the object surface being measured must have a minimum diameter of:
 *
 *      d = 2D * tan(B)
 *
 *  d = minimum diameter
 *  D = distance to object surface
 *  B = 1/2 FOV of the LiDAR
 */
double Lidar_ComputeMinObjDiameter(double objectDist)
{
    return 2 * objectDist * radToDeg(tan(FOV / 2));
}


#endif /* LIDAR_H */
