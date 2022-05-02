/*
 * lidar_memmap.h
 *
 * Memory map of registers for controlling and interfacing with the
 * TF-Luna
 *
 * @author kward
 */

#ifndef LIDAR_MEMMAP_H
#define LIDAR_MEMMAP_H


/*********************************
 *
 *        Data Registers
 *
 *********************************/

#define DIST_LOW            0x00    //  Distance reading low byte (Unit: cm *depending upon config*)
#define DIST_HIGH           0x01    //  Distance reading high byte
#define AMP_LOW             0x02    //  Amplitude reading low byte
#define AMP_HIGH            0x03    //  Amplitude reading high byte
#define TEMP_LOW            0x04    //  Chip temperature reading low byte (Unit: 0.01 Celsius)
#define TEMP_HIGH           0x05    //  Chip temperature reading high byte
#define TIME_LOW            0x06    //  Timestamp low byte (Unit: seconds?)
#define TIME_HIGH           0x07    //  Timestamp high byte
#define ERROR_LOW           0x08    //  Error code low byte
#define ERROR_HIGH          0x09    //  Error code high byte



/*********************************
 *
 *  Product Information Registers
 *
 *********************************/

#define VERSION_REVISION    0x0A    //  Product version revision number
#define VERSION_MINOR       0x0B    //  Product version number (low byte?)
#define VERSION_MAJOR       0x0C    //  Product version number (high byte?)


/*********************************
 *
 *   Control/Config Registers
 *
 *********************************/

#define SLAVE_ADDR          0x10    //  Device's I2C slave address
#define SAVE                0x20    //  Write 0x01 to this address to save current settings
#define SHUTDOWN_REBOOT     0x21    //  Write 0x01 to shutdown. Write 0x02 to reboot
#define WHOAMI              0x22    //  R/W register containing the device's slave address (Initial value is 0x10)
#define MODE                0x23    //  0x00 for continuous ranging mode, 0x01 for trigger mode (Initiali value is 0x00)
#define TRIGGER             0x24    //  0x01 to trigger device measurement (ONLY IN TRIGGER MODE)
#define ENABLE              0x25    //  0x00 to turn LiDAR on, 0x01 to turn LiDAR off
#define FPS_LOW             0x26    //  Frame rate / Frequency low byte (default = 0x64 = 100kbps)
#define FPS_HIGH            0x27    //  Frame rate / Frequency high byte
#define POWER_MODE          0x28    //  Power mode. 0x00 for normal power, 0x01 for power saving. (default = 0x00)
#define FACTORY_RESET       0x29    //  0x01 to restore to factory settings
#define AMP_THR_LOW         0x2A    //  Amplitude threshold value low byte (default = 0x64 = 100)
#define AMP_THR_HIGH        0x2B    //  Amplitude threshold value high byte
#define DUMMY_DIST_LOW      0x2C    //  Dummy distance value low byte
#define DUMMY_DIST_HIGH     0x2D    //  Dummy distance value high byte
#define MIN_DIST_LOW        0x2E    //  Minimum distance in cm low byte
#define MIN_DIST_HIGH       0x2F    //  Minimum distance in cm high byte
#define MAX_DIST_LOW        0x30    //  Maximum distance in cm low byte
#define MAX_DIST_HIGH       0x31    //  Maximum distance in cm high byte



#endif /* LIDAR_MEMMAP_H */
