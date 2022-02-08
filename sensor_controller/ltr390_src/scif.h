/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     I2C Optical Sensor
  * - <b>Project file</b>:     C:/Users/BigBob/github/sensor_controller/ltr390.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI-RTOS
  * - <b>Tool version</b>:     2.7.0.155
  * - <b>Tool patches</b>:     None
  * - <b>Target chip</b>:      CC2652R1F, package QFN48 7x7 RGZ, revision E (2.1)
  * - <b>Created</b>:          2021-03-29 21:49:54.406
  * - <b>Computer</b>:         DESKTOP-MMLJVDE
  * - <b>User</b>:             BigBob
  *
  * No user-provided resource definitions were used to generate this driver.
  *
  * No user-provided procedure definitions were used to generate this driver.
  *
  * Do not edit the generated source code files other than temporarily for debug purposes. Any
  * modifications will be overwritten by the Sensor Controller Studio when generating new output.
  *
  * \section section_drv_modules Driver Modules
  * The driver is divided into three modules:
  * - \ref module_scif_generic_interface, providing the API for:
  *     - Initializing and uninitializing the driver
  *     - Task control (for starting, stopping and executing Sensor Controller tasks)
  *     - Task data exchange (for producing input data to and consume output data from Sensor Controller
  *       tasks)
  * - \ref module_scif_driver_setup, containing:
  *     - The AUX RAM image (Sensor Controller code and data)
  *     - I/O mapping information
  *     - Task data structure information
  *     - Driver setup data, to be used in the driver initialization
  *     - Project-specific functionality
  * - \ref module_scif_osal, for flexible OS support:
  *     - Interfaces with the selected operating system
  *
  * It is possible to use output from multiple Sensor Controller Studio projects in one application. Only
  * one driver setup may be active at a time, but it is possible to switch between these setups. When
  * using this option, there is one instance of the \ref module_scif_generic_interface and
  * \ref module_scif_osal modules, and multiple instances of the \ref module_scif_driver_setup module.
  * This requires that:
  * - The outputs must be generated using the same version of Sensor Controller Studio
  * - The outputs must use the same operating system
  * - The outputs must use different source code prefixes (inserted into all globals of the
  *   \ref module_scif_driver_setup)
  *
  *
  * \section section_project_info Project Description
  * Demonstrates use of:
  * - I2C Master high-level and low-level API, to operate the HDC2080 humidity and temperature sensor on
  * the LaunchPad SensorTag Kit (LPSTK).
  * - Timer 2, to pulse-width modulate the RGB LEDs on the LaunchPad SensorTag Kit (LPSTK).
  * 
  * The "I2C Temp and Humidity Sensor" task polls the HDC2080 sensor at 1 Hz, logs the results, and
  * reports the following events to the application:
  * - Temperature change by more than a configurable amount.
  * - Humidity change by more than a configurable amount.
  * - Temperature and humidity log buffers are full.
  * - Error handling: HDC2080 interrupt timeout.
  * - Error handling: I2C missing acknowledgment or SCL stretch timeout.
  * 
  * The application uses the "RGB LED Blinker" task to indicate temperature and humidity changes, and
  * errors:
  * - A blue or red blink indicates temperature decrease or increase, respectively.
  * - A yellow or green blink indicates humidity decrease or increase, respectively.
  * - Continuous red or blue blinking indicates error.
  * 
  * The application also prints temperature and humidity over UART to a terminal window.
  * 
  * BOARD SETUP (requires a LaunchPad, for example LAUNCHXL-CC1352R1):
  * - Remove all 11 jumpers on the pin row between the XDS110 and the device on the LaunchPad.
  * - Use the supplied cables to connect JTAG and UART from the LPSTK board to the LaunchPad.
  * 
  * See the header in the application source file ("main.c" or similar) for further details and
  * instructions. This file is located in the source code output directory.
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - I2C Temp and Humidity Sensor:
  *     - <b>I2C SCL</b>: DIO4
  *     - <b>I2C SDA</b>: DIO5
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_i2c_temp_and_humidity_sensor I2C Temp and Humidity Sensor
  * The task configures the HDC2080 sensor for autonomous temperature and humidity measurements at 1 Hz,
  * with interrupt on data ready.
  * 
  * The task uses a combination of high-level and low-level I2C API:
  * - High-level I2C procedures are used to initialize the HDC2080, and start measurements. For a series
  * of I2C device register accesses, this gives both smaller task code and reduces RAM code size.
  * - Low-level I2C procedures are used to read out the results of each measurement. This minimizes I2C
  * read operation overhead, and thereby reduces current consumption.
  * 
  * The task wakes up on the HDC2080 interrupt approximately every 1 second, or on Timer 0 Event Trigger
  * if no interrupt occurs for 1.5 seconds.
  * 
  * The task reports the following events to the application:
  * - Temperature change by more than a configurable amount.
  * - Humidity change by more than a configurable amount.
  * - Temperature and humidity log buffers are full.
  * - Error handling: HDC2080 interrupt timeout.
  * - Error handling: I2C missing acknowledgment or SCL stretch timeout.
  * 
  * It is possible to trigger the error conditions on purpose:
  * - Connect the interrupt pin to VDD to cause interrupt timeout.
  * - Connect I2C SCL to GND to cause SCL stretch timeout.
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     I2C Optical Sensor
  * - <b>Code prefix</b>:      -
  *
  * The driver setup module contains the generated output from the Sensor Controller Studio project:
  * - Location of task control and scheduling data structures in AUX RAM
  * - The AUX RAM image, and the size the image
  * - Task data structure information (location, size and buffer count)
  * - I/O pin mapping translation table
  * - Task resource initialization and uninitialization functions
  * - Hooks for run-time logging
  *
  * @{
  */
#ifndef SCIF_H
#define SCIF_H

#include <stdint.h>
#include <stdbool.h>
#include "scif_framework.h"
#include "scif_osal_tirtos.h"


/// Target chip name
#define SCIF_TARGET_CHIP_NAME_CC2652R1F
/// Target chip package
#define SCIF_TARGET_CHIP_PACKAGE_QFN48_7X7_RGZ

/// Number of tasks implemented by this driver
#define SCIF_TASK_COUNT 1

/// I2C Temp and Humidity Sensor: Task ID
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID 0


/// I2C Temp and Humidity Sensor: 
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_LTR_ADDR_I2C 166
/// I2C Temp and Humidity Sensor I/O mapping: I2C SCL
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_DIO_I2C_SCL 4
/// I2C Temp and Humidity Sensor I/O mapping: I2C SDA
#define SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_DIO_I2C_SDA 5


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// I2C Temp and Humidity Sensor: Task output data structure
typedef struct {
    uint16_t byte1;  ///< Relative humidity [1/64 %]
    uint16_t byte2;  ///< Temperature [1/64 degC]
    uint16_t byte3;  ///< 
    uint16_t status; ///< 
} SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T;


/// I2C Temp and Humidity Sensor: Task state structure
typedef struct {
    uint16_t i2cStatus; ///< I2C master status
} SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
        SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_STATE_T state;
    } i2cTempAndHumiditySensor;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E015C))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// RTC-based tick generation control
void scifStartRtcTicks(uint32_t tickStart, uint32_t tickPeriod);
void scifStartRtcTicksNow(uint32_t tickPeriod);
void scifStopRtcTicks(void);


#endif
//@}


// Generated by DESKTOP-MMLJVDE at 2021-03-29 21:49:54.406
