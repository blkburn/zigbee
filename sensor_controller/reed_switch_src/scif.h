/** \mainpage Driver Overview
  *
  * \section section_drv_info Driver Information
  * This Sensor Controller Interface driver has been generated by the Texas Instruments Sensor Controller
  * Studio tool:
  * - <b>Project name</b>:     Reed Switch debouncer
  * - <b>Project file</b>:     C:/Users/BigBob/github/sensor_controller/reed_switch.scp
  * - <b>Code prefix</b>:      -
  * - <b>Operating system</b>: TI-RTOS
  * - <b>Tool version</b>:     2.8.0.170
  * - <b>Tool patches</b>:     1, 2 and 3
  * - <b>Target chip</b>:      CC2652R1F, package QFN48 7x7 RGZ, revision E (2.1) or F (3.0)
  * - <b>Created</b>:          2022-01-31 16:05:05.101
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
  * Demonstrates use of GPIO and timer event triggers and event handler code to implement a low-power
  * button debouncer on the LaunchPad.
  * 
  * The System CPU application is notified when the button is pressed. The System CPU application then
  * toggles an LED on the LaunchPad.
  * 
  * See the header in the application source file ("main.c" or similar) for further details and
  * instructions. This file is located in the source code output directory.
  *
  *
  * \subsection section_io_mapping I/O Mapping
  * Task I/O functions are mapped to the following pins:
  * - Button Debouncer:
  *     - <b>I: Button pin</b>: DIO23
  *     - <b>O: green led</b>: DIO7
  *
  *
  * \section section_task_info Task Description(s)
  * This driver supports the following task(s):
  *
  *
  * \subsection section_task_desc_button_debouncer Button Debouncer
  * Debounces BTN-1 on the LaunchPad.
  * 
  * This example is not useful for actual applications, but demonstrates use of "Event Handler Code" and
  * alternating between timer and GPIO triggers.
  * 
  * An ALERT interrupt is generated to the System CPU application each time the button is pressed.
  *
  */




/** \addtogroup module_scif_driver_setup Driver Setup
  *
  * \section section_driver_setup_overview Overview
  *
  * This driver setup instance has been generated for:
  * - <b>Project name</b>:     Reed Switch debouncer
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

/// Button Debouncer: Task ID
#define SCIF_BUTTON_DEBOUNCER_TASK_ID 0


/// Button Debouncer: I/O level when button is pressed
#define SCIF_BUTTON_DEBOUNCER_BUTTON_PRESSED 0
/// Button Debouncer: I/O level when button is released
#define SCIF_BUTTON_DEBOUNCER_BUTTON_RELEASED 1
/// Button Debouncer I/O mapping: Button pin
#define SCIF_BUTTON_DEBOUNCER_DIO_I_BUTTON 23
/// Button Debouncer I/O mapping: green led
#define SCIF_BUTTON_DEBOUNCER_DIO_O_LED 7


// All shared data structures in AUX RAM need to be packed
#pragma pack(push, 2)


/// Button Debouncer: Task output data structure
typedef struct {
    uint16_t buttonState; ///< Current button state
} SCIF_BUTTON_DEBOUNCER_OUTPUT_T;


/// Button Debouncer: Task state structure
typedef struct {
    uint16_t isDebouncing; ///< Is button debouncing in progress?
} SCIF_BUTTON_DEBOUNCER_STATE_T;


/// Sensor Controller task data (configuration, input buffer(s), output buffer(s) and internal state)
typedef struct {
    struct {
        SCIF_BUTTON_DEBOUNCER_OUTPUT_T output;
        SCIF_BUTTON_DEBOUNCER_STATE_T state;
    } buttonDebouncer;
} SCIF_TASK_DATA_T;

/// Sensor Controller task generic control (located in AUX RAM)
#define scifTaskData    (*((volatile SCIF_TASK_DATA_T*) 0x400E015C))


// Initialized internal driver data, to be used in the call to \ref scifInit()
extern const SCIF_DATA_T scifDriverSetup;


// Restore previous struct packing setting
#pragma pack(pop)


// AUX I/O re-initialization functions
void scifReinitTaskIo(uint32_t bvTaskIds);


// No task-specific API available


#endif
//@}


// Generated by DESKTOP-MMLJVDE at 2022-01-31 16:05:05.101
