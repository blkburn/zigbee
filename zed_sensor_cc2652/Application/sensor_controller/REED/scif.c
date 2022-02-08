/// \addtogroup module_scif_driver_setup
//@{
#ifdef SENSOR_REED


#include "scif.h"
#include "scif_framework.h"
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(inc/hw_types.h)
#include DeviceFamily_constructPath(inc/hw_memmap.h)
#include DeviceFamily_constructPath(inc/hw_aon_event.h)
#include DeviceFamily_constructPath(inc/hw_aon_rtc.h)
#include DeviceFamily_constructPath(inc/hw_aon_pmctl.h)
#include DeviceFamily_constructPath(inc/hw_aux_sce.h)
#include DeviceFamily_constructPath(inc/hw_aux_smph.h)
#include DeviceFamily_constructPath(inc/hw_aux_spim.h)
#include DeviceFamily_constructPath(inc/hw_aux_evctl.h)
#include DeviceFamily_constructPath(inc/hw_aux_aiodio.h)
#include DeviceFamily_constructPath(inc/hw_aux_timer01.h)
#include DeviceFamily_constructPath(inc/hw_aux_sysif.h)
#include DeviceFamily_constructPath(inc/hw_event.h)
#include DeviceFamily_constructPath(inc/hw_ints.h)
#include DeviceFamily_constructPath(inc/hw_ioc.h)
#include <string.h>
#if defined(__IAR_SYSTEMS_ICC__)
    #include <intrinsics.h>
#endif


// OSAL function prototypes
uint32_t scifOsalEnterCriticalSection(void);
void scifOsalLeaveCriticalSection(uint32_t key);




/// Firmware image to be uploaded to the AUX RAM
static const uint16_t pAuxRamImage[] = {
    /*0x0000*/ 0x140E, 0x0417, 0x140E, 0x0438, 0x140E, 0x0442, 0x140E, 0x045F, 0x140E, 0x0468, 0x140E, 0x0471, 0x140E, 0x047A, 0x8953, 0x9954, 
    /*0x0020*/ 0x8D29, 0xBEFD, 0x4553, 0x2554, 0xAEFE, 0x445C, 0xADB7, 0x745B, 0x545B, 0x7000, 0x7CA2, 0x68AB, 0x009F, 0x1431, 0x68AC, 0x00A0, 
    /*0x0040*/ 0x1431, 0x68AD, 0x00A1, 0x1431, 0x78A2, 0xF801, 0xFA01, 0xBEF2, 0x78A9, 0x68AB, 0xFD0E, 0x68AD, 0xED92, 0xFD06, 0x7CA9, 0x6440, 
    /*0x0060*/ 0x047F, 0x78A2, 0x8F1F, 0xED8F, 0xEC01, 0xBE01, 0xADB7, 0x8DB7, 0x755B, 0x555B, 0x78A7, 0x60BF, 0xEF27, 0xE240, 0xEF27, 0x7000, 
    /*0x0080*/ 0x7CA7, 0x047F, 0x6477, 0x0000, 0x18A9, 0x9D88, 0x9C01, 0xB60E, 0x109E, 0xAF19, 0xAA00, 0xB60A, 0xA8FF, 0xAF39, 0xBE07, 0x0CA2, 
    /*0x00A0*/ 0x8600, 0x88A0, 0x8F08, 0xFD47, 0x9DB7, 0x08A2, 0x8801, 0x8A01, 0xBEEB, 0x254F, 0xAEFE, 0x645B, 0x445B, 0x4477, 0x047F, 0x5656, 
    /*0x00C0*/ 0x655B, 0x455B, 0x0000, 0x0CA2, 0x0001, 0x0CA3, 0x14B7, 0x047F, 0x5657, 0x665B, 0x465B, 0x0000, 0x0CA2, 0x0002, 0x0CA3, 0x1416, 
    /*0x00E0*/ 0x047F, 0x5658, 0x675B, 0x475B, 0x0000, 0x0CA2, 0x0004, 0x0CA3, 0x1416, 0x047F, 0x765B, 0x565B, 0x86FF, 0x03FF, 0x0CA5, 0x645C, 
    /*0x0100*/ 0x78A4, 0x68A5, 0xED37, 0xB605, 0x0000, 0x0CA4, 0x7CAA, 0x6540, 0x0CA5, 0x78A5, 0x68A6, 0xFD0E, 0xF801, 0xE95A, 0xFD0E, 0xBE01, 
    /*0x0120*/ 0x6553, 0xBDB7, 0x700B, 0xFB96, 0x4453, 0x2454, 0xAEFE, 0xADB7, 0x6453, 0x2454, 0xA6FE, 0x7000, 0xFB96, 0xADB7, 0x0000, 0x00B0, 
    /*0x0140*/ 0x00B6, 0x00E1, 0x0000, 0x0000, 0x0000, 0xFFFF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0001, 0x0000, 
    /*0x0160*/ 0x009A, 0x8B56, 0x655B, 0x455B, 0x7656, 0xADB7, 0xADB7, 0x67BB, 0x08AF, 0x8A00, 0xBE14, 0x08AE, 0x8401, 0x0CAE, 0x08A4, 0x8201, 
    /*0x0180*/ 0x0CA4, 0x00C8, 0x8B82, 0x8623, 0x0322, 0x8B7E, 0x0035, 0x8B56, 0x655B, 0x455B, 0x7656, 0x6480, 0x0001, 0x0CAF, 0x04DF, 0x08AE, 
    /*0x01A0*/ 0x8A00, 0xBE06, 0x001A, 0x8B56, 0x655B, 0x455B, 0x7656, 0x04DD, 0x009A, 0x8B56, 0x655B, 0x455B, 0x7656, 0x0000, 0x0CAF, 0x47BB, 
    /*0x01C0*/ 0xADB7, 0x5656, 0x655B, 0x455B, 0xADB7
};


/// Look-up table that converts from AUX I/O index to MCU IOCFG offset
static const uint8_t pAuxIoIndexToMcuIocfgOffsetLut[] = {
    0, 68, 64, 60, 56, 52, 48, 44, 40, 36, 32, 28, 24, 20, 16, 12, 8, 4, 0, 120, 116, 112, 108, 104, 100, 96, 92, 88, 84, 80, 76, 72
};


/** \brief Look-up table of data structure information for each task
  *
  * There is one entry per data structure (\c cfg, \c input, \c output and \c state) per task:
  * - [31:20] Data structure size (number of 16-bit words)
  * - [19:12] Buffer count (when 2+, first data structure is preceded by buffering control variables)
  * - [11:0] Address of the first data structure
  */
static const uint32_t pScifTaskDataStructInfoLut[] = {
//  cfg         input       output      state       
    0x00000000, 0x00000000, 0x0010115C, 0x0010115E  // Button Debouncer
};




// No run-time logging task data structure signatures needed in this project




// No task-specific initialization functions




// No task-specific uninitialization functions




/** \brief Performs driver setup dependent hardware initialization
  *
  * This function is called by the internal driver initialization function, \ref scifInit().
  */
static void scifDriverSetupInit(void) {

    // Select SCE clock frequency in active mode
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = AON_PMCTL_AUXSCECLK_SRC_SCLK_HFDIV2;

    // Set the default power mode
    scifSetSceOpmode(AUX_SYSIF_OPMODEREQ_REQ_LP);

    // Initialize task resource dependencies
    scifInitIo(26, AUXIOMODE_INPUT, 1, 0);
    scifInitIo(11, AUXIOMODE_OUTPUT | (0 << BI_AUXIOMODE_OUTPUT_DRIVE_STRENGTH), -1, 0);
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) & (~AON_PMCTL_AUXSCECLK_PD_SRC_M)) | AON_PMCTL_AUXSCECLK_PD_SRC_SCLK_LF;
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) |= AON_RTC_CTL_RTC_4KHZ_EN;

} // scifDriverSetupInit




/** \brief Performs driver setup dependent hardware uninitialization
  *
  * This function is called by the internal driver uninitialization function, \ref scifUninit().
  */
static void scifDriverSetupUninit(void) {

    // Uninitialize task resource dependencies
    scifUninitIo(26, -1);
    scifUninitIo(11, -1);
    HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) = (HWREG(AON_PMCTL_BASE + AON_PMCTL_O_AUXSCECLK) & (~AON_PMCTL_AUXSCECLK_PD_SRC_M)) | AON_PMCTL_AUXSCECLK_PD_SRC_NO_CLOCK;
    HWREG(AON_RTC_BASE + AON_RTC_O_CTL) &= ~AON_RTC_CTL_RTC_4KHZ_EN;

} // scifDriverSetupUninit




/** \brief Re-initializes I/O pins used by the specified tasks
  *
  * It is possible to stop a Sensor Controller task and let the System CPU borrow and operate its I/O
  * pins. For example, the Sensor Controller can operate an SPI interface in one application state while
  * the System CPU with SSI operates the SPI interface in another application state.
  *
  * This function must be called before \ref scifExecuteTasksOnceNbl() or \ref scifStartTasksNbl() if
  * I/O pins belonging to Sensor Controller tasks have been borrowed System CPU peripherals.
  *
  * \param[in]      bvTaskIds
  *     Bit-vector of task IDs for the task I/Os to be re-initialized
  */
void scifReinitTaskIo(uint32_t bvTaskIds) {
    if (bvTaskIds & (1 << SCIF_BUTTON_DEBOUNCER_TASK_ID)) {
        scifReinitIo(26, 1, 0);
        scifReinitIo(11, -1, 0);
    }
} // scifReinitTaskIo




/// Driver setup data, to be used in the call to \ref scifInit()
const SCIF_DATA_T scifDriverSetup = {
    (volatile SCIF_INT_DATA_T*) 0x400E0144,
    (volatile SCIF_TASK_CTRL_T*) 0x400E0152,
    (volatile uint16_t*) 0x400E013C,
    0x0000,
    sizeof(pAuxRamImage),
    pAuxRamImage,
    pScifTaskDataStructInfoLut,
    pAuxIoIndexToMcuIocfgOffsetLut,
    0x0000,
    24,
    scifDriverSetupInit,
    scifDriverSetupUninit,
    (volatile uint16_t*) NULL,
    (volatile uint16_t*) NULL,
    NULL
};




// No task-specific API available


//@}

#endif
// Generated by DESKTOP-MMLJVDE at 2021-12-06 14:14:28.708
