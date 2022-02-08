/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor.c
  Revised:        $Date: 2014-10-24 16:04:46 -0700 (Fri, 24 Oct 2014) $
  Revision:       $Revision: 40796 $

  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application implements a ZigBee Temperature Sensor, based on Z-Stack 3.0.

  This application is based on the common sample-application user interface. Please see the main
  comment in zcl_sampleapp_ui.c. The rest of this comment describes only the content specific for
  this sample application.

  Application-specific UI peripherals being used:

  - LEDs:
    LED1 is not used in this application

  Application-specific menu system:

    <SET LOCAL TEMP> Set the temperature of the local temperature sensor
      Up/Down changes the temperature
      This screen shows the following information:
        Line2:
          Shows the temperature of the local temperature sensor

*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include <Application/zcl_sensor.h>
#include "rom_jt_154.h"
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"
#include <string.h>
 #include <driverlib/sys_ctrl.h>

#include "bdb_interface.h"
#include "bdb_reporting.h"
#include "ti_drivers_config.h"

#include <ti/drivers/apps/Button.h>
#include <ti/drivers/apps/LED.h>

#include "nvintf.h"
#include "zstackmsg.h"
#include "zcl_port.h"

#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>
#include "zstackapi.h"
#include "util_timer.h"
#include "mac_util.h"

#include <aux_adc.h>

#if defined(SENSOR_AHT10)
#include <Application/sensor_controller/AHT10/scif.h>
#elif defined(SENSOR_SI7021)
#include <Application/sensor_controller/SI7021/scif.h>
#elif defined(SENSOR_LTR390)
#include <Application/sensor_controller/LTR390/scif.h>
#elif defined(SENSOR_APDS9930)
#include <Application/sensor_controller/ADPS9930/scif.h>
#elif defined(SENSOR_MOVEMENT)
#include <Application/sensor_controller/MOVEMENT/scif.h>
#elif defined(SENSOR_REED)
#include <Application/sensor_controller/REED/scif.h>
#elif defined(SENSOR_PWR_MEAS)
#include <Application/sensor_controller/POWER/scif.h>
#endif

/* Driver Header files */
#include <ti/drivers/ADC.h>
/* Driver configuration */
#include "ti_drivers_config.h"

#include "math.h"

#include "zcl_consts.h"

/*********************************************************************
 * MACROS
 */
#define GUI_LOCAL_TEMP    1

#define APP_TITLE "   Temp Sensor  "

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */


#ifdef SENSOR_AHT10
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
extern void process_AHT10_temp_humid_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output,uint8_t appServiceTaskId);
#define SCIF_TASK_ID SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID
#elif defined(SENSOR_SI7021)
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
extern void process_SI7021_temp_humid_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId);
#define SCIF_TASK_ID SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID

#elif defined(SENSOR_LTR390)
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
extern void process_LTR390_light_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId);
#define SCIF_TASK_ID SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID

#elif defined(SENSOR_APDS9930)
SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
extern void process_APDS9930_light_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId);
#define SCIF_TASK_ID SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_TASK_ID

#elif defined(SENSOR_MOVEMENT)
SCIF_BUTTON_DEBOUNCER_OUTPUT_T output;
extern void process_movement_sensor(SCIF_BUTTON_DEBOUNCER_OUTPUT_T *output, uint8_t appServiceTaskId);
#define SCIF_TASK_ID SCIF_BUTTON_DEBOUNCER_TASK_ID

#elif defined(SENSOR_REED)
SCIF_BUTTON_DEBOUNCER_OUTPUT_T output;
extern void process_door_sensor(SCIF_BUTTON_DEBOUNCER_OUTPUT_T *output, uint8_t appServiceTaskId);
#define SCIF_TASK_ID SCIF_BUTTON_DEBOUNCER_TASK_ID

#elif defined(SENSOR_PWR_MEAS)
SCIF_POWER_SENSOR_OUTPUT_T output;
extern uint16_t zclPowerMeasurementSensor_RmsCurrent;
extern uint16_t zclPowerMeasurementSensor_ApparentPower;
#define SCIF_TASK_ID SCIF_POWER_SENSOR_TASK_ID
extern void process_power_sensor(SCIF_POWER_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId, int32_t gain, int32_t offset);

#endif

// Semaphore used to post events to the sensor application thread
static Semaphore_Handle appSensorSemHandle;
static Semaphore_Struct appSensorSem;

/* ADC conversion result variables */
uint16_t adcValue0;
uint32_t adcValue0MicroVolt;
uint16_t batt = 0;
uint16_t batt_prev = 0;

void scTaskAlertCallback(void);

LED_Handle gGreenLedHandle;

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
static bool forcedSensorControllerUpdate = false;
static bool onNwk = false;
static bdbReportAttrCfgData_t report;

// Semaphore used to post events to the application thread
static Semaphore_Handle appSemHandle;
static Semaphore_Struct appSem;

/* App service ID used for messaging with stack service task */
static uint8_t  appServiceTaskId;
/* App service task events, set by the stack service task when sending a message */
static uint32_t appServiceTaskEvents;
static endPointDesc_t  zclSensorEpDesc = {0};

#if ZG_BUILD_ENDDEVICE_TYPE
static Clock_Handle EndDeviceRejoinClkHandle;
static Clock_Struct EndDeviceRejoinClkStruct;
#endif

#if defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
static Clock_Handle SensorControllerForceUpdateHandle;
static Clock_Struct SensorControllerForceUpdateStruct;
#endif


//#ifndef CUI_DISABLE
static uint16_t zclSensor_BdbCommissioningModes;
//#endif

// Passed in function pointers to the NV driver
static NVINTF_nvFuncts_t *pfnZdlNV = NULL;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void zclSensor_Init( void );
static void zclSensor_initialization(void);
static void zclSensor_process_loop(void);
static void zclSensor_initParameters(void);
static void zclSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t *bdbCommissioningModeMsg);

static void zclSensor_initializeClocks(void);
static void zclSensor_processZStackMsgs(zstackmsg_genericReq_t *pMsg);

static void zclSensor_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg);
#if ZG_BUILD_ENDDEVICE_TYPE
static void zclSensor_processEndDeviceRejoinTimeoutCallback(UArg a0);
#endif
#if defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
static void zclSensor_processSCForceUpdateTimeoutCallback(UArg a0);
#endif
static void zclSensor_processKey(uint8_t key, Button_EventMask buttonEvents, uint32_t duration);
#ifndef CUI_DISABLE
static void zclSensor_RemoveAppNvmData(void);
static void zclSampleTemperatureSensor_InitializeStatusLine(CUI_clientHandle_t gCuiHandle);
void zclSampleTemperatureSensor_UpdateStatusLine(void);
#endif

static void zclSensor_BasicResetCB( void );
//static void zclSampleTemperatureSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg);

// Functions to process ZCL Foundation incoming Command/Response messages
static uint8_t zclSensor_ProcessIncomingMsg( zclIncoming_t *msg );
#ifdef ZCL_READ
static uint8_t zclSensor_ProcessInReadRspCmd( zclIncoming_t *pInMsg );
#endif
#ifdef ZCL_WRITE
static uint8_t zclSensor_ProcessInWriteRspCmd( zclIncoming_t *pInMsg );
#endif
static uint8_t zclSensor_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg );
#ifdef ZCL_DISCOVER
static uint8_t zclSensor_ProcessInDiscCmdsRspCmd( zclIncoming_t *pInMsg );
static uint8_t zclSensor_ProcessInDiscAttrsRspCmd( zclIncoming_t *pInMsg );
static uint8_t zclSensor_ProcessInDiscAttrsExtRspCmd( zclIncoming_t *pInMsg );
#endif // ZCL_DISCOVER


/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * ZCL General Profile Callback table
 */
static zclGeneral_AppCallbacks_t zclSensor_CmdCallbacks =
{
  zclSensor_BasicResetCB,        // Basic Cluster Reset command
  NULL,                                           // Identfiy cmd
  NULL,                                           // Identify Query command
  NULL,                                           // Identify Query Response command
  NULL,                                           // Identify Trigger Effect command
#ifdef ZCL_ON_OFF
  NULL,             				                      // On/Off cluster command
  NULL,                                           // On/Off cluster enhanced command Off with Effect
  NULL,                                           // On/Off cluster enhanced command On with Recall Global Scene
  NULL,                                           // On/Off cluster enhanced command On with Timed Off
#endif
#ifdef ZCL_LEVEL_CTRL
  NULL,                                           // Level Control Move to Level command
  NULL,                                           // Level Control Move command
  NULL,                                           // Level Control Step command
  NULL,                                           // Level Control Stop command
  NULL,                                           // Level Control Move to Closest Frequency command
#endif
#ifdef ZCL_GROUPS
  NULL,                                           // Group Response commands
#endif
#ifdef ZCL_SCENES
  NULL,                                           // Scene Store Request command
  NULL,                                           // Scene Recall Request command
  NULL,                                           // Scene Response command
#endif
#ifdef ZCL_ALARMS
  NULL,                                           // Alarm (Response) commands
#endif
#ifdef SE_UK_EXT
  NULL,                                           // Get Event Log command
  NULL,                                           // Publish Event Log command
#endif
  NULL,                                           // RSSI Location command
  NULL                                            // RSSI Location Response command
};

// Setup the zcl attributes based on defines
typedef void (*addZclAttribs_t)(uint8_t);
typedef void (*zclAddEndPoints_t)(uint8_t, uint8_t);
typedef void (*zclGenerateReport_t)(void);
typedef void (*zclConfigureReport_t)(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId);
typedef uint8_t (*zlcGetBdbReport_t)(bdbReportAttrCfgData_t *report, bool check);

#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)
const addZclAttribs_t addZclAttribs = &zclAddMeasAttrs;
const zclAddEndPoints_t zclAddEndPoints = &zclAddMeasEndPoints;
const zclGenerateReport_t zclGenerateReport = &ZclTempHumidReport;
const zclConfigureReport_t zclConfigureReport = &ZclTempHumidConfigureReport;
const zlcGetBdbReport_t zlcGetBdbReport = &zlcGetTempHumidBdbReport;

#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)

const addZclAttribs_t addZclAttribs = &zclAddIllumAttrs;
const zclAddEndPoints_t zclAddEndPoints = &zclAddIllumEndPoints;
const zclGenerateReport_t zclGenerateReport = &ZclIllumReport;
const zclConfigureReport_t zclConfigureReport = &ZclIllumConfigureReport;
const zlcGetBdbReport_t zlcGetBdbReport = &zlcGetIllumBdbReport;

#elif defined(SENSOR_PWR_MEAS)

const addZclAttribs_t addZclAttribs = &zclAddAdcAttrs;
const zclAddEndPoints_t zclAddEndPoints = &zclAddAdcEndPoints;
const zclGenerateReport_t zclGenerateReport = &ZclAdcReport;
const zclConfigureReport_t zclConfigureReport = &ZclAdcConfigureReport;
const zlcGetBdbReport_t zlcGetBdbReport = &zlcGetAdcBdbReport;

#elif defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)

const addZclAttribs_t addZclAttribs = &zclAddButtonAttrs;
const zclAddEndPoints_t zclAddEndPoints = &zclAddButtonEndPoints;
const zclGenerateReport_t zclGenerateReport = &ZclButtonReport;
const zclConfigureReport_t zclConfigureReport = &ZclButtonConfigureReport;
const zlcGetBdbReport_t zlcGetBdbReport = &zlcGetButtonBdbReport;

#endif

/*******************************************************************************
 * @fn          sampleApp_task
 *
 * @brief       Application task entry point for the Z-Stack
 *              Sample Application
 *
 * @param       pfnNV - pointer to the NV functions
 *
 * @return      none
 */
void sampleApp_task(NVINTF_nvFuncts_t *pfnNV)
{
  // Save and register the function pointers to the NV drivers
  pfnZdlNV = pfnNV;
  zclport_registerNV(pfnZdlNV, ZCL_PORT_SCENE_TABLE_NV_ID);

  // Initialize application
  zclSensor_initialization();

  // No return from task process
  zclSensor_process_loop();
}

static sensorControllerStarted = false;

//// star the sensor controller
void sensorControllerStart() {

    uint16_t SensorControllerTickSecs = MAX(15, report.minReportInt);
//    uint16_t SensorControllerTickSecs = 15;

    uint32_t rtcTicks = SensorControllerTickSecs << 16;

    if (sensorControllerStarted) {

        // Stop the Sensor Controller task
#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021) || defined(SENSOR_PWR_MEAS) || defined(SENSOR_APDS9930) || defined(SENSOR_LTR390)
        while (scifWaitOnNbl(0) != SCIF_SUCCESS);
#endif
        SCIF_RESULT_T result = scifStopTasksNbl(1 << SCIF_TASK_ID);

#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021) || defined(SENSOR_PWR_MEAS) || defined(SENSOR_APDS9930) || defined(SENSOR_LTR390)
        scifStopRtcTicks();
        if ( result != SCIF_SUCCESS ) {
            return;
        }
#endif
    }

    if (!sensorControllerStarted) {
        // Initialize the SCIF operating system abstraction layer
        scifOsalInit();
    //    scifOsalRegisterCtrlReadyCallback(scCtrlReadyCallback);
        scifOsalRegisterTaskAlertCallback(scTaskAlertCallback);

        // Initialize the SCIF driver
        SCIF_RESULT_T result = scifInit(&scifDriverSetup);

    } else
    {
        // Start the Sensor Controller task
        scifResetTaskStructs(1 << SCIF_TASK_ID, 0);
    }
#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021) || defined(SENSOR_PWR_MEAS) || defined(SENSOR_APDS9930) || defined(SENSOR_LTR390)
    // Enable RTC ticks, with N Hz tick interval
    scifStartRtcTicksNow(rtcTicks);
#endif

    // Start the "I2C Temp and Humidity Sensor" Sensor Controller task
    scifStartTasksNbl(1 << SCIF_TASK_ID);

    sensorControllerStarted = true;


}

//////////// add sensor task

/*******************************************************************************
 * @fn          sensorApp_task
 *
 * @brief       Application task entry point for the processing
 *              the sensor controller data
 *
 * @param       none
 *
 * @return      none
 */

void sensorApp_task()
{
    /* create semaphores for messages / events
     */
    Semaphore_Params semSensorParam;
    Semaphore_Params_init(&semSensorParam);
    semSensorParam.mode = ti_sysbios_knl_Semaphore_Mode_BINARY;
    Semaphore_construct(&appSensorSem, 0, &semSensorParam);
    appSensorSemHandle = Semaphore_handle(&appSensorSem);

    ADC_Handle   adc;
    ADC_Params   params;
    int_fast16_t res;
    zstack_bdbRepChangedAttrValueReq_t Req;

    int32_t gain = AUXADCGetAdjustmentGain(AUXADC_REF_VDDS_REL);
    int32_t offset = AUXADCGetAdjustmentOffset(AUXADC_REF_VDDS_REL);

    ADC_init();

    /* Forever loop */
    for(;;)
    {
        /* Wait for response message */
        if(Semaphore_pend(appSensorSemHandle, BIOS_WAIT_FOREVER ))
        {

#ifdef SENSOR_AHT10
    process_AHT10_temp_humid_sensor(&output, appServiceTaskId);
#elif defined(SENSOR_SI7021)
    process_SI7021_temp_humid_sensor(&output, appServiceTaskId);
#elif defined(SENSOR_LTR390)
    process_LTR390_light_sensor(&output, appServiceTaskId);
#elif defined(SENSOR_APDS9930)
    process_APDS9930_light_sensor(&output, appServiceTaskId);
#elif defined(SENSOR_MOVEMENT)
    process_movement_sensor(&output, appServiceTaskId);
#elif defined(SENSOR_REED)
    process_door_sensor(&output, appServiceTaskId);
#elif defined(SENSOR_PWR_MEAS)
    process_power_sensor(&output, appServiceTaskId, gain, offset);
#endif

            if (true) {
                ADC_Params_init(&params);
                adc = ADC_open(CONFIG_ADC_0, &params);

                if (adc == NULL) {
                    continue;
                }

                /* Blocking mode conversion */
                res = ADC_convert(adc, &adcValue0);

                if (res == ADC_STATUS_SUCCESS) {

                    adcValue0MicroVolt = ADC_convertRawToMicroVolts(adc, adcValue0);
                    batt = (uint16_t) (adcValue0MicroVolt/1000);
                    if (batt != batt_prev) {
                        batt_prev = batt;
                        if (batt >= 3000) {
                            zclBatterySensor_MeasuredValue = 3000;
                            zclBatteryPercentSensor_MeasuredValue = (uint8_t) 200;
                        } else {
                            zclBatterySensor_MeasuredValue = batt;
                            zclBatteryPercentSensor_MeasuredValue = (uint8_t)( 2 * batt  / 30);
                        }
                    }
                }

                ADC_close(adc);
            }
        }
    }
}


// SCIF driver callback: Sensor Controller task code has generated an alert interrupt
void scTaskAlertCallback(void) {

    scifClearAlertIntSource();

#ifdef SENSOR_AHT10
    output = scifTaskData.i2cTempAndHumiditySensor.output;
#elif defined(SENSOR_SI7021)
    output = scifTaskData.i2cTempAndHumiditySensor.output;
#elif defined(SENSOR_LTR390)
    output = scifTaskData.i2cTempAndHumiditySensor.output;
#elif defined(SENSOR_APDS9930)
    output = scifTaskData.i2cTempAndHumiditySensor.output;
#elif defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
    output = scifTaskData.buttonDebouncer.output;
#elif defined(SENSOR_PWR_MEAS)
    output = scifTaskData.powerSensor.output;
#endif

    Semaphore_post(appSensorSemHandle);
    scifAckAlertEvents();

}

/////////////////////// sensor

/*******************************************************************************
 * @fn          zclSensor_initialization
 *
 * @brief       Initialize the application
 *
 * @param       none
 *
 * @return      none
 */
static void zclSensor_initialization(void)
{
    /* Initialize user clocks */
    zclSensor_initializeClocks();

    /* create semaphores for messages / events
     */
    Semaphore_Params semParam;
    Semaphore_Params_init(&semParam);
    semParam.mode = ti_sysbios_knl_Semaphore_Mode_COUNTING;
    Semaphore_construct(&appSem, 0, &semParam);
    appSemHandle = Semaphore_handle(&appSem);

    appServiceTaskId = OsalPort_registerTask(Task_self(), appSemHandle, &appServiceTaskEvents);

    //Initialize stack
    zclSensor_Init();
}



/*******************************************************************************
 * @fn      SetupZStackCallbacks
 *
 * @brief   Setup the Zstack Callbacks wanted
 *
 * @param   none
 *
 * @return  none
 */
static void SetupZStackCallbacks(void)
{
    zstack_devZDOCBReq_t zdoCBReq = {0};

    // Register for Callbacks, turn on:
    //  Device State Change,
    //  ZDO Match Descriptor Response,
    zdoCBReq.has_devStateChange = true;
    zdoCBReq.devStateChange = true;
    zdoCBReq.has_ieeeAddrRsp = true;
    zdoCBReq.ieeeAddrRsp = true;

    (void)Zstackapi_DevZDOCBReq(appServiceTaskId, &zdoCBReq);
}

#define SAMPLEAPP_KEY_EVT_UI                  0x0200
#define SENSOR_CONTROLLER_UPDATE              0x1000 // add a new event to handle changes in the sensor controller interupts


typedef void (* uiAppProcessKeyCB_t)(uint8_t key, Button_EventMask _buttonEvents, uint32_t duration);
static uint16_t events = 0;
static Button_Handle  keys;
static uiAppProcessKeyCB_t gpAppKeyCB;
static Button_Handle gLeftButtonHandle;


/*********************************************************************
 * @fn      zclSampleAppsUI_changeKeyCallback
 *
 * @brief   Key event handler function
 *
 * @param   keysPressed - keys to be process in application context
 *
 * @return  none
 */
void zclSampleAppsUI_changeKeyCallback(Button_Handle _buttonHandle, Button_EventMask _buttonEvents)
{
    if (_buttonEvents & (Button_EV_CLICKED | Button_EV_LONGCLICKED))
    {
        keys = _buttonHandle;

        events |= SAMPLEAPP_KEY_EVT_UI;

        // Wake up the application thread when it waits for clock event
        Semaphore_post(appSemHandle);
    }
}



static void UI_processKey(Button_Handle _buttonHandle,
                                 Button_EventMask _buttonEvents)
{
  if((_buttonHandle == gLeftButtonHandle) && gpAppKeyCB)
  {
      uint32_t duration = Button_getLastPressedDuration(_buttonHandle);
      gpAppKeyCB(CONFIG_BTN_LEFT, _buttonEvents, duration);
  }
//    if((_buttonHandle == gRightButtonHandle) && gpAppKeyCB)
//    {
//        gpAppKeyCB(CONFIG_BTN_RIGHT, _buttonEvents);
//    }
}


/*********************************************************************
 * @fn          zclSampleTemperatureSensor_Init
 *
 * @brief       Initialization function for the zclGeneral layer.
 *
 * @param       none
 *
 * @return      none
 */

static void zclSensor_Init( void )
{
#ifdef BDB_REPORTING
      zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};
#endif

  //Register Endpoint
  zclAddEndPoints(SENSOR_ENDPOINT, appServiceTaskId);

  // Register the ZCL General Cluster Library callback functions
  zclGeneral_RegisterCmdCallbacks( SENSOR_ENDPOINT, &zclSensor_CmdCallbacks );

  // Register the application's attribute list and reset to default values
  zclSensor_ResetAttributesToDefaultValues();
  addZclAttribs( SENSOR_ENDPOINT);
  zcl_registerAttrList( SENSOR_ENDPOINT, zclBasic_NumAttributes, zclBasic_Attrs );

  // Register the Application to receive the unprocessed Foundation command/response messages
  zclport_registerZclHandleExternal(SENSOR_ENDPOINT, zclSensor_ProcessIncomingMsg);

  //Write the bdb initialization parameters
  zclSensor_initParameters();

  //Setup ZDO callbacks
  SetupZStackCallbacks();

  zclConfigureReport(&report, appServiceTaskId);

  /* Initialize btns */
  Button_Params bparams;
  Button_Params_init(&bparams);
  gLeftButtonHandle = Button_open(CONFIG_BTN_LEFT, NULL, &bparams);

  gpAppKeyCB = zclSensor_processKey;

  // Set button callback
  Button_setCallback(gLeftButtonHandle, zclSampleAppsUI_changeKeyCallback);

  zclSensor_BdbCommissioningModes = BDB_COMMISSIONING_MODE_NWK_STEERING | BDB_COMMISSIONING_MODE_FINDING_BINDING;

  LED_Params ledParams;
  LED_Params_init(&ledParams);
  gGreenLedHandle = LED_open(CONFIG_LED_GREEN, &ledParams);

    // have reporting data - update the sensor controller update clock
    events |= SENSOR_CONTROLLER_UPDATE;
    // Wake up the application thread when it waits for clock event
    Semaphore_post(appSemHandle);

  // Call BDB initialization. Should be called once from application at startup to restore
  // previous network configuration, if applicable.
  zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
  zstack_bdbStartCommissioningReq.commissioning_mode = 0;
  Zstackapi_bdbStartCommissioningReq(appServiceTaskId,&zstack_bdbStartCommissioningReq);
}

/*********************************************************************
 * @fn          zclSampleTemperatureSensor_RemoveAppNvmData
 *
 * @brief       Callback when Application performs reset to Factory New Reset.
 *              Application must restore the application to default values
 *
 * @param       none
 *
 * @return      none
 */
static void zclSensor_RemoveAppNvmData(void)
{

}

static void zclSensor_initParameters(void)
{
    zstack_bdbSetAttributesReq_t zstack_bdbSetAttrReq;

    zstack_bdbSetAttrReq.bdbCommissioningGroupID              = BDB_DEFAULT_COMMISSIONING_GROUP_ID;
    zstack_bdbSetAttrReq.bdbPrimaryChannelSet                 = 0x07FFF800;//BDB_DEFAULT_PRIMARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.bdbScanDuration                      = BDB_DEFAULT_SCAN_DURATION;
    zstack_bdbSetAttrReq.bdbSecondaryChannelSet               = BDB_DEFAULT_SECONDARY_CHANNEL_SET;
    zstack_bdbSetAttrReq.has_bdbCommissioningGroupID          = TRUE;
    zstack_bdbSetAttrReq.has_bdbPrimaryChannelSet             = TRUE;
    zstack_bdbSetAttrReq.has_bdbScanDuration                  = TRUE;
    zstack_bdbSetAttrReq.has_bdbSecondaryChannelSet           = TRUE;

#if (ZG_BUILD_JOINING_TYPE)
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeAttemptsMax  = TRUE;
    zstack_bdbSetAttrReq.has_bdbTCLinkKeyExchangeMethod       = TRUE;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeAttemptsMax      = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_ATTEMPS_MAX;
    zstack_bdbSetAttrReq.bdbTCLinkKeyExchangeMethod           = BDB_DEFAULT_TC_LINK_KEY_EXCHANGE_METHOD;
#endif

    Zstackapi_bdbSetAttributesReq(appServiceTaskId, &zstack_bdbSetAttrReq);
}

/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_initializeClocks
 *
 * @brief   Initialize Clocks
 *
 * @param   none
 *
 * @return  none
 */
static void zclSensor_initializeClocks(void)
{
#if ZG_BUILD_ENDDEVICE_TYPE
    // Initialize the timers needed for this application
    EndDeviceRejoinClkHandle = UtilTimer_construct(
    &EndDeviceRejoinClkStruct,
    zclSensor_processEndDeviceRejoinTimeoutCallback,
    SAMPLEAPP_END_DEVICE_REJOIN_DELAY,
    0, false, 0);
#endif

#if defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
    // Initialize the timers needed for this application
    SensorControllerForceUpdateHandle = UtilTimer_construct(
    &SensorControllerForceUpdateStruct,
    zclSensor_processSCForceUpdateTimeoutCallback,
    SENSOR_FORCE_UPDATE_SENSOR_CONTROLLER,
    0, false, 0);

#endif
}

#if ZG_BUILD_ENDDEVICE_TYPE
/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_processEndDeviceRejoinTimeoutCallback
 *
 * @brief   Timeout handler function
 *
 * @param   a0 - ignored
 *
 * @return  none
 */
static void zclSensor_processEndDeviceRejoinTimeoutCallback(UArg a0)
{
    (void)a0; // Parameter is not used

    appServiceTaskEvents |= SAMPLEAPP_END_DEVICE_REJOIN_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(appSemHandle);
}
#endif

#if defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
static void zclSensor_processSCForceUpdateTimeoutCallback(UArg a0)
{
    (void)a0; // Parameter is not used

    appServiceTaskEvents |= SENSOR_FORCE_UPDATE_SC_EVT;

    // Wake up the application thread when it waits for clock event
    Semaphore_post(appSemHandle);
}
#endif

/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_process_loop
 *
 * @brief   Application task processing start.
 *
 * @param   none
 *
 * @return  void
 */
static void zclSensor_process_loop(void)
{
    /* Forever loop */
    for(;;)
    {
        zstackmsg_genericReq_t *pMsg = NULL;
        bool msgProcessed = FALSE;

        /* Wait for response message */
        if(Semaphore_pend(appSemHandle, BIOS_WAIT_FOREVER ))
        {
            /* Retrieve the response message */
            if( (pMsg = (zstackmsg_genericReq_t*) OsalPort_msgReceive( appServiceTaskId )) != NULL)
            {
                /* Process the message from the stack */
                zclSensor_processZStackMsgs(pMsg);
                // Free any separately allocated memory
                msgProcessed = Zstackapi_freeIndMsg(pMsg);
            }

            if((msgProcessed == FALSE) && (pMsg != NULL))
            {
                OsalPort_msgDeallocate((uint8_t*)pMsg);
            }

            if (events & SAMPLEAPP_KEY_EVT_UI)
            {
              UI_processKey(keys, Button_EV_CLICKED);
              events &= ~SAMPLEAPP_KEY_EVT_UI;
            }

            if (events & SENSOR_CONTROLLER_UPDATE)
            {
                // reconfigure sensor controller interupts
                sensorControllerStart();
                events &= ~SENSOR_CONTROLLER_UPDATE;
            }

#if ZG_BUILD_ENDDEVICE_TYPE
            if ( appServiceTaskEvents & SAMPLEAPP_END_DEVICE_REJOIN_EVT )
            {
                zstack_bdbRecoverNwkRsp_t zstack_bdbRecoverNwkRsp;
                Zstackapi_bdbRecoverNwkReq(appServiceTaskId,&zstack_bdbRecoverNwkRsp);
                appServiceTaskEvents &= ~SAMPLEAPP_END_DEVICE_REJOIN_EVT;

            }
#endif
#if defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
            if ( appServiceTaskEvents & SENSOR_FORCE_UPDATE_SC_EVT )
            {
                Semaphore_post(appSensorSemHandle);
                appServiceTaskEvents &= ~SENSOR_FORCE_UPDATE_SC_EVT;
            }
#endif


        }
    }
}




/*******************************************************************************
 * @fn      zclSampleTemperatureSensor_processZStackMsgs
 *
 * @brief   Process event from Stack
 *
 * @param   pMsg - pointer to incoming ZStack message to process
 *
 * @return  void
 */
static void zclSensor_processZStackMsgs(zstackmsg_genericReq_t *pMsg)
{
      switch(pMsg->hdr.event)
      {
          case zstackmsg_CmdIDs_AF_DATA_CONFIRM_IND:
          {
              /// test when we get this message
              zstackmsg_afDataConfirmInd_t *pInd;
              pInd = (zstackmsg_afDataConfirmInd_t*)pMsg;
              if (zlcGetBdbReport(&report, false) == BDBREPORTING_SUCCESS)
              {
                  events |= SENSOR_CONTROLLER_UPDATE;
                  // Wake up the application thread when it waits for clock event
                  Semaphore_post(appSemHandle);
              }
          }
          case zstackmsg_CmdIDs_BDB_NOTIFICATION:
              {
                  zstackmsg_bdbNotificationInd_t *pInd;
                  pInd = (zstackmsg_bdbNotificationInd_t*)pMsg;
                  zclSensor_ProcessCommissioningStatus(&(pInd->Req));
              }
              break;

          case zstackmsg_CmdIDs_BDB_IDENTIFY_TIME_CB:
              {
              }
              break;

          case zstackmsg_CmdIDs_BDB_BIND_NOTIFICATION_CB:
              {
              }
              break;

          case zstackmsg_CmdIDs_AF_INCOMING_MSG_IND:
              {
                  // Process incoming data messages
                  zstackmsg_afIncomingMsgInd_t *pInd;
                  pInd = (zstackmsg_afIncomingMsgInd_t *)pMsg;
                  zclSensor_processAfIncomingMsgInd( &(pInd->req) );
              }
              break;

#if (ZG_BUILD_JOINING_TYPE)
          case zstackmsg_CmdIDs_BDB_CBKE_TC_LINK_KEY_EXCHANGE_IND:
          {
            zstack_bdbCBKETCLinkKeyExchangeAttemptReq_t zstack_bdbCBKETCLinkKeyExchangeAttemptReq;
            /* Z3.0 has not defined CBKE yet, so lets attempt default TC Link Key exchange procedure
             * by reporting CBKE failure.
             */

            zstack_bdbCBKETCLinkKeyExchangeAttemptReq.didSuccess = FALSE;

            Zstackapi_bdbCBKETCLinkKeyExchangeAttemptReq(appServiceTaskId,
                                                         &zstack_bdbCBKETCLinkKeyExchangeAttemptReq);
          }
          break;

          case zstackmsg_CmdIDs_BDB_FILTER_NWK_DESCRIPTOR_IND:

           /*   User logic to remove networks that do not want to join
            *   Networks to be removed can be released with Zstackapi_bdbNwkDescFreeReq
            */

            Zstackapi_bdbFilterNwkDescComplete(appServiceTaskId);
          break;

#endif
          case zstackmsg_CmdIDs_DEV_STATE_CHANGE_IND:
              zstackmsg_devStateChangeInd_t *pInd =
                          (zstackmsg_devStateChangeInd_t *)pMsg;
              zstack_devStateChangeInd_t* pReq = &(pInd->req);
              if (pReq != NULL)
              {
                switch (pReq->state)
                {
                case zstack_DevState_DEV_ZB_COORD:
                case zstack_DevState_DEV_ROUTER:
                case zstack_DevState_DEV_END_DEVICE:
                    onNwk = true;
                    LED_stopBlinking(gGreenLedHandle);
                    LED_startBlinking(gGreenLedHandle, 500, 3);
                    break;
                default:
                    onNwk = false;
                    LED_stopBlinking(gGreenLedHandle);
                    LED_setOff(gGreenLedHandle);
                  break;
                }
              }
              else
              {
                  onNwk = false;
                  LED_stopBlinking(gGreenLedHandle);
                  LED_setOff(gGreenLedHandle);
               }
          break;

          ///////////////////
          case zstackmsg_CmdIDs_ZDO_IEEE_ADDR_RSP:
          {
              zstackmsg_zdoIeeeAddrRspInd_t *pInd =
                      (zstackmsg_zdoIeeeAddrRspInd_t *)pMsg;

              if(pInd->rsp.status == zstack_ZdpStatus_SUCCESS)
              {

                zstack_sysNwkInfoReadRsp_t  Rsp;
                //Get our IEEE address
                Zstackapi_sysNwkInfoReadReq(appServiceTaskId, &Rsp);

                uint8_t numCluster = 2;
#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)
                uint16 clusterIds[] = {ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, ZCL_CLUSTER_ID_GENERAL_POWER_CFG, ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY};
                numCluster = 3;
#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
                uint16 clusterIds[] = {ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT, ZCL_CLUSTER_ID_GENERAL_POWER_CFG};
#elif defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
                uint16 clusterIds[] = {ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING, ZCL_CLUSTER_ID_GENERAL_POWER_CFG};
#elif defined(SENSOR_PWR_MEAS)
                uint16 clusterIds[] = {ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT, ZCL_CLUSTER_ID_GENERAL_POWER_CFG};

#endif
                zAddrType_t dstaddr;
                OsalPort_memcpy(dstaddr.addr.extAddr, pInd->rsp.ieeeAddr, Z_EXTADDR_LEN);
                dstaddr.addrMode = Addr64Bit;
                uint8_t srcEp = SENSOR_ENDPOINT;
                uint8_t dstEp = 1;
                // The response to MT interface has to be pack into buf
                BindingEntry_t *result = bindAddEntry( srcEp, &dstaddr, dstEp, numCluster, clusterIds );

              }
          }

          /*
           * These are messages/indications from ZStack that this
           * application doesn't process.  These message can be
           * processed by your application, remove from this list and
           * process them here in this switch statement.
           */
          case zstackmsg_CmdIDs_DEV_PERMIT_JOIN_IND:
          case zstackmsg_CmdIDs_BDB_TC_LINK_KEY_EXCHANGE_NOTIFICATION_IND:
          case zstackmsg_CmdIDs_ZDO_DEVICE_ANNOUNCE:
          case zstackmsg_CmdIDs_ZDO_NWK_ADDR_RSP:
          case zstackmsg_CmdIDs_ZDO_NODE_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_POWER_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_SIMPLE_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_ACTIVE_EP_RSP:
          case zstackmsg_CmdIDs_ZDO_COMPLEX_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_USER_DESC_RSP:
          case zstackmsg_CmdIDs_ZDO_USER_DESC_SET_RSP:
          case zstackmsg_CmdIDs_ZDO_SERVER_DISC_RSP:
          case zstackmsg_CmdIDs_ZDO_END_DEVICE_BIND_RSP:
          case zstackmsg_CmdIDs_ZDO_BIND_RSP:
          case zstackmsg_CmdIDs_ZDO_UNBIND_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_NWK_DISC_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_LQI_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_RTG_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_BIND_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_LEAVE_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_DIRECT_JOIN_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_PERMIT_JOIN_RSP:
          case zstackmsg_CmdIDs_ZDO_MGMT_NWK_UPDATE_NOTIFY:
          case zstackmsg_CmdIDs_ZDO_SRC_RTG_IND:
          case zstackmsg_CmdIDs_ZDO_CONCENTRATOR_IND:
          case zstackmsg_CmdIDs_ZDO_LEAVE_CNF:
          case zstackmsg_CmdIDs_ZDO_LEAVE_IND:
          case zstackmsg_CmdIDs_SYS_RESET_IND:
          case zstackmsg_CmdIDs_AF_REFLECT_ERROR_IND:
          case zstackmsg_CmdIDs_ZDO_TC_DEVICE_IND:
              break;

          default:
              break;
      }

}



/*******************************************************************************
 *
 * @fn          zclSensor_processAfIncomingMsgInd
 *
 * @brief       Process AF Incoming Message Indication message
 *
 * @param       pInMsg - pointer to incoming message
 *
 * @return      none
 *
 */
static void zclSensor_processAfIncomingMsgInd(zstack_afIncomingMsgInd_t *pInMsg)
{
    afIncomingMSGPacket_t afMsg;

    /*
     * All incoming messages are passed to the ZCL message processor,
     * first convert to a structure that ZCL can process.
     */
    afMsg.groupId = pInMsg->groupID;
    afMsg.clusterId = pInMsg->clusterId;
    afMsg.srcAddr.endPoint = pInMsg->srcAddr.endpoint;
    afMsg.srcAddr.panId = pInMsg->srcAddr.panID;
    afMsg.srcAddr.addrMode = (afAddrMode_t)pInMsg->srcAddr.addrMode;
    if( (afMsg.srcAddr.addrMode == afAddr16Bit)
        || (afMsg.srcAddr.addrMode == afAddrGroup)
        || (afMsg.srcAddr.addrMode == afAddrBroadcast) )
    {
        afMsg.srcAddr.addr.shortAddr = pInMsg->srcAddr.addr.shortAddr;
    }
    else if(afMsg.srcAddr.addrMode == afAddr64Bit)
    {
        OsalPort_memcpy(afMsg.srcAddr.addr.extAddr, &(pInMsg->srcAddr.addr.extAddr), 8);
    }
    afMsg.macDestAddr = pInMsg->macDestAddr;
    afMsg.endPoint = pInMsg->endpoint;
    afMsg.wasBroadcast = pInMsg->wasBroadcast;
    afMsg.LinkQuality = pInMsg->linkQuality;
    afMsg.correlation = pInMsg->correlation;
    afMsg.rssi = pInMsg->rssi;
    afMsg.SecurityUse = pInMsg->securityUse;
    afMsg.timestamp = pInMsg->timestamp;
    afMsg.nwkSeqNum = pInMsg->nwkSeqNum;
    afMsg.macSrcAddr = pInMsg->macSrcAddr;
    afMsg.radius = pInMsg->radius;
    afMsg.cmd.DataLength = pInMsg->n_payload;
    afMsg.cmd.Data = pInMsg->pPayload;

    zcl_ProcessMessageMSG(&afMsg);
}




/*********************************************************************
 * @fn      zclSensor_ProcessCommissioningStatus
 *
 * @brief   Callback in which the status of the commissioning process are reported
 *
 * @param   bdbCommissioningModeMsg - Context message of the status of a commissioning process
 *
 * @return  none
 */
static void zclSensor_ProcessCommissioningStatus(bdbCommissioningModeMsg_t* bdbCommissioningModeMsg)
{
    switch(bdbCommissioningModeMsg->bdbCommissioningMode)
    {
      case BDB_COMMISSIONING_FORMATION:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:

        }
        else
        {
          //Want to try other channels?
          //try with bdb_setChannelAttribute
        }
      break;
      case BDB_COMMISSIONING_NWK_STEERING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
          //We are on the nwk, what now?
            zstack_zdoIeeeAddrReq_t Req_addr;
            Req_addr.startIndex = 0;
            Req_addr.type = zstack_NwkAddrReqType_SINGLE_DEVICE;
            Req_addr.nwkAddr = 0x0000; //pInd->rsp.nwkAddrOfInterest;
            Zstackapi_ZdoIeeeAddrReq(appServiceTaskId, &Req_addr);

        }
        else
        {
          //See the possible errors for nwk steering procedure
          //No suitable networks found
          //Want to try other channels?
          //try with bdb_setChannelAttribute
            onNwk = false;
        }
      break;
      case BDB_COMMISSIONING_FINDING_BINDING:
        if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_SUCCESS)
        {
          //YOUR JOB:
        }
        else
        {
          //YOUR JOB:
          //retry?, wait for user interaction?
        }
      break;
      case BDB_COMMISSIONING_INITIALIZATION:
        //Initialization notification can only be successful. Failure on initialization
        //only happens for ZED and is notified as BDB_COMMISSIONING_PARENT_LOST notification

        //YOUR JOB:
        //We are on a network, what now?

      break;
#if ZG_BUILD_ENDDEVICE_TYPE
    case BDB_COMMISSIONING_PARENT_LOST:
      if(bdbCommissioningModeMsg->bdbCommissioningStatus == BDB_COMMISSIONING_NETWORK_RESTORED)
      {
        //We did recover from losing parent
          onNwk = true;

      }
      else
      {
        //Parent not found, attempt to rejoin again after a fixed delay
        UtilTimer_setTimeout( EndDeviceRejoinClkHandle, SAMPLEAPP_END_DEVICE_REJOIN_DELAY );
        UtilTimer_start(&EndDeviceRejoinClkStruct);
      }
    break;
#endif
    }

#ifndef CUI_DISABLE
  UI_UpdateBdbStatusLine(bdbCommissioningModeMsg);
#endif
}

/*********************************************************************
 * @fn      zclSensor_BasicResetCB
 *
 * @brief   Callback from the ZCL General Cluster Library
 *          to set all the Basic Cluster attributes to default values.
 *
 * @param   none
 *
 * @return  none
 */
static void zclSensor_BasicResetCB( void )
{
  zclSensor_ResetAttributesToDefaultValues();
}


/******************************************************************************
 *
 *  Functions for processing ZCL Foundation incoming Command/Response messages
 *
 *****************************************************************************/


/*********************************************************************
 * @fn      zclSensor_ProcessIncomingMsg
 *
 * @brief   Process ZCL Foundation incoming message
 *
 * @param   pInMsg - pointer to the received message
 *
 * @return  uint8_t - TRUE if got handled
 */
static uint8_t zclSensor_ProcessIncomingMsg( zclIncoming_t *pInMsg)
{
  uint8_t handled = FALSE;
  switch ( pInMsg->hdr.commandID )
  {
#ifdef ZCL_READ
    case ZCL_CMD_READ_RSP:
      zclSensor_ProcessInReadRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
#ifdef ZCL_WRITE
    case ZCL_CMD_WRITE_RSP:
      zclSensor_ProcessInWriteRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
    case ZCL_CMD_DEFAULT_RSP:
      zclSensor_ProcessInDefaultRspCmd( pInMsg );
      handled = TRUE;
      break;
#ifdef ZCL_DISCOVER
    case ZCL_CMD_DISCOVER_CMDS_RECEIVED_RSP:
      zclSensor_ProcessInDiscCmdsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_CMDS_GEN_RSP:
      zclSensor_ProcessInDiscCmdsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_ATTRS_RSP:
      zclSensor_ProcessInDiscAttrsRspCmd( pInMsg );
      handled = TRUE;
      break;

    case ZCL_CMD_DISCOVER_ATTRS_EXT_RSP:
      zclSensor_ProcessInDiscAttrsExtRspCmd( pInMsg );
      handled = TRUE;
      break;
#endif
    default:
      break;
  }


  return handled;
}

#ifdef ZCL_READ
/*********************************************************************
 * @fn      zclSensor_ProcessInReadRspCmd
 *
 * @brief   Process the "Profile" Read Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInReadRspCmd( zclIncoming_t *pInMsg )
{
  zclReadRspCmd_t *readRspCmd;
  uint8_t i;

  readRspCmd = (zclReadRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < readRspCmd->numAttr; i++ )
  {
    // Notify the originator of the results of the original read attributes
    // attempt and, for each successfull request, the value of the requested
    // attribute
  }

  return ( TRUE );
}
#endif // ZCL_READ

#ifdef ZCL_WRITE
/*********************************************************************
 * @fn      zclSensor_ProcessInWriteRspCmd
 *
 * @brief   Process the "Profile" Write Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInWriteRspCmd( zclIncoming_t *pInMsg )
{
  zclWriteRspCmd_t *writeRspCmd;
  uint8_t i;

  writeRspCmd = (zclWriteRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < writeRspCmd->numAttr; i++ )
  {
    // Notify the device of the results of the its original write attributes
    // command.
  }

  return ( TRUE );
}
#endif // ZCL_WRITE

/*********************************************************************
 * @fn      zclSensor_ProcessInDefaultRspCmd
 *
 * @brief   Process the "Profile" Default Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDefaultRspCmd( zclIncoming_t *pInMsg )
{
  // zclDefaultRspCmd_t *defaultRspCmd = (zclDefaultRspCmd_t *)pInMsg->attrCmd;

  // Device is notified of the Default Response command.
  (void)pInMsg;

  return ( TRUE );
}

#ifdef ZCL_DISCOVER
/*********************************************************************
 * @fn      zclSensor_ProcessInDiscCmdsRspCmd
 *
 * @brief   Process the Discover Commands Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDiscCmdsRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverCmdsCmdRsp_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverCmdsCmdRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numCmd; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSensor_ProcessInDiscAttrsRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDiscAttrsRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverAttrsRspCmd_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverAttrsRspCmd_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}

/*********************************************************************
 * @fn      zclSensor_ProcessInDiscAttrsExtRspCmd
 *
 * @brief   Process the "Profile" Discover Attributes Extended Response Command
 *
 * @param   pInMsg - incoming message to process
 *
 * @return  none
 */
static uint8_t zclSensor_ProcessInDiscAttrsExtRspCmd( zclIncoming_t *pInMsg )
{
  zclDiscoverAttrsExtRsp_t *discoverRspCmd;
  uint8_t i;

  discoverRspCmd = (zclDiscoverAttrsExtRsp_t *)pInMsg->attrCmd;
  for ( i = 0; i < discoverRspCmd->numAttr; i++ )
  {
    // Device is notified of the result of its attribute discovery command.
  }

  return ( TRUE );
}
#endif // ZCL_DISCOVER


/*********************************************************************
 * @fn      zclSensor_processKey
 *
 * @brief   Key event handler function
 *
 * @param   key - key to handle action for
 *          buttonEvents - event to handle action for
 *
 * @return  none
 */

static void zclSensor_processKey(uint8_t key, Button_EventMask buttonEvents, uint32_t duration)
{
    if (buttonEvents & Button_EV_CLICKED)
    {
        // publish a zcl_SendReportCmd for temp and humidity
        if ((key == CONFIG_BTN_LEFT) && (duration < 1000))
        {
            if (onNwk) {

                // test getting the attributes
                zclGenerateReport();
                LED_startBlinking(gGreenLedHandle, 250, 2);

            } else {
                LED_startBlinking(gGreenLedHandle, 100, 5);

            }

        }
        //////////////////////////
        // commission if holding the button for 1 to 5 seconds
        if ((key == CONFIG_BTN_LEFT) && (duration >= 1000 & duration < 5000))
        {
            zstack_bdbStartCommissioningReq_t zstack_bdbStartCommissioningReq;
            zstack_bdbStartCommissioningReq.commissioning_mode = zclSensor_BdbCommissioningModes;
            Zstackapi_bdbStartCommissioningReq(appServiceTaskId,&zstack_bdbStartCommissioningReq);
            LED_startBlinking(gGreenLedHandle, 500, 3);
        }
        //////////////////////////
        // hardware reset if greater than 5 seconds
        if ((key == CONFIG_BTN_LEFT) && (duration > 5000))
        {
            onNwk = false;
            LED_startBlinking(gGreenLedHandle, 1000, 2);
            zclSensor_RemoveAppNvmData();
            Zstackapi_bdbResetLocalActionReq(appServiceTaskId);
            SysCtrlSystemReset();

        }
    }
}





