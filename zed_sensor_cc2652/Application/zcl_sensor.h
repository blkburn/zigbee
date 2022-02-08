/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor.h
  Revised:        $Date: 2013-04-22 14:49:05 -0700 (Mon, 22 Apr 2013) $
  Revision:       $Revision: 33994 $

  Description:    This file contains the Zigbee Cluster Library Home
                  Automation Sample Application.


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

#ifndef ZCL_SAMPLETEMPERATURESENSOR_H
#define ZCL_SAMPLETEMPERATURESENSOR_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "zcl.h"
#include "nvintf.h"
#ifndef CUI_DISABLE
#include "cui.h"
#endif
/*********************************************************************
 * CONSTANTS
 */
#define SENSOR_ENDPOINT            8

#define LIGHT_OFF                       0x00
#define LIGHT_ON                        0x01

// Application Events
#define SAMPLETEMPERATURESENSOR_TEMP_SEND_EVT   0x0001
#define SAMPLEAPP_END_DEVICE_REJOIN_EVT         0x0002
#define SENSOR_FORCE_UPDATE_SC_EVT              0x0003


// Green Power Events
#define SAMPLEAPP_PROCESS_GP_DATA_SEND_EVT              0x0100
#define SAMPLEAPP_PROCESS_GP_EXPIRE_DUPLICATE_EVT       0x0200
#define SAMPLEAPP_PROCESS_GP_TEMP_MASTER_EVT            0x0400

#define SAMPLEAPP_END_DEVICE_REJOIN_DELAY 1000
#define SENSOR_FORCE_UPDATE_SENSOR_CONTROLLER 1000  // Timeout value in milliseconds

/*********************************************************************
 * MACROS
 */
/*********************************************************************
 * TYPEDEFS
 */
extern SimpleDescriptionFormat_t zclSensor_SimpleDesc;

extern CONST zclAttrRec_t zclSensor_Attrs[];
extern CONST uint8_t zclSensor_NumAttributes;

/*********************************************************************
 * VARIABLES
 */
extern CONST zclAttrRec_t zclBasic_Attrs[];
extern CONST uint8_t zclBasic_NumAttributes;

extern uint8_t  zclSampleTemperatureSensor_OnOff;
extern uint16_t zclSensor_IdentifyTime;

// Temperature Measurement Cluster
extern int16_t zclTemperatureSensor_MeasuredValue;
extern const int16_t zclTemperatureSensor_MinMeasuredValue;
extern const int16_t zclTemperatureSensor_MaxMeasuredValue;

// Humidity Sensor Cluster
extern uint16_t zclHumiditySensor_MeasuredValue;
extern const uint16_t zclHumiditySensor_MinMeasuredValue;
extern const uint16_t zclHumiditySensor_MaxMeasuredValue;

// Illuminance Sensor Cluster
extern uint16_t zclIlluminanceSensor_MeasuredValue;
extern const uint16_t zclIlluminanceSensor_MinMeasuredValue;
extern const uint16_t zclIlluminanceSensor_MaxMeasuredValue;

extern uint16_t zclBatterySensor_MeasuredValue;
extern uint8_t zclBatteryPercentSensor_MeasuredValue;

// Occupancy cluster
extern uint8_t zclOccupancySensor_Occupancy;
extern const uint8_t zclOccupancySensor_OccupancySensorType;

extern const uint16_t zclPowerMeasurementSensor_Type;  // Phase A measurement
extern uint16_t zclPowerMeasurementSensor_RmsCurrent;
extern uint16_t zclPowerMeasurementSensor_ApparentPower;

extern uint8_t zclOccupancySensor_Occupancy;


//extern void zclSampleAppsUI_changeKeyCallback(Button_Handle _buttonHandle, Button_EventMask _buttonEvents);



/*********************************************************************
 * FUNCTIONS
 */

/*
 *  Reset all writable attributes to their default values.
 */
extern void zclSensor_ResetAttributesToDefaultValues(void);
#ifndef CUI_DISABLE
extern void zclSampleTemperatureSensor_UiActionChangeTemp(const char _input, char* _lines[3], CUI_cursorInfo_t * _curInfo);
#endif

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZCL_SAMPLETEMPERATURESENSOR_H */
