/**************************************************************************************************
  Filename:       zcl_sampletemperaturesensor_data.c
  Revised:        $Date: 2014-09-25 13:20:41 -0700 (Thu, 25 Sep 2014) $
  Revision:       $Revision: 40295 $


  Description:    Zigbee Cluster Library - sample device application.


  Copyright 2013-2014 Texas Instruments Incorporated. All rights reserved.

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
 * INCLUDES
 */
#include <Application/zcl_sensor.h>
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"


/*********************************************************************
 * CONSTANTS
 */

#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021) || defined(SENSOR_LTR390) || defined(SENSOR_APDS9930) || defined(SENSOR_MOVEMENT) || defined(SENSOR_REED) || defined(SENSOR_PWR_MEAS)
#define SENSOR_DEVICE_VERSION     0
#define SENSOR_FLAGS              0

#define SENSOR_HWVERSION          1
#define SENSOR_ZCLVERSION         BASIC_ZCL_VERSION

#define TEMPERATURESENSOR_MAX_MEASURED_VALUE  10000  // 27.00C
#define TEMPERATURESENSOR_MIN_MEASURED_VALUE  00  // 17.00C

#define HUMIDITYSENSOR_MAX_MEASURED_VALUE  10000  // 100%
#define HUMIDITYSENSOR_MIN_MEASURED_VALUE  00000  // 0%

#define ILLUMINANCESENSOR_MAX_MEASURED_VALUE  500000  // 100%
#define ILLUMINANCESENSOR_MIN_MEASURED_VALUE  00000  // 0%

#define DOOR_STATE_OPEN 1
#define DOOR_STATE_CLOSED 0

#define MOVEMENT_STATE_ACTIVE 1
#define MOVEMENT_STATE_NO_ACTIVITY 0
#define PIR_SENSOR_TYPE 0
#endif

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// Global attributes
const uint16_t zclSensor_basic_clusterRevision = 0x0002;
const uint16_t zclSensor_identify_clusterRevision = 0x0001;
const uint16_t zclSensor_temperature_clusterRevision = 0x0001;
const uint16_t zclSensor_humidity_clusterRevision = 0x0001;
const uint16_t zclSensor_battery_clusterRevision = 0x0001;

// Basic Cluster
const uint8_t zclSensor_HWRevision = SENSOR_HWVERSION;
const uint8_t zclSensor_ZCLVersion = SENSOR_ZCLVERSION;
const uint8_t zclSensor_ManufacturerName[] = { 11, 'S','p','r','i','n','g','f','i','e','l','d' };
#ifdef SENSOR_AHT10
const uint8_t zclSensor_ModelId[] = { 8, 'S','A','-','A','H','T','1','0' };
#endif
#ifdef SENSOR_SI7021
const uint8_t zclSensor_ModelId[] = { 9, 'S','A','-','S','I','7','0','2','1' };
#endif
#ifdef SENSOR_LTR390
const uint8_t zclSensor_ModelId[] = { 9, 'S','A','-','L','T','R','3','9','0' };
#endif
#ifdef SENSOR_APDS9930
const uint8_t zclSensor_ModelId[] = { 11, 'S','A','-','A','P','D','S','9','9','3','0' };
#endif
#ifdef SENSOR_MOVEMENT
const uint8_t zclSensor_ModelId[] = { 11, 'S','A','-','M','O','V','E','M','E','N','T' };
#endif
#ifdef SENSOR_REED
const uint8_t zclSensor_ModelId[] = { 7, 'S','A','-','D','O','O','R' };
#endif
#ifdef SENSOR_PWR_MEAS
const uint8_t zclSensor_ModelId[] = { 8, 'S','A','-','P','O','W','E','R' };
#endif

const uint8_t zclSensor_PowerSource = POWER_SOURCE_BATTERY;
uint8_t zclSensor_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;

// Identify Cluster
uint16_t zclSensor_IdentifyTime;

#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)

// Temperature Sensor Cluster
int16_t zclTemperatureSensor_MeasuredValue = TEMPERATURESENSOR_MIN_MEASURED_VALUE;
const int16_t zclTemperatureSensor_MinMeasuredValue = TEMPERATURESENSOR_MIN_MEASURED_VALUE;
const int16_t zclTemperatureSensor_MaxMeasuredValue = TEMPERATURESENSOR_MAX_MEASURED_VALUE;

// Humidity Sensor Cluster
uint16_t zclHumiditySensor_MeasuredValue = HUMIDITYSENSOR_MIN_MEASURED_VALUE;
const uint16_t zclHumiditySensor_MinMeasuredValue = HUMIDITYSENSOR_MIN_MEASURED_VALUE;
const uint16_t zclHumiditySensor_MaxMeasuredValue = HUMIDITYSENSOR_MAX_MEASURED_VALUE;

#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)

// Illuminance Sensor Cluster
uint16_t zclIlluminanceSensor_MeasuredValue = HUMIDITYSENSOR_MIN_MEASURED_VALUE;
const uint16_t zclIlluminanceSensor_MinMeasuredValue = ILLUMINANCESENSOR_MIN_MEASURED_VALUE;
const uint16_t zclIlluminanceSensor_MaxMeasuredValue = ILLUMINANCESENSOR_MAX_MEASURED_VALUE;

#elif defined(SENSOR_MOVEMENT)

// Occupancy Sensor Cluster
//uint8_t zclOccupancySensor_Occupancy = MOVEMENT_STATE_NO_ACTIVITY;
//const uint8_t zclOccupancySensor_OccupancySensorType = PIR_SENSOR_TYPE;

#elif defined(SENSOR_REED)

//// Occupancy Sensor Cluster
//uint8_t zclOccupancySensor_Occupancy = DOOR_STATE_CLOSED;
//const uint8_t zclOccupancySensor_OccupancySensorType = PIR_SENSOR_TYPE;

#elif defined(SENSOR_PWR_MEAS)

const uint16_t zclPowerMeasurementSensor_Type = 0x08;  // Phase A measurement
uint16_t zclPowerMeasurementSensor_RmsCurrent = 0;
uint16_t zclPowerMeasurementSensor_ApparentPower = 0;

#endif



uint16_t zclBatterySensor_MeasuredValue = 0;
uint8_t zclBatteryPercentSensor_MeasuredValue = 0;



CONST zclAttrRec_t zclBasic_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSensor_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSensor_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_IDENTIFIER,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSensor_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENVIRONMENT,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSensor_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_basic_clusterRevision
    }
  },
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSensor_IdentifyTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclSensor_identify_clusterRevision
    }
  },
  {
   ZCL_CLUSTER_ID_GENERAL_POWER_CFG,
    { // Attribute record
      ATTRID_POWER_CONFIGURATION_BATTERY_PERCENTAGE_REMAINING,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclBatteryPercentSensor_MeasuredValue
    }
  },
//  {
//   ZCL_CLUSTER_ID_GENERAL_POWER_CFG,
//    { // Attribute record
//      ATTRID_POWER_CONFIGURATION_BATTERY_VOLTAGE,
//      ZCL_DATATYPE_UINT8,
//      ACCESS_CONTROL_READ,
//      (void *)&zclSampleBatterySensor_MeasuredValue
//    }
//  },
  {
   ZCL_CLUSTER_ID_GENERAL_POWER_CFG,
    { // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_battery_clusterRevision
    }
  },
};

uint8_t CONST zclBasic_NumAttributes = ( sizeof(zclBasic_Attrs) / sizeof(zclBasic_Attrs[0]) );


/*********************************************************************
 * ATTRIBUTE DEFINITIONS - Uses REAL cluster IDs
 */

// NOTE: The attributes listed in the AttrRec must be in ascending order
// per cluster to allow right function of the Foundation discovery commands

CONST zclAttrRec_t zclSensor_Attrs[] =
{
  // *** General Basic Cluster Attributes ***
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_ZCL_VERSION,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_ZCLVersion
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,             // Cluster IDs - defined in the foundation (ie. zcl.h)
    {  // Attribute record
      ATTRID_BASIC_HW_VERSION,            // Attribute ID - Found in Cluster Library header (ie. zcl_general.h)
      ZCL_DATATYPE_UINT8,                 // Data Type - found in zcl.h
      ACCESS_CONTROL_READ,                // Variable access control - found in zcl.h
      (void *)&zclSensor_HWRevision  // Pointer to attribute variable
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_MANUFACTURER_NAME,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSensor_ManufacturerName
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_MODEL_IDENTIFIER,
      ZCL_DATATYPE_CHAR_STR,
      ACCESS_CONTROL_READ,
      (void *)zclSensor_ModelId
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_POWER_SOURCE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_PowerSource
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    { // Attribute record
      ATTRID_BASIC_PHYSICAL_ENVIRONMENT,
      ZCL_DATATYPE_ENUM8,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSensor_PhysicalEnvironment
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_basic_clusterRevision
    }
  },
  // *** Identify Cluster Attribute ***
  {
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    { // Attribute record
      ATTRID_IDENTIFY_IDENTIFY_TIME,
      ZCL_DATATYPE_UINT16,
      (ACCESS_CONTROL_READ | ACCESS_CONTROL_WRITE),
      (void *)&zclSensor_IdentifyTime
    }
  },
  {
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_GLOBAL,
      (void *)&zclSensor_identify_clusterRevision
    }
  },
#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)
  // *** Temperature Measurement Attriubtes ***
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclTemperatureSensor_MeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_TEMPERATURE_MEASUREMENT_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclTemperatureSensor_MinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    { // Attribute record
      ATTRID_TEMPERATURE_MEASUREMENT_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclTemperatureSensor_MaxMeasuredValue
    }
  },

  {
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_temperature_clusterRevision
    }
  },
  // Humidify measurements
  {
   ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // Attribute record
      ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclHumiditySensor_MeasuredValue
    }
  },
  {
   ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // Attribute record
      ATTRID_RELATIVITY_HUMIDITY_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclHumiditySensor_MinMeasuredValue
    }
  },
  {
   ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    { // Attribute record
      ATTRID_RELATIVITY_HUMIDITY_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclHumiditySensor_MaxMeasuredValue
    }
  },
  {
   ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    {  // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_humidity_clusterRevision
    }
  },
#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclIlluminanceSensor_MeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_ILLUMINANCE_MEASUREMENT_MIN_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclIlluminanceSensor_MinMeasuredValue
    }
  },
  {
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
    { // Attribute record
      ATTRID_ILLUMINANCE_MEASUREMENT_MAX_MEASURED_VALUE,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ,
      (void *)&zclIlluminanceSensor_MaxMeasuredValue
    }
  },
#elif defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)
  {
   ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    { // Attribute record
      ATTRID_OCCUPANCY_SENSING_OCCUPANCY,
      ZCL_DATATYPE_BITMAP8,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclOccupancySensor_Occupancy
    }
  },
  {
   ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
    { // Attribute record
      ATTRID_OCCUPANCY_SENSING_OCCUPANCY_SENSOR_TYPE,
      ZCL_DATATYPE_ENUM8,
      ACCESS_CONTROL_READ,
      (void *)&zclOccupancySensor_OccupancySensorType
    }
  },
#elif defined(SENSOR_PWR_MEAS)
  {
   ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT,
    { // Attribute record
      ATTRID_ELECTRICAL_MEASUREMENT_MEASUREMENT_TYPE,
      ZCL_DATATYPE_BITMAP32,
      ACCESS_CONTROL_READ,
      (void *)&zclPowerMeasurementSensor_Type
    }
  },
  {
   ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT,
    { // Attribute record
      ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclPowerMeasurementSensor_RmsCurrent
    }
  },
  {
   ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT,
    { // Attribute record
      ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER,
      ZCL_DATATYPE_UINT16,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclPowerMeasurementSensor_ApparentPower
    }
  },
#endif
// power config - report battery level
  {
   ZCL_CLUSTER_ID_GENERAL_POWER_CFG,
    { // Attribute record
      ATTRID_POWER_CONFIGURATION_BATTERY_PERCENTAGE_REMAINING,
      ZCL_DATATYPE_UINT8,
      ACCESS_CONTROL_READ | ACCESS_REPORTABLE,
      (void *)&zclBatteryPercentSensor_MeasuredValue
    }
  },
//  {
//   ZCL_CLUSTER_ID_GENERAL_POWER_CFG,
//    { // Attribute record
//      ATTRID_POWER_CONFIGURATION_BATTERY_VOLTAGE,
//      ZCL_DATATYPE_UINT8,
//      ACCESS_CONTROL_READ,
//      (void *)&zclSampleBatterySensor_MeasuredValue
//    }
//  },
  {
   ZCL_CLUSTER_ID_GENERAL_POWER_CFG,
    { // Attribute record
      ATTRID_CLUSTER_REVISION,
      ZCL_DATATYPE_INT16,
      ACCESS_CONTROL_READ,
      (void *)&zclSensor_battery_clusterRevision
    }
  },
};

uint8_t CONST zclSensor_NumAttributes = ( sizeof(zclSensor_Attrs) / sizeof(zclSensor_Attrs[0]) );

/*********************************************************************
 * SIMPLE DESCRIPTOR
 */
// This is the Cluster ID List and should be filled with Application
// specific cluster IDs.
#if defined(SENSOR_AHT10) || defined(SENSOR_SI7021)

#define ZCLSENSOR_MAX_INCLUSTERS       5
const cId_t zclSensor_InClusterList[ZCLSENSOR_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GENERAL_BASIC,
  ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
  ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
  ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
  ZCL_CLUSTER_ID_GENERAL_POWER_CFG
};
#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)

#define ZCLSENSOR_MAX_INCLUSTERS       4
const cId_t zclSensor_InClusterList[ZCLSENSOR_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GENERAL_BASIC,
  ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
  ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
  ZCL_CLUSTER_ID_GENERAL_POWER_CFG
};

#elif defined(SENSOR_MOVEMENT) || defined(SENSOR_REED)

#define ZCLSENSOR_MAX_INCLUSTERS       4
const cId_t zclSensor_InClusterList[ZCLSENSOR_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GENERAL_BASIC,
  ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
  ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
  ZCL_CLUSTER_ID_GENERAL_POWER_CFG
};

#elif defined(SENSOR_PWR_MEAS)

#define ZCLSENSOR_MAX_INCLUSTERS       4
const cId_t zclSensor_InClusterList[ZCLSENSOR_MAX_INCLUSTERS] =
{
  ZCL_CLUSTER_ID_GENERAL_BASIC,
  ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
  ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT,
  ZCL_CLUSTER_ID_GENERAL_POWER_CFG
};

#endif

#define ZCLSENSOR_MAX_OUTCLUSTERS       1
const cId_t zclSensor_OutClusterList[ZCLSENSOR_MAX_OUTCLUSTERS] =
{
  ZCL_CLUSTER_ID_GENERAL_IDENTIFY
};

SimpleDescriptionFormat_t zclSensor_SimpleDesc =
{
  SENSOR_ENDPOINT,                  //  int Endpoint;
  ZCL_HA_PROFILE_ID,                                 //  uint16_t AppProfId[2];
  ZCL_DEVICEID_TEMPERATURE_SENSOR,                //  uint16_t AppDeviceId[2];
  SENSOR_DEVICE_VERSION,            //  int   AppDevVer:4;
  SENSOR_FLAGS,                     //  int   AppFlags:4;
  ZCLSENSOR_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
  (cId_t *)zclSensor_InClusterList, //  byte *pAppInClusterList;
  ZCLSENSOR_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
  (cId_t *)zclSensor_OutClusterList //  byte *pAppInClusterList;
};

/*********************************************************************
 * GLOBAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      zclSampleLight_ResetAttributesToDefaultValues
 *
 * @brief   Reset all writable attributes to their default values.
 *
 * @param   none
 *
 * @return  none
 */
void zclSensor_ResetAttributesToDefaultValues(void)
{

  zclSensor_PhysicalEnvironment = PHY_UNSPECIFIED_ENV;

#ifdef ZCL_IDENTIFY
  zclSensor_IdentifyTime = 0;
#endif

}

/****************************************************************************
****************************************************************************/


