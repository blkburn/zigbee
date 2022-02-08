/*
 * zcl_temp_humid.c
 *
 *  Created on: 26 Jan 2022
 *      Author: BigBob
 */
/*********************************************************************
 * INCLUDES
 */
#include <Application/zcl_sensor.h>
#include "zcomdef.h"
#include "zcl.h"
#include "zcl_general.h"
#include "zcl_ha.h"
#include "zcl_ms.h"
#include "zcl_port.h"
#include "zstackapi.h"

#include "zcl_consts.h"
#include "zcl_sensor.h"

#if defined(SENSOR_AHT10)
#include <Application/sensor_controller/AHT10/scif.h>
#elif defined(SENSOR_SI7021)
#include <Application/sensor_controller/SI7021/scif.h>
#endif

static uint8_t reportableChangeT[] = {0x32, 0x00, 0x00, 0x00}; // 50 in int16_t
static uint8_t reportableChangeH[] = {0xFF, 0x00, 0x00, 0x00}; // 50 in int16_t

static CONST zclAttrRec_t zclSensor_Attrs[] =
{
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
};

static uint8_t CONST zclSensor_NumAttributes = ( sizeof(zclSensor_Attrs) / sizeof(zclSensor_Attrs[0]) );


  /*********************************************************************
   * SIMPLE DESCRIPTOR
   */
  // This is the Cluster ID List and should be filled with Application
  // specific cluster IDs.

  #define ZCLSENSOR_MAX_INCLUSTERS       5
  static const cId_t zclSensor_InClusterList[ZCLSENSOR_MAX_INCLUSTERS] =
  {
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT,
    ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY,
    ZCL_CLUSTER_ID_GENERAL_POWER_CFG
  };

#define ZCLSENSOR_MAX_OUTCLUSTERS       1
static const cId_t zclSensor_OutClusterList[ZCLSENSOR_MAX_OUTCLUSTERS] =
{
     ZCL_CLUSTER_ID_GENERAL_IDENTIFY
};

static SimpleDescriptionFormat_t zclSensor_SimpleDesc =
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



void zclAddMeasAttrs(uint8_t endPoint) {
    zcl_registerAttrList( endPoint, zclSensor_NumAttributes, zclSensor_Attrs );
}

static endPointDesc_t  zclSensorEpDesc = {0};

void zclAddMeasEndPoints(uint8_t endPoint, uint8_t appServiceTaskId) {

    zclSensorEpDesc.endPoint = endPoint;
    zclSensorEpDesc.simpleDesc = &zclSensor_SimpleDesc;
    zclport_registerEndpoint(appServiceTaskId, &zclSensorEpDesc);
}

void ZclTempHumidReport(void) {

    zclReportCmd_t *pReportCmd;
    pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if(pReportCmd != NULL)
    {
        // Fill in the single attribute information for the temperature reading
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_INT16;
        pReportCmd->attrList[0].attrData = (uint8 *)&zclTemperatureSensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

        afAddrType_t dstaddr;
        dstaddr.addr.shortAddr = 0x0000;
        dstaddr.addrMode = afAddr16Bit;
        dstaddr.endPoint = 1;
        dstaddr.panId = 0;

        // Call ZCL function to send the report
        zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

        free(pReportCmd);
    }

    pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if(pReportCmd != NULL)
    {
        // Fill in the single attribute information for the temperature reading
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
        pReportCmd->attrList[0].attrData = (uint8 *)&zclHumiditySensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

        afAddrType_t dstaddr;
        dstaddr.addr.shortAddr = 0x0000;
        dstaddr.addrMode = afAddr16Bit;
        dstaddr.endPoint = 1;
        dstaddr.panId = 0;

        // Call ZCL function to send the report
        zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

        free(pReportCmd);
    }
}

void ZclTempHumidConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId) {
    zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};

    if (zlcGetTempHumidBdbReport(report, true) != BDBREPORTING_SUCCESS) {
        Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
        Req.endpoint = SENSOR_ENDPOINT;
        Req.maxReportInt = 3600;
        Req.minReportInt = 60;
        OsalPort_memcpy(Req.reportableChange,reportableChangeT,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
        Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

        /// setup Humidity reporting
        Req.attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY;
        Req.endpoint = SENSOR_ENDPOINT;
        Req.maxReportInt = 3600;
        Req.minReportInt = 60;
        OsalPort_memcpy(Req.reportableChange,reportableChangeH,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
        Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

    }
}

uint8_t zlcGetTempHumidBdbReport(bdbReportAttrCfgData_t *report, bool check) {

    return bdb_getReport(8, ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT, ATTRID_DEVICE_TEMPERATURE_CONFIGURATION_CURRENT_TEMPERATURE, report, check );
}

#if defined(SENSOR_AHT10)
void process_AHT10_temp_humid_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId) {
    zstack_bdbRepChangedAttrValueReq_t Req;

    if (output->status == 25) {
        int32_t calc;

        // calculate the relative humidity from reported controller value
        // update the Zigbee stack that the humidity value has changed
        calc = (output->byte1 << 8) | (output->byte2); // | (output.byte3 >> 4);
        calc = (calc * 10000) >> 16;
        zclHumiditySensor_MeasuredValue = (int16_t)(calc);
        Req.attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY;
        Req.endpoint = SENSOR_ENDPOINT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);

        // calculate the temperature from reported controller value
        // update the Zigbee stack that the humidity value has changed
        calc = (((output->byte3 & 0x0F) << 16) | ((output->byte4) << 8) | output->byte5);
        calc = ((((calc) >> 10) * 20000) >> 10) - 5000;
        zclTemperatureSensor_MeasuredValue = (int16_t)(calc);
        Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
        Req.endpoint = SENSOR_ENDPOINT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);

    } else {
       // no valid data - do nothing
    }
}

#elif defined(SENSOR_SI7021)
void process_SI7021_temp_humid_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId) {
    zstack_bdbRepChangedAttrValueReq_t Req;

//    if (output.status == 25) {
        int32_t calc;

        calc = (((uint16_t)output->hum * 12500) >> 16) - 600;
        zclHumiditySensor_MeasuredValue = (int16_t)(calc);
        Req.attrID = ATTRID_RELATIVITY_HUMIDITY_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_RELATIVE_HUMIDITY;
        Req.endpoint = SENSOR_ENDPOINT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);

        calc = (((uint16_t)output->temp * 17672) >> 16) - 4785;
        zclTemperatureSensor_MeasuredValue = (int16_t)(calc);
        Req.attrID = ATTRID_TEMPERATURE_MEASUREMENT_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_TEMPERATURE_MEASUREMENT;
        Req.endpoint = SENSOR_ENDPOINT;
        Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);

}

#endif
