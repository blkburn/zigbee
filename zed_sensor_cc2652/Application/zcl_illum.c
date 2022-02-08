/*
 * zcl_light.c
 *
 *  Created on: 27 Jan 2022
 *      Author: BigBob
 */

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
#include "math.h"

#if defined(SENSOR_APDS9930)
#include <Application/sensor_controller/ADPS9930/scif.h>
#elif defined(SENSOR_LTR390)
#include <Application/sensor_controller/LTR390/scif.h>
#endif

static uint8_t reportableChange[] = {0x32, 0x00, 0x00, 0x00}; // 50 in int16_t


static CONST zclAttrRec_t zclSensor_Attrs[] =
{
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
};

static uint8_t CONST zclSensor_NumAttributes = ( sizeof(zclSensor_Attrs) / sizeof(zclSensor_Attrs[0]) );


  /*********************************************************************
   * SIMPLE DESCRIPTOR
   */
  // This is the Cluster ID List and should be filled with Application
  // specific cluster IDs.

#define ZCLSENSOR_MAX_INCLUSTERS       4
static const cId_t zclSensor_InClusterList[ZCLSENSOR_MAX_INCLUSTERS] =
{
    ZCL_CLUSTER_ID_GENERAL_BASIC,
    ZCL_CLUSTER_ID_GENERAL_IDENTIFY,
    ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT,
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
    ZCL_DEVICEID_LIGHT_SENSOR,                //  uint16_t AppDeviceId[2];
    SENSOR_DEVICE_VERSION,            //  int   AppDevVer:4;
    SENSOR_FLAGS,                     //  int   AppFlags:4;
    ZCLSENSOR_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSensor_InClusterList, //  byte *pAppInClusterList;
    ZCLSENSOR_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSensor_OutClusterList //  byte *pAppInClusterList;
};


void zclAddIllumAttrs(uint8_t endPoint) {

    zcl_registerAttrList( endPoint, zclSensor_NumAttributes, zclSensor_Attrs );
}

static endPointDesc_t  zclSensorEpDesc = {0};

void zclAddIllumEndPoints(uint8_t endPoint, uint8_t appServiceTaskId) {

    zclSensorEpDesc.endPoint = endPoint;
    zclSensorEpDesc.simpleDesc = &zclSensor_SimpleDesc;
    zclport_registerEndpoint(appServiceTaskId, &zclSensorEpDesc);
}

void ZclIllumReport(void) {

    zclReportCmd_t *pReportCmd;
    pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if(pReportCmd != NULL)
    {
        // Fill in the single attribute information for the temperature reading
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
        pReportCmd->attrList[0].attrData = (void *)&zclIlluminanceSensor_MeasuredValue;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

        afAddrType_t dstaddr;
        dstaddr.addr.shortAddr = 0x0000;
        dstaddr.addrMode = afAddr16Bit;
        dstaddr.endPoint = 1;
        dstaddr.panId = 0;

        // Call ZCL function to send the report
        zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

        free(pReportCmd);
    }
}

void ZclIllumConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId) {
    zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};

    if (zlcGetIllumBdbReport(report, true) != BDBREPORTING_SUCCESS) {
        Req.attrID = ATTRID_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE;
        Req.cluster = ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT;
        Req.endpoint = SENSOR_ENDPOINT;
        Req.maxReportInt = 60;
        Req.minReportInt = 30;
        OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
        Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);
    }

}

uint8_t zlcGetIllumBdbReport(bdbReportAttrCfgData_t *report, bool check) {

//    return BDBREPORTING_SUCCESS;
    return bdb_getReport(8, ZCL_CLUSTER_ID_MS_ILLUMINANCE_MEASUREMENT, ATTRID_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE, report, check );
}

#if defined(SENSOR_APDS9930)
void process_APDS9930_light_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId) {
    uint32_t calc1;
    uint32_t calc2;
    uint32_t iac;
    uint32_t lpc;
    calc1 = (output->ch0data << 10) - 1907 * (output->ch1data);
    calc2 = 764 * (output->ch0data) - 1322 * (output->ch1data);
    iac = (calc1 > calc2 ? calc1 : calc2) >> 10;
    lpc = 9594;
    calc1 = (iac * lpc) >> 16;
    zclIlluminanceSensor_MeasuredValue = log10( calc1 ) * 10000;
}
#elif defined(SENSOR_LTR390)
void process_LTR390_light_sensor(SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId) {
    uint32_t calc;
    calc = ((output->byte3 & 0x0F) << 16) | (output->byte2 << 8) | output->byte1; // | (output.byte3 >> 4);
    calc /=  5;
    calc = log10(calc) * 10000;
    zclIlluminanceSensor_MeasuredValue = calc;
}
#endif




