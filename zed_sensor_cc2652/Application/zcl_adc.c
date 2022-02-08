/*
 * zcl_adc.c
 *
 *  Created on: 31 Jan 2022
 *      Author: BigBob
 */
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
#include <aux_adc.h>
#include "math.h"

#if defined(SENSOR_PWR_MEAS)
#include <Application/sensor_controller/POWER/scif.h>
#endif

static uint8_t reportableChange[] = {0x32, 0x00, 0x00, 0x00}; // 50 in int16_t


static CONST zclAttrRec_t zclSensor_Attrs[] =
{
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
     ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT,
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
    ZCL_DEVICEID_MAINS_POWER_OUTLET,                //  uint16_t AppDeviceId[2];
    SENSOR_DEVICE_VERSION,            //  int   AppDevVer:4;
    SENSOR_FLAGS,                     //  int   AppFlags:4;
    ZCLSENSOR_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSensor_InClusterList, //  byte *pAppInClusterList;
    ZCLSENSOR_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSensor_OutClusterList //  byte *pAppInClusterList;
};


void zclAddAdcAttrs(uint8_t endPoint) {

    zcl_registerAttrList( endPoint, zclSensor_NumAttributes, zclSensor_Attrs );
}

static endPointDesc_t  zclSensorEpDesc = {0};

void zclAddAdcEndPoints(uint8_t endPoint, uint8_t appServiceTaskId) {

    zclSensorEpDesc.endPoint = endPoint;
    zclSensorEpDesc.simpleDesc = &zclSensor_SimpleDesc;
    zclport_registerEndpoint(appServiceTaskId, &zclSensorEpDesc);
}

void ZclAdcReport(void) {

    zclReportCmd_t *pReportCmd;
    pReportCmd = malloc( 2 * (sizeof(zclReportCmd_t) + sizeof(zclReport_t)) );
    if(pReportCmd != NULL)
    {
        // Fill in the single attribute information for the temperature reading
        pReportCmd->numAttr = 2;
        pReportCmd->attrList[0].attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_UINT16;
        pReportCmd->attrList[0].attrData = (void *)&zclPowerMeasurementSensor_RmsCurrent;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

        pReportCmd->attrList[1].attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER;
        pReportCmd->attrList[1].dataType = ZCL_DATATYPE_UINT16;
        pReportCmd->attrList[1].attrData = (uint8 *)&zclPowerMeasurementSensor_ApparentPower;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

        afAddrType_t dstaddr;
        dstaddr.addr.shortAddr = 0x0000;
        dstaddr.addrMode = afAddr16Bit;
        dstaddr.endPoint = 1;
        dstaddr.panId = 0;

        // Call ZCL function to send the report
        zcl_SendReportCmd(8, &dstaddr, ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

        free(pReportCmd);
    }
}

void ZclAdcConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId)
{
    zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};

    if (zlcGetAdcBdbReport(report, true) != BDBREPORTING_SUCCESS) {

        Req.attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT;
        Req.cluster = ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT;
        Req.endpoint = SENSOR_ENDPOINT;
        Req.maxReportInt = 3600;
        Req.minReportInt = 60;
        OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
        Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

        Req.attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER;
        Req.cluster = ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT;
        Req.endpoint = SENSOR_ENDPOINT;
        Req.maxReportInt = 3600;
        Req.minReportInt = 60;
        OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
        Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

    }

}

uint8_t zlcGetAdcBdbReport(bdbReportAttrCfgData_t *report, bool check) {

    return bdb_getReport(SENSOR_ENDPOINT, ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT, ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT, report, check );
}

#if defined(SENSOR_PWR_MEAS)
#define powerBufSize 1024 // from sensor controller BUFFER_SIZE
int32_t sensorPowerAdjusted[powerBufSize];
int32_t prev_avg = 1970;

void process_power_sensor(SCIF_POWER_SENSOR_OUTPUT_T *output, uint8_t appServiceTaskId, int32_t gain, int32_t offset) {
    zstack_bdbRepChangedAttrValueReq_t Req;

    // read the output buffer from the sensor controller and compensate the ADC values
    // then calculate the Irms and apperent power usage
    int32_t sqr = 0;
    int32_t sum = 0;
    int32_t min = 0;
    int32_t max = 0;
    int32_t offsetI = 1965;
    int32_t avg = 0;
    for (int i=0; i<powerBufSize; i++) {

        sensorPowerAdjusted[i] = AUXADCAdjustValueForGainAndOffset(output->pSamples[i], gain, offset);

        offsetI += (sensorPowerAdjusted[i]-offsetI)>>10;
        avg += sensorPowerAdjusted[i];

        int32_t filtered = sensorPowerAdjusted[i] - prev_avg; //1958; //offsetI;

        sum += (filtered * filtered);

        if (filtered>max)
            max = filtered;
        if (filtered < min)
            min = filtered;
    }
    prev_avg = avg >> 10;
    sqr = sqrt(sum / powerBufSize);
    zclPowerMeasurementSensor_RmsCurrent = (91*sqr*zclBatterySensor_MeasuredValue) >> 12; //91=2000/22 (turns / burden resistor
    zclPowerMeasurementSensor_ApparentPower = (zclPowerMeasurementSensor_RmsCurrent * 220) / 1000;

    Req.attrID = ATTRID_ELECTRICAL_MEASUREMENT_RMS_CURRENT;
    Req.cluster = ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT;
    Req.endpoint = SENSOR_ENDPOINT;
    Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);

    Req.attrID = ATTRID_ELECTRICAL_MEASUREMENT_ACTIVE_POWER;
    Req.cluster = ZCL_CLUSTER_ID_MS_ELECTRICAL_MEASUREMENT;
    Req.endpoint = SENSOR_ENDPOINT;
    Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);

}
#endif




