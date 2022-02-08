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

#if defined(SENSOR_MOVEMENT)
#include <Application/sensor_controller/MOVEMENT/scif.h>
#elif defined(SENSOR_REED)
#include <Application/sensor_controller/REED/scif.h>
#else
typedef struct {
    uint16_t buttonState; ///< Current button state
} SCIF_BUTTON_DEBOUNCER_OUTPUT_T;

#endif

#include "zcl_consts.h"

static uint8_t reportableChange[] = {0x32, 0x00, 0x00, 0x00}; // 50 in int16_t

// Occupancy Sensor Cluster
uint8_t zclOccupancySensor_Occupancy = DOOR_STATE_CLOSED;
const uint8_t zclOccupancySensor_OccupancySensorType = PIR_SENSOR_TYPE;


static CONST zclAttrRec_t zclSensor_Attrs[] =
{
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
     ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING,
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
    ZCL_DEVICEID_OCCUPANCY_SENSOR,                //  uint16_t AppDeviceId[2];
    SENSOR_DEVICE_VERSION,            //  int   AppDevVer:4;
    SENSOR_FLAGS,                     //  int   AppFlags:4;
    ZCLSENSOR_MAX_INCLUSTERS,         //  byte  AppNumInClusters;
    (cId_t *)zclSensor_InClusterList, //  byte *pAppInClusterList;
    ZCLSENSOR_MAX_OUTCLUSTERS,        //  byte  AppNumInClusters;
    (cId_t *)zclSensor_OutClusterList //  byte *pAppInClusterList;
};


void zclAddButtonAttrs(uint8_t endPoint) {

    zcl_registerAttrList( endPoint, zclSensor_NumAttributes, zclSensor_Attrs );
}

static endPointDesc_t  zclSensorEpDesc = {0};

void zclAddButtonEndPoints(uint8_t endPoint, uint8_t appServiceTaskId) {

    zclSensorEpDesc.endPoint = endPoint;
    zclSensorEpDesc.simpleDesc = &zclSensor_SimpleDesc;
    zclport_registerEndpoint(appServiceTaskId, &zclSensorEpDesc);
}

void ZclButtonReport(void) {

    zclReportCmd_t *pReportCmd;
    pReportCmd = malloc( sizeof(zclReportCmd_t) + sizeof(zclReport_t) );
    if(pReportCmd != NULL)
    {
        // Fill in the single attribute information for the temperature reading
        pReportCmd->numAttr = 1;
        pReportCmd->attrList[0].attrID = ATTRID_OCCUPANCY_SENSING_OCCUPANCY;
        pReportCmd->attrList[0].dataType = ZCL_DATATYPE_BITMAP8;
        pReportCmd->attrList[0].attrData = (void *)&zclOccupancySensor_Occupancy;// (uint8 *)&(zclSampleTemperatureSensor_MeasuredValue);

        afAddrType_t dstaddr;
        dstaddr.addr.shortAddr = 0x0000;
        dstaddr.addrMode = afAddr16Bit;
        dstaddr.endPoint = 1;
        dstaddr.panId = 0;

        // Call ZCL function to send the report
        zcl_SendReportCmd(SENSOR_ENDPOINT, &dstaddr, ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING, pReportCmd, ZCL_FRAME_SERVER_CLIENT_DIR, 1, 0 );

        free(pReportCmd);
    }
}

void ZclButtonConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId) {
    zstack_bdbRepAddAttrCfgRecordDefaultToListReq_t Req = {0};

    if (zlcGetButtonBdbReport(report, true) != BDBREPORTING_SUCCESS) {
        Req.attrID = ATTRID_OCCUPANCY_SENSING_OCCUPANCY;
        Req.cluster = ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING;
        Req.endpoint = SENSOR_ENDPOINT;
        Req.maxReportInt = 3600;
        Req.minReportInt = 0;
        OsalPort_memcpy(Req.reportableChange,reportableChange,BDBREPORTING_MAX_ANALOG_ATTR_SIZE);
        Zstackapi_bdbRepAddAttrCfgRecordDefaultToListReq(appServiceTaskId,&Req);

    }
}

uint8_t zlcGetButtonBdbReport(bdbReportAttrCfgData_t *report, bool check) {

//    return BDBREPORTING_SUCCESS;
    return bdb_getReport(SENSOR_ENDPOINT, ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING, ATTRID_OCCUPANCY_SENSING_OCCUPANCY, report, check );
}


void process_door_sensor(SCIF_BUTTON_DEBOUNCER_OUTPUT_T *output, uint8_t appServiceTaskId) {
    zstack_bdbRepChangedAttrValueReq_t Req;

    zclOccupancySensor_Occupancy = 1-output->buttonState;
    Req.attrID = ATTRID_OCCUPANCY_SENSING_OCCUPANCY;
    Req.cluster = ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING;
    Req.endpoint = SENSOR_ENDPOINT;
    Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);
}

void process_movement_sensor(SCIF_BUTTON_DEBOUNCER_OUTPUT_T *output, uint8_t appServiceTaskId) {
    zstack_bdbRepChangedAttrValueReq_t Req;

    zclOccupancySensor_Occupancy = output->buttonState;
    Req.attrID = ATTRID_OCCUPANCY_SENSING_OCCUPANCY;
    Req.cluster = ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING;
    Req.endpoint = SENSOR_ENDPOINT;
    Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);
}



