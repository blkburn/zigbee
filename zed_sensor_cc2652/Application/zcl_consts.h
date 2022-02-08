#ifndef APPLICATION_ZCL_CONSTS_H_
#define APPLICATION_ZCL_CONSTS_H_

#include "bdb_reporting.h"

/*********************************************************************
 * CONSTANTS
 */
// Global attributes
extern const uint16_t zclSensor_basic_clusterRevision;
extern const uint16_t zclSensor_identify_clusterRevision;
extern const uint16_t zclSensor_battery_clusterRevision;
extern const uint16_t zclSensor_temperature_clusterRevision;
extern const uint16_t zclSensor_humidity_clusterRevision;

//extern const zclAttrRec_t zclMeasSense_Attrs[];

extern void zclAddMeasAttrs(uint8_t endPoint);
extern void zclAddMeasEndPoints(uint8_t endPoint, uint8_t appServiceTaskId);
extern void ZclTempHumidReport(void);
extern void ZclTempHumidConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId);
extern uint8_t zlcGetTempHumidBdbReport(bdbReportAttrCfgData_t *report, bool check);

extern void ZclIllumReport(void);
extern void ZclIllumConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId);
extern void zclAddIllumAttrs(uint8_t endPoint);
extern void zclAddIllumEndPoints(uint8_t endPoint, uint8_t appServiceTaskId);
extern uint8_t zlcGetIllumBdbReport(bdbReportAttrCfgData_t *report, bool check);

extern void ZclAdcReport(void);
extern void ZclAdcConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId);
extern void zclAddAdcAttrs(uint8_t endPoint);
extern void zclAddAdcEndPoints(uint8_t endPoint, uint8_t appServiceTaskId);
extern uint8_t zlcGetAdcBdbReport(bdbReportAttrCfgData_t *report, bool check);

extern void ZclButtonReport(void);
extern void ZclButtonConfigureReport(bdbReportAttrCfgData_t *report, uint8_t appServiceTaskId);
extern void zclAddButtonAttrs(uint8_t endPoint);
extern void zclAddButtonEndPoints(uint8_t endPoint, uint8_t appServiceTaskId);
extern uint8_t zlcGetButtonBdbReport(bdbReportAttrCfgData_t *report, bool check);

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




#endif /* APPLICATION_ZCL_CONSTS_H_ */
