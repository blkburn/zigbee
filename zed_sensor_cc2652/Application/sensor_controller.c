//#include <Application/zcl_sensor.h>
//#include "zcomdef.h"
//#include "zcl.h"
//#include "zcl_general.h"
//#include "zcl_ha.h"
//#include "zcl_ms.h"
//#include "zcl_port.h"
//#include "zstackapi.h"
//#include <Application/sensor_controller/REED/scif.h>
//
//#include "zcl_consts.h"
//
//
//void process_door_sensor(SCIF_BUTTON_DEBOUNCER_OUTPUT_T *output, uint8_t appServiceTaskId) {
//    zstack_bdbRepChangedAttrValueReq_t Req;
//
//    zclOccupancySensor_Occupancy = 1-output->buttonState;
//    Req.attrID = ATTRID_OCCUPANCY_SENSING_OCCUPANCY;
//    Req.cluster = ZCL_CLUSTER_ID_MS_OCCUPANCY_SENSING;
//    Req.endpoint = SENSOR_ENDPOINT;
//    Zstackapi_bdbRepChangedAttrValueReq(appServiceTaskId, &Req);
//}
