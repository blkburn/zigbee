///*
// * sensor_controller.h
// *
// *  Created on: 3 Feb 2022
// *      Author: BigBob
// */
//
//#ifndef APPLICATION_SENSOR_CONTROLLER_H_
//#define APPLICATION_SENSOR_CONTROLLER_H_
//
//#if defined(SENSOR_AHT10)
//#include <Application/sensor_controller/AHT10/scif.h>
//#elif defined(SENSOR_SI7021)
//#include <Application/sensor_controller/SI7021/scif.h>
//#elif defined(SENSOR_LTR390)
//#include <Application/sensor_controller/LTR390/scif.h>
//#elif defined(SENSOR_APDS9930)
//#include <Application/sensor_controller/ADPS9930/scif.h>
//#elif defined(SENSOR_MOVEMENT)
//#include <Application/sensor_controller/MOVEMENT/scif.h>
//#elif defined(SENSOR_REED)
//#include <Application/sensor_controller/REED/scif.h>
//#elif defined(SENSOR_PWR_MEAS)
//#include <Application/sensor_controller/POWER/scif.h>
//#endif
//
//
//#ifdef SENSOR_AHT10
//int32_t prev_temp = 0;
//int32_t prev_hum = 0;
//SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
//#elif defined(SENSOR_SI7021)
//int32_t prev_temp = 0;
//int32_t prev_hum = 0;
//SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
//#elif defined(SENSOR_LTR390) || defined(SENSOR_APDS9930)
//int32_t prev_lux = 0;
//SCIF_I2C_TEMP_AND_HUMIDITY_SENSOR_OUTPUT_T output;
//#elif defined(SENSOR_MOVEMENT)
//int32_t prev_state = 0;
//SCIF_BUTTON_DEBOUNCER_OUTPUT_T output;
//#elif defined(SENSOR_REED)
//SCIF_BUTTON_DEBOUNCER_OUTPUT_T output;
//#elif defined(SENSOR_PWR_MEAS)
//int32_t prev_state = 0;
//int32_t prev_avg = 1970;
//SCIF_POWER_SENSOR_OUTPUT_T output;
//#define powerBufSize 1024 // from sensor controller BUFFER_SIZE
//extern uint16_t zclPowerMeasurementSensor_RmsCurrent;
//extern uint16_t zclPowerMeasurementSensor_ApparentPower;
//
//#endif
//
//
//
//
//#endif /* APPLICATION_SENSOR_CONTROLLER_H_ */
