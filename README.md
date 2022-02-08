# Zigbee

This repository contains multiple projects relating to teh Zigbee wireless protocol.

- zed_sensor_cc2652
- hardware
- sensor controller
- zigbee2mqtt

---

## zed_sensor_cc2652

This is a Zigbee End Device based on the Texas Instruments CC2652R1. It supports the following sensors (with build switches):
- AHT10 Temperature and Humidity Sensor
- APDS9930 Digital Proximity and Ambient Light Sensor
- LTR390 UV Light Sensor
- PIR Movement Sensor
- Reed Switch (Door opening sensor)
- Mains Energy Monitor (split coil current transformer)

The Sensor Controller handles all the sensor communications, while the main process handles the Zigbee stack and a custom task to process sensor reading and notify the Zigbee stack.

Tested with Zigbee2MQTT version 1.22.1 commit: 89996835

Master branch is currently built with:
* CCStudio version: 11.1.0.00011 
* SDK: simplelink_cc13x2_26x2_sdk_5_20_00_52

update of https://github.com/blkburn/cc2652_zigbee_zed

---

## Hardware

KiCad schematic and PCB designs. Both boards have be manufactured, tested, and working.

### Zigbee End Device
Based on the CC2652R module from RF Star (RF-BM-2652B1).
This is version 2 of the Zigbee End Device design in https://github.com/blkburn/cc2652_zigbee_zed/tree/master/hardware/cc2652_rf_star, and supports all the sensors described above. The version 1 board working unmodified with the zed_sensor_cc2652 software.

### Zigbee Coordinator
Based on the CC2652P module from RF Star (RF-BM-2652P2). It is a daughter board for the Orange Pi Zero 2 (http://www.orangepi.org/Orange%20Pi%20Zero2/). The Orange Pi uses an Allwinner H616 processor and comfortably runs Ubuntu or Debian Linux (plus much cheaper than a Raspberry Pi 4).

---

## Sensor Controller

This contains all the sensor controller projects that are used in the Zigbee End Device.

## Zigbee2MQTT

This directory contains the custom javascript device file for zigbee2mqtt to recognize the sensors.

