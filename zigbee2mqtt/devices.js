// zigbee2mqtt external converter
const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const extend = require('zigbee-herdsman-converters/lib/extend');
const e = exposes.presets;
const ea = exposes.access;

const definition = [
       {
    zigbeeModel: ['SA-AHT10'],
    model: 'AHT10 Sensor',
    vendor: 'Springfield',
    description: 'AHT10 Temperature and Humidity Sensor',
    supports: '',
    fromZigbee: [fz.temperature, fz.humidity, fz.battery],
    toZigbee: [],
    exposes: [e.temperature(), e.humidity(), e.battery()],
//    configure: async (device, coordinatorEndpoint, logger) => {
//            const endpoint = device.getEndpoint(1);
//            const binds = ['msTemperatureMeasurement', 'msRelativeHumidity', 'genPowerCfg'];
//            await reporting.bind(endpoint, coordinatorEndpoint, binds);
//            await reporting.batteryVoltage(endpoint);
//            await reporting.humidity(endpoint);
//            await reporting.temperature(endpoint);
//    },
    },
    {
    zigbeeModel: ['SA-MOVEMENT'],
    model: 'PIR Sensor',
    vendor: 'Springfield',
    description: 'PIR Movement Sensor',
    supports: '',
    fromZigbee: [fz.occupancy, fz.battery],
    toZigbee: [],
    exposes: [e.occupancy(), e.battery()],
    },
    {
    zigbeeModel: ['SA-DOOR'],
    model: 'Reed Switch Door Sensor',
    vendor: 'Springfield',
    description: 'Door Open/Close Sensor',
    supports: '',
    fromZigbee: [fz.occupancy, fz.battery],
    toZigbee: [],
    exposes: [e.occupancy(), e.battery()],
    },
    {
    zigbeeModel: ['SA-POWER'],
    model: 'Power Measurement Sensor',
    vendor: 'Springfield',
    description: 'Current and Power Sensor',
    supports: '',
    fromZigbee: [fz.electrical_measurement, fz.battery],
    toZigbee: [],
    exposes: [e.current(), e.power(), e.battery()],
    }
];

module.exports = definition;