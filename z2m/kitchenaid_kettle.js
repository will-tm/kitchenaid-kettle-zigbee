/**
 * Zigbee2MQTT External Converter for KitchenAid 5KEK1522 Zigbee Kettle
 *
 * Installation:
 * 1. Copy this file to your Zigbee2MQTT data folder: data/external_converters/kitchenaid_kettle.js
 * 2. Add to configuration.yaml:
 *    external_converters:
 *      - kitchenaid_kettle.js
 * 3. Restart Zigbee2MQTT
 *
 * Exposed entities:
 * - state (binary): Kettle on/off state (controllable via Zigbee)
 * - current_temperature (numeric): Current water temperature (°C, read-only)
 * - target_temperature (numeric): Target temperature from dial (50-100°C, read-only)
 * - system_mode (enum): Heating mode (off/heat, read-only)
 */

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');
const e = exposes.presets;
const ea = exposes.access;

// Custom fromZigbee converters
const fzLocal = {
    kettle_on_off: {
        cluster: 'genOnOff',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.hasOwnProperty('onOff')) {
                return {state: msg.data['onOff'] ? 'ON' : 'OFF'};
            }
        },
    },

    kettle_thermostat: {
        cluster: 'hvacThermostat',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const result = {};

            // Local temperature (current water temperature)
            if (msg.data.hasOwnProperty('localTemp')) {
                const temp = msg.data['localTemp'];
                if (temp !== 0x8000) { // 0x8000 = invalid/unknown
                    result.current_temperature = temp / 100;
                }
            }

            // Occupied heating setpoint (target temperature)
            if (msg.data.hasOwnProperty('occupiedHeatingSetpoint')) {
                result.target_temperature = msg.data['occupiedHeatingSetpoint'] / 100;
            }

            // System mode (off/heat based on kettle state)
            if (msg.data.hasOwnProperty('systemMode')) {
                const mode = msg.data['systemMode'];
                result.system_mode = mode === 4 ? 'heat' : 'off';
            }

            return result;
        },
    },

    kettle_temperature_measurement: {
        cluster: 'msTemperatureMeasurement',
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data.hasOwnProperty('measuredValue')) {
                const temp = msg.data['measuredValue'];
                if (temp !== 0x8000) {
                    return {current_temperature: temp / 100};
                }
            }
        },
    },
};

// Custom toZigbee converters
const tzLocal = {
    kettle_on_off: {
        key: ['state'],
        convertSet: async (entity, key, value, meta) => {
            const state = value.toUpperCase() === 'ON';
            await entity.command('genOnOff', state ? 'on' : 'off', {});
            // Don't return state - let the device report actual state
            return {};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read('genOnOff', ['onOff']);
        },
    },

};

const definition = {
    zigbeeModel: ['5KEK1522-ZB'],
    model: '5KEK1522-ZB',
    vendor: 'KitchenAid',
    description: 'Smart kettle with temperature control (Zigbee)',
    icon: 'https://www.whirlpool.com/is/image/content/dam/emea/kitchenaid/image/product-shot/profile/MNRCNO_Kitchenaid_Kettle_5KEK1522BER_empire_red_face.jpg',
    fromZigbee: [
        fzLocal.kettle_on_off,
        fzLocal.kettle_thermostat,
        fzLocal.kettle_temperature_measurement,
        fz.identify,
    ],
    toZigbee: [
        tzLocal.kettle_on_off,
        tz.identify,
    ],
    exposes: [
        // Kettle on/off state (controllable - sends command, reports actual state)
        e.binary('state', ea.ALL, 'ON', 'OFF')
            .withDescription('Kettle heating state'),

        // Current water temperature
        e.numeric('current_temperature', ea.STATE)
            .withUnit('°C')
            .withValueMin(0)
            .withValueMax(100)
            .withDescription('Current water temperature'),

        // Target temperature setpoint (read-only, set by physical dial)
        e.numeric('target_temperature', ea.STATE)
            .withUnit('°C')
            .withValueMin(50)
            .withValueMax(100)
            .withDescription('Target temperature from dial'),

        // System mode (reflects kettle state)
        e.enum('system_mode', ea.STATE, ['off', 'heat'])
            .withDescription('Heating system mode'),
    ],
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);

        // Bind clusters for reporting
        await reporting.bind(endpoint, coordinatorEndpoint, [
            'genOnOff',
            'hvacThermostat',
            'msTemperatureMeasurement',
        ]);

        // Configure reporting for on/off state
        await reporting.onOff(endpoint);

        // Note: Thermostat and temperature reporting is handled internally by the
        // firmware via zb_zcl_mark_attr_for_reporting() - configureReporting would
        // fail with UNREPORTABLE_ATTRIBUTE since the attributes don't have the
        // reportable flag set in the ZCL layer.

        // Read initial values
        await endpoint.read('genOnOff', ['onOff']);
        await endpoint.read('hvacThermostat', [
            'localTemp',
            'occupiedHeatingSetpoint',
            'systemMode',
        ]);
        await endpoint.read('msTemperatureMeasurement', ['measuredValue']);
    },
    meta: {
        multiEndpoint: false,
    },
};

module.exports = definition;
