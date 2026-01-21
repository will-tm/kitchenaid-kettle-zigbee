/**
 * @file main.c
 * @brief KitchenAid 5KEK1522 Zigbee Kettle Controller
 *
 * Zigbee Router device that monitors and reports kettle state:
 * - On/Off state from GPIO input (reads kettle heating element state)
 * - Target temperature from linear analog input (50-100°C)
 * - Current temperature from 100K NTC thermistor
 * - Button for manual toggle and pairing (long press)
 * - Status LED for network indication
 */

#include <zephyr/types.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zboss_api.h>
#include <zboss_api_addons.h>
#include <zb_mem_config_med.h>
#include <zigbee/zigbee_app_utils.h>
#include <zigbee/zigbee_error_handler.h>
#include <zb_nrf_platform.h>
#include "zb_kettle.h"

#ifdef CONFIG_ZIGBEE_FOTA
#include <zigbee/zigbee_fota.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#endif

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ==========================================================================
 * UTC Time Stub (required by Zigbee stack)
 * ========================================================================== */

/**
 * Provide UTC time to Zigbee stack. Returns 0 to indicate time unavailable.
 * The stack will function without accurate time - this silences the warning.
 */
zb_uint32_t zb_get_utc_time(void)
{
	return 0;
}

/* ==========================================================================
 * Configuration
 * ========================================================================== */

#define KETTLE_ENDPOINT                 1

#define KETTLE_INIT_BASIC_APP_VERSION   1
#define KETTLE_INIT_BASIC_STACK_VERSION 1
#define KETTLE_INIT_BASIC_HW_VERSION    1
#define KETTLE_INIT_BASIC_MANUF_NAME    "KitchenAid"
#define KETTLE_INIT_BASIC_MODEL_ID      "5KEK1522-ZB"
#define KETTLE_INIT_BASIC_DATE_CODE     "20260116"
#define KETTLE_INIT_BASIC_LOCATION_DESC ""
#define KETTLE_INIT_BASIC_PH_ENV        ZB_ZCL_BASIC_ENV_UNSPECIFIED

#define BUTTON_LONG_PRESS_MS            3000
#define KETTLE_BUTTON_PULSE_MS          200     /* Duration to hold simulated button press */
#define KETTLE_TRANSITION_TIMEOUT_MS    5000    /* Max time to wait for kettle state change */

/* Kettle heating state machine */
typedef enum {
	KETTLE_STATE_OFF,           /* Not heating, idle */
	KETTLE_STATE_TURNING_ON,    /* Button pressed, waiting for heating to start */
	KETTLE_STATE_ON,            /* Heating active */
	KETTLE_STATE_TURNING_OFF,   /* Button pressed, waiting for heating to stop */
} kettle_state_t;

/* Temperature ranges (in 0.01°C units for Zigbee) */
#define TEMP_MIN_CELSIUS        50
#define TEMP_MAX_CELSIUS        100
#define TEMP_MIN_ZB             (TEMP_MIN_CELSIUS * 100)   /* 5000 = 50.00°C */
#define TEMP_MAX_ZB             (TEMP_MAX_CELSIUS * 100)   /* 10000 = 100.00°C */
#define TEMP_INVALID_ZB         0x8000                      /* Invalid temperature */

/* ADC voltage divider after op-amp buffer */
#define ADC_DIVIDER_RATIO       2       /* 10K:10K divider after buffer */

/* ADC configuration */
#define ADC_RESOLUTION          12
#define ADC_MAX_VALUE           ((1 << ADC_RESOLUTION) - 1)
#define ADC_SAMPLE_INTERVAL_MS  1000    /* Sample every second */

/* EMA filter: filtered = prev + (new - prev) / ADC_FILTER_COEFF
 * Higher value = more smoothing, slower response
 * 4 = moderate smoothing, 8 = heavy smoothing
 */
#define ADC_FILTER_COEFF        8

/* ==========================================================================
 * Device Tree
 * ========================================================================== */

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec kettle_state_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(kettle_state_gpio), gpios);
static const struct gpio_dt_spec kettle_button_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(kettle_button_gpio), gpios);

/* ADC device and channel configuration using devicetree io-channels */
static const struct adc_dt_spec adc_target_temp =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0);
static const struct adc_dt_spec adc_current_temp =
	ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1);

/* ==========================================================================
 * Application Context
 * ========================================================================== */

/* On/Off cluster attributes (read-only - reports kettle state) */
typedef struct {
	zb_bool_t on_off;
} on_off_attrs_t;

/* Thermostat cluster attributes (target temperature setpoint) */
typedef struct {
	zb_int16_t local_temperature;           /* Current temp (0.01°C) - mirrored from temp measurement */
	zb_int16_t occupied_cooling_setpoint;   /* Not used, but required */
	zb_int16_t occupied_heating_setpoint;   /* Target temperature (0.01°C) */
	zb_int16_t min_heat_setpoint_limit;     /* 50°C min */
	zb_int16_t max_heat_setpoint_limit;     /* 100°C max */
	zb_uint8_t control_sequence;            /* Heating only */
	zb_uint8_t system_mode;                 /* Off/Heat */
} thermostat_attrs_t;

/* Temperature measurement cluster attributes */
typedef struct {
	zb_int16_t measured_value;              /* Current temp (0.01°C) */
	zb_int16_t min_measured_value;          /* 50°C */
	zb_int16_t max_measured_value;          /* 100°C */
} temp_measurement_attrs_t;

typedef struct {
	zb_zcl_basic_attrs_ext_t    basic_attr;
	zb_zcl_identify_attrs_t     identify_attr;
	zb_zcl_groups_attrs_t       groups_attr;
	on_off_attrs_t              on_off_attr;
	thermostat_attrs_t          thermostat_attr;
	temp_measurement_attrs_t    temp_measurement_attr;
} kettle_device_ctx_t;

static kettle_device_ctx_t dev_ctx;

/* Button state */
static struct {
	int64_t press_time;
	bool    pressed;
} button_state;

/* Kettle heating state machine */
static kettle_state_t kettle_heating_state = KETTLE_STATE_OFF;

static struct gpio_callback button_cb_data;
static struct gpio_callback kettle_state_cb_data;
static struct k_work button_work;
static struct k_work_delayable long_press_work;
static struct k_work_delayable adc_sample_work;
static struct k_work_delayable kettle_button_release_work;
static struct k_work_delayable kettle_transition_timeout_work;

/* ADC buffer and sequence (configured per-read) */
static int16_t adc_buffer;

/* EMA filtered ADC values (initialized to -1 to indicate first sample) */
static int32_t adc_target_filtered = -1;
static int32_t adc_current_filtered = -1;

/* ==========================================================================
 * Persistent Settings
 * ========================================================================== */

static int kettle_settings_set(const char *name, size_t len,
			       settings_read_cb read_cb, void *cb_arg)
{
	if (!strcmp(name, "target_temp")) {
		if (len != sizeof(dev_ctx.thermostat_attr.occupied_heating_setpoint)) {
			return -EINVAL;
		}
		read_cb(cb_arg, &dev_ctx.thermostat_attr.occupied_heating_setpoint, len);
		LOG_INF("Restored target temp: %d (0.01°C)",
			dev_ctx.thermostat_attr.occupied_heating_setpoint);
	}
	return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(kettle, "kettle", NULL, kettle_settings_set, NULL, NULL);

static void save_kettle_state(void)
{
	settings_save_one("kettle/target_temp",
			  &dev_ctx.thermostat_attr.occupied_heating_setpoint,
			  sizeof(dev_ctx.thermostat_attr.occupied_heating_setpoint));
}

/* ==========================================================================
 * Zigbee Cluster Declarations
 * ========================================================================== */

ZB_ZCL_DECLARE_BASIC_ATTRIB_LIST_EXT(
	basic_attr_list,
	&dev_ctx.basic_attr.zcl_version,
	&dev_ctx.basic_attr.app_version,
	&dev_ctx.basic_attr.stack_version,
	&dev_ctx.basic_attr.hw_version,
	dev_ctx.basic_attr.mf_name,
	dev_ctx.basic_attr.model_id,
	dev_ctx.basic_attr.date_code,
	&dev_ctx.basic_attr.power_source,
	dev_ctx.basic_attr.location_id,
	&dev_ctx.basic_attr.ph_env,
	dev_ctx.basic_attr.sw_ver);

ZB_ZCL_DECLARE_IDENTIFY_ATTRIB_LIST(
	identify_attr_list,
	&dev_ctx.identify_attr.identify_time);

ZB_ZCL_DECLARE_GROUPS_ATTRIB_LIST(
	groups_attr_list,
	&dev_ctx.groups_attr.name_support);

/* On/Off attribute list (controllable kettle state) */
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(on_off_attr_list, ZB_ZCL_ON_OFF)
ZB_ZCL_SET_ATTR_DESC_M(ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
	(&dev_ctx.on_off_attr.on_off),
	ZB_ZCL_ATTR_TYPE_BOOL,
	ZB_ZCL_ATTR_ACCESS_READ_WRITE | ZB_ZCL_ATTR_ACCESS_REPORTING)
ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST;

/* Thermostat cluster attributes
 * Use _M macro for attributes that need reporting access flag
 */
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(thermostat_attr_list, ZB_ZCL_THERMOSTAT)
ZB_ZCL_SET_ATTR_DESC_M(ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
	(&dev_ctx.thermostat_attr.local_temperature),
	ZB_ZCL_ATTR_TYPE_S16,
	ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING)
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_COOLING_SETPOINT_ID,
	(&dev_ctx.thermostat_attr.occupied_cooling_setpoint))
ZB_ZCL_SET_ATTR_DESC_M(ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
	(&dev_ctx.thermostat_attr.occupied_heating_setpoint),
	ZB_ZCL_ATTR_TYPE_S16,
	ZB_ZCL_ATTR_ACCESS_READ_WRITE | ZB_ZCL_ATTR_ACCESS_REPORTING)
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MIN_HEAT_SETPOINT_LIMIT_ID,
	(&dev_ctx.thermostat_attr.min_heat_setpoint_limit))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_MAX_HEAT_SETPOINT_LIMIT_ID,
	(&dev_ctx.thermostat_attr.max_heat_setpoint_limit))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_THERMOSTAT_CONTROL_SEQUENCE_OF_OPERATION_ID,
	(&dev_ctx.thermostat_attr.control_sequence))
ZB_ZCL_SET_ATTR_DESC_M(ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
	(&dev_ctx.thermostat_attr.system_mode),
	ZB_ZCL_ATTR_TYPE_8BIT_ENUM,
	ZB_ZCL_ATTR_ACCESS_READ_WRITE | ZB_ZCL_ATTR_ACCESS_REPORTING)
ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST;

/* Temperature measurement cluster attributes */
ZB_ZCL_START_DECLARE_ATTRIB_LIST_CLUSTER_REVISION(temp_measurement_attr_list, ZB_ZCL_TEMP_MEASUREMENT)
ZB_ZCL_SET_ATTR_DESC_M(ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
	(&dev_ctx.temp_measurement_attr.measured_value),
	ZB_ZCL_ATTR_TYPE_S16,
	ZB_ZCL_ATTR_ACCESS_READ_ONLY | ZB_ZCL_ATTR_ACCESS_REPORTING)
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_TEMP_MEASUREMENT_MIN_VALUE_ID,
	(&dev_ctx.temp_measurement_attr.min_measured_value))
ZB_ZCL_SET_ATTR_DESC(ZB_ZCL_ATTR_TEMP_MEASUREMENT_MAX_VALUE_ID,
	(&dev_ctx.temp_measurement_attr.max_measured_value))
ZB_ZCL_FINISH_DECLARE_ATTRIB_LIST;

ZB_DECLARE_KETTLE_CLUSTER_LIST(
	kettle_clusters,
	basic_attr_list,
	identify_attr_list,
	groups_attr_list,
	on_off_attr_list,
	thermostat_attr_list,
	temp_measurement_attr_list);

ZB_DECLARE_KETTLE_EP(
	kettle_ep,
	KETTLE_ENDPOINT,
	kettle_clusters);

#ifdef CONFIG_ZIGBEE_FOTA
extern zb_af_endpoint_desc_t zigbee_fota_client_ep;

ZBOSS_DECLARE_DEVICE_CTX_2_EP(
	kettle_ctx,
	zigbee_fota_client_ep,
	kettle_ep);
#else
ZBOSS_DECLARE_DEVICE_CTX_1_EP(
	kettle_ctx,
	kettle_ep);
#endif

/* ==========================================================================
 * Temperature Conversion Functions
 * ========================================================================== */

/**
 * Convert ADC value to temperature for target dial (non-linear)
 * Kettle dial outputs 0-5V but NOT linear. We read through buffer + 2:1 divider.
 *
 * Calibration points (original voltage before divider):
 *   0.0V = 100°C,  0.8V = 95°C,  1.7V = 90°C,  2.6V = 80°C
 *   3.7V = 70°C,   4.5V = 60°C,  5.0V = 50°C
 */

/* Lookup table: {voltage_mv, temp_zb} - voltage is BEFORE the 2:1 divider */
static const struct {
	int16_t voltage_mv;
	int16_t temp_zb;
} target_temp_lut[] = {
	{    0, 10000 },  /* 0.0V = 100°C */
	{  800,  9500 },  /* 0.8V = 95°C */
	{ 1700,  9000 },  /* 1.7V = 90°C */
	{ 2600,  8000 },  /* 2.6V = 80°C */
	{ 3700,  7000 },  /* 3.7V = 70°C */
	{ 4500,  6000 },  /* 4.5V = 60°C */
	{ 5000,  5000 },  /* 5.0V = 50°C */
};
#define TARGET_TEMP_LUT_SIZE (sizeof(target_temp_lut) / sizeof(target_temp_lut[0]))

static int16_t adc_to_target_temp(int16_t adc_val)
{
	if (adc_val < 0) {
		adc_val = 0;
	}

	/* Convert ADC to voltage (mV) - 3.6V full scale with GAIN_1_4 */
	int32_t adc_mv = (int32_t)adc_val * 3600 / ADC_MAX_VALUE;

	/* Convert to original voltage (before 2:1 divider) */
	int32_t orig_mv = adc_mv * ADC_DIVIDER_RATIO;

	/* Find the two calibration points to interpolate between */
	size_t i;
	for (i = 0; i < TARGET_TEMP_LUT_SIZE - 1; i++) {
		if (orig_mv <= target_temp_lut[i + 1].voltage_mv) {
			break;
		}
	}

	/* Clamp to table bounds */
	if (i >= TARGET_TEMP_LUT_SIZE - 1) {
		return target_temp_lut[TARGET_TEMP_LUT_SIZE - 1].temp_zb;
	}

	/* Linear interpolation between points i and i+1 */
	int32_t v0 = target_temp_lut[i].voltage_mv;
	int32_t v1 = target_temp_lut[i + 1].voltage_mv;
	int32_t t0 = target_temp_lut[i].temp_zb;
	int32_t t1 = target_temp_lut[i + 1].temp_zb;

	/* temp = t0 + (t1 - t0) * (v - v0) / (v1 - v0) */
	int32_t temp_zb = t0 + (t1 - t0) * (orig_mv - v0) / (v1 - v0);

	return (int16_t)temp_zb;
}

/**
 * Convert ADC value to temperature for NTC thermistor (current temperature)
 * Uses calibrated lookup table with linear interpolation.
 *
 * Circuit: 5V -> NTC -> NTC_junction -> 10K -> GND
 * We read NTC_junction through buffer + 2:1 divider
 *
 * Calibration points (original voltage before divider):
 *   1200mV = 25°C,  1900mV = 50°C,  2200mV = 70°C
 *   3000mV = 90°C,  3300mV = 100°C
 */

/* Voltage threshold below which kettle is considered off base (mV, before divider) */
#define KETTLE_OFF_BASE_MV      1000

/* Lookup table: {voltage_mv, temp_zb} - voltage is BEFORE the 2:1 divider */
static const struct {
	int16_t voltage_mv;
	int16_t temp_zb;
} current_temp_lut[] = {
	{ 1200,  2500 },  /* 1.2V = 25°C */
	{ 1900,  5000 },  /* 1.9V = 50°C */
	{ 2200,  7000 },  /* 2.2V = 70°C */
	{ 3000,  9000 },  /* 3.0V = 90°C */
	{ 3300, 10000 },  /* 3.3V = 100°C */
};
#define CURRENT_TEMP_LUT_SIZE (sizeof(current_temp_lut) / sizeof(current_temp_lut[0]))

static int16_t adc_to_current_temp(int16_t adc_val)
{
	if (adc_val < 10) {
		return TEMP_INVALID_ZB;
	}

	/* Convert ADC to voltage (mV) - 3.6V full scale with GAIN_1_4 */
	int32_t adc_mv = (int32_t)adc_val * 3600 / ADC_MAX_VALUE;

	/* Convert to original voltage (before 2:1 divider) */
	int32_t orig_mv = adc_mv * ADC_DIVIDER_RATIO;

	/* Check if kettle is off base (voltage too low) */
	if (orig_mv < KETTLE_OFF_BASE_MV) {
		return TEMP_INVALID_ZB;
	}

	/* Check if voltage is below minimum calibration point but above off-base threshold */
	if (orig_mv < current_temp_lut[0].voltage_mv) {
		/* Extrapolate below 25°C using first two points */
		int32_t v0 = current_temp_lut[0].voltage_mv;
		int32_t v1 = current_temp_lut[1].voltage_mv;
		int32_t t0 = current_temp_lut[0].temp_zb;
		int32_t t1 = current_temp_lut[1].temp_zb;
		int32_t temp_zb = t0 + (t1 - t0) * (orig_mv - v0) / (v1 - v0);
		return (int16_t)(temp_zb < 0 ? 0 : temp_zb);
	}

	/* Find the two calibration points to interpolate between */
	size_t i;
	for (i = 0; i < CURRENT_TEMP_LUT_SIZE - 1; i++) {
		if (orig_mv <= current_temp_lut[i + 1].voltage_mv) {
			break;
		}
	}

	/* Clamp to table bounds */
	if (i >= CURRENT_TEMP_LUT_SIZE - 1) {
		return current_temp_lut[CURRENT_TEMP_LUT_SIZE - 1].temp_zb;
	}

	/* Linear interpolation between points i and i+1 */
	int32_t v0 = current_temp_lut[i].voltage_mv;
	int32_t v1 = current_temp_lut[i + 1].voltage_mv;
	int32_t t0 = current_temp_lut[i].temp_zb;
	int32_t t1 = current_temp_lut[i + 1].temp_zb;

	/* temp = t0 + (t1 - t0) * (v - v0) / (v1 - v0) */
	int32_t temp_zb = t0 + (t1 - t0) * (orig_mv - v0) / (v1 - v0);

	return (int16_t)temp_zb;
}

/* ==========================================================================
 * ADC Sampling
 * ========================================================================== */

/* Forward declarations for reporting helpers */
static void mark_attribute_changed(zb_uint8_t endpoint, zb_uint16_t cluster_id,
				   zb_uint16_t attr_id);
static void schedule_state_report(void);

static void update_temperatures(void)
{
	int ret;
	int16_t target_temp, current_temp;
	struct adc_sequence sequence = {
		.buffer = &adc_buffer,
		.buffer_size = sizeof(adc_buffer),
	};

	/* Sample target temperature (channel 0) */
	ret = adc_sequence_init_dt(&adc_target_temp, &sequence);
	if (ret == 0) {
		ret = adc_read_dt(&adc_target_temp, &sequence);
	}

	if (ret == 0) {
		/* Apply EMA filter to ADC reading */
		if (adc_target_filtered < 0) {
			adc_target_filtered = adc_buffer;  /* First sample */
		} else {
			adc_target_filtered += (adc_buffer - adc_target_filtered) / ADC_FILTER_COEFF;
		}
		int16_t filtered_adc = (int16_t)adc_target_filtered;

		/* Calculate raw voltage: GAIN_1_4 + 0.9V internal ref = 3.6V full scale */
		int32_t adc_mv = (int32_t)filtered_adc * 3600 / ADC_MAX_VALUE;
		int32_t orig_mv = adc_mv * ADC_DIVIDER_RATIO;  /* Voltage before divider */

		target_temp = adc_to_target_temp(filtered_adc);
		int16_t current_setpoint = dev_ctx.thermostat_attr.occupied_heating_setpoint;

		LOG_INF("Target: raw=%d, filt=%d, %dmV, measured=%d.%02d°C, zigbee=%d.%02d°C",
			adc_buffer, filtered_adc, orig_mv,
			target_temp / 100, target_temp % 100,
			current_setpoint / 100, current_setpoint % 100);

		/* Update thermostat setpoint if significantly changed (>0.5°C hysteresis) */
		int16_t diff = target_temp - current_setpoint;
		if (diff < 0) diff = -diff;

		if (diff > 50) {  /* 0.5°C threshold */
			dev_ctx.thermostat_attr.occupied_heating_setpoint = target_temp;

			ZB_ZCL_SET_ATTRIBUTE(
				KETTLE_ENDPOINT,
				ZB_ZCL_CLUSTER_ID_THERMOSTAT,
				ZB_ZCL_CLUSTER_SERVER_ROLE,
				ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID,
				(zb_uint8_t *)&target_temp,
				ZB_FALSE);

			/* Mark for reporting - stack will send based on configured intervals */
			mark_attribute_changed(KETTLE_ENDPOINT, ZB_ZCL_CLUSTER_ID_THERMOSTAT,
				ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID);

			save_kettle_state();
			LOG_INF("Target temp updated to %d.%02d°C", target_temp / 100, target_temp % 100);
		}
	} else {
		LOG_WRN("Target temp ADC read failed: %d", ret);
	}

	/* Sample current temperature (channel 1) */
	ret = adc_sequence_init_dt(&adc_current_temp, &sequence);
	if (ret == 0) {
		ret = adc_read_dt(&adc_current_temp, &sequence);
	}

	if (ret == 0) {
		/* Calculate raw voltage first to check for off-base condition */
		int32_t raw_adc_mv = (int32_t)adc_buffer * 3600 / ADC_MAX_VALUE;
		int32_t raw_orig_mv = raw_adc_mv * ADC_DIVIDER_RATIO;

		/* Check if kettle is off base before filtering */
		if (raw_orig_mv < KETTLE_OFF_BASE_MV) {
			/* Kettle off base - reset filter and report invalid */
			adc_current_filtered = -1;
			current_temp = TEMP_INVALID_ZB;

			LOG_INF("Current: raw=%d, %dmV, OFF BASE (kettle lifted)", adc_buffer, raw_orig_mv);

			/* Report invalid temperature to Zigbee if it changed */
			if (dev_ctx.temp_measurement_attr.measured_value != TEMP_INVALID_ZB) {
				dev_ctx.temp_measurement_attr.measured_value = TEMP_INVALID_ZB;
				dev_ctx.thermostat_attr.local_temperature = TEMP_INVALID_ZB;

				ZB_ZCL_SET_ATTRIBUTE(
					KETTLE_ENDPOINT,
					ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
					ZB_ZCL_CLUSTER_SERVER_ROLE,
					ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
					(zb_uint8_t *)&current_temp,
					ZB_FALSE);

				ZB_ZCL_SET_ATTRIBUTE(
					KETTLE_ENDPOINT,
					ZB_ZCL_CLUSTER_ID_THERMOSTAT,
					ZB_ZCL_CLUSTER_SERVER_ROLE,
					ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
					(zb_uint8_t *)&current_temp,
					ZB_FALSE);

				/* Mark attributes for reporting */
				mark_attribute_changed(KETTLE_ENDPOINT, ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
					ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
				mark_attribute_changed(KETTLE_ENDPOINT, ZB_ZCL_CLUSTER_ID_THERMOSTAT,
					ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID);

				LOG_INF("Kettle off base - marked for reporting");
			}
		} else {
			/* Apply EMA filter to ADC reading */
			if (adc_current_filtered < 0) {
				adc_current_filtered = adc_buffer;  /* First sample */
			} else {
				adc_current_filtered += (adc_buffer - adc_current_filtered) / ADC_FILTER_COEFF;
			}
			int16_t filtered_adc = (int16_t)adc_current_filtered;

			/* Calculate filtered voltage */
			int32_t adc_mv_cur = (int32_t)filtered_adc * 3600 / ADC_MAX_VALUE;
			int32_t orig_mv_cur = adc_mv_cur * ADC_DIVIDER_RATIO;

			current_temp = adc_to_current_temp(filtered_adc);
			int16_t current_zb = dev_ctx.temp_measurement_attr.measured_value;

			if (current_temp != TEMP_INVALID_ZB) {
				LOG_INF("Current: raw=%d, filt=%d, %dmV, measured=%d.%02d°C, zigbee=%d.%02d°C",
					adc_buffer, filtered_adc, orig_mv_cur,
					current_temp / 100, current_temp % 100,
					current_zb / 100, current_zb % 100);
			} else {
				LOG_INF("Current: raw=%d, filt=%d, %dmV, INVALID", adc_buffer, filtered_adc, orig_mv_cur);
			}

			if (current_temp != TEMP_INVALID_ZB) {
				/* Check if temperature changed significantly (>0.5°C) */
				int16_t diff = current_temp - dev_ctx.temp_measurement_attr.measured_value;
				if (diff < 0) diff = -diff;

				if (diff > 50 || dev_ctx.temp_measurement_attr.measured_value == TEMP_INVALID_ZB) {
					/* Update both temperature measurement and thermostat local temp */
					dev_ctx.temp_measurement_attr.measured_value = current_temp;
					dev_ctx.thermostat_attr.local_temperature = current_temp;

					ZB_ZCL_SET_ATTRIBUTE(
						KETTLE_ENDPOINT,
						ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
						ZB_ZCL_CLUSTER_SERVER_ROLE,
						ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
						(zb_uint8_t *)&current_temp,
						ZB_FALSE);

					ZB_ZCL_SET_ATTRIBUTE(
						KETTLE_ENDPOINT,
						ZB_ZCL_CLUSTER_ID_THERMOSTAT,
						ZB_ZCL_CLUSTER_SERVER_ROLE,
						ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID,
						(zb_uint8_t *)&current_temp,
						ZB_FALSE);

					/* Mark attributes for reporting - stack manages timing */
					mark_attribute_changed(KETTLE_ENDPOINT, ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
						ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID);
					mark_attribute_changed(KETTLE_ENDPOINT, ZB_ZCL_CLUSTER_ID_THERMOSTAT,
						ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID);

					LOG_INF("Current temp: %d.%02d°C", current_temp / 100, current_temp % 100);
				}
			}
		}  /* end of else (kettle on base) */
	} else {
		LOG_WRN("Current temp ADC read failed: %d", ret);
	}
}

static void adc_sample_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	update_temperatures();

	/* Schedule next sample */
	k_work_schedule(&adc_sample_work, K_MSEC(ADC_SAMPLE_INTERVAL_MS));
}

/* ==========================================================================
 * Kettle State Machine and GPIO Handling
 * ========================================================================== */

static const char *kettle_state_name(kettle_state_t state)
{
	switch (state) {
	case KETTLE_STATE_OFF: return "OFF";
	case KETTLE_STATE_TURNING_ON: return "TURNING_ON";
	case KETTLE_STATE_ON: return "ON";
	case KETTLE_STATE_TURNING_OFF: return "TURNING_OFF";
	default: return "UNKNOWN";
	}
}

static void report_kettle_on_off(zb_bool_t on)
{
	dev_ctx.on_off_attr.on_off = on;

	ZB_ZCL_SET_ATTRIBUTE(
		KETTLE_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_ON_OFF,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
		(zb_uint8_t *)&on,
		ZB_FALSE);

	/* Update thermostat system mode based on kettle state */
	zb_uint8_t system_mode = on ?
		ZB_ZCL_THERMOSTAT_SYSTEM_MODE_HEAT : ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;
	dev_ctx.thermostat_attr.system_mode = system_mode;

	ZB_ZCL_SET_ATTRIBUTE(
		KETTLE_ENDPOINT,
		ZB_ZCL_CLUSTER_ID_THERMOSTAT,
		ZB_ZCL_CLUSTER_SERVER_ROLE,
		ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID,
		&system_mode,
		ZB_FALSE);

	/* Schedule immediate report via ZBOSS callback */
	schedule_state_report();

	LOG_INF("Kettle state changed: %s (system_mode=%d)", on ? "ON" : "OFF", system_mode);
}

static void kettle_transition_timeout_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (kettle_heating_state == KETTLE_STATE_TURNING_ON) {
		/* Timeout waiting for kettle to start heating - kettle declined */
		LOG_WRN("Kettle declined to heat (timeout) - no water?");
		kettle_heating_state = KETTLE_STATE_OFF;
		report_kettle_on_off(ZB_FALSE);
	} else if (kettle_heating_state == KETTLE_STATE_TURNING_OFF) {
		/* Timeout waiting for kettle to stop - unusual, just report current state */
		LOG_WRN("Kettle turn-off timeout");
		bool actual_state = gpio_pin_get_dt(&kettle_state_gpio);
		kettle_heating_state = actual_state ? KETTLE_STATE_ON : KETTLE_STATE_OFF;
		report_kettle_on_off(actual_state ? ZB_TRUE : ZB_FALSE);
	}
}

static void update_kettle_state(void)
{
	bool gpio_heating = gpio_pin_get_dt(&kettle_state_gpio) ? true : false;
	kettle_state_t prev_state = kettle_heating_state;

	switch (kettle_heating_state) {
	case KETTLE_STATE_OFF:
		if (gpio_heating) {
			/* Kettle started heating (physical button or external) */
			kettle_heating_state = KETTLE_STATE_ON;
			report_kettle_on_off(ZB_TRUE);
			LOG_INF("Kettle heating started");
		}
		break;

	case KETTLE_STATE_TURNING_ON:
		if (gpio_heating) {
			/* Transition complete - kettle accepted the command */
			k_work_cancel_delayable(&kettle_transition_timeout_work);
			kettle_heating_state = KETTLE_STATE_ON;
			report_kettle_on_off(ZB_TRUE);
			LOG_INF("Kettle heating started (command accepted)");
		}
		/* If not heating yet, wait for timeout */
		break;

	case KETTLE_STATE_ON:
		if (!gpio_heating) {
			/* Kettle stopped heating (reached temp, manual off, or lifted) */
			kettle_heating_state = KETTLE_STATE_OFF;
			report_kettle_on_off(ZB_FALSE);
			LOG_INF("Kettle heating stopped");
		}
		break;

	case KETTLE_STATE_TURNING_OFF:
		if (!gpio_heating) {
			/* Transition complete - kettle turned off */
			k_work_cancel_delayable(&kettle_transition_timeout_work);
			kettle_heating_state = KETTLE_STATE_OFF;
			report_kettle_on_off(ZB_FALSE);
			LOG_INF("Kettle heating stopped (command accepted)");
		}
		/* If still heating, wait for timeout */
		break;
	}

	if (prev_state != kettle_heating_state) {
		LOG_INF("Kettle state: %s -> %s",
			kettle_state_name(prev_state),
			kettle_state_name(kettle_heating_state));
	}
}

static void kettle_state_gpio_handler(const struct device *dev,
				      struct gpio_callback *cb,
				      uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	update_kettle_state();
}

/* ==========================================================================
 * Kettle Button Simulation - Pulse GPIO to simulate physical button press
 * ========================================================================== */

static void kettle_button_release_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	/* Release the simulated button press */
	gpio_pin_set_dt(&kettle_button_gpio, 0);
	LOG_DBG("Kettle button released");
}

/**
 * Simulate a button press on the kettle by pulsing the GPIO output.
 * This pulls the kettle's 5V button line low via the MOSFET for a short duration.
 */
static void simulate_kettle_button_press(void)
{
	if (!device_is_ready(kettle_button_gpio.port)) {
		LOG_WRN("Kettle button GPIO not ready");
		return;
	}

	LOG_INF("Simulating kettle button press");

	/* Press the button (pull line low via MOSFET) */
	gpio_pin_set_dt(&kettle_button_gpio, 1);

	/* Schedule release after pulse duration */
	k_work_schedule(&kettle_button_release_work, K_MSEC(KETTLE_BUTTON_PULSE_MS));
}

/**
 * Request kettle to turn on via Zigbee command.
 * Simulates button press and starts transition timeout.
 */
static void request_kettle_on(void)
{
	if (kettle_heating_state == KETTLE_STATE_ON ||
	    kettle_heating_state == KETTLE_STATE_TURNING_ON) {
		LOG_INF("Kettle already on or turning on");
		return;
	}

	LOG_INF("Requesting kettle ON");
	kettle_heating_state = KETTLE_STATE_TURNING_ON;
	simulate_kettle_button_press();

	/* Start timeout - if kettle doesn't respond, it declined */
	k_work_schedule(&kettle_transition_timeout_work,
			K_MSEC(KETTLE_TRANSITION_TIMEOUT_MS));
}

/**
 * Request kettle to turn off via Zigbee command.
 * Simulates button press and starts transition timeout.
 */
static void request_kettle_off(void)
{
	if (kettle_heating_state == KETTLE_STATE_OFF ||
	    kettle_heating_state == KETTLE_STATE_TURNING_OFF) {
		LOG_INF("Kettle already off or turning off");
		return;
	}

	LOG_INF("Requesting kettle OFF");
	kettle_heating_state = KETTLE_STATE_TURNING_OFF;
	simulate_kettle_button_press();

	/* Start timeout */
	k_work_schedule(&kettle_transition_timeout_work,
			K_MSEC(KETTLE_TRANSITION_TIMEOUT_MS));
}

/* ==========================================================================
 * Status LED - Blinks when not joined, off when joined
 * ========================================================================== */

static struct k_work_delayable status_led_work;

static void status_led_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (!device_is_ready(status_led.port)) {
		return;
	}

	if (ZB_JOINED()) {
		/* Joined - LED off, stop blinking */
		gpio_pin_set_dt(&status_led, 0);
	} else {
		/* Not joined - toggle LED and reschedule */
		gpio_pin_toggle_dt(&status_led);
		k_work_schedule(&status_led_work, K_MSEC(500));
	}
}

static void update_status_led(void)
{
	if (!device_is_ready(status_led.port)) {
		return;
	}

	if (ZB_JOINED()) {
		/* Joined - ensure LED is off and stop blinking */
		k_work_cancel_delayable(&status_led_work);
		gpio_pin_set_dt(&status_led, 0);
	} else {
		/* Not joined - start blinking if not already */
		if (!k_work_delayable_is_pending(&status_led_work)) {
			k_work_schedule(&status_led_work, K_NO_WAIT);
		}
	}
}

/* ==========================================================================
 * Button Handling
 * ========================================================================== */

static void button_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	bool pressed = (gpio_pin_get_dt(&button) == 1);

	if (pressed && !button_state.pressed) {
		/* Button pressed */
		button_state.pressed = true;
		button_state.press_time = k_uptime_get();
		k_work_schedule(&long_press_work, K_MSEC(BUTTON_LONG_PRESS_MS));
		LOG_INF("Pairing button pressed");
	} else if (!pressed && button_state.pressed) {
		/* Button released */
		button_state.pressed = false;
		k_work_cancel_delayable(&long_press_work);

		int64_t duration = k_uptime_get() - button_state.press_time;
		if (duration < BUTTON_LONG_PRESS_MS) {
			/* Short press - just log, no action */
			LOG_INF("Pairing button short press (%lld ms) - ignored", duration);
		}
	}
}

static void long_press_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	if (button_state.pressed) {
		LOG_INF("Long press - factory reset and pairing");

		/* Blink LED to indicate reset */
		for (int i = 0; i < 6; i++) {
			gpio_pin_toggle_dt(&status_led);
			k_msleep(100);
		}

		/* Leave network and restart steering */
		if (ZB_JOINED()) {
			zb_bdb_reset_via_local_action(0);
		}
	}
}

static void button_gpio_handler(const struct device *dev,
				struct gpio_callback *cb,
				uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	/* Log interrupt trigger with raw GPIO state */
	int raw_state = gpio_pin_get_raw(button.port, button.pin);
	int logical_state = gpio_pin_get_dt(&button);
	LOG_INF("Button IRQ: raw=%d, logical=%d", raw_state, logical_state);

	k_work_submit(&button_work);
}

/* ==========================================================================
 * Initialization
 * ========================================================================== */

static int button_init(void)
{
	int ret;

	if (!device_is_ready(button.port)) {
		LOG_ERR("Pairing button device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Pairing button config failed: %d", ret);
		return ret;
	}

	/* Note: GPIO interrupts not working on nRF54L15, using polling instead */
	gpio_init_callback(&button_cb_data, button_gpio_handler, BIT(button.pin));
	ret = gpio_add_callback(button.port, &button_cb_data);
	if (ret < 0) {
		LOG_WRN("Pairing button callback add failed: %d (using polling)", ret);
	}

	k_work_init(&button_work, button_work_handler);
	k_work_init_delayable(&long_press_work, long_press_work_handler);

	LOG_INF("Pairing button initialized (P2.%d)", button.pin);
	return 0;
}

static int kettle_state_init(void)
{
	int ret;

	if (!device_is_ready(kettle_state_gpio.port)) {
		LOG_ERR("Kettle state GPIO device not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&kettle_state_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Kettle state GPIO config failed: %d", ret);
		return ret;
	}

	/* Note: GPIO interrupts not working on nRF54L15, using polling instead */
	gpio_init_callback(&kettle_state_cb_data, kettle_state_gpio_handler,
			   BIT(kettle_state_gpio.pin));
	ret = gpio_add_callback(kettle_state_gpio.port, &kettle_state_cb_data);
	if (ret < 0) {
		LOG_WRN("Kettle state callback add failed: %d (using polling)", ret);
	}

	/* Initialize state machine from current GPIO state */
	bool initial_heating = gpio_pin_get_dt(&kettle_state_gpio) ? true : false;
	kettle_heating_state = initial_heating ? KETTLE_STATE_ON : KETTLE_STATE_OFF;
	report_kettle_on_off(initial_heating ? ZB_TRUE : ZB_FALSE);

	LOG_INF("Kettle state GPIO initialized (heating=%s)",
		initial_heating ? "ON" : "OFF");
	return 0;
}

static int adc_init(void)
{
	int ret;

	/* Check if ADC channels are ready */
	if (!adc_is_ready_dt(&adc_target_temp)) {
		LOG_ERR("ADC target temp channel not ready");
		return -ENODEV;
	}

	if (!adc_is_ready_dt(&adc_current_temp)) {
		LOG_ERR("ADC current temp channel not ready");
		return -ENODEV;
	}

	/* Configure channel 0 (target temperature) */
	ret = adc_channel_setup_dt(&adc_target_temp);
	if (ret < 0) {
		LOG_ERR("ADC channel 0 setup failed: %d", ret);
		return ret;
	}

	/* Configure channel 1 (current temperature) */
	ret = adc_channel_setup_dt(&adc_current_temp);
	if (ret < 0) {
		LOG_ERR("ADC channel 1 setup failed: %d", ret);
		return ret;
	}

	k_work_init_delayable(&adc_sample_work, adc_sample_work_handler);

	LOG_INF("ADC initialized");
	return 0;
}

/* ==========================================================================
 * Zigbee Reporting
 *
 * Hybrid approach:
 * - Temperature: Uses stack's automatic reporting (frequent, benefits from
 *   stack-managed timing and buffering)
 * - On/Off + System Mode: Manual report via ZBOSS callback (infrequent state
 *   changes need immediate feedback to coordinator)
 * ========================================================================== */

/* Forward declarations for ZBOSS callbacks */
static void send_state_report_cb(zb_uint8_t param);
static void send_system_mode_report_cb(zb_uint8_t param);

/**
 * Mark an attribute as changed to trigger the stack's automatic reporting.
 * Used for temperature attributes where timing isn't critical.
 */
static void mark_attribute_changed(zb_uint8_t endpoint, zb_uint16_t cluster_id,
				   zb_uint16_t attr_id)
{
	zb_zcl_mark_attr_for_reporting(endpoint, cluster_id, ZB_ZCL_CLUSTER_SERVER_ROLE, attr_id);
}

/**
 * ZBOSS callback to send on/off and system_mode reports.
 * Runs in ZBOSS context for proper buffer management.
 */
static void send_state_report_cb(zb_uint8_t param)
{
	zb_bufid_t bufid;
	zb_uint8_t *cmd_ptr;
	zb_uint16_t dst_addr = 0x0000;  /* Coordinator */

	if (!ZB_JOINED()) {
		if (param) {
			zb_buf_free(param);
		}
		return;
	}

	/* Get a buffer for the report */
	bufid = param ? param : zb_buf_get_out();
	if (!bufid) {
		LOG_WRN("No buffer for state report, scheduling retry");
		ZB_SCHEDULE_APP_ALARM(send_state_report_cb, 0, ZB_TIME_ONE_SECOND);
		return;
	}

	/* Build On/Off report - frame control: server-to-client, global cmd, disable default response */
	cmd_ptr = ZB_ZCL_START_PACKET(bufid);
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, 0x18);  /* 0x08 (srv->cli) | 0x10 (disable default resp) */
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, ZB_ZCL_GET_SEQ_NUM());
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, ZB_ZCL_CMD_REPORT_ATTRIB);

	/* On/Off attribute */
	ZB_ZCL_PACKET_PUT_DATA16_VAL(cmd_ptr, ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID);
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, ZB_ZCL_ATTR_TYPE_BOOL);
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, dev_ctx.on_off_attr.on_off);

	ZB_ZCL_FINISH_PACKET(bufid, cmd_ptr)
	ZB_ZCL_SEND_COMMAND_SHORT(bufid, dst_addr, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
				  1, KETTLE_ENDPOINT, ZB_AF_HA_PROFILE_ID,
				  ZB_ZCL_CLUSTER_ID_ON_OFF, NULL);

	LOG_INF("Sent on_off report: %d", dev_ctx.on_off_attr.on_off);

	/* Schedule system_mode report (separate buffer needed) */
	ZB_SCHEDULE_APP_CALLBACK(send_system_mode_report_cb, 0);
}

/**
 * ZBOSS callback to send system_mode report.
 */
static void send_system_mode_report_cb(zb_uint8_t param)
{
	zb_bufid_t bufid;
	zb_uint8_t *cmd_ptr;
	zb_uint16_t dst_addr = 0x0000;  /* Coordinator */

	if (!ZB_JOINED()) {
		if (param) {
			zb_buf_free(param);
		}
		return;
	}

	bufid = param ? param : zb_buf_get_out();
	if (!bufid) {
		LOG_WRN("No buffer for system_mode report, scheduling retry");
		ZB_SCHEDULE_APP_ALARM(send_system_mode_report_cb, 0, ZB_TIME_ONE_SECOND);
		return;
	}

	/* Build system_mode report - frame control: server-to-client, global cmd, disable default response */
	cmd_ptr = ZB_ZCL_START_PACKET(bufid);
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, 0x18);  /* 0x08 (srv->cli) | 0x10 (disable default resp) */
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, ZB_ZCL_GET_SEQ_NUM());
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, ZB_ZCL_CMD_REPORT_ATTRIB);

	/* System mode attribute */
	ZB_ZCL_PACKET_PUT_DATA16_VAL(cmd_ptr, ZB_ZCL_ATTR_THERMOSTAT_SYSTEM_MODE_ID);
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, ZB_ZCL_ATTR_TYPE_8BIT_ENUM);
	ZB_ZCL_PACKET_PUT_DATA8(cmd_ptr, dev_ctx.thermostat_attr.system_mode);

	ZB_ZCL_FINISH_PACKET(bufid, cmd_ptr)
	ZB_ZCL_SEND_COMMAND_SHORT(bufid, dst_addr, ZB_APS_ADDR_MODE_16_ENDP_PRESENT,
				  1, KETTLE_ENDPOINT, ZB_AF_HA_PROFILE_ID,
				  ZB_ZCL_CLUSTER_ID_THERMOSTAT, NULL);

	LOG_INF("Sent system_mode report: %d", dev_ctx.thermostat_attr.system_mode);
}

/**
 * Schedule state reports via ZBOSS callback (proper context for buffer ops)
 */
static void schedule_state_report(void)
{
	if (ZB_JOINED()) {
		ZB_SCHEDULE_APP_CALLBACK(send_state_report_cb, 0);
	}
}

static void configure_reporting(void)
{
	zb_zcl_reporting_info_t rep_info;
	zb_ret_t ret;

	LOG_INF("Configuring attribute reporting...");

	/* Note: On/Off and System Mode are reported manually via schedule_state_report()
	 * to ensure immediate feedback. Only temperature attributes use automatic reporting. */

	/* Configure Temperature Measurement reporting
	 * min_interval: 5s (responsive during active heating)
	 * max_interval: 300s (heartbeat when idle)
	 * delta: 50 = 0.5°C change threshold
	 *
	 * During boiling: ~0.3°C/sec rise, so reports every 5-10s
	 * When idle: only heartbeat every 5 min
	 */
	memset(&rep_info, 0, sizeof(rep_info));
	rep_info.direction = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	rep_info.ep = KETTLE_ENDPOINT;
	rep_info.cluster_id = ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
	rep_info.cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE;
	rep_info.attr_id = ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
	rep_info.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	rep_info.dst.endpoint = 1;
	rep_info.dst.short_addr = 0x0000;
	rep_info.u.send_info.min_interval = 5;
	rep_info.u.send_info.max_interval = 300;
	rep_info.u.send_info.delta.s16 = 50;  /* 0.5°C change */
	rep_info.flags = ZB_ZCL_REPORTING_SLOT_BUSY;

	ret = zb_zcl_put_reporting_info(&rep_info, ZB_TRUE);
	LOG_INF("Temp measurement reporting: %s", ret == RET_OK ? "OK" : "FAILED");

	/* Configure Thermostat local temperature reporting (mirrors temp measurement) */
	memset(&rep_info, 0, sizeof(rep_info));
	rep_info.direction = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	rep_info.ep = KETTLE_ENDPOINT;
	rep_info.cluster_id = ZB_ZCL_CLUSTER_ID_THERMOSTAT;
	rep_info.cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE;
	rep_info.attr_id = ZB_ZCL_ATTR_THERMOSTAT_LOCAL_TEMPERATURE_ID;
	rep_info.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	rep_info.dst.endpoint = 1;
	rep_info.dst.short_addr = 0x0000;
	rep_info.u.send_info.min_interval = 5;
	rep_info.u.send_info.max_interval = 300;
	rep_info.u.send_info.delta.s16 = 50;  /* 0.5°C change */
	rep_info.flags = ZB_ZCL_REPORTING_SLOT_BUSY;

	ret = zb_zcl_put_reporting_info(&rep_info, ZB_TRUE);
	LOG_INF("Thermostat local temp reporting: %s", ret == RET_OK ? "OK" : "FAILED");

	/* Configure Thermostat setpoint reporting */
	memset(&rep_info, 0, sizeof(rep_info));
	rep_info.direction = ZB_ZCL_CONFIGURE_REPORTING_SEND_REPORT;
	rep_info.ep = KETTLE_ENDPOINT;
	rep_info.cluster_id = ZB_ZCL_CLUSTER_ID_THERMOSTAT;
	rep_info.cluster_role = ZB_ZCL_CLUSTER_SERVER_ROLE;
	rep_info.attr_id = ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID;
	rep_info.dst.profile_id = ZB_AF_HA_PROFILE_ID;
	rep_info.dst.endpoint = 1;
	rep_info.dst.short_addr = 0x0000;
	rep_info.u.send_info.min_interval = 10;
	rep_info.u.send_info.max_interval = 3600;  /* Setpoint rarely changes */
	rep_info.u.send_info.delta.s16 = 100;  /* 1.0°C change */
	rep_info.flags = ZB_ZCL_REPORTING_SLOT_BUSY;

	ret = zb_zcl_put_reporting_info(&rep_info, ZB_TRUE);
	LOG_INF("Thermostat setpoint reporting: %s", ret == RET_OK ? "OK" : "FAILED");

	LOG_INF("Attribute reporting configured");
}

static void clusters_attr_init(void)
{
	/* Basic cluster */
	dev_ctx.basic_attr.zcl_version = ZB_ZCL_VERSION;
	dev_ctx.basic_attr.app_version = KETTLE_INIT_BASIC_APP_VERSION;
	dev_ctx.basic_attr.stack_version = KETTLE_INIT_BASIC_STACK_VERSION;
	dev_ctx.basic_attr.hw_version = KETTLE_INIT_BASIC_HW_VERSION;
	dev_ctx.basic_attr.power_source = ZB_ZCL_BASIC_POWER_SOURCE_MAINS_SINGLE_PHASE;
	dev_ctx.basic_attr.ph_env = KETTLE_INIT_BASIC_PH_ENV;

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.mf_name,
		KETTLE_INIT_BASIC_MANUF_NAME,
		ZB_ZCL_STRING_CONST_SIZE(KETTLE_INIT_BASIC_MANUF_NAME));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.model_id,
		KETTLE_INIT_BASIC_MODEL_ID,
		ZB_ZCL_STRING_CONST_SIZE(KETTLE_INIT_BASIC_MODEL_ID));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.date_code,
		KETTLE_INIT_BASIC_DATE_CODE,
		ZB_ZCL_STRING_CONST_SIZE(KETTLE_INIT_BASIC_DATE_CODE));

	ZB_ZCL_SET_STRING_VAL(
		dev_ctx.basic_attr.location_id,
		KETTLE_INIT_BASIC_LOCATION_DESC,
		ZB_ZCL_STRING_CONST_SIZE(KETTLE_INIT_BASIC_LOCATION_DESC));

	/* Identify cluster */
	dev_ctx.identify_attr.identify_time = ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE;

	/* On/Off cluster (read-only, reports kettle state) */
	dev_ctx.on_off_attr.on_off = ZB_ZCL_ON_OFF_IS_OFF;

	/* Thermostat cluster */
	dev_ctx.thermostat_attr.local_temperature = TEMP_INVALID_ZB;
	dev_ctx.thermostat_attr.occupied_cooling_setpoint = TEMP_MAX_ZB;  /* Not used */
	dev_ctx.thermostat_attr.occupied_heating_setpoint = 8000;  /* Default 80°C */
	dev_ctx.thermostat_attr.min_heat_setpoint_limit = TEMP_MIN_ZB;
	dev_ctx.thermostat_attr.max_heat_setpoint_limit = TEMP_MAX_ZB;
	dev_ctx.thermostat_attr.control_sequence = ZB_ZCL_THERMOSTAT_CONTROL_SEQ_OF_OPERATION_HEATING_ONLY;
	dev_ctx.thermostat_attr.system_mode = ZB_ZCL_THERMOSTAT_SYSTEM_MODE_OFF;

	/* Temperature measurement cluster */
	dev_ctx.temp_measurement_attr.measured_value = TEMP_INVALID_ZB;
	dev_ctx.temp_measurement_attr.min_measured_value = TEMP_MIN_ZB;
	dev_ctx.temp_measurement_attr.max_measured_value = TEMP_MAX_ZB;
}

/* ==========================================================================
 * Zigbee FOTA (Over-The-Air Updates)
 * ========================================================================== */

#ifdef CONFIG_ZIGBEE_FOTA
static void fota_evt_handler(const struct zigbee_fota_evt *evt)
{
	switch (evt->id) {
	case ZIGBEE_FOTA_EVT_PROGRESS:
		LOG_INF("OTA progress: %d%%", evt->dl.progress);
		/* Blink status LED during download */
		if (device_is_ready(status_led.port)) {
			gpio_pin_toggle_dt(&status_led);
		}
		break;

	case ZIGBEE_FOTA_EVT_FINISHED:
		LOG_INF("OTA download complete, rebooting...");
		sys_reboot(SYS_REBOOT_COLD);
		break;

	case ZIGBEE_FOTA_EVT_ERROR:
		LOG_ERR("OTA transfer failed");
		break;

	default:
		break;
	}
}
#endif

/* ==========================================================================
 * Zigbee Callbacks
 * ========================================================================== */

static void zcl_device_cb(zb_bufid_t bufid)
{
	zb_zcl_device_callback_param_t *param =
		ZB_BUF_GET_PARAM(bufid, zb_zcl_device_callback_param_t);

	param->status = RET_OK;

	switch (param->device_cb_id) {
	case ZB_ZCL_SET_ATTR_VALUE_CB_ID:
		/* Handle On/Off commands */
		if (param->cb_param.set_attr_value_param.cluster_id ==
		    ZB_ZCL_CLUSTER_ID_ON_OFF) {
			if (param->cb_param.set_attr_value_param.attr_id ==
			    ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID) {
				zb_bool_t requested_state = param->cb_param.set_attr_value_param.values.data8;
				LOG_INF("On/Off command: %s", requested_state ? "ON" : "OFF");

				if (requested_state) {
					request_kettle_on();
				} else {
					request_kettle_off();
				}
				/* Note: The actual state will be updated by kettle_state_gpio
				 * when the kettle responds, or by timeout if it declines */
			}
		}
		/* Handle thermostat setpoint changes from Zigbee */
		else if (param->cb_param.set_attr_value_param.cluster_id ==
		    ZB_ZCL_CLUSTER_ID_THERMOSTAT) {
			if (param->cb_param.set_attr_value_param.attr_id ==
			    ZB_ZCL_ATTR_THERMOSTAT_OCCUPIED_HEATING_SETPOINT_ID) {
				int16_t new_setpoint = param->cb_param.set_attr_value_param.values.data16;
				LOG_INF("Thermostat setpoint changed: %d.%02d°C",
					new_setpoint / 100, new_setpoint % 100);
				save_kettle_state();
			}
		}
		break;

#ifdef CONFIG_ZIGBEE_FOTA
	case ZB_ZCL_OTA_UPGRADE_VALUE_CB_ID:
		zigbee_fota_zcl_cb(bufid);
		break;
#endif

	default:
		param->status = RET_NOT_IMPLEMENTED;
		break;
	}
}

void zboss_signal_handler(zb_bufid_t bufid)
{
	zb_zdo_app_signal_hdr_t *sig_hdr = NULL;
	zb_zdo_app_signal_type_t sig_type = zb_get_app_signal(bufid, &sig_hdr);
	zb_ret_t status = ZB_GET_APP_SIGNAL_STATUS(bufid);

	/* Update status LED */
	update_status_led();

#ifdef CONFIG_ZIGBEE_FOTA
	/* Pass signals to FOTA library */
	zigbee_fota_signal_handler(bufid);
#endif

	switch (sig_type) {
	case ZB_ZDO_SIGNAL_PRODUCTION_CONFIG_READY:
		LOG_INF("Production config ready (status=%d)", status);
		break;

	case ZB_ZDO_SIGNAL_SKIP_STARTUP:
		LOG_INF("Skip startup signal");
		break;

	case ZB_BDB_SIGNAL_DEVICE_FIRST_START:
		LOG_INF("Device first start (status=%d)", status);
		if (status == RET_OK) {
			LOG_INF("Starting network steering...");
			bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
		}
		break;

	case ZB_BDB_SIGNAL_DEVICE_REBOOT:
		LOG_INF("Device reboot (status=%d)", status);
		if (status == RET_OK) {
			LOG_INF("Joined Zigbee network as router");
			configure_reporting();
		} else {
			LOG_INF("Not joined, starting network steering...");
			bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
		}
		break;

	case ZB_BDB_SIGNAL_STEERING:
		if (status == RET_OK) {
			LOG_INF("Network steering successful - joined!");
			configure_reporting();
		} else {
			LOG_WRN("Network steering failed (status=%d), retrying...", status);
			bdb_start_top_level_commissioning(ZB_BDB_NETWORK_STEERING);
		}
		break;

	case ZB_NLME_STATUS_INDICATION:
		/* Network layer status - handled internally by stack */
		break;

	default:
		break;
	}

	/* Use default signal handler for other signals */
	ZB_ERROR_CHECK(zigbee_default_signal_handler(bufid));

	if (bufid) {
		zb_buf_free(bufid);
	}
}

/* ==========================================================================
 * Main
 * ========================================================================== */

int main(void)
{
	int err;

	LOG_INF("========================================");
	LOG_INF("KitchenAid 5KEK1522 Zigbee Kettle v1.0.0");
	LOG_INF("Board: %s", CONFIG_BOARD);
	LOG_INF("Role: Zigbee Router");
	LOG_INF("========================================");

	/* Initialize status LED */
	if (device_is_ready(status_led.port)) {
		err = gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			LOG_WRN("Status LED config failed: %d", err);
		}
	}
	k_work_init_delayable(&status_led_work, status_led_work_handler);

	/* Initialize button */
	err = button_init();
	if (err) {
		LOG_ERR("Button init failed: %d", err);
		return err;
	}

	/* Initialize kettle state GPIO */
	err = kettle_state_init();
	if (err) {
		LOG_ERR("Kettle state init failed: %d", err);
		return err;
	}

	/* Initialize kettle button output (for simulating button press) */
	if (device_is_ready(kettle_button_gpio.port)) {
		err = gpio_pin_configure_dt(&kettle_button_gpio, GPIO_OUTPUT_INACTIVE);
		if (err < 0) {
			LOG_ERR("Kettle button GPIO config failed: %d", err);
			return err;
		}
		k_work_init_delayable(&kettle_button_release_work, kettle_button_release_handler);
		k_work_init_delayable(&kettle_transition_timeout_work, kettle_transition_timeout_handler);
		LOG_INF("Kettle button output initialized");
	} else {
		LOG_WRN("Kettle button GPIO not ready");
	}

	/* Initialize ADC for temperature sensing */
	err = adc_init();
	if (err) {
		LOG_ERR("ADC init failed: %d", err);
		return err;
	}

	/* Initialize settings subsystem */
	err = settings_subsys_init();
	if (err) {
		LOG_ERR("Settings init failed: %d", err);
	}

	/* Note: Reporting is handled by ZBOSS stack's built-in reporting mechanism
	 * configured in configure_reporting(). No manual periodic reports needed. */

#ifdef CONFIG_ZIGBEE_FOTA
	/* Initialize OTA client */
	err = zigbee_fota_init(fota_evt_handler);
	if (err) {
		LOG_ERR("FOTA init failed: %d", err);
	}

	/* Confirm current image to prevent rollback on next boot */
	if (!boot_is_img_confirmed()) {
		err = boot_write_img_confirmed();
		if (err) {
			LOG_ERR("Failed to confirm image: %d", err);
		} else {
			LOG_INF("Image confirmed");
		}
	}
#endif

	/* Register ZCL device callback */
	ZB_ZCL_REGISTER_DEVICE_CB(zcl_device_cb);

	/* Register device context */
	ZB_AF_REGISTER_DEVICE_CTX(&kettle_ctx);

	/* Initialize cluster attributes */
	clusters_attr_init();

	/* Load settings (restores previous target temperature) */
	err = settings_load();
	if (err) {
		LOG_ERR("Settings load failed: %d", err);
	}

	/* Start ADC sampling */
	k_work_schedule(&adc_sample_work, K_NO_WAIT);

	LOG_INF("Hold button 3s to reset/pair");
	LOG_INF("Starting Zigbee stack...");

	/* Start Zigbee stack (Router mode - always on) */
	zigbee_enable();

	/* Main loop - poll GPIOs since interrupts aren't working on nRF54L15 */
	static int last_button_state = -1;
	static int last_kettle_gpio_state = -1;

	while (1) {
		/* Poll pairing button */
		int btn = gpio_pin_get_dt(&button);
		if (btn != last_button_state) {
			last_button_state = btn;
			k_work_submit(&button_work);
		}

		/* Poll kettle state GPIO */
		int kettle_gpio = gpio_pin_get_dt(&kettle_state_gpio);
		if (kettle_gpio != last_kettle_gpio_state) {
			LOG_INF("Kettle GPIO: %d -> %d", last_kettle_gpio_state, kettle_gpio);
			last_kettle_gpio_state = kettle_gpio;
			update_kettle_state();
		}

		k_sleep(K_MSEC(50));
	}

	return 0;
}
