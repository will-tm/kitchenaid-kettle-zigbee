/*
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Zigbee Kettle Device Definition
 *
 * This device implements:
 * - Basic Cluster (0x0000) - Device information
 * - Identify Cluster (0x0003) - Device identification
 * - Groups Cluster (0x0004) - Group membership
 * - On/Off Cluster (0x0006) - Kettle power state (read-only reporting)
 * - Thermostat Cluster (0x0201) - Target temperature setpoint
 * - Temperature Measurement Cluster (0x0402) - Current water temperature
 */

#ifndef ZB_KETTLE_H
#define ZB_KETTLE_H 1

#include <zboss_api.h>

/**
 * Kettle Device ID
 * Using HVAC Thermostat device type as closest match
 */
#define ZB_KETTLE_DEVICE_ID 0x0301  /* Thermostat device */

/** Kettle device version */
#define ZB_DEVICE_VER_KETTLE 1

/** Kettle IN (server) clusters number */
#define ZB_KETTLE_IN_CLUSTER_NUM 6

/** Kettle OUT (client) clusters number */
#define ZB_KETTLE_OUT_CLUSTER_NUM 0

/** Total cluster number */
#define ZB_KETTLE_CLUSTER_NUM \
	(ZB_KETTLE_IN_CLUSTER_NUM + ZB_KETTLE_OUT_CLUSTER_NUM)

/** Number of attributes for reporting */
#define ZB_KETTLE_REPORT_ATTR_COUNT 4  /* on_off, temp_measurement, local_temp, occupied_setpoint */

/**
 * @brief Declare cluster list for Kettle device
 */
#define ZB_DECLARE_KETTLE_CLUSTER_LIST(					\
	cluster_list_name,						\
	basic_attr_list,						\
	identify_attr_list,						\
	groups_attr_list,						\
	on_off_attr_list,						\
	thermostat_attr_list,						\
	temp_measurement_attr_list)					\
	zb_zcl_cluster_desc_t cluster_list_name[] =			\
	{								\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_BASIC,			\
			ZB_ZCL_ARRAY_SIZE(basic_attr_list, zb_zcl_attr_t), \
			(basic_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_IDENTIFY,			\
			ZB_ZCL_ARRAY_SIZE(identify_attr_list, zb_zcl_attr_t), \
			(identify_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_GROUPS,			\
			ZB_ZCL_ARRAY_SIZE(groups_attr_list, zb_zcl_attr_t), \
			(groups_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_ON_OFF,			\
			ZB_ZCL_ARRAY_SIZE(on_off_attr_list, zb_zcl_attr_t), \
			(on_off_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_THERMOSTAT,			\
			ZB_ZCL_ARRAY_SIZE(thermostat_attr_list, zb_zcl_attr_t), \
			(thermostat_attr_list),				\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		),							\
		ZB_ZCL_CLUSTER_DESC(					\
			ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,		\
			ZB_ZCL_ARRAY_SIZE(temp_measurement_attr_list, zb_zcl_attr_t), \
			(temp_measurement_attr_list),			\
			ZB_ZCL_CLUSTER_SERVER_ROLE,			\
			ZB_ZCL_MANUF_CODE_INVALID			\
		)							\
	}

/**
 * @brief Declare simple descriptor for Kettle device
 */
#define ZB_ZCL_DECLARE_KETTLE_SIMPLE_DESC(ep_name, ep_id, in_clust_num, out_clust_num) \
	ZB_DECLARE_SIMPLE_DESC(in_clust_num, out_clust_num);				\
	ZB_AF_SIMPLE_DESC_TYPE(in_clust_num, out_clust_num) simple_desc_##ep_name =	\
	{										\
		ep_id,									\
		ZB_AF_HA_PROFILE_ID,							\
		ZB_KETTLE_DEVICE_ID,							\
		ZB_DEVICE_VER_KETTLE,							\
		0,									\
		in_clust_num,								\
		out_clust_num,								\
		{									\
			ZB_ZCL_CLUSTER_ID_BASIC,					\
			ZB_ZCL_CLUSTER_ID_IDENTIFY,					\
			ZB_ZCL_CLUSTER_ID_GROUPS,					\
			ZB_ZCL_CLUSTER_ID_ON_OFF,					\
			ZB_ZCL_CLUSTER_ID_THERMOSTAT,					\
			ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,				\
		}									\
	}

/**
 * @brief Declare endpoint for Kettle device
 */
#define ZB_DECLARE_KETTLE_EP(ep_name, ep_id, cluster_list)			\
	ZB_ZCL_DECLARE_KETTLE_SIMPLE_DESC(ep_name, ep_id,			\
		ZB_KETTLE_IN_CLUSTER_NUM, ZB_KETTLE_OUT_CLUSTER_NUM);		\
	ZBOSS_DEVICE_DECLARE_REPORTING_CTX(reporting_info##ep_name,		\
		ZB_KETTLE_REPORT_ATTR_COUNT);					\
	ZB_AF_DECLARE_ENDPOINT_DESC(ep_name, ep_id, ZB_AF_HA_PROFILE_ID,		\
		0,								\
		NULL,								\
		ZB_ZCL_ARRAY_SIZE(cluster_list, zb_zcl_cluster_desc_t), cluster_list, \
		(zb_af_simple_desc_1_1_t *)&simple_desc_##ep_name,		\
		ZB_KETTLE_REPORT_ATTR_COUNT,					\
		reporting_info##ep_name,					\
		0,								\
		NULL)

#endif /* ZB_KETTLE_H */
