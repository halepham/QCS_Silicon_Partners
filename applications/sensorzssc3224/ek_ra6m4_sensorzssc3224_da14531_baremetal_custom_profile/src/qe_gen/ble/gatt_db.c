/***********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO THIS
 * SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2019-2020 Renesas Electronics Corporation. All rights reserved.
 ***********************************************************************************************************************/

/**
 *  GATT DATABASE QUICK REFERENCE TABLE:
 *  Abbreviations used for PROPERTIES:
 *      BC = Broadcast
 *      RD = Read
 *      WW = Write Without Response
 *      WR = Write
 *      NT = Notification
 *      IN = Indication
 *      RW = Reliable Write
 * 
 *  HANDLE | ATT_TYPE          | PROPERTIES  | ATT_VALUE                        | DEFINITION
 *  ============================================================================================
 *  GAP Service
 *  ============================================================================================
 *  0x0001 | 0x28,0x00         | RD          | 0x00,0x18                        | GAP Service Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0002 | 0x28,0x03         | RD          | 0x0A,0x03,0x00,0x00,0x2A         | Device Name characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0003 | 0x00,0x2A         | RD,WR       | 0x00,0x00,0x00,0x00,0x00,0x00... | Device Name characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0004 | 0x28,0x03         | RD          | 0x02,0x05,0x00,0x01,0x2A         | Appearance characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0005 | 0x01,0x2A         | RD          | 0x00,0x00                        | Appearance characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0006 | 0x28,0x03         | RD          | 0x02,0x07,0x00,0x04,0x2A         | Peripheral Preferred Connection Parameters characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0007 | 0x04,0x2A         | RD          | 0x00,0x00,0x00,0x00,0x00,0x00... | Peripheral Preferred Connection Parameters characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0008 | 0x28,0x03         | RD          | 0x02,0x09,0x00,0xA6,0x2A         | Central Address Resolution characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0009 | 0xA6,0x2A         | RD          | 0x00                             | Central Address Resolution characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x000A | 0x28,0x03         | RD          | 0x02,0x0B,0x00,0xC9,0x2A         | Resolvable Private Address Only characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x000B | 0xC9,0x2A         | RD          | 0x00                             | Resolvable Private Address Only characteristic value
 *  ============================================================================================
 *  GATT Service
 *  ============================================================================================
 *  0x000C | 0x28,0x00         | RD          | 0x01,0x18                        | GATT Service Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x000D | 0x28,0x03         | RD          | 0x20,0x0E,0x00,0x05,0x2A         | Service Changed characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x000E | 0x05,0x2A         | IN          | 0x00,0x00,0x00,0x00              | Service Changed characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x000F | 0x02,0x29         | RD,WR       | 0x00,0x00                        | Client Characteristic Configuration descriptor
 *  ============================================================================================
 *  Quick Connect Service
 *  ============================================================================================
 *  0x0010 | 0x28,0x00         | RD          | 0x59,0x5a,0x08,0xe4,0x86,0x2a... | Quick Connect Service Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0011 | 0x28,0x03         | RD          | 0x08,0x12,0x00,0x20,0xee,0x8d... | Request characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0012 | 0x20,0xee,0x8d... | WR          | 0x00,0x00,0x00,0x00,0x00,0x00... | Request characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0013 | 0x01,0x29         | RD,WR       |  'R', 'e', 'q', 'u', 'e', 's'... | Characteristic User Description descriptor
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0014 | 0x28,0x03         | RD          | 0x12,0x15,0x00,0x22,0xee,0x8d... | Response characteristic Declaration
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0015 | 0x22,0xee,0x8d... | RD,NT       | 0x00,0x00,0x00,0x00,0x00,0x00... | Response characteristic value
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0016 | 0x02,0x29         | RD,WR       | 0x00,0x00                        | Client Characteristic Configuration descriptor
 *  -------+-------------------+-------------+----------------------------------+---------------
 *  0x0017 | 0x01,0x29         | RD,WR       |  'R', 'e', 's', 'p', 'o', 'n'... | Characteristic User Description descriptor
 *  ============================================================================================
 
 */

/*******************************************************************************
 * Includes   <System Includes> , "Project Includes"
 *******************************************************************************/
#include <stdio.h>
#include "gatt_db.h"

/*****************************************************************************
 * Global definition
 ******************************************************************************/
static const uint8_t gs_gatt_const_uuid_arr[] = {
/* Primary Service Declaration : 0 */
0x00, 0x28,

/* Secondary Service Declaration : 2 */
0x01, 0x28,

/* Included Service Declaration : 4 */
0x02, 0x28,

/* Characteristic Declaration : 6 */
0x03, 0x28,

/* GAP Service : 8 */
0x00, 0x18,

/* Device Name : 10 */
0x00, 0x2A,

/* Appearance : 12 */
0x01, 0x2A,

/* Peripheral Preferred Connection Parameters : 14 */
0x04, 0x2A,

/* Central Address Resolution : 16 */
0xA6, 0x2A,

/* Resolvable Private Address Only : 18 */
0xC9, 0x2A,

/* GATT Service : 20 */
0x01, 0x18,

/* Service Changed : 22 */
0x05, 0x2A,

/* Client Characteristic Configuration : 24 */
0x02, 0x29,

/* Quick Connect Service : 26 */
0x59, 0x5a, 0x08, 0xe4, 0x86, 0x2a, 0x9e, 0x8f, 0xe9, 0x11, 0xbc, 0x7c, 0x98,
		0x43, 0x42, 0x18,

		/* Request : 42 */
		0x20, 0xee, 0x8d, 0x0c, 0xe1, 0xf0, 0x4a, 0x0c, 0xb3, 0x25, 0xdc, 0x53,
		0x6a, 0x68, 0x86, 0x2d,

		/* Characteristic User Description : 58 */
		0x01, 0x29,

		/* Response : 60 */
		0x22, 0xee, 0x8d, 0x0c, 0xe1, 0xf0, 0x4a, 0x0c, 0xb3, 0x25, 0xdc, 0x53,
		0x6a, 0x68, 0x86, 0x2d,

};

static uint8_t gs_gatt_value_arr[] = {
/* Device Name */
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

		/* Appearance */
		0x00, 0x00,

		/* Peripheral Preferred Connection Parameters */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

		/* Central Address Resolution */
		0x00,

		/* Resolvable Private Address Only */
		0x00,

		/* Request */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0,

		/* Request : Characteristic User Description */
		'R', 'e', 'q', 'u', 'e', 's', 't', 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00,

		/* Response */
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0,

		/* Response : Characteristic User Description */
		'R', 'e', 's', 'p', 'o', 'n', 's', 'e', 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00,

};

static const uint8_t gs_gatt_const_value_arr[] = {
		/* Device Name */
		0x0A,       // Properties
		0x03,
		0x00, // Attr Handle
		0x00,
		0x2A, // UUID

		/* Appearance */
		0x02,       // Properties
		0x05,
		0x00, // Attr Handle
		0x01,
		0x2A, // UUID

		/* Peripheral Preferred Connection Parameters */
		0x02,       // Properties
		0x07,
		0x00, // Attr Handle
		0x04,
		0x2A, // UUID

		/* Central Address Resolution */
		0x02,       // Properties
		0x09,
		0x00, // Attr Handle
		0xA6,
		0x2A, // UUID

		/* Resolvable Private Address Only */
		0x02,       // Properties
		0x0B,
		0x00, // Attr Handle
		0xC9,
		0x2A, // UUID

		/* Service Changed */
		0x20,       // Properties
		0x0E,
		0x00, // Attr Handle
		0x05,
		0x2A, // UUID

		/* Request */
		0x08,       // Properties
		0x12,
		0x00, // Attr Handle
		0x20, 0xee, 0x8d, 0x0c, 0xe1, 0xf0, 0x4a, 0x0c, 0xb3, 0x25, 0xdc, 0x53,
		0x6a, 0x68, 0x86,
		0x2d, // UUID

		/* Response */
		0x12,       // Properties
		0x15,
		0x00, // Attr Handle
		0x22, 0xee, 0x8d, 0x0c, 0xe1, 0xf0, 0x4a, 0x0c, 0xb3, 0x25, 0xdc, 0x53,
		0x6a, 0x68, 0x86, 0x2d, // UUID

		};

static const uint8_t gs_gatt_db_const_peer_specific_val_arr[] = {
/* Service Changed : Client Characteristic Configuration */
0x00, 0x00,

/* Response : Client Characteristic Configuration */
0x00, 0x00,

};

#ifdef BSP_MCU_GROUP_RA4W1 /* RA4W1 */
static uint8_t gs_gatt_db_peer_specific_val_arr[sizeof(gs_gatt_db_const_peer_specific_val_arr)*(BLE_ABS_CFG_RF_CONNECTION_MAXIMUM+1)];
#else /* RX23W or RE01B */
static uint8_t gs_gatt_db_peer_specific_val_arr[sizeof(gs_gatt_db_const_peer_specific_val_arr)
		* (BLE_CFG_RF_CONN_MAX + 1)];
#endif
static const st_ble_gatts_db_uuid_cfg_t gs_gatt_type_table[] = {
/* 0 : Primary Service Declaration */
{
/* UUID Offset */
0,
/* First Occurrence for Type */
0x0001,
/* Last Occurrence for Type */
0x0010, },

/* 1 : GAP Service */
{
/* UUID Offset */
8,
/* First Occurrence for Type */
0x0001,
/* Last Occurrence for Type */
0x0000, },

/* 2 : Characteristic Declaration */
{
/* UUID Offset */
6,
/* First Occurrence for Type */
0x0002,
/* Last Occurrence for Type */
0x0014, },

/* 3 : Device Name */
{
/* UUID Offset */
10,
/* First Occurrence for Type */
0x0003,
/* Last Occurrence for Type */
0x0000, },

/* 4 : Appearance */
{
/* UUID Offset */
12,
/* First Occurrence for Type */
0x0005,
/* Last Occurrence for Type */
0x0000, },

/* 5 : Peripheral Preferred Connection Parameters */
{
/* UUID Offset */
14,
/* First Occurrence for Type */
0x0007,
/* Last Occurrence for Type */
0x0000, },

/* 6 : Central Address Resolution */
{
/* UUID Offset */
16,
/* First Occurrence for Type */
0x0009,
/* Last Occurrence for Type */
0x0000, },

/* 7 : Resolvable Private Address Only */
{
/* UUID Offset */
18,
/* First Occurrence for Type */
0x000B,
/* Last Occurrence for Type */
0x0000, },

/* 8 : GATT Service */
{
/* UUID Offset */
20,
/* First Occurrence for Type */
0x000C,
/* Last Occurrence for Type */
0x0000, },

/* 9 : Service Changed */
{
/* UUID Offset */
22,
/* First Occurrence for Type */
0x000E,
/* Last Occurrence for Type */
0x0000, },

/* 10 : Client Characteristic Configuration */
{
/* UUID Offset */
24,
/* First Occurrence for Type */
0x000F,
/* Last Occurrence for Type */
0x0016, },

/* 11 : Quick Connect Service */
{
/* UUID Offset */
26,
/* First Occurrence for Type */
0x0010,
/* Last Occurrence for Type */
0x0000, },

/* 12 : Request */
{
/* UUID Offset */
42,
/* First Occurrence for Type */
0x0012,
/* Last Occurrence for Type */
0x0000, },

/* 13 : Characteristic User Description */
{
/* UUID Offset */
58,
/* First Occurrence for Type */
0x0013,
/* Last Occurrence for Type */
0x0017, },

/* 14 : Response */
{
/* UUID Offset */
60,
/* First Occurrence for Type */
0x0015,
/* Last Occurrence for Type */
0x0000, },

};

static const st_ble_gatts_db_attr_cfg_t gs_gatt_db_attr_table[] = {
/* Handle : 0x0000 */
/* Blank */
{
/* Properties */
0,
/* Auxiliary Properties */
BLE_GATT_DB_NO_AUXILIARY_PROPERTY,
/* Value Size */
1,
/* Next Attribute Type Index */
0x0001,
/* UUID Offset */
0,
/* Value */
NULL, },

/* Handle : 0x0001 */
/* GAP Service : Primary Service Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
2,
/* Next Attribute Type Index */
0x000C,
/* UUID Offset */
0,
/* Value */
(uint8_t*) (gs_gatt_const_uuid_arr + 8), },

/* Handle : 0x0002 */
/* Device Name : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
5,
/* Next Attribute Type Index */
0x0004,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 0), },

/* Handle : 0x0003 */
/* Device Name */
{
/* Properties */
BLE_GATT_DB_READ | BLE_GATT_DB_WRITE,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
128,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
10,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 0), },

/* Handle : 0x0004 */
/* Appearance : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
5,
/* Next Attribute Type Index */
0x0006,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 5), },

/* Handle : 0x0005 */
/* Appearance */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
2,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
12,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 128), },

/* Handle : 0x0006 */
/* Peripheral Preferred Connection Parameters : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
5,
/* Next Attribute Type Index */
0x0008,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 10), },

/* Handle : 0x0007 */
/* Peripheral Preferred Connection Parameters */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
8,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
14,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 130), },

/* Handle : 0x0008 */
/* Central Address Resolution : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
5,
/* Next Attribute Type Index */
0x000A,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 15), },

/* Handle : 0x0009 */
/* Central Address Resolution */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
1,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
16,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 138), },

/* Handle : 0x000A */
/* Resolvable Private Address Only : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
5,
/* Next Attribute Type Index */
0x000D,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 20), },

/* Handle : 0x000B */
/* Resolvable Private Address Only */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
1,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
18,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 139), },

/* Handle : 0x000C */
/* GATT Service : Primary Service Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
2,
/* Next Attribute Type Index */
0x0010,
/* UUID Offset */
0,
/* Value */
(uint8_t*) (gs_gatt_const_uuid_arr + 20), },

/* Handle : 0x000D */
/* Service Changed : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
5,
/* Next Attribute Type Index */
0x0011,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 25), },

/* Handle : 0x000E */
/* Service Changed */
{
/* Properties */
0,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
4,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
22,
/* Value */
(NULL), },

/* Handle : 0x000F */
/* Service Changed : Client Characteristic Configuration */
{
/* Properties */
BLE_GATT_DB_READ | BLE_GATT_DB_WRITE,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY | BLE_GATT_DB_PEER_SPECIFIC_VAL_PROPERTY,
/* Value Size */
2,
/* Next Attribute Type Index */
0x0016,
/* UUID Offset */
24,
/* Value */
(uint8_t*) (gs_gatt_db_peer_specific_val_arr + 0), },

/* Handle : 0x0010 */
/* Quick Connect Service : Primary Service Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY | BLE_GATT_DB_128_BIT_UUID_FORMAT,
/* Value Size */
16,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
0,
/* Value */
(uint8_t*) (gs_gatt_const_uuid_arr + 26), },

/* Handle : 0x0011 */
/* Request : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
19,
/* Next Attribute Type Index */
0x0014,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 30), },

/* Handle : 0x0012 */
/* Request */
{
/* Properties */
BLE_GATT_DB_WRITE,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY | BLE_GATT_DB_128_BIT_UUID_FORMAT,
/* Value Size */
32,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
42,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 140), },

/* Handle : 0x0013 */
/* Request : Characteristic User Description */
{
/* Properties */
BLE_GATT_DB_READ | BLE_GATT_DB_WRITE,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
100,
/* Next Attribute Type Index */
0x0017,
/* UUID Offset */
58,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 172), },

/* Handle : 0x0014 */
/* Response : Characteristic Declaration */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
19,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
6,
/* Value */
(uint8_t*) (gs_gatt_const_value_arr + 49), },

/* Handle : 0x0015 */
/* Response */
{
/* Properties */
BLE_GATT_DB_READ,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY | BLE_GATT_DB_128_BIT_UUID_FORMAT,
/* Value Size */
101,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
60,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 272), },

/* Handle : 0x0016 */
/* Response : Client Characteristic Configuration */
{
/* Properties */
BLE_GATT_DB_READ | BLE_GATT_DB_WRITE,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY | BLE_GATT_DB_PEER_SPECIFIC_VAL_PROPERTY,
/* Value Size */
2,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
24,
/* Value */
(uint8_t*) (gs_gatt_db_peer_specific_val_arr + 2), },

/* Handle : 0x0017 */
/* Response : Characteristic User Description */
{
/* Properties */
BLE_GATT_DB_READ | BLE_GATT_DB_WRITE,
/* Auxiliary Properties */
BLE_GATT_DB_FIXED_LENGTH_PROPERTY,
/* Value Size */
100,
/* Next Attribute Type Index */
0x0000,
/* UUID Offset */
58,
/* Value */
(uint8_t*) (gs_gatt_value_arr + 373), },

};

static const st_ble_gatts_db_char_cfg_t gs_gatt_characteristic[] = {
/* 0 : Device Name */
{
/* Number of Attributes */
{ 2, },
/* Start Handle */
0x0002,
/* Service Index */
0, },

/* 1 : Appearance */
{
/* Number of Attributes */
{ 2, },
/* Start Handle */
0x0004,
/* Service Index */
0, },

/* 2 : Peripheral Preferred Connection Parameters */
{
/* Number of Attributes */
{ 2, },
/* Start Handle */
0x0006,
/* Service Index */
0, },

/* 3 : Central Address Resolution */
{
/* Number of Attributes */
{ 2, },
/* Start Handle */
0x0008,
/* Service Index */
0, },

/* 4 : Resolvable Private Address Only */
{
/* Number of Attributes */
{ 2, },
/* Start Handle */
0x000A,
/* Service Index */
0, },

/* 5 : Service Changed */
{
/* Number of Attributes */
{ 3, },
/* Start Handle */
0x000D,
/* Service Index */
1, },

/* 6 : Request */
{
/* Number of Attributes */
{ 3, },
/* Start Handle */
0x0011,
/* Service Index */
2, },

/* 7 : Response */
{
/* Number of Attributes */
{ 4, },
/* Start Handle */
0x0014,
/* Service Index */
2, },

};

static const st_ble_gatts_db_serv_cfg_t gs_gatt_service[] = {
/* GAP Service */
{
/* Num of Services */
{ 1, },
/* Description */
0,
/* Service Start Handle */
0x0001,
/* Service End Handle */
0x000B,
/* Characteristic Start Index */
0,
/* Characteristic End Index */
4, },

/* GATT Service */
{
/* Num of Services */
{ 1, },
/* Description */
0,
/* Service Start Handle */
0x000C,
/* Service End Handle */
0x000F,
/* Characteristic Start Index */
5,
/* Characteristic End Index */
5, },

/* Quick Connect Service */
{
/* Num of Services */
{ 1, },
/* Description */
0,
/* Service Start Handle */
0x0010,
/* Service End Handle */
0x0017,
/* Characteristic Start Index */
6,
/* Characteristic End Index */
7, },

};

st_ble_gatts_db_cfg_t g_gatt_db_table = { gs_gatt_const_uuid_arr,
		gs_gatt_value_arr, gs_gatt_const_value_arr,
		gs_gatt_db_peer_specific_val_arr,
		gs_gatt_db_const_peer_specific_val_arr, gs_gatt_type_table,
		gs_gatt_db_attr_table, gs_gatt_characteristic, gs_gatt_service,
		ARRAY_SIZE(gs_gatt_service), ARRAY_SIZE(gs_gatt_characteristic),
		ARRAY_SIZE(gs_gatt_type_table), ARRAY_SIZE(
				gs_gatt_db_const_peer_specific_val_arr), };
