/*
 * Goodix Moc driver for libfprint
 * Copyright (C) 2019 Shenzhen Goodix Technology Co., Ltd.
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define PACKAGE_CRC_SIZE (4)
#define PACKAGE_HEADER_SIZE (8)

#define FP_MAX_FINGERNUM (10)

#define TEMPLATE_ID_SIZE (32)

#define GX_VERSION_LEN (8)

/* Type covert */
#define MAKE_CMD_EX(cmd0, cmd1)    ((uint16_t) (((cmd0) << 8) | (cmd1)))
#define LOBYTE(value) ((uint8_t) (value))
#define HIBYTE(value) ((uint8_t) (((uint16_t) (value) >> 8) & 0xFF))


/* Error code */
#define GX_SUCCESS 0x00
#define GX_FAILED 0x80
#define GX_ERROR_FINGER_ID_NOEXIST 0x9C
#define GX_ERROR_TEMPLATE_INCOMPLETE 0xB8
#define GX_ERROR_WAIT_FINGER_UP_TIMEOUT 0xC7
#define GX_ERROR_NO_AVAILABLE_SPACE 0x8F

/* Command Type Define */
#define RESPONSE_PACKAGE_CMD 0xAA

#define MOC_CMD0_ENROLL 0xA0
#define MOC_CMD0_ENROLL_INIT 0xA1
#define MOC_CMD0_CAPTURE_DATA 0xA2
#define MOC_CMD0_CHECK4DUPLICATE 0xA3
#define MOC_CMD0_COMMITENROLLMENT 0xA4

#define MOC_CMD0_IDENTIFY 0xA5
#define MOC_CMD0_GETFINGERLIST 0xA6
#define MOC_CMD0_DELETETEMPLATE 0xA7

#define MOC_CMD1_DEFAULT 0x00
#define MOC_CMD1_UNTIL_DOWN 0x00
#define MOC_CMD1_WHEN_DOWN 0x01

#define MOC_CMD1_DELETE_TEMPLATE 0x04
#define MOC_CMD1_DELETE_ALL 0x01

#define MOC_CMD0_GET_VERSION 0xD0

#define MOC_CMD0_UPDATE_CONFIG 0xC0
#define MOC_CMD1_NWRITE_CFG_TO_FLASH 0x00
#define MOC_CMD1_WRITE_CFG_TO_FLASH 0x01

#define MOC_CMD0_FINGER_MODE 0xB0
#define MOC_CMD1_GET_FINGER_MODE 0x00
#define MOC_CMD1_SET_FINGER_DOWN 0x01
#define MOC_CMD1_SET_FINGER_UP 0x02
/* */

typedef struct _gxfp_version_info
{
  uint8_t format[2];
  uint8_t fwtype[GX_VERSION_LEN];
  uint8_t fwversion[GX_VERSION_LEN];
  uint8_t customer[GX_VERSION_LEN];
  uint8_t mcu[GX_VERSION_LEN];
  uint8_t sensor[GX_VERSION_LEN];
  uint8_t algversion[GX_VERSION_LEN];
  uint8_t interface[GX_VERSION_LEN];
  uint8_t protocol[GX_VERSION_LEN];
  uint8_t flashVersion[GX_VERSION_LEN];
  uint8_t reserved[62];
} gxfp_version_info_t, *pgxfp_version_info_t;


typedef struct _gxfp_parse_msg
{
  uint8_t ack_cmd;
  bool    has_no_config;
} gxfp_parse_msg_t, *pgxfp_parse_msg_t;


typedef struct _gxfp_enroll_init
{
  uint8_t tid[TEMPLATE_ID_SIZE];
} gxfp_enroll_init_t, *pgxfp_enroll_init_t;

#pragma pack(push, 1)
typedef struct _template_format
{
  uint8_t type;
  uint8_t finger_index;
  uint8_t accountid[32];
  uint8_t tid[32];
  struct
  {
    uint32_t size;
    uint8_t  data[56];
  } payload;
  uint8_t reserve[2];
} template_format_t, *ptemplate_format_t;

#pragma pack(pop)


typedef struct _gxfp_verify
{
  bool     match;
  uint32_t rejectdetail;
  template_format_t  template;
} gxfp_verify_t, *pgxfp_verify_t;


typedef struct _gxfp_capturedata
{
  uint8_t img_quality;
  uint8_t img_coverage;
} gxfp_capturedata_t, *pgxfp_capturedata_t;

typedef struct _gxfp_check_duplicate
{
  bool duplicate;
  template_format_t  template;
} gxfp_check_duplicate_t, *pgxfp_check_duplicate_t;


typedef struct _gxfp_enroll_update
{
  bool    rollback;
  uint8_t img_overlay;
  uint8_t img_preoverlay;
} gxfp_enroll_update_t, *Pgxfp_enroll_update_t;

typedef struct _gxfp_enum_fingerlist
{
  uint8_t           finger_num;
  template_format_t finger_list[FP_MAX_FINGERNUM];
} gxfp_enum_fingerlist_t, *pgxfp_enum_fingerlist_t;

typedef struct _gxfp_enroll_commit
{
  uint8_t result;
} gxfp_enroll_commit_t, *pgxfp_enroll_commit_t;

typedef struct _fp_finger_status
{
  uint8_t status;
} fp_finger_status_t, *pfp_finger_status_t;


typedef struct _fp_cmd_response
{
  uint8_t result;
  union
  {
    gxfp_parse_msg_t       parse_msg;
    gxfp_verify_t          verify;
    gxfp_enroll_init_t     enroll_init;
    gxfp_capturedata_t     capture_data_resp;
    gxfp_check_duplicate_t check_duplicate_resp;
    gxfp_enroll_commit_t   enroll_commit;
    gxfp_enroll_update_t   enroll_update;
    gxfp_enum_fingerlist_t finger_list_resp;
    gxfp_version_info_t    version_info;
    fp_finger_status_t     finger_status;
  };
} gxfp_cmd_response_t, *pgxfp_cmd_response_t;


typedef struct _pack_header
{
  uint8_t  cmd0;
  uint8_t  cmd1;
  uint8_t  packagenum;
  uint8_t  reserved;
  uint16_t len;
  uint8_t  crc8;
  uint8_t  rev_crc8;
} pack_header, *ppack_header;


typedef struct _gxfp_sensor_cfg
{
  uint8_t config[26];
  uint8_t reserved[98];
  uint8_t crc_value[4];
} gxfp_sensor_cfg_t, *pgxfp_sensor_cfg_t;
/* */

int gx_proto_build_package (uint8_t       *ppackage,
                            uint32_t      *package_len,
                            uint16_t       cmd,
                            const uint8_t *payload,
                            uint32_t       payload_size);

int gx_proto_parse_header (uint8_t     *buffer,
                           uint32_t     buffer_len,
                           pack_header *pheader);

int gx_proto_parse_body (uint16_t             cmd,
                         uint8_t             *buffer,
                         uint32_t             buffer_len,
                         pgxfp_cmd_response_t presponse);

int gx_proto_init_sensor_config (pgxfp_sensor_cfg_t pconfig);

uint8_t gx_proto_crc8_calc (uint8_t *lubp_date,
                            uint32_t lui_len);

uint8_t gx_proto_crc32_calc (uint8_t *pchMsg,
                             uint32_t wDataLen,
                             uint8_t *pchMsgDst);
