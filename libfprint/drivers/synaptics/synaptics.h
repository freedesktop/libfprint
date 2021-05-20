/*
 * Copyright (C) 2019 Synaptics Inc
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

#include "fpi-device.h"
#include "fpi-ssm.h"

#define SYNAPTICS_VENDOR_ID 0x06cb

G_DECLARE_FINAL_TYPE (FpiDeviceSynaptics, fpi_device_synaptics, FPI, DEVICE_SYNAPTICS, FpDevice)


#define MAX_TRANSFER_LEN 263 + 1 /* SPI Header */ + 2                                /* VCSFW header */

#define USB_EP_REQUEST 0x01
#define USB_EP_REPLY 0x81
#define USB_EP_FINGERPRINT 0x82
#define USB_EP_INTERRUPT 0x83

#define USB_ASYNC_MESSAGE_PENDING 0x4
#define USB_INTERRUPT_DATA_SIZE 7

#define SENSOR_CMD_GET_VERSION 1
#define SENSOR_CMD_ACE_COMMAND 167
#define SENSOR_CMD_ASYNCMSG_READ 168

#define SENSOR_FW_CMD_HEADER_LEN 1
#define SENSOR_FW_REPLY_HEADER_LEN 2


/* Number of enroll stages */
#define ENROLL_SAMPLES 8


#define SYNAPTICS_DRIVER_FULLNAME "Synaptics Sensors"
#include "bmkt.h"
#include "bmkt_response.h"


typedef struct bmkt_sensor_version
{
  uint32_t build_time;
  uint32_t build_num;
  uint8_t  version_major;
  uint8_t  version_minor;
  uint8_t  target;
  uint8_t  product;
  uint8_t  silicon_rev;
  uint8_t  formal_release;
  uint8_t  platform;
  uint8_t  patch;
  uint8_t  serial_number[6];
  uint16_t security;
  uint8_t  iface;
  uint8_t  device_type;
} bmkt_sensor_version_t;


struct syna_enroll_resp_data
{
  int progress;
};
typedef enum syna_state {
  SYNA_STATE_UNINIT                                                       = 0,
  SYNA_STATE_IDLE,
  SYNA_STATE_ENROLL,
  SYNA_STATE_IDENTIFY,
  SYNA_STATE_IDENTIFY_DELAY_RESULT,
  SYNA_STATE_VERIFY,
  SYNA_STATE_VERIFY_DELAY_RESULT,
  SYNA_STATE_DELETE,
} syna_state_t;

typedef enum {
  SYNAPTICS_CMD_SEND_PENDING = 0,
  SYNAPTICS_CMD_GET_RESP,
  SYNAPTICS_CMD_WAIT_INTERRUPT,
  SYNAPTICS_CMD_SEND_ASYNC,
  SYNAPTICS_CMD_RESTART,
  SYNAPTICS_CMD_SUSPENDED,
  SYNAPTICS_CMD_RESUME,
  SYNAPTICS_CMD_NUM_STATES,
} SynapticsCmdState;


typedef void (*SynCmdMsgCallback) (FpiDeviceSynaptics *self,
                                   bmkt_response_t    *resp,
                                   GError             *error);

struct _FpiDeviceSynaptics
{
  FpDevice              parent;

  guint8                cmd_seq_num;
  guint8                last_seq_num;
  FpiSsm               *cmd_ssm;
  FpiUsbTransfer       *cmd_pending_transfer;
  gboolean              cmd_complete_on_removal;
  gboolean              cmd_suspended;
  guint8                id_idx;

  bmkt_sensor_version_t mis_version;

  gboolean              action_starting;
  GCancellable         *interrupt_cancellable;

  gint                  enroll_stage;
  gboolean              finger_on_sensor;
  GPtrArray            *list_result;


  struct syna_enroll_resp_data enroll_resp_data;
  syna_state_t                 state;
};
