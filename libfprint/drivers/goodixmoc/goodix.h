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

#include "fpi-device.h"
#include "fpi-ssm.h"

G_DECLARE_FINAL_TYPE (FpiDeviceGoodixMoc, fpi_device_goodixmoc, FPI, DEVICE_GOODIXMOC, FpDevice)

typedef enum {
  FP_CMD_SEND = 0,
  FP_CMD_GET_ACK,
  FP_CMD_GET_DATA,
  FP_CMD_NUM_STATES,
} FpCmdState;


typedef enum {
  FP_INIT_VERSION = 0,
  FP_INIT_CONFIG,
  FP_INIT_NUM_STATES,
} FpInitState;


typedef enum {
  FP_ENROLL_ENUM = 0,
  FP_ENROLL_IDENTIFY,
  FP_ENROLL_CREATE,
  FP_ENROLL_CAPTURE,
  FP_ENROLL_UPDATE,
  FP_ENROLL_WAIT_FINGER_UP,
  FP_ENROLL_CHECK_DUPLICATE,
  FP_ENROLL_COMMIT,
  FP_ENROLL_NUM_STATES,
} FpEnrollState;

typedef enum {
  FP_VERIFY_CAPTURE = 0,
  FP_VERIFY_IDENTIFY,
  FP_VERIFY_NUM_STATES,
} FpVerifyState;
