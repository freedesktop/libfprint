/*
 * Virtual driver for device debugging
 *
 * Copyright (C) 2019 Marco Trevisan <marco.trevisan@canonical.com>
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

#define FPI_TYPE_DEVICE_FAKE (fpi_device_fake_get_type ())
G_DECLARE_FINAL_TYPE (FpiDeviceFake, fpi_device_fake, FPI, DEVICE_FAKE, FpDevice)

struct _FpiDeviceFake
{
  FpDevice        parent;

  gpointer        last_called_function;
  gboolean        return_action_error;

  GCancellable   *ext_cancellable;

  GError         *ret_error;
  FpPrint        *ret_print;
  FpPrint        *ret_match;
  FpiMatchResult  ret_result;
  FpImage        *ret_image;
  GPtrArray      *ret_list;

  GError         *ret_suspend;
  GError         *ret_resume;

  gpointer        action_data;
  gpointer        user_data;

  FpDeviceFeature probe_features_update;
  FpDeviceFeature probe_features_value;
};
