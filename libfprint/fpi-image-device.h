/*
 * FpImageDevice - An image based fingerprint reader device
 * Copyright (C) 2019 Benjamin Berg <bberg@redhat.com>
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
#include "fp-image-device.h"

/**
 * FpiImageDeviceState:
 * @FPI_IMAGE_DEVICE_STATE_INACTIVE: inactive
 * @FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON: waiting for the finger to be pressed or swiped
 * @FPI_IMAGE_DEVICE_STATE_CAPTURE: capturing an image
 * @FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF: waiting for the finger to be removed
 *
 * The state of an imaging device while doing a capture. The state is
 * passed through to the driver using the ::activate() or ::change_state() vfuncs.
 *
 * The driver needs to call fpi_image_device_report_finger_status() to move
 * between the different states. Note that the capture state might be entered
 * unconditionally if the device supports raw capturing.
 */
typedef enum {
  FPI_IMAGE_DEVICE_STATE_INACTIVE,
  FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON,
  FPI_IMAGE_DEVICE_STATE_CAPTURE,
  FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF,
} FpiImageDeviceState;

/**
 * FpImageDeviceClass:
 * @bz3_threshold: Threshold to consider bozorth3 score a match, default: 40
 * @img_width: Width of the image, only provide if constant
 * @img_height: Height of the image, only provide if constant
 * @img_open: Open the device and do basic initialization
 *   (use this instead of the #FpDeviceClass open vfunc)
 * @img_close: Close the device
 *   (use this instead of the #FpDeviceClass close vfunc)
 * @activate: Start image capture and finger detection
 * @deactivate: Stop image capture and finger detection
 * @change_state: Notification about the current device state (i.e. waiting for
 *   finger or image capture). Implementing this is optional, it can e.g. be
 *   used to flash an LED when waiting for a finger.
 *
 * These are the main entry points for image based drivers. For all but the
 * change_state vfunc, implementations *must* eventually call the corresponding
 * function to finish the operation. It is also acceptable to call the generic
 *
 *
 * These are the main entry points for drivers to implement. Drivers may not
 * implement all of these entry points if they do not support the operation
 * (or a default implementation is sufficient).
 *
 * Drivers *must* eventually call the corresponding function to finish the
 * operation. It is also acceptable to call the generic
 * fpi_device_action_error() function but doing so is not recommended in most
 * usecases.
 *
 * Image drivers must expect a @deactivate call to happen at any point during
 * image capture.
 *
 * This API is solely intended for drivers. It is purely internal and neither
 * API nor ABI stable.
 */
struct _FpImageDeviceClass
{
  FpDeviceClass parent_class;

  gint          bz3_threshold;
  gint          img_width;
  gint          img_height;

  void          (*img_open)     (FpImageDevice *dev);
  void          (*img_close)    (FpImageDevice *dev);
  void          (*activate)     (FpImageDevice *dev);
  void          (*change_state) (FpImageDevice      *dev,
                                 FpiImageDeviceState state);
  void          (*deactivate)   (FpImageDevice *dev);
};

void fpi_image_device_set_bz3_threshold (FpImageDevice *self,
                                         gint           bz3_threshold);

void fpi_image_device_session_error (FpImageDevice *self,
                                     GError        *error);

void fpi_image_device_open_complete (FpImageDevice *self,
                                     GError        *error);
void fpi_image_device_close_complete (FpImageDevice *self,
                                      GError        *error);
void fpi_image_device_activate_complete (FpImageDevice *self,
                                         GError        *error);
void fpi_image_device_deactivate_complete (FpImageDevice *self,
                                           GError        *error);

void fpi_image_device_report_finger_status (FpImageDevice *self,
                                            gboolean       present);
void fpi_image_device_image_captured (FpImageDevice *self,
                                      FpImage       *image);
void fpi_image_device_retry_scan (FpImageDevice *self,
                                  FpDeviceRetry  retry);
