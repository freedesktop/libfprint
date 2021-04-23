/*
 * FpDevice - A fingerprint reader device
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

#include <gusb.h>
#include "fp-device.h"
#include "fp-image.h"
#include "fpi-print.h"

#include <config.h>

/**
 * FpiDeviceUdevSubtype:
 * @FPI_DEVICE_UDEV_SUBTYPE_SPIDEV: The device requires an spidev node
 * @FPI_DEVICE_UDEV_SUBTYPE_HIDRAW: The device requires a hidraw node
 */
typedef enum {
  FPI_DEVICE_UDEV_SUBTYPE_SPIDEV = 1 << 0,
  FPI_DEVICE_UDEV_SUBTYPE_HIDRAW = 1 << 1,
} FpiDeviceUdevSubtypeFlags;

/**
 * FpIdEntry:
 *
 * An entry in the table of supported hardware. For USB devices, the product ID
 * and vendor ID should be provided. The optional @driver_data field defaults
 * to 0 and can be used as a simple flag for device quirks.
 */
typedef struct _FpIdEntry FpIdEntry;

struct _FpIdEntry
{
  union
  {
    struct
    {
      guint pid;
      guint vid;
    };
    const gchar *virtual_envvar;
    struct
    {
      FpiDeviceUdevSubtypeFlags udev_types;
      const gchar              *spi_acpi_id;
      struct
      {
        guint pid;
        guint vid;
      } hid_id;
    };
  };
  guint64 driver_data;
};

/**
 * FpDeviceClass:
 * @id: ID string for the driver. Should be a valid C identifier and should
 *   match the drivers file name.
 * @full_name: Human readable description of the driver
 * @type: The type of driver
 * @id_table: The table of IDs to bind the driver to
 * @features: The features the device supports, it can be initialized using
 *   fpi_device_class_auto_initialize_features() on @class_init.
 * @nr_enroll_stages: The number of enroll stages supported devices need; use
 *   fpi_device_set_nr_enroll_stages() from @probe if this is dynamic.
 * @scan_type: The scan type of supported devices; use
 *   fpi_device_set_scan_type() from @probe if this is dynamic.
 * @temp_hot_seconds: Assumed time in minutes for the device to become too hot
 *   after being mostly cold.
 * @temp_cold_seconds: Assumed time in minutes for the device to be mostly cold
 *   after having been too hot to operate.
 * @usb_discover: Class method to check whether a USB device is supported by
 *  the driver. Should return 0 if the device is unsupported and a positive
 *  score otherwise. The default score is 50 and the driver with the highest
 *  score will be loaded.
 * @probe: Called immediately for all devices. Most drivers will not need to
 *   implement this. Drivers should setup the device identifier from the probe
 *   callback which will be used to verify the compatibility of stored
 *   #FpPrint's. It is permissible to temporarily open the USB device if this
 *   is required for the operation. If an error is returned, then the device
 *   will be destroyed again immediately and never reported to the API user.
 * @open: Open the device for further operations. Any of the normal actions are
 *   guaranteed to only happen when the device is open (this includes delete).
 * @close: Close the device again
 * @enroll: Start an enroll operation
 * @verify: Start a verify operation
 * @identify: Start an identify operation
 * @capture: Start a capture operation
 * @list: List prints stored on the device
 * @delete: Delete a print from the device
 * @clear_storage: Delete all prints from the device
 * @cancel: Called on cancellation, this is a convenience to not need to handle
 *   the #GCancellable directly by using fpi_device_get_cancellable().
 *
 * NOTE: If your driver is image based, then you should subclass #FpImageDevice
 * instead. #FpImageDevice based drivers use a different way of interacting
 * with libfprint.
 *
 * These are the main entry points for drivers to implement. Drivers may not
 * implement all of these entry points if they do not support the operation
 * (or a default implementation is sufficient).
 *
 * Drivers must eventually call the corresponding function to finish the
 * operation. It is also acceptable to call the generic
 * fpi_device_action_error() function but doing so is not recommended in most
 * usecases.
 *
 * Drivers must also handle cancellation properly for any long running
 * operation (i.e. any operation that requires capturing). It is entirely fine
 * to ignore cancellation requests for short operations (e.g. open/close).
 *
 * This API is solely intended for drivers. It is purely internal and neither
 * API nor ABI stable.
 */
struct _FpDeviceClass
{
  /*< private >*/
  GObjectClass parent_class;

  /*< public >*/
  /* Static information about the driver. */
  const gchar     *id;
  const gchar     *full_name;
  FpDeviceType     type;
  const FpIdEntry *id_table;
  FpDeviceFeature  features;

  /* Defaults for device properties */
  gint       nr_enroll_stages;
  FpScanType scan_type;

  /* Simple device temperature model constants */
  gint32 temp_hot_seconds;
  gint32 temp_cold_seconds;

  /* Callbacks */
  gint (*usb_discover) (GUsbDevice *usb_device);
  void (*probe)    (FpDevice *device);
  void (*open)     (FpDevice *device);
  void (*close)    (FpDevice *device);
  void (*enroll)   (FpDevice *device);
  void (*verify)   (FpDevice *device);
  void (*identify) (FpDevice *device);
  void (*capture)  (FpDevice *device);
  void (*list)     (FpDevice *device);
  void (*delete)   (FpDevice * device);
  void (*clear_storage)  (FpDevice * device);

  void (*cancel)   (FpDevice *device);
};

void fpi_device_class_auto_initialize_features (FpDeviceClass *device_class);

/**
 * FpTimeoutFunc:
 * @device: The #FpDevice passed to fpi_device_add_timeout()
 * @user_data: the data passed to fpi_device_add_timeout()
 *
 * The prototype of the callback function for fpi_device_add_timeout().
 */
typedef void (*FpTimeoutFunc) (FpDevice *device,
                               gpointer  user_data);

/**
 * FpiDeviceAction:
 * @FPI_DEVICE_ACTION_NONE: No action is active.
 * @FPI_DEVICE_ACTION_PROBE: Probe device for support and information.
 * @FPI_DEVICE_ACTION_OPEN: Device is currently being opened.
 * @FPI_DEVICE_ACTION_CLOSE: Device is currently being closed.
 * @FPI_DEVICE_ACTION_ENROLL: Device is currently enrolling.
 * @FPI_DEVICE_ACTION_VERIFY: Device is currently verifying.
 * @FPI_DEVICE_ACTION_IDENTIFY: Device is currently identifying.
 * @FPI_DEVICE_ACTION_CAPTURE: Device is currently capturing an image.
 * @FPI_DEVICE_ACTION_LIST: Device stored prints are being queried.
 * @FPI_DEVICE_ACTION_DELETE: Device stored print is being deleted.
 * @FPI_DEVICE_ACTION_CLEAR_STORAGE: Device stored prints are being deleted.
 *
 * Current active action of the device. A driver can retrieve the action.
 */
typedef enum {
  FPI_DEVICE_ACTION_NONE = 0,
  FPI_DEVICE_ACTION_PROBE,
  FPI_DEVICE_ACTION_OPEN,
  FPI_DEVICE_ACTION_CLOSE,
  FPI_DEVICE_ACTION_ENROLL,
  FPI_DEVICE_ACTION_VERIFY,
  FPI_DEVICE_ACTION_IDENTIFY,
  FPI_DEVICE_ACTION_CAPTURE,
  FPI_DEVICE_ACTION_LIST,
  FPI_DEVICE_ACTION_DELETE,
  FPI_DEVICE_ACTION_CLEAR_STORAGE,
} FpiDeviceAction;

GUsbDevice  *fpi_device_get_usb_device (FpDevice *device);
const gchar *fpi_device_get_virtual_env (FpDevice *device);
gpointer     fpi_device_get_udev_data (FpDevice                 *device,
                                       FpiDeviceUdevSubtypeFlags subtype);
//const gchar *fpi_device_get_spi_dev (FpDevice *device);


FpiDeviceAction fpi_device_get_current_action (FpDevice *device);
gboolean fpi_device_action_is_cancelled (FpDevice *device);

GError * fpi_device_retry_new (FpDeviceRetry error);
GError * fpi_device_error_new (FpDeviceError error);

GError * fpi_device_retry_new_msg (FpDeviceRetry error,
                                   const gchar  *msg,
                                   ...) G_GNUC_PRINTF (2, 3);
GError * fpi_device_error_new_msg (FpDeviceError error,
                                   const gchar  *msg,
                                   ...) G_GNUC_PRINTF (2, 3);

guint64 fpi_device_get_driver_data (FpDevice *device);

void fpi_device_get_enroll_data (FpDevice *device,
                                 FpPrint **print);

void fpi_device_get_capture_data (FpDevice *device,
                                  gboolean *wait_for_finger);
void fpi_device_get_verify_data (FpDevice *device,
                                 FpPrint **print);
void fpi_device_get_identify_data (FpDevice   *device,
                                   GPtrArray **prints);
void fpi_device_get_delete_data (FpDevice *device,
                                 FpPrint **print);
GCancellable *fpi_device_get_cancellable (FpDevice *device);

void fpi_device_remove (FpDevice *device);

GSource * fpi_device_add_timeout (FpDevice      *device,
                                  gint           interval,
                                  FpTimeoutFunc  func,
                                  gpointer       user_data,
                                  GDestroyNotify destroy_notify);

void fpi_device_set_nr_enroll_stages (FpDevice *device,
                                      gint      enroll_stages);

void fpi_device_set_scan_type (FpDevice  *device,
                               FpScanType scan_type);

void fpi_device_update_features (FpDevice       *device,
                                 FpDeviceFeature update,
                                 FpDeviceFeature value);

void fpi_device_action_error (FpDevice *device,
                              GError   *error);

void fpi_device_critical_enter (FpDevice *device);
void fpi_device_critical_leave (FpDevice *device);

void fpi_device_probe_complete (FpDevice    *device,
                                const gchar *device_id,
                                const gchar *device_name,
                                GError      *error);
void fpi_device_open_complete (FpDevice *device,
                               GError   *error);
void fpi_device_close_complete (FpDevice *device,
                                GError   *error);
void fpi_device_enroll_complete (FpDevice *device,
                                 FpPrint  *print,
                                 GError   *error);
void fpi_device_verify_complete (FpDevice *device,
                                 GError   *error);
void fpi_device_identify_complete (FpDevice *device,
                                   GError   *error);
void fpi_device_capture_complete (FpDevice *device,
                                  FpImage  *image,
                                  GError   *error);
void fpi_device_delete_complete (FpDevice *device,
                                 GError   *error);
void fpi_device_list_complete (FpDevice  *device,
                               GPtrArray *prints,
                               GError    *error);
void fpi_device_clear_storage_complete (FpDevice *device,
                                        GError   *error);

void fpi_device_enroll_progress (FpDevice *device,
                                 gint      completed_stages,
                                 FpPrint  *print,
                                 GError   *error);
void fpi_device_verify_report (FpDevice      *device,
                               FpiMatchResult result,
                               FpPrint       *print,
                               GError        *error);
void fpi_device_identify_report (FpDevice *device,
                                 FpPrint  *match,
                                 FpPrint  *print,
                                 GError   *error);

gboolean fpi_device_report_finger_status (FpDevice           *device,
                                          FpFingerStatusFlags finger_status);
gboolean fpi_device_report_finger_status_changes (FpDevice           *device,
                                                  FpFingerStatusFlags added_status,
                                                  FpFingerStatusFlags removed_status);

G_END_DECLS
