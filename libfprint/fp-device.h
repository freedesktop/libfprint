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

#include "fp-image.h"
#include <glib-object.h>
#include <gio/gio.h>

G_BEGIN_DECLS

#define FP_TYPE_DEVICE (fp_device_get_type ())
#define FP_DEVICE_RETRY (fp_device_retry_quark ())
#define FP_DEVICE_ERROR (fp_device_error_quark ())
G_DECLARE_DERIVABLE_TYPE (FpDevice, fp_device, FP, DEVICE, GObject)

#include "fp-print.h"

/* NOTE: We keep the class struct private! */

/**
 * FpDeviceType:
 * @FP_DEVICE_TYPE_VIRTUAL: The device is a virtual device
 * @FP_DEVICE_TYPE_USB: The device is a USB device
 */
typedef enum {
  FP_DEVICE_TYPE_VIRTUAL,
  FP_DEVICE_TYPE_USB,
} FpDeviceType;

/**
 * FpScanType:
 * @FP_SCAN_TYPE_SWIPE: Sensor requires swiping the finger.
 * @FP_SCAN_TYPE_PRESS: Sensor requires placing/pressing down the finger.
 */
typedef enum {
  FP_SCAN_TYPE_SWIPE,
  FP_SCAN_TYPE_PRESS,
} FpScanType;

/**
 * FpDeviceRetry:
 * @FP_DEVICE_RETRY_GENERAL: The scan did not succeed due to poor scan quality
 *   or other general user scanning problem.
 * @FP_DEVICE_RETRY_TOO_SHORT: The scan did not succeed because the finger
 *   swipe was too short.
 * @FP_DEVICE_RETRY_CENTER_FINGER: The scan did not succeed because the finger
 *   was not centered on the scanner.
 * @FP_DEVICE_RETRY_REMOVE_FINGER: The scan did not succeed due to quality or
 *   pressure problems; the user should remove their finger from the scanner
 *   before retrying.
 *
 * Error codes representing scan failures resulting in the user needing to
 * retry.
 */
typedef enum {
  FP_DEVICE_RETRY_GENERAL,
  FP_DEVICE_RETRY_TOO_SHORT,
  FP_DEVICE_RETRY_CENTER_FINGER,
  FP_DEVICE_RETRY_REMOVE_FINGER,
} FpDeviceRetry;

/**
 * FpDeviceError:
 * @FP_DEVICE_ERROR_GENERAL: A general error occurred.
 * @FP_DEVICE_ERROR_NOT_SUPPORTED: The device does not support the requested
 *  operation.
 * @FP_DEVICE_ERROR_NOT_OPEN: The device needs to be opened to start this
 *   operation.
 * @FP_DEVICE_ERROR_ALREADY_OPEN: The device has already been opened.
 * @FP_DEVICE_ERROR_BUSY: The device is busy with another request.
 * @FP_DEVICE_ERROR_PROTO: Protocol error
 * @FP_DEVICE_ERROR_DATA_INVALID: The passed data is invalid
 * @FP_DEVICE_ERROR_DATA_NOT_FOUND: Requested print was not found on device
 * @FP_DEVICE_ERROR_DATA_FULL: No space on device available for operation
 * @FP_DEVICE_ERROR_DATA_DUPLICATE: Enrolling template duplicates storaged templates
 *
 * Error codes for device operations. More specific errors from other domains
 * such as #G_IO_ERROR or #G_USB_DEVICE_ERROR may also be reported.
 */
typedef enum {
  FP_DEVICE_ERROR_GENERAL,
  FP_DEVICE_ERROR_NOT_SUPPORTED,
  FP_DEVICE_ERROR_NOT_OPEN,
  FP_DEVICE_ERROR_ALREADY_OPEN,
  FP_DEVICE_ERROR_BUSY,
  FP_DEVICE_ERROR_PROTO,
  FP_DEVICE_ERROR_DATA_INVALID,
  FP_DEVICE_ERROR_DATA_NOT_FOUND,
  FP_DEVICE_ERROR_DATA_FULL,
  FP_DEVICE_ERROR_DATA_DUPLICATE,
} FpDeviceError;

GQuark fp_device_retry_quark (void);
GQuark fp_device_error_quark (void);

/**
 * FpEnrollProgress:
 * @device: a #FpDevice
 * @completed_stages: Number of completed stages
 * @print: (nullable) (transfer none): The last scanned print
 * @user_data: (nullable) (transfer none): User provided data
 * @error: (nullable) (transfer none): #GError or %NULL
 *
 * The passed error is guaranteed to be of type %FP_DEVICE_RETRY if set.
 */
typedef void (*FpEnrollProgress) (FpDevice *device,
                                  gint      completed_stages,
                                  FpPrint  *print,
                                  gpointer  user_data,
                                  GError   *error);

/**
 * FpMatchCb:
 * @device: a #FpDevice
 * @match: (nullable) (transfer none): The matching print if any matched @print
 * @print: (nullable) (transfer none): The newly scanned print
 * @user_data: (nullable) (transfer none): User provided data
 * @error: (nullable) (transfer none): #GError or %NULL
 *
 * Report the result of a match (identify or verify) operation.
 *
 * If @match is non-%NULL, then it is set to the matching #FpPrint as passed
 * to the match operation. In this case @error will always be %NULL.
 *
 * If @error is not %NULL then its domain is guaranteed to be
 * %FP_DEVICE_RETRY. All other error conditions will not be reported using
 * this callback. If such an error occurs before a match/no-match decision
 * can be made, then this callback will not be called. Should an error
 * happen afterwards, then you will get a match report through this callback
 * and an error when the operation finishes.
 *
 * If @match and @error are %NULL, then a finger was presented but it did not
 * match any known print.
 *
 * @print represents the newly scanned print. The driver may or may not
 * provide this information. Image based devices will provide it and it
 * allows access to the raw data.
 *
 * This callback exists because it makes sense for drivers to wait e.g. on
 * finger removal before completing the match operation. However, the
 * success/failure can often be reported at an earlier time, and there is
 * no need to make the user wait.
 */
typedef void (*FpMatchCb) (FpDevice *device,
                           FpPrint  *match,
                           FpPrint  *print,
                           gpointer  user_data,
                           GError   *error);

const gchar *fp_device_get_driver (FpDevice *device);
const gchar *fp_device_get_device_id (FpDevice *device);
const gchar *fp_device_get_name (FpDevice *device);
gboolean     fp_device_is_open (FpDevice *device);
FpScanType   fp_device_get_scan_type (FpDevice *device);
gint         fp_device_get_nr_enroll_stages (FpDevice *device);

gboolean     fp_device_supports_identify (FpDevice *device);
gboolean     fp_device_supports_capture (FpDevice *device);
gboolean     fp_device_has_storage (FpDevice *device);

/* Opening the device */
void fp_device_open (FpDevice           *device,
                     GCancellable       *cancellable,
                     GAsyncReadyCallback callback,
                     gpointer            user_data);

void fp_device_close (FpDevice           *device,
                      GCancellable       *cancellable,
                      GAsyncReadyCallback callback,
                      gpointer            user_data);

void fp_device_enroll (FpDevice           *device,
                       FpPrint            *template_print,
                       GCancellable       *cancellable,
                       FpEnrollProgress    progress_cb,
                       gpointer            progress_data,
                       GDestroyNotify      progress_destroy,
                       GAsyncReadyCallback callback,
                       gpointer            user_data);

void fp_device_verify (FpDevice           *device,
                       FpPrint            *enrolled_print,
                       GCancellable       *cancellable,
                       FpMatchCb           match_cb,
                       gpointer            match_data,
                       GDestroyNotify      match_destroy,
                       GAsyncReadyCallback callback,
                       gpointer            user_data);

void fp_device_identify (FpDevice           *device,
                         GPtrArray          *prints,
                         GCancellable       *cancellable,
                         FpMatchCb           match_cb,
                         gpointer            match_data,
                         GDestroyNotify      match_destroy,
                         GAsyncReadyCallback callback,
                         gpointer            user_data);

void fp_device_capture (FpDevice           *device,
                        gboolean            wait_for_finger,
                        GCancellable       *cancellable,
                        GAsyncReadyCallback callback,
                        gpointer            user_data);

void fp_device_delete_print (FpDevice           *device,
                             FpPrint            *enrolled_print,
                             GCancellable       *cancellable,
                             GAsyncReadyCallback callback,
                             gpointer            user_data);

void fp_device_list_prints (FpDevice           *device,
                            GCancellable       *cancellable,
                            GAsyncReadyCallback callback,
                            gpointer            user_data);

gboolean fp_device_open_finish (FpDevice     *device,
                                GAsyncResult *result,
                                GError      **error);
gboolean fp_device_close_finish (FpDevice     *device,
                                 GAsyncResult *result,
                                 GError      **error);
FpPrint *fp_device_enroll_finish (FpDevice     *device,
                                  GAsyncResult *result,
                                  GError      **error);
gboolean fp_device_verify_finish (FpDevice     *device,
                                  GAsyncResult *result,
                                  gboolean     *match,
                                  FpPrint     **print,
                                  GError      **error);
gboolean fp_device_identify_finish (FpDevice     *device,
                                    GAsyncResult *result,
                                    FpPrint     **match,
                                    FpPrint     **print,
                                    GError      **error);
FpImage * fp_device_capture_finish (FpDevice     *device,
                                    GAsyncResult *result,
                                    GError      **error);
gboolean fp_device_delete_print_finish (FpDevice     *device,
                                        GAsyncResult *result,
                                        GError      **error);
GPtrArray * fp_device_list_prints_finish (FpDevice     *device,
                                          GAsyncResult *result,
                                          GError      **error);


gboolean fp_device_open_sync (FpDevice     *device,
                              GCancellable *cancellable,
                              GError      **error);
gboolean fp_device_close_sync (FpDevice     *device,
                               GCancellable *cancellable,
                               GError      **error);
FpPrint * fp_device_enroll_sync (FpDevice        *device,
                                 FpPrint         *template_print,
                                 GCancellable    *cancellable,
                                 FpEnrollProgress progress_cb,
                                 gpointer         progress_data,
                                 GError         **error);
gboolean fp_device_verify_sync (FpDevice     *device,
                                FpPrint      *enrolled_print,
                                GCancellable *cancellable,
                                FpMatchCb     match_cb,
                                gpointer      match_data,
                                gboolean     *match,
                                FpPrint     **print,
                                GError      **error);
gboolean fp_device_identify_sync (FpDevice     *device,
                                  GPtrArray    *prints,
                                  GCancellable *cancellable,
                                  FpMatchCb     match_cb,
                                  gpointer      match_data,
                                  FpPrint     **match,
                                  FpPrint     **print,
                                  GError      **error);
FpImage * fp_device_capture_sync (FpDevice     *device,
                                  gboolean      wait_for_finger,
                                  GCancellable *cancellable,
                                  GError      **error);
gboolean fp_device_delete_print_sync (FpDevice     *device,
                                      FpPrint      *enrolled_print,
                                      GCancellable *cancellable,
                                      GError      **error);
GPtrArray * fp_device_list_prints_sync (FpDevice     *device,
                                        GCancellable *cancellable,
                                        GError      **error);


G_END_DECLS
