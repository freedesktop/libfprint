/*
 * FPrint USB transfer handling
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
#include "fpi-device.h"

G_BEGIN_DECLS

#define FPI_TYPE_USB_TRANSFER (fpi_usb_transfer_get_type ())

#define FPI_USB_ENDPOINT_IN 0x80
#define FPI_USB_ENDPOINT_OUT 0x00

typedef struct _FpiUsbTransfer FpiUsbTransfer;
typedef struct _FpiSsm         FpiSsm;

typedef void (*FpiUsbTransferCallback)(FpiUsbTransfer *transfer,
                                       FpDevice       *dev,
                                       gpointer        user_data,
                                       GError         *error);

/**
 * FpiTransferType:
 * @FP_TRANSFER_NONE: Type not set
 * @FP_TRANSFER_BULK: Bulk transfer
 * @FP_TRANSFER_CONTROL: Control transfer
 * @FP_TRANSFER_INTERRUPT: Interrupt transfer
 *
 * Type of the transfer.
 */
typedef enum  {
  FP_TRANSFER_NONE = -1,
  FP_TRANSFER_CONTROL = 0,
  FP_TRANSFER_BULK = 2,
  FP_TRANSFER_INTERRUPT = 3,
} FpiTransferType;

/**
 * FpiUsbTransfer:
 * @device: The #FpDevice that the transfer belongs to.
 * @ssm: Storage slot to associate the transfer with a state machine.
 *   Used by fpi_ssm_usb_transfer_cb() to modify the given state machine.
 * @length: The requested length of the transfer in bytes.
 * @actual_length: The actual length of the transfer
 *  (see also fpi_usb_transfer_set_short_error())
 * @buffer: The transferred data.
 *
 * Helper for handling USB transfers.
 */
struct _FpiUsbTransfer
{
  /*< public >*/
  FpDevice *device;

  FpiSsm   *ssm;

  gssize    length;
  gssize    actual_length;

  guchar   *buffer;

  /*< private >*/
  guint ref_count;

  /* USB Transfer information */
  FpiTransferType type;
  guint8          endpoint;

  /* Control Transfer options */
  GUsbDeviceDirection   direction;
  GUsbDeviceRequestType request_type;
  GUsbDeviceRecipient   recipient;
  guint8                request;
  guint16               value;
  guint16               idx;

  /* Flags */
  gboolean short_is_error;

  /* Callbacks */
  gpointer               user_data;
  FpiUsbTransferCallback callback;

  /* Data free function */
  GDestroyNotify free_buffer;
};

GType              fpi_usb_transfer_get_type (void) G_GNUC_CONST;
FpiUsbTransfer     *fpi_usb_transfer_new (FpDevice *device);
FpiUsbTransfer     *fpi_usb_transfer_ref (FpiUsbTransfer *self);
void               fpi_usb_transfer_unref (FpiUsbTransfer *self);

void               fpi_usb_transfer_set_short_error (FpiUsbTransfer *transfer,
                                                     gboolean        short_is_error);

void               fpi_usb_transfer_fill_bulk (FpiUsbTransfer *transfer,
                                               guint8          endpoint,
                                               gsize           length);

void               fpi_usb_transfer_fill_bulk_full (FpiUsbTransfer *transfer,
                                                    guint8          endpoint,
                                                    guint8         *buffer,
                                                    gsize           length,
                                                    GDestroyNotify  free_func);

void               fpi_usb_transfer_fill_control (FpiUsbTransfer       *transfer,
                                                  GUsbDeviceDirection   direction,
                                                  GUsbDeviceRequestType request_type,
                                                  GUsbDeviceRecipient   recipient,
                                                  guint8                request,
                                                  guint16               value,
                                                  guint16               idx,
                                                  gsize                 length);

void               fpi_usb_transfer_fill_interrupt (FpiUsbTransfer *transfer,
                                                    guint8          endpoint,
                                                    gsize           length);

void               fpi_usb_transfer_fill_interrupt_full (FpiUsbTransfer *transfer,
                                                         guint8          endpoint,
                                                         guint8         *buffer,
                                                         gsize           length,
                                                         GDestroyNotify  free_func);

void               fpi_usb_transfer_submit (FpiUsbTransfer        *transfer,
                                            guint                  timeout_ms,
                                            GCancellable          *cancellable,
                                            FpiUsbTransferCallback callback,
                                            gpointer               user_data);

gboolean           fpi_usb_transfer_submit_sync (FpiUsbTransfer *transfer,
                                                 guint           timeout_ms,
                                                 GError        **error);


G_DEFINE_AUTOPTR_CLEANUP_FUNC (FpiUsbTransfer, fpi_usb_transfer_unref)

G_END_DECLS
