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

#include "fpi-usb-transfer.h"

/**
 * SECTION:fpi-usb-transfer
 * @title: USB transfer helpers
 * @short_description: Helpers for libgusb to ease transfer handling
 *
 * #FpiUsbTransfer is a structure to simplify the USB transfer handling.
 * The main goal is to ease memory management and provide more parameters
 * to callbacks that are useful for libfprint drivers.
 *
 * Drivers should use this API only rather than accessing the GUsbDevice
 * directly in most cases.
 */


G_DEFINE_BOXED_TYPE (FpiUsbTransfer, fpi_usb_transfer, fpi_usb_transfer_ref, fpi_usb_transfer_unref)

static void
log_transfer (FpiUsbTransfer *transfer, gboolean submit, GError *error)
{
  if (g_getenv ("FP_DEBUG_TRANSFER"))
    {
      if (!submit)
        {
          g_autofree gchar *error_str = NULL;
          if (error)
            error_str = g_strdup_printf ("with error (%s)", error->message);
          else
            error_str = g_strdup ("successfully");

          g_debug ("Transfer %p completed %s, requested length %zd, actual length %zd, endpoint 0x%x",
                   transfer,
                   error_str,
                   transfer->length,
                   transfer->actual_length,
                   transfer->endpoint);
        }
      else
        {
          g_debug ("Transfer %p submitted, requested length %zd, endpoint 0x%x",
                   transfer,
                   transfer->length,
                   transfer->endpoint);
        }

      if (!submit == !!(transfer->endpoint & FPI_USB_ENDPOINT_IN))
        {
          g_autoptr(GString) line = NULL;
          gssize dump_len;

          dump_len = (transfer->endpoint & FPI_USB_ENDPOINT_IN) ? transfer->actual_length : transfer->length;

          line = g_string_new ("");
          /* Dump the buffer. */
          for (gint i = 0; i < dump_len; i++)
            {
              g_string_append_printf (line, "%02x ", transfer->buffer[i]);
              if ((i + 1) % 16 == 0)
                {
                  g_debug ("%s", line->str);
                  g_string_set_size (line, 0);
                }
            }

          if (line->len)
            g_debug ("%s", line->str);
        }
    }
}

/**
 * fpi_usb_transfer_new:
 * @device: The #FpDevice the transfer is for
 *
 * Creates a new #FpiUsbTransfer.
 *
 * Returns: (transfer full): A newly created #FpiUsbTransfer
 */
FpiUsbTransfer *
fpi_usb_transfer_new (FpDevice * device)
{
  FpiUsbTransfer *self;

  g_assert (device != NULL);

  self = g_slice_new0 (FpiUsbTransfer);
  self->ref_count = 1;
  self->type = FP_TRANSFER_NONE;

  self->device = device;

  return self;
}

static void
fpi_usb_transfer_free (FpiUsbTransfer *self)
{
  g_assert (self);
  g_assert_cmpint (self->ref_count, ==, 0);

  if (self->free_buffer && self->buffer)
    self->free_buffer (self->buffer);
  self->buffer = NULL;

  g_slice_free (FpiUsbTransfer, self);
}

/**
 * fpi_usb_transfer_ref:
 * @self: A #FpiUsbTransfer
 *
 * Increments the reference count of @self by one.
 *
 * Returns: (transfer full): @self
 */
FpiUsbTransfer *
fpi_usb_transfer_ref (FpiUsbTransfer *self)
{
  g_return_val_if_fail (self, NULL);
  g_return_val_if_fail (self->ref_count, NULL);

  g_atomic_int_inc (&self->ref_count);

  return self;
}

/**
 * fpi_usb_transfer_unref:
 * @self: A #FpiUsbTransfer
 *
 * Decrements the reference count of @self by one, freeing the structure when
 * the reference count reaches zero.
 */
void
fpi_usb_transfer_unref (FpiUsbTransfer *self)
{
  g_return_if_fail (self);
  g_return_if_fail (self->ref_count);

  if (g_atomic_int_dec_and_test (&self->ref_count))
    fpi_usb_transfer_free (self);
}

/**
 * fpi_usb_transfer_fill_bulk:
 * @transfer: The #FpiUsbTransfer
 * @endpoint: The endpoint to send the transfer to
 * @length: The buffer size to allocate
 *
 * Prepare a bulk transfer. A buffer will be created for you, use
 * fpi_usb_transfer_fill_bulk_full() if you want to send a static buffer
 * or receive a pre-defined buffer.
 */
void
fpi_usb_transfer_fill_bulk (FpiUsbTransfer *transfer,
                            guint8          endpoint,
                            gsize           length)
{
  fpi_usb_transfer_fill_bulk_full (transfer,
                                   endpoint,
                                   g_malloc0 (length),
                                   length,
                                   g_free);
}

/**
 * fpi_usb_transfer_fill_bulk_full:
 * @transfer: The #FpiUsbTransfer
 * @endpoint: The endpoint to send the transfer to
 * @buffer: The data to send. A buffer will be created and managed for you if you pass NULL.
 * @length: The size of @buffer
 * @free_func: (destroy buffer): Destroy notify for @buffer
 *
 * Prepare a bulk transfer.
 */
void
fpi_usb_transfer_fill_bulk_full (FpiUsbTransfer *transfer,
                                 guint8          endpoint,
                                 guint8         *buffer,
                                 gsize           length,
                                 GDestroyNotify  free_func)
{
  g_assert (transfer->type == FP_TRANSFER_NONE);
  g_assert (buffer != NULL);

  transfer->type = FP_TRANSFER_BULK;
  transfer->endpoint = endpoint;

  transfer->buffer = buffer;
  transfer->length = length;
  transfer->free_buffer = free_func;
}

/**
 * fpi_usb_transfer_fill_control:
 * @transfer: The #FpiUsbTransfer
 * @direction: The direction of the control transfer
 * @request_type: The request type
 * @recipient: The recipient
 * @request: The control transfer request
 * @value: The control transfer value
 * @idx: The control transfer index
 * @length: The size of the transfer
 *
 * Prepare a control transfer. The function will create a new buffer,
 * you can initialize the buffer after calling this function.
 */
void
fpi_usb_transfer_fill_control (FpiUsbTransfer       *transfer,
                               GUsbDeviceDirection   direction,
                               GUsbDeviceRequestType request_type,
                               GUsbDeviceRecipient   recipient,
                               guint8                request,
                               guint16               value,
                               guint16               idx,
                               gsize                 length)
{
  g_assert (transfer->type == FP_TRANSFER_NONE);

  transfer->type = FP_TRANSFER_CONTROL;
  transfer->direction = direction;
  transfer->request_type = request_type;
  transfer->recipient = recipient;
  transfer->request = request;
  transfer->value = value;
  transfer->idx = idx;

  transfer->length = length;
  transfer->buffer = g_malloc0 (length);
  transfer->free_buffer = g_free;
}

/**
 * fpi_usb_transfer_fill_interrupt:
 * @transfer: The #FpiUsbTransfer
 * @endpoint: The endpoint to send the transfer to
 * @length: The size of the transfer
 *
 * Prepare an interrupt transfer. The function will create a new buffer,
 * you can initialize the buffer after calling this function.
 */
void
fpi_usb_transfer_fill_interrupt (FpiUsbTransfer *transfer,
                                 guint8          endpoint,
                                 gsize           length)
{
  fpi_usb_transfer_fill_interrupt_full (transfer,
                                        endpoint,
                                        g_malloc0 (length),
                                        length,
                                        g_free);
}

/**
 * fpi_usb_transfer_fill_interrupt_full:
 * @transfer: The #FpiUsbTransfer
 * @endpoint: The endpoint to send the transfer to
 * @buffer: The data to send. A buffer will be created and managed for you if you pass NULL.
 * @length: The size of @buffer
 * @free_func: (destroy buffer): Destroy notify for @buffer
 *
 * Prepare an interrupt transfer.
 */
void
fpi_usb_transfer_fill_interrupt_full (FpiUsbTransfer *transfer,
                                      guint8          endpoint,
                                      guint8         *buffer,
                                      gsize           length,
                                      GDestroyNotify  free_func)
{
  g_assert (transfer->type == FP_TRANSFER_NONE);
  g_assert (buffer != NULL);

  transfer->type = FP_TRANSFER_INTERRUPT;
  transfer->endpoint = endpoint;

  transfer->buffer = buffer;
  transfer->length = length;
  transfer->free_buffer = free_func;
}

static void
transfer_finish_cb (GObject *source_object, GAsyncResult *res, gpointer user_data)
{
  GError *error = NULL;
  FpiUsbTransfer *transfer = user_data;
  FpiUsbTransferCallback callback;

  switch (transfer->type)
    {
    case FP_TRANSFER_BULK:
      transfer->actual_length =
        g_usb_device_bulk_transfer_finish (G_USB_DEVICE (source_object),
                                           res,
                                           &error);
      break;

    case FP_TRANSFER_CONTROL:
      transfer->actual_length =
        g_usb_device_control_transfer_finish (G_USB_DEVICE (source_object),
                                              res,
                                              &error);
      break;

    case FP_TRANSFER_INTERRUPT:
      transfer->actual_length =
        g_usb_device_interrupt_transfer_finish (G_USB_DEVICE (source_object),
                                                res,
                                                &error);
      break;

    case FP_TRANSFER_NONE:
    default:
      g_assert_not_reached ();
    }

  log_transfer (transfer, FALSE, error);

  /* Check for short error, and set an error if requested */
  if (error == NULL &&
      transfer->short_is_error &&
      transfer->actual_length > 0 &&
      transfer->actual_length != transfer->length)
    {
      error = g_error_new (G_USB_DEVICE_ERROR,
                           G_USB_DEVICE_ERROR_IO,
                           "Unexpected short error of %zd size (expected %zd)", transfer->actual_length, transfer->length);
    }

  callback = transfer->callback;
  transfer->callback = NULL;
  callback (transfer, transfer->device, transfer->user_data, error);

  fpi_usb_transfer_unref (transfer);
}


/**
 * fpi_usb_transfer_submit:
 * @transfer: (transfer full): The transfer to submit, must have been filled.
 * @timeout_ms: Timeout for the transfer in ms
 * @cancellable: Cancellable to use, e.g. fpi_device_get_cancellable()
 * @callback: Callback on completion or error
 * @user_data: Data to pass to callback
 *
 * Submit a USB transfer with a specific timeout and callback functions.
 *
 * Note that #FpiUsbTransfer will be stolen when this function is called.
 * So that all associated data will be free'ed automatically, after the
 * callback ran unless fpi_usb_transfer_ref() is explicitly called.
 */
void
fpi_usb_transfer_submit (FpiUsbTransfer        *transfer,
                         guint                  timeout_ms,
                         GCancellable          *cancellable,
                         FpiUsbTransferCallback callback,
                         gpointer               user_data)
{
  g_return_if_fail (transfer);
  g_return_if_fail (callback);

  /* Recycling is allowed, but not two at the same time. */
  g_return_if_fail (transfer->callback == NULL);

  transfer->callback = callback;
  transfer->user_data = user_data;

  log_transfer (transfer, TRUE, NULL);

  switch (transfer->type)
    {
    case FP_TRANSFER_BULK:
      g_usb_device_bulk_transfer_async (fpi_device_get_usb_device (transfer->device),
                                        transfer->endpoint,
                                        transfer->buffer,
                                        transfer->length,
                                        timeout_ms,
                                        cancellable,
                                        transfer_finish_cb,
                                        transfer);
      break;

    case FP_TRANSFER_CONTROL:
      g_usb_device_control_transfer_async (fpi_device_get_usb_device (transfer->device),
                                           transfer->direction,
                                           transfer->request_type,
                                           transfer->recipient,
                                           transfer->request,
                                           transfer->value,
                                           transfer->idx,
                                           transfer->buffer,
                                           transfer->length,
                                           timeout_ms,
                                           cancellable,
                                           transfer_finish_cb,
                                           transfer);
      break;

    case FP_TRANSFER_INTERRUPT:
      g_usb_device_interrupt_transfer_async (fpi_device_get_usb_device (transfer->device),
                                             transfer->endpoint,
                                             transfer->buffer,
                                             transfer->length,
                                             timeout_ms,
                                             cancellable,
                                             transfer_finish_cb,
                                             transfer);
      break;

    case FP_TRANSFER_NONE:
    default:
      fpi_usb_transfer_unref (transfer);
      g_return_if_reached ();
    }
}

/**
 * fpi_usb_transfer_submit_sync:
 * @transfer: The transfer to submit, must have been filled.
 * @timeout_ms: Timeout for the transfer in millisecnods
 * @error: Location to store #GError to
 *
 * Synchronously submit a USB transfer with a specific timeout.
 * Only use this function with short timeouts as the application will
 * be blocked otherwise.
 *
 * Note that you still need to fpi_usb_transfer_unref() the
 * #FpiUsbTransfer afterwards.
 *
 * Returns: #TRUE on success, otherwise #FALSE and @error will be set
 */
gboolean
fpi_usb_transfer_submit_sync (FpiUsbTransfer *transfer,
                              guint           timeout_ms,
                              GError        **error)
{
  gboolean res;
  gsize actual_length;

  g_return_val_if_fail (transfer, FALSE);

  /* Recycling is allowed, but not two at the same time. */
  g_return_val_if_fail (transfer->callback == NULL, FALSE);

  log_transfer (transfer, TRUE, NULL);

  switch (transfer->type)
    {
    case FP_TRANSFER_BULK:
      res = g_usb_device_bulk_transfer (fpi_device_get_usb_device (transfer->device),
                                        transfer->endpoint,
                                        transfer->buffer,
                                        transfer->length,
                                        &actual_length,
                                        timeout_ms,
                                        NULL,
                                        error);
      break;

    case FP_TRANSFER_CONTROL:
      res = g_usb_device_control_transfer (fpi_device_get_usb_device (transfer->device),
                                           transfer->direction,
                                           transfer->request_type,
                                           transfer->recipient,
                                           transfer->request,
                                           transfer->value,
                                           transfer->idx,
                                           transfer->buffer,
                                           transfer->length,
                                           &actual_length,
                                           timeout_ms,
                                           NULL,
                                           error);
      break;

    case FP_TRANSFER_INTERRUPT:
      res = g_usb_device_interrupt_transfer (fpi_device_get_usb_device (transfer->device),
                                             transfer->endpoint,
                                             transfer->buffer,
                                             transfer->length,
                                             &actual_length,
                                             timeout_ms,
                                             NULL,
                                             error);
      break;

    case FP_TRANSFER_NONE:
    default:
      g_return_val_if_reached (FALSE);
    }

  log_transfer (transfer, FALSE, *error);

  if (!res)
    transfer->actual_length = -1;
  else
    transfer->actual_length = actual_length;

  return res;
}
