/*
 * Next Biometrics driver for libfprint
 *
 * Copyright (C) 2021 Huan Wang <fredwanghuan@gmail.com>
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

#define FP_COMPONENT "nb1010"

#include "fpi-log.h"
#include "drivers_api.h"
#include "nb1010.h"

struct _FpiDeviceNb1010
{
  FpImageDevice parent;
  guint8       *scanline_buf;
  int           partial_received;
};
G_DECLARE_FINAL_TYPE (FpiDeviceNb1010, fpi_device_nb1010, FPI, DEVICE_NB1010, FpImageDevice);
G_DEFINE_TYPE (FpiDeviceNb1010, fpi_device_nb1010, FP_TYPE_IMAGE_DEVICE);

static void
nb1010_dev_init (FpImageDevice *dev)
{
  GError *error = NULL;

  g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);
  fpi_image_device_open_complete (dev, error);

  FpiDeviceNb1010 *self = FPI_DEVICE_NB1010 (dev);
  self->scanline_buf = g_malloc0 (FRAME_WIDTH * FRAME_HEIGHT);

  fp_dbg ("nb1010 inited");
}

static void
nb1010_dev_deinit (FpImageDevice *dev)
{
  FpiDeviceNb1010 *self = FPI_DEVICE_NB1010 (dev);
  GError *error = NULL;

  g_clear_pointer (&self->scanline_buf, g_free);

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);
  fpi_image_device_close_complete (dev, error);

  fp_dbg ("nb1010 deinited");
}

static void
nb1010_dev_activate (FpImageDevice *dev)
{
  fp_dbg ("nb1010 activating");
  fpi_image_device_activate_complete (dev, NULL);
}

static void
nb1010_dev_deactivate (FpImageDevice *dev)
{
  fp_dbg ("nb1010 deactivating");
  fpi_image_device_deactivate_complete (dev, NULL);
}

static void
usb_send (FpDevice *dev, guint8 *data, gssize length, GError **error)
{
  GError *err = NULL;

  g_autoptr(FpiUsbTransfer) transfer = NULL;

  transfer = fpi_usb_transfer_new (dev);
  transfer->short_is_error = TRUE;

  fpi_usb_transfer_fill_bulk_full (transfer, NB1010_EP_OUT, data, length, NULL);
  fpi_usb_transfer_submit_sync (transfer, NB1010_DEFAULT_TIMEOUT, &err);
  if (err)
    {
      g_warning ("Error while sending data, continuing anyway: %s", err->message);
      g_propagate_error (error, err);
    }
}

static void
usb_recv (FpDevice *dev, int max_bytes, FpiUsbTransfer **out, GError **error)
{
  GError *err = NULL;

  g_autoptr(FpiUsbTransfer) transfer = NULL;

  /* XXX: This function swallows any transfer errors, that is obviously
   *      quite bad (it used to assert on no-error)! */

  transfer = fpi_usb_transfer_new (dev);
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (transfer, NB1010_EP_IN, max_bytes);
  fpi_usb_transfer_submit_sync (transfer, NB1010_DEFAULT_TIMEOUT, &err);

  if (err)
    {
      if (!error)
        g_warning ("Unhandled receive error: %s", err->message);
      g_propagate_error (error, err);
    }

  if (out)
    *out = g_steal_pointer (&transfer);
}


static void
nb1010_request_fingerprint (FpiDeviceNb1010 *dev)
{
  usb_send (FP_DEVICE (dev), nb1010_check_finger, nb1010_check_finger_len, NULL);
}

static int
nb1010_check_fingerprint (FpiDeviceNb1010 *dev)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpiUsbTransfer) transfer = NULL;
  usb_recv (FP_DEVICE (dev), NB1010_CMD_RECV_LEN, &transfer, &error);

  /* XXX: This is obviously not a sane error handling! */
  if (error)
    g_warning ("Unhandled receive error: %s", error->message);
  g_assert (!error);

  if (transfer->buffer[NB1010_SENSITIVITY_BIT] > 0x60)
    return TRUE;
  return FALSE;
}

static void
nb1010_start_capture (FpiDeviceNb1010 *dev)
{
  usb_send (FP_DEVICE (dev), nb1010_pre_capture, nb1010_pre_capture_len, NULL);
  usb_recv (FP_DEVICE (dev), NB1010_CMD_RECV_LEN, NULL, NULL);  // dont care about the return

  usb_send (FP_DEVICE (dev), nb1010_capture, nb1010_capture_len, NULL);
  usb_recv (FP_DEVICE (dev), NB1010_CMD_RECV_LEN, NULL, NULL);  // dont care about the return
}

static void
nb1010_read_partial (FpiDeviceNb1010 *dev)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpiUsbTransfer) transfer = NULL;

  usb_recv (FP_DEVICE (dev), NB1010_CAPTURE_RECV_LEN, &transfer, &error);

  g_assert (transfer->actual_length == NB1010_CAPTURE_RECV_LEN);

  /* XXX: This is obviously not a sane error handling! */
  g_assert (!error);

  size_t offset = dev->partial_received * NB1010_LINE_PER_PARTIAL * FRAME_WIDTH;
  memcpy (dev->scanline_buf + offset, transfer->buffer + NB1010_CAPTURE_HEADER_LEN, NB1010_LINE_PER_PARTIAL * FRAME_WIDTH);
  dev->partial_received++;
}

static void
nb1010_read_capture (FpiDeviceNb1010 *dev)
{
  for (int i = 0; i < NB1010_N_PARTIAL; i++)
    nb1010_read_partial (dev);
}

static int
submit_image (FpiSsm        *ssm,
              FpImageDevice *dev)
{
  FpiDeviceNb1010 *self = FPI_DEVICE_NB1010 (dev);
  FpImage *img;

  img = fp_image_new (FRAME_WIDTH, FRAME_HEIGHT);
  if (img == NULL)
    return 0;

  memcpy (img->data, self->scanline_buf, FRAME_WIDTH * FRAME_HEIGHT);
  fpi_image_device_image_captured (dev, img);

  return 1;
}

/* Loop ssm states */
enum {
  /* Step 0 - Scan finger */
  M_REQUEST_PRINT,
  M_WAIT_PRINT,
  M_CHECK_PRINT,
  M_READ_PRINT_START,
  M_READ_PRINT_WAIT,
  M_READ_PRINT_POLL,
  M_SUBMIT_PRINT,

  /* Number of states */
  M_LOOP_NUM_STATES,
};

/* Exec loop sequential state machine
   Mostly linear in the order of declaring the enum. The only part is after successful scan,
   it will jump back to M_REQUEST_PRINT to wait for finger off. */
static void
m_loop_state (FpiSsm *ssm, FpDevice *_dev)
{
  FpImageDevice *dev = FP_IMAGE_DEVICE (_dev);
  FpiDeviceNb1010 *self = FPI_DEVICE_NB1010 (_dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case M_REQUEST_PRINT:
      nb1010_request_fingerprint (self);
      fpi_ssm_next_state (ssm);
      break;

    case M_WAIT_PRINT:
      /* Wait fingerprint scanning */
      fpi_ssm_next_state_delayed (ssm, NB1010_TRANSITION_DELAY, NULL);
      break;

    case M_CHECK_PRINT:
      {
        if (!nb1010_check_fingerprint (self))
          fpi_ssm_jump_to_state (ssm, M_REQUEST_PRINT);
        fpi_ssm_next_state (ssm);
      }
      break;

    case M_READ_PRINT_START:
      fpi_image_device_report_finger_status (dev, TRUE);
      nb1010_start_capture (self);
      self->partial_received = 0;
      fpi_ssm_next_state (ssm);
      break;

    case M_READ_PRINT_WAIT:
      /* Wait fingerprint scanning */
      fpi_ssm_next_state_delayed (ssm, NB1010_TRANSITION_DELAY, NULL);
      break;

    case M_READ_PRINT_POLL:
      {
        nb1010_read_capture (self);
        fpi_ssm_next_state (ssm);
      }
      break;

    case M_SUBMIT_PRINT:
      {
        if (submit_image (ssm, dev))
          {
            fpi_ssm_mark_completed (ssm);
            fpi_image_device_report_finger_status (dev, FALSE);
          }
        else
          {
            fpi_ssm_jump_to_state (ssm, M_REQUEST_PRINT);
          }
      }
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
nb1010_dev_change_state (FpImageDevice *dev, FpiImageDeviceState state)
{
  FpiSsm *ssm_loop;

  if (state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON)
    {
      ssm_loop = fpi_ssm_new (FP_DEVICE (dev), m_loop_state, M_LOOP_NUM_STATES);
      fpi_ssm_start (ssm_loop, NULL);
    }
}


static const FpIdEntry id_table[] = {
  { .vid = 0x298d,  .pid = 0x1010, },
  { .vid = 0,  .pid = 0,  .driver_data = 0 },
};

static void
fpi_device_nb1010_init (FpiDeviceNb1010 *self)
{
}

static void
fpi_device_nb1010_class_init (FpiDeviceNb1010Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "NextBiometrics NB-1010-U";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;

  img_class->img_height = FRAME_HEIGHT;
  img_class->img_width = FRAME_WIDTH;

  img_class->img_open = nb1010_dev_init;
  img_class->img_close = nb1010_dev_deinit;
  img_class->activate = nb1010_dev_activate;
  img_class->deactivate = nb1010_dev_deactivate;
  img_class->change_state = nb1010_dev_change_state;
}
