/*
 * Validity Sensors, Inc. VFS5011 Fingerprint Reader driver for libfprint
 * Copyright (C) 2013 Arseniy Lartsev <arseniy@chalmers.se>
 *                    AceLan Kao <acelan.kao@canonical.com>
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

#define FP_COMPONENT "vfs7552"

#include "drivers_api.h"
#include "vfs7552_proto.h"

#define VFS7552_CONTROL_PIXELS (8)
#define VFS7552_LINE_SIZE (VFS7552_IMAGE_WIDTH + VFS7552_CONTROL_PIXELS)
#define VFS7552_IMAGE_CHUNKS (3)
#define VARIANCE_THRESHOLD 600

/* =================== sync/async USB transfer sequence ==================== */

enum
{
  ACTION_SEND,
  ACTION_RECEIVE,
};

struct usb_action
{
  int type;
  const char *name;
  int endpoint;
  int size;
  unsigned char *data;
  int correct_reply_size;
};

#define SEND(ENDPOINT, COMMAND) \
  {                             \
      .type = ACTION_SEND,      \
      .endpoint = ENDPOINT,     \
      .name = #COMMAND,         \
      .size = sizeof(COMMAND),  \
      .data = COMMAND},

#define RECV(ENDPOINT, SIZE)  \
  {                           \
      .type = ACTION_RECEIVE, \
      .endpoint = ENDPOINT,   \
      .size = SIZE,           \
      .data = NULL},

#define RECV_CHECK(ENDPOINT, SIZE, EXPECTED) \
  {                                          \
      .type = ACTION_RECEIVE,                \
      .endpoint = ENDPOINT,                  \
      .size = SIZE,                          \
      .data = EXPECTED,                      \
      .correct_reply_size = sizeof(EXPECTED)},

#define RECV_CHECK_SIZE(ENDPOINT, SIZE, EXPECTED) \
  {                                               \
      .type = ACTION_RECEIVE,                     \
      .endpoint = ENDPOINT,                       \
      .size = SIZE,                               \
      .data = NULL,                               \
      .correct_reply_size = sizeof(EXPECTED)},

struct usbexchange_data
{
  int stepcount;
  FpImageDevice *device;
  struct usb_action *actions;
  void *receive_buf;
  int timeout;
};

/* ================== USB Communication ================== */

static void
async_send_cb(FpiUsbTransfer *transfer, FpDevice *device,
              gpointer user_data, GError *error)
{
  fp_dbg("--> async_send_cb");
  struct usbexchange_data *data = fpi_ssm_get_data(transfer->ssm);
  struct usb_action *action;

  g_assert(!(fpi_ssm_get_cur_state(transfer->ssm) >= data->stepcount));

  action = &data->actions[fpi_ssm_get_cur_state(transfer->ssm)];
  g_assert(!(action->type != ACTION_SEND));

  if (error)
  {
    /* Transfer not completed, return IO error */
    fpi_ssm_mark_failed(transfer->ssm, error);
    return;
  }

  /* success */
  fpi_ssm_next_state(transfer->ssm);
}

static void
async_recv_cb(FpiUsbTransfer *transfer, FpDevice *device,
              gpointer user_data, GError *error)
{
  fp_dbg("--> async_recv_cb");
  struct usbexchange_data *data = fpi_ssm_get_data(transfer->ssm);
  struct usb_action *action;

  if (error)
  {
    /* Transfer not completed, return IO error */
    fpi_ssm_mark_failed(transfer->ssm, error);
    return;
  }

  g_assert(!(fpi_ssm_get_cur_state(transfer->ssm) >= data->stepcount));

  action = &data->actions[fpi_ssm_get_cur_state(transfer->ssm)];
  g_assert(!(action->type != ACTION_RECEIVE));

  if (action->data != NULL)
  {
    if (transfer->actual_length != action->correct_reply_size)
    {
      fp_err("Got %d bytes instead of %d",
             (gint)transfer->actual_length,
             action->correct_reply_size);
      fpi_ssm_mark_failed(transfer->ssm, fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
      return;
    }
    if (memcmp(transfer->buffer, action->data,
               action->correct_reply_size) != 0)
    {
      fp_dbg("Wrong reply:");
      fpi_ssm_mark_failed(transfer->ssm, fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
      return;
    }
  }
  else
  {
    fp_dbg("Got %d bytes out of %d",
           (gint)transfer->actual_length,
           (gint)transfer->length);
  }

  fpi_ssm_next_state(transfer->ssm);
}

static void
usbexchange_loop(FpiSsm *ssm, FpDevice *_dev)
{
  fp_dbg("--> usbexchange_loop");
  struct usbexchange_data *data = fpi_ssm_get_data(ssm);
  struct usb_action *action = &data->actions[fpi_ssm_get_cur_state(ssm)];
  FpiUsbTransfer *transfer;

  g_assert(fpi_ssm_get_cur_state(ssm) < data->stepcount);

  switch (action->type)
  {
  case ACTION_SEND:
    fp_dbg("Sending %s", action->name);
    transfer = fpi_usb_transfer_new(_dev);
    fpi_usb_transfer_fill_bulk_full(transfer, action->endpoint,
                                    action->data, action->size,
                                    NULL);
    transfer->ssm = ssm;
    transfer->short_is_error = TRUE;
    fpi_usb_transfer_submit(transfer, data->timeout, NULL,
                            async_send_cb, NULL);
    break;

  case ACTION_RECEIVE:
    fp_dbg("Receiving %d bytes", action->size);
    transfer = fpi_usb_transfer_new(_dev);
    fpi_usb_transfer_fill_bulk_full(transfer, action->endpoint,
                                    data->receive_buf,
                                    action->size, NULL);
    transfer->ssm = ssm;
    fpi_usb_transfer_submit(transfer, data->timeout, NULL,
                            async_recv_cb, NULL);
    break;

  default:
    fp_err("Bug detected: invalid action %d", action->type);
    fpi_ssm_mark_failed(ssm, fpi_device_error_new(FP_DEVICE_ERROR_GENERAL));
    return;
  }
}

static void
usb_exchange_async(FpiSsm *ssm,
                   struct usbexchange_data *data,
                   const char *exchange_name)
{
  fp_dbg("--> usb_exchange_async");
  FpiSsm *subsm = fpi_ssm_new_full(FP_DEVICE(data->device),
                                   usbexchange_loop,
                                   data->stepcount,
                                   exchange_name);

  fpi_ssm_set_data(subsm, data, NULL);
  fpi_ssm_start_subsm(ssm, subsm);
}

/* ================== Class Definition =================== */
struct _FpDeviceVfs7552
{
  FpImageDevice parent;

  unsigned char *capture_buffer;
  FpiImageDeviceState dev_state;
  GSList *rows;
  unsigned char *image;
  gint image_index;
  gint chunks_captured;

  gboolean loop_running;
  gboolean deactivating;
  struct usbexchange_data init_sequence;
  FpiUsbTransfer *flying_transfer;
};

G_DECLARE_FINAL_TYPE(FpDeviceVfs7552, fpi_device_vfs7552, FPI, DEVICE_VFS7552,
                     FpImageDevice);
G_DEFINE_TYPE(FpDeviceVfs7552, fpi_device_vfs7552, FP_TYPE_IMAGE_DEVICE);

/* ======================= States ======================== */

enum
{
  DEV_OPEN_START,
  DEV_OPEN_NUM_STATES
};

enum
{
  AWAIT_FINGER_ON_INIT,
  AWAIT_FINGER_ON_INTERRUPT_QUERY,
  AWAIT_FINGER_ON_INTERRUPT_CHECK,
  AWAIT_FINGER_ON_QUERY_DATA_READY,
  AWAIT_FINGER_ON_CHECK_DATA_READY,
  AWAIT_FINGER_ON_REQUEST_CHUNK,
  AWAIT_FINGER_ON_READ_CHUNK,
  AWAIT_FINGER_ON_COMPLETE,
  AWAIT_FINGER_ON_FINALIZE,
  AWAIT_FINGER_ON_NUM_STATES
};

enum
{
  CAPTURE_QUERY_DATA_READY,
  CAPTURE_CHECK_DATA_READY,
  CAPTURE_REQUEST_CHUNK,
  CAPTURE_READ_CHUNK,
  CAPTURE_COMPLETE,
  CAPTURE_DISABLE_SENSOR,
  CAPTURE_DISABLE_COMPLETE,
  CAPTURE_NUM_STATES
};

/* ============== USB Sequence Definitions =============== */

struct usb_action vfs7552_initialization[] = {
    SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_01)
        RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 64, vfs7552_cmd_01_recv)

            SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_19)
                RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 128, vfs7552_cmd_19_recv)

                    SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_00)
                        RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)

                            SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_01)
                                RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)

                                    SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_02)
                                        RECV_CHECK(VFS7552_IN_ENDPOINT, 64, vfs7552_init_02_recv)

                                            SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_03)
                                                RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 64, vfs7552_init_03_recv)

                                                    SEND(VFS7552_OUT_ENDPOINT, vfs7552_init_04)
                                                        RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)
    /*
     * Windows driver does this and it works
     * But in this driver this call never returns...
     * RECV(VFS7552_IN_ENDPOINT_CTRL2, 8)
     */
};

struct usb_action vfs7552_stop_capture[] = {
    SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_04)
        RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)

            SEND(VFS7552_OUT_ENDPOINT, vfs7552_cmd_52)
                RECV_CHECK(VFS7552_IN_ENDPOINT, 64, VFS7552_NORMAL_REPLY)};

struct usb_action vfs7552_initiate_capture[] = {
    SEND(VFS7552_OUT_ENDPOINT, vfs7552_image_start)
        RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 2048, vfs7552_image_start_resp)};

struct usb_action vfs7552_wait_finger_init[] = {
    RECV_CHECK_SIZE(VFS7552_INTERRUPT_ENDPOINT, 8, interrupt_ok)};

struct usb_action vfs7552_data_ready_query[] = {
    SEND(VFS7552_OUT_ENDPOINT, vfs7552_is_image_ready)
        RECV_CHECK_SIZE(VFS7552_IN_ENDPOINT, 64, vfs7552_is_image_ready_resp_ready)

};
struct usb_action vfs7552_request_chunk[] = {
    SEND(VFS7552_OUT_ENDPOINT, vfs7552_read_image_chunk)};

/* ============= SSM Finalization Functions ============== */

static void
open_loop_complete(FpiSsm *ssm, FpDevice *_dev, GError *error)
{
  fp_dbg("--> open_loop_complete");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(_dev);
  g_free(self->init_sequence.receive_buf);
  self->init_sequence.receive_buf = NULL;

  fpi_image_device_open_complete(dev, error);
}

static void
report_finger_on(FpiSsm *ssm, FpDevice *_dev, GError *error)
{
  fp_dbg("--> report_finger_on");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(_dev);
  g_free(self->init_sequence.receive_buf);
  self->init_sequence.receive_buf = NULL;

  fpi_image_device_report_finger_status(dev, TRUE);
}

static void
submit_image(FpiSsm *ssm, FpDevice *_dev, GError *error)
{
  fp_dbg("--> submit_image");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;
  FpImage *img;
  img = fp_image_new(VFS7552_IMAGE_WIDTH,
                     VFS7552_IMAGE_HEIGHT);

  self = FPI_DEVICE_VFS7552(_dev);
  g_free(self->init_sequence.receive_buf);
  self->init_sequence.receive_buf = NULL;

  gint variance = fpi_std_sq_dev(self->image, VFS7552_IMAGE_SIZE);
  fp_dbg("variance = %d", variance);

  memcpy(img->data, self->image, VFS7552_IMAGE_SIZE);
  fpi_image_device_image_captured(dev, img);
}

static void
report_finger_off(FpiSsm *ssm, FpDevice *_dev, GError *error)
{
  fp_dbg("--> report_finger_off");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(_dev);
  g_free(self->init_sequence.receive_buf);
  self->init_sequence.receive_buf = NULL;

  fpi_image_device_report_finger_status(dev, FALSE);
}

/* =========== Image Capturing and Processing ============ */

enum
{
  CHUNK_READ_FINISHED,
  CHUNK_READ_NEED_MORE
};

static int
process_chunk(FpDeviceVfs7552 *self, int transferred)
{
  fp_dbg("--> process_chunk");
  fp_dbg("chunks captured: %d", self->chunks_captured + 1);

  unsigned char *ptr;
  int n_bytes_in_chunk;
  int n_lines;
  int i;

  ptr = self->capture_buffer;
  n_bytes_in_chunk = ptr[2] + ptr[3] * 256;

  ptr = ptr + 6;
  n_lines = n_bytes_in_chunk / VFS7552_LINE_SIZE;

  for (i = 0; i < n_lines; i++)
  {
    ptr = ptr + 8; // 8 bytes code at the beginning of each line
    memcpy(&self->image[self->image_index], ptr, VFS7552_IMAGE_WIDTH);
    ptr = ptr + VFS7552_IMAGE_WIDTH;
    self->image_index = self->image_index + VFS7552_IMAGE_WIDTH;
  }
  self->chunks_captured = self->chunks_captured + 1;
  if (self->chunks_captured == VFS7552_IMAGE_CHUNKS)
  {
    self->image_index = 0;
    self->chunks_captured = 0;
    return CHUNK_READ_FINISHED;
  }
  return CHUNK_READ_NEED_MORE;
}

static void
chunk_capture_callback(FpiUsbTransfer *transfer, FpDevice *device,
                       gpointer user_data, GError *error)
{
  fp_dbg("--> chunk_capture_callback");
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(device);

  if (self->dev_state == FPI_IMAGE_DEVICE_STATE_CAPTURE || self->dev_state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF)
  {
    if (error)
    {
      if (!self->deactivating)
      {
        fp_err("Failed to capture data");
        fpi_ssm_mark_failed(transfer->ssm, error);
      }
      else
      {
        fpi_ssm_mark_completed(transfer->ssm);
      }
    }
    else
    {
      if (process_chunk(self, transfer->actual_length) == CHUNK_READ_FINISHED)
      {
        fpi_ssm_next_state(transfer->ssm);
      }
      else
      {
        fpi_ssm_jump_to_state(transfer->ssm, CAPTURE_REQUEST_CHUNK);
      }
    }
  } else {
    if (error)
    {
      fp_err("Failed to capture data");
      fpi_ssm_mark_failed(transfer->ssm, error);
    }
    else
    {
      if (process_chunk(self, transfer->actual_length) == CHUNK_READ_FINISHED)
      {
        fpi_ssm_next_state(transfer->ssm);
      }
      else
      {
        fpi_ssm_jump_to_state(transfer->ssm, AWAIT_FINGER_ON_REQUEST_CHUNK);
      }
    }
  }

  self->flying_transfer = NULL;
}

static void
capture_chunk_async(FpiSsm *ssm, FpDevice *_dev, guint timeout)
{
  fp_dbg("--> capture_chunk_async");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(_dev);
  self->flying_transfer = fpi_usb_transfer_new(FP_DEVICE(_dev));

  fpi_usb_transfer_fill_bulk_full(self->flying_transfer, VFS7552_IN_ENDPOINT,
                                  self->capture_buffer, VFS7552_RECEIVE_BUF_SIZE,
                                  NULL);

  self->flying_transfer->ssm = ssm;
  fpi_usb_transfer_submit(self->flying_transfer, timeout, NULL,
                          chunk_capture_callback, NULL);
}

static void
capture_init(FpDeviceVfs7552 *self)
{
  fp_dbg("--> capture_init");
  self->image_index = 0;
  self->chunks_captured = 0;
}

/* ================ Delegation Functions ================= */

static void
open_loop(FpiSsm *ssm, FpDevice *_dev)
{
  fp_dbg("--> open_loop");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(_dev);

  switch (fpi_ssm_get_cur_state(ssm))
  {
  case DEV_OPEN_START:
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_initialization);
    self->init_sequence.actions = vfs7552_initialization;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = VFS7552_DEFAULT_WAIT_TIMEOUT;
    usb_exchange_async(ssm, &self->init_sequence, "DEVICE OPEN");
    break;
  }
}

static void
await_finger_on_loop(FpiSsm *ssm, FpDevice *_dev)
{
  fp_dbg("--> await_finger_on_loop");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;
  unsigned char *receive_buf;

  self = FPI_DEVICE_VFS7552(_dev);
  switch (fpi_ssm_get_cur_state(ssm))
  {
  case AWAIT_FINGER_ON_INIT:
    fp_dbg("== AWAIT_FINGER_ON_INIT");
    // This sequence prepares the sensor for capturing the image.
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_initiate_capture);
    self->init_sequence.actions = vfs7552_initiate_capture;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = VFS7552_DEFAULT_WAIT_TIMEOUT;
    usb_exchange_async(ssm, &self->init_sequence, "INITIATE CAPTURE");
    break;
  case AWAIT_FINGER_ON_INTERRUPT_QUERY:
    fp_dbg("== AWAIT_FINGER_ON_INTERRUPT_QUERY");
    // This sequence configures the sensor to listen to finger placement events.
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_wait_finger_init);
    self->init_sequence.actions = vfs7552_wait_finger_init;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = 0; // Do not time out
    usb_exchange_async(ssm, &self->init_sequence, "WAIT FOR FINGER");
    break;
  case AWAIT_FINGER_ON_INTERRUPT_CHECK:
    fp_dbg("== AWAIT_FINGER_ON_INTERRUPT_CHECK");
    receive_buf = ((unsigned char *)self->init_sequence.receive_buf);
    if (receive_buf[0] == interrupt_ok[0])
    {
      // This seems to mean: "Sensor is all good"
      fpi_ssm_jump_to_state(ssm, AWAIT_FINGER_ON_INTERRUPT_QUERY);
    }
    else if (receive_buf[0] == interrupt_ready[0])
    {
      // This seems to mean: "We detected a finger"
      fpi_ssm_next_state(ssm);
    }
    else if (receive_buf[0] == interrupt_dont_ask[0])
    {
      // This seems to mean: "We already told you we detected a finger, stop asking us"
      // It will not respond to another request on the interrupt endpoint
      fpi_ssm_next_state(ssm);
    }
    else
    {
      fp_dbg("Unknown response 0x%02x", receive_buf[0]);
      fpi_ssm_next_state(ssm);
    }
    break;
  case AWAIT_FINGER_ON_QUERY_DATA_READY:
    fp_dbg("== AWAIT_FINGER_ON_QUERY_DATA_READY");
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_data_ready_query);
    self->init_sequence.actions = vfs7552_data_ready_query;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = 0; // Do not time out
    usb_exchange_async(ssm, &self->init_sequence, "QUERY DATA READY");
    break;
  case AWAIT_FINGER_ON_CHECK_DATA_READY:
    fp_dbg("== AWAIT_FINGER_ON_CHECK_DATA_READY");
    receive_buf = ((unsigned char *)self->init_sequence.receive_buf);
    if (receive_buf[0] == vfs7552_is_image_ready_resp_not_ready[0])
    {
      fpi_ssm_jump_to_state(ssm, AWAIT_FINGER_ON_QUERY_DATA_READY);
    }
    else if (receive_buf[0] == vfs7552_is_image_ready_resp_ready[0])
    {
      capture_init(self);
      fpi_ssm_next_state(ssm);
    }
    else if (receive_buf[0] == vfs7552_is_image_ready_resp_finger_off[0])
    {
      fpi_ssm_jump_to_state(ssm, AWAIT_FINGER_ON_FINALIZE);
    }
    else
    {
      fp_dbg("Unknown response 0x%02x", receive_buf[0]);
      fpi_image_device_session_error(dev, NULL);
      fpi_ssm_mark_failed(ssm, NULL);
    }
    break;
  case AWAIT_FINGER_ON_REQUEST_CHUNK:
    fp_dbg("== AWAIT_FINGER_ON_REQUEST_CHUNK");
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_request_chunk);
    self->init_sequence.actions = vfs7552_request_chunk;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = 1000;
    usb_exchange_async(ssm, &self->init_sequence, "REQUEST CHUNK");
    break;
  case AWAIT_FINGER_ON_READ_CHUNK:
    fp_dbg("== AWAIT_FINGER_ON_READ_CHUNK");
    capture_chunk_async(ssm, _dev, 1000);
    break;
  case AWAIT_FINGER_ON_COMPLETE:
    fp_dbg("== AWAIT_FINGER_ON_COMPLETE");
    gint variance = fpi_std_sq_dev(self->image, VFS7552_IMAGE_SIZE);
    fp_dbg("variance = %d\n", variance);
    // If the finger is placed on the sensor, the variance should ideally increase above a certain
    // threshold. Otherwise request a new image and test again.
    if (variance > VARIANCE_THRESHOLD)
      fpi_ssm_next_state(ssm);
    else
      fpi_ssm_jump_to_state(ssm, AWAIT_FINGER_ON_QUERY_DATA_READY);
    break;
  case AWAIT_FINGER_ON_FINALIZE:
    fp_dbg("== AWAIT_FINGER_ON_FINALIZE");
    fpi_ssm_mark_completed(ssm);
    break;
  }
}

static void
capture_loop(FpiSsm *ssm, FpDevice *_dev)
{
  fp_dbg("--> capture_loop");
  FpImageDevice *dev = FP_IMAGE_DEVICE(_dev);
  FpDeviceVfs7552 *self;
  unsigned char *receive_buf;

  self = FPI_DEVICE_VFS7552(_dev);
  switch (fpi_ssm_get_cur_state(ssm))
  {
  case CAPTURE_QUERY_DATA_READY:
    fp_dbg("== CAPTURE_QUERY_DATA_READY");
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_data_ready_query);
    self->init_sequence.actions = vfs7552_data_ready_query;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = 0; // Do not time out
    usb_exchange_async(ssm, &self->init_sequence, "QUERY DATA READY");
    break;
  case CAPTURE_CHECK_DATA_READY:
    fp_dbg("== CAPTURE_CHECK_DATA_READY");
    receive_buf = ((unsigned char *)self->init_sequence.receive_buf);
    if (receive_buf[0] == vfs7552_is_image_ready_resp_not_ready[0])
    {
      fpi_ssm_jump_to_state(ssm, CAPTURE_QUERY_DATA_READY);
    }
    else if (receive_buf[0] == vfs7552_is_image_ready_resp_ready[0])
    {
      capture_init(self);
      fpi_ssm_next_state(ssm);
    }
    else if (receive_buf[0] == vfs7552_is_image_ready_resp_finger_off[0])
    {
      fpi_ssm_jump_to_state(ssm, CAPTURE_DISABLE_SENSOR);
    }
    else
    {
      fp_dbg("Unknown response 0x%02x", receive_buf[0]);
      fpi_image_device_session_error(dev, NULL);
      fpi_ssm_mark_failed(ssm, NULL);
    }
    break;
  case CAPTURE_REQUEST_CHUNK:
    fp_dbg("== CAPTURE_REQUEST_CHUNK");
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_request_chunk);
    self->init_sequence.actions = vfs7552_request_chunk;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = 1000;
    usb_exchange_async(ssm, &self->init_sequence, "REQUEST CHUNK");
    break;
  case CAPTURE_READ_CHUNK:
    fp_dbg("== CAPTURE_READ_CHUNK");
    capture_chunk_async(ssm, _dev, 1000);
    break;
  case CAPTURE_COMPLETE:
    fp_dbg("== CAPTURE_COMPLETE");
    if (self->dev_state == FPI_IMAGE_DEVICE_STATE_CAPTURE)
    {
      fpi_ssm_mark_completed(ssm);
    }
    else if (self->dev_state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF)
    {
      gint variance = fpi_std_sq_dev(self->image, VFS7552_IMAGE_SIZE);
      fp_dbg("variance = %d\n", variance);
      // If the finger is removed from the sensor, the variance should ideally drop below a certain
      // threshold.
      if (variance < VARIANCE_THRESHOLD)
        fpi_ssm_jump_to_state(ssm, CAPTURE_DISABLE_SENSOR);
      else
        fpi_ssm_jump_to_state(ssm, CAPTURE_QUERY_DATA_READY);
    }
    break;
  case CAPTURE_DISABLE_SENSOR:
    fp_dbg("== CAPTURE_DISABLE_SENSOR");
    self->init_sequence.stepcount =
        G_N_ELEMENTS(vfs7552_stop_capture);
    self->init_sequence.actions = vfs7552_stop_capture;
    self->init_sequence.device = dev;
    self->init_sequence.receive_buf =
        g_malloc0(VFS7552_RECEIVE_BUF_SIZE);
    self->init_sequence.timeout = 1000;
    usb_exchange_async(ssm, &self->init_sequence, "STOP CAPTURE");
    break;
  case CAPTURE_DISABLE_COMPLETE:
    fp_dbg("== CAPTURE_DISABLE_COMPLETE");
    fpi_ssm_mark_completed(ssm);
    break;
  }
}

/* ================== Driver Entrypoints =================== */

static void
dev_change_state(FpImageDevice *dev, FpiImageDeviceState state)
{
  fp_dbg("--> dev_change_state");
  FpiSsm *ssm;
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(dev);

  switch (state)
  {
  case FPI_IMAGE_DEVICE_STATE_INACTIVE:
    // This state is never used...
    fp_dbg("== FPI_IMAGE_DEVICE_STATE_INACTIVE");
    break;
  case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON:
    fp_dbg("== FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON");
    // This state is called after activation completed or another enroll stage started
    self->dev_state = FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON;
    ssm = fpi_ssm_new(FP_DEVICE(dev), await_finger_on_loop, AWAIT_FINGER_ON_NUM_STATES);
    fpi_ssm_start(ssm, report_finger_on);
    break;
  case FPI_IMAGE_DEVICE_STATE_CAPTURE:
    fp_dbg("== FPI_IMAGE_DEVICE_STATE_CAPTURE");
    self->dev_state = FPI_IMAGE_DEVICE_STATE_CAPTURE;
    ssm = fpi_ssm_new(FP_DEVICE(dev), capture_loop, CAPTURE_NUM_STATES);
    fpi_ssm_start(ssm, submit_image);
    break;
  case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF:
    fp_dbg("== FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF");
    self->dev_state = FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF;
    ssm = fpi_ssm_new(FP_DEVICE(dev), capture_loop, CAPTURE_NUM_STATES);
    fpi_ssm_start(ssm, report_finger_off);
    break;
  default:
    fp_err("unrecognised state %d", state);
  }
}

/**
 * This is the first entrypoint that's called by libfprint. Here we claim the interface and start
 * the open loop.
 */
static void
dev_open(FpImageDevice *dev)
{
  fp_dbg("--> dev_open");
  FpiSsm *ssm;
  GError *error = NULL;
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(dev);
  self->capture_buffer = g_new0(unsigned char, VFS7552_RECEIVE_BUF_SIZE);
  self->image =
      (unsigned char *)g_malloc0(VFS7552_IMAGE_HEIGHT * VFS7552_IMAGE_WIDTH);

  // First we need to reset the device, otherwise opening will fail at state 13
  if (!g_usb_device_reset(fpi_device_get_usb_device(FP_DEVICE(dev)), &error))
  {
    fpi_image_device_open_complete(dev, error);
    return;
  }

  if (!g_usb_device_claim_interface(fpi_device_get_usb_device(FP_DEVICE(dev)), 0, 0, &error))
  {
    fpi_image_device_open_complete(dev, error);
    return;
  }

  ssm = fpi_ssm_new(FP_DEVICE(dev), open_loop, DEV_OPEN_NUM_STATES);
  fpi_ssm_start(ssm, open_loop_complete);
}

static void
dev_close(FpImageDevice *dev)
{
  fp_dbg("--> dev_close");
  GError *error = NULL;
  FpDeviceVfs7552 *self = FPI_DEVICE_VFS7552(dev);

  g_usb_device_release_interface(fpi_device_get_usb_device(FP_DEVICE(dev)),
                                 0, 0, &error);

  g_free(self->capture_buffer);
  g_slist_free_full(self->rows, g_free);

  fpi_image_device_close_complete(dev, error);
}

/**
 * The second step after opening the connection to the device is the device activation.
 * We don't actually do any communication with the device here.
 */
static void
dev_activate(FpImageDevice *dev)
{
  fp_dbg("--> dev_activate");
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(dev);
  self->deactivating = FALSE;

  fpi_image_device_activate_complete(dev, NULL);
}

static void
dev_deactivate(FpImageDevice *dev)
{
  fp_dbg("--> dev_deactivate");
  FpDeviceVfs7552 *self;

  self = FPI_DEVICE_VFS7552(dev);
  if (self->loop_running)
    self->deactivating = TRUE;
  else
    fpi_image_device_deactivate_complete(dev, NULL);
}

static const FpIdEntry id_table[] = {
    {/* Validity device from some Dell XPS laptops (9560, 9360 at least) */ .vid = 0x138a, .pid = 0x0091},
};

static void
fpi_device_vfs7552_init(FpDeviceVfs7552 *self)
{
}

static void
fpi_device_vfs7552_class_init(FpDeviceVfs7552Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS(klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS(klass);

  dev_class->id = "vfs7552";
  dev_class->full_name = "Validity VFS7552";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;

  img_class->img_open = dev_open;
  img_class->img_close = dev_close;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;
  img_class->change_state = dev_change_state;

  img_class->bz3_threshold = 20;

  img_class->img_width = VFS7552_IMAGE_WIDTH;
  img_class->img_height = VFS7552_IMAGE_HEIGHT;
}