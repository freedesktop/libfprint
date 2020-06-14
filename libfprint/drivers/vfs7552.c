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

#define SEND(ENDPOINT, COMMAND)  \
    {                            \
        .type = ACTION_SEND,     \
        .endpoint = ENDPOINT,    \
        .name = #COMMAND,        \
        .size = sizeof(COMMAND), \
        .data = COMMAND},

#define RECV(ENDPOINT, SIZE)    \
    {                           \
        .type = ACTION_RECEIVE, \
        .endpoint = ENDPOINT,   \
        .size = SIZE,           \
        .data = NULL},

#define RECV_CHECK(ENDPOINT, SIZE, EXPECTED) \
    {                                        \
        .type = ACTION_RECEIVE,              \
        .endpoint = ENDPOINT,                \
        .size = SIZE,                        \
        .data = EXPECTED,                    \
        .correct_reply_size = sizeof(EXPECTED)},

#define RECV_CHECK_SIZE(ENDPOINT, SIZE, EXPECTED) \
    {                                             \
        .type = ACTION_RECEIVE,                   \
        .endpoint = ENDPOINT,                     \
        .size = SIZE,                             \
        .data = NULL,                             \
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
async_send_cb (FpiUsbTransfer *transfer, FpDevice *device,
               gpointer user_data, GError *error)
{
  struct usbexchange_data *data = fpi_ssm_get_data (transfer->ssm);
  struct usb_action *action;

  g_assert (!(fpi_ssm_get_cur_state (transfer->ssm) >= data->stepcount));

  action = &data->actions[fpi_ssm_get_cur_state (transfer->ssm)];
  g_assert (!(action->type != ACTION_SEND));

  if (error)
    {
      /* Transfer not completed, return IO error */
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  /* success */
  fpi_ssm_next_state (transfer->ssm);
}

static void
async_recv_cb (FpiUsbTransfer *transfer, FpDevice *device,
               gpointer user_data, GError *error)
{
  struct usbexchange_data *data = fpi_ssm_get_data (transfer->ssm);
  struct usb_action *action;

  if (error)
    {
      /* Transfer not completed, return IO error */
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  g_assert (!(fpi_ssm_get_cur_state (transfer->ssm) >= data->stepcount));

  action = &data->actions[fpi_ssm_get_cur_state (transfer->ssm)];
  g_assert (!(action->type != ACTION_RECEIVE));

  if (action->data != NULL)
    {
      if (transfer->actual_length != action->correct_reply_size)
        {
          fp_err ("Got %d bytes instead of %d",
                  (gint) transfer->actual_length,
                  action->correct_reply_size);
          fpi_ssm_mark_failed (transfer->ssm, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
          return;
        }
      if (memcmp (transfer->buffer, action->data,
                  action->correct_reply_size) != 0)
        {
          fp_dbg ("Wrong reply:");
          fpi_ssm_mark_failed (transfer->ssm, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
          return;
        }
    }
  else
    {
      fp_dbg ("Got %d bytes out of %d",
              (gint) transfer->actual_length,
              (gint) transfer->length);
    }

  fpi_ssm_next_state (transfer->ssm);
}

static void
usbexchange_loop (FpiSsm *ssm, FpDevice *_dev)
{
  struct usbexchange_data *data = fpi_ssm_get_data (ssm);
  struct usb_action *action = &data->actions[fpi_ssm_get_cur_state (ssm)];
  FpiUsbTransfer *transfer;

  g_assert (fpi_ssm_get_cur_state (ssm) < data->stepcount);

  switch (action->type)
    {
    case ACTION_SEND:
      fp_dbg ("Sending %s", action->name);
      transfer = fpi_usb_transfer_new (_dev);
      fpi_usb_transfer_fill_bulk_full (transfer, action->endpoint,
                                       action->data, action->size,
                                       NULL);
      transfer->ssm = ssm;
      transfer->short_is_error = TRUE;
      fpi_usb_transfer_submit (transfer, data->timeout, NULL,
                               async_send_cb, NULL);
      break;

    case ACTION_RECEIVE:
      fp_dbg ("Receiving %d bytes", action->size);
      transfer = fpi_usb_transfer_new (_dev);
      fpi_usb_transfer_fill_bulk_full (transfer, action->endpoint,
                                       data->receive_buf,
                                       action->size, NULL);
      transfer->ssm = ssm;
      fpi_usb_transfer_submit (transfer, data->timeout, NULL,
                               async_recv_cb, NULL);
      break;

    default:
      fp_err ("Bug detected: invalid action %d", action->type);
      fpi_ssm_mark_failed (ssm, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      return;
    }
}

static void
usb_exchange_async (FpiSsm                  *ssm,
                    struct usbexchange_data *data,
                    const char              *exchange_name)
{
  FpiSsm *subsm = fpi_ssm_new_full (FP_DEVICE (data->device),
                                    usbexchange_loop,
                                    data->stepcount,
                                    exchange_name);

  fpi_ssm_set_data (subsm, data, NULL);
  fpi_ssm_start_subsm (ssm, subsm);
}

/* ================== Class Definition =================== */
struct _FpDeviceVfs7552
{
    FpImageDevice parent;

    unsigned char *total_buffer;
    unsigned char *capture_buffer;
    unsigned char *row_buffer;
    unsigned char *lastline;
    GSList *rows;
    int lines_captured, lines_recorded, empty_lines;
    int max_lines_captured, max_lines_recorded;
    int lines_total, lines_total_allocated;
    gboolean loop_running;
    gboolean deactivating;
    struct usbexchange_data init_sequence;
};

G_DECLARE_FINAL_TYPE(FpDeviceVfs7552, fpi_device_vfs7552, FPI, DEVICE_VFS7552,
                     FpImageDevice);
G_DEFINE_TYPE(FpDeviceVfs7552, fpi_device_vfs7552, FP_TYPE_IMAGE_DEVICE);

enum {
    DEV_OPEN_START,
    DEV_OPEN_NUM_STATES
};

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

static void
open_loop(FpiSsm *ssm, FpDevice *_dev)
{
    fp_dbg("--> open_loop");
    FpImageDevice *dev = FP_IMAGE_DEVICE (_dev);
    FpDeviceVfs7552 *self;

    self = FPI_DEVICE_VFS7552 (_dev);

    switch (fpi_ssm_get_cur_state (ssm))
    {
        case DEV_OPEN_START:
            self->init_sequence.stepcount =
                G_N_ELEMENTS (vfs7552_initialization);
            self->init_sequence.actions = vfs7552_initialization;
            self->init_sequence.device = dev;
            self->init_sequence.receive_buf =
                g_malloc0 (VFS7552_RECEIVE_BUF_SIZE);
            self->init_sequence.timeout = VFS7552_DEFAULT_WAIT_TIMEOUT;
            usb_exchange_async (ssm, &self->init_sequence, "DEVICE OPEN");
            break;
    }
}

static void
open_loop_complete(FpiSsm *ssm, FpDevice *_dev, GError *error)
{
    fp_dbg("--> open_loop_complete");
    FpImageDevice *dev = FP_IMAGE_DEVICE (_dev);
    FpDeviceVfs7552 *self;

    self = FPI_DEVICE_VFS7552 (_dev);
    g_free (self->init_sequence.receive_buf);
    self->init_sequence.receive_buf = NULL;

    fpi_image_device_open_complete (dev, error);
}

/* ================== Driver Entrypoints =================== */

static void
dev_change_state(FpImageDevice *dev, FpiImageDeviceState state)
{
    fp_dbg("--> dev_change_state");
    FpiSsm *ssm;

    switch (state) {
      case FPI_IMAGE_DEVICE_STATE_INACTIVE:
        // This state is never used...
        fp_dbg("== FPI_IMAGE_DEVICE_STATE_INACTIVE");
        break;
      case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON:
        fp_dbg("== FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON");
        // This state is called after activation completed or another enroll stage started
        //ssm = fpi_ssm_new(FP_DEVICE(dev), open_loop, DEV_OPEN_NUM_STATES);
        //fpi_ssm_start(ssm, open_loop_complete);
        break;
      case FPI_IMAGE_DEVICE_STATE_CAPTURE:
        fp_dbg("== FPI_IMAGE_DEVICE_STATE_CAPTURE");
        break;
      case FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF:
        fp_dbg("== FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF");
        break;
      default:
        fp_err("unrecognised state %d", state);
    }
}

/*
static int execute_state_change(struct fp_img_dev *dev){
	struct vfs7552_data *data = (struct vfs7552_data *)dev->priv;
  struct fpi_ssm *ssm;

  fp_dbg("state %d", data->activate_state);
  switch (data->activate_state) {
  case IMGDEV_STATE_INACTIVE:
    // This state is never used...
    break;
  case IMGDEV_STATE_AWAIT_FINGER_ON:
    ssm = fpi_ssm_new(dev->dev, await_finger_on_run_state, AWAIT_FINGER_ON_NUM_STATES);
    ssm->priv = dev;
		fpi_ssm_start(ssm, report_finger_on); // await_finger_on_complete
		break;
  case IMGDEV_STATE_CAPTURE:
    ssm = fpi_ssm_new(dev->dev, capture_run_state, CAPTURE_NUM_STATES);
    ssm->priv = dev;
    fpi_ssm_start(ssm, submit_image); // await_finger_on_complete
    break;
  case IMGDEV_STATE_AWAIT_FINGER_OFF:
    ssm = fpi_ssm_new(dev->dev, capture_run_state, CAPTURE_NUM_STATES);
    ssm->priv = dev;
    fpi_ssm_start(ssm, report_finger_off); // await_finger_on_complete
    break;
  }
  return 0;
}
*/

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

    // First we need to reset the device, otherwise opening will fail at state 13
	if (!g_usb_device_reset(fpi_device_get_usb_device(FP_DEVICE(dev)), &error)) {
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
    FpDeviceVfs7552 *self = FPI_DEVICE_VFS7552 (dev);

    g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)),
                                    0, 0, &error);

    g_free (self->capture_buffer);
    g_slist_free_full (self->rows, g_free);

    fpi_image_device_close_complete (dev, error);
}

/**
 * The second step after opening the connection to the device is the device activation.
 */
static void
dev_activate(FpImageDevice *dev)
{
    fp_dbg("--> dev_activate");
    FpDeviceVfs7552 *self;

    self = FPI_DEVICE_VFS7552 (dev);
    self->deactivating = FALSE;

    fpi_image_device_activate_complete(dev, NULL);
}

static void
dev_deactivate(FpImageDevice *dev)
{
    fp_dbg("--> dev_deactivate");
    FpDeviceVfs7552 *self;

    self = FPI_DEVICE_VFS7552 (dev);
    if (self->loop_running)
      self->deactivating = TRUE;
    else
      fpi_image_device_deactivate_complete (dev, NULL);
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

    //img_class->bz3_threshold = 20;

    img_class->img_width = VFS7552_IMAGE_WIDTH;
    img_class->img_height = VFS7552_IMAGE_HEIGHT;
}
