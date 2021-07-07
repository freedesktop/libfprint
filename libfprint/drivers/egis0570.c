#define FP_COMPONENT "egis0570"

#include "egis0570.h"
#include "drivers_api.h"

/* Packet types */
#define PKT_TYPE_INIT 0
#define PKT_TYPE_REPEAT 1

/* Struct */
struct _FpDeviceEgis0570
{
  FpImageDevice parent;

  gboolean      running;
  gboolean      stop;

  GSList       *strips;
  gsize         strips_len;

  int           pkt_num;
  int           pkt_type;
};
G_DECLARE_FINAL_TYPE (FpDeviceEgis0570, fpi_device_egis0570, FPI, DEVICE_EGIS0570, FpImageDevice);
G_DEFINE_TYPE (FpDeviceEgis0570, fpi_device_egis0570, FP_TYPE_IMAGE_DEVICE);

static unsigned char
egis_get_pixel (struct fpi_frame_asmbl_ctx *ctx, struct fpi_frame *frame, unsigned int x, unsigned int y)
{
  return frame->data[x + y * ctx->frame_width];
}

static struct fpi_frame_asmbl_ctx assembling_ctx = {
  .frame_width = EGIS0570_IMGWIDTH,
  .frame_height = EGIS0570_RFMGHEIGHT,
  .image_width = EGIS0570_IMGWIDTH * 4 / 3,
  .get_pixel = egis_get_pixel,
};

/*
 * Service
 */

static gboolean
is_last_pkt (FpDevice *dev)
{
  FpDeviceEgis0570 *self = FPI_DEVICE_EGIS0570 (dev);

  int type = self->pkt_type;
  int num = self->pkt_num;

  gboolean r;

  r = ((type == PKT_TYPE_INIT) && (num == (EGIS0570_INIT_TOTAL - 1)));
  r |= ((type == PKT_TYPE_REPEAT) && (num == (EGIS0570_REPEAT_TOTAL - 1)));

  return r;
}

static char
finger_status (guchar * img)
{
  size_t total[EGIS0570_IMGCOUNT] = {0, 0, 0, 0, 0};
  unsigned char min, max;

  min = max = img[0];

  for (size_t k = 0; k < EGIS0570_IMGCOUNT; ++k)
    {
      for (size_t i = ((k * EGIS0570_IMGSIZE) + EGIS0570_RFMDIS * EGIS0570_IMGWIDTH); i < (((k + 1) * EGIS0570_IMGSIZE) - EGIS0570_RFMDIS * EGIS0570_IMGWIDTH); ++i)
        {
          total[k] += img[i];

          if (img[i] < min)
            min = img[i];

          if (img[i] > max)
            max = img[i];
        }
    }

  unsigned char avg = 0;
  char result = 0;

  for (size_t k = 0; k < EGIS0570_IMGCOUNT; ++k)
    {
      avg = total[k] / (EGIS0570_IMGWIDTH * EGIS0570_RFMGHEIGHT);
      if (avg > EGIS0570_MIN_FINGER_PRESENT_AVG)           /* ReThink on values */
        result |= 1 << k;
    }

  fp_dbg ("Finger status (min, max, biggest avg) : %d : %d - %d", min, max, avg);

  return result;
}

/*
 * Device communication
 */

static void
data_resp_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  unsigned char *stripdata;
  gboolean end = FALSE;
  FpImageDevice *img_self = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0570 *self = FPI_DEVICE_EGIS0570 (dev);

  if (error)
    {
      fp_dbg ("request is not completed, %s", error->message);
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  int where_finger_is = finger_status (transfer->buffer);

  if (where_finger_is > 0)
    {
      fpi_image_device_report_finger_status (img_self, TRUE);
      for (size_t k = 0; k < EGIS0570_IMGCOUNT; ++k)
        {
          if (where_finger_is & (1 << k))
            {
              struct fpi_frame *stripe = g_malloc (EGIS0570_IMGWIDTH * EGIS0570_RFMGHEIGHT + sizeof (struct fpi_frame));
              stripe->delta_x = 0;
              stripe->delta_y = 0;
              stripdata = stripe->data;
              memcpy (stripdata, (transfer->buffer) + (((k) * EGIS0570_IMGSIZE) + EGIS0570_IMGWIDTH * EGIS0570_RFMDIS), EGIS0570_IMGWIDTH * EGIS0570_RFMGHEIGHT);
              self->strips = g_slist_prepend (self->strips, stripe);
              self->strips_len++;
            }
          else
            {
              end = TRUE;
              break;
            }
        }
    }
  else
    {
      end = TRUE;
    }

  if (end & (self->strips_len > 0))
    {
      if (!self->stop)
        {
          FpImage *img;
          self->strips = g_slist_reverse (self->strips);
          fpi_do_movement_estimation (&assembling_ctx, self->strips);
          img = fpi_assemble_frames (&assembling_ctx, self->strips);
          img->flags |= (FPI_IMAGE_COLORS_INVERTED | FPI_IMAGE_PARTIAL);
          g_slist_free_full (self->strips, g_free);
          self->strips = NULL;
          self->strips_len = 0;
          fpi_image_device_image_captured (img_self, img);
          fpi_image_device_report_finger_status (img_self, FALSE);
        }
    }

  fpi_ssm_next_state (transfer->ssm);
}

static void
recv_data_resp (FpiSsm *ssm, FpDevice *dev)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EGIS0570_EPIN, EGIS0570_INPSIZE);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;

  fpi_usb_transfer_submit (transfer, EGIS0570_TIMEOUT, NULL, data_resp_cb, NULL);
}

static void
cmd_resp_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  if (error)
    {
      fp_dbg ("bad answer, %s", error->message);
      fpi_ssm_mark_failed (transfer->ssm, error);
    }
}

static void
recv_cmd_resp (FpiSsm *ssm, FpDevice *dev)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EGIS0570_EPIN, EGIS0570_PKTSIZE);

  transfer->ssm = ssm;

  fpi_usb_transfer_submit (transfer, EGIS0570_TIMEOUT, NULL, cmd_resp_cb, NULL);

}

static void
cmd_req_cb (FpiUsbTransfer *transfer, FpDevice *dev, gpointer user_data, GError *error)
{
  if (!error)
    {
      fpi_ssm_next_state (transfer->ssm);
    }
  else
    {
      fp_dbg ("request is not completed, %s", error->message);
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }


}

static void
send_cmd_req (FpiSsm *ssm, FpDevice *dev, unsigned char *pkt)
{
  FpiUsbTransfer *transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk_full (transfer, EGIS0570_EPOUT, pkt, EGIS0570_PKTSIZE, NULL);

  transfer->ssm = ssm;
  transfer->short_is_error = TRUE;

  fpi_usb_transfer_submit (transfer, EGIS0570_TIMEOUT, NULL, cmd_req_cb, NULL);
}

/*
 * SSM States
 */

enum sm_states {
  SM_INIT,
  SM_START,
  SM_REQ,
  SM_RESP,
  SM_REC_DATA,
  SM_DONE,
  SM_STATES_NUM
};

static void
ssm_run_state (FpiSsm *ssm, FpDevice *dev)
{
  FpDeviceEgis0570 *self = FPI_DEVICE_EGIS0570 (dev);
  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case SM_INIT:
      self->pkt_type = PKT_TYPE_INIT;
      fpi_ssm_next_state (ssm);
      break;

    case SM_START:
      if (self->stop)
        {
          fp_dbg ("deactivating, marking completed");
          self->running = FALSE;
          fpi_ssm_mark_completed (ssm);
          fpi_image_device_deactivate_complete (img_dev, NULL);
        }
      else
        {
          self->running = TRUE;
          self->pkt_num = 0;
          fpi_ssm_next_state (ssm);
        }
      break;

    case SM_REQ:
      if (self->pkt_type == PKT_TYPE_INIT)
        send_cmd_req (ssm, dev, init_pkts[self->pkt_num]);
      else
        send_cmd_req (ssm, dev, repeat_pkts[self->pkt_num]);
      break;

    case SM_RESP:
      if (is_last_pkt (dev) == FALSE)
        {
          recv_cmd_resp (ssm, dev);
          ++(self->pkt_num);
          fpi_ssm_jump_to_state (ssm, SM_REQ);
        }
      else
        {
          if (self->pkt_type == PKT_TYPE_INIT)
            self->pkt_type = PKT_TYPE_REPEAT;

          fpi_ssm_next_state (ssm);
        }
      break;

    case SM_REC_DATA:
      recv_data_resp (ssm, dev);
      break;

    case SM_DONE:
      fpi_ssm_jump_to_state (ssm, SM_START);
      break;

    default:
      g_assert_not_reached ();
    }
}

/*
 * Activation
 */

static void
loop_complete (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpImageDevice *img_dev = FP_IMAGE_DEVICE (dev);
  FpDeviceEgis0570 *self = FPI_DEVICE_EGIS0570 (dev);

  //deallocation here
  self->running = FALSE;

  if (error)
    fpi_image_device_session_error (img_dev, error);
}

static void
dev_activate (FpImageDevice *dev)
{
  FpDeviceEgis0570 *self = FPI_DEVICE_EGIS0570 (dev);
  FpiSsm *ssm = fpi_ssm_new (FP_DEVICE (dev), ssm_run_state, SM_STATES_NUM);

  self->stop    = FALSE;

  fpi_ssm_start (ssm, loop_complete);

  self->running = TRUE;

  fpi_image_device_activate_complete (dev, NULL);
}

/*
 * Opening
 */

static void
dev_init (FpImageDevice *dev)
{
  GError *error = NULL;

  g_usb_device_claim_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);

  fpi_image_device_open_complete (dev, error);
}

/*
 * Closing
 */

static void
dev_deinit (FpImageDevice *dev)
{
  GError *error = NULL;

  /* should free leaking erea here */

  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (dev)), 0, 0, &error);

  fpi_image_device_close_complete (dev, error);
}

/*
 * Deactivation
 */

static void
dev_deactivate (FpImageDevice *dev)
{
  FpDeviceEgis0570 *self = FPI_DEVICE_EGIS0570 (dev);

  if (self->running)
    self->stop = TRUE;
  else
    fpi_image_device_deactivate_complete (dev, NULL);
}

/*
 * Driver data
 */

static const FpIdEntry id_table[] = {
  { .vid = EGIS0570_VID, .pid = EGIS0570_PID, },
  { .vid = 0,                .pid = 0,    .driver_data = 0 },                   /* terminating entry */
};

static void
fpi_device_egis0570_init (FpDeviceEgis0570 *self)
{
}

static void
fpi_device_egis0570_class_init (FpDeviceEgis0570Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);
  FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS (klass);

  dev_class->id = "egis0570";
  dev_class->full_name = "Egis Technology Inc. (aka. LighTuning) 0570";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->id_table = id_table;
  dev_class->scan_type = FP_SCAN_TYPE_SWIPE;

  img_class->img_open = dev_init;
  img_class->img_close = dev_deinit;
  img_class->activate = dev_activate;
  img_class->deactivate = dev_deactivate;

  img_class->img_width = EGIS0570_IMGWIDTH;
  img_class->img_height = -1;

  img_class->bz3_threshold = EGIS0570_BZ3_THRESHOLD; /* security issue */
}
