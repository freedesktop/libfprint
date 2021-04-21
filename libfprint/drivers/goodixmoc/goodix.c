/*
 * Goodix Moc driver for libfprint
 * Copyright (C) 2019 Shenzhen Goodix Technology Co., Ltd.
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



#define FP_COMPONENT "goodixmoc"

#include "drivers_api.h"

#include "goodix_proto.h"
#include "goodix.h"


/* Default enroll stages number */
#define DEFAULT_ENROLL_SAMPLES 8
/* Usb port setting */
#define EP_IN (3 | FPI_USB_ENDPOINT_IN)
#define EP_OUT (1 | FPI_USB_ENDPOINT_OUT)

#define EP_IN_MAX_BUF_SIZE (2048)

#define MAX_USER_ID_LEN (64)

/* Command transfer timeout :ms*/
#define CMD_TIMEOUT (1000)
#define ACK_TIMEOUT (2000)
#define DATA_TIMEOUT (5000)


struct _FpiDeviceGoodixMoc
{
  FpDevice           parent;
  FpiSsm            *task_ssm;
  FpiSsm            *cmd_ssm;
  FpiUsbTransfer    *cmd_transfer;
  gboolean           cmd_cancelable;
  pgxfp_sensor_cfg_t sensorcfg;
  gint               enroll_stage;
  gint               max_enroll_stage;
  gint               max_stored_prints;
  GPtrArray         *list_result;
  guint8             template_id[TEMPLATE_ID_SIZE];
  gboolean           is_enroll_identify;
  gboolean           is_power_button_shield_on;

};

G_DEFINE_TYPE (FpiDeviceGoodixMoc, fpi_device_goodixmoc, FP_TYPE_DEVICE)

typedef void (*SynCmdMsgCallback) (FpiDeviceGoodixMoc  *self,
                                   gxfp_cmd_response_t *resp,
                                   GError              *error);

typedef struct
{
  guint8            cmd;
  SynCmdMsgCallback callback;
} CommandData;

static gboolean parse_print_data (GVariant      *data,
                                  guint8        *finger,
                                  const guint8 **tid,
                                  gsize         *tid_len,
                                  const guint8 **user_id,
                                  gsize         *user_id_len);
/******************************************************************************
 *
 *  fp_cmd_xxx Function
 *
 *****************************************************************************/
static void
fp_cmd_receive_cb (FpiUsbTransfer *transfer,
                   FpDevice       *device,
                   gpointer        user_data,
                   GError         *error)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  CommandData *data = user_data;
  int ret = -1, ssm_state = 0;
  gxfp_cmd_response_t cmd_reponse = {0, };
  pack_header header;
  guint32 crc32_calc = 0;
  guint16 cmd = 0;

  if (error)
    {
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }
  if (data == NULL)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      return;
    }
  ssm_state = fpi_ssm_get_cur_state (transfer->ssm);
  /* skip zero length package */
  if (transfer->actual_length == 0)
    {
      fpi_ssm_jump_to_state (transfer->ssm, ssm_state);
      return;
    }

  ret = gx_proto_parse_header (transfer->buffer, transfer->actual_length, &header);
  if (ret != 0)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Corrupted message received"));
      return;
    }

  gx_proto_crc32_calc (transfer->buffer, PACKAGE_HEADER_SIZE + header.len, (uint8_t *) &crc32_calc);
  if(crc32_calc != GUINT32_FROM_LE (*(uint32_t *) (transfer->buffer + PACKAGE_HEADER_SIZE + header.len)))
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Package crc check failed"));
      return;
    }

  cmd = MAKE_CMD_EX (header.cmd0, header.cmd1);

  ret = gx_proto_parse_body (cmd, &transfer->buffer[PACKAGE_HEADER_SIZE], header.len, &cmd_reponse);
  if (ret != 0)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Corrupted message received"));
      return;
    }
  /* ack */
  if(header.cmd0 == RESPONSE_PACKAGE_CMD)
    {
      if (data->cmd != cmd_reponse.parse_msg.ack_cmd)
        {
          fpi_ssm_mark_failed (transfer->ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Unexpected response, got 0x%x",
                                                         cmd_reponse.parse_msg.ack_cmd));

          return;
        }
      fpi_ssm_next_state (transfer->ssm);
      return;
    }
  /* data */
  if (data->cmd != header.cmd0)
    {
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Unexpected cmd, got 0x%x",
                                                     header.cmd0));
      return;
    }
  if (data->callback)
    data->callback (self, &cmd_reponse, NULL);

  fpi_ssm_mark_completed (transfer->ssm);
}


static void
fp_cmd_run_state (FpiSsm   *ssm,
                  FpDevice *dev)
{
  FpiUsbTransfer *transfer;
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FP_CMD_SEND:
      if (self->cmd_transfer)
        {
          self->cmd_transfer->ssm = ssm;
          fpi_usb_transfer_submit (g_steal_pointer (&self->cmd_transfer),
                                   CMD_TIMEOUT,
                                   NULL,
                                   fpi_ssm_usb_transfer_cb,
                                   NULL);
        }
      else
        {
          fpi_ssm_next_state (ssm);
        }
      break;

    case FP_CMD_GET_ACK:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, EP_IN, EP_IN_MAX_BUF_SIZE);
      fpi_usb_transfer_submit (transfer,
                               ACK_TIMEOUT,
                               NULL,
                               fp_cmd_receive_cb,
                               fpi_ssm_get_data (ssm));

      break;

    case FP_CMD_GET_DATA:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, EP_IN, EP_IN_MAX_BUF_SIZE);
      fpi_usb_transfer_submit (transfer,
                               self->cmd_cancelable ? 0 : DATA_TIMEOUT,
                               self->cmd_cancelable ? fpi_device_get_cancellable (dev) : NULL,
                               fp_cmd_receive_cb,
                               fpi_ssm_get_data (ssm));
      break;
    }

}


static void
fp_cmd_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (dev);
  CommandData *data = fpi_ssm_get_data (ssm);

  self->cmd_ssm = NULL;
  /* Notify about the SSM failure from here instead. */
  if (error)
    {
      if (data->callback)
        data->callback (self, NULL, error);
      else
        g_error_free (error);
    }
}

static FpiUsbTransfer *
alloc_cmd_transfer (FpDevice     *dev,
                    guint8        cmd0,
                    guint8        cmd1,
                    const guint8 *data,
                    guint16       data_len)
{
  gint ret = -1;

  g_autoptr(FpiUsbTransfer) transfer = NULL;

  guint32 total_len = data_len + PACKAGE_HEADER_SIZE + PACKAGE_CRC_SIZE;

  g_return_val_if_fail (data || data_len == 0, NULL);

  transfer = fpi_usb_transfer_new (dev);

  fpi_usb_transfer_fill_bulk (transfer, EP_OUT, total_len);

  ret = gx_proto_build_package (transfer->buffer, &total_len, MAKE_CMD_EX (cmd0, cmd1), data, data_len);

  g_return_val_if_fail (ret == 0, NULL);

  return g_steal_pointer (&transfer);
}

static void
fp_cmd_ssm_done_data_free (CommandData *data)
{
  g_free (data);
}

static void
goodix_sensor_cmd (FpiDeviceGoodixMoc *self,
                   guint8              cmd0,
                   guint8              cmd1,
                   gboolean            bwait_data_delay,
                   const guint8      * payload,
                   gssize              payload_len,
                   SynCmdMsgCallback   callback)
{

  g_autoptr(FpiUsbTransfer) transfer = NULL;

  CommandData *data = g_new0 (CommandData, 1);

  transfer = alloc_cmd_transfer (FP_DEVICE (self), cmd0, cmd1, payload, payload_len);

  data->cmd = cmd0;
  data->callback = callback;

  self->cmd_transfer = g_steal_pointer (&transfer);
  self->cmd_cancelable = bwait_data_delay;

  self->cmd_ssm = fpi_ssm_new (FP_DEVICE (self),
                               fp_cmd_run_state,
                               FP_CMD_NUM_STATES);

  fpi_ssm_set_data (self->cmd_ssm, data, (GDestroyNotify) fp_cmd_ssm_done_data_free);

  fpi_ssm_start (self->cmd_ssm, fp_cmd_ssm_done);


}

/******************************************************************************
 *
 *  fp_pwr_btn_shield_cb Function
 *
 *****************************************************************************/
static void
fp_pwr_btn_shield_cb (FpiDeviceGoodixMoc  *self,
                      gxfp_cmd_response_t *resp,
                      GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->result >= GX_FAILED)
    {
      fp_dbg ("Setting power button shield failed, result: 0x%x", resp->result);
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      return;
    }
  if (resp->power_button_shield_resp.resp_cmd1 == MOC_CMD1_PWR_BTN_SHIELD_ON)
    self->is_power_button_shield_on = true;
  else
    self->is_power_button_shield_on = false;
  fpi_ssm_next_state (self->task_ssm);
}
/******************************************************************************
 *
 *  fp_verify_xxxx Function
 *
 *****************************************************************************/
static void
fp_verify_capture_cb (FpiDeviceGoodixMoc  *self,
                      gxfp_cmd_response_t *resp,
                      GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->result >= GX_FAILED)
    {
      fp_dbg ("Capture sample failed, result: 0x%x", resp->result);
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      return;
    }
  fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                           FP_FINGER_STATUS_PRESENT,
                                           FP_FINGER_STATUS_NONE);
  if (resp->capture_data_resp.img_quality == 0)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_retry_new (FP_DEVICE_RETRY_REMOVE_FINGER));
      return;
    }
  else if (resp->capture_data_resp.img_coverage < 35)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_verify_cb (FpiDeviceGoodixMoc  *self,
              gxfp_cmd_response_t *resp,
              GError              *error)
{
  g_autoptr(GPtrArray) templates = NULL;
  FpDevice *device = FP_DEVICE (self);
  FpPrint *print = NULL;
  gint cnt = 0;
  gboolean find = false;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->verify.match)
    {
      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_VERIFY)
        {
          templates = g_ptr_array_sized_new (1);
          fpi_device_get_verify_data (device, &print);
          g_ptr_array_add (templates, print);
        }
      else
        {
          fpi_device_get_identify_data (device, &templates);
          g_ptr_array_ref (templates);
        }
      for (cnt = 0; cnt < templates->len; cnt++)
        {
          g_autoptr(GVariant) data = NULL;
          guint8 finger;
          const guint8 *user_id;
          gsize user_id_len = 0;
          const guint8 *tid;
          gsize tid_len = 0;
          print = g_ptr_array_index (templates, cnt);
          g_object_get (print, "fpi-data", &data, NULL);
          if (!parse_print_data (data, &finger, &tid, &tid_len, &user_id, &user_id_len))
            {
              fpi_ssm_mark_failed (self->task_ssm,
                                   fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_INVALID,
                                                             "Parse print error"));
              return;
            }
          if (memcmp (&resp->verify.template.tid, tid, TEMPLATE_ID_SIZE) == 0)
            {
              find = true;
              break;
            }

        }
      if (find)
        {
          if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_VERIFY)
            fpi_device_verify_report (device, FPI_MATCH_SUCCESS, NULL, error);
          else
            fpi_device_identify_report (device, print, print, error);
        }
    }

  if (!find)
    {
      if (fpi_device_get_current_action (device) == FPI_DEVICE_ACTION_VERIFY)
        fpi_device_verify_report (device, FPI_MATCH_FAIL, NULL, error);
      else
        fpi_device_identify_report (device, NULL, NULL, error);
    }

  fpi_ssm_next_state (self->task_ssm);

}

static void
fp_verify_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  guint8 param[3] = { 0 };
  guint8 nonce[TEMPLATE_ID_SIZE] = { 0 };

  param[0] = 0x01;
  param[1] = self->sensorcfg->config[10];
  param[2] = self->sensorcfg->config[11];

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FP_VERIFY_PWR_BTN_SHIELD_ON:
      goodix_sensor_cmd (self, MOC_CMD0_PWR_BTN_SHIELD, MOC_CMD1_PWR_BTN_SHIELD_ON,
                         false,
                         NULL,
                         0,
                         fp_pwr_btn_shield_cb);
      break;

    case FP_VERIFY_CAPTURE:
      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_NONE);
      goodix_sensor_cmd (self, MOC_CMD0_CAPTURE_DATA, MOC_CMD1_DEFAULT,
                         true,
                         (const guint8 *) &param,
                         G_N_ELEMENTS (param),
                         fp_verify_capture_cb);
      break;

    case FP_VERIFY_IDENTIFY:
      goodix_sensor_cmd (self, MOC_CMD0_IDENTIFY, MOC_CMD1_DEFAULT,
                         false,
                         (const guint8 *) nonce,
                         TEMPLATE_ID_SIZE,
                         fp_verify_cb);
      break;

    case FP_VERIFY_PWR_BTN_SHIELD_OFF:
      goodix_sensor_cmd (self, MOC_CMD0_PWR_BTN_SHIELD, MOC_CMD1_PWR_BTN_SHIELD_OFF,
                         false,
                         NULL,
                         0,
                         fp_pwr_btn_shield_cb);
      break;
    }

}

static void
fp_verify_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (dev);

  fp_info ("Verify complete!");

  if (error && error->domain == FP_DEVICE_RETRY)
    {
      if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_VERIFY)
        fpi_device_verify_report (dev, FPI_MATCH_ERROR, NULL, g_steal_pointer (&error));
      else
        fpi_device_identify_report (dev, NULL, NULL, g_steal_pointer (&error));
    }

  if (fpi_device_get_current_action (dev) == FPI_DEVICE_ACTION_VERIFY)
    fpi_device_verify_complete (dev, error);

  else
    fpi_device_identify_complete (dev, error);

  self->task_ssm = NULL;
}


/******************************************************************************
 *
 *  fp__xxxx Function
 *
 *****************************************************************************/
static gboolean
encode_finger_id (
  const guint8 * tid,
  guint16        tid_len,
  const guint8 * uid,
  guint16        uid_len,
  guint8      ** fid,
  guint16      * fid_len
                 )
{
  guint8 * buffer = NULL;
  guint16 offset = 0;

  g_return_val_if_fail (tid != NULL, FALSE);
  g_return_val_if_fail (uid != NULL, FALSE);
  g_return_val_if_fail (fid != NULL, FALSE);
  g_return_val_if_fail (fid_len != NULL, FALSE);

  *fid_len = (guint16) (70 + uid_len);  // must include fingerid length

  *fid = (guint8 *) g_malloc0 (*fid_len + 2);

  buffer = *fid;
  offset = 0;
  buffer[offset++] = LOBYTE (*fid_len);
  buffer[offset++] = HIBYTE (*fid_len);

  buffer[offset++] = 67;
  buffer[offset++] = 1;
  buffer[offset++] = 1;   // finger index
  buffer[offset++] = 0;   //

  offset += 32;

  memcpy (&buffer[offset], tid, MIN (tid_len, TEMPLATE_ID_SIZE));
  offset += 32;   // offset == 68

  buffer[offset++] = uid_len;
  memcpy (&buffer[offset], uid, uid_len);
  offset += (guint8) uid_len;

  buffer[offset++] = 0;

  if (offset != (*fid_len + 2))
    {
      memset (buffer, 0, *fid_len);
      *fid_len = 0;

      fp_err ("offset != fid_len, %d != %d", offset, *fid_len);
      return FALSE;
    }
  *fid_len += 2;

  return TRUE;
}
/******************************************************************************
 *
 *  fp_enroll_xxxx Function
 *
 *****************************************************************************/

static void
fp_enroll_enum_cb (FpiDeviceGoodixMoc  *self,
                   gxfp_cmd_response_t *resp,
                   GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->result != GX_SUCCESS)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                     "Failed to enumerate fingers, result: 0x%x",
                                                     resp->result));
      return;
    }
  if (resp->finger_list_resp.finger_num >= self->max_stored_prints)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_FULL));
      return;
    }

  fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_CAPTURE);
}

static void
fp_enroll_identify_cb (FpiDeviceGoodixMoc  *self,
                       gxfp_cmd_response_t *resp,
                       GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->verify.match)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_DUPLICATE,
                                                     "Finger is too similar to another, try use a different finger"));
      // maybe need fpi_device_enroll_report_message ...
      return;
    }
  fpi_ssm_next_state (self->task_ssm);

}

static void
fp_enroll_init_cb (FpiDeviceGoodixMoc  *self,
                   gxfp_cmd_response_t *resp,
                   GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  memcpy (self->template_id, resp->enroll_init.tid, TEMPLATE_ID_SIZE);
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_capture_cb (FpiDeviceGoodixMoc  *self,
                      gxfp_cmd_response_t *resp,
                      GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  /* */
  if (resp->result >= GX_FAILED)
    {
      fp_warn ("Capture sample failed, result: 0x%x", resp->result);
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
      fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_CAPTURE);
      return;
    }
  fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                           FP_FINGER_STATUS_PRESENT,
                                           FP_FINGER_STATUS_NONE);
  if ((resp->capture_data_resp.img_quality < self->sensorcfg->config[4]) ||
      (resp->capture_data_resp.img_coverage < self->sensorcfg->config[5]))
    {
      fp_warn ("Capture sample poor quality(%d): %d or coverage(%d): %d",
               self->sensorcfg->config[4],
               resp->capture_data_resp.img_quality,
               self->sensorcfg->config[5],
               resp->capture_data_resp.img_coverage);
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_CENTER_FINGER));
      fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_CAPTURE);
      return;
    }
  if (self->is_enroll_identify)
    {
      self->is_enroll_identify = false;
      fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_IDENTIFY);
    }
  else
    {
      fpi_ssm_next_state (self->task_ssm);
    }

}
static void
fp_enroll_update_cb (FpiDeviceGoodixMoc  *self,
                     gxfp_cmd_response_t *resp,
                     GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  if (resp->enroll_update.img_preoverlay > self->sensorcfg->config[3])
    {
      fp_dbg ("Sample overlapping ratio is too High(%d): %d ",
              self->sensorcfg->config[3],
              resp->enroll_update.img_preoverlay);
      /* here should tips move finger and try again */
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_REMOVE_FINGER));
    }
  else if (resp->enroll_update.rollback)
    {
      fpi_device_enroll_progress (FP_DEVICE (self),
                                  self->enroll_stage,
                                  NULL,
                                  fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
    }
  else
    {
      self->enroll_stage++;
      fpi_device_enroll_progress (FP_DEVICE (self), self->enroll_stage, NULL, NULL);
    }
  /* if enroll complete, no need to wait finger up */
  if (self->enroll_stage >= self->max_enroll_stage)
    {
      fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_CHECK_DUPLICATE);
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_check_duplicate_cb (FpiDeviceGoodixMoc  *self,
                              gxfp_cmd_response_t *resp,
                              GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->check_duplicate_resp.duplicate)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_DATA_DUPLICATE,
                                                     "Finger has already enrolled"));
      return;
    }

  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_commit_cb (FpiDeviceGoodixMoc  *self,
                     gxfp_cmd_response_t *resp,
                     GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  if (resp->result >= GX_FAILED)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Commit template failed with errcode: 0x%x", resp->result));
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_finger_mode_cb (FpiDeviceGoodixMoc  *self,
                   gxfp_cmd_response_t *resp,
                   GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  /* if reach max timeout(5sec) finger not up, switch to finger up again */
  if (resp->finger_status.status == GX_ERROR_WAIT_FINGER_UP_TIMEOUT)
    {
      fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_WAIT_FINGER_UP);
      return;
    }
  else if (resp->finger_status.status != GX_SUCCESS)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                     "Switch finger mode failed"));
      return;
    }
  fpi_device_report_finger_status_changes (FP_DEVICE (self),
                                           FP_FINGER_STATUS_NONE,
                                           FP_FINGER_STATUS_PRESENT);
  if (self->enroll_stage < self->max_enroll_stage)
    {
      fpi_ssm_jump_to_state (self->task_ssm, FP_ENROLL_CAPTURE);
      return;
    }
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_enroll_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  FpPrint *print = NULL;
  GVariant *data = NULL;
  GVariant *uid = NULL;
  GVariant *tid = NULL;
  guint finger;
  guint16 user_id_len;
  guint16 payload_len = 0;
  g_autofree gchar *user_id = NULL;
  g_autofree guint8 *payload = NULL;
  guint8 dummy[3] = { 0 };

  dummy[1] = self->sensorcfg->config[4];
  dummy[2] = self->sensorcfg->config[5];

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FP_ENROLL_ENUM:
      {
        goodix_sensor_cmd (self, MOC_CMD0_GETFINGERLIST, MOC_CMD1_DEFAULT,
                           false,
                           (const guint8 *) &dummy,
                           1,
                           fp_enroll_enum_cb);
      }
      break;

    case FP_ENROLL_PWR_BTN_SHIELD_ON:
      {
        goodix_sensor_cmd (self, MOC_CMD0_PWR_BTN_SHIELD, MOC_CMD1_PWR_BTN_SHIELD_ON,
                           false,
                           NULL,
                           0,
                           fp_pwr_btn_shield_cb);
      }
      break;

    case FP_ENROLL_IDENTIFY:
      {
        dummy[0] = 0x01;
        dummy[1] = self->sensorcfg->config[10];
        dummy[2] = self->sensorcfg->config[11];
        goodix_sensor_cmd (self, MOC_CMD0_IDENTIFY, MOC_CMD1_DEFAULT,
                           false,
                           (const guint8 *) &self->template_id,
                           TEMPLATE_ID_SIZE,
                           fp_enroll_identify_cb);
      }
      break;

    case FP_ENROLL_CREATE:
      {
        goodix_sensor_cmd (self, MOC_CMD0_ENROLL_INIT, MOC_CMD1_DEFAULT,
                           false,
                           (const guint8 *) &dummy,
                           1,
                           fp_enroll_init_cb);
      }
      break;

    case FP_ENROLL_CAPTURE:
      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_NONE);
      goodix_sensor_cmd (self, MOC_CMD0_CAPTURE_DATA, MOC_CMD1_DEFAULT,
                         true,
                         (const guint8 *) &dummy,
                         3,
                         fp_enroll_capture_cb);
      break;

    case FP_ENROLL_UPDATE:
      dummy[0] = 1;
      dummy[1] = self->sensorcfg->config[2];
      dummy[2] = self->sensorcfg->config[3];
      goodix_sensor_cmd (self, MOC_CMD0_ENROLL, MOC_CMD1_DEFAULT,
                         false,
                         (const guint8 *) &dummy,
                         3,
                         fp_enroll_update_cb);
      break;

    case FP_ENROLL_WAIT_FINGER_UP:
      dummy[0] = 0;
      goodix_sensor_cmd (self, MOC_CMD0_FINGER_MODE, MOC_CMD1_SET_FINGER_UP,
                         true,
                         (const guint8 *) &dummy,
                         1,
                         fp_finger_mode_cb);
      break;

    case FP_ENROLL_CHECK_DUPLICATE:
      goodix_sensor_cmd (self, MOC_CMD0_CHECK4DUPLICATE, MOC_CMD1_DEFAULT,
                         false,
                         (const guint8 *) &dummy,
                         3,
                         fp_enroll_check_duplicate_cb);
      break;

    case FP_ENROLL_COMMIT:
      {
        fpi_device_get_enroll_data (device, &print);
        user_id = fpi_print_generate_user_id (print);
        user_id_len = strlen (user_id);
        user_id_len = MIN (100, user_id_len);
        finger = 1;

        if (g_strcmp0 (g_getenv ("FP_DEVICE_EMULATION"), "1") == 0)
          memset (self->template_id, 0, TEMPLATE_ID_SIZE);
        uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                         user_id,
                                         user_id_len,
                                         1);

        tid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                         self->template_id,
                                         TEMPLATE_ID_SIZE,
                                         1);

        data = g_variant_new ("(y@ay@ay)",
                              finger,
                              tid,
                              uid);

        fpi_print_set_type (print, FPI_PRINT_RAW);
        fpi_print_set_device_stored (print, TRUE);
        g_object_set (print, "fpi-data", data, NULL);
        g_object_set (print, "description", user_id, NULL);

        g_debug ("user_id: %s, user_id_len: %d, finger: %d", user_id, user_id_len, finger);

        if(!encode_finger_id (self->template_id,
                              TEMPLATE_ID_SIZE,
                              (guint8 *) user_id,
                              user_id_len,
                              &payload,
                              &payload_len))
          {
            fpi_ssm_mark_failed (ssm,
                                 fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                           "encode_finger_id failed"));
            return;
          }
        goodix_sensor_cmd (self, MOC_CMD0_COMMITENROLLMENT, MOC_CMD1_DEFAULT,
                           false,
                           (const guint8 *) payload,
                           payload_len,
                           fp_enroll_commit_cb);

      }
      break;

    case FP_ENROLL_PWR_BTN_SHIELD_OFF:
      {
        goodix_sensor_cmd (self, MOC_CMD0_PWR_BTN_SHIELD, MOC_CMD1_PWR_BTN_SHIELD_OFF,
                           false,
                           NULL,
                           0,
                           fp_pwr_btn_shield_cb);
      }
      break;
    }
}

static void
fp_enroll_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (dev);
  FpPrint *print = NULL;

  if (error)
    {
      fpi_device_enroll_complete (dev, NULL, error);
      return;
    }
  fp_info ("Enrollment complete!");

  fpi_device_get_enroll_data (FP_DEVICE (self), &print);

  fpi_device_enroll_complete (FP_DEVICE (self), g_object_ref (print), NULL);

  self->task_ssm = NULL;
}
/******************************************************************************
 *
 *  fp_init_xxxx Function
 *
 *****************************************************************************/
static void
fp_init_version_cb (FpiDeviceGoodixMoc  *self,
                    gxfp_cmd_response_t *resp,
                    GError              *error)
{
  g_autofree gchar *fw_type = NULL;
  g_autofree gchar *fw_version = NULL;

  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }

  G_STATIC_ASSERT (sizeof (resp->version_info.fwtype) == 8);
  G_STATIC_ASSERT (sizeof (resp->version_info.fwversion) == 8);

  fw_type = g_strndup ((const char *) resp->version_info.fwtype, sizeof (resp->version_info.fwtype));

  fp_info ("Firmware type: %s", fw_type);
  if (g_strcmp0 (fw_type, "APP") != 0)
    {
      fpi_ssm_mark_failed (self->task_ssm,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Please update firmware using fwupd"));
      return;
    }
  fw_version = g_strndup ((const char *) resp->version_info.fwversion, sizeof (resp->version_info.fwversion));
  fp_info ("Firmware version: %s", fw_version);
  fpi_ssm_next_state (self->task_ssm);
}

static void
fp_init_config_cb (FpiDeviceGoodixMoc  *self,
                   gxfp_cmd_response_t *resp,
                   GError              *error)
{
  if (error)
    {
      fpi_ssm_mark_failed (self->task_ssm, error);
      return;
    }
  self->max_stored_prints = resp->finger_config.max_stored_prints;
  fpi_ssm_next_state (self->task_ssm);
}


static void
fp_init_sm_run_state (FpiSsm *ssm, FpDevice *device)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  guint8 dummy = 0;

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case FP_INIT_VERSION:
      goodix_sensor_cmd (self, MOC_CMD0_GET_VERSION, MOC_CMD1_DEFAULT,
                         false,
                         &dummy,
                         1,
                         fp_init_version_cb);
      break;

    case FP_INIT_CONFIG:
      goodix_sensor_cmd (self, MOC_CMD0_UPDATE_CONFIG, MOC_CMD1_WRITE_CFG_TO_FLASH,
                         false,
                         (guint8 *) self->sensorcfg,
                         sizeof (gxfp_sensor_cfg_t),
                         fp_init_config_cb);
      break;
    }


}

static void
fp_init_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (dev);

  if (error)
    {
      fpi_device_open_complete (dev, error);
      return;
    }
  self->task_ssm = NULL;
  fpi_device_open_complete (dev, NULL);
}
/******************************************************************************
 *
 *  fp_template_delete Function
 *
 *****************************************************************************/
static gboolean
parse_print_data (GVariant      *data,
                  guint8        *finger,
                  const guint8 **tid,
                  gsize         *tid_len,
                  const guint8 **user_id,
                  gsize         *user_id_len)
{
  g_autoptr(GVariant) user_id_var = NULL;
  g_autoptr(GVariant) tid_var = NULL;

  g_return_val_if_fail (data != NULL, FALSE);
  g_return_val_if_fail (finger != NULL, FALSE);
  g_return_val_if_fail (tid != NULL, FALSE);
  g_return_val_if_fail (tid_len != NULL, FALSE);
  g_return_val_if_fail (user_id != NULL, FALSE);
  g_return_val_if_fail (user_id_len != NULL, FALSE);

  *tid = NULL;
  *tid_len = 0;
  *user_id = NULL;
  *user_id_len = 0;


  if (!g_variant_check_format_string (data, "(y@ay@ay)", FALSE))
    return FALSE;

  g_variant_get (data,
                 "(y@ay@ay)",
                 finger,
                 &tid_var,
                 &user_id_var);

  *tid     = g_variant_get_fixed_array (tid_var, tid_len, 1);
  *user_id = g_variant_get_fixed_array (user_id_var, user_id_len, 1);

  if (*user_id_len == 0 || *user_id_len > 100)
    return FALSE;

  if (*user_id_len <= 0 || *user_id[0] == ' ')
    return FALSE;

  if(*tid_len != TEMPLATE_ID_SIZE)
    return FALSE;

  return TRUE;
}

static void
fp_template_delete_cb (FpiDeviceGoodixMoc  *self,
                       gxfp_cmd_response_t *resp,
                       GError              *error)
{
  FpDevice *device = FP_DEVICE (self);

  if (error)
    {
      fpi_device_delete_complete (device, error);
      return;
    }
  if ((resp->result >= GX_FAILED) && (resp->result != GX_ERROR_FINGER_ID_NOEXIST))
    {
      fpi_device_delete_complete (FP_DEVICE (self),
                                  fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                            "Failed delete enrolled users, result: 0x%x",
                                                            resp->result));
      return;
    }

  fp_info ("Successfully deleted enrolled user");
  fpi_device_delete_complete (device, NULL);
}
/******************************************************************************
 *
 *  fp_template_list Function
 *
 *****************************************************************************/
static void
fp_template_list_cb (FpiDeviceGoodixMoc  *self,
                     gxfp_cmd_response_t *resp,
                     GError              *error)
{
  FpDevice *device = FP_DEVICE (self);

  if (error)
    {
      fpi_device_list_complete (FP_DEVICE (self), NULL, error);
      return;
    }
  if (resp->result != GX_SUCCESS)
    {
      fp_info ("Failed to query enrolled users: %d", resp->result);
      fpi_device_list_complete (FP_DEVICE (self),
                                NULL,
                                fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                          "Failed to query enrolled users, result: 0x%x",
                                                          resp->result));
      return;
    }

  self->list_result = g_ptr_array_new_with_free_func (g_object_unref);

  if(resp->finger_list_resp.finger_num == 0)
    {
      fp_info ("Database is empty");
      fpi_device_list_complete (device,
                                g_steal_pointer (&self->list_result),
                                NULL);
      return;
    }

  for (int n = 0; n < resp->finger_list_resp.finger_num; n++)
    {
      GVariant *data = NULL;
      GVariant *tid = NULL;
      GVariant *uid = NULL;
      FpPrint *print;
      gchar *userid;

      userid = (gchar *) resp->finger_list_resp.finger_list[n].payload.data;

      print = fp_print_new (FP_DEVICE (self));

      tid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                       resp->finger_list_resp.finger_list[n].tid,
                                       TEMPLATE_ID_SIZE,
                                       1);

      uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                       resp->finger_list_resp.finger_list[n].payload.data,
                                       resp->finger_list_resp.finger_list[n].payload.size,
                                       1);

      data = g_variant_new ("(y@ay@ay)",
                            resp->finger_list_resp.finger_list[n].finger_index,
                            tid,
                            uid);

      fpi_print_set_type (print, FPI_PRINT_RAW);
      fpi_print_set_device_stored (print, TRUE);
      g_object_set (print, "fpi-data", data, NULL);
      g_object_set (print, "description", userid, NULL);
      fpi_print_fill_from_user_id (print, userid);
      g_ptr_array_add (self->list_result, g_object_ref_sink (print));
    }

  fp_info ("Query complete!");
  fpi_device_list_complete (device,
                            g_steal_pointer (&self->list_result),
                            NULL);

}

/******************************************************************************
 *
 *  Interface Function
 *
 *****************************************************************************/
static void
gx_fp_probe (FpDevice *device)
{
  GUsbDevice *usb_dev;
  GError *error = NULL;
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  g_autofree gchar *serial = NULL;
  gint productid = 0;

  /* Claim usb interface */
  usb_dev = fpi_device_get_usb_device (device);
  if (!g_usb_device_open (usb_dev, &error))
    {
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  if (!g_usb_device_reset (usb_dev, &error))
    goto err_close;

  if (!g_usb_device_claim_interface (usb_dev, 0, 0, &error))
    goto err_close;

  if (g_strcmp0 (g_getenv ("FP_DEVICE_EMULATION"), "1") == 0)
    {

      serial = g_strdup ("emulated-device");
    }
  else
    {
      serial = g_usb_device_get_string_descriptor (usb_dev,
                                                   g_usb_device_get_serial_number_index (usb_dev),
                                                   &error);

      if (serial && !g_str_has_suffix (serial, "B0"))
        fp_warn ("Device with serial %s not supported", serial);
      if (error)
        {
          g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (device)),
                                          0, 0, NULL);
          goto err_close;
        }
    }
  productid = g_usb_device_get_pid (usb_dev);
  switch (productid)
    {
    case 0x6496:
    case 0x60A2:
    case 0x609C:
    case 0x639C:
    case 0x63AC:
    case 0x6A94:
      self->max_enroll_stage = 12;
      break;

    default:
      self->max_enroll_stage = DEFAULT_ENROLL_SAMPLES;
      break;
    }

  fpi_device_set_nr_enroll_stages (device, self->max_enroll_stage);

  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, serial, NULL, error);
  return;
err_close:

  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, NULL, NULL, error);

}

static void
gx_fp_init (FpDevice *device)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  GError *error = NULL;
  int ret = 0;

  self->max_stored_prints = FP_MAX_FINGERNUM;
  self->is_power_button_shield_on = false;

  self->sensorcfg = g_new0 (gxfp_sensor_cfg_t, 1);

  ret = gx_proto_init_sensor_config (self->sensorcfg);
  if (ret != 0)
    {
      error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL, "Init sensor failed");
      fpi_device_open_complete (FP_DEVICE (self), error);
      return;
    }
  self->sensorcfg->config[6] = self->max_enroll_stage;

  if (!g_usb_device_reset (fpi_device_get_usb_device (device), &error))
    {
      fpi_device_open_complete (FP_DEVICE (self), error);
      return;
    }

  /* Claim usb interface */
  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (device), 0, 0, &error))
    {
      fpi_device_open_complete (FP_DEVICE (self), error);
      return;
    }

  self->task_ssm = fpi_ssm_new (device, fp_init_sm_run_state,
                                FP_INIT_NUM_STATES);

  fpi_ssm_start (self->task_ssm, fp_init_ssm_done);

}

static void
gx_fp_release_interface (FpiDeviceGoodixMoc *self,
                         GError             *error)
{
  g_autoptr(GError) release_error = NULL;

  g_clear_pointer (&self->sensorcfg, g_free);

  /* Release usb interface */
  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (self)),
                                  0, 0, &release_error);
  /* Retain passed error if set, otherwise propagate error from release. */
  if (error == NULL)
    error = g_steal_pointer (&release_error);

  /* Notify close complete */
  fpi_device_close_complete (FP_DEVICE (self), error);

}

static void
gx_fp_exit_cb (FpiDeviceGoodixMoc  *self,
               gxfp_cmd_response_t *resp,
               GError              *error)
{


  if (resp->result >= GX_FAILED)
    fp_dbg ("Setting power button shield failed, result: 0x%x", resp->result);
  self->is_power_button_shield_on = false;
  gx_fp_release_interface (self, error);
}

static void
gx_fp_exit (FpDevice *device)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);

  if (self->is_power_button_shield_on)
    {
      goodix_sensor_cmd (self,
                         MOC_CMD0_PWR_BTN_SHIELD,
                         MOC_CMD1_PWR_BTN_SHIELD_OFF,
                         false,
                         NULL,
                         0,
                         gx_fp_exit_cb);
    }
  else
    {
      gx_fp_release_interface (self, NULL);
    }

}


static void
gx_fp_verify_identify (FpDevice *device)
{
  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);

  self->task_ssm = fpi_ssm_new_full (device, fp_verify_sm_run_state,
                                     FP_VERIFY_NUM_STATES,
                                     FP_VERIFY_PWR_BTN_SHIELD_OFF,
                                     "verify");

  fpi_ssm_start (self->task_ssm, fp_verify_ssm_done);

}

static void
gx_fp_enroll (FpDevice *device)
{

  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);

  self->enroll_stage = 0;
  self->is_enroll_identify = true;

  self->task_ssm = fpi_ssm_new_full (device, fp_enroll_sm_run_state,
                                     FP_ENROLL_NUM_STATES,
                                     FP_ENROLL_PWR_BTN_SHIELD_OFF,
                                     "enroll");

  fpi_ssm_start (self->task_ssm, fp_enroll_ssm_done);

}


static void
gx_fp_template_list (FpDevice *device)
{

  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  guint8 dummy[1] = { 0 };

  G_DEBUG_HERE ();

  goodix_sensor_cmd (self, MOC_CMD0_GETFINGERLIST, MOC_CMD1_DEFAULT,
                     false,
                     (const guint8 *) &dummy,
                     1,
                     fp_template_list_cb);
}



static void
gx_fp_template_delete (FpDevice *device)
{

  FpiDeviceGoodixMoc *self = FPI_DEVICE_GOODIXMOC (device);
  FpPrint *print = NULL;

  g_autoptr(GVariant) data = NULL;
  guint8 finger;
  const guint8 *user_id;
  gsize user_id_len = 0;
  const guint8 *tid;
  gsize tid_len = 0;
  gsize payload_len = 0;
  g_autofree guint8 *payload = NULL;

  fpi_device_get_delete_data (device, &print);

  g_object_get (print, "fpi-data", &data, NULL);

  if (!parse_print_data (data, &finger, &tid, &tid_len, &user_id, &user_id_len))
    {
      fpi_device_delete_complete (device,
                                  fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }
  if (!encode_finger_id (tid, tid_len, user_id, user_id_len, &payload, (guint16 *) &payload_len))
    {
      fpi_device_delete_complete (device,
                                  fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                            "encode_finger_id failed"));
      return;
    }

  goodix_sensor_cmd (self, MOC_CMD0_DELETETEMPLATE, MOC_CMD1_DEFAULT,
                     false,
                     (const guint8 *) payload,
                     payload_len,
                     fp_template_delete_cb);

}

static void
fpi_device_goodixmoc_init (FpiDeviceGoodixMoc *self)
{

}

static const FpIdEntry id_table[] = {
  { .vid = 0x27c6,  .pid = 0x5840,  },
  { .vid = 0x27c6,  .pid = 0x609C,  },
  { .vid = 0x27c6,  .pid = 0x60A2,  },
  { .vid = 0x27c6,  .pid = 0x639C,  },
  { .vid = 0x27c6,  .pid = 0x63AC,  },
  { .vid = 0x27c6,  .pid = 0x6496,  },
  { .vid = 0x27c6,  .pid = 0x6584,  },
  { .vid = 0x27c6,  .pid = 0x658C,  },
  { .vid = 0x27c6,  .pid = 0x6592,  },
  { .vid = 0x27c6,  .pid = 0x6594,  },
  { .vid = 0x27c6,  .pid = 0x659C,  },
  { .vid = 0x27c6,  .pid = 0x6A94,  },
  { .vid = 0,  .pid = 0,  .driver_data = 0 },   /* terminating entry */
};


static void
fpi_device_goodixmoc_class_init (FpiDeviceGoodixMocClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = "goodixmoc";
  dev_class->full_name = "Goodix MOC Fingerprint Sensor";
  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = DEFAULT_ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1;

  dev_class->open   = gx_fp_init;
  dev_class->close  = gx_fp_exit;
  dev_class->probe  = gx_fp_probe;
  dev_class->enroll = gx_fp_enroll;
  dev_class->delete = gx_fp_template_delete;
  dev_class->list   = gx_fp_template_list;
  dev_class->verify   = gx_fp_verify_identify;
  dev_class->identify = gx_fp_verify_identify;

  fpi_device_class_auto_initialize_features (dev_class);
  dev_class->features |= FP_DEVICE_FEATURE_DUPLICATES_CHECK;
}
