/*
 * Copyright (C) 2019 Synaptics Inc
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

#define FP_COMPONENT "synaptics"

#include "drivers_api.h"

#include "fpi-byte-reader.h"

#include "synaptics.h"
#include "bmkt_message.h"

G_DEFINE_TYPE (FpiDeviceSynaptics, fpi_device_synaptics, FP_TYPE_DEVICE)

static void init_identify_msg (FpDevice *device);
static void compose_and_send_identify_msg (FpDevice *device);

static const FpIdEntry id_table[] = {
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00BD,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00DF,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00F9,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00FC,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00C2,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00C9,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x0100,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x00F0,  },
  { .vid = SYNAPTICS_VENDOR_ID,  .pid = 0x0103,  },
  { .vid = 0,  .pid = 0,  .driver_data = 0 },   /* terminating entry */
};


static void
cmd_receive_cb (FpiUsbTransfer *transfer,
                FpDevice       *device,
                gpointer        user_data,
                GError         *error)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  SynCmdMsgCallback callback = user_data;
  int res;
  bmkt_msg_resp_t msg_resp;
  bmkt_response_t resp;

  if (error)
    {
      /* NOTE: assumes timeout should never happen for receiving. */
      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  res = bmkt_parse_message_header (&transfer->buffer[SENSOR_FW_REPLY_HEADER_LEN],
                                   transfer->actual_length - SENSOR_FW_REPLY_HEADER_LEN,
                                   &msg_resp);
  if (res != BMKT_SUCCESS)
    {
      g_warning ("Corrupted message received");
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }

  /* Special case events */
  if (msg_resp.msg_id == BMKT_EVT_FINGER_REPORT)
    {
      if (msg_resp.payload_len != 1)
        {
          g_warning ("Corrupted finger report received");
          fpi_ssm_mark_failed (transfer->ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
          return;
        }

      if (msg_resp.payload[0] == 0x01)
        {
          self->finger_on_sensor = TRUE;
          fpi_device_report_finger_status_changes (device,
                                                   FP_FINGER_STATUS_PRESENT,
                                                   FP_FINGER_STATUS_NONE);
        }
      else
        {
          self->finger_on_sensor = FALSE;
          fpi_device_report_finger_status_changes (device,
                                                   FP_FINGER_STATUS_NONE,
                                                   FP_FINGER_STATUS_PRESENT);

          if (self->cmd_complete_on_removal)
            {
              fpi_ssm_mark_completed (transfer->ssm);
              return;
            }
        }

      fp_dbg ("Finger is now %s the sensor", self->finger_on_sensor ? "on" : "off");

      /* XXX: Call callback!?! */
    }

  res = bmkt_parse_message_payload (&msg_resp, &resp);
  if (res != BMKT_SUCCESS)
    {
      g_warning ("Could not parse message payload: %i", res);
      fpi_ssm_mark_failed (transfer->ssm,
                           fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
      return;
    }

  /* Special cancellation handling */
  if (resp.response_id == BMKT_RSP_CANCEL_OP_OK || resp.response_id == BMKT_RSP_CANCEL_OP_FAIL)
    {
      if (resp.response_id == BMKT_RSP_CANCEL_OP_OK)
        {
          fp_dbg ("Received cancellation success response");
          fpi_ssm_mark_failed (transfer->ssm,
                               g_error_new_literal (G_IO_ERROR,
                                                    G_IO_ERROR_CANCELLED,
                                                    "Device reported cancellation of operation"));
        }
      else
        {
          fp_dbg ("Cancellation failed, this should not happen");
          fpi_ssm_mark_failed (transfer->ssm,
                               fpi_device_error_new (FP_DEVICE_ERROR_PROTO));
        }
      return;
    }

  if (msg_resp.seq_num == 0)
    {
      /* XXX: Should we really abort the command on general error?
       *      The original code did not! */
      if (msg_resp.msg_id == BMKT_RSP_GENERAL_ERROR)
        {
          guint16 err;

          /* XXX: It is weird that this is big endian. */
          err = FP_READ_UINT16_BE (msg_resp.payload);

          fp_warn ("Received General Error %d from the sensor", (guint) err);
          fpi_ssm_mark_failed (transfer->ssm,
                               fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                         "Received general error %u from device",
                                                         (guint) err));
          //fpi_ssm_jump_to_state (transfer->ssm, fpi_ssm_get_cur_state (transfer->ssm));
          return;
        }
      else
        {
          fp_dbg ("Received message with 0 sequence number 0x%02x, ignoring!",
                  msg_resp.msg_id);
          fpi_ssm_next_state (transfer->ssm);
          return;
        }
    }

  /* We should only ever have one command running, and the sequence num needs
   * to match. */
  if (msg_resp.seq_num != self->cmd_seq_num)
    {
      fp_warn ("Got unexpected sequence number from device, %d instead of %d",
               msg_resp.seq_num,
               self->cmd_seq_num);
    }

  if (callback)
    callback (self, &resp, NULL);

  /* Callback may have queued a follow up command, then we need
   * to restart the SSM. If not, we'll finish/wait for interrupt
   * depending on resp.complete. */
  if (self->cmd_pending_transfer)
    fpi_ssm_jump_to_state (transfer->ssm, SYNAPTICS_CMD_SEND_PENDING);
  else if (!resp.complete || self->cmd_complete_on_removal)
    fpi_ssm_next_state (transfer->ssm);             /* SYNAPTICS_CMD_WAIT_INTERRUPT */
  else
    fpi_ssm_mark_completed (transfer->ssm);
}

static void
cmd_interrupt_cb (FpiUsbTransfer *transfer,
                  FpDevice       *device,
                  gpointer        user_data,
                  GError         *error)
{
  g_debug ("interrupt transfer done");
  if (error)
    {
      if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        {
          g_error_free (error);
          fpi_ssm_jump_to_state (transfer->ssm, SYNAPTICS_CMD_GET_RESP);
          return;
        }

      fpi_ssm_mark_failed (transfer->ssm, error);
      return;
    }

  if (transfer->buffer[0] & USB_ASYNC_MESSAGE_PENDING)
    {
      fpi_ssm_next_state (transfer->ssm);
    }
  else
    {
      fpi_usb_transfer_submit (fpi_usb_transfer_ref (transfer),
                               0,
                               NULL,
                               cmd_interrupt_cb,
                               NULL);
    }
}

static void
synaptics_cmd_run_state (FpiSsm   *ssm,
                         FpDevice *dev)
{
  FpiUsbTransfer *transfer;
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (dev);

  switch (fpi_ssm_get_cur_state (ssm))
    {
    case SYNAPTICS_CMD_SEND_PENDING:
      if (self->cmd_pending_transfer)
        {
          self->cmd_pending_transfer->ssm = ssm;
          fpi_usb_transfer_submit (self->cmd_pending_transfer,
                                   1000,
                                   NULL,
                                   fpi_ssm_usb_transfer_cb,
                                   NULL);
          self->cmd_pending_transfer = NULL;
        }
      else
        {
          fpi_ssm_next_state (ssm);
        }
      break;

    case SYNAPTICS_CMD_GET_RESP:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, USB_EP_REPLY, MAX_TRANSFER_LEN);
      fpi_usb_transfer_submit (transfer,
                               5000,
                               NULL,
                               cmd_receive_cb,
                               fpi_ssm_get_data (ssm));

      break;

    case SYNAPTICS_CMD_WAIT_INTERRUPT:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_interrupt (transfer, USB_EP_INTERRUPT, USB_INTERRUPT_DATA_SIZE);
      fpi_usb_transfer_submit (transfer,
                               0,
                               self->interrupt_cancellable,
                               cmd_interrupt_cb,
                               NULL);

      break;

    case SYNAPTICS_CMD_SEND_ASYNC:
      transfer = fpi_usb_transfer_new (dev);
      transfer->ssm = ssm;
      fpi_usb_transfer_fill_bulk (transfer, USB_EP_REQUEST, SENSOR_FW_CMD_HEADER_LEN);
      transfer->buffer[0] = SENSOR_CMD_ASYNCMSG_READ;
      fpi_usb_transfer_submit (transfer,
                               1000,
                               NULL,
                               fpi_ssm_usb_transfer_cb,
                               NULL);

      break;

    case SYNAPTICS_CMD_RESTART:
      fpi_ssm_jump_to_state (ssm, SYNAPTICS_CMD_SEND_PENDING);
      break;
    }
}

static void
cmd_ssm_done (FpiSsm *ssm, FpDevice *dev, GError *error)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (dev);
  SynCmdMsgCallback callback = fpi_ssm_get_data (ssm);

  self->cmd_ssm = NULL;

  /* Notify about the SSM failure from here instead. */
  if (error || self->cmd_complete_on_removal)
    callback (self, NULL, error);

  self->cmd_complete_on_removal = FALSE;
}

static void
cmd_forget_cb (FpiUsbTransfer *transfer,
               FpDevice       *device,
               gpointer        user_data,
               GError         *error)
{
  if (error)
    {
      g_warning ("Async command sending failed: %s", error->message);
      g_error_free (error);
    }
  else
    {
      g_debug ("Async command sent successfully");
    }
}

static void
synaptics_sensor_cmd (FpiDeviceSynaptics *self,
                      gint                seq_num,
                      guint8              msg_id,
                      const guint8      * payload,
                      gssize              payload_len,
                      SynCmdMsgCallback   callback)
{
  FpiUsbTransfer *transfer;
  guint8 real_seq_num;
  gint msg_len;
  gint res;

  /* callback may be NULL in two cases:
   *  - seq_num == -1
   *  - a state machine is already running, continued command */
  g_assert (payload || payload_len == 0);

  /* seq_num of 0 means a normal command, -1 means the current commands
   * sequence number should not be updated (i.e. second async command which
   * may only be a cancellation currently). */
  if (seq_num <= 0)
    {
      self->last_seq_num = MAX (1, (self->last_seq_num + 1) & 0xff);
      real_seq_num = self->last_seq_num;
      if (seq_num == 0)
        self->cmd_seq_num = self->last_seq_num;
    }
  else
    {
      real_seq_num = seq_num;
      self->last_seq_num = real_seq_num;
    }
  g_debug ("sequence number is %d", real_seq_num);

  /* We calculate the exact length here (we could also just create a larger
   * buffer instead and check the result of bmkt_compose_message. */
  msg_len = BMKT_MESSAGE_HEADER_LEN + payload_len;

  /* Send out the command */
  transfer = fpi_usb_transfer_new (FP_DEVICE (self));
  transfer->short_is_error = TRUE;
  fpi_usb_transfer_fill_bulk (transfer,
                              USB_EP_REQUEST,
                              msg_len + SENSOR_FW_CMD_HEADER_LEN);

  /* MIS sensors send ACE commands encapsulated in FW commands*/
  transfer->buffer[0] = SENSOR_CMD_ACE_COMMAND;
  res = bmkt_compose_message (&transfer->buffer[1],
                              &msg_len, msg_id,
                              real_seq_num,
                              payload_len,
                              payload);
  g_assert (res == BMKT_SUCCESS);
  g_assert (msg_len + SENSOR_FW_CMD_HEADER_LEN == transfer->length);

  /* Special case for async command sending (should only be used for
   * cancellation). */
  if (seq_num == -1)
    {
      g_assert (callback == NULL);

      /* We just send and forget here. */
      fpi_usb_transfer_submit (transfer, 1000, NULL, cmd_forget_cb, NULL);
    }
  else
    {
      /* Command should be send using the state machine. */
      g_assert (self->cmd_pending_transfer == NULL);

      self->cmd_pending_transfer = g_steal_pointer (&transfer);

      if (self->cmd_ssm)
        {
          /* Continued command, we already have an SSM with a callback.
           * There is nothing to do in this case, the command will be
           * sent automatically. */
          g_assert (callback == NULL);
        }
      else
        {
          /* Start of a new command, create the state machine. */
          g_assert (callback != NULL);

          self->cmd_ssm = fpi_ssm_new (FP_DEVICE (self),
                                       synaptics_cmd_run_state,
                                       SYNAPTICS_CMD_NUM_STATES);
          fpi_ssm_set_data (self->cmd_ssm, callback, NULL);

          fpi_ssm_start (self->cmd_ssm, cmd_ssm_done);
        }
    }
}

static gboolean
parse_print_data (GVariant      *data,
                  guint8        *finger,
                  const guint8 **user_id,
                  gsize         *user_id_len)
{
  g_autoptr(GVariant) user_id_var = NULL;

  g_return_val_if_fail (data != NULL, FALSE);
  g_return_val_if_fail (finger != NULL, FALSE);
  g_return_val_if_fail (user_id != NULL, FALSE);
  g_return_val_if_fail (user_id_len != NULL, FALSE);

  *user_id = NULL;
  *user_id_len = 0;

  if (!g_variant_check_format_string (data, "(y@ay)", FALSE))
    return FALSE;

  g_variant_get (data,
                 "(y@ay)",
                 finger,
                 &user_id_var);

  *user_id = g_variant_get_fixed_array (user_id_var, user_id_len, 1);

  if (*user_id_len == 0 || *user_id_len > BMKT_MAX_USER_ID_LEN)
    return FALSE;

  if (*user_id_len <= 0 || *user_id[0] == ' ')
    return FALSE;

  return TRUE;
}

static FpPrint *
create_print (FpiDeviceSynaptics *self,
              guint8             *user_id,
              guint8              finger_id)
{
  FpPrint *print;
  g_autofree gchar *user_id_safe;
  GVariant *data = NULL;
  GVariant *uid = NULL;

  user_id_safe = g_strndup ((char *) user_id, BMKT_MAX_USER_ID_LEN);

  print = fp_print_new (FP_DEVICE (self));
  uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                   user_id_safe,
                                   strlen (user_id_safe),
                                   1);
  data = g_variant_new ("(y@ay)",
                        finger_id,
                        uid);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, TRUE);
  g_object_set (print, "fpi-data", data, NULL);
  g_object_set (print, "description", user_id_safe, NULL);

  fpi_print_fill_from_user_id (print, user_id_safe);

  return print;
}

static void
verify_complete_after_finger_removal (FpiDeviceSynaptics *self)
{
  FpDevice *device = FP_DEVICE (self);

  if (self->finger_on_sensor)
    {
      fp_dbg ("delaying verify report until after finger removal!");
      self->cmd_complete_on_removal = TRUE;
    }
  else
    {
      fpi_device_verify_complete (device, NULL);
    }
}

static void
verify_msg_cb (FpiDeviceSynaptics *self,
               bmkt_response_t    *resp,
               GError             *error)
{
  FpDevice *device = FP_DEVICE (self);
  bmkt_verify_resp_t *verify_resp;

  if (error)
    {
      fpi_device_verify_complete (device, error);
      return;
    }

  if (resp == NULL && self->cmd_complete_on_removal)
    {
      fpi_device_verify_complete (device, NULL);
      return;
    }

  g_assert (resp != NULL);

  verify_resp = &resp->response.verify_resp;

  switch (resp->response_id)
    {
    case BMKT_RSP_VERIFY_READY:
      fpi_device_report_finger_status_changes (device,
                                               FP_FINGER_STATUS_NEEDED,
                                               FP_FINGER_STATUS_NONE);
      fp_info ("Place Finger on the Sensor!");
      break;

    case BMKT_RSP_CAPTURE_COMPLETE:
      fp_info ("Fingerprint image capture complete!");
      break;

    case BMKT_RSP_VERIFY_FAIL:
      if (resp->result == BMKT_SENSOR_STIMULUS_ERROR)
        {
          fp_info ("Match error occurred");
          fpi_device_verify_report (device, FPI_MATCH_ERROR, NULL,
                                    fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
          verify_complete_after_finger_removal (self);
        }
      else if (resp->result == BMKT_FP_NO_MATCH)
        {
          fp_info ("Print didn't match");
          fpi_device_verify_report (device, FPI_MATCH_FAIL, NULL, NULL);
          verify_complete_after_finger_removal (self);
        }
      else if (resp->result == BMKT_FP_DATABASE_NO_RECORD_EXISTS)
        {
          fp_info ("Print is not in database");
          fpi_device_verify_complete (device,
                                      fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
        }
      else
        {
          fp_warn ("Verify has failed: %d", resp->result);
          fpi_device_verify_complete (device,
                                      fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                                "Unexpected result from device %d",
                                                                resp->result));
        }
      break;

    case BMKT_RSP_VERIFY_OK:
      fp_info ("Verify was successful! for user: %s finger: %d score: %f",
               verify_resp->user_id, verify_resp->finger_id, verify_resp->match_result);
      fpi_device_verify_report (device, FPI_MATCH_SUCCESS, NULL, NULL);
      verify_complete_after_finger_removal (self);
      break;
    }
}

static void
verify (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  FpPrint *print = NULL;

  g_autoptr(GVariant) data = NULL;
  guint8 finger;
  const guint8 *user_id;
  gsize user_id_len = 0;

  fpi_device_get_verify_data (device, &print);

  g_object_get (print, "fpi-data", &data, NULL);
  g_debug ("data is %p", data);
  if (!parse_print_data (data, &finger, &user_id, &user_id_len))
    {
      fpi_device_verify_complete (device,
                                  fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  G_DEBUG_HERE ();

  synaptics_sensor_cmd (self, 0, BMKT_CMD_VERIFY_USER, user_id, user_id_len, verify_msg_cb);
}

static void
identify_complete_after_finger_removal (FpiDeviceSynaptics *self)
{
  FpDevice *device = FP_DEVICE (self);

  if (self->finger_on_sensor)
    {
      fp_dbg ("delaying identify report until after finger removal!");
      self->cmd_complete_on_removal = TRUE;
    }
  else
    {
      fpi_device_identify_complete (device, NULL);
    }
}


static void
identify_msg_cb (FpiDeviceSynaptics *self,
                 bmkt_response_t    *resp,
                 GError             *error)
{
  FpDevice *device = FP_DEVICE (self);

  if (error)
    {
      fpi_device_identify_complete (device, error);
      return;
    }

  if (resp == NULL && self->cmd_complete_on_removal)
    {
      fpi_device_identify_complete (device, NULL);
      return;
    }

  g_assert (resp != NULL);

  switch (resp->response_id)
    {
    case BMKT_RSP_ID_READY:
      fp_info ("Place Finger on the Sensor!");
      break;

    case BMKT_RSP_SEND_NEXT_USER_ID:
      {
        compose_and_send_identify_msg (device);
        break;
      }

    case BMKT_RSP_ID_FAIL:
      if (resp->result == BMKT_SENSOR_STIMULUS_ERROR)
        {
          fp_info ("Match error occurred");
          fpi_device_identify_report (device, NULL, NULL,
                                      fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
          identify_complete_after_finger_removal (self);
        }
      else if (resp->result == BMKT_FP_NO_MATCH)
        {
          fp_info ("Print didn't match");
          fpi_device_identify_report (device, NULL, NULL, NULL);
          identify_complete_after_finger_removal (self);
        }
      else if (resp->result == BMKT_FP_DATABASE_NO_RECORD_EXISTS)
        {
          fp_info ("Print is not in database");
          fpi_device_identify_complete (device,
                                        fpi_device_error_new (FP_DEVICE_ERROR_DATA_NOT_FOUND));
        }
      else
        {
          fp_warn ("identify has failed: %d", resp->result);
          fpi_device_identify_complete (device,
                                        fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                                  "Unexpected result from device %d",
                                                                  resp->result));
        }
      break;

    case BMKT_RSP_ID_OK:
      {
        FpPrint *print = NULL;
        GPtrArray *prints = NULL;
        g_autoptr(GVariant) data = NULL;
        gboolean found = FALSE;
        guint index;

        print = create_print (self,
                              resp->response.id_resp.user_id,
                              resp->response.id_resp.finger_id);

        fpi_device_get_identify_data (device, &prints);

        found = g_ptr_array_find_with_equal_func (prints,
                                                  print,
                                                  (GEqualFunc) fp_print_equal,
                                                  &index);

        if (found)
          fpi_device_identify_report (device, g_ptr_array_index (prints, index), print, NULL);
        else
          fpi_device_identify_report (device, NULL, print, NULL);

        identify_complete_after_finger_removal (self);
      }
    }
}

static void
identify (FpDevice *device)
{
  init_identify_msg (device);
  compose_and_send_identify_msg (device);
}

static void
init_identify_msg (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);

  self->id_idx = 0;
}

static void
compose_and_send_identify_msg (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  FpPrint *print = NULL;
  GPtrArray *prints = NULL;

  g_autoptr(GVariant) data = NULL;
  guint8 finger;
  const guint8 *user_id;
  gsize user_id_len = 0;
  g_autofree guint8 *payload = NULL;
  guint8 payload_len = 0;
  guint8 payloadOffset = 0;

  fpi_device_get_identify_data (device, &prints);
  if (prints->len > UINT8_MAX)
    {
      fpi_device_identify_complete (device,
                                    fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }
  if(self->id_idx >= prints->len)
    {
      fp_warn ("Device asked for more prints than we are providing.");
      fpi_device_identify_complete (device,
                                    fpi_device_error_new_msg (FP_DEVICE_ERROR_PROTO,
                                                              "Unexpected index"));
      return;
    }
  print = g_ptr_array_index (prints, self->id_idx);
  g_object_get (print, "fpi-data", &data, NULL);
  g_debug ("data is %p", data);
  if (!parse_print_data (data, &finger, &user_id, &user_id_len))
    {
      fpi_device_identify_complete (device,
                                    fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }
  if(self->id_idx == 0)
    {
      /*
       * Construct payload.
       * 1st byte is total number of IDs in list.
       * 2nd byte is number of IDs in list.
       * 1 byte for each ID length, maximum id length is 100.
       * user_id_len bytes of each ID
       */
      payload_len = 2 + 1 + user_id_len;
      payload = g_malloc0 (payload_len);
      payload[payloadOffset] = prints->len;
      payloadOffset += 1;
      payload[payloadOffset] = 1; /* send one id per message */
      payloadOffset += 1;
      payload[payloadOffset] = user_id_len;
      payloadOffset += 1;
      memcpy (&payload[payloadOffset], user_id, user_id_len);
      payloadOffset += user_id_len;

      G_DEBUG_HERE ();

      synaptics_sensor_cmd (self, 0, BMKT_CMD_ID_USER_IN_ORDER, payload, payloadOffset, identify_msg_cb);
    }
  else
    {
      /*
       * 1st byte is the number of IDs
       * 1 byte for each ID length
       * id_length bytes for each ID
       */
      payload_len = 1 + 1 + user_id_len;
      payload = g_malloc0 (payload_len);
      payload[payloadOffset] = 1; /* send one id per message */
      payloadOffset += 1;
      payload[payloadOffset] = user_id_len;
      payloadOffset += 1;
      memcpy (&payload[payloadOffset], user_id, user_id_len);
      payloadOffset += user_id_len;
      synaptics_sensor_cmd (self, self->cmd_seq_num, BMKT_CMD_ID_NEXT_USER, payload, payloadOffset, NULL);
    }
  self->id_idx++;
}
static void
enroll_msg_cb (FpiDeviceSynaptics *self,
               bmkt_response_t    *resp,
               GError             *error)
{
  FpDevice *device = FP_DEVICE (self);
  bmkt_enroll_resp_t *enroll_resp;

  if (error)
    {
      fpi_device_enroll_complete (device, NULL, error);
      return;
    }

  enroll_resp = &resp->response.enroll_resp;

  switch (resp->response_id)
    {
    case BMKT_RSP_ENROLL_READY:
      {
        self->enroll_stage = 0;
        fpi_device_report_finger_status_changes (device,
                                                 FP_FINGER_STATUS_NEEDED,
                                                 FP_FINGER_STATUS_NONE);
        fp_info ("Place Finger on the Sensor!");
        break;
      }

    case BMKT_RSP_CAPTURE_COMPLETE:
      {
        fp_info ("Fingerprint image capture complete!");
        break;
      }

    case BMKT_RSP_ENROLL_REPORT:
      {
        gint done_stages;
        fp_info ("Enrollment is %d %% ", enroll_resp->progress);

        done_stages = (enroll_resp->progress * ENROLL_SAMPLES + 99) / 100;
        if (enroll_resp->progress < 100)
          done_stages = MIN (done_stages, ENROLL_SAMPLES - 1);

        /* Emit a retry error if there has been no discernible
         * progress. Some firmware revisions report more required
         * touches. */
        if (self->enroll_stage == done_stages)
          {
            fpi_device_enroll_progress (device,
                                        done_stages,
                                        NULL,
                                        fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL));
          }

        while (self->enroll_stage < done_stages)
          {
            self->enroll_stage += 1;
            fpi_device_enroll_progress (device, self->enroll_stage, NULL, NULL);
          }
        break;
      }

    case BMKT_RSP_ENROLL_PAUSED:
      {
        fp_info ("Enrollment has been paused!");
        break;
      }

    case BMKT_RSP_ENROLL_RESUMED:
      {
        fp_info ("Enrollment has been resumed!");
        break;
      }

    case BMKT_RSP_ENROLL_FAIL:
      {
        fp_info ("Enrollment has failed!: %d", resp->result);
        if (resp->result == BMKT_FP_DATABASE_FULL)
          {
            fpi_device_enroll_complete (device,
                                        NULL,
                                        fpi_device_error_new (FP_DEVICE_ERROR_DATA_FULL));
          }
        else
          {
            fpi_device_enroll_complete (device,
                                        NULL,
                                        fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                                                  "Enrollment failed (%d)",
                                                                  resp->result));
          }
        break;
      }

    case BMKT_RSP_ENROLL_OK:
      {
        FpPrint *print = NULL;

        fp_info ("Enrollment was successful!");

        fpi_device_get_enroll_data (device, &print);

        fpi_device_enroll_complete (device, g_object_ref (print), NULL);
        break;
      }
    }
}

static void
enroll (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  FpPrint *print = NULL;
  GVariant *data = NULL;
  GVariant *uid = NULL;
  guint finger;
  g_autofree gchar *user_id = NULL;
  gssize user_id_len;
  g_autofree guint8 *payload = NULL;

  fpi_device_get_enroll_data (device, &print);

  G_DEBUG_HERE ();

  user_id = fpi_print_generate_user_id (print);

  user_id_len = strlen (user_id);
  user_id_len = MIN (BMKT_MAX_USER_ID_LEN, user_id_len);

  /* We currently always use finger 1 from the devices point of view */
  finger = 1;

  uid = g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                   user_id,
                                   user_id_len,
                                   1);
  data = g_variant_new ("(y@ay)",
                        finger,
                        uid);

  fpi_print_set_type (print, FPI_PRINT_RAW);
  fpi_print_set_device_stored (print, TRUE);
  g_object_set (print, "fpi-data", data, NULL);
  g_object_set (print, "description", user_id, NULL);

  g_debug ("user_id: %s, finger: %d", user_id, finger);

  payload = g_malloc0 (user_id_len + 2);

  /* Backup options are not supported for Prometheus */
  payload[0] = 0;
  payload[1] = finger;
  memcpy (payload + 2, user_id, user_id_len);

  synaptics_sensor_cmd (self, 0, BMKT_CMD_ENROLL_USER, payload, user_id_len + 2, enroll_msg_cb);
}

static void
delete_msg_cb (FpiDeviceSynaptics *self,
               bmkt_response_t    *resp,
               GError             *error)
{
  FpDevice *device = FP_DEVICE (self);
  bmkt_del_user_resp_t *del_user_resp;

  if (error)
    {
      fpi_device_delete_complete (device, error);
      return;
    }

  del_user_resp = &resp->response.del_user_resp;

  switch (resp->response_id)
    {
    case BMKT_RSP_DELETE_PROGRESS:
      fp_info ("Deleting Enrolled Users is %d%% complete",
               del_user_resp->progress);
      break;

    case BMKT_RSP_DEL_USER_FP_FAIL:
      if (resp->result == BMKT_FP_DATABASE_NO_RECORD_EXISTS ||
          resp->result == BMKT_FP_DATABASE_EMPTY)
        {
          fp_info ("Database no record");
          fpi_device_delete_complete (device, NULL);
        }
      else
        {
          fp_info ("Failed to delete enrolled user: %d", resp->result);
          fpi_device_delete_complete (device,
                                      fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
        }
      break;

    case BMKT_RSP_DEL_USER_FP_OK:
      fp_info ("Successfully deleted enrolled user");
      fpi_device_delete_complete (device, NULL);
      break;
    }
}

static void
delete_print (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  FpPrint *print = NULL;

  g_autoptr(GVariant) data = NULL;
  guint8 finger;
  const guint8 *user_id;
  gsize user_id_len = 0;
  g_autofree guint8 *payload = NULL;

  fpi_device_get_delete_data (device, &print);

  g_object_get (print, "fpi-data", &data, NULL);
  g_debug ("data is %p", data);
  if (!parse_print_data (data, &finger, &user_id, &user_id_len))
    {
      fpi_device_delete_complete (device,
                                  fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  G_DEBUG_HERE ();

  payload = g_malloc0 (1 + user_id_len);
  payload[0] = finger;
  memcpy (payload + 1, user_id, user_id_len);

  synaptics_sensor_cmd (self, 0, BMKT_CMD_DEL_USER_FP, payload, user_id_len + 1, delete_msg_cb);
}

static void
clear_storage_msg_cb (FpiDeviceSynaptics *self,
                      bmkt_response_t    *resp,
                      GError             *error)
{
  FpDevice *device = FP_DEVICE (self);
  bmkt_del_all_users_resp_t *del_all_user_resp;

  if (error)
    {
      fpi_device_clear_storage_complete (device, error);
      return;
    }
  del_all_user_resp = &resp->response.del_all_user_resp;

  switch (resp->response_id)
    {
    case BMKT_RSP_DELETE_PROGRESS:
      fp_info ("Deleting All Enrolled Users is %d%% complete",
               del_all_user_resp->progress);
      break;

    case BMKT_RSP_DEL_FULL_DB_FAIL:
      if (resp->result == BMKT_FP_DATABASE_EMPTY)
        fpi_device_clear_storage_complete (device, NULL);
      else
        fpi_device_clear_storage_complete (device,
                                           fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      break;

    case BMKT_RSP_DEL_FULL_DB_OK:
      fp_info ("Successfully deleted all enrolled user");
      fpi_device_clear_storage_complete (device, NULL);
      break;
    }
}

static void
clear_storage (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);

  g_debug ("clear all prints in database");
  synaptics_sensor_cmd (self, 0, BMKT_CMD_DEL_FULL_DB, NULL, 0, clear_storage_msg_cb);
  return;
}


static void
prob_msg_cb (FpiDeviceSynaptics *self,
             bmkt_response_t    *resp,
             GError             *error)
{
  GUsbDevice *usb_dev = NULL;
  g_autofree gchar *serial = NULL;
  GError *err = NULL;

  usb_dev = fpi_device_get_usb_device (FP_DEVICE (self));

  if (error)
    {
      if (!g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        err = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL, "unsupported firmware version");

      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (FP_DEVICE (self), NULL, NULL, err);
      g_clear_error (&error);
      return;
    }

  if (g_strcmp0 (g_getenv ("FP_DEVICE_EMULATION"), "1") == 0)
    serial = g_strdup ("emulated-device");
  else
    serial = g_usb_device_get_string_descriptor (usb_dev,
                                                 g_usb_device_get_serial_number_index (usb_dev),
                                                 &err);

  /* BMKT_OPERATION_DENIED is returned if the sensor is already initialized */
  if (resp->result == BMKT_SUCCESS || resp->result == BMKT_OPERATION_DENIED)
    {
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (FP_DEVICE (self), serial, NULL, err);
    }
  else if (resp->result == BMKT_FP_SYSTEM_BUSY)
    {
      synaptics_sensor_cmd (self, self->cmd_seq_num, BMKT_CMD_CANCEL_OP, NULL, 0, NULL);
    }
  else
    {
      g_warning ("Probe fingerprint sensor failed with %d!", resp->result);
      g_usb_device_close (usb_dev, NULL);
      fpi_device_probe_complete (FP_DEVICE (self), serial, NULL, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
    }
}

static void
dev_probe (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  GUsbDevice *usb_dev;

  g_autoptr(FpiUsbTransfer) transfer = NULL;
  FpiByteReader reader;
  GError *error = NULL;
  guint16 status;
  const guint8 *data;
  gboolean read_ok = TRUE;
  g_autofree gchar *serial = NULL;
  gboolean retry = TRUE;

  G_DEBUG_HERE ();

  /* Claim usb interface */
  usb_dev = fpi_device_get_usb_device (device);
  if (!g_usb_device_open (usb_dev, &error))
    {
      fpi_device_probe_complete (device, NULL, NULL, error);
      return;
    }

  if (!g_usb_device_claim_interface (usb_dev, 0, 0, &error))
    goto err_close;

  while(1)
    {
      /* TODO: Do not do this synchronous. */
      transfer = fpi_usb_transfer_new (device);
      fpi_usb_transfer_fill_bulk (transfer, USB_EP_REQUEST, SENSOR_FW_CMD_HEADER_LEN);
      transfer->short_is_error = TRUE;
      transfer->buffer[0] = SENSOR_CMD_GET_VERSION;
      if (!fpi_usb_transfer_submit_sync (transfer, 1000, &error))
        goto err_close;

      g_clear_pointer (&transfer, fpi_usb_transfer_unref);
      transfer = fpi_usb_transfer_new (device);
      fpi_usb_transfer_fill_bulk (transfer, USB_EP_REPLY, 40);
      if (!fpi_usb_transfer_submit_sync (transfer, 1000, &error))
        goto err_close;

      fpi_byte_reader_init (&reader, transfer->buffer, transfer->actual_length);

      if (!fpi_byte_reader_get_uint16_le (&reader, &status))
        {
          g_warning ("Transfer in response to version query was too short");
          error = fpi_device_error_new (FP_DEVICE_ERROR_PROTO);
          goto err_close;
        }
      if (status != 0)
        {
          g_warning ("Device responded with error: %d retry: %d", status, retry);
          if(retry)
            {
              retry = FALSE;
              continue;
            }
          error = fpi_device_error_new (FP_DEVICE_ERROR_PROTO);
          goto err_close;
        }
      break;
    }
  read_ok &= fpi_byte_reader_get_uint32_le (&reader, &self->mis_version.build_time);
  read_ok &= fpi_byte_reader_get_uint32_le (&reader, &self->mis_version.build_num);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.version_major);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.version_minor);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.target);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.product);

  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.silicon_rev);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.formal_release);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.platform);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.patch);
  if (fpi_byte_reader_get_data (&reader, sizeof (self->mis_version.serial_number), &data))
    memcpy (self->mis_version.serial_number, data, sizeof (self->mis_version.serial_number));
  else
    read_ok = FALSE;
  read_ok &= fpi_byte_reader_get_uint16_le (&reader, &self->mis_version.security);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.iface);
  read_ok &= fpi_byte_reader_get_uint8 (&reader, &self->mis_version.device_type);

  if (!read_ok)
    {
      g_warning ("Transfer in response to version query was too short");
      error = fpi_device_error_new (FP_DEVICE_ERROR_PROTO);
      goto err_close;
    }

  fp_dbg ("Build Time: %d", self->mis_version.build_time);
  fp_dbg ("Build Num: %d", self->mis_version.build_num);
  fp_dbg ("Version: %d.%d", self->mis_version.version_major, self->mis_version.version_minor);
  fp_dbg ("Target: %d", self->mis_version.target);
  fp_dbg ("Product: %d", self->mis_version.product);

  synaptics_sensor_cmd (self, 0, BMKT_CMD_FPS_INIT, NULL, 0, prob_msg_cb);

  return;

err_close:
  g_usb_device_close (usb_dev, NULL);
  fpi_device_probe_complete (device, NULL, NULL, error);
}

static void
fps_init_msg_cb (FpiDeviceSynaptics *self,
                 bmkt_response_t    *resp,
                 GError             *error)
{
  if (error)
    {
      if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        g_clear_error (&error);

      fpi_device_open_complete (FP_DEVICE (self), error);
      return;
    }

  /* BMKT_OPERATION_DENIED is returned if the sensor is already initialized */
  if (resp->result == BMKT_SUCCESS || resp->result == BMKT_OPERATION_DENIED)
    {
      fpi_device_open_complete (FP_DEVICE (self), NULL);
    }
  else if (resp->result == BMKT_FP_SYSTEM_BUSY)
    {
      synaptics_sensor_cmd (self, self->cmd_seq_num, BMKT_CMD_CANCEL_OP, NULL, 0, NULL);
    }
  else
    {
      g_warning ("Initializing fingerprint sensor failed with %d!", resp->result);
      fpi_device_open_complete (FP_DEVICE (self),
                                fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
    }
}
static void
fps_deinit_cb (FpiDeviceSynaptics *self,
               bmkt_response_t    *resp,
               GError             *error)
{
  /* Release usb interface */
  g_usb_device_release_interface (fpi_device_get_usb_device (FP_DEVICE (self)), 0, 0, &error);

  g_clear_object (&self->interrupt_cancellable);

  if (!error)
    {
      switch (resp->response_id)
        {
        case BMKT_RSP_POWER_DOWN_READY:
          fp_info ("Fingerprint sensor ready to be powered down");
          break;

        case BMKT_RSP_POWER_DOWN_FAIL:
          fp_info ("Failed to go to power down mode: %d", resp->result);
          error = fpi_device_error_new_msg (FP_DEVICE_ERROR_GENERAL,
                                            "Power down failed: %d", resp->result);

          break;
        }
    }
  fpi_device_close_complete (FP_DEVICE (self), error);
}

static void
dev_init (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);
  GError *error = NULL;

  G_DEBUG_HERE ();

  self->interrupt_cancellable = g_cancellable_new ();

  /* Claim usb interface */
  if (!g_usb_device_claim_interface (fpi_device_get_usb_device (device), 0, 0, &error))
    goto error;

  synaptics_sensor_cmd (self, 0, BMKT_CMD_FPS_INIT, NULL, 0, fps_init_msg_cb);

  return;

error:
  fpi_device_open_complete (FP_DEVICE (self), error);
}

static void
dev_exit (FpDevice *device)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (device);

  G_DEBUG_HERE ();

  synaptics_sensor_cmd (self, 0, BMKT_CMD_POWER_DOWN_NOTIFY, NULL, 0, fps_deinit_cb);
}

static void
cancel (FpDevice *dev)
{
  FpiDeviceSynaptics *self = FPI_DEVICE_SYNAPTICS (dev);

  /* We just send out a cancel command and hope for the best. */
  synaptics_sensor_cmd (self, -1, BMKT_CMD_CANCEL_OP, NULL, 0, NULL);

  /* Cancel any current interrupt transfer (resulting us to go into
   * response reading mode again); then create a new cancellable
   * for the next transfers. */
  g_cancellable_cancel (self->interrupt_cancellable);
  g_clear_object (&self->interrupt_cancellable);
  self->interrupt_cancellable = g_cancellable_new ();
}

static void
fpi_device_synaptics_init (FpiDeviceSynaptics *self)
{
}

static void
fpi_device_synaptics_class_init (FpiDeviceSynapticsClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = SYNAPTICS_DRIVER_FULLNAME;

  dev_class->type = FP_DEVICE_TYPE_USB;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  dev_class->id_table = id_table;
  dev_class->nr_enroll_stages = ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1;

  dev_class->open = dev_init;
  dev_class->close = dev_exit;
  dev_class->probe = dev_probe;
  dev_class->verify = verify;
  dev_class->identify = identify;
  dev_class->enroll = enroll;
  dev_class->delete = delete_print;
  dev_class->clear_storage = clear_storage;
  dev_class->cancel = cancel;

  fpi_device_class_auto_initialize_features (dev_class);
}
