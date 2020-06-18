/*
 * FpImageDevice - An image based fingerprint reader device - Private APIs
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

#define FP_COMPONENT "image_device"
#include "fpi-log.h"

#include "fp-image-device-private.h"
#include "fp-image-device.h"

/**
 * SECTION: fpi-image-device
 * @title: Internal FpImageDevice
 * @short_description: Internal image device functions
 *
 * Internal image device functions. See #FpImageDevice for public routines.
 */

/* Manually redefine what G_DEFINE_* macro does */
static inline gpointer
fp_image_device_get_instance_private (FpImageDevice *self)
{
  FpImageDeviceClass *img_class = g_type_class_peek_static (FP_TYPE_IMAGE_DEVICE);

  return G_STRUCT_MEMBER_P (self,
                            g_type_class_get_instance_private_offset (img_class));
}

/* Private shared functions */

void
fpi_image_device_activate (FpImageDevice *self)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpImageDeviceClass *cls = FP_IMAGE_DEVICE_GET_CLASS (self);

  g_assert (!priv->active);

  /* We don't have a neutral ACTIVE state, but we always will
   * go into WAIT_FINGER_ON afterwards. */
  priv->state = FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON;
  g_object_notify (G_OBJECT (self), "fpi-image-device-state");

  /* We might have been waiting for deactivation to finish before
   * starting the next operation. */
  g_clear_handle_id (&priv->pending_activation_timeout_id, g_source_remove);

  fp_dbg ("Activating image device\n");
  cls->activate (self);
}

void
fpi_image_device_deactivate (FpImageDevice *self)
{
  FpDevice *device = FP_DEVICE (self);
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpImageDeviceClass *cls = FP_IMAGE_DEVICE_GET_CLASS (device);

  if (!priv->active)
    {
      /* XXX: We currently deactivate both from minutiae scan result
       *      and finger off report. */
      fp_dbg ("Already deactivated, ignoring request.");
      return;
    }
  if (!priv->cancelling && priv->state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON)
    g_warning ("Deactivating image device while waiting for finger, this should not happen.");

  priv->state = FPI_IMAGE_DEVICE_STATE_INACTIVE;
  g_object_notify (G_OBJECT (self), "fpi-image-device-state");

  fp_dbg ("Deactivating image device\n");
  cls->deactivate (self);
}

/* Static helper functions */

static void
fp_image_device_change_state (FpImageDevice *self, FpiImageDeviceState state)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  g_autofree char *prev_state_str = NULL;
  g_autofree char *state_str = NULL;

  /* Cannot change to inactive using this function. */
  g_assert (state != FPI_IMAGE_DEVICE_STATE_INACTIVE);

  /* We might have been waiting for the finger to go OFF to start the
   * next operation. */
  g_clear_handle_id (&priv->pending_activation_timeout_id, g_source_remove);

  prev_state_str = g_enum_to_string (FPI_TYPE_IMAGE_DEVICE_STATE, priv->state);
  state_str = g_enum_to_string (FPI_TYPE_IMAGE_DEVICE_STATE, state);
  fp_dbg ("Image device internal state change from %s to %s\n",
          prev_state_str, state_str);

  priv->state = state;
  g_object_notify (G_OBJECT (self), "fpi-image-device-state");
  g_signal_emit_by_name (self, "fpi-image-device-state-changed", priv->state);
}

static void
fp_image_device_enroll_maybe_await_finger_on (FpImageDevice *self)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);

  if (priv->enroll_await_on_pending)
    {
      priv->enroll_await_on_pending = FALSE;
      fp_image_device_change_state (self, FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON);
    }
  else
    {
      fp_dbg ("Awaiting finger on");
      priv->enroll_await_on_pending = TRUE;
    }
}

static void
fpi_image_device_minutiae_detected (GObject *source_object, GAsyncResult *res, gpointer user_data)
{
  g_autoptr(FpImage) image = FP_IMAGE (source_object);
  g_autoptr(FpPrint) print = NULL;
  GError *error = NULL;
  FpImageDevice *self = FP_IMAGE_DEVICE (user_data);
  FpDevice *device = FP_DEVICE (self);
  FpImageDevicePrivate *priv;
  FpiDeviceAction action;

  /* Note: We rely on the device to not disappear during an operation. */

  if (!fp_image_detect_minutiae_finish (image, res, &error))
    {
      /* Cancel operation . */
      if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        {
          fpi_device_action_error (device, g_steal_pointer (&error));
          fpi_image_device_deactivate (self);
          return;
        }

      /* Replace error with a retry condition. */
      g_warning ("Failed to detect minutiae: %s", error->message);
      g_clear_pointer (&error, g_error_free);

      error = fpi_device_retry_new_msg (FP_DEVICE_RETRY_GENERAL, "Minutiae detection failed, please retry");
    }

  priv = fp_image_device_get_instance_private (FP_IMAGE_DEVICE (device));
  action = fpi_device_get_current_action (device);

  if (action == FPI_DEVICE_ACTION_CAPTURE)
    {
      fpi_device_capture_complete (device, g_steal_pointer (&image), error);
      fpi_image_device_deactivate (self);
      return;
    }

  if (!error)
    {
      print = fp_print_new (device);
      fpi_print_set_type (print, FPI_PRINT_NBIS);
      if (!fpi_print_add_from_image (print, image, &error))
        {
          g_clear_object (&print);

          if (error->domain != FP_DEVICE_RETRY)
            {
              fpi_device_action_error (device, error);
              fpi_image_device_deactivate (self);
              return;
            }
        }
    }

  if (action == FPI_DEVICE_ACTION_ENROLL)
    {
      FpPrint *enroll_print;
      fpi_device_get_enroll_data (device, &enroll_print);

      if (print)
        {
          fpi_print_add_print (enroll_print, print);
          priv->enroll_stage += 1;
        }

      fpi_device_enroll_progress (device, priv->enroll_stage,
                                  g_steal_pointer (&print), error);

      /* Start another scan or deactivate. */
      if (priv->enroll_stage == IMG_ENROLL_STAGES)
        {
          fpi_device_enroll_complete (device, g_object_ref (enroll_print), NULL);
          fpi_image_device_deactivate (self);
        }
      else
        {
          fp_image_device_enroll_maybe_await_finger_on (FP_IMAGE_DEVICE (device));
        }
    }
  else if (action == FPI_DEVICE_ACTION_VERIFY)
    {
      FpPrint *template;
      FpiMatchResult result;

      fpi_device_get_verify_data (device, &template);
      if (print)
        result = fpi_print_bz3_match (template, print, priv->bz3_threshold, &error);
      else
        result = FPI_MATCH_ERROR;

      if (!error || error->domain == FP_DEVICE_RETRY)
        fpi_device_verify_report (device, result, g_steal_pointer (&print), g_steal_pointer (&error));
      fpi_device_verify_complete (device, error);
      fpi_image_device_deactivate (self);
    }
  else if (action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      gint i;
      GPtrArray *templates;
      FpPrint *result = NULL;

      fpi_device_get_identify_data (device, &templates);
      for (i = 0; !error && i < templates->len; i++)
        {
          FpPrint *template = g_ptr_array_index (templates, i);

          if (fpi_print_bz3_match (template, print, priv->bz3_threshold, &error) == FPI_MATCH_SUCCESS)
            {
              result = template;
              break;
            }
        }

      if (!error || error->domain == FP_DEVICE_RETRY)
        fpi_device_identify_report (device, result, g_steal_pointer (&print), g_steal_pointer (&error));
      fpi_device_identify_complete (device, error);
      fpi_image_device_deactivate (self);
    }
  else
    {
      /* XXX: This can be hit currently due to a race condition in the enroll code!
       *      In that case we scan a further image even though the minutiae for the previous
       *      one have not yet been detected.
       *      We need to keep track on the pending minutiae detection and the fact that
       *      it will finish eventually (or we may need to retry on error and activate the
       *      device again). */
      g_assert_not_reached ();
    }
}

/*********************************************************/
/* Private API */

/**
 * fpi_image_device_set_bz3_threshold:
 * @self: a #FpImageDevice imaging fingerprint device
 * @bz3_threshold: BZ3 threshold to use
 *
 * Dynamically adjust the bz3 threshold. This is only needed for drivers
 * that support devices with different properties. It should generally be
 * called from the probe callback, but is acceptable to call from the open
 * callback.
 */
void
fpi_image_device_set_bz3_threshold (FpImageDevice *self,
                                    gint           bz3_threshold)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);

  g_return_if_fail (FP_IS_IMAGE_DEVICE (self));
  g_return_if_fail (bz3_threshold > 0);

  priv->bz3_threshold = bz3_threshold;
}

/**
 * fpi_image_device_report_finger_status:
 * @self: a #FpImageDevice imaging fingerprint device
 * @present: whether the finger is present on the sensor
 *
 * Reports from the driver whether the user's finger is on
 * the sensor.
 */
void
fpi_image_device_report_finger_status (FpImageDevice *self,
                                       gboolean       present)
{
  FpDevice *device = FP_DEVICE (self);
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpiDeviceAction action;

  if (priv->state == FPI_IMAGE_DEVICE_STATE_INACTIVE)
    {
      /* Do we really want to always ignore such reports? We could
       * also track the state in case the user had the finger on
       * the device at initialisation time and the driver reports
       * this early.
       */
      g_debug ("Ignoring finger presence report as the device is not active!");
      return;
    }

  action = fpi_device_get_current_action (device);

  g_assert (action != FPI_DEVICE_ACTION_OPEN);
  g_assert (action != FPI_DEVICE_ACTION_CLOSE);

  g_debug ("Image device reported finger status: %s", present ? "on" : "off");

  if (present && priv->state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON)
    {
      fp_image_device_change_state (self, FPI_IMAGE_DEVICE_STATE_CAPTURE);
    }
  else if (!present && priv->state == FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF)
    {
      /* We need to deactivate or continue to await finger */

      /* There are three possible situations:
       *  1. We are deactivating the device and the action is still in progress
       *     (minutiae detection).
       *  2. We are still deactivating the device after an action completed
       *  3. We were waiting for finger removal to start the new action
       * Either way, we always end up deactivating except for the enroll case.
       *
       * The enroll case is special as AWAIT_FINGER_ON should only happen after
       * minutiae detection to prevent deactivation (without cancellation)
       * from the AWAIT_FINGER_ON state.
       */
      if (action != FPI_DEVICE_ACTION_ENROLL)
        fpi_image_device_deactivate (self);
      else
        fp_image_device_enroll_maybe_await_finger_on (self);
    }
}

/**
 * fpi_image_device_image_captured:
 * @self: a #FpImageDevice imaging fingerprint device
 * @image: whether the finger is present on the sensor
 *
 * Reports an image capture. Only use this function if the image was
 * captured successfully. If there was an issue where the user should
 * retry, use fpi_image_device_retry_scan() to report the retry condition.
 *
 * In the event of a fatal error for the operation use
 * fpi_image_device_session_error(). This will abort the entire operation
 * including e.g. an enroll operation which captures multiple images during
 * one session.
 */
void
fpi_image_device_image_captured (FpImageDevice *self, FpImage *image)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpiDeviceAction action;

  action = fpi_device_get_current_action (FP_DEVICE (self));

  g_return_if_fail (image != NULL);
  g_return_if_fail (priv->state == FPI_IMAGE_DEVICE_STATE_CAPTURE);
  g_return_if_fail (action == FPI_DEVICE_ACTION_ENROLL ||
                    action == FPI_DEVICE_ACTION_VERIFY ||
                    action == FPI_DEVICE_ACTION_IDENTIFY ||
                    action == FPI_DEVICE_ACTION_CAPTURE);

  fp_image_device_change_state (self, FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF);

  g_debug ("Image device captured an image");

  /* XXX: We also detect minutiae in capture mode, we solely do this
   *      to normalize the image which will happen as a by-product. */
  fp_image_detect_minutiae (image,
                            fpi_device_get_cancellable (FP_DEVICE (self)),
                            fpi_image_device_minutiae_detected,
                            self);
}

/**
 * fpi_image_device_retry_scan:
 * @self: a #FpImageDevice imaging fingerprint device
 * @retry: The #FpDeviceRetry error code to report
 *
 * Reports a scan failure to the user. This may or may not abort the
 * current session. It is the equivalent of fpi_image_device_image_captured()
 * in the case of a retryable error condition (e.g. short swipe).
 */
void
fpi_image_device_retry_scan (FpImageDevice *self, FpDeviceRetry retry)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpiDeviceAction action;
  GError *error;

  action = fpi_device_get_current_action (FP_DEVICE (self));

  /* We might be waiting for a finger at this point, so just accept
   * all but INACTIVE */
  g_return_if_fail (priv->state != FPI_IMAGE_DEVICE_STATE_INACTIVE);
  g_return_if_fail (action == FPI_DEVICE_ACTION_ENROLL ||
                    action == FPI_DEVICE_ACTION_VERIFY ||
                    action == FPI_DEVICE_ACTION_IDENTIFY ||
                    action == FPI_DEVICE_ACTION_CAPTURE);

  error = fpi_device_retry_new (retry);

  if (action == FPI_DEVICE_ACTION_ENROLL)
    {
      g_debug ("Reporting retry during enroll");
      fpi_device_enroll_progress (FP_DEVICE (self), priv->enroll_stage, NULL, error);

      /* Wait for finger removal and re-touch.
       * TODO: Do we need to check that the finger is already off? */
      priv->enroll_await_on_pending = TRUE;
      fp_image_device_change_state (self, FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_OFF);
    }
  else if (action == FPI_DEVICE_ACTION_VERIFY)
    {
      fpi_device_verify_report (FP_DEVICE (self), FPI_MATCH_ERROR, NULL, error);
      priv->cancelling = TRUE;
      fpi_image_device_deactivate (self);
      priv->cancelling = FALSE;
      fpi_device_verify_complete (FP_DEVICE (self), NULL);
    }
  else if (action == FPI_DEVICE_ACTION_IDENTIFY)
    {
      fpi_device_identify_report (FP_DEVICE (self), NULL, NULL, error);
      priv->cancelling = TRUE;
      fpi_image_device_deactivate (self);
      priv->cancelling = FALSE;
      fpi_device_identify_complete (FP_DEVICE (self), NULL);
    }
  else
    {
      /* We abort the operation and let the surrounding code retry in the
       * non-enroll case (this is identical to a session error). */
      g_debug ("Abort current operation due to retry (non-enroll case)");
      priv->cancelling = TRUE;
      fpi_image_device_deactivate (self);
      priv->cancelling = FALSE;
      fpi_device_action_error (FP_DEVICE (self), error);
    }
}

/**
 * fpi_image_device_session_error:
 * @self: a #FpImageDevice imaging fingerprint device
 * @error: The #GError to report
 *
 * Report an error while interacting with the device. This effectively
 * aborts the current ongoing action. Note that doing so will result in
 * the deactivation handler to be called and this function must not be
 * used to report an error during deactivation.
 */
void
fpi_image_device_session_error (FpImageDevice *self, GError *error)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);

  g_return_if_fail (self);

  if (!error)
    {
      g_warning ("Driver did not provide an error, generating a generic one");
      error = g_error_new (FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL, "Driver reported session error without an error");
    }

  if (!priv->active)
    {
      FpiDeviceAction action = fpi_device_get_current_action (FP_DEVICE (self));
      g_warning ("Driver reported session error, but device is inactive.");

      if (action != FPI_DEVICE_ACTION_NONE)
        {
          g_warning ("Translating to activation failure!");
          fpi_image_device_activate_complete (self, error);
          return;
        }
    }
  else if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED) &&
           fpi_device_action_is_cancelled (FP_DEVICE (self)))
    {
      /* Ignore cancellation errors here, as we will explicitly deactivate
       * anyway (or, may already have done so at this point).
       */
      g_debug ("Driver reported a cancellation error, this is expected but not required. Ignoring.");
      g_clear_error (&error);
      return;
    }
  else if (priv->state == FPI_IMAGE_DEVICE_STATE_INACTIVE)
    {
      g_warning ("Driver reported session error while deactivating already, ignoring. This indicates a driver bug.");
      g_clear_error (&error);
      return;
    }

  if (error->domain == FP_DEVICE_RETRY)
    g_warning ("Driver should report retries using fpi_image_device_retry_scan!");

  priv->cancelling = TRUE;
  fpi_image_device_deactivate (self);
  priv->cancelling = FALSE;
  fpi_device_action_error (FP_DEVICE (self), error);
}

/**
 * fpi_image_device_activate_complete:
 * @self: a #FpImageDevice imaging fingerprint device
 * @error: A #GError or %NULL on success
 *
 * Reports completion of device activation.
 */
void
fpi_image_device_activate_complete (FpImageDevice *self, GError *error)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpiDeviceAction action;

  action = fpi_device_get_current_action (FP_DEVICE (self));

  g_return_if_fail (priv->active == FALSE);
  g_return_if_fail (action == FPI_DEVICE_ACTION_ENROLL ||
                    action == FPI_DEVICE_ACTION_VERIFY ||
                    action == FPI_DEVICE_ACTION_IDENTIFY ||
                    action == FPI_DEVICE_ACTION_CAPTURE);

  if (error)
    {
      g_debug ("Image device activation failed");
      fpi_device_action_error (FP_DEVICE (self), error);
      return;
    }

  g_debug ("Image device activation completed");

  priv->active = TRUE;

  /* We always want to capture at this point, move to AWAIT_FINGER
   * state. */
  fp_image_device_change_state (self, FPI_IMAGE_DEVICE_STATE_AWAIT_FINGER_ON);
}

/**
 * fpi_image_device_deactivate_complete:
 * @self: a #FpImageDevice imaging fingerprint device
 * @error: A #GError or %NULL on success
 *
 * Reports completion of device deactivation.
 */
void
fpi_image_device_deactivate_complete (FpImageDevice *self, GError *error)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpImageDeviceClass *cls = FP_IMAGE_DEVICE_GET_CLASS (self);
  FpiDeviceAction action;

  g_return_if_fail (priv->active == TRUE);
  g_return_if_fail (priv->state == FPI_IMAGE_DEVICE_STATE_INACTIVE);

  g_debug ("Image device deactivation completed");

  priv->active = FALSE;

  /* Deactivation completed. As we deactivate in the background
   * there may already be a new task pending. Check whether we
   * need to do anything. */
  action = fpi_device_get_current_action (FP_DEVICE (self));

  /* Special case, if we should be closing, but didn't due to a running
   * deactivation, then do so now. */
  if (action == FPI_DEVICE_ACTION_CLOSE)
    {
      cls->img_close (self);
      return;
    }

  /* We might be waiting to be able to activate again. */
  if (priv->pending_activation_timeout_id)
    {
      g_clear_handle_id (&priv->pending_activation_timeout_id, g_source_remove);
      priv->pending_activation_timeout_id =
        g_idle_add ((GSourceFunc) fpi_image_device_activate, self);
    }
}

/**
 * fpi_image_device_open_complete:
 * @self: a #FpImageDevice imaging fingerprint device
 * @error: A #GError or %NULL on success
 *
 * Reports completion of open operation.
 */
void
fpi_image_device_open_complete (FpImageDevice *self, GError *error)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpiDeviceAction action;

  action = fpi_device_get_current_action (FP_DEVICE (self));

  g_return_if_fail (priv->active == FALSE);
  g_return_if_fail (action == FPI_DEVICE_ACTION_OPEN);

  g_debug ("Image device open completed");

  priv->state = FPI_IMAGE_DEVICE_STATE_INACTIVE;
  g_object_notify (G_OBJECT (self), "fpi-image-device-state");

  fpi_device_open_complete (FP_DEVICE (self), error);
}

/**
 * fpi_image_device_close_complete:
 * @self: a #FpImageDevice imaging fingerprint device
 * @error: A #GError or %NULL on success
 *
 * Reports completion of close operation.
 */
void
fpi_image_device_close_complete (FpImageDevice *self, GError *error)
{
  FpImageDevicePrivate *priv = fp_image_device_get_instance_private (self);
  FpiDeviceAction action;

  action = fpi_device_get_current_action (FP_DEVICE (self));

  g_debug ("Image device close completed");

  g_return_if_fail (priv->active == FALSE);
  g_return_if_fail (action == FPI_DEVICE_ACTION_CLOSE);

  priv->state = FPI_IMAGE_DEVICE_STATE_INACTIVE;
  g_object_notify (G_OBJECT (self), "fpi-image-device-state");

  fpi_device_close_complete (FP_DEVICE (self), error);
}
