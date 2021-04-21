/*
 * FpDevice - A fingerprint reader device
 * Copyright (C) 2019 Benjamin Berg <bberg@redhat.com>
 * Copyright (C) 2019 Marco Trevisan <marco.trevisan@canonical.com>
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

#define FP_COMPONENT "device"
#include "fpi-log.h"

#include "fp-device-private.h"

/**
 * SECTION: fp-device
 * @title: FpDevice
 * @short_description: Fingerpint device routines
 *
 * These are the public #FpDevice routines.
 */

static void fp_device_async_initable_iface_init (GAsyncInitableIface *iface);

G_DEFINE_TYPE_EXTENDED (FpDevice, fp_device, G_TYPE_OBJECT, G_TYPE_FLAG_ABSTRACT,
                        G_IMPLEMENT_INTERFACE (G_TYPE_ASYNC_INITABLE,
                                               fp_device_async_initable_iface_init)
                        G_ADD_PRIVATE (FpDevice))

enum {
  PROP_0,
  PROP_DRIVER,
  PROP_DEVICE_ID,
  PROP_NAME,
  PROP_OPEN,
  PROP_REMOVED,
  PROP_NR_ENROLL_STAGES,
  PROP_SCAN_TYPE,
  PROP_FINGER_STATUS,
  PROP_TEMPERATURE,
  PROP_FPI_ENVIRON,
  PROP_FPI_USB_DEVICE,
  PROP_FPI_UDEV_DATA_SPIDEV,
  PROP_FPI_UDEV_DATA_HIDRAW,
  PROP_FPI_DRIVER_DATA,
  N_PROPS
};

static GParamSpec *properties[N_PROPS];

enum {
  REMOVED_SIGNAL,
  N_SIGNALS
};

static guint signals[N_SIGNALS] = { 0, };

/**
 * fp_device_retry_quark:
 *
 * Return value: Quark representing a retryable error.
 **/
G_DEFINE_QUARK (fp - device - retry - quark, fp_device_retry)

/**
 * fp_device_error_quark:
 *
 * Return value: Quark representing a device error.
 **/
G_DEFINE_QUARK (fp - device - error - quark, fp_device_error)

static gboolean
fp_device_cancel_in_idle_cb (gpointer user_data)
{
  FpDevice *self = user_data;
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (self);
  FpDevicePrivate *priv = fp_device_get_instance_private (self);

  g_assert (cls->cancel);
  g_assert (priv->current_action != FPI_DEVICE_ACTION_NONE);

  g_debug ("Idle cancelling on ongoing operation!");

  priv->current_idle_cancel_source = NULL;

  cls->cancel (self);

  fpi_device_report_finger_status (self, FP_FINGER_STATUS_NONE);

  return G_SOURCE_REMOVE;
}

/* Notify the class that the task was cancelled; this should be connected
 * with the GTask as the user_data object for automatic cleanup together
 * with the task. */
static void
fp_device_cancelled_cb (GCancellable *cancellable, FpDevice *self)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (self);

  priv->current_idle_cancel_source = g_idle_source_new ();
  g_source_set_callback (priv->current_idle_cancel_source,
                         fp_device_cancel_in_idle_cb,
                         self,
                         NULL);
  g_source_attach (priv->current_idle_cancel_source,
                   g_task_get_context (priv->current_task));
  g_source_unref (priv->current_idle_cancel_source);
}

static void
maybe_cancel_on_cancelled (FpDevice     *device,
                           GCancellable *cancellable)
{
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  if (!cancellable || !cls->cancel)
    return;

  priv->current_cancellable_id = g_cancellable_connect (cancellable,
                                                        G_CALLBACK (fp_device_cancelled_cb),
                                                        device,
                                                        NULL);
}

static void
fp_device_constructed (GObject *object)
{
  FpDevice *self = (FpDevice *) object;
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (self);
  FpDevicePrivate *priv = fp_device_get_instance_private (self);

  g_assert (cls->features != FP_DEVICE_FEATURE_NONE);

  priv->type = cls->type;
  if (cls->nr_enroll_stages)
    priv->nr_enroll_stages = cls->nr_enroll_stages;
  priv->scan_type = cls->scan_type;
  priv->features = cls->features;
  priv->device_name = g_strdup (cls->full_name);
  priv->device_id = g_strdup ("0");

  if (cls->temp_hot_seconds > 0)
    {
      priv->temp_hot_seconds = cls->temp_hot_seconds;
      priv->temp_cold_seconds = cls->temp_cold_seconds;
      g_assert (priv->temp_cold_seconds > 0);
    }
  else if (cls->temp_hot_seconds == 0)
    {
      priv->temp_hot_seconds = DEFAULT_TEMP_HOT_SECONDS;
      priv->temp_cold_seconds = DEFAULT_TEMP_COLD_SECONDS;
    }
  else
    {
      /* Temperature management disabled */
      priv->temp_hot_seconds = -1;
      priv->temp_cold_seconds = -1;
    }

  /* Start out at not completely cold (i.e. assume we are only at the upper
   * bound of COLD).
   * To be fair, the warm-up from 0 to WARM should be really short either way.
   *
   * Note that a call to fpi_device_update_temp() is not needed here as no
   * timeout must be registered.
   */
  priv->temp_current = FP_TEMPERATURE_COLD;
  priv->temp_current_ratio = TEMP_COLD_THRESH;
  priv->temp_last_update = g_get_monotonic_time ();
  priv->temp_last_active = FALSE;

  G_OBJECT_CLASS (fp_device_parent_class)->constructed (object);
}

static void
fp_device_finalize (GObject *object)
{
  FpDevice *self = (FpDevice *) object;
  FpDevicePrivate *priv = fp_device_get_instance_private (self);

  g_assert (priv->current_action == FPI_DEVICE_ACTION_NONE);
  g_assert (priv->current_task == NULL);
  if (priv->is_open)
    g_warning ("User destroyed open device! Not cleaning up properly!");

  g_clear_pointer (&priv->temp_timeout, g_source_destroy);

  g_slist_free_full (priv->sources, (GDestroyNotify) g_source_destroy);

  g_clear_pointer (&priv->current_idle_cancel_source, g_source_destroy);
  g_clear_pointer (&priv->current_task_idle_return_source, g_source_destroy);

  g_clear_pointer (&priv->device_id, g_free);
  g_clear_pointer (&priv->device_name, g_free);

  g_clear_object (&priv->usb_device);
  g_clear_pointer (&priv->virtual_env, g_free);
  g_clear_pointer (&priv->udev_data.spidev_path, g_free);
  g_clear_pointer (&priv->udev_data.hidraw_path, g_free);

  G_OBJECT_CLASS (fp_device_parent_class)->finalize (object);
}

static void
fp_device_get_property (GObject    *object,
                        guint       prop_id,
                        GValue     *value,
                        GParamSpec *pspec)
{
  FpDevice *self = FP_DEVICE (object);
  FpDevicePrivate *priv = fp_device_get_instance_private (self);

  switch (prop_id)
    {
    case PROP_NR_ENROLL_STAGES:
      g_value_set_uint (value, priv->nr_enroll_stages);
      break;

    case PROP_SCAN_TYPE:
      g_value_set_enum (value, priv->scan_type);
      break;

    case PROP_FINGER_STATUS:
      g_value_set_flags (value, priv->finger_status);
      break;

    case PROP_TEMPERATURE:
      g_value_set_enum (value, priv->temp_current);
      break;

    case PROP_DRIVER:
      g_value_set_static_string (value, FP_DEVICE_GET_CLASS (self)->id);
      break;

    case PROP_DEVICE_ID:
      g_value_set_string (value, priv->device_id);
      break;

    case PROP_NAME:
      g_value_set_string (value, priv->device_name);
      break;

    case PROP_OPEN:
      g_value_set_boolean (value, priv->is_open);
      break;

    case PROP_REMOVED:
      g_value_set_boolean (value, priv->is_removed);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
fp_device_set_property (GObject      *object,
                        guint         prop_id,
                        const GValue *value,
                        GParamSpec   *pspec)
{
  FpDevice *self = FP_DEVICE (object);
  FpDevicePrivate *priv = fp_device_get_instance_private (self);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (self);

  /* _construct has not run yet, so we cannot use priv->type. */
  switch (prop_id)
    {
    case PROP_FPI_ENVIRON:
      if (cls->type == FP_DEVICE_TYPE_VIRTUAL)
        priv->virtual_env = g_value_dup_string (value);
      else
        g_assert (g_value_get_string (value) == NULL);
      break;

    case PROP_FPI_USB_DEVICE:
      if (cls->type == FP_DEVICE_TYPE_USB)
        priv->usb_device = g_value_dup_object (value);
      else
        g_assert (g_value_get_object (value) == NULL);
      break;

    case PROP_FPI_UDEV_DATA_SPIDEV:
      if (cls->type == FP_DEVICE_TYPE_UDEV)
        priv->udev_data.spidev_path = g_value_dup_string (value);
      else
        g_assert (g_value_get_string (value) == NULL);
      break;

    case PROP_FPI_UDEV_DATA_HIDRAW:
      if (cls->type == FP_DEVICE_TYPE_UDEV)
        priv->udev_data.hidraw_path = g_value_dup_string (value);
      else
        g_assert (g_value_get_string (value) == NULL);
      break;

    case PROP_FPI_DRIVER_DATA:
      priv->driver_data = g_value_get_uint64 (value);
      break;

    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
fp_device_async_initable_init_async (GAsyncInitable     *initable,
                                     int                 io_priority,
                                     GCancellable       *cancellable,
                                     GAsyncReadyCallback callback,
                                     gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevice *self = FP_DEVICE (initable);
  FpDevicePrivate *priv = fp_device_get_instance_private (self);

  /* It is next to impossible to call async_init at the wrong time. */
  g_assert (!priv->is_open);
  g_assert (!priv->current_task);

  task = g_task_new (self, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!FP_DEVICE_GET_CLASS (self)->probe)
    {
      g_task_return_boolean (task, TRUE);
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_PROBE;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (self, cancellable);

  FP_DEVICE_GET_CLASS (self)->probe (self);
}

static gboolean
fp_device_async_initable_init_finish (GAsyncInitable *initable,
                                      GAsyncResult   *res,
                                      GError        **error)
{
  return g_task_propagate_boolean (G_TASK (res), error);
}

static void
fp_device_async_initable_iface_init (GAsyncInitableIface *iface)
{
  iface->init_async = fp_device_async_initable_init_async;
  iface->init_finish = fp_device_async_initable_init_finish;
}

static void
fp_device_class_init (FpDeviceClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructed = fp_device_constructed;
  object_class->finalize = fp_device_finalize;
  object_class->get_property = fp_device_get_property;
  object_class->set_property = fp_device_set_property;

  properties[PROP_NR_ENROLL_STAGES] =
    g_param_spec_uint ("nr-enroll-stages",
                       "EnrollStages",
                       "Number of enroll stages needed on the device",
                       0, G_MAXUINT,
                       0,
                       G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_SCAN_TYPE] =
    g_param_spec_enum ("scan-type",
                       "ScanType",
                       "The scan type of the device",
                       FP_TYPE_SCAN_TYPE, FP_SCAN_TYPE_SWIPE,
                       G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_FINGER_STATUS] =
    g_param_spec_flags ("finger-status",
                        "FingerStatus",
                        "The status of the finger",
                        FP_TYPE_FINGER_STATUS_FLAGS, FP_FINGER_STATUS_NONE,
                        G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_TEMPERATURE] =
    g_param_spec_enum ("temperature",
                       "Temperature",
                       "The temperature estimation for device to prevent overheating.",
                       FP_TYPE_TEMPERATURE, FP_TEMPERATURE_COLD,
                       G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_DRIVER] =
    g_param_spec_string ("driver",
                         "Driver",
                         "String describing the driver",
                         NULL,
                         G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_DEVICE_ID] =
    g_param_spec_string ("device-id",
                         "Device ID",
                         "String describing the device, often generic but may be a serial number",
                         "",
                         G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_NAME] =
    g_param_spec_string ("name",
                         "Device Name",
                         "Human readable name for the device",
                         NULL,
                         G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_OPEN] =
    g_param_spec_boolean ("open",
                          "Opened",
                          "Whether the device is open or not", FALSE,
                          G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  properties[PROP_REMOVED] =
    g_param_spec_boolean ("removed",
                          "Removed",
                          "Whether the device has been removed from the system", FALSE,
                          G_PARAM_STATIC_STRINGS | G_PARAM_READABLE);

  /**
   * FpDevice::removed:
   * @device: the #FpDevice instance that emitted the signal
   *
   * This signal is emitted after the device has been removed and no operation
   * is pending anymore.
   *
   * The API user is still required to close a removed device. The above
   * guarantee means that the call to close the device can be made immediately
   * from the signal handler.
   *
   * The close operation will return FP_DEVICE_ERROR_REMOVED, but the device
   * will still be considered closed afterwards.
   *
   * The device will only be removed from the #FpContext after it has been
   * closed by the API user.
   **/
  signals[REMOVED_SIGNAL] = g_signal_new ("removed",
                                          G_TYPE_FROM_CLASS (klass),
                                          G_SIGNAL_RUN_LAST,
                                          0,
                                          NULL,
                                          NULL,
                                          g_cclosure_marshal_VOID__VOID,
                                          G_TYPE_NONE,
                                          0);

  /* Private properties */

  /**
   * FpDevice::fpi-environ: (skip)
   *
   * This property is only for internal purposes.
   *
   * Stability: private
   */
  properties[PROP_FPI_ENVIRON] =
    g_param_spec_string ("fpi-environ",
                         "Virtual Environment",
                         "Private: The environment variable for the virtual device",
                         NULL,
                         G_PARAM_STATIC_STRINGS | G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY);

  /**
   * FpDevice::fpi-usb-device: (skip)
   *
   * This property is only for internal purposes.
   *
   * Stability: private
   */
  properties[PROP_FPI_USB_DEVICE] =
    g_param_spec_object ("fpi-usb-device",
                         "USB Device",
                         "Private: The USB device for the device",
                         G_USB_TYPE_DEVICE,
                         G_PARAM_STATIC_STRINGS | G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY);
  /**
   * FpDevice::fpi-udev-data-spidev: (skip)
   *
   * This property is only for internal purposes.
   *
   * Stability: private
   */
  properties[PROP_FPI_UDEV_DATA_SPIDEV] =
    g_param_spec_string ("fpi-udev-data-spidev",
                         "Udev data: spidev path",
                         "Private: The path to /dev/spidevN.M",
                         NULL,
                         G_PARAM_STATIC_STRINGS | G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY);
  /**
   * FpDevice::fpi-udev-data-hidraw: (skip)
   *
   * This property is only for internal purposes.
   *
   * Stability: private
   */
  properties[PROP_FPI_UDEV_DATA_HIDRAW] =
    g_param_spec_string ("fpi-udev-data-hidraw",
                         "Udev data: hidraw path",
                         "Private: The path to /dev/hidrawN",
                         NULL,
                         G_PARAM_STATIC_STRINGS | G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY);

  /**
   * FpDevice::fpi-driver-data: (skip)
   *
   * This property is only for internal purposes.
   *
   * Stability: private
   */
  properties[PROP_FPI_DRIVER_DATA] =
    g_param_spec_uint64 ("fpi-driver-data",
                         "Driver Data",
                         "Private: The driver data from the ID table entry",
                         0,
                         G_MAXUINT64,
                         0,
                         G_PARAM_STATIC_STRINGS | G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY);

  g_object_class_install_properties (object_class, N_PROPS, properties);
}

static void
fp_device_init (FpDevice *self)
{
}

/**
 * fp_device_get_driver:
 * @device: A #FpDevice
 *
 * Returns: (transfer none): The ID of the driver
 */
const gchar *
fp_device_get_driver (FpDevice *device)
{
  g_return_val_if_fail (FP_IS_DEVICE (device), NULL);

  return FP_DEVICE_GET_CLASS (device)->id;
}

/**
 * fp_device_get_device_id:
 * @device: A #FpDevice
 *
 * Returns: (transfer none): The ID of the device
 */
const gchar *
fp_device_get_device_id (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), NULL);

  return priv->device_id;
}

/**
 * fp_device_get_name:
 * @device: A #FpDevice
 *
 * Returns: (transfer none): The human readable name of the device
 */
const gchar *
fp_device_get_name (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), NULL);

  return priv->device_name;
}

/**
 * fp_device_is_open:
 * @device: A #FpDevice
 *
 * Returns: Whether the device is open or not
 */
gboolean
fp_device_is_open (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  return priv->is_open;
}

/**
 * fp_device_get_scan_type:
 * @device: A #FpDevice
 *
 * Retrieves the scan type of the device.
 *
 * Returns: The #FpScanType
 */
FpScanType
fp_device_get_scan_type (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FP_SCAN_TYPE_SWIPE);

  return priv->scan_type;
}

/**
 * fp_device_get_finger_status:
 * @device: A #FpDevice
 *
 * Retrieves the finger status flags for the device.
 * This can be used by the UI to present the relevant feedback, although it
 * is not guaranteed to be a relevant value when not performing any action.
 *
 * Returns: The current #FpFingerStatusFlags
 */
FpFingerStatusFlags
fp_device_get_finger_status (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FP_FINGER_STATUS_NONE);

  return priv->finger_status;
}

/**
 * fp_device_get_nr_enroll_stages:
 * @device: A #FpDevice
 *
 * Retrieves the number of enroll stages for this device.
 *
 * Returns: The number of enroll stages
 */
gint
fp_device_get_nr_enroll_stages (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), -1);

  return priv->nr_enroll_stages;
}

/**
 * fp_device_supports_identify:
 * @device: A #FpDevice
 *
 * Check whether the device supports identification.
 *
 * Returns: Whether the device supports identification
 * Deprecated: 1.92.0: Use fp_device_has_feature() instead.
 */
gboolean
fp_device_supports_identify (FpDevice *device)
{
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  return cls->identify && !!(priv->features & FP_DEVICE_FEATURE_IDENTIFY);
}

/**
 * fp_device_supports_capture:
 * @device: A #FpDevice
 *
 * Check whether the device supports capturing images.
 *
 * Returns: Whether the device supports image capture
 * Deprecated: 1.92.0: Use fp_device_has_feature() instead.
 */
gboolean
fp_device_supports_capture (FpDevice *device)
{
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  return cls->capture && !!(priv->features & FP_DEVICE_FEATURE_CAPTURE);
}

/**
 * fp_device_has_storage:
 * @device: A #FpDevice
 *
 * Whether the device has on-chip storage. If it has, you can list the
 * prints stored on the with fp_device_list_prints() and you should
 * always delete prints from the device again using
 * fp_device_delete_print().
 * Deprecated: 1.92.0: Use fp_device_has_feature() instead.
 */
gboolean
fp_device_has_storage (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  return !!(priv->features & FP_DEVICE_FEATURE_STORAGE);
}

/**
 * fp_device_open:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to open the device. The callback will
 * be called once the operation has finished. Retrieve the result with
 * fp_device_open_finish().
 */
void
fp_device_open (FpDevice           *device,
                GCancellable       *cancellable,
                GAsyncReadyCallback callback,
                gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  GError *error = NULL;

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_ALREADY_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  switch (priv->type)
    {
    case FP_DEVICE_TYPE_USB:
      if (!g_usb_device_open (priv->usb_device, &error))
        {
          g_task_return_error (task, error);
          return;
        }
      break;

    case FP_DEVICE_TYPE_VIRTUAL:
    case FP_DEVICE_TYPE_UDEV:
      break;

    default:
      g_assert_not_reached ();
      g_task_return_error (task, fpi_device_error_new (FP_DEVICE_ERROR_GENERAL));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_OPEN;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);
  fpi_device_report_finger_status (device, FP_FINGER_STATUS_NONE);

  FP_DEVICE_GET_CLASS (device)->open (device);
}

/**
 * fp_device_open_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to open the device.
 * See fp_device_open().
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_open_finish (FpDevice     *device,
                       GAsyncResult *result,
                       GError      **error)
{
  return g_task_propagate_boolean (G_TASK (result), error);
}

/**
 * fp_device_close:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to close the device. The callback will
 * be called once the operation has finished. Retrieve the result with
 * fp_device_close_finish().
 */
void
fp_device_close (FpDevice           *device,
                 GCancellable       *cancellable,
                 GAsyncReadyCallback callback,
                 gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_CLOSE;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  FP_DEVICE_GET_CLASS (device)->close (device);
}

/**
 * fp_device_close_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to close the device.
 * See fp_device_close().
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_close_finish (FpDevice     *device,
                        GAsyncResult *result,
                        GError      **error)
{
  return g_task_propagate_boolean (G_TASK (result), error);
}


/**
 * fp_device_enroll:
 * @device: a #FpDevice
 * @template_print: (transfer floating): a #FpPrint
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @progress_cb: (nullable) (scope notified): progress reporting callback
 * @progress_data: (closure progress_cb): user data for @progress_cb
 * @progress_destroy: (destroy progress_data): Destroy notify for @progress_data
 * @callback: (scope async): the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to enroll a print. The callback will
 * be called once the operation has finished. Retrieve the result with
 * fp_device_enroll_finish().
 *
 * The @template_print parameter is a #FpPrint with available metadata filled
 * in. The driver may make use of this metadata, when e.g. storing the print on
 * device memory. It is undefined whether this print is filled in by the driver
 * and returned, or whether the driver will return a newly created print after
 * enrollment succeeded.
 */
void
fp_device_enroll (FpDevice           *device,
                  FpPrint            *template_print,
                  GCancellable       *cancellable,
                  FpEnrollProgress    progress_cb,
                  gpointer            progress_data,
                  GDestroyNotify      progress_destroy,
                  GAsyncReadyCallback callback,
                  gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpEnrollData *data;
  FpiPrintType print_type;

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  if (!FP_IS_PRINT (template_print))
    {
      g_warning ("User did not pass a print template!");
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  g_object_get (template_print, "fpi-type", &print_type, NULL);
  if (print_type != FPI_PRINT_UNDEFINED)
    {
      g_warning ("Passed print template must be newly created and blank!");
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_ENROLL;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  fpi_device_update_temp (device, TRUE);

  data = g_new0 (FpEnrollData, 1);
  data->print = g_object_ref_sink (template_print);
  data->enroll_progress_cb = progress_cb;
  data->enroll_progress_data = progress_data;
  data->enroll_progress_destroy = progress_destroy;

  // Attach the progress data as task data so that it is destroyed
  g_task_set_task_data (priv->current_task, data, (GDestroyNotify) enroll_data_free);

  FP_DEVICE_GET_CLASS (device)->enroll (device);
}

/**
 * fp_device_enroll_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to enroll a print. You should check
 * for an error of type %FP_DEVICE_RETRY to prompt the user again if there
 * was an interaction issue.
 * See fp_device_enroll().
 *
 * Returns: (transfer full): The enrolled #FpPrint, or %NULL on error
 */
FpPrint *
fp_device_enroll_finish (FpDevice     *device,
                         GAsyncResult *result,
                         GError      **error)
{
  return g_task_propagate_pointer (G_TASK (result), error);
}

/**
 * fp_device_verify:
 * @device: a #FpDevice
 * @enrolled_print: a #FpPrint to verify
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @match_cb: (nullable) (scope notified): match reporting callback
 * @match_data: (closure match_cb): user data for @match_cb
 * @match_destroy: (destroy match_data): Destroy notify for @match_data
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to verify a print. The callback will
 * be called once the operation has finished. Retrieve the result with
 * fp_device_verify_finish().
 */
void
fp_device_verify (FpDevice           *device,
                  FpPrint            *enrolled_print,
                  GCancellable       *cancellable,
                  FpMatchCb           match_cb,
                  gpointer            match_data,
                  GDestroyNotify      match_destroy,
                  GAsyncReadyCallback callback,
                  gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);
  FpMatchData *data;

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  if (!cls->verify || !(priv->features & FP_DEVICE_FEATURE_VERIFY))
    {
      g_task_return_error (task,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Device has no verification support"));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_VERIFY;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  fpi_device_update_temp (device, TRUE);

  data = g_new0 (FpMatchData, 1);
  data->enrolled_print = g_object_ref (enrolled_print);
  data->match_cb = match_cb;
  data->match_data = match_data;
  data->match_destroy = match_destroy;

  // Attach the match data as task data so that it is destroyed
  g_task_set_task_data (priv->current_task, data, (GDestroyNotify) match_data_free);

  cls->verify (device);
}

/**
 * fp_device_verify_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @match: (out): Whether the user presented the correct finger
 * @print: (out) (transfer full) (nullable): Location to store the scanned print, or %NULL to ignore
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to verify an enrolled print. You should check
 * for an error of type %FP_DEVICE_RETRY to prompt the user again if there
 * was an interaction issue.
 *
 * With @print you can fetch the newly created print and retrieve the image data if available.
 *
 * See fp_device_verify().
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_verify_finish (FpDevice     *device,
                         GAsyncResult *result,
                         gboolean     *match,
                         FpPrint     **print,
                         GError      **error)
{
  gint res;

  res = g_task_propagate_int (G_TASK (result), error);

  if (print)
    {
      FpMatchData *data;

      data = g_task_get_task_data (G_TASK (result));

      *print = data ? data->print : NULL;
      if (*print)
        g_object_ref (*print);
    }

  if (match)
    *match = res == FPI_MATCH_SUCCESS;

  return res != FPI_MATCH_ERROR;
}

/**
 * fp_device_identify:
 * @device: a #FpDevice
 * @prints: (element-type FpPrint) (transfer none): #GPtrArray of #FpPrint
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @match_cb: (nullable) (scope notified): match reporting callback
 * @match_data: (closure match_cb): user data for @match_cb
 * @match_destroy: (destroy match_data): Destroy notify for @match_data
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to identify prints. The callback will
 * be called once the operation has finished. Retrieve the result with
 * fp_device_identify_finish().
 */
void
fp_device_identify (FpDevice           *device,
                    GPtrArray          *prints,
                    GCancellable       *cancellable,
                    FpMatchCb           match_cb,
                    gpointer            match_data,
                    GDestroyNotify      match_destroy,
                    GAsyncReadyCallback callback,
                    gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);
  FpMatchData *data;
  int i;

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  if (!cls->identify || !(priv->features & FP_DEVICE_FEATURE_IDENTIFY))
    {
      g_task_return_error (task,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Device has not identification support"));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_IDENTIFY;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  fpi_device_update_temp (device, TRUE);

  data = g_new0 (FpMatchData, 1);
  /* We cannot store the gallery directly, because the ptr array may not own
   * a reference to each print. Also, the caller could in principle modify the
   * GPtrArray afterwards.
   */
  data->gallery = g_ptr_array_new_full (prints->len, g_object_unref);
  for (i = 0; i < prints->len; i++)
    g_ptr_array_add (data->gallery, g_object_ref (g_ptr_array_index (prints, i)));
  data->match_cb = match_cb;
  data->match_data = match_data;
  data->match_destroy = match_destroy;

  // Attach the match data as task data so that it is destroyed
  g_task_set_task_data (priv->current_task, data, (GDestroyNotify) match_data_free);

  cls->identify (device);
}

/**
 * fp_device_identify_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @match: (out) (transfer full) (nullable): Location for the matched #FpPrint, or %NULL
 * @print: (out) (transfer full) (nullable): Location for the new #FpPrint, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to identify a print. You should check
 * for an error of type %FP_DEVICE_RETRY to prompt the user again if there
 * was an interaction issue.
 *
 * Use @match to find the print that matched. With @print you can fetch the
 * newly created print and retrieve the image data if available.
 *
 * See fp_device_identify().
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_identify_finish (FpDevice     *device,
                           GAsyncResult *result,
                           FpPrint     **match,
                           FpPrint     **print,
                           GError      **error)
{
  FpMatchData *data;

  data = g_task_get_task_data (G_TASK (result));

  if (print)
    {
      *print = data ? data->print : NULL;
      if (*print)
        g_object_ref (*print);
    }
  if (match)
    {
      *match = data ? data->match : NULL;
      if (*match)
        g_object_ref (*match);
    }

  return g_task_propagate_boolean (G_TASK (result), error);
}

/**
 * fp_device_capture:
 * @device: a #FpDevice
 * @wait_for_finger: Whether to wait for a finger or not
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to capture an image. The callback will
 * be called once the operation has finished. Retrieve the result with
 * fp_device_capture_finish().
 */
void
fp_device_capture (FpDevice           *device,
                   gboolean            wait_for_finger,
                   GCancellable       *cancellable,
                   GAsyncReadyCallback callback,
                   gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  if (!cls->capture || !(priv->features & FP_DEVICE_FEATURE_CAPTURE))
    {
      g_task_return_error (task,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Device has no verification support"));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_CAPTURE;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  fpi_device_update_temp (device, TRUE);

  priv->wait_for_finger = wait_for_finger;

  cls->capture (device);
}

/**
 * fp_device_capture_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to capture an image. You should check
 * for an error of type %FP_DEVICE_RETRY to prompt the user again if there
 * was an interaction issue.
 *
 * See fp_device_capture().
 *
 * Returns: (transfer full): #FpImage or %NULL on error
 */
FpImage *
fp_device_capture_finish (FpDevice     *device,
                          GAsyncResult *result,
                          GError      **error)
{
  return g_task_propagate_pointer (G_TASK (result), error);
}

/**
 * fp_device_delete_print:
 * @device: a #FpDevice
 * @enrolled_print: a #FpPrint to delete
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to delete a print from the device.
 * The callback will be called once the operation has finished. Retrieve
 * the result with fp_device_delete_print_finish().
 *
 * This only makes sense on devices that store prints on-chip, but is safe
 * to always call.
 */
void
fp_device_delete_print (FpDevice           *device,
                        FpPrint            *enrolled_print,
                        GCancellable       *cancellable,
                        GAsyncReadyCallback callback,
                        gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  /* Succeed immediately if delete is not implemented. */
  if (!cls->delete || !(priv->features & FP_DEVICE_FEATURE_STORAGE_DELETE))
    {
      g_task_return_boolean (task, TRUE);
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_DELETE;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  g_task_set_task_data (priv->current_task,
                        g_object_ref (enrolled_print),
                        g_object_unref);

  cls->delete (device);
}

/**
 * fp_device_delete_print_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to delete an enrolled print.
 *
 * See fp_device_delete_print().
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_delete_print_finish (FpDevice     *device,
                               GAsyncResult *result,
                               GError      **error)
{
  return g_task_propagate_boolean (G_TASK (result), error);
}

/**
 * fp_device_list_prints:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to list all prints stored on the device.
 * This only makes sense on devices that store prints on-chip.
 *
 * Retrieve the result with fp_device_list_prints_finish().
 */
void
fp_device_list_prints (FpDevice           *device,
                       GCancellable       *cancellable,
                       GAsyncReadyCallback callback,
                       gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  if (!cls->list || !(priv->features & FP_DEVICE_FEATURE_STORAGE))
    {
      g_task_return_error (task,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Device has no storage"));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_LIST;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  cls->list (device);
}

/**
 * fp_device_list_prints_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to list all device stored prints.
 *
 * See fp_device_list_prints().
 *
 * Returns: (element-type FpPrint) (transfer container): Array of prints or %NULL on error
 */
GPtrArray *
fp_device_list_prints_finish (FpDevice     *device,
                              GAsyncResult *result,
                              GError      **error)
{
  return g_task_propagate_pointer (G_TASK (result), error);
}

/**
 * fp_device_clear_storage:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @callback: the function to call on completion
 * @user_data: the data to pass to @callback
 *
 * Start an asynchronous operation to delete all prints from the device.
 * The callback will be called once the operation has finished. Retrieve
 * the result with fp_device_clear_storage_finish().
 *
 * This only makes sense on devices that store prints on-chip, but is safe
 * to always call.
 */
void
fp_device_clear_storage (FpDevice           *device,
                         GCancellable       *cancellable,
                         GAsyncReadyCallback callback,
                         gpointer            user_data)
{
  g_autoptr(GTask) task = NULL;
  FpDevicePrivate *priv = fp_device_get_instance_private (device);
  FpDeviceClass *cls = FP_DEVICE_GET_CLASS (device);

  task = g_task_new (device, cancellable, callback, user_data);
  if (g_task_return_error_if_cancelled (task))
    return;

  if (!priv->is_open)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_NOT_OPEN));
      return;
    }

  if (priv->current_task)
    {
      g_task_return_error (task,
                           fpi_device_error_new (FP_DEVICE_ERROR_BUSY));
      return;
    }

  if (!(priv->features & FP_DEVICE_FEATURE_STORAGE))
    {
      g_task_return_error (task,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Device has no storage."));
      return;
    }

  if (!(priv->features & FP_DEVICE_FEATURE_STORAGE_CLEAR))
    {
      g_task_return_error (task,
                           fpi_device_error_new_msg (FP_DEVICE_ERROR_NOT_SUPPORTED,
                                                     "Device doesn't support clearing storage."));
      return;
    }

  priv->current_action = FPI_DEVICE_ACTION_CLEAR_STORAGE;
  priv->current_task = g_steal_pointer (&task);
  maybe_cancel_on_cancelled (device, cancellable);

  cls->clear_storage (device);

  return;
}

/**
 * fp_device_clear_storage_finish:
 * @device: A #FpDevice
 * @result: A #GAsyncResult
 * @error: Return location for errors, or %NULL to ignore
 *
 * Finish an asynchronous operation to delete all enrolled prints.
 *
 * See fp_device_clear_storage().
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_clear_storage_finish (FpDevice     *device,
                                GAsyncResult *result,
                                GError      **error)
{
  return g_task_propagate_boolean (G_TASK (result), error);
}

static void
async_result_ready (GObject *source_object, GAsyncResult *res, gpointer user_data)
{
  GTask **task = user_data;

  *task = g_object_ref (G_TASK (res));
}

/**
 * fp_device_open_sync:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Open the device synchronously.
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_open_sync (FpDevice     *device,
                     GCancellable *cancellable,
                     GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_open (device, cancellable, async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_open_finish (device, task, error);
}

/**
 * fp_device_close_sync:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Close the device synchronously.
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_close_sync (FpDevice     *device,
                      GCancellable *cancellable,
                      GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_close (device, cancellable, async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_close_finish (device, task, error);
}

/**
 * fp_device_enroll_sync:
 * @device: a #FpDevice
 * @template_print: (transfer floating): A #FpPrint to fill in or use
 *   as a template.
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @progress_cb: (nullable) (scope call): progress reporting callback
 * @progress_data: user data for @progress_cb
 * @error: Return location for errors, or %NULL to ignore
 *
 * Enroll a new print. See fp_device_enroll(). It is undefined whether
 * @template_print is updated or a newly created #FpPrint is returned.
 *
 * Returns:(transfer full): A #FpPrint on success, %NULL otherwise
 */
FpPrint *
fp_device_enroll_sync (FpDevice        *device,
                       FpPrint         *template_print,
                       GCancellable    *cancellable,
                       FpEnrollProgress progress_cb,
                       gpointer         progress_data,
                       GError         **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_enroll (device, template_print, cancellable,
                    progress_cb, progress_data, NULL,
                    async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_enroll_finish (device, task, error);
}

/**
 * fp_device_verify_sync:
 * @device: a #FpDevice
 * @enrolled_print: a #FpPrint to verify
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @match_cb: (nullable) (scope call): match reporting callback
 * @match_data: (closure match_cb): user data for @match_cb
 * @match: (out): Whether the user presented the correct finger
 * @print: (out) (transfer full) (nullable): Location to store the scanned print, or %NULL to ignore
 * @error: Return location for errors, or %NULL to ignore
 *
 * Verify a given print synchronously.
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_verify_sync (FpDevice     *device,
                       FpPrint      *enrolled_print,
                       GCancellable *cancellable,
                       FpMatchCb     match_cb,
                       gpointer      match_data,
                       gboolean     *match,
                       FpPrint     **print,
                       GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_verify (device,
                    enrolled_print,
                    cancellable,
                    match_cb, match_data, NULL,
                    async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_verify_finish (device, task, match, print, error);
}

/**
 * fp_device_identify_sync:
 * @device: a #FpDevice
 * @prints: (element-type FpPrint) (transfer none): #GPtrArray of #FpPrint
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @match_cb: (nullable) (scope call): match reporting callback
 * @match_data: (closure match_cb): user data for @match_cb
 * @match: (out) (transfer full) (nullable): Location for the matched #FpPrint, or %NULL
 * @print: (out) (transfer full) (nullable): Location for the new #FpPrint, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Identify a print synchronously.
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_identify_sync (FpDevice     *device,
                         GPtrArray    *prints,
                         GCancellable *cancellable,
                         FpMatchCb     match_cb,
                         gpointer      match_data,
                         FpPrint     **match,
                         FpPrint     **print,
                         GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_identify (device,
                      prints,
                      cancellable,
                      match_cb, match_data, NULL,
                      async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_identify_finish (device, task, match, print, error);
}


/**
 * fp_device_capture_sync:
 * @device: a #FpDevice
 * @wait_for_finger: Whether to wait for a finger or not
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Start an synchronous operation to capture an image.
 *
 * Returns: (transfer full): A new #FpImage or %NULL on error
 */
FpImage *
fp_device_capture_sync (FpDevice     *device,
                        gboolean      wait_for_finger,
                        GCancellable *cancellable,
                        GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_capture (device,
                     wait_for_finger,
                     cancellable,
                     async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_capture_finish (device, task, error);
}

/**
 * fp_device_delete_print_sync:
 * @device: a #FpDevice
 * @enrolled_print: a #FpPrint to verify
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Delete a given print from the device.
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_delete_print_sync (FpDevice     *device,
                             FpPrint      *enrolled_print,
                             GCancellable *cancellable,
                             GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_delete_print (device,
                          enrolled_print,
                          cancellable,
                          async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_delete_print_finish (device, task, error);
}

/**
 * fp_device_list_prints_sync:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * List device stored prints synchronously.
 *
 * Returns: (element-type FpPrint) (transfer container): Array of prints, or %NULL on error
 */
GPtrArray *
fp_device_list_prints_sync (FpDevice     *device,
                            GCancellable *cancellable,
                            GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_list_prints (device,
                         NULL,
                         async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_list_prints_finish (device, task, error);
}

/**
 * fp_device_clear_storage_sync:
 * @device: a #FpDevice
 * @cancellable: (nullable): a #GCancellable, or %NULL
 * @error: Return location for errors, or %NULL to ignore
 *
 * Clear sensor storage.
 *
 * Returns: (type void): %FALSE on error, %TRUE otherwise
 */
gboolean
fp_device_clear_storage_sync (FpDevice     *device,
                              GCancellable *cancellable,
                              GError      **error)
{
  g_autoptr(GAsyncResult) task = NULL;

  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  fp_device_clear_storage (device,
                           cancellable,
                           async_result_ready, &task);
  while (!task)
    g_main_context_iteration (NULL, TRUE);

  return fp_device_clear_storage_finish (device, task, error);
}

/**
 * fp_device_get_features:
 * @device: a #FpDevice
 *
 * Gets the #FpDeviceFeature's supported by the @device.
 *
 * Returns: #FpDeviceFeature flags of supported features
 */
FpDeviceFeature
fp_device_get_features (FpDevice *device)
{
  FpDevicePrivate *priv = fp_device_get_instance_private (device);

  g_return_val_if_fail (FP_IS_DEVICE (device), FP_DEVICE_FEATURE_NONE);

  return priv->features;
}

/**
 * fp_device_has_feature:
 * @device: a #FpDevice
 * @feature: #FpDeviceFeature flags to check against device supported features
 *
 * Checks if @device supports the requested #FpDeviceFeature's.
 * See fp_device_get_features()
 *
 * Returns: %TRUE if supported, %FALSE otherwise
 */
gboolean
fp_device_has_feature (FpDevice       *device,
                       FpDeviceFeature feature)
{
  g_return_val_if_fail (FP_IS_DEVICE (device), FALSE);

  if (feature == FP_DEVICE_FEATURE_NONE)
    return fp_device_get_features (device) == feature;

  return (fp_device_get_features (device) & feature) == feature;
}
