/*
 * Unit tests for the internal fingerprint drivers API
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

#include "fp-device.h"
#include "fp-enums.h"
#include <libfprint/fprint.h>

#define FP_COMPONENT "device"

#include "fpi-device.h"
#include "fpi-compat.h"
#include "fpi-log.h"
#include "test-device-fake.h"

/* Utility functions */

typedef FpDevice FpAutoCloseDevice;

static FpAutoCloseDevice *
auto_close_fake_device_new (void)
{
  FpAutoCloseDevice *device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  return device;
}

static void
auto_close_fake_device_free (FpAutoCloseDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  if (fake_dev->return_action_error)
    {
      fake_dev->return_action_error = FALSE;
      fake_dev->ret_error = NULL;
    }

  if (fp_device_is_open (device))
    g_assert_true (fp_device_close_sync (device, NULL, NULL));

  g_object_unref (device);
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC (FpAutoCloseDevice, auto_close_fake_device_free)

typedef FpDeviceClass FpAutoResetClass;
static FpAutoResetClass default_fake_dev_class = {0};

static FpAutoResetClass *
auto_reset_device_class (void)
{
  g_autoptr(FpDeviceClass) type_class = NULL;
  FpDeviceClass *dev_class = g_type_class_peek_static (FPI_TYPE_DEVICE_FAKE);

  if (!dev_class)
    {
      type_class = g_type_class_ref (FPI_TYPE_DEVICE_FAKE);
      dev_class = type_class;
      g_assert_nonnull (dev_class);
    }

  default_fake_dev_class = *dev_class;

  return dev_class;
}

static void
auto_reset_device_class_cleanup (FpAutoResetClass *dev_class)
{
  *dev_class = default_fake_dev_class;

  g_assert_cmpint (memcmp (dev_class, &default_fake_dev_class,
                           sizeof (FpAutoResetClass)), ==, 0);
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC (FpAutoResetClass, auto_reset_device_class_cleanup)


static void
assert_equal_galleries (GPtrArray *g1,
                        GPtrArray *g2)
{
  unsigned i;

  g_assert ((g1 && g2) || (!g1 || !g1));

  if (g1 == g2)
    return;

  g_assert_cmpuint (g1->len, ==, g2->len);

  for (i = 0; i < g1->len; i++)
    {
      FpPrint *print = g_ptr_array_index (g1, i);

      g_assert_true (g_ptr_array_find_with_equal_func (g2, print, (GEqualFunc)
                                                       fp_print_equal, NULL));
    }
}

static void
on_device_notify (FpDevice *device, GParamSpec *spec, gpointer user_data)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = on_device_notify;
  fake_dev->user_data = g_param_spec_ref (spec);
}

static FpPrint *
make_fake_print (FpDevice *device,
                 GVariant *print_data)
{
  FpPrint *enrolled_print = fp_print_new (device);

  fpi_print_set_type (enrolled_print, FPI_PRINT_RAW);

  if (!print_data)
    print_data = g_variant_new_string ("Test print private data");
  g_object_set (G_OBJECT (enrolled_print), "fpi-data", print_data, NULL);

  return enrolled_print;
}

static FpPrint *
make_fake_print_reffed (FpDevice *device,
                        GVariant *print_data)
{
  return g_object_ref_sink (make_fake_print (device, print_data));
}

static GPtrArray *
make_fake_prints_gallery (FpDevice *device,
                          size_t    size)
{
  GPtrArray *array;
  size_t i;

  array = g_ptr_array_new_full (size, g_object_unref);

  for (i = 0; i < size; i++)
    g_ptr_array_add (array, make_fake_print_reffed (device, g_variant_new_uint64 (i)));

  return array;
}

/* Tests */

static void
test_driver_get_driver (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->id = "test-fpi-device-driver";
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_cmpstr (fp_device_get_driver (device), ==, "test-fpi-device-driver");
}

static void
test_driver_get_device_id (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_cmpstr (fp_device_get_device_id (device), ==, "0");
}

static void
test_driver_get_name (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->full_name = "Test Device Full Name!";
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_cmpstr (fp_device_get_name (device), ==, "Test Device Full Name!");
}

static void
test_driver_is_open (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_false (fp_device_is_open (device));
  fp_device_open_sync (device, NULL, NULL);
  g_assert_true (fp_device_is_open (device));
  fp_device_close_sync (FP_DEVICE (device), NULL, NULL);
  g_assert_false (fp_device_is_open (device));
}

static void
test_driver_get_scan_type_press (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->scan_type = FP_SCAN_TYPE_PRESS;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_assert_cmpuint (fp_device_get_scan_type (device), ==, FP_SCAN_TYPE_PRESS);
}

static void
test_driver_get_scan_type_swipe (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->scan_type = FP_SCAN_TYPE_SWIPE;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_assert_cmpuint (fp_device_get_scan_type (device), ==, FP_SCAN_TYPE_SWIPE);
}

static void
test_driver_set_scan_type_press (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GParamSpec) pspec = NULL;
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_signal_connect (device, "notify::scan-type", G_CALLBACK (on_device_notify), NULL);

  fpi_device_set_scan_type (device, FP_SCAN_TYPE_PRESS);
  g_assert_cmpuint (fp_device_get_scan_type (device), ==, FP_SCAN_TYPE_PRESS);
  g_assert (fake_dev->last_called_function == on_device_notify);

  pspec = g_steal_pointer (&fake_dev->user_data);
  g_assert_cmpstr (pspec->name, ==, "scan-type");
}

static void
test_driver_set_scan_type_swipe (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GParamSpec) pspec = NULL;
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_signal_connect (device, "notify::scan-type", G_CALLBACK (on_device_notify), NULL);

  fpi_device_set_scan_type (device, FP_SCAN_TYPE_SWIPE);
  g_assert_cmpuint (fp_device_get_scan_type (device), ==, FP_SCAN_TYPE_SWIPE);
  g_assert (fake_dev->last_called_function == on_device_notify);

  pspec = g_steal_pointer (&fake_dev->user_data);
  g_assert_cmpstr (pspec->name, ==, "scan-type");
}

static void
test_driver_finger_status_inactive (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpFingerStatusFlags finger_status;

  g_signal_connect (device, "notify::finger-status", G_CALLBACK (on_device_notify), NULL);

  g_assert_false (fpi_device_report_finger_status (device, FP_FINGER_STATUS_NONE));
  g_assert_cmpuint (fp_device_get_finger_status (device), ==, FP_FINGER_STATUS_NONE);

  g_object_get (fake_dev, "finger-status", &finger_status, NULL);
  g_assert_cmpuint (finger_status, ==, FP_FINGER_STATUS_NONE);

  g_assert (fake_dev->last_called_function != on_device_notify);
  g_assert_null (g_steal_pointer (&fake_dev->user_data));
}

static void
test_driver_finger_status_needed (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GParamSpec) pspec = NULL;
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpFingerStatusFlags finger_status;

  g_signal_connect (device, "notify::finger-status", G_CALLBACK (on_device_notify), NULL);

  g_assert_true (fpi_device_report_finger_status (device, FP_FINGER_STATUS_NEEDED));
  g_assert_cmpuint (fp_device_get_finger_status (device), ==, FP_FINGER_STATUS_NEEDED);

  g_object_get (fake_dev, "finger-status", &finger_status, NULL);
  g_assert_cmpuint (finger_status, ==, FP_FINGER_STATUS_NEEDED);

  g_assert (fake_dev->last_called_function == on_device_notify);
  pspec = g_steal_pointer (&fake_dev->user_data);
  g_assert_cmpstr (pspec->name, ==, "finger-status");

  fake_dev->last_called_function = NULL;
  g_assert_false (fpi_device_report_finger_status (device, FP_FINGER_STATUS_NEEDED));
  g_assert_null (fake_dev->last_called_function);
  g_assert_null (g_steal_pointer (&fake_dev->user_data));
}

static void
test_driver_finger_status_present (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GParamSpec) pspec = NULL;
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpFingerStatusFlags finger_status;

  g_signal_connect (device, "notify::finger-status", G_CALLBACK (on_device_notify), NULL);

  g_assert_true (fpi_device_report_finger_status (device, FP_FINGER_STATUS_PRESENT));
  g_assert_cmpuint (fp_device_get_finger_status (device), ==, FP_FINGER_STATUS_PRESENT);

  g_object_get (fake_dev, "finger-status", &finger_status, NULL);
  g_assert_cmpuint (finger_status, ==, FP_FINGER_STATUS_PRESENT);

  g_assert (fake_dev->last_called_function == on_device_notify);
  pspec = g_steal_pointer (&fake_dev->user_data);
  g_assert_cmpstr (pspec->name, ==, "finger-status");

  fake_dev->last_called_function = NULL;
  g_assert_false (fpi_device_report_finger_status (device, FP_FINGER_STATUS_PRESENT));
  g_assert_null (fake_dev->last_called_function);
  g_assert_null (g_steal_pointer (&fake_dev->user_data));
}

static void
driver_finger_status_changes_check (FpDevice *device, gboolean add)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_autoptr(GFlagsClass) status_class = g_type_class_ref (FP_TYPE_FINGER_STATUS_FLAGS);
  guint expected_status;
  guint initial_value;
  guint i;
  gulong signal_id;

  if (add)
    initial_value = FP_FINGER_STATUS_NONE;
  else
    initial_value = status_class->mask;

  g_assert_cmpuint (fp_device_get_finger_status (device), ==, initial_value);

  signal_id = g_signal_connect (device, "notify::finger-status",
                                G_CALLBACK (on_device_notify), NULL);

  for (i = 0, expected_status = initial_value; i < status_class->n_values; ++i)
    {
      g_autoptr(GParamSpec) pspec = NULL;
      FpFingerStatusFlags finger_status = status_class->values[i].value;
      FpFingerStatusFlags added_status = add ? finger_status : FP_FINGER_STATUS_NONE;
      FpFingerStatusFlags removed_status = add ? FP_FINGER_STATUS_NONE : finger_status;
      gboolean ret;

      fake_dev->last_called_function = NULL;
      ret = fpi_device_report_finger_status_changes (device,
                                                     added_status,
                                                     removed_status);
      if (finger_status != FP_FINGER_STATUS_NONE)
        g_assert_true (ret);
      else
        g_assert_false (ret);

      expected_status |= added_status;
      expected_status &= ~removed_status;

      g_assert_cmpuint (fp_device_get_finger_status (device), ==, expected_status);

      if (finger_status != FP_FINGER_STATUS_NONE)
        {
          g_assert (fake_dev->last_called_function == on_device_notify);
          pspec = g_steal_pointer (&fake_dev->user_data);
          g_assert_cmpstr (pspec->name, ==, "finger-status");
        }

      fake_dev->last_called_function = NULL;
      g_assert_false (fpi_device_report_finger_status_changes (device,
                                                               added_status,
                                                               removed_status));
      g_assert_null (fake_dev->last_called_function);
      g_assert_null (g_steal_pointer (&fake_dev->user_data));
    }

  if (add)
    g_assert_cmpuint (fp_device_get_finger_status (device), ==, status_class->mask);
  else
    g_assert_cmpuint (fp_device_get_finger_status (device), ==, FP_FINGER_STATUS_NONE);

  fake_dev->last_called_function = NULL;
  g_assert_false (fpi_device_report_finger_status_changes (device,
                                                           FP_FINGER_STATUS_NONE,
                                                           FP_FINGER_STATUS_NONE));

  g_assert_null (fake_dev->last_called_function);
  g_assert_null (g_steal_pointer (&fake_dev->user_data));

  g_signal_handler_disconnect (device, signal_id);
}

static void
test_driver_finger_status_changes (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  driver_finger_status_changes_check (device, TRUE);
  driver_finger_status_changes_check (device, FALSE);
}

static void
test_driver_get_nr_enroll_stages (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;
  int expected_stages = g_random_int_range (G_MININT32, G_MAXINT32);

  dev_class->nr_enroll_stages = expected_stages;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_cmpint (fp_device_get_nr_enroll_stages (device), ==, expected_stages);
}

static void
test_driver_set_nr_enroll_stages (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(GParamSpec) pspec = NULL;
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  int expected_stages = g_random_int_range (1, G_MAXINT32);

  g_signal_connect (device, "notify::nr-enroll-stages", G_CALLBACK (on_device_notify), NULL);
  fpi_device_set_nr_enroll_stages (device, expected_stages);

  g_assert_cmpint (fp_device_get_nr_enroll_stages (device), ==, expected_stages);
  g_assert (fake_dev->last_called_function == on_device_notify);

  pspec = g_steal_pointer (&fake_dev->user_data);
  g_assert_cmpstr (pspec->name, ==, "nr-enroll-stages");

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*enroll_stages > 0*");
  fpi_device_set_nr_enroll_stages (device, 0);
  g_assert_cmpint (fp_device_get_nr_enroll_stages (device), ==, expected_stages);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*enroll_stages > 0*");
  fpi_device_set_nr_enroll_stages (device, -2);
  g_assert_cmpint (fp_device_get_nr_enroll_stages (device), ==, expected_stages);
  g_test_assert_expected_messages ();
}

static void
test_driver_get_usb_device (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->type = FP_DEVICE_TYPE_USB;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, "fpi-usb-device", NULL, NULL);
  g_assert_null (fpi_device_get_usb_device (device));

  g_clear_object (&device);
  dev_class->type = FP_DEVICE_TYPE_VIRTUAL;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*type*FP_DEVICE_TYPE_USB*failed*");
  g_assert_null (fpi_device_get_usb_device (device));
  g_test_assert_expected_messages ();
}

static void
test_driver_get_virtual_env (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->type = FP_DEVICE_TYPE_VIRTUAL;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, "fpi-environ", "TEST_VIRTUAL_ENV_GETTER", NULL);
  g_assert_cmpstr (fpi_device_get_virtual_env (device), ==, "TEST_VIRTUAL_ENV_GETTER");

  g_clear_object (&device);
  dev_class->type = FP_DEVICE_TYPE_USB;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*type*FP_DEVICE_TYPE_VIRTUAL*failed*");
  g_assert_null (fpi_device_get_virtual_env (device));
  g_test_assert_expected_messages ();
}

static void
test_driver_get_driver_data (void)
{
  g_autoptr(FpDevice) device = NULL;
  guint64 driver_data;

  driver_data = g_random_int ();
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, "fpi-driver-data", driver_data, NULL);
  g_assert_cmpuint (fpi_device_get_driver_data (device), ==, driver_data);
}

static void
test_driver_features_probe_updates (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev;

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);

  /* Effectively clears FP_DEVICE_FEATURE_STORAGE_DELETE */
  fake_dev = FPI_DEVICE_FAKE (device);
  fake_dev->probe_features_update = FP_DEVICE_FEATURE_STORAGE_LIST | FP_DEVICE_FEATURE_STORAGE_DELETE;
  fake_dev->probe_features_value = FP_DEVICE_FEATURE_STORAGE_LIST;

  g_async_initable_init_async (G_ASYNC_INITABLE (device),
                               G_PRIORITY_DEFAULT, NULL, NULL, NULL);
  while (g_main_context_iteration (NULL, FALSE))
    continue;

  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_CAPTURE));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_VERIFY));
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_DUPLICATES_CHECK));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE_LIST));
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE_DELETE));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE_CLEAR));

  g_assert_cmpuint (fp_device_get_features (device),
                    ==,
                    FP_DEVICE_FEATURE_CAPTURE |
                    FP_DEVICE_FEATURE_IDENTIFY |
                    FP_DEVICE_FEATURE_VERIFY |
                    FP_DEVICE_FEATURE_STORAGE |
                    FP_DEVICE_FEATURE_STORAGE_LIST |
                    FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);

  g_async_initable_init_async (G_ASYNC_INITABLE (device),
                               G_PRIORITY_DEFAULT, NULL, NULL, NULL);
  while (g_main_context_iteration (NULL, FALSE))
    continue;

  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_CAPTURE));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_VERIFY));
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_DUPLICATES_CHECK));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE_LIST));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE_DELETE));
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE_CLEAR));

  g_assert_cmpuint (fp_device_get_features (device),
                    ==,
                    FP_DEVICE_FEATURE_CAPTURE |
                    FP_DEVICE_FEATURE_IDENTIFY |
                    FP_DEVICE_FEATURE_VERIFY |
                    FP_DEVICE_FEATURE_STORAGE |
                    FP_DEVICE_FEATURE_STORAGE_LIST |
                    FP_DEVICE_FEATURE_STORAGE_DELETE |
                    FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_none (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->list = NULL;
  dev_class->capture = NULL;
  dev_class->verify = NULL;
  dev_class->identify = NULL;
  dev_class->delete = NULL;
  dev_class->clear_storage = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, ==, FP_DEVICE_FEATURE_NONE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_capture (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->capture = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_verify (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->verify = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_identify (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->identify = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_storage (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->delete = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_list (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->list = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_delete (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->delete = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
test_driver_initial_features_no_clear (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();

  dev_class->clear_storage = NULL;
  dev_class->features = FP_DEVICE_FEATURE_NONE;

  fpi_device_class_auto_initialize_features (dev_class);

  g_assert_cmpuint (dev_class->features, !=, FP_DEVICE_FEATURE_NONE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_CAPTURE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_IDENTIFY);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_VERIFY);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_DUPLICATES_CHECK);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_LIST);
  g_assert_true (dev_class->features & FP_DEVICE_FEATURE_STORAGE_DELETE);
  g_assert_false (dev_class->features & FP_DEVICE_FEATURE_STORAGE_CLEAR);
}

static void
on_driver_probe_async (GObject *initable, GAsyncResult *res, gpointer user_data)
{
  g_autoptr(GError) error = NULL;
  FpDevice **out_device = user_data;
  FpDevice *device;
  FpDeviceClass *dev_class;
  FpiDeviceFake *fake_dev;

  device = FP_DEVICE (g_async_initable_new_finish (G_ASYNC_INITABLE (initable), res, &error));
  dev_class = FP_DEVICE_GET_CLASS (device);
  fake_dev = FPI_DEVICE_FAKE (device);

  g_assert (fake_dev->last_called_function == dev_class->probe);
  g_assert_no_error (error);

  g_assert_false (fp_device_is_open (device));

  *out_device = device;
}

static void
test_driver_probe (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->id = "Probed device ID";
  dev_class->full_name = "Probed device name";
  g_async_initable_new_async (FPI_TYPE_DEVICE_FAKE, G_PRIORITY_DEFAULT, NULL,
                              on_driver_probe_async, &device, NULL);

  while (!FP_IS_DEVICE (device))
    g_main_context_iteration (NULL, TRUE);

  g_assert_false (fp_device_is_open (device));
  g_assert_cmpstr (fp_device_get_device_id (device), ==, "Probed device ID");
  g_assert_cmpstr (fp_device_get_name (device), ==, "Probed device name");
}

static void
fake_device_probe_error (FpDevice *device)
{
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);

  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_PROBE);

  fpi_device_probe_complete (device, dev_class->id, dev_class->full_name,
                             fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED));
}

static void
fake_device_probe_action_error (FpDevice *device)
{
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_PROBE);

  fpi_device_action_error (device, fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED));
}

static void
on_driver_probe_error_async (GObject *initable, GAsyncResult *res, gpointer user_data)
{
  g_autoptr(GError) error = NULL;
  gboolean *out_done = user_data;
  FpDevice *device;

  device = FP_DEVICE (g_async_initable_new_finish (G_ASYNC_INITABLE (initable), res, &error));
  g_assert_null (device);

  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED);
  *out_done = TRUE;
}

static void
test_driver_probe_error (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  gboolean done = FALSE;

  dev_class->id = "Error device ID";
  dev_class->probe = fake_device_probe_error;
  g_async_initable_new_async (FPI_TYPE_DEVICE_FAKE, G_PRIORITY_DEFAULT, NULL,
                              on_driver_probe_error_async, &done, NULL);

  while (!done)
    g_main_context_iteration (NULL, TRUE);
}

static void
test_driver_probe_action_error (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  gboolean done = FALSE;

  dev_class->id = "Error device ID";
  dev_class->probe = fake_device_probe_action_error;
  g_async_initable_new_async (FPI_TYPE_DEVICE_FAKE, G_PRIORITY_DEFAULT, NULL,
                              on_driver_probe_error_async, &done, NULL);

  while (!done)
    g_main_context_iteration (NULL, TRUE);
}

static void
test_driver_open (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert (fake_dev->last_called_function != dev_class->probe);

  g_assert_true (fp_device_open_sync (device, NULL, &error));
  g_assert (fake_dev->last_called_function == dev_class->open);
  g_assert_no_error (error);
  g_assert_true (fp_device_is_open (device));

  g_assert_true (fp_device_close_sync (FP_DEVICE (device), NULL, &error));
  g_assert_no_error (error);
}

static void
test_driver_open_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  g_assert_false (fp_device_open_sync (device, NULL, &error));
  g_assert (fake_dev->last_called_function == dev_class->open);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_false (fp_device_is_open (device));
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
}

static void
test_driver_close (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_true (fp_device_close_sync (device, NULL, &error));
  g_assert (fake_dev->last_called_function == dev_class->close);

  g_assert_no_error (error);
  g_assert_false (fp_device_is_open (device));
}

static void
test_driver_close_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  g_assert_false (fp_device_close_sync (device, NULL, &error));

  g_assert (fake_dev->last_called_function == dev_class->close);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_false (fp_device_is_open (device));
}

static void
test_driver_enroll (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) template_print = fp_print_new (device);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *out_print = NULL;

  out_print =
    fp_device_enroll_sync (device, template_print, NULL, NULL, NULL, &error);

  g_assert (fake_dev->last_called_function == dev_class->enroll);
  g_assert (fake_dev->action_data == template_print);

  g_assert_no_error (error);
  g_assert (out_print == template_print);
}

static void
test_driver_enroll_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *template_print = fp_print_new (device);
  FpPrint *out_print = NULL;

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  out_print =
    fp_device_enroll_sync (device, template_print, NULL, NULL, NULL, &error);

  g_assert (fake_dev->last_called_function == dev_class->enroll);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_null (out_print);
}

static void
test_driver_enroll_complete_simple (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = test_driver_enroll_complete_simple;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_ENROLL);

  fpi_device_enroll_complete (device, fake_dev->ret_print, fake_dev->ret_error);
}

static void
test_driver_enroll_error_no_print (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(FpPrint) out_print = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->enroll = test_driver_enroll_complete_simple;
  device = auto_close_fake_device_new ();
  fake_dev = FPI_DEVICE_FAKE (device);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver did not provide a valid print and failed to provide an error*");
  out_print =
    fp_device_enroll_sync (device, fp_print_new (device), NULL, NULL, NULL, &error);

  g_test_assert_expected_messages ();
  g_assert (fake_dev->last_called_function == dev_class->enroll);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_null (out_print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver passed an error but also provided a print, returning error*");

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  fake_dev->ret_print = make_fake_print_reffed (device, NULL);
  g_object_add_weak_pointer (G_OBJECT (fake_dev->ret_print),
                             (gpointer) (&fake_dev->ret_print));
  out_print =
    fp_device_enroll_sync (device, fp_print_new (device), NULL, NULL, NULL, &error);

  g_test_assert_expected_messages ();
  g_assert (fake_dev->last_called_function == dev_class->enroll);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_true (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_null (out_print);
  g_assert_null (fake_dev->ret_print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver did not set the type on the returned print*");

  fake_dev->ret_error = NULL;
  fake_dev->ret_print = fp_print_new (device); /* Type not set. */
  g_object_add_weak_pointer (G_OBJECT (fake_dev->ret_print),
                             (gpointer) (&fake_dev->ret_print));
  out_print =
    fp_device_enroll_sync (device, fp_print_new (device), NULL, NULL, NULL, &error);

  g_test_assert_expected_messages ();
  g_assert (fake_dev->last_called_function == dev_class->enroll);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_null (out_print);
  g_assert_null (fake_dev->ret_print);
  g_clear_error (&error);
}

typedef struct
{
  gint     completed_stages;
  FpPrint *print;
  GError  *error;
} ExpectedEnrollData;

static void
test_driver_enroll_progress_callback (FpDevice *device,
                                      gint      completed_stages,
                                      FpPrint  *print,
                                      gpointer  user_data,
                                      GError   *error)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  ExpectedEnrollData *expected_data = user_data;

  g_assert_cmpint (expected_data->completed_stages, ==, completed_stages);
  g_assert (expected_data->print == print);
  g_assert_true (print == NULL || FP_IS_PRINT (print));
  g_assert (expected_data->error == error);

  fake_dev->last_called_function = test_driver_enroll_progress_callback;
}

static void
test_driver_enroll_progress_vfunc (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  ExpectedEnrollData *expected_data = fake_dev->user_data;

  g_autoptr(GError) error = NULL;

  expected_data->completed_stages =
    g_random_int_range (fp_device_get_nr_enroll_stages (device), G_MAXINT32);
  expected_data->print = fp_print_new (device);
  expected_data->error = NULL;

  g_object_add_weak_pointer (G_OBJECT (expected_data->print),
                             (gpointer) & expected_data->print);

  fpi_device_enroll_progress (device, expected_data->completed_stages,
                              expected_data->print, expected_data->error);
  g_assert (fake_dev->last_called_function == test_driver_enroll_progress_callback);
  g_assert_null (expected_data->print);


  expected_data->completed_stages =
    g_random_int_range (fp_device_get_nr_enroll_stages (device), G_MAXINT32);
  expected_data->print = NULL;
  expected_data->error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);

  fpi_device_enroll_progress (device, expected_data->completed_stages,
                              expected_data->print, expected_data->error);
  g_assert (fake_dev->last_called_function == test_driver_enroll_progress_callback);


  expected_data->completed_stages =
    g_random_int_range (fp_device_get_nr_enroll_stages (device), G_MAXINT32);
  expected_data->print = make_fake_print_reffed (device,
                                                 g_variant_new_int32 (expected_data->completed_stages));
  expected_data->error = NULL;

  error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*error*FP_DEVICE_RETRY*failed");
  fpi_device_enroll_progress (device, expected_data->completed_stages,
                              expected_data->print, error);
  g_assert (fake_dev->last_called_function == test_driver_enroll_progress_callback);
  g_clear_object (&expected_data->print);
  g_test_assert_expected_messages ();

  expected_data->completed_stages =
    g_random_int_range (fp_device_get_nr_enroll_stages (device), G_MAXINT32);
  expected_data->print = NULL;
  expected_data->error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver passed an error and also provided a print*");
  fpi_device_enroll_progress (device, expected_data->completed_stages,
                              fp_print_new (device), expected_data->error);
  g_assert (fake_dev->last_called_function == test_driver_enroll_progress_callback);
  g_test_assert_expected_messages ();

  default_fake_dev_class.enroll (device);
  fake_dev->last_called_function = test_driver_enroll_progress_vfunc;
}

static void
test_driver_enroll_progress (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  G_GNUC_UNUSED g_autoptr(FpPrint) enrolled_print = NULL;
  ExpectedEnrollData expected_enroll_data = {0};
  FpiDeviceFake *fake_dev;

  dev_class->nr_enroll_stages = g_random_int_range (10, G_MAXINT32);
  dev_class->enroll = test_driver_enroll_progress_vfunc;
  device = auto_close_fake_device_new ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*FPI_DEVICE_ACTION_ENROLL*failed");
  fpi_device_enroll_progress (device, 0, NULL, NULL);
  g_test_assert_expected_messages ();

  fake_dev = FPI_DEVICE_FAKE (device);
  fake_dev->user_data = &expected_enroll_data;

  enrolled_print = fp_device_enroll_sync (device, fp_print_new (device), NULL,
                                          test_driver_enroll_progress_callback,
                                          &expected_enroll_data, NULL);

  g_assert (fake_dev->last_called_function == test_driver_enroll_progress_vfunc);
}

typedef struct
{
  gboolean   called;
  FpPrint   *match;
  FpPrint   *print;
  GPtrArray *gallery;
  GError    *error;
} MatchCbData;

static void
test_driver_match_data_clear (MatchCbData *data)
{
  data->called = FALSE;
  g_clear_object (&data->match);
  g_clear_object (&data->print);
  g_clear_error (&data->error);
}

static void
test_driver_match_data_free (MatchCbData *data)
{
  test_driver_match_data_clear (data);
  g_free (data);
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC (MatchCbData, test_driver_match_data_free);

static void
test_driver_match_cb (FpDevice *device,
                      FpPrint  *match,
                      FpPrint  *print,
                      gpointer  user_data,
                      GError   *error)
{
  MatchCbData *data = user_data;

  g_assert (data->called == FALSE);
  data->called = TRUE;
  if (match)
    data->match = g_object_ref (match);
  if (print)
    data->print = g_object_ref (print);
  if (error)
    {
      data->error = g_error_copy (error);
      g_assert_null (match);
    }

  if (match)
    g_assert_no_error (error);

  /* Compar gallery if this is an identify operation */
  if (data->gallery)
    {
      FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
      g_assert_false (fake_dev->action_data == data->gallery);
      assert_equal_galleries (fake_dev->action_data, data->gallery);
    }
}

static void
test_driver_verify (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  g_autoptr(FpPrint) out_print = NULL;
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean match;

  fake_dev->ret_result = FPI_MATCH_SUCCESS;
  g_assert_true (fp_device_verify_sync (device, enrolled_print, NULL,
                                        test_driver_match_cb, match_data,
                                        &match, &out_print, &error));

  g_assert (fake_dev->last_called_function == dev_class->verify);
  g_assert (fake_dev->action_data == enrolled_print);
  g_assert_no_error (error);

  g_assert_true (match_data->called);
  g_assert_nonnull (match_data->match);
  g_assert_true (match_data->print == out_print);
  g_assert_true (match_data->match == enrolled_print);

  g_assert (out_print == enrolled_print);
  g_assert_true (match);
}

static void
test_driver_verify_not_supported (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(FpPrint) out_print = NULL;
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  FpiDeviceFake *fake_dev;
  gboolean match;

  dev_class->features &= ~FP_DEVICE_FEATURE_VERIFY;

  device = auto_close_fake_device_new ();
  fake_dev = FPI_DEVICE_FAKE (device);
  fake_dev->last_called_function = NULL;

  enrolled_print = make_fake_print_reffed (device, g_variant_new_uint64 (3));
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         test_driver_match_cb, match_data,
                                         &match, &out_print, &error));

  g_assert_null (fake_dev->last_called_function);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED);

  g_assert_false (match_data->called);
  g_assert_no_error (match_data->error);

  g_assert_null (out_print);
  g_assert_false (match);
}

static void
test_driver_verify_fail (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(FpPrint) out_print = NULL;
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean match;

  enrolled_print = make_fake_print_reffed (device, g_variant_new_uint64 (3));
  fake_dev->ret_result = FPI_MATCH_FAIL;
  g_assert_true (fp_device_verify_sync (device, enrolled_print, NULL,
                                        test_driver_match_cb, match_data,
                                        &match, &out_print, &error));

  g_assert (fake_dev->last_called_function == dev_class->verify);
  g_assert_no_error (error);

  g_assert_true (match_data->called);
  g_assert_no_error (match_data->error);
  g_assert_true (match_data->print == out_print);
  g_assert_null (match_data->match);

  g_assert (out_print == enrolled_print);
  g_assert_false (match);
}

static void
test_driver_verify_retry (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  g_autoptr(FpPrint) out_print = NULL;
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean match;

  fake_dev->ret_result = FPI_MATCH_ERROR;
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL);
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         test_driver_match_cb, match_data,
                                         &match, &out_print, &error));

  g_assert_true (match_data->called);
  g_assert_null (match_data->match);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);

  g_assert (fake_dev->last_called_function == dev_class->verify);
  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_false (match);
}

static void
test_driver_verify_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  g_autoptr(FpPrint) out_print = NULL;
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean match;

  fake_dev->ret_result = FPI_MATCH_ERROR;
  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         test_driver_match_cb, match_data,
                                         &match, &out_print, &error));

  g_assert_false (match_data->called);
  g_assert_null (match_data->match);
  g_assert_no_error (match_data->error);

  g_assert (fake_dev->last_called_function == dev_class->verify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_false (match);
}

static void
fake_device_verify_immediate_complete (FpDevice *device)
{
  fpi_device_verify_complete (device, NULL);
}

static void
test_driver_verify_not_reported (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(GError) error = NULL;

  dev_class->verify = fake_device_verify_immediate_complete;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  enrolled_print = make_fake_print_reffed (device, NULL);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*reported successful verify complete*not report*result*");

  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         NULL, NULL,
                                         NULL, NULL, &error));

  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);

  g_test_assert_expected_messages ();
}

static void
fake_device_verify_complete_error (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  GError *complete_error = fake_dev->user_data;

  fake_dev->last_called_function = fake_device_verify_complete_error;

  fpi_device_verify_report (device, fake_dev->ret_result, fake_dev->ret_print, fake_dev->ret_error);
  fpi_device_verify_complete (device, complete_error);
}

static void
test_driver_verify_report_no_callback (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(GError) error = NULL;
  FpiDeviceFake *fake_dev;
  gboolean match;

  dev_class->verify = fake_device_verify_complete_error;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);
  enrolled_print = make_fake_print_reffed (device, NULL);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a verify error that was not in the retry domain*");

  fake_dev->ret_result = FPI_MATCH_ERROR;
  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED);
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         test_driver_match_cb, match_data,
                                         &match, &print, &error));

  g_test_assert_expected_messages ();

  g_assert_false (match_data->called);
  g_assert_null (match_data->match);
  g_assert_no_error (match_data->error);

  g_assert (fake_dev->last_called_function == dev_class->verify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_false (match);
}

static void
test_driver_verify_complete_retry (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(GError) error = NULL;
  FpiDeviceFake *fake_dev;
  gboolean match;

  dev_class->verify = fake_device_verify_complete_error;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);
  enrolled_print = make_fake_print_reffed (device, NULL);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported an error code without setting match result to error*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_result = FPI_MATCH_FAIL;
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL, test_driver_match_cb,
                                         match_data, &match, &print, &error));
  g_test_assert_expected_messages ();

  g_assert_true (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_false (match);
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_null (print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported an error code without setting match result to error*");
  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a retry error to fpi_device_verify_complete"
                         "*reporting general verification failure*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_result = FPI_MATCH_FAIL;
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);
  fake_dev->user_data = g_error_copy (fake_dev->ret_error);
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL, test_driver_match_cb,
                                         match_data, &match, &print, &error));

  g_test_assert_expected_messages ();
  g_assert_true (error != g_steal_pointer (&fake_dev->ret_error));
  g_steal_pointer (&fake_dev->user_data);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_false (match);
  g_assert_null (print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a retry error to fpi_device_verify_complete"
                         "*reporting general verification failure*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_result = FPI_MATCH_ERROR;
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);
  fake_dev->user_data = g_error_copy (fake_dev->ret_error);

  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL, test_driver_match_cb,
                                         match_data, &match, &print, &error));
  g_test_assert_expected_messages ();

  g_assert_true (error != g_steal_pointer (&fake_dev->ret_error));
  g_steal_pointer (&fake_dev->user_data);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_false (match);
  g_assert_null (print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported an error without specifying a retry "
                         "code, assuming general retry error*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_result = FPI_MATCH_ERROR;

  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL, test_driver_match_cb,
                                         match_data, &match, &print, &error));
  g_test_assert_expected_messages ();

  g_assert_true (error != g_steal_pointer (&fake_dev->ret_error));
  g_steal_pointer (&fake_dev->user_data);
  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);
  g_assert_false (match);
  g_assert_null (print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a print together with an error*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_result = FPI_MATCH_ERROR;
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);
  fake_dev->ret_print = make_fake_print (device, NULL);
  g_object_add_weak_pointer (G_OBJECT (fake_dev->ret_print),
                             (gpointer) (&fake_dev->ret_print));

  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL, test_driver_match_cb,
                                         match_data, &match, &print, &error));
  g_test_assert_expected_messages ();

  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_true (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_null (fake_dev->ret_print);
  g_assert_false (match);
  g_assert_null (print);
  g_clear_error (&error);
}

static void
fake_device_stub_identify (FpDevice *device)
{
}

static void
test_driver_supports_identify (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->identify = fake_device_stub_identify;

  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_supports_identify (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));
}

static void
test_driver_do_not_support_identify (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->features &= ~FP_DEVICE_FEATURE_IDENTIFY;

  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_false (fp_device_supports_identify (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));
}

static void
test_driver_identify (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(FpPrint) matched_print = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 500);
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *expected_matched;

  expected_matched = g_ptr_array_index (prints, g_random_int_range (0, 499));
  fp_print_set_description (expected_matched, "fake-verified");

  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_supports_identify (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));

  match_data->gallery = prints;

  fake_dev->ret_print = make_fake_print (device, NULL);
  g_assert_true (fp_device_identify_sync (device, prints, NULL,
                                          test_driver_match_cb, match_data,
                                          &matched_print, &print, &error));

  g_assert_true (match_data->called);
  g_assert_nonnull (match_data->match);
  g_assert_true (match_data->match == matched_print);
  g_assert_true (match_data->print == print);

  g_assert (fake_dev->last_called_function == dev_class->identify);
  g_assert_no_error (error);

  g_assert (print != NULL && print == fake_dev->ret_print);
  g_assert (expected_matched == matched_print);
}

static void
test_driver_identify_fail (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(FpPrint) matched_print = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 500);
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_supports_identify (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));

  fake_dev->ret_print = make_fake_print (device, NULL);
  g_assert_true (fp_device_identify_sync (device, prints, NULL,
                                          test_driver_match_cb, match_data,
                                          &matched_print, &print, &error));

  g_assert_true (match_data->called);
  g_assert_null (match_data->match);
  g_assert_no_error (match_data->error);
  g_assert_true (match_data->match == matched_print);
  g_assert_true (match_data->print == print);

  g_assert (fake_dev->last_called_function == dev_class->identify);
  g_assert_no_error (error);

  g_assert (print != NULL && print == fake_dev->ret_print);
  g_assert_null (matched_print);
}

static void
test_driver_identify_retry (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(FpPrint) matched_print = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 500);
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *expected_matched;

  expected_matched = g_ptr_array_index (prints, g_random_int_range (0, 499));
  fp_print_set_description (expected_matched, "fake-verified");

  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_supports_identify (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));

  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_GENERAL);
  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           test_driver_match_cb, match_data,
                                           &matched_print, &print, &error));

  g_assert_true (match_data->called);
  g_assert_null (match_data->match);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);

  g_assert (fake_dev->last_called_function == dev_class->identify);
  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_null (matched_print);
  g_assert_null (print);
}

static void
test_driver_identify_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(FpPrint) matched_print = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 500);
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *expected_matched;

  expected_matched = g_ptr_array_index (prints, g_random_int_range (0, 499));
  fp_print_set_description (expected_matched, "fake-verified");

  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_supports_identify (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_IDENTIFY));

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           test_driver_match_cb, match_data,
                                           &matched_print, &print, &error));

  g_assert_false (match_data->called);
  g_assert_null (match_data->match);
  g_assert_no_error (match_data->error);

  g_assert (fake_dev->last_called_function == dev_class->identify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_null (matched_print);
  g_assert_null (print);
}

static void
fake_device_identify_immediate_complete (FpDevice *device)
{
  fpi_device_identify_complete (device, NULL);
}

static void
test_driver_identify_not_reported (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(GPtrArray) prints = NULL;
  g_autoptr(GError) error = NULL;

  dev_class->identify = fake_device_identify_immediate_complete;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  prints = make_fake_prints_gallery (device, 500);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*reported successful identify complete*not report*result*");

  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           NULL, NULL,
                                           NULL, NULL, &error));

  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);

  g_test_assert_expected_messages ();
}

static void
fake_device_identify_complete_error (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  GError *complete_error = fake_dev->user_data;

  fake_dev->last_called_function = fake_device_identify_complete_error;

  fpi_device_identify_report (device, fake_dev->ret_match, fake_dev->ret_print, fake_dev->ret_error);
  fpi_device_identify_complete (device, complete_error);
}

static void
test_driver_identify_complete_retry (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(GPtrArray) prints = NULL;
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(FpPrint) match = NULL;
  g_autoptr(GError) error = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->identify = fake_device_identify_complete_error;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);
  prints = make_fake_prints_gallery (device, 500);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a retry error to fpi_device_identify_complete"
                         "*reporting general identification failure*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_TOO_SHORT);
  fake_dev->user_data = g_error_copy (fake_dev->ret_error);

  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           test_driver_match_cb, match_data,
                                           &match, &print, &error));
  g_test_assert_expected_messages ();

  g_assert_true (error != g_steal_pointer (&fake_dev->ret_error));
  g_steal_pointer (&fake_dev->user_data);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_TOO_SHORT);
  g_assert_null (match);
  g_assert_null (print);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a match to a print that was not in the gallery*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_match = make_fake_print_reffed (device, NULL);
  g_object_add_weak_pointer (G_OBJECT (fake_dev->ret_match),
                             (gpointer) (&fake_dev->ret_match));
  g_assert_true (fp_device_identify_sync (device, prints, NULL,
                                          test_driver_match_cb, match_data,
                                          &match, &print, &error));
  g_test_assert_expected_messages ();

  g_object_unref (fake_dev->ret_match);
  g_assert_null (fake_dev->ret_match);
  g_assert_true (match_data->called);
  g_assert_no_error (match_data->error);
  g_assert_no_error (error);
  g_assert_false (match);
  g_assert_null (print);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported an error code but also provided a match*");
  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a print together with an error*");

  test_driver_match_data_clear (match_data);
  fake_dev->ret_error = fpi_device_retry_new (FP_DEVICE_RETRY_REMOVE_FINGER);
  fake_dev->ret_match = prints->pdata[0];
  fake_dev->ret_print = make_fake_print (device, NULL);
  g_object_add_weak_pointer (G_OBJECT (fake_dev->ret_print),
                             (gpointer) (&fake_dev->ret_print));
  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           test_driver_match_cb, match_data,
                                           &match, &print, &error));
  g_test_assert_expected_messages ();

  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_REMOVE_FINGER);
  g_assert_true (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_null (fake_dev->ret_print);
  g_assert_true (match_data->called);
  g_assert_error (match_data->error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_REMOVE_FINGER);
  g_assert_false (match);
  g_assert_null (print);
  g_clear_error (&error);
}

static void
test_driver_identify_report_no_callback (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(MatchCbData) match_data = g_new0 (MatchCbData, 1);
  g_autoptr(GPtrArray) prints = NULL;
  g_autoptr(FpAutoCloseDevice) device = NULL;
  G_GNUC_UNUSED g_autoptr(FpPrint) enrolled_print = NULL;
  g_autoptr(FpPrint) print = NULL;
  g_autoptr(FpPrint) match = NULL;
  g_autoptr(GError) error = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->identify = fake_device_identify_complete_error;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);
  prints = make_fake_prints_gallery (device, 0);
  enrolled_print = make_fake_print_reffed (device, NULL);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Driver reported a verify error that was not in the retry domain*");

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_NOT_SUPPORTED);
  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           test_driver_match_cb, match_data,
                                           &match, &print, &error));

  g_test_assert_expected_messages ();

  g_assert_null (match);
  g_assert_null (print);
  g_assert_false (match_data->called);
  g_assert_null (match_data->match);
  g_assert_no_error (match_data->error);

  g_assert (fake_dev->last_called_function == dev_class->identify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));
  g_assert_false (match);
}

static void
fake_device_stub_capture (FpDevice *device)
{
}

static void
test_driver_supports_capture (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->features |= FP_DEVICE_FEATURE_CAPTURE;
  dev_class->capture = fake_device_stub_capture;

  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_supports_capture (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_CAPTURE));
}

static void
test_driver_do_not_support_capture (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->features &= ~FP_DEVICE_FEATURE_CAPTURE;
  dev_class->capture = NULL;

  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_false (fp_device_supports_capture (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_CAPTURE));
}

static void
test_driver_capture (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpImage) image = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean wait_for_finger = TRUE;

  fake_dev->ret_image = fp_image_new (500, 500);
  image = fp_device_capture_sync (device, wait_for_finger, NULL, &error);
  g_assert (fake_dev->last_called_function == dev_class->capture);
  g_assert_true (GPOINTER_TO_UINT (fake_dev->action_data));
  g_assert_no_error (error);

  g_assert (image == fake_dev->ret_image);
}

static void
test_driver_capture_not_supported (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpImage) image = NULL;
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  gboolean wait_for_finger = TRUE;
  FpiDeviceFake *fake_dev;

  dev_class->features &= ~FP_DEVICE_FEATURE_CAPTURE;

  device = auto_close_fake_device_new ();
  fake_dev = FPI_DEVICE_FAKE (device);
  fake_dev->last_called_function = NULL;

  image = fp_device_capture_sync (device, wait_for_finger, NULL, &error);
  g_assert_null (fake_dev->last_called_function);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED);

  g_assert_null (image);
}

static void
test_driver_capture_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpImage) image = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean wait_for_finger = TRUE;

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  image = fp_device_capture_sync (device, wait_for_finger, NULL, &error);
  g_assert (fake_dev->last_called_function == dev_class->capture);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));

  g_assert_null (image);
}

static void
test_driver_has_storage (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->features |= FP_DEVICE_FEATURE_STORAGE;

  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_true (fp_device_has_storage (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_true (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE));
}

static void
test_driver_has_not_storage (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpDevice) device = NULL;

  dev_class->features &= ~FP_DEVICE_FEATURE_STORAGE;

  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_false (fp_device_has_storage (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE));
}

static void
test_driver_list (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 500);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->ret_list = g_steal_pointer (&prints);
  prints = fp_device_list_prints_sync (device, NULL, &error);

  g_assert (fake_dev->last_called_function == dev_class->list);
  g_assert_no_error (error);

  g_assert (prints == fake_dev->ret_list);
}

static void
test_driver_list_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_autoptr(GPtrArray) prints = NULL;

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  prints = fp_device_list_prints_sync (device, NULL, &error);

  g_assert (fake_dev->last_called_function == dev_class->list);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));

  g_assert_null (prints);
}

static void
test_driver_list_no_storage (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(GPtrArray) prints = NULL;
  g_autoptr(GError) error = NULL;

  dev_class->features &= ~FP_DEVICE_FEATURE_STORAGE;

  device = auto_close_fake_device_new ();
  G_GNUC_BEGIN_IGNORE_DEPRECATIONS
    g_assert_false (fp_device_has_storage (device));
  G_GNUC_END_IGNORE_DEPRECATIONS
  g_assert_false (fp_device_has_feature (device, FP_DEVICE_FEATURE_STORAGE));

  prints = fp_device_list_prints_sync (device, NULL, &error);
  g_assert_null (prints);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED);
}

static void
test_driver_delete (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean ret;

  ret = fp_device_delete_print_sync (device, enrolled_print, NULL, &error);
  g_assert (fake_dev->last_called_function == dev_class->delete);
  g_assert (fake_dev->action_data == enrolled_print);
  g_assert_no_error (error);
  g_assert_true (ret);
}

static void
test_driver_delete_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean ret;

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  ret = fp_device_delete_print_sync (device, enrolled_print, NULL, &error);
  g_assert (fake_dev->last_called_function == dev_class->delete);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));

  g_assert_false (ret);
}

static void
test_driver_clear_storage (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean ret;

  ret = fp_device_clear_storage_sync (device, NULL, &error);
  g_assert (fake_dev->last_called_function == dev_class->clear_storage);
  g_assert_no_error (error);
  g_assert_true (ret);
}

static void
test_driver_clear_storage_error (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean ret;

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_GENERAL);
  ret = fp_device_clear_storage_sync (device, NULL, &error);
  g_assert (fake_dev->last_called_function == dev_class->clear_storage);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_assert (error == g_steal_pointer (&fake_dev->ret_error));

  g_assert_false (ret);
}

static gboolean
fake_device_delete_wait_for_cancel_timeout (gpointer data)
{
  FpDevice *device = data;
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);

  g_assert (fake_dev->last_called_function == dev_class->cancel);
  default_fake_dev_class.delete (device);

  g_assert (fake_dev->last_called_function == default_fake_dev_class.delete);
  fake_dev->last_called_function = fake_device_delete_wait_for_cancel_timeout;

  return G_SOURCE_REMOVE;
}

static void
fake_device_delete_wait_for_cancel (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fake_device_delete_wait_for_cancel;

  g_timeout_add (100, fake_device_delete_wait_for_cancel_timeout, device);
}

static void
on_driver_cancel_delete (GObject *obj, GAsyncResult *res, gpointer user_data)
{
  g_autoptr(GError) error = NULL;
  FpDevice *device = FP_DEVICE (obj);
  gboolean *completed = user_data;

  fp_device_delete_print_finish (device, res, &error);
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_CANCELLED);

  *completed = TRUE;
}

static void
test_driver_cancel (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(GCancellable) cancellable = NULL;
  g_autoptr(FpPrint) enrolled_print = NULL;
  gboolean completed = FALSE;
  FpiDeviceFake *fake_dev;

  dev_class->delete = fake_device_delete_wait_for_cancel;

  device = auto_close_fake_device_new ();
  fake_dev = FPI_DEVICE_FAKE (device);
  cancellable = g_cancellable_new ();
  enrolled_print = make_fake_print_reffed (device, NULL);

  fp_device_delete_print (device, enrolled_print, cancellable,
                          on_driver_cancel_delete, &completed);
  g_cancellable_cancel (cancellable);

  while (!completed)
    g_main_context_iteration (NULL, TRUE);

  g_assert (fake_dev->last_called_function == fake_device_delete_wait_for_cancel_timeout);
}

static void
test_driver_cancel_fail (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(FpAutoCloseDevice) device = auto_close_fake_device_new ();
  g_autoptr(GCancellable) cancellable = g_cancellable_new ();
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_true (fp_device_delete_print_sync (device, enrolled_print, cancellable, &error));
  g_assert (fake_dev->last_called_function == dev_class->delete);
  g_cancellable_cancel (cancellable);

  while (g_main_context_iteration (NULL, FALSE))
    continue;

  g_assert (fake_dev->last_called_function == dev_class->delete);
  g_assert_no_error (error);
}

static void
test_driver_current_action (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_assert_cmpint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_NONE);
}

static void
test_driver_current_action_open_vfunc (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_OPEN);
  fake_dev->last_called_function = test_driver_current_action_open_vfunc;

  fpi_device_open_complete (device, NULL);
}

static void
test_driver_current_action_open (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->open = test_driver_current_action_open_vfunc;
  device = auto_close_fake_device_new ();
  fake_dev = FPI_DEVICE_FAKE (device);
  g_assert (fake_dev->last_called_function == test_driver_current_action_open_vfunc);

  g_assert_cmpint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_NONE);
}

static void
test_driver_action_get_cancellable_open_vfunc (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_OPEN);
  fake_dev->last_called_function = test_driver_action_get_cancellable_open_vfunc;

  g_assert_true (G_IS_CANCELLABLE (fpi_device_get_cancellable (device)));

  fpi_device_open_complete (device, NULL);
}

static void
test_driver_action_get_cancellable_open (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(GCancellable) cancellable = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->open = test_driver_action_get_cancellable_open_vfunc;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);

  cancellable = g_cancellable_new ();
  g_assert_true (fp_device_open_sync (device, cancellable, NULL));

  g_assert (fake_dev->last_called_function == test_driver_action_get_cancellable_open_vfunc);
}

static void
test_driver_action_get_cancellable_open_internal_vfunc (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_OPEN);
  fake_dev->last_called_function = test_driver_action_get_cancellable_open_internal_vfunc;

  g_assert_true (G_IS_CANCELLABLE (fpi_device_get_cancellable (device)));

  fpi_device_open_complete (device, NULL);
}

static void
test_driver_action_get_cancellable_open_internal (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->open = test_driver_action_get_cancellable_open_internal_vfunc;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_assert (fake_dev->last_called_function == test_driver_action_get_cancellable_open_internal_vfunc);
}

static void
test_driver_action_get_cancellable_error (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*FPI_DEVICE_ACTION_NONE*failed");
  g_assert_null (fpi_device_get_cancellable (device));
  g_test_assert_expected_messages ();
}

static void
test_driver_action_is_cancelled_open_vfunc (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_OPEN);
  fake_dev->last_called_function = test_driver_action_is_cancelled_open_vfunc;

  g_assert_true (G_IS_CANCELLABLE (fpi_device_get_cancellable (device)));
  g_assert_false (fpi_device_action_is_cancelled (device));

  if (fake_dev->ext_cancellable)
    g_cancellable_cancel (fake_dev->ext_cancellable);
  else
    g_cancellable_cancel (fpi_device_get_cancellable (device));

  g_assert_true (fpi_device_action_is_cancelled (device));

  fpi_device_open_complete (device, NULL);
}

static void
test_driver_action_is_cancelled_open (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(GCancellable) cancellable = NULL;
  g_autoptr(GError) error = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->open = test_driver_action_is_cancelled_open_vfunc;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);

  cancellable = fake_dev->ext_cancellable = g_cancellable_new ();
  g_assert_false (fp_device_open_sync (device, cancellable, &error));
  g_assert_error (error, G_IO_ERROR, G_IO_ERROR_CANCELLED);

  g_assert (fake_dev->last_called_function == test_driver_action_is_cancelled_open_vfunc);
}

static void
test_driver_action_internally_cancelled_open (void)
{
  g_autoptr(FpAutoResetClass) dev_class = auto_reset_device_class ();
  g_autoptr(FpAutoCloseDevice) device = NULL;
  g_autoptr(GCancellable) cancellable = NULL;
  g_autoptr(GError) error = NULL;
  FpiDeviceFake *fake_dev;

  dev_class->open = test_driver_action_is_cancelled_open_vfunc;
  device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  fake_dev = FPI_DEVICE_FAKE (device);

  /* No error, just some internal cancellation but we let nothing happen externally. */
  cancellable = g_cancellable_new ();
  g_assert_true (fp_device_open_sync (device, cancellable, &error));
  g_assert_null (error);

  g_assert (fake_dev->last_called_function == test_driver_action_is_cancelled_open_vfunc);
}

static void
test_driver_action_is_cancelled_error (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*FPI_DEVICE_ACTION_NONE*failed");
  g_assert_true (fpi_device_action_is_cancelled (device));
  g_test_assert_expected_messages ();
}

static void
test_driver_complete_actions_errors (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_probe_complete (device, NULL, NULL, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_open_complete (device, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_close_complete (device, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_enroll_complete (device, NULL, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_verify_complete (device, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_identify_complete (device, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_capture_complete (device, NULL, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_delete_complete (device, NULL);
  g_test_assert_expected_messages ();

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*failed");
  fpi_device_list_complete (device, NULL, NULL);
  g_test_assert_expected_messages ();
}

static void
test_driver_action_error_error (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_CRITICAL,
                         "*assertion*current_action*FPI_DEVICE_ACTION_NONE*failed");
  fpi_device_action_error (device, NULL);
  g_test_assert_expected_messages ();
}

static void
test_driver_action_error_all (void)
{
  g_autoptr(FpAutoCloseDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 0);
  g_autoptr(GError) error = NULL;
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev;

  fake_dev = FPI_DEVICE_FAKE (device);
  fake_dev->return_action_error = TRUE;
  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);

  g_assert_false (fp_device_open_sync (device, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->open);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->return_action_error = FALSE;
  fake_dev->ret_error = NULL;
  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  fake_dev->return_action_error = TRUE;

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_null (fp_device_enroll_sync (device, fp_print_new (device), NULL,
                                        NULL, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->enroll);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         NULL, NULL, NULL, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->verify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           NULL, NULL, NULL, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->identify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_null (fp_device_capture_sync (device, TRUE, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->capture);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_null (fp_device_list_prints_sync (device, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->list);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_false (fp_device_delete_print_sync (device, enrolled_print, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->delete);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_false (fp_device_clear_storage_sync (device, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->clear_storage);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);

  /* Test close last, as we can't operate on a closed device. */
  fake_dev->ret_error = fpi_device_error_new (FP_DEVICE_ERROR_DATA_INVALID);
  g_assert_false (fp_device_close_sync (device, NULL, &error));
  g_assert_true (fake_dev->last_called_function == dev_class->close);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_DATA_INVALID);
  g_clear_error (&error);
}

static void
test_driver_action_error_fallback_all (void)
{
  g_autoptr(FpAutoCloseDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  g_autoptr(FpPrint) enrolled_print = make_fake_print_reffed (device, NULL);
  g_autoptr(GPtrArray) prints = make_fake_prints_gallery (device, 0);
  g_autoptr(GError) error = NULL;
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev;

  fake_dev = FPI_DEVICE_FAKE (device);
  fake_dev->return_action_error = TRUE;

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_false (fp_device_open_sync (device, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->open);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  fake_dev->return_action_error = FALSE;
  g_assert_true (fp_device_open_sync (device, NULL, NULL));

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  fake_dev->return_action_error = TRUE;
  g_assert_null (fp_device_enroll_sync (device, fp_print_new (device), NULL,
                                        NULL, NULL, &error));
  g_test_assert_expected_messages ();
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->enroll);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_false (fp_device_verify_sync (device, enrolled_print, NULL,
                                         NULL, NULL, NULL, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->verify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_false (fp_device_identify_sync (device, prints, NULL,
                                           NULL, NULL, NULL, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->identify);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_null (fp_device_capture_sync (device, TRUE, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->capture);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_null (fp_device_list_prints_sync (device, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->list);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_false (fp_device_delete_print_sync (device, enrolled_print, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->delete);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_false (fp_device_clear_storage_sync (device, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->clear_storage);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);

  /* Test close last, as we can't operate on a closed device. */
  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING,
                         "*Device failed to pass an error to generic action "
                         "error function*");

  g_assert_false (fp_device_close_sync (device, NULL, &error));
  g_test_assert_expected_messages ();
  g_assert_true (fake_dev->last_called_function == dev_class->close);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_clear_error (&error);
}

static void
test_driver_add_timeout_func (FpDevice *device, gpointer user_data)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = test_driver_add_timeout_func;
}

static void
test_driver_add_timeout (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpDevice *data_check = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);

  g_object_add_weak_pointer (G_OBJECT (data_check), (gpointer) & data_check);
  fpi_device_add_timeout (device, 50, test_driver_add_timeout_func,
                          data_check, g_object_unref);

  g_assert_nonnull (data_check);

  while (FP_IS_DEVICE (data_check))
    g_main_context_iteration (NULL, TRUE);

  g_assert_null (data_check);
  g_assert (fake_dev->last_called_function == test_driver_add_timeout_func);
}

static gboolean
test_driver_add_timeout_cancelled_timeout (gpointer data)
{
  GSource *source = data;

  g_source_destroy (source);

  return G_SOURCE_REMOVE;
}

static void
test_driver_add_timeout_cancelled (void)
{
  g_autoptr(FpDevice) device = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpDevice *data_check = g_object_new (FPI_TYPE_DEVICE_FAKE, NULL);
  GSource *source;

  g_object_add_weak_pointer (G_OBJECT (data_check), (gpointer) & data_check);
  source = fpi_device_add_timeout (device, 2000, test_driver_add_timeout_func,
                                   data_check, g_object_unref);

  g_timeout_add (20, test_driver_add_timeout_cancelled_timeout, source);
  g_assert_nonnull (data_check);

  while (FP_IS_DEVICE (data_check))
    g_main_context_iteration (NULL, TRUE);

  g_assert_null (data_check);
  g_assert_null (fake_dev->last_called_function);
}

static void
test_driver_error_types (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(GEnumClass) errors_enum = g_type_class_ref (FP_TYPE_DEVICE_ERROR);
  int i;

  for (i = 0; g_enum_get_value (errors_enum, i); ++i)
    {
      g_autoptr(GError) e = NULL;
      g_autoptr(GError) msg_e = NULL;
      g_autofree char *expected_msg = NULL;
      g_autofree char *enum_string = g_enum_to_string (FP_TYPE_DEVICE_ERROR, i);

      e = fpi_device_error_new (i);
      g_assert_error (e, FP_DEVICE_ERROR, i);

      expected_msg = g_strdup_printf ("Error message %s", enum_string);
      msg_e = fpi_device_error_new_msg (i, "Error message %s", enum_string);
      g_assert_error (msg_e, FP_DEVICE_ERROR, i);
      g_assert_cmpstr (msg_e->message, ==, expected_msg);
    }

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "*Unsupported error*");
  error = fpi_device_error_new (i + 1);
  g_assert_error (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_GENERAL);
  g_test_assert_expected_messages ();
}

static void
test_driver_retry_error_types (void)
{
  g_autoptr(GError) error = NULL;
  g_autoptr(GEnumClass) errors_enum = g_type_class_ref (FP_TYPE_DEVICE_RETRY);
  int i;

  for (i = 0; g_enum_get_value (errors_enum, i); ++i)
    {
      g_autoptr(GError) e = NULL;
      g_autoptr(GError) msg_e = NULL;
      g_autofree char *expected_msg = NULL;
      g_autofree char *enum_string = g_enum_to_string (FP_TYPE_DEVICE_RETRY, i);

      e = fpi_device_retry_new (i);
      g_assert_error (e, FP_DEVICE_RETRY, i);

      expected_msg = g_strdup_printf ("Retry error message %s", enum_string);
      msg_e = fpi_device_retry_new_msg (i, "Retry error message %s", enum_string);
      g_assert_error (msg_e, FP_DEVICE_RETRY, i);
      g_assert_cmpstr (msg_e->message, ==, expected_msg);
    }

  g_test_expect_message (G_LOG_DOMAIN, G_LOG_LEVEL_WARNING, "*Unsupported error*");
  error = fpi_device_retry_new (i + 1);
  g_assert_error (error, FP_DEVICE_RETRY, FP_DEVICE_RETRY_GENERAL);
  g_test_assert_expected_messages ();
}

int
main (int argc, char *argv[])
{
  g_test_init (&argc, &argv, NULL);

  g_test_add_func ("/driver/get_driver", test_driver_get_driver);
  g_test_add_func ("/driver/get_device_id", test_driver_get_device_id);
  g_test_add_func ("/driver/get_name", test_driver_get_name);
  g_test_add_func ("/driver/is_open", test_driver_is_open);
  g_test_add_func ("/driver/get_scan_type/press", test_driver_get_scan_type_press);
  g_test_add_func ("/driver/get_scan_type/swipe", test_driver_get_scan_type_swipe);
  g_test_add_func ("/driver/set_scan_type/press", test_driver_set_scan_type_press);
  g_test_add_func ("/driver/set_scan_type/swipe", test_driver_set_scan_type_swipe);
  g_test_add_func ("/driver/finger_status/inactive", test_driver_finger_status_inactive);
  g_test_add_func ("/driver/finger_status/waiting", test_driver_finger_status_needed);
  g_test_add_func ("/driver/finger_status/present", test_driver_finger_status_present);
  g_test_add_func ("/driver/finger_status/changes", test_driver_finger_status_changes);
  g_test_add_func ("/driver/get_nr_enroll_stages", test_driver_get_nr_enroll_stages);
  g_test_add_func ("/driver/set_nr_enroll_stages", test_driver_set_nr_enroll_stages);
  g_test_add_func ("/driver/supports_identify", test_driver_supports_identify);
  g_test_add_func ("/driver/supports_capture", test_driver_supports_capture);
  g_test_add_func ("/driver/has_storage", test_driver_has_storage);
  g_test_add_func ("/driver/do_not_support_identify", test_driver_do_not_support_identify);
  g_test_add_func ("/driver/do_not_support_capture", test_driver_do_not_support_capture);
  g_test_add_func ("/driver/has_not_storage", test_driver_has_not_storage);
  g_test_add_func ("/driver/get_usb_device", test_driver_get_usb_device);
  g_test_add_func ("/driver/get_virtual_env", test_driver_get_virtual_env);
  g_test_add_func ("/driver/get_driver_data", test_driver_get_driver_data);
  g_test_add_func ("/driver/features/probe_updates", test_driver_features_probe_updates);
  g_test_add_func ("/driver/initial_features", test_driver_initial_features);
  g_test_add_func ("/driver/initial_features/none", test_driver_initial_features_none);
  g_test_add_func ("/driver/initial_features/no_capture", test_driver_initial_features_no_capture);
  g_test_add_func ("/driver/initial_features/no_verify", test_driver_initial_features_no_verify);
  g_test_add_func ("/driver/initial_features/no_identify", test_driver_initial_features_no_identify);
  g_test_add_func ("/driver/initial_features/no_storage", test_driver_initial_features_no_storage);
  g_test_add_func ("/driver/initial_features/no_list", test_driver_initial_features_no_list);
  g_test_add_func ("/driver/initial_features/no_delete", test_driver_initial_features_no_delete);
  g_test_add_func ("/driver/initial_features/no_clear", test_driver_initial_features_no_clear);


  g_test_add_func ("/driver/probe", test_driver_probe);
  g_test_add_func ("/driver/probe/error", test_driver_probe_error);
  g_test_add_func ("/driver/probe/action_error", test_driver_probe_action_error);
  g_test_add_func ("/driver/open", test_driver_open);
  g_test_add_func ("/driver/open/error", test_driver_open_error);
  g_test_add_func ("/driver/close", test_driver_close);
  g_test_add_func ("/driver/close/error", test_driver_close_error);
  g_test_add_func ("/driver/enroll", test_driver_enroll);
  g_test_add_func ("/driver/enroll/error", test_driver_enroll_error);
  g_test_add_func ("/driver/enroll/error/no_print", test_driver_enroll_error_no_print);
  g_test_add_func ("/driver/enroll/progress", test_driver_enroll_progress);
  g_test_add_func ("/driver/verify", test_driver_verify);
  g_test_add_func ("/driver/verify/fail", test_driver_verify_fail);
  g_test_add_func ("/driver/verify/retry", test_driver_verify_retry);
  g_test_add_func ("/driver/verify/error", test_driver_verify_error);
  g_test_add_func ("/driver/verify/not_supported", test_driver_verify_not_supported);
  g_test_add_func ("/driver/verify/report_no_cb", test_driver_verify_report_no_callback);
  g_test_add_func ("/driver/verify/not_reported", test_driver_verify_not_reported);
  g_test_add_func ("/driver/verify/complete_retry", test_driver_verify_complete_retry);
  g_test_add_func ("/driver/identify", test_driver_identify);
  g_test_add_func ("/driver/identify/fail", test_driver_identify_fail);
  g_test_add_func ("/driver/identify/retry", test_driver_identify_retry);
  g_test_add_func ("/driver/identify/error", test_driver_identify_error);
  g_test_add_func ("/driver/identify/not_reported", test_driver_identify_not_reported);
  g_test_add_func ("/driver/identify/complete_retry", test_driver_identify_complete_retry);
  g_test_add_func ("/driver/identify/report_no_cb", test_driver_identify_report_no_callback);
  g_test_add_func ("/driver/capture", test_driver_capture);
  g_test_add_func ("/driver/capture/not_supported", test_driver_capture_not_supported);
  g_test_add_func ("/driver/capture/error", test_driver_capture_error);
  g_test_add_func ("/driver/list", test_driver_list);
  g_test_add_func ("/driver/list/error", test_driver_list_error);
  g_test_add_func ("/driver/list/no_storage", test_driver_list_no_storage);
  g_test_add_func ("/driver/delete", test_driver_delete);
  g_test_add_func ("/driver/delete/error", test_driver_delete_error);
  g_test_add_func ("/driver/clear_storage", test_driver_clear_storage);
  g_test_add_func ("/driver/clear_storage/error", test_driver_clear_storage_error);
  g_test_add_func ("/driver/cancel", test_driver_cancel);
  g_test_add_func ("/driver/cancel/fail", test_driver_cancel_fail);

  g_test_add_func ("/driver/get_current_action", test_driver_current_action);
  g_test_add_func ("/driver/get_current_action/open", test_driver_current_action_open);
  g_test_add_func ("/driver/get_cancellable/error", test_driver_action_get_cancellable_error);
  g_test_add_func ("/driver/get_cancellable/open", test_driver_action_get_cancellable_open);
  g_test_add_func ("/driver/get_cancellable/open/internal", test_driver_action_get_cancellable_open_internal);
  g_test_add_func ("/driver/action_is_cancelled/open", test_driver_action_is_cancelled_open);
  g_test_add_func ("/driver/action_is_cancelled/open/internal", test_driver_action_internally_cancelled_open);
  g_test_add_func ("/driver/action_is_cancelled/error", test_driver_action_is_cancelled_error);
  g_test_add_func ("/driver/complete_action/all/error", test_driver_complete_actions_errors);
  g_test_add_func ("/driver/action_error/error", test_driver_action_error_error);
  g_test_add_func ("/driver/action_error/all", test_driver_action_error_all);
  g_test_add_func ("/driver/action_error/fail", test_driver_action_error_fallback_all);

  g_test_add_func ("/driver/timeout", test_driver_add_timeout);
  g_test_add_func ("/driver/timeout/cancelled", test_driver_add_timeout_cancelled);

  g_test_add_func ("/driver/error_types", test_driver_error_types);
  g_test_add_func ("/driver/retry_error_types", test_driver_retry_error_types);

  return g_test_run ();
}
