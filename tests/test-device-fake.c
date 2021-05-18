/*
 * Virtual driver for device debugging
 *
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

#define FP_COMPONENT "fake_test_dev"

#include "test-device-fake.h"

G_DEFINE_TYPE (FpiDeviceFake, fpi_device_fake, FP_TYPE_DEVICE)

static const FpIdEntry driver_ids[] = {
  { .virtual_envvar = "FP_VIRTUAL_FAKE_DEVICE" },
  { .virtual_envvar = NULL }
};

static void
fpi_device_fake_probe (FpDevice *device)
{
  FpDeviceClass *dev_class = FP_DEVICE_GET_CLASS (device);
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_probe;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_PROBE);

  fpi_device_update_features (device, fake_dev->probe_features_update, fake_dev->probe_features_value);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_probe_complete (device, dev_class->id, dev_class->full_name,
                             fake_dev->ret_error);
}

static void
fpi_device_fake_open (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_open;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_OPEN);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_open_complete (device, fake_dev->ret_error);
}

static void
fpi_device_fake_close (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_close;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_CLOSE);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_close_complete (device, fake_dev->ret_error);
}

static void
fpi_device_fake_enroll (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *print = fake_dev->ret_print;

  fake_dev->last_called_function = fpi_device_fake_enroll;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_ENROLL);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_get_enroll_data (device, (FpPrint **) &fake_dev->action_data);

  if (!print && !fake_dev->ret_error)
    {
      fpi_device_get_enroll_data (device, &print);
      fpi_print_set_type (print, FPI_PRINT_RAW);
    }

  fpi_device_enroll_complete (device,
                              print ? g_object_ref (print) : NULL,
                              fake_dev->ret_error);
}

static void
fpi_device_fake_verify (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *print = fake_dev->ret_print;

  fake_dev->last_called_function = fpi_device_fake_verify;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_VERIFY);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_get_verify_data (device, (FpPrint **) &fake_dev->action_data);

  if (!print && !fake_dev->ret_error)
    fpi_device_get_verify_data (device, &print);

  if (!fake_dev->ret_error || fake_dev->ret_error->domain == FP_DEVICE_RETRY)
    {
      fpi_device_verify_report (device, fake_dev->ret_result, print, fake_dev->ret_error);
      fpi_device_verify_complete (device, NULL);
    }
  else
    {
      fpi_device_verify_complete (device, fake_dev->ret_error);
    }
}

static void
fpi_device_fake_identify (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  FpPrint *match = fake_dev->ret_match;

  fake_dev->last_called_function = fpi_device_fake_identify;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_IDENTIFY);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_get_identify_data (device, (GPtrArray **) &fake_dev->action_data);

  if (!match && !fake_dev->ret_error)
    {
      GPtrArray *prints;
      unsigned int i;

      fpi_device_get_identify_data (device, &prints);

      for (i = 0; prints && i < prints->len; ++i)
        {
          FpPrint *print = g_ptr_array_index (prints, i);

          if (g_strcmp0 (fp_print_get_description (print), "fake-verified") == 0)
            {
              match = print;
              break;
            }
        }
    }

  if (!fake_dev->ret_error || fake_dev->ret_error->domain == FP_DEVICE_RETRY)
    {
      fpi_device_identify_report (device, match, fake_dev->ret_print, fake_dev->ret_error);
      fpi_device_identify_complete (device, NULL);
    }
  else
    {
      fpi_device_identify_complete (device, fake_dev->ret_error);
    }
}

static void
fpi_device_fake_capture (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);
  gboolean wait_for_finger;

  fake_dev->last_called_function = fpi_device_fake_capture;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_CAPTURE);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_get_capture_data (device, &wait_for_finger);
  fake_dev->action_data = GINT_TO_POINTER (wait_for_finger);
  fpi_device_capture_complete (device, fake_dev->ret_image, fake_dev->ret_error);
}

static void
fpi_device_fake_list (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_list;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_LIST);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_list_complete (device, fake_dev->ret_list, fake_dev->ret_error);
}

static void
fpi_device_fake_delete (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_delete;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_DELETE);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_get_delete_data (device, (FpPrint **) (&fake_dev->action_data));
  fpi_device_delete_complete (device, fake_dev->ret_error);
}

static void
fpi_device_fake_clear_storage (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_clear_storage;
  g_assert_cmpuint (fpi_device_get_current_action (device), ==, FPI_DEVICE_ACTION_CLEAR_STORAGE);

  if (fake_dev->return_action_error)
    {
      fpi_device_action_error (device, fake_dev->ret_error);
      return;
    }

  fpi_device_clear_storage_complete (device, fake_dev->ret_error);
}

static void
fpi_device_fake_cancel (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_cancel;
  g_assert_cmpuint (fpi_device_get_current_action (device), !=, FPI_DEVICE_ACTION_NONE);
}

static void
fpi_device_fake_suspend (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_suspend;

  fpi_device_suspend_complete (device, g_steal_pointer (&fake_dev->ret_suspend));
}

static void
fpi_device_fake_resume (FpDevice *device)
{
  FpiDeviceFake *fake_dev = FPI_DEVICE_FAKE (device);

  fake_dev->last_called_function = fpi_device_fake_resume;

  fpi_device_resume_complete (device, g_steal_pointer (&fake_dev->ret_resume));
}

static void
fpi_device_fake_init (FpiDeviceFake *self)
{
}

static void
fpi_device_fake_class_init (FpiDeviceFakeClass *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id = FP_COMPONENT;
  dev_class->full_name = "Virtual device for debugging";
  dev_class->type = FP_DEVICE_TYPE_VIRTUAL;
  dev_class->id_table = driver_ids;
  dev_class->nr_enroll_stages = 5;
  dev_class->scan_type = FP_SCAN_TYPE_PRESS;

  dev_class->probe = fpi_device_fake_probe;
  dev_class->open = fpi_device_fake_open;
  dev_class->close = fpi_device_fake_close;
  dev_class->enroll = fpi_device_fake_enroll;
  dev_class->verify = fpi_device_fake_verify;
  dev_class->identify = fpi_device_fake_identify;
  dev_class->capture = fpi_device_fake_capture;
  dev_class->list = fpi_device_fake_list;
  dev_class->delete = fpi_device_fake_delete;
  dev_class->cancel = fpi_device_fake_cancel;
  dev_class->clear_storage = fpi_device_fake_clear_storage;
  dev_class->suspend = fpi_device_fake_suspend;
  dev_class->resume = fpi_device_fake_resume;

  fpi_device_class_auto_initialize_features (dev_class);
}
