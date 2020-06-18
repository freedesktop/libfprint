/*
 * Example fingerprint verification program, which verifies the
 * finger which has been previously enrolled to disk.
 * Copyright (C) 2007 Daniel Drake <dsd@gentoo.org>
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

#define FP_COMPONENT "example-verify"

#include <stdio.h>
#include <libfprint/fprint.h>
#include <glib-unix.h>

#include "storage.h"
#include "utilities.h"

typedef struct _VerifyData
{
  GMainLoop    *loop;
  GCancellable *cancellable;
  unsigned int  sigint_handler;
  FpFinger      finger;
  int           ret_value;
} VerifyData;

static void
verify_data_free (VerifyData *verify_data)
{
  g_clear_handle_id (&verify_data->sigint_handler, g_source_remove);
  g_clear_object (&verify_data->cancellable);
  g_main_loop_unref (verify_data->loop);
  g_free (verify_data);
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC (VerifyData, verify_data_free)

static void
on_device_closed (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  VerifyData *verify_data = user_data;

  g_autoptr(GError) error = NULL;

  fp_device_close_finish (dev, res, &error);

  if (error)
    g_warning ("Failed closing device %s\n", error->message);

  g_main_loop_quit (verify_data->loop);
}

static void
verify_quit (FpDevice   *dev,
             VerifyData *verify_data)
{
  if (!fp_device_is_open (dev))
    {
      g_main_loop_quit (verify_data->loop);
      return;
    }

  fp_device_close (dev, NULL, (GAsyncReadyCallback) on_device_closed, verify_data);
}

static void start_verification (FpDevice   *dev,
                                VerifyData *verify_data);

static void
on_verify_completed (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  VerifyData *verify_data = user_data;

  g_autoptr(FpPrint) print = NULL;
  g_autoptr(GError) error = NULL;
  char buffer[20];
  gboolean match;

  if (!fp_device_verify_finish (dev, res, &match, &print, &error))
    {
      g_warning ("Failed to verify print: %s", error->message);
      verify_data->ret_value = EXIT_FAILURE;

      if (error->domain != FP_DEVICE_RETRY)
        {
          verify_quit (dev, verify_data);
          return;
        }
    }

  g_print ("Verify again? [Y/n]? ");
  if (fgets (buffer, sizeof (buffer), stdin) &&
      (buffer[0] == 'Y' || buffer[0] == 'y' || buffer[0] == '\n'))
    {
      start_verification (dev, verify_data);
      return;
    }

  verify_quit (dev, verify_data);
}

static void
on_match_cb (FpDevice *dev, FpPrint *match, FpPrint *print,
             gpointer user_data, GError *error)
{
  VerifyData *verify_data = user_data;

  if (error)
    {
      g_warning ("Match report: Finger not matched, retry error reported: %s",
                 error->message);
      return;
    }

  if (print && fp_print_get_image (print) &&
      print_image_save (print, "verify.pgm"))
    g_print ("Print image saved as verify.pgm\n");

  if (match)
    {
      char date_str[128];

      verify_data->ret_value = EXIT_SUCCESS;

      g_date_strftime (date_str, G_N_ELEMENTS (date_str), "%Y-%m-%d\0",
                       fp_print_get_enroll_date (match));
      g_debug ("Match report: device %s matched finger %s successifully "
               "with print %s, enrolled on date %s by user %s",
               fp_device_get_name (dev),
               finger_to_string (fp_print_get_finger (match)),
               fp_print_get_description (match), date_str,
               fp_print_get_username (match));

      g_print ("MATCH!\n");
    }
  else
    {
      g_debug ("Match report: Finger not matched");
      g_print ("NO MATCH!\n");
    }
}

static void
on_list_completed (FpDevice *dev, GAsyncResult *res, gpointer user_data)
{
  VerifyData *verify_data = user_data;

  g_autoptr(GPtrArray) prints = NULL;
  g_autoptr(GError) error = NULL;

  prints = fp_device_list_prints_finish (dev, res, &error);

  if (!error)
    {
      FpPrint *verify_print = NULL;
      guint i;

      if (!prints->len)
        g_warning ("No prints saved on device");

      for (i = 0; i < prints->len; ++i)
        {
          FpPrint *print = prints->pdata[i];

          if (fp_print_get_finger (print) == verify_data->finger &&
              g_strcmp0 (fp_print_get_username (print), g_get_user_name ()) == 0)
            {
              const GDate *verify_print_date = NULL;
              const GDate *print_date = fp_print_get_enroll_date (print);

              if (verify_print)
                verify_print_date = fp_print_get_enroll_date (verify_print);

              if (!verify_print || !print_date || !verify_print_date ||
                  g_date_compare (print_date, verify_print_date) >= 0)
                verify_print = print;
            }
        }

      if (!verify_print)
        {
          g_warning ("Did you remember to enroll your %s finger first?",
                     finger_to_string (verify_data->finger));
          verify_quit (dev, verify_data);
          return;
        }

      g_debug ("Comparing print with %s",
               fp_print_get_description (verify_print));

      g_print ("Print loaded. Time to verify!\n");
      fp_device_verify (dev, verify_print, verify_data->cancellable,
                        on_match_cb, verify_data, NULL,
                        (GAsyncReadyCallback) on_verify_completed,
                        verify_data);
    }
  else
    {
      g_warning ("Loading prints failed with error %s", error->message);
      verify_quit (dev, verify_data);
    }
}

static void
start_verification (FpDevice *dev, VerifyData *verify_data)
{
  if (verify_data->finger == FP_FINGER_UNKNOWN)
    {
      g_print ("Choose the finger to verify:\n");
      verify_data->finger = finger_chooser ();
    }

  if (verify_data->finger == FP_FINGER_UNKNOWN)
    {
      g_warning ("Unknown finger selected");
      verify_data->ret_value = EXIT_FAILURE;
      verify_quit (dev, verify_data);
      return;
    }

  if (fp_device_has_storage (dev))
    {
      g_print ("Creating finger template, using device storage...\n");
      fp_device_list_prints (dev, NULL,
                             (GAsyncReadyCallback) on_list_completed,
                             verify_data);
    }
  else
    {
      g_print ("Loading previously enrolled %s finger data...\n",
               finger_to_string (verify_data->finger));
      g_autoptr(FpPrint) verify_print = NULL;

      verify_print = print_data_load (dev, verify_data->finger);

      if (!verify_print)
        {
          g_warning ("Failed to load fingerprint data");
          g_warning ("Did you remember to enroll your %s finger first?",
                     finger_to_string (verify_data->finger));
          verify_quit (dev, verify_data);
          return;
        }

      g_print ("Print loaded. Time to verify!\n");
      fp_device_verify (dev, verify_print, verify_data->cancellable,
                        on_match_cb, verify_data, NULL,
                        (GAsyncReadyCallback) on_verify_completed,
                        verify_data);
    }
}

static void
on_device_opened (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  VerifyData *verify_data = user_data;

  g_autoptr(GError) error = NULL;

  if (!fp_device_open_finish (dev, res, &error))
    {
      g_warning ("Failed to open device: %s", error->message);
      verify_quit (dev, verify_data);
      return;
    }

  g_print ("Opened device. ");

  start_verification (dev, verify_data);
}

static gboolean
sigint_cb (void *user_data)
{
  VerifyData *verify_data = user_data;

  g_cancellable_cancel (verify_data->cancellable);

  return G_SOURCE_CONTINUE;
}

int
main (void)
{
  g_autoptr(FpContext) ctx = NULL;
  g_autoptr(VerifyData) verify_data = NULL;
  GPtrArray *devices;
  FpDevice *dev;

  setenv ("G_MESSAGES_DEBUG", "all", 0);
  setenv ("LIBUSB_DEBUG", "3", 0);

  ctx = fp_context_new ();

  devices = fp_context_get_devices (ctx);
  if (!devices)
    {
      g_warning ("Impossible to get devices");
      return EXIT_FAILURE;
    }

  dev = discover_device (devices);
  if (!dev)
    {
      g_warning ("No devices detected.");
      return EXIT_FAILURE;
    }

  verify_data = g_new0 (VerifyData, 1);
  verify_data->ret_value = EXIT_FAILURE;
  verify_data->loop = g_main_loop_new (NULL, FALSE);
  verify_data->cancellable = g_cancellable_new ();
  verify_data->sigint_handler = g_unix_signal_add_full (G_PRIORITY_HIGH,
                                                        SIGINT,
                                                        sigint_cb,
                                                        verify_data,
                                                        NULL);
  fp_device_open (dev, verify_data->cancellable,
                  (GAsyncReadyCallback) on_device_opened,
                  verify_data);

  g_main_loop_run (verify_data->loop);

  return verify_data->ret_value;
}
