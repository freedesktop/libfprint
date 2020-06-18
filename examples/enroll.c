/*
 * Example fingerprint enrollment program
 * Enrolls your chosen finger and saves the print to disk
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

#define FP_COMPONENT "example-enroll"

#include <stdio.h>
#include <libfprint/fprint.h>
#include <glib-unix.h>

#include "storage.h"
#include "utilities.h"

typedef struct _EnrollData
{
  GMainLoop    *loop;
  GCancellable *cancellable;
  unsigned int  sigint_handler;
  FpFinger      finger;
  int           ret_value;
} EnrollData;

static void
enroll_data_free (EnrollData *enroll_data)
{
  g_clear_handle_id (&enroll_data->sigint_handler, g_source_remove);
  g_clear_object (&enroll_data->cancellable);
  g_main_loop_unref (enroll_data->loop);
  g_free (enroll_data);
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC (EnrollData, enroll_data_free)

static void
on_device_closed (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  EnrollData *enroll_data = user_data;

  g_autoptr(GError) error = NULL;

  fp_device_close_finish (dev, res, &error);

  if (error)
    g_warning ("Failed closing device %s\n", error->message);

  g_main_loop_quit (enroll_data->loop);
}

static void
on_enroll_completed (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  EnrollData *enroll_data = user_data;

  g_autoptr(FpPrint) print = NULL;
  g_autoptr(GError) error = NULL;

  print = fp_device_enroll_finish (dev, res, &error);

  if (!error)
    {
      enroll_data->ret_value = EXIT_SUCCESS;

      if (!fp_device_has_storage (dev))
        {
          g_debug ("Device has not storage, saving locally");
          int r = print_data_save (print, enroll_data->finger);
          if (r < 0)
            {
              g_warning ("Data save failed, code %d", r);
              enroll_data->ret_value = EXIT_FAILURE;
            }
        }
    }
  else
    {
      g_warning ("Enroll failed with error %s\n", error->message);
    }

  fp_device_close (dev, NULL, (GAsyncReadyCallback) on_device_closed,
                   enroll_data);
}

static void
on_enroll_progress (FpDevice *device,
                    gint      completed_stages,
                    FpPrint  *print,
                    gpointer  user_data,
                    GError   *error)
{
  if (error)
    {
      g_warning ("Enroll stage %d of %d failed with error %s",
                 completed_stages,
                 fp_device_get_nr_enroll_stages (device),
                 error->message);
      return;
    }

  if (print && fp_print_get_image (print) &&
      print_image_save (print, "enrolled.pgm"))
    printf ("Wrote scanned image to enrolled.pgm\n");

  printf ("Enroll stage %d of %d passed. Yay!\n", completed_stages,
          fp_device_get_nr_enroll_stages (device));
}

static void
on_device_opened (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  EnrollData *enroll_data = user_data;
  FpPrint *print_template;

  g_autoptr(GError) error = NULL;

  if (!fp_device_open_finish (dev, res, &error))
    {
      g_warning ("Failed to open device: %s", error->message);
      g_main_loop_quit (enroll_data->loop);
      return;
    }

  printf ("Opened device. It's now time to enroll your finger.\n\n");
  printf ("You will need to successfully scan your %s finger %d times to "
          "complete the process.\n\n", finger_to_string (enroll_data->finger),
          fp_device_get_nr_enroll_stages (dev));
  printf ("Scan your finger now.\n");

  print_template = print_create_template (dev, enroll_data->finger);
  fp_device_enroll (dev, print_template, enroll_data->cancellable,
                    on_enroll_progress, NULL, NULL,
                    (GAsyncReadyCallback) on_enroll_completed,
                    enroll_data);
}

static gboolean
sigint_cb (void *user_data)
{
  EnrollData *enroll_data = user_data;

  g_cancellable_cancel (enroll_data->cancellable);

  return G_SOURCE_CONTINUE;
}

int
main (void)
{
  g_autoptr(FpContext) ctx = NULL;
  g_autoptr(EnrollData) enroll_data = NULL;
  GPtrArray *devices;
  FpDevice *dev;
  FpFinger finger;

  g_print ("This program will enroll the selected finger, unconditionally "
           "overwriting any print for the same finger that was enrolled "
           "previously. If you want to continue, press enter, otherwise hit "
           "Ctrl+C\n");
  getchar ();

  g_print ("Choose the finger to enroll:\n");
  finger = finger_chooser ();

  if (finger == FP_FINGER_UNKNOWN)
    {
      g_warning ("Unknown finger selected");
      return EXIT_FAILURE;
    }

  setenv ("G_MESSAGES_DEBUG", "all", 0);

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

  enroll_data = g_new0 (EnrollData, 1);
  enroll_data->finger = finger;
  enroll_data->ret_value = EXIT_FAILURE;
  enroll_data->loop = g_main_loop_new (NULL, FALSE);
  enroll_data->cancellable = g_cancellable_new ();
  enroll_data->sigint_handler = g_unix_signal_add_full (G_PRIORITY_HIGH,
                                                        SIGINT,
                                                        sigint_cb,
                                                        enroll_data,
                                                        NULL);

  fp_device_open (dev, enroll_data->cancellable,
                  (GAsyncReadyCallback) on_device_opened,
                  enroll_data);

  g_main_loop_run (enroll_data->loop);

  return enroll_data->ret_value;
}
