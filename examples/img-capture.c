/*
 * Example fingerprint verification program, which verifies the
 * finger which has been previously enrolled to disk.
 * Copyright (C) 2007 Daniel Drake <dsd@gentoo.org>
 * Copyright (C) 2019 Marco Trevisan <marco.trevisan@canonical.com>
 * Copyright (C) 2020 Vasily Khoruzhick <anarsoul@gmail.com>
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

#define FP_COMPONENT "example-capture"

#include <stdio.h>
#include <libfprint/fprint.h>
#include <glib-unix.h>

#include "storage.h"
#include "utilities.h"

typedef struct CaptureData
{
  GMainLoop    *loop;
  GCancellable *cancellable;
  unsigned int  sigint_handler;
  int           ret_value;
  const char   *filename;
} CaptureData;

static void
capture_data_free (CaptureData *capture_data)
{
  g_clear_handle_id (&capture_data->sigint_handler, g_source_remove);
  g_clear_object (&capture_data->cancellable);
  g_main_loop_unref (capture_data->loop);
  g_free (capture_data);
}
G_DEFINE_AUTOPTR_CLEANUP_FUNC (CaptureData, capture_data_free)

static void
on_device_closed (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  CaptureData *capture_data = user_data;

  g_autoptr(GError) error = NULL;

  fp_device_close_finish (dev, res, &error);

  if (error)
    g_warning ("Failed closing device %s\n", error->message);

  g_main_loop_quit (capture_data->loop);
}

static void
capture_quit (FpDevice    *dev,
              CaptureData *capture_data)
{
  if (!fp_device_is_open (dev))
    {
      g_main_loop_quit (capture_data->loop);
      return;
    }

  fp_device_close (dev, NULL, (GAsyncReadyCallback) on_device_closed, capture_data);
}

static void
dev_capture_cb (FpDevice     *dev,
                GAsyncResult *res,
                void         *user_data)
{
  g_autoptr(GError) error = NULL;
  CaptureData *capture_data = user_data;
  FpImage *image = NULL;

  g_clear_object (&capture_data->cancellable);

  image = fp_device_capture_finish (dev, res, &error);
  if (!image)
    {
      g_warning ("Error capturing data: %s", error->message);
      capture_quit (dev, capture_data);
      return;
    }

  save_image_to_pgm (image, capture_data->filename);

  capture_quit (dev, capture_data);
}

static void
start_capture (FpDevice *dev, CaptureData *capture_data)
{
  fp_device_capture (dev, TRUE, capture_data->cancellable, (GAsyncReadyCallback) dev_capture_cb, capture_data);
}

static void
on_device_opened (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  CaptureData *capture_data = user_data;

  g_autoptr(GError) error = NULL;

  if (!fp_device_open_finish (dev, res, &error))
    {
      g_warning ("Failed to open device: %s", error->message);
      capture_quit (dev, capture_data);
      return;
    }

  g_print ("Opened device. ");

  start_capture (dev, capture_data);
}

static gboolean
sigint_cb (void *user_data)
{
  CaptureData *capture_data = user_data;

  g_cancellable_cancel (capture_data->cancellable);

  return G_SOURCE_CONTINUE;
}

int
main (int argc, const char *argv[])
{
  g_autoptr(FpContext) ctx = NULL;
  g_autoptr(CaptureData) capture_data = NULL;
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

  capture_data = g_new0 (CaptureData, 1);
  capture_data->ret_value = EXIT_FAILURE;
  capture_data->loop = g_main_loop_new (NULL, FALSE);
  capture_data->cancellable = g_cancellable_new ();
  capture_data->sigint_handler = g_unix_signal_add_full (G_PRIORITY_HIGH,
                                                         SIGINT,
                                                         sigint_cb,
                                                         capture_data,
                                                         NULL);
  if (argc == 2)
    capture_data->filename = argv[1];
  else
    capture_data->filename = "finger.pgm";
  fp_device_open (dev, capture_data->cancellable,
                  (GAsyncReadyCallback) on_device_opened,
                  capture_data);

  g_main_loop_run (capture_data->loop);

  return capture_data->ret_value;
}
