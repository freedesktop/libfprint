/*
 * Trivial storage driver for example programs
 *
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

#define FP_COMPONENT "example-storage"

#include <libfprint/fprint.h>
#include <libfprint/fpi-compat.h>
#include "storage.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#define STORAGE_FILE "test-storage.variant"

static char *
get_print_data_descriptor (FpPrint *print, FpDevice *dev, FpFinger finger)
{
  const char *driver;
  const char *dev_id;

  if (print)
    {
      driver = fp_print_get_driver (print);
      dev_id = fp_print_get_device_id (print);
    }
  else
    {
      driver = fp_device_get_driver (dev);
      dev_id = fp_device_get_device_id (dev);
    }

  return g_strdup_printf ("%s/%s/%x",
                          driver,
                          dev_id,
                          finger);
}

static GVariantDict *
load_data (void)
{
  GVariantDict *res;
  GVariant *var;
  gchar *contents = NULL;
  gsize length = 0;

  if (!g_file_get_contents (STORAGE_FILE, &contents, &length, NULL))
    {
      g_warning ("Error loading storage, assuming it is empty");
      return g_variant_dict_new (NULL);
    }

  var = g_variant_new_from_data (G_VARIANT_TYPE_VARDICT,
                                 contents,
                                 length,
                                 FALSE,
                                 g_free,
                                 contents);

  res = g_variant_dict_new (var);
  g_variant_unref (var);
  return res;
}

static int
save_data (GVariant *data)
{
  const gchar *contents = NULL;
  gsize length;

  length = g_variant_get_size (data);
  contents = (gchar *) g_variant_get_data (data);

  if (!g_file_set_contents (STORAGE_FILE, contents, length, NULL))
    {
      g_warning ("Error saving storage,!");
      return -1;
    }

  g_variant_ref_sink (data);
  g_variant_unref (data);

  return 0;
}

int
print_data_save (FpPrint *print, FpFinger finger)
{
  g_autofree gchar *descr = get_print_data_descriptor (print, NULL, finger);

  g_autoptr(GError) error = NULL;
  g_autoptr(GVariantDict) dict = NULL;
  g_autofree guchar *data = NULL;
  GVariant *val;
  gsize size;
  int res;

  dict = load_data ();

  fp_print_serialize (print, &data, &size, &error);
  if (error)
    {
      g_warning ("Error serializing data: %s", error->message);
      return -1;
    }
  val = g_variant_new_fixed_array (G_VARIANT_TYPE ("y"), data, size, 1);
  g_variant_dict_insert_value (dict, descr, val);

  res = save_data (g_variant_dict_end (dict));

  return res;
}

FpPrint *
print_data_load (FpDevice *dev, FpFinger finger)
{
  g_autofree gchar *descr = get_print_data_descriptor (NULL, dev, finger);

  g_autoptr(GVariant) val = NULL;
  g_autoptr(GVariantDict) dict = NULL;
  const guchar *stored_data = NULL;
  gsize stored_len;

  dict = load_data ();
  val = g_variant_dict_lookup_value (dict, descr, G_VARIANT_TYPE ("ay"));

  if (val)
    {
      FpPrint *print;
      g_autoptr(GError) error = NULL;

      stored_data = (const guchar *) g_variant_get_fixed_array (val, &stored_len, 1);
      print = fp_print_deserialize (stored_data, stored_len, &error);

      if (error)
        g_warning ("Error deserializing data: %s", error->message);

      return print;
    }

  return NULL;
}

FpPrint *
print_create_template (FpDevice *dev, FpFinger finger)
{
  g_autoptr(GDateTime) datetime = NULL;
  g_autoptr(GDate) date = NULL;
  FpPrint *template = NULL;
  gint year, month, day;

  template = fp_print_new (dev);
  fp_print_set_finger (template, finger);
  fp_print_set_username (template, g_get_user_name ());
  datetime = g_date_time_new_now_local ();
  g_date_time_get_ymd (datetime, &year, &month, &day);
  date = g_date_new_dmy (day, month, year);
  fp_print_set_enroll_date (template, date);

  return template;
}


gboolean
save_image_to_pgm (FpImage *img, const char *path)
{
  FILE *fd = fopen (path, "w");
  size_t write_size;
  const guchar *data = fp_image_get_data (img, &write_size);
  int r;

  if (!fd)
    {
      g_warning ("could not open '%s' for writing: %d", path, errno);
      return FALSE;
    }

  r = fprintf (fd, "P5 %d %d 255\n",
               fp_image_get_width (img), fp_image_get_height (img));
  if (r < 0)
    {
      fclose (fd);
      g_critical ("pgm header write failed, error %d", r);
      return FALSE;
    }

  r = fwrite (data, 1, write_size, fd);
  if (r < write_size)
    {
      fclose (fd);
      g_critical ("short write (%d)", r);
      return FALSE;
    }

  fclose (fd);
  g_debug ("written to '%s'", path);

  return TRUE;
}

gboolean
print_image_save (FpPrint *print, const char *path)
{
  FpImage *img = NULL;

  g_return_val_if_fail (FP_IS_PRINT (print), FALSE);
  g_return_val_if_fail (path != NULL, FALSE);

  img = fp_print_get_image (print);

  if (img)
    return save_image_to_pgm (img, path);

  return FALSE;
}
