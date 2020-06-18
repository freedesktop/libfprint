/*
 * Example libfprint GTK+ image capture program
 * Copyright (C) 2018 Bastien Nocera <hadess@hadess.net>
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

#include "config.h"

#include <gtk/gtk.h>
#include <libfprint/fprint.h>

struct _LibfprintDemo
{
  GtkApplication parent;
};
G_DECLARE_FINAL_TYPE (LibfprintDemo, libfprint_demo, FP, DEMO, GtkApplication)
G_DEFINE_TYPE (LibfprintDemo, libfprint_demo, GTK_TYPE_APPLICATION)

typedef enum {
  IMAGE_DISPLAY_NONE     =  0,
  IMAGE_DISPLAY_MINUTIAE =  1 << 0,
  IMAGE_DISPLAY_BINARY   =  1 << 1
} ImageDisplayFlags;

struct _LibfprintDemoWindow
{
  GtkApplicationWindow parent_instance;

  GtkWidget           *header_bar;
  GtkWidget           *mode_stack;
  GtkWidget           *capture_button;
  GtkWidget           *cancel_button;
  GtkWidget           *capture_image;
  GtkWidget           *spinner;
  GtkWidget           *instructions;

  GCancellable        *cancellable;

  gboolean             opened;
  FpDevice            *dev;

  FpImage             *img;
  ImageDisplayFlags    img_flags;
};

G_DECLARE_FINAL_TYPE (LibfprintDemoWindow, libfprint_demo_window, FP, DEMO_WINDOW, GtkApplicationWindow)
G_DEFINE_TYPE (LibfprintDemoWindow, libfprint_demo_window, GTK_TYPE_APPLICATION_WINDOW)

typedef enum {
  EMPTY_MODE,
  NOIMAGING_MODE,
  CAPTURE_MODE,
  SPINNER_MODE,
  ERROR_MODE,
  RETRY_MODE
} LibfprintDemoMode;

static void libfprint_demo_set_mode (LibfprintDemoWindow *win,
                                     LibfprintDemoMode    mode);

static unsigned char *
img_to_rgbdata (const guint8 *imgdata,
                int           width,
                int           height)
{
  int size = width * height;
  guint8 *rgbdata = g_malloc (size * 3);
  size_t i;
  size_t rgb_offset = 0;

  for (i = 0; i < size; i++)
    {
      guint8 pixel = imgdata[i];

      rgbdata[rgb_offset++] = pixel;
      rgbdata[rgb_offset++] = pixel;
      rgbdata[rgb_offset++] = pixel;
    }

  return rgbdata;
}

static void
plot_minutiae (unsigned char *rgbdata,
               int            width,
               int            height,
               GPtrArray     *minutiae)
{
  int i;

#define write_pixel(num) do { \
      rgbdata[((num) * 3)] = 0xff; \
      rgbdata[((num) * 3) + 1] = 0; \
      rgbdata[((num) * 3) + 2] = 0; \
  } while(0)

  for (i = 0; i < minutiae->len; i++)
    {
      struct fp_minutia *min = g_ptr_array_index (minutiae, i);
      int x, y;
      size_t pixel_offset;

      fp_minutia_get_coords (min, &x, &y);
      pixel_offset = (y * width) + x;
      write_pixel (pixel_offset - 2);
      write_pixel (pixel_offset - 1);
      write_pixel (pixel_offset);
      write_pixel (pixel_offset + 1);
      write_pixel (pixel_offset + 2);

      write_pixel (pixel_offset - (width * 2));
      write_pixel (pixel_offset - (width * 1) - 1);
      write_pixel (pixel_offset - (width * 1));
      write_pixel (pixel_offset - (width * 1) + 1);
      write_pixel (pixel_offset + (width * 1) - 1);
      write_pixel (pixel_offset + (width * 1));
      write_pixel (pixel_offset + (width * 1) + 1);
      write_pixel (pixel_offset + (width * 2));
    }
}

static GdkPixbuf *
img_to_pixbuf (FpImage          *img,
               ImageDisplayFlags flags)
{
  int width;
  int height;
  const guint8 *data;
  unsigned char *rgbdata;

  width = fp_image_get_width (img);
  height = fp_image_get_height (img);

  if (flags & IMAGE_DISPLAY_BINARY)
    data = fp_image_get_binarized (img, NULL);
  else
    data = fp_image_get_data (img, NULL);

  if (!data)
    return NULL;

  rgbdata = img_to_rgbdata (data, width, height);

  if (flags & IMAGE_DISPLAY_MINUTIAE)
    {
      GPtrArray *minutiae;

      minutiae = fp_image_get_minutiae (img);
      plot_minutiae (rgbdata, width, height, minutiae);
    }

  return gdk_pixbuf_new_from_data (rgbdata, GDK_COLORSPACE_RGB,
                                   FALSE, 8, width, height,
                                   width * 3, (GdkPixbufDestroyNotify) g_free,
                                   NULL);
}

static void
update_image (LibfprintDemoWindow *win)
{
  GdkPixbuf *pixbuf;

  if (win->img == NULL)
    {
      gtk_image_clear (GTK_IMAGE (win->capture_image));
      return;
    }

  g_debug ("Updating image, minutiae %s, binary mode %s",
           win->img_flags & IMAGE_DISPLAY_MINUTIAE ? "shown" : "hidden",
           win->img_flags & IMAGE_DISPLAY_BINARY ? "on" : "off");
  pixbuf = img_to_pixbuf (win->img, win->img_flags);
  gtk_image_set_from_pixbuf (GTK_IMAGE (win->capture_image), pixbuf);
  g_object_unref (pixbuf);
}

static void
libfprint_demo_set_spinner_label (LibfprintDemoWindow *win,
                                  const char          *message)
{
  char *label;

  label = g_strdup_printf ("<b><span size=\"large\">%s</span></b>", message);
  gtk_label_set_markup (GTK_LABEL (win->instructions), label);
  g_free (label);
}

static void
libfprint_demo_set_capture_label (LibfprintDemoWindow *win)
{
  FpScanType scan_type;
  const char *message;

  scan_type = fp_device_get_scan_type (win->dev);

  switch (scan_type)
    {
    case FP_SCAN_TYPE_PRESS:
      message = "Place your finger on the fingerprint reader";
      break;

    case FP_SCAN_TYPE_SWIPE:
      message = "Swipe your finger across the fingerprint reader";
      break;

    default:
      g_assert_not_reached ();
    }

  libfprint_demo_set_spinner_label (win, message);
}

static void
dev_capture_start_cb (FpDevice     *dev,
                      GAsyncResult *res,
                      void         *user_data)
{
  g_autoptr(GError) error = NULL;
  LibfprintDemoWindow *win = user_data;
  FpImage *image = NULL;

  g_clear_object (&win->cancellable);

  image = fp_device_capture_finish (dev, res, &error);
  if (!image)
    {
      g_warning ("Error capturing data: %s", error->message);
      if (error->domain == FP_DEVICE_RETRY ||
          g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
        libfprint_demo_set_mode (win, RETRY_MODE);
      else if (g_error_matches (error, FP_DEVICE_ERROR, FP_DEVICE_ERROR_NOT_SUPPORTED))
        libfprint_demo_set_mode (win, NOIMAGING_MODE);
      else
        libfprint_demo_set_mode (win, ERROR_MODE);
      return;
    }

  g_clear_object (&win->img);
  win->img = image;
  update_image (win);

  libfprint_demo_set_mode (win, CAPTURE_MODE);
}

static void
dev_start_capture (LibfprintDemoWindow *win)
{
  libfprint_demo_set_capture_label (win);

  fp_device_capture (win->dev, TRUE, win->cancellable, (GAsyncReadyCallback) dev_capture_start_cb, win);
}

static void
dev_open_cb (FpDevice *dev, GAsyncResult *res, void *user_data)
{
  LibfprintDemoWindow *win = user_data;

  g_autoptr(GError) error = NULL;

  if (!fp_device_open_finish (dev, res, &error))
    {
      g_warning ("Failed to open device: %s", error->message);
      libfprint_demo_set_mode (win, ERROR_MODE);
      return;
    }

  dev_start_capture (win);
}

static void
activate_capture (GSimpleAction *action,
                  GVariant      *parameter,
                  gpointer       user_data)
{
  LibfprintDemoWindow *win = user_data;

  libfprint_demo_set_mode (win, SPINNER_MODE);
  g_clear_pointer (&win->img, g_object_unref);

  g_clear_object (&win->cancellable);
  win->cancellable = g_cancellable_new ();

  if (win->opened)
    {
      dev_start_capture (win);
      return;
    }

  libfprint_demo_set_spinner_label (win, "Opening fingerprint reader");

  win->opened = TRUE;
  fp_device_open (win->dev, win->cancellable, (GAsyncReadyCallback) dev_open_cb, user_data);
}

static void
cancel_capture (GSimpleAction *action,
                GVariant      *parameter,
                gpointer       user_data)
{
  LibfprintDemoWindow *win = user_data;

  g_debug ("cancelling %p", win->cancellable);

  if (win->cancellable)
    g_cancellable_cancel (win->cancellable);
}

static void
activate_quit (GSimpleAction *action,
               GVariant      *parameter,
               gpointer       user_data)
{
  GtkApplication *app = user_data;
  GtkWidget *win;
  GList *list, *next;

  list = gtk_application_get_windows (app);
  while (list)
    {
      win = list->data;
      next = list->next;

      gtk_widget_destroy (GTK_WIDGET (win));

      list = next;
    }
}

static void
activate_show_minutiae (GSimpleAction *action,
                        GVariant      *parameter,
                        gpointer       user_data)
{
  LibfprintDemoWindow *win = user_data;
  GVariant *state;
  gboolean new_state;

  state = g_action_get_state (G_ACTION (action));
  new_state = !g_variant_get_boolean (state);
  g_action_change_state (G_ACTION (action), g_variant_new_boolean (new_state));
  g_variant_unref (state);

  if (new_state)
    win->img_flags |= IMAGE_DISPLAY_MINUTIAE;
  else
    win->img_flags &= ~IMAGE_DISPLAY_MINUTIAE;

  update_image (win);
}

static void
activate_show_binary (GSimpleAction *action,
                      GVariant      *parameter,
                      gpointer       user_data)
{
  LibfprintDemoWindow *win = user_data;
  GVariant *state;
  gboolean new_state;

  state = g_action_get_state (G_ACTION (action));
  new_state = !g_variant_get_boolean (state);
  g_action_change_state (G_ACTION (action), g_variant_new_boolean (new_state));
  g_variant_unref (state);

  if (new_state)
    win->img_flags |= IMAGE_DISPLAY_BINARY;
  else
    win->img_flags &= ~IMAGE_DISPLAY_BINARY;

  update_image (win);
}

static void
change_show_minutiae_state (GSimpleAction *action,
                            GVariant      *state,
                            gpointer       user_data)
{
  g_simple_action_set_state (action, state);
}

static void
change_show_binary_state (GSimpleAction *action,
                          GVariant      *state,
                          gpointer       user_data)
{
  g_simple_action_set_state (action, state);
}

static GActionEntry app_entries[] = {
  { "quit", activate_quit, NULL, NULL, NULL },
};

static GActionEntry win_entries[] = {
  { "show-minutiae", activate_show_minutiae, NULL, "false", change_show_minutiae_state },
  { "show-binary", activate_show_binary, NULL, "false", change_show_binary_state },
  { "capture", activate_capture, NULL, NULL, NULL },
  { "cancel", cancel_capture, NULL, NULL, NULL }
};

static void
activate (GApplication *app)
{
  LibfprintDemoWindow *window;

  window = g_object_new (libfprint_demo_window_get_type (),
                         "application", app,
                         NULL);
  gtk_widget_show (GTK_WIDGET (window));
}

static void
libfprint_demo_set_mode (LibfprintDemoWindow *win,
                         LibfprintDemoMode    mode)
{
  char *title;

  switch (mode)
    {
    case EMPTY_MODE:
      gtk_stack_set_visible_child_name (GTK_STACK (win->mode_stack), "empty-mode");
      gtk_widget_set_sensitive (win->capture_button, FALSE);
      gtk_widget_set_sensitive (win->cancel_button, FALSE);
      gtk_spinner_stop (GTK_SPINNER (win->spinner));
      break;

    case NOIMAGING_MODE:
      gtk_stack_set_visible_child_name (GTK_STACK (win->mode_stack), "noimaging-mode");
      gtk_widget_set_sensitive (win->capture_button, FALSE);
      gtk_widget_set_sensitive (win->cancel_button, FALSE);
      gtk_spinner_stop (GTK_SPINNER (win->spinner));
      break;

    case CAPTURE_MODE:
      gtk_stack_set_visible_child_name (GTK_STACK (win->mode_stack), "capture-mode");
      gtk_widget_set_sensitive (win->capture_button, TRUE);
      gtk_widget_set_sensitive (win->cancel_button, FALSE);

      title = g_strdup_printf ("%s Test", fp_device_get_name (win->dev));
      gtk_header_bar_set_title (GTK_HEADER_BAR (win->header_bar), title);
      g_free (title);

      gtk_spinner_stop (GTK_SPINNER (win->spinner));
      break;

    case SPINNER_MODE:
      gtk_stack_set_visible_child_name (GTK_STACK (win->mode_stack), "spinner-mode");
      gtk_widget_set_sensitive (win->capture_button, FALSE);
      gtk_widget_set_sensitive (win->cancel_button, TRUE);
      gtk_spinner_start (GTK_SPINNER (win->spinner));
      break;

    case ERROR_MODE:
      gtk_stack_set_visible_child_name (GTK_STACK (win->mode_stack), "error-mode");
      gtk_widget_set_sensitive (win->capture_button, FALSE);
      gtk_widget_set_sensitive (win->cancel_button, FALSE);
      gtk_spinner_stop (GTK_SPINNER (win->spinner));
      break;

    case RETRY_MODE:
      gtk_stack_set_visible_child_name (GTK_STACK (win->mode_stack), "retry-mode");
      gtk_widget_set_sensitive (win->capture_button, TRUE);
      gtk_widget_set_sensitive (win->cancel_button, FALSE);
      gtk_spinner_stop (GTK_SPINNER (win->spinner));
      break;

    default:
      g_assert_not_reached ();
    }
}

static void
libfprint_demo_init (LibfprintDemo *app)
{
  g_action_map_add_action_entries (G_ACTION_MAP (app),
                                   app_entries, G_N_ELEMENTS (app_entries),
                                   app);
}

static void
libfprint_demo_class_init (LibfprintDemoClass *class)
{
  GApplicationClass *app_class = G_APPLICATION_CLASS (class);

  app_class->activate = activate;
}

static void
libfprint_demo_window_init (LibfprintDemoWindow *window)
{
  FpContext *ctx;
  GPtrArray *devices;

  gtk_widget_init_template (GTK_WIDGET (window));
  gtk_window_set_default_size (GTK_WINDOW (window), 700, 500);

  g_action_map_add_action_entries (G_ACTION_MAP (window),
                                   win_entries, G_N_ELEMENTS (win_entries),
                                   window);

  ctx = fp_context_new ();

  devices = fp_context_get_devices (ctx);
  if (!devices)
    {
      libfprint_demo_set_mode (window, ERROR_MODE);
      return;
    }

  /* Empty list? */
  if (devices->len == 0)
    {
      libfprint_demo_set_mode (window, EMPTY_MODE);
      return;
    }

  if (!fp_device_supports_capture (g_ptr_array_index (devices, 0)))
    {
      libfprint_demo_set_mode (window, NOIMAGING_MODE);
      return;
    }

  window->dev = g_object_ref (g_ptr_array_index (devices, 0));
  libfprint_demo_set_mode (window, CAPTURE_MODE);
}

static void
libfprint_demo_window_class_init (LibfprintDemoWindowClass *class)
{
  GtkWidgetClass *widget_class = GTK_WIDGET_CLASS (class);

  gtk_widget_class_set_template_from_resource (widget_class, "/libfprint_demo/gtk-libfprint-test.ui");
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, header_bar);
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, mode_stack);
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, capture_button);
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, cancel_button);
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, capture_image);
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, spinner);
  gtk_widget_class_bind_template_child (widget_class, LibfprintDemoWindow, instructions);

  //FIXME setup dispose
}

int
main (int argc, char **argv)
{
  GtkApplication *app;

  app = GTK_APPLICATION (g_object_new (libfprint_demo_get_type (),
                                       "application-id", "org.freedesktop.libfprint.Demo",
                                       "flags", G_APPLICATION_FLAGS_NONE,
                                       NULL));

  return g_application_run (G_APPLICATION (app), 0, NULL);
}
