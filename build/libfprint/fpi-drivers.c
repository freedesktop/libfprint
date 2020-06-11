#include <glib-object.h>
#include "fpi-context.h"

extern GType (fpi_device_vfs7552_get_type) (void);

GArray *
fpi_get_driver_types (void)
{
  GArray *drivers = g_array_new (TRUE, FALSE, sizeof (GType));
  GType t;

  t = fpi_device_vfs7552_get_type ();
  g_array_append_val (drivers, t);

  return drivers;
}
