/*
 * FpImageDevice - An image based fingerprint reader device
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

#pragma once

#include "fp-device.h"

G_BEGIN_DECLS

#define FP_TYPE_IMAGE_DEVICE (fp_image_device_get_type ())
G_DECLARE_DERIVABLE_TYPE (FpImageDevice, fp_image_device, FP, IMAGE_DEVICE, FpDevice)

G_END_DECLS
