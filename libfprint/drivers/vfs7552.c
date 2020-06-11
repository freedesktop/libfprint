/*
 * Validity Sensors, Inc. VFS5011 Fingerprint Reader driver for libfprint
 * Copyright (C) 2013 Arseniy Lartsev <arseniy@chalmers.se>
 *                    AceLan Kao <acelan.kao@canonical.com>
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

#define FP_COMPONENT "vfs7552"

#include "drivers_api.h"
#include "vfs7552_proto.h"

/* =================== sync/async USB transfer sequence ==================== */

enum
{
    ACTION_SEND,
    ACTION_RECEIVE,
};

struct usb_action
{
    int type;
    const char *name;
    int endpoint;
    int size;
    unsigned char *data;
    int correct_reply_size;
};

#define SEND(ENDPOINT, COMMAND)  \
    {                            \
        .type = ACTION_SEND,     \
        .endpoint = ENDPOINT,    \
        .name = #COMMAND,        \
        .size = sizeof(COMMAND), \
        .data = COMMAND},

#define RECV(ENDPOINT, SIZE)    \
    {                           \
        .type = ACTION_RECEIVE, \
        .endpoint = ENDPOINT,   \
        .size = SIZE,           \
        .data = NULL},

#define RECV_CHECK(ENDPOINT, SIZE, EXPECTED) \
    {                                        \
        .type = ACTION_RECEIVE,              \
        .endpoint = ENDPOINT,                \
        .size = SIZE,                        \
        .data = EXPECTED,                    \
        .correct_reply_size = sizeof(EXPECTED)},

struct usbexchange_data
{
    int stepcount;
    FpImageDevice *device;
    struct usb_action *actions;
    void *receive_buf;
    int timeout;
};

struct _FpDeviceVfs7552
{
    FpImageDevice parent;

    unsigned char *total_buffer;
    unsigned char *capture_buffer;
    unsigned char *row_buffer;
    unsigned char *lastline;
    GSList *rows;
    int lines_captured, lines_recorded, empty_lines;
    int max_lines_captured, max_lines_recorded;
    int lines_total, lines_total_allocated;
    gboolean loop_running;
    gboolean deactivating;
    struct usbexchange_data init_sequence;
};

G_DECLARE_FINAL_TYPE(FpDeviceVfs7552, fpi_device_vfs7552, FPI, DEVICE_VFS7552,
                     FpImageDevice);
G_DEFINE_TYPE(FpDeviceVfs7552, fpi_device_vfs7552, FP_TYPE_IMAGE_DEVICE);

static void
dev_change_state(FpImageDevice *dev, FpiImageDeviceState state)
{
    fp_dbg("--> dev_change_state");
}

static void
dev_open(FpImageDevice *dev)
{
    fp_dbg("--> dev_open");
}

static void
dev_close(FpImageDevice *dev)
{
    fp_dbg("--> dev_close");
}

static void
dev_activate(FpImageDevice *dev)
{
    fp_dbg("--> dev_activate");
}

static void
dev_deactivate(FpImageDevice *dev)
{
    fp_dbg("--> dev_deactivate");
}

static const FpIdEntry id_table[] = {
    {/* Validity device from some Dell XPS laptops (9560, 9360 at least) */ .vid = 0x138a, .pid = 0x0091},
};

static void
fpi_device_vfs7552_init(FpDeviceVfs7552 *self)
{
}

static void
fpi_device_vfs7552_class_init(FpDeviceVfs7552Class *klass)
{
    FpDeviceClass *dev_class = FP_DEVICE_CLASS(klass);
    FpImageDeviceClass *img_class = FP_IMAGE_DEVICE_CLASS(klass);

    dev_class->id = "vfs7552";
    dev_class->full_name = "Validity VFS7552";
    dev_class->type = FP_DEVICE_TYPE_USB;
    dev_class->id_table = id_table;
    dev_class->scan_type = FP_SCAN_TYPE_PRESS;

    img_class->img_open = dev_open;
    img_class->img_close = dev_close;
    img_class->activate = dev_activate;
    img_class->deactivate = dev_deactivate;
    img_class->change_state = dev_change_state;

    //img_class->bz3_threshold = 20;

    img_class->img_width = VFS7552_IMAGE_WIDTH;
    img_class->img_height = VFS7552_IMAGE_HEIGHT;
}
