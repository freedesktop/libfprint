/*
 * Next Biometrics driver for libfprint
 *
 * Copyright (C) 2021 Huan Wang <fredwanghuan@gmail.com>
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

#define FRAME_HEIGHT 180
#define FRAME_WIDTH 256

#define NB1010_EP_OUT 0x02 | FPI_USB_ENDPOINT_OUT
#define NB1010_EP_IN 0x03 | FPI_USB_ENDPOINT_IN

#define NB1010_SENSITIVITY_BIT 12

#define NB1010_CMD_RECV_LEN 16
#define NB1010_CAPTURE_RECV_LEN 540

#define NB1010_DEFAULT_TIMEOUT 500

#define NB1010_N_PARTIAL 90
#define NB1010_LINE_PER_PARTIAL 2
#define NB1010_CAPTURE_HEADER_LEN 25

#define NB1010_TRANSITION_DELAY 50

/*
* The Follow Commands are obtained by decoding the usbcap, so it does not expose all the command available to the device.
* Known:
* 1. every command starts with 0x80
* 2. second byte is the comand, third byte is the seqence nubmer, init with rand, gets incremented
*    everytime a new instruction is sent to the device. However device does not care or check the sequence, just echo back
*    whatever chosen by the host.
* 3. cmd: 0x07 check, expect [0x80, 0x29...] as response
* 4. cmd: 0x16 ???, expect [0x80, 0x20...] as response. Happens during device init.
* 5. cmd: 0x13 print device, expect [0x80, 0x23...] as response. Response contains the device string
* 6. cmd: 0x38 check finger, expect [0x80, 0x37...] as response. The 14th byte indicate whether finger present [0-255]
* 7. cmd: 0x0d ???, expect [0x80, 0x20...] as response. Happens before capture.
* 8. cmd: 0x12 capture, expect [0x80, 0x20...] as response. After capture read 90 times in sequence to get all the frame.
*/

static guint8 nb1010_check_finger[] = {
  0x80, 0x38, 0x01, 0x00, 0x08, 0x00, 0x00, 0x00,
};
static size_t nb1010_check_finger_len = sizeof nb1010_check_finger / sizeof *nb1010_check_finger;

// pre capture, dont know what does it do, but appears everytime a capture begins
static guint8 nb1010_pre_capture[] = {
  0x80, 0x0d, 0x03, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00,
};
static size_t nb1010_pre_capture_len = sizeof nb1010_pre_capture / sizeof *nb1010_pre_capture;

static guint8 nb1010_capture[] = {
  0x80, 0x12, 0x04, 0x00, 0x20, 0x00, 0x00, 0x00,
  0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00,
};
static size_t nb1010_capture_len = sizeof nb1010_capture / sizeof *nb1010_capture;
