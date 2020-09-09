/*
 * Goodix Moc driver for libfprint
 * Copyright (C) 2019 Shenzhen Goodix Technology Co., Ltd.
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

#include <glib.h>
#include "goodix_proto.h"

/*
 *  Crc functions
 */

#define WIDTH (8 * sizeof (uint32_t))
#define FINAL_XOR_VALUE 0xFFFFFFFF
#define REFLECT_DATA(X)         ((uint8_t) reflect ((X), 8))
#define REFLECT_REMAINDER(X)    ((unsigned int) reflect ((X), WIDTH))


uint8_t
gx_proto_crc8_calc (uint8_t *lubp_date, uint32_t lui_len)
{
  const uint8_t *data = lubp_date;
  unsigned int crc = 0;
  int i, j;

  for (j = lui_len; j; j--, data++)
    {
      crc ^= (*data << 8);
      for (i = 8; i; i--)
        {
          if (crc & 0x8000)
            crc ^= (0x1070 << 3);
          crc <<= 1;
        }
    }

  crc >>= 8;
  crc = ~crc;
  return (uint8_t) crc;
}

typedef struct
{
  uint32_t crc;
} gf_crc32_context;

static uint32_t s_crc_table[256] =
{ 0x0,         0x4c11db7,  0x9823b6e,  0xd4326d9, 0x130476dc, 0x17c56b6b, 0x1a864db2, 0x1e475005,
  0x2608edb8, 0x22c9f00f, 0x2f8ad6d6, 0x2b4bcb61, 0x350c9b64, 0x31cd86d3, 0x3c8ea00a, 0x384fbdbd,
  0x4c11db70, 0x48d0c6c7, 0x4593e01e, 0x4152fda9, 0x5f15adac, 0x5bd4b01b, 0x569796c2, 0x52568b75,
  0x6a1936c8, 0x6ed82b7f, 0x639b0da6, 0x675a1011, 0x791d4014, 0x7ddc5da3, 0x709f7b7a, 0x745e66cd,
  0x9823b6e0, 0x9ce2ab57, 0x91a18d8e, 0x95609039, 0x8b27c03c, 0x8fe6dd8b, 0x82a5fb52, 0x8664e6e5,
  0xbe2b5b58, 0xbaea46ef, 0xb7a96036, 0xb3687d81, 0xad2f2d84, 0xa9ee3033, 0xa4ad16ea, 0xa06c0b5d,
  0xd4326d90, 0xd0f37027, 0xddb056fe, 0xd9714b49, 0xc7361b4c, 0xc3f706fb, 0xceb42022, 0xca753d95,
  0xf23a8028, 0xf6fb9d9f, 0xfbb8bb46, 0xff79a6f1, 0xe13ef6f4, 0xe5ffeb43, 0xe8bccd9a, 0xec7dd02d,
  0x34867077, 0x30476dc0, 0x3d044b19, 0x39c556ae, 0x278206ab, 0x23431b1c, 0x2e003dc5, 0x2ac12072,
  0x128e9dcf, 0x164f8078, 0x1b0ca6a1, 0x1fcdbb16, 0x18aeb13,   0x54bf6a4,  0x808d07d,  0xcc9cdca,
  0x7897ab07, 0x7c56b6b0, 0x71159069, 0x75d48dde, 0x6b93dddb, 0x6f52c06c, 0x6211e6b5, 0x66d0fb02,
  0x5e9f46bf, 0x5a5e5b08, 0x571d7dd1, 0x53dc6066, 0x4d9b3063, 0x495a2dd4, 0x44190b0d, 0x40d816ba,
  0xaca5c697, 0xa864db20, 0xa527fdf9, 0xa1e6e04e, 0xbfa1b04b, 0xbb60adfc, 0xb6238b25, 0xb2e29692,
  0x8aad2b2f, 0x8e6c3698, 0x832f1041, 0x87ee0df6, 0x99a95df3, 0x9d684044, 0x902b669d, 0x94ea7b2a,
  0xe0b41de7, 0xe4750050, 0xe9362689, 0xedf73b3e, 0xf3b06b3b, 0xf771768c, 0xfa325055, 0xfef34de2,
  0xc6bcf05f, 0xc27dede8, 0xcf3ecb31, 0xcbffd686, 0xd5b88683, 0xd1799b34, 0xdc3abded, 0xd8fba05a,
  0x690ce0ee, 0x6dcdfd59, 0x608edb80, 0x644fc637, 0x7a089632, 0x7ec98b85, 0x738aad5c, 0x774bb0eb,
  0x4f040d56, 0x4bc510e1, 0x46863638, 0x42472b8f, 0x5c007b8a, 0x58c1663d, 0x558240e4, 0x51435d53,
  0x251d3b9e, 0x21dc2629, 0x2c9f00f0, 0x285e1d47, 0x36194d42, 0x32d850f5, 0x3f9b762c, 0x3b5a6b9b,
  0x315d626,  0x7d4cb91,  0xa97ed48,  0xe56f0ff, 0x1011a0fa, 0x14d0bd4d, 0x19939b94, 0x1d528623,
  0xf12f560e, 0xf5ee4bb9, 0xf8ad6d60, 0xfc6c70d7, 0xe22b20d2, 0xe6ea3d65, 0xeba91bbc, 0xef68060b,
  0xd727bbb6, 0xd3e6a601, 0xdea580d8, 0xda649d6f, 0xc423cd6a, 0xc0e2d0dd, 0xcda1f604, 0xc960ebb3,
  0xbd3e8d7e, 0xb9ff90c9, 0xb4bcb610, 0xb07daba7, 0xae3afba2, 0xaafbe615, 0xa7b8c0cc, 0xa379dd7b,
  0x9b3660c6, 0x9ff77d71, 0x92b45ba8, 0x9675461f, 0x8832161a, 0x8cf30bad, 0x81b02d74, 0x857130c3,
  0x5d8a9099, 0x594b8d2e, 0x5408abf7, 0x50c9b640, 0x4e8ee645, 0x4a4ffbf2, 0x470cdd2b, 0x43cdc09c,
  0x7b827d21, 0x7f436096, 0x7200464f, 0x76c15bf8, 0x68860bfd, 0x6c47164a, 0x61043093, 0x65c52d24,
  0x119b4be9, 0x155a565e, 0x18197087, 0x1cd86d30, 0x29f3d35,  0x65e2082,   0xb1d065b,  0xfdc1bec,
  0x3793a651, 0x3352bbe6, 0x3e119d3f, 0x3ad08088, 0x2497d08d, 0x2056cd3a, 0x2d15ebe3, 0x29d4f654,
  0xc5a92679, 0xc1683bce, 0xcc2b1d17, 0xc8ea00a0, 0xd6ad50a5, 0xd26c4d12, 0xdf2f6bcb, 0xdbee767c,
  0xe3a1cbc1, 0xe760d676, 0xea23f0af, 0xeee2ed18, 0xf0a5bd1d, 0xf464a0aa, 0xf9278673, 0xfde69bc4,
  0x89b8fd09, 0x8d79e0be, 0x803ac667, 0x84fbdbd0, 0x9abc8bd5, 0x9e7d9662, 0x933eb0bb, 0x97ffad0c,
  0xafb010b1, 0xab710d06, 0xa6322bdf, 0xa2f33668, 0xbcb4666d, 0xb8757bda, 0xb5365d03, 0xb1f740b4};

static uint32_t
reflect (uint32_t data, uint8_t n_bits)
{
  unsigned long reflection = 0x00000000;
  uint8_t bit;

  /*
   * Reflect the data about the center bit.
   */
  for (bit = 0; bit < n_bits; ++bit)
    {
      /*
       * If the LSB bit is set, set the reflection of it.
       */
      if (data & 0x01)
        reflection |= (1 << ((n_bits - 1) - bit));

      data = (data >> 1);
    }

  return reflection;

}

static void
crc32_init (gf_crc32_context *ctx)
{
  ctx->crc = 0xFFFFFFFF;
}

static void
crc32_update (gf_crc32_context *ctx, const uint8_t *message, uint32_t n_bytes)
{
  uint8_t data;
  uint32_t byte;

  /*
   * Divide the message by the polynomial, a byte at a time.
   */
  for (byte = 0; byte < n_bytes; ++byte)
    {
      data = REFLECT_DATA (message[byte]) ^ (ctx->crc >> (WIDTH - 8));
      ctx->crc = s_crc_table[data] ^ (ctx->crc << 8);
    }
}

static void
crc32_final (gf_crc32_context *ctx, uint8_t *md)
{
  ctx->crc = (REFLECT_REMAINDER (ctx->crc) ^ FINAL_XOR_VALUE);
  memcpy (md, &ctx->crc, 4);
}

uint8_t
gx_proto_crc32_calc (uint8_t *pchMsg, uint32_t wDataLen, uint8_t *pchMsgDst)
{
  gf_crc32_context context = { 0 };

  if (!pchMsg)
    return 0;

  crc32_init (&context);

  crc32_update (&context, pchMsg, wDataLen);

  crc32_final (&context, pchMsgDst);


  return 1;
}
/*
 *  protocol
 *
 */

static uint8_t dump_seq = 0;

static void
init_pack_header (
  ppack_header pheader,
  uint16_t     len,
  uint16_t     cmd,
  uint8_t      packagenum
                 )
{
  g_assert (pheader);

  memset (pheader, 0, sizeof (*pheader));
  pheader->cmd0 = HIBYTE (cmd);
  pheader->cmd1 = LOBYTE (cmd);
  pheader->packagenum = packagenum;
  pheader->reserved = dump_seq++;
  pheader->len = len + PACKAGE_CRC_SIZE;
  pheader->crc8 = gx_proto_crc8_calc ((uint8_t *) pheader, 6);
  pheader->rev_crc8 = ~pheader->crc8;
}


int
gx_proto_build_package (uint8_t       *ppackage,
                        uint32_t      *package_len,
                        uint16_t       cmd,
                        const uint8_t *payload,
                        uint32_t       payload_size)
{
  pack_header header;

  if (!ppackage || !package_len)
    return -1;

  if(*package_len < (payload_size + PACKAGE_HEADER_SIZE + PACKAGE_CRC_SIZE))
    return -1;

  init_pack_header (&header, payload_size, cmd, 0);

  memcpy (ppackage, &header, PACKAGE_HEADER_SIZE);
  memcpy (ppackage + PACKAGE_HEADER_SIZE, payload, payload_size);

  gx_proto_crc32_calc (ppackage, PACKAGE_HEADER_SIZE + payload_size, ppackage + PACKAGE_HEADER_SIZE + payload_size);

  return 0;
}


int
gx_proto_parse_header (
  uint8_t     *buffer,
  uint32_t     buffer_len,
  pack_header *pheader)
{
  if (!buffer || !pheader)
    return -1;
  if (buffer_len < PACKAGE_HEADER_SIZE)
    return -1;

  memcpy (pheader, buffer, sizeof (pack_header));

  pheader->len = GUINT16_FROM_LE (*(buffer + 4));
  pheader->len -= PACKAGE_CRC_SIZE;

  return 0;
}

static int
gx_proto_parse_fingerid (
  uint8_t          * fid_buffer,
  uint16_t           fid_buffer_size,
  ptemplate_format_t template
                        )
{
  uint8_t * buffer = NULL;
  uint16_t Offset = 0;

  if (!template || !fid_buffer)
    return -1;

  if (fid_buffer_size < 70)
    return -1;

  buffer = fid_buffer;
  Offset = 0;

  if (buffer[Offset++] != 67)
    return -1;

  template->type = buffer[Offset++];
  template->finger_index = buffer[Offset++];
  Offset++;

  memcpy (template->accountid, &buffer[Offset], 32);
  Offset += 32;

  memcpy (template->tid, &buffer[Offset], 32);
  Offset += 32;   // Offset == 68

  template->payload.size = buffer[Offset++];
  memset (template->payload.data, 0, 56);
  memcpy (template->payload.data, &buffer[Offset], template->payload.size);

  return 0;
}

int
gx_proto_parse_body (uint16_t cmd, uint8_t *buffer, uint32_t buffer_len, pgxfp_cmd_response_t presp)
{
  uint32_t offset = 0;
  uint8_t *fingerlist = NULL;

  if (!buffer || !presp)
    return -1;
  if (buffer_len < 1)
    return -1;
  presp->result = buffer[0];
  switch (HIBYTE (cmd))
    {
    case RESPONSE_PACKAGE_CMD:
      {
        presp->parse_msg.ack_cmd = buffer[1];
      }
      break;

    case MOC_CMD0_UPDATE_CONFIG:
    case MOC_CMD0_COMMITENROLLMENT:
    case MOC_CMD0_DELETETEMPLATE:
      break;

    case MOC_CMD0_GET_VERSION:
      memcpy (&presp->version_info, buffer + 1, sizeof (gxfp_version_info_t));
      break;

    case MOC_CMD0_CAPTURE_DATA:
      if (LOBYTE (cmd) == MOC_CMD1_DEFAULT)
        {
          presp->capture_data_resp.img_quality  = buffer[1];
          presp->capture_data_resp.img_coverage = buffer[2];
        }
      break;

    case MOC_CMD0_ENROLL_INIT:
      if (presp->result == GX_SUCCESS)
        memcpy (&presp->enroll_init.tid, &buffer[1], TEMPLATE_ID_SIZE);
      break;

    case MOC_CMD0_ENROLL:
      presp->enroll_update.rollback = (buffer[0] < 0x80) ? false : true;
      presp->enroll_update.img_overlay    = buffer[1];
      presp->enroll_update.img_preoverlay = buffer[2];
      break;

    case MOC_CMD0_CHECK4DUPLICATE:
      presp->check_duplicate_resp.duplicate = (presp->result == 0) ? false : true;
      if (presp->check_duplicate_resp.duplicate)
        {
          uint16_t tid_size = GUINT16_FROM_LE (*(buffer + 1));
          memcpy (&presp->check_duplicate_resp.template, buffer + 3, tid_size);
        }
      break;

    case MOC_CMD0_GETFINGERLIST:
      if (presp->result != GX_SUCCESS)
        break;
      presp->finger_list_resp.finger_num = buffer[1];
      if (presp->finger_list_resp.finger_num > FP_MAX_FINGERNUM)
        {
          presp->finger_list_resp.finger_num = 0;
          presp->result = GX_ERROR_NO_AVAILABLE_SPACE;
          break;
        }
      fingerlist = buffer + 2;
      for(uint8_t num = 0; num < presp->finger_list_resp.finger_num; num++)
        {
          uint16_t fingerid_length = GUINT16_FROM_LE (*(fingerlist + offset));
          offset += 2;
          if (gx_proto_parse_fingerid (fingerlist + offset,
                                       fingerid_length,
                                       &presp->finger_list_resp.finger_list[num]) != 0)
            {
              g_error ("parse fingerlist error");
              presp->finger_list_resp.finger_num = 0;
              presp->result = GX_FAILED;
              break;
            }
          offset += fingerid_length;
        }
      break;

    case MOC_CMD0_IDENTIFY:
      {
        uint32_t score = 0;
        uint8_t study = 0;
        uint16_t fingerid_size = 0;
        presp->verify.match = (buffer[0] == 0) ? true : false;
        if (presp->verify.match)
          {
            offset += 1;
            presp->verify.rejectdetail = GUINT16_FROM_LE (*(buffer + offset));
            offset += 2;
            score = GUINT32_FROM_LE (*(buffer + offset));
            offset += 4;
            study = GUINT16_FROM_LE (*(buffer + offset));
            offset += 1;
            fingerid_size = GUINT16_FROM_LE (*(buffer + offset));
            offset += 2;
            if (gx_proto_parse_fingerid (buffer + offset, fingerid_size, &presp->verify.template) != 0)
              {
                presp->result = GX_FAILED;
                break;
              }
            g_debug ("match, score: %d, study: %d", score, study);
          }
      }
      break;

    case MOC_CMD0_FINGER_MODE:
      presp->finger_status.status = buffer[0];
      break;

    default:
      break;
    }

  return 0;
}

static uint8_t sensor_config[26] = {
  0x00, 0x00, 0x64, 0x50, 0x0f, 0x41, 0x08, 0x0a, 0x18, 0x00, 0x00, 0x23, 0x00,
  0x00, 0x01, 0x01, 0x00, 0x01, 0x01, 0x01, 0x01, 0x00, 0x01, 0x01, 0x05, 0x05
};

int
gx_proto_init_sensor_config (pgxfp_sensor_cfg_t pconfig)
{
  uint32_t crc32_calc = 0;

  if (!pconfig)
    return -1;
  memset (pconfig, 0, sizeof (*pconfig));

  //NOTICE: Do not change any value!
  memcpy (&pconfig->config, sensor_config, 26);
  pconfig->reserved[0] = 1;

  gx_proto_crc32_calc ((uint8_t *) pconfig, sizeof (*pconfig) - PACKAGE_CRC_SIZE, (uint8_t *) &crc32_calc);

  memcpy (pconfig->crc_value, &crc32_calc, PACKAGE_CRC_SIZE);

  return 0;
}