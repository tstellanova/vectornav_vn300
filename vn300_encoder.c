//
// Created by Aero on 2/2/17.
//

#include <stdlib.h>
#include <string.h>

#include "vn300_encoder.h"
#include "vn300_msg_int.h"


//TODO we're assuming same endianness here, might be unsafe

static void vn_encode_float(float src, uint8_t** hOut)
{
  uint8_t* pOut = *hOut;
  memcpy(pOut, &src, sizeof(float));
  *hOut += sizeof(float);
}

static void vn_encode_double(double src, uint8_t **hOut)
{
  uint8_t* pOut = *hOut;
  memcpy(pOut, &src, sizeof(double));
  *hOut += sizeof(double);
}

static void vn_encode_u16(uint16_t src, uint8_t** hOut)
{
  uint8_t* pOut = *hOut;
  memcpy(pOut, &src, sizeof(uint16_t));
  *hOut += sizeof(uint16_t);
}

static void vn_encode_u64(const uint64_t* src, uint8_t** hOut)
{
  uint8_t* pOut = *hOut;
  memcpy(pOut, src, sizeof(uint64_t));
  *hOut += sizeof(uint64_t);
}

static void vn_encode_vec3f(vn_vec3f* in, uint8_t** hOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_encode_float(in->c[i], hOut);
  }
}

static void vn_encode_vec4f(vn_vec4f* in, uint8_t** hOut)
{
  for (uint8_t i = 0; i < 4; i++) {
    vn_encode_float(in->c[i], hOut);
  }
}

static void vn_encode_vec3d(vn_vec3d* in, uint8_t** hOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_encode_double(in->c[i], hOut);
  }
}

static void vn_encode_header_group_fields(uint16_t fields, uint8_t* pOut)
{
  memcpy(pOut, &fields, sizeof(uint16_t));
}

void vn_encode_standard_header_group_fields(uint8_t* pBuf)
{
  // encode header fields
  pBuf[VN_HEADER_Sync] = VECTORNAV_HEADER_SYNC_BYTE;
  pBuf[VN_HEADER_Groups] = VN300_SELECTED_GROUPS;
  vn_encode_header_group_fields(VN300_TIME_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_TIME]);
  vn_encode_header_group_fields(VN300_IMU_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_IMU]);
  vn_encode_header_group_fields(VN300_ATT_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_ATT]);
  vn_encode_header_group_fields(VN300_INS_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_INS]);
}

vn300_encode_res vn300_encode_standard_msg(vn300_standard_msg_t* in, vn300_msg_buf_wrap_t* out)
{
  if (NULL == in) {
    return VN300_ENCODE_BAD_INPUT;
  }

  //We assume that the buffer has already been allocated
  if ((NULL == out->buf) || (vn300_standard_message_length() != out->len)) {
    return VN300_ENCODE_BAD_INPUT;
  }

  uint8_t *pBuf =  out->buf;

  // encode header fields indicating which fields are active
  vn_encode_standard_header_group_fields(pBuf);

  // see vn300_standard_payload_length for the fields included in the standard payload
  pBuf += VN_HEADER_Payload; //skip the header, get to the payload
  const uint8_t* payloadStart = pBuf;

  vn_encode_u64(&in->gps_nanoseconds, &pBuf); //VN_TIME_TimeGps
  vn_encode_vec3f(&in->angular_rate, &pBuf); //VN_IMU_AngularRate
  vn_encode_vec3f(&in->euler_yaw_pitch_roll, &pBuf); //VN_ATT_YawPitchRoll
  vn_encode_vec4f(&in->att_quaternion, &pBuf); //VN_ATT_Quaternion

  vn_encode_vec3d(&in->pos_lla, &pBuf); //VN_INS_PosLla
  vn_encode_vec3d(&in->pos_ecef, &pBuf);//VN_INS_PosEcef
  vn_encode_vec3f(&in->vel_body, &pBuf);//VN_INS_VelBody
  vn_encode_vec3f(&in->vel_ned, &pBuf);//VN_INS_VelNed

  vn_encode_float(in->pos_uncertainty, &pBuf); //VN_INS_PosU
  vn_encode_float(in->vel_uncertainty, &pBuf); //VN_INS_VelU

  const uint32_t payloadLen = (pBuf - payloadStart);

  //The CRC is calculated over the packet starting just after the sync byte in the header
  // (not including the sync byte) and ending at the end of the payload.
  const uint8_t* pCrcDataStart = (out->buf + 1); //skip SYNC byte
  const uint32_t kCrcDataLen = vn300_standard_message_length() - 1 - VN_CRC_LEN;
  uint16_t current_crc = vn_u16_crc(pCrcDataStart,kCrcDataLen);

  vn_encode_u16(current_crc, &pBuf);


  const uint32_t expectedPayloadLen = vn300_standard_payload_length();
  uint32_t encodedLen = (pBuf - out->buf );
  if ((payloadLen != expectedPayloadLen) || (out->len != encodedLen )) {
    return VN300_ENCODE_FAIL;
  }

  return VN300_ENCODE_OK;
}


