//
// Created by Aero on 2/2/17.
//


#include <stdlib.h>
#include <string.h>



#include "vn300_decoder.h"
#include "vn300_msg_int.h"


//TODO we're assuming same endianness here, might be unsafe

static void vn_decode_u16(uint16_t* dest, uint8_t** hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(uint16_t));
  *hInOut += sizeof(uint16_t);
}

static void vn_decode_u64(uint64_t* dest, uint8_t** hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(uint64_t));
  *hInOut += sizeof(uint64_t);
}

static void vn_decode_float(float* dest, uint8_t** hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(float));
  *hInOut += sizeof(float);
}

static void vn_decode_double(double* dest, uint8_t **hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(double));
  *hInOut += sizeof(double);
}

static void vn_decode_vec3f(vn_vec3f* dest, uint8_t** hInOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_decode_float(&dest->c[i] , hInOut);
  }
}

static void vn_decode_vec4f(vn_vec4f* dest, uint8_t** hInOut)
{
  for (uint8_t i = 0; i < 4; i++) {
    vn_decode_float(&dest->c[i] , hInOut);
  }
}

static void vn_decode_vec3d(vn_pos3_t *dest, uint8_t **hInOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_decode_double(&dest->c[i], hInOut);
  }
}

vn300_decode_res vn300_decode_standard_msg(const vn300_msg_buf_wrap_t *in, vn300_standard_msg_t *out)
{
  if ((NULL == in) || (in->len < vn300_standard_message_length())) {
    return VN300_DECODE_EMPTY_INPUT;
  }

  if (VECTORNAV_HEADER_SYNC_BYTE != in->buf[VN_HEADER_Sync] ){
    return VN300_DECODE_BAD_INPUT;
  }

  //clear any state in the output decoded msg
  memset(out, 0, sizeof(vn300_standard_msg_t));

  //Verify CRC before trying to process the packet
  //The CRC is calculated over the packet starting just after the sync byte in the header
  // (not including the sync byte) and ending at the end of the payload.
  const uint8_t* pCrcDataStart = (in->buf + 1); //skip SYNC byte
  const uint32_t kCrcDataLen = vn300_standard_message_length() - 1 - VN_CRC_LEN;
  uint16_t current_crc = vn_u16_crc(pCrcDataStart,kCrcDataLen);
  uint16_t input_crc = 0;
  uint8_t* pCrcRead = in->buf + (vn300_standard_message_length() - VN_CRC_LEN);
  vn_decode_u16(&input_crc, &pCrcRead);
  if (input_crc != current_crc) {
    return VN300_DECODE_BAD_CRC;
  }

  uint8_t* pBuf = in->buf;

  // see vn300_standard_payload_length for the fields included in the standard payload
  pBuf += VN_HEADER_Payload; //skip the header, get to the payload
  const uint8_t* payloadStart = pBuf;


  vn_decode_u64(&out->gps_nanoseconds, &pBuf); //VN_TIME_TimeGps
  vn_decode_vec3f(&out->angular_rate, &pBuf); //VN_IMU_AngularRate
  vn_decode_vec3f(&out->euler_yaw_pitch_roll, &pBuf); //VN_ATT_YawPitchRoll
  vn_decode_vec4f(&out->att_quaternion, &pBuf); //VN_ATT_Quaternion

  vn_decode_vec3d(&out->pos_lla,&pBuf); //VN_INS_PosLla
  vn_decode_vec3d(&out->pos_ecef,&pBuf); //VN_INS_PosEcef
  vn_decode_vec3f(&out->vel_body,&pBuf);//VN_INS_VelBody
  vn_decode_vec3f(&out->vel_ned,&pBuf);//VN_INS_VelNed

  vn_decode_float(&out->pos_uncertainty, &pBuf); //VN_INS_PosU
  vn_decode_float(&out->vel_uncertainty, &pBuf); //VN_INS_VelU


  const uint32_t payloadLen = (pBuf - payloadStart);
  if (payloadLen != vn300_standard_payload_length()) {
    return VN300_DECODE_FAIL;
  }



  return VN300_DECODE_OK;
}
