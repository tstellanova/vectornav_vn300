//
// Created by Aero on 2/2/17.
//

#include "vn300_decoder.h"
#include "vn300_msg_types.h"
#include "vn300_msg_int.h"

#include <stdlib.h>
#include <memory.h>



extern const uint8_t kVNGroupFieldLengths[VN_GROUP_COUNT][VN_GROUP_FIELD_COUNT];


//TODO we're assuming same endianness here, might be unsafe

static void vn_decode_u16(uint16_t* dest, uint8_t** hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(uint16_t));
  *hInOut += sizeof(uint16_t);
}

static void vn_decode_crc(uint16_t* crc, uint8_t** hInOut)
{
  vn_decode_u16(crc,hInOut);
}

static void vn_decode_float(float* dest, uint8_t** hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(float));
  *hInOut += sizeof(float);
}

static void vn_decode_uncertainty(float* dest, uint8_t** hInOut)
{
  vn_decode_float(dest,hInOut);
}

static void vn_decode_vel(float* dest, uint8_t** hInOut)
{
  vn_decode_float(dest,hInOut);
}

static void vn_decode_vel3(vn300_vel3_t* dest, uint8_t** hInOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_decode_float(&dest->c[i] ,hInOut);
  }
}

static void vn_decode_double(double* dest, uint8_t **hInOut)
{
  uint8_t* pIn = *hInOut;
  memcpy(dest, pIn, sizeof(double));
  *hInOut += sizeof(double);
}


static void vn_decode_position(vn300_pos* dest, uint8_t **hInOut)
{
  vn_decode_double(dest, hInOut);
}

static void vn_decode_pos3(vn300_pos3_t *dest, uint8_t **hInOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_decode_double(&dest->c[i], hInOut);
  }
}



vn300_decode_res decode_standard_msg(const vn300_msg_buf_wrap_t* in, vn300_standard_msg_t* out)
{
  if ((NULL == in) || (in->len < vn300_standard_message_length())) {
    return VN300_DECODE_EMPTY_INPUT;
  }

  if (VECTORNAV_HEADER_SYNC_BYTE != in->buf[VN_HEADER_Sync] ){
    return VN300_DECODE_BAD_INPUT;
  }

  //Verify CRC before trying to process the packet
  //The CRC is calculated over the packet starting just after the sync byte in the header
  // (not including the sync byte) and ending at the end of the payload.
  const uint8_t* pCrcDataStart = (in->buf + 1); //skip SYNC byte
  const uint32_t kCrcDataLen = vn300_standard_message_length() - 1 - VN_CRC_LEN;
  uint16_t current_crc = vn_u16_crc(pCrcDataStart,kCrcDataLen);
  uint16_t input_crc = 0;
  uint8_t* pCrcRead = in->buf + (vn300_standard_message_length() - VN_CRC_LEN);
  vn_decode_crc(&input_crc, &pCrcRead);
  if (input_crc != current_crc) {
    return VN300_DECODE_BAD_CRC;
  }

  uint8_t* pBuf = in->buf;


  // see vn300_standard_payload_length for the fields included in the standard payload
  uint8_t groupIdx = 0;
  uint32_t field_len = 0;
  pBuf += VN_HEADER_Payload; //skip the header, get to the payload
  const uint8_t* payloadStart = pBuf;

  //====== TODO properly decode the following
  groupIdx = VN_GROUP_INDEX_TIME;
  field_len = kVNGroupFieldLengths[groupIdx][VN_TIME_TimeGpsPps];
  pBuf+=field_len;//TODO VN_TIME_TimeGpsPps

  groupIdx = VN_GROUP_INDEX_IMU;
  field_len = kVNGroupFieldLengths[groupIdx][VN_IMU_AngularRate];
  pBuf+=field_len;; //TODO VN_IMU_AngularRate

  groupIdx = VN_GROUP_INDEX_ATT;
  field_len =  kVNGroupFieldLengths[groupIdx][VN_ATT_YawPitchRoll];
  pBuf+=field_len; //TODO VN_ATT_YawPitchRoll
  field_len =  kVNGroupFieldLengths[groupIdx][VN_ATT_Quaternion];
  pBuf+=field_len; //TODO VN_ATT_Quaternion
  //====== TODO properly decode the above


  vn_decode_pos3(&out->pos_lla,&pBuf); //VN_INS_PosLla
  vn_decode_pos3(&out->pos_ecef,&pBuf); //VN_INS_PosEcef
  vn_decode_vel3(&out->vel_body,&pBuf);//VN_INS_VelBody
  vn_decode_vel3(&out->vel_ned,&pBuf);//VN_INS_VelNed

  vn_decode_uncertainty(&out->pos_uncertainty, &pBuf); //VN_INS_PosU
  vn_decode_uncertainty(&out->vel_uncertainty, &pBuf); //VN_INS_VelU


  const uint32_t payloadLen = (pBuf - payloadStart);
  if (payloadLen != vn300_standard_payload_length()) {
    return VN300_DECODE_FAIL;
  }



  return VN300_DECODE_OK;
}
