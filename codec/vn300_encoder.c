//
// Created by Aero on 2/2/17.
//

#include <stdlib.h>
#include <memory.h>
#include <printf.h>

#include "vn300_msg_types.h"
#include "vn300_msg_int.h"
#include "vn300_encoder.h"

extern const uint8_t kVNGroupFieldLengths[VN_GROUP_COUNT][VN_GROUP_FIELD_COUNT];


static void vn_encode_float(float src, uint8_t** hOut)
{
  uint8_t* pOut = *hOut;
  memcpy(pOut, &src, sizeof(float));
  *hOut += sizeof(float);
}

static void vn_encode_uncertainty(float src, uint8_t** hOut)
{
  vn_encode_float(src, hOut);
}


static void vn_encode_vel(float src, uint8_t** hOut)
{
  vn_encode_float(src, hOut);
}

static void vn_encode_vel3(vn300_vel3_t* in, uint8_t** hOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_encode_vel(in->c[i], hOut);
  }
}

static void vn_write_double(double src, uint8_t** hOut)
{
  uint8_t* pOut = *hOut;
  memcpy(pOut, &src, sizeof(double));
  *hOut += sizeof(double);
}


static void vn_encode_position(vn300_pos in, uint8_t **hOut)
{
  vn_write_double((double)in, hOut);
}

static void vn_encoded_pos3(vn300_pos3_t *in, uint8_t **hOut)
{
  for (uint8_t i = 0; i < 3; i++) {
    vn_encode_position(in->c[i], hOut);
  }
}


static void vn_encode_header_group_fields(uint16_t fields, uint8_t* pOut)
{
  memcpy(pOut, &fields, sizeof(uint16_t));
}

vn300_encode_res encode_standard_msg(vn300_standard_msg_t* in, vn300_msg_buf_wrap_t* out)
{
  if (NULL == in) {
    return VN300_ENCODE_BAD_INPUT;
  }

  uint8_t *pBuf =  malloc(vn300_standard_message_length());
  if (NULL == pBuf) {
    return VN300_ENCODE_FAIL;
  }
  out->buf = pBuf;
  out->len = vn300_standard_message_length();


  // encode header fields
  pBuf[VN_HEADER_Sync] = VECTORNAV_HEADER_SYNC_BYTE;
  pBuf[VN_HEADER_Groups] = VN300_SELECTED_GROUPS;
  vn_encode_header_group_fields(VN300_TIME_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_TIME]);
  vn_encode_header_group_fields(VN300_IMU_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_IMU]);
  vn_encode_header_group_fields(VN300_ATT_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_ATT]);
  vn_encode_header_group_fields(VN300_INS_SELECTED_FIELDS,  &pBuf[VN_HEADER_GroupFields_INS]);


  // see vn300_standard_payload_length for the fields included in the standard payload
  uint8_t groupIdx = 0;
  uint32_t field_len = 0;
  pBuf += VN_HEADER_Payload; //skip the header, get to the payload
  const uint8_t* payloadStart = pBuf;

  //TODO properly encode the following
  groupIdx = VN_GROUP_INDEX_TIME;
  field_len = kVNGroupFieldLengths[groupIdx][VN_TIME_TimeGpsPps];
  pBuf+=field_len;//TODO VN_TIME_TimeGpsPps
  field_len = kVNGroupFieldLengths[groupIdx][VN_TIME_TimeUTC];
  pBuf+=field_len; //TODO VN_TIME_TimeUTC

  groupIdx = VN_GROUP_INDEX_IMU;
  field_len = kVNGroupFieldLengths[groupIdx][VN_IMU_AngularRate];
  pBuf+=field_len;; //TODO VN_IMU_AngularRate

  groupIdx = VN_GROUP_INDEX_ATT;
  field_len =  kVNGroupFieldLengths[groupIdx][VN_ATT_YawPitchRoll];
  pBuf+=field_len; //TODO VN_ATT_YawPitchRoll
  field_len =  kVNGroupFieldLengths[groupIdx][VN_ATT_Quaternion];
  pBuf+=field_len; //TODO VN_ATT_Quaternion

  vn_encoded_pos3(&in->pos_lla, &pBuf); //VN_INS_PosLla
  vn_encoded_pos3(&in->pos_ecef,&pBuf);//VN_INS_PosEcef
  vn_encode_vel3(&in->vel_body, &pBuf);//VN_INS_VelBody
  vn_encode_vel3(&in->vel_ned, &pBuf);//VN_INS_VelNed

  vn_encode_uncertainty(in->pos_uncertainty, &pBuf); //VN_INS_PosU
  vn_encode_uncertainty(in->vel_uncertainty, &pBuf); //VN_INS_VelU

  const uint32_t payloadLen = (pBuf - payloadStart);

  pBuf += VN_CRC_LEN; //TODO add CRC

  const uint32_t expectedPayloadLen = vn300_standard_payload_length();
  uint32_t encodedLen = (pBuf - out->buf );
  if ((payloadLen != expectedPayloadLen) || (out->len != encodedLen )) {
    printf("payloadLen: %d  expected: %d",payloadLen, expectedPayloadLen);
    return VN300_ENCODE_FAIL;
  }

  return VN300_ENCODE_OK;
}


