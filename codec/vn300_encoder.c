//
// Created by Aero on 2/2/17.
//

#include <stdlib.h>

#include "vn300_msg_types.h"
#include "vn300_msg_int.h"
#include "vn300_encoder.h"

extern kVN_Group_Field_Lengths;

vn300_encode_res encode_standard_msg(vn300_standard_msg_t* in, vn300_msg_buf_wrap_t* out)
{
  if (NULL == in) {
    return VN300_ENCODE_BAD_INPUT;
  }

  uint8_t *pBuf =  malloc(vn300_standard_message_length());
  if (NULL == out->buf) {
    return VN300_ENCODE_FAIL;
  }
  out->len = vn300_standard_message_length();
  out->buf = pBuf;

  pBuf[VN_HEADER_Sync] = VECTORNAV_HEADER_SYNC_BYTE;
  pBuf[VN_HEADER_Groups] = VN300_SELECTED_GROUPS;
  pBuf[VN_HEADER_GroupFields_TIME] = VN300_TIME_SELECTED_FIELDS;
  pBuf[VN_HEADER_GroupFields_IMU] = VN300_IMU_SELECTED_FIELDS;
  pBuf[VN_HEADER_GroupFields_ATT] = VN300_ATT_SELECTED_FIELDS;
  pBuf[VN_HEADER_GroupFields_INS] = VN300_INS_SELECTED_FIELDS;


  //TODO actual encode

  // see vn300_standard_payload_length for the fields included in the standard payload
  uint32_t idx = VN_HEADER_Payload;

  uint32_t field_len = 0;

  field_len = kVN_Group_Field_Lengths[VN_GROUP_INDEX_TIME][VN_TIME_TimeGpsPps];
  pBuf[idx+=field_len] = 0; //TODO VN_TIME_TimeGpsPps
  field_len = kVN_Group_Field_Lengths[VN_GROUP_INDEX_TIME][VN_TIME_TimeUTC];
  pBuf[idx+=field_len] = 0; //TODO VN_TIME_TimeUTC

  field_len = kVN_Group_Field_Lengths[VN_GROUP_INDEX_IMU][VN_IMU_AngularRate];
  pBuf[idx+=field_len] = 0; //TODO VN_IMU_AngularRate

  field_len =  kVN_Group_Field_Lengths[VN_GROUP_INDEX_ATT][VN_ATT_YawPitchRoll];
  pBuf[idx+=field_len] = 0; //TODO VN_ATT_YawPitchRoll

  field_len =  kVN_Group_Field_Lengths[VN_GROUP_INDEX_ATT][VN_ATT_Quaternion];
  pBuf[idx+=field_len] = 0; //TODO VN_ATT_Quaternion


//  // Group VN_GROUP_INDEX_INS
//  kVN_Group_Field_Lengths[VN_GROUP_INDEX_INS][VN_INS_Status] +
//  kVN_Group_Field_Lengths[VN_GROUP_INDEX_INS][VN_INS_PosLla] +
//  kVN_Group_Field_Lengths[VN_GROUP_INDEX_INS][VN_INS_PosEcef] +
//  kVN_Group_Field_Lengths[VN_GROUP_INDEX_INS][VN_INS_VelBody] +
//  kVN_Group_Field_Lengths[VN_GROUP_INDEX_INS][VN_INS_PosU] +
//  kVN_Group_Field_Lengths[VN_GROUP_INDEX_INS][VN_INS_VelU] +



  uint32_t idx = VN_HEADER_PAYLOAD_OFF;

  //TODO encode every field



//The header is variable length depending upon the number of groups active in the message.
  //num groups = 1



//  vn300_ins_status_t ins_status;
//  vn300_PosLla_t    pos_lla;
//  vn300_PosEcef_t   pos_ecef;
//  vn300_VelBody_t   vel_body;
//  vn300_VelNed_t    vel_ned; //velocity in m/s
//  vn300_vel         vel_uncertainty; //estimated uncertainty (1 Sigma) in the current velocity estimate, given in m/s.
//  vn300_pos         pos_uncertainty; //estimated uncertainty (1 Sigma) in the current position estimate, given in meters.

  return VN300_ENCODE_FAIL;
}


