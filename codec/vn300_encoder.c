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


  //message format
  //sync  (uint8_t)
  //groups (uint8_t)
  //group field1 (u16)
  //group field 2 (u16)
  //payload (n)
  //CRC (u16)

  //TODO actual encode
  pBuf[VN_HEADER_SYNC_OFF] = VECTORNAV_HEADER_SYNC_BYTE;
  pBuf[VN_HEADER_GROUPS_OFF] = (1<< 5); //select INS group only
  pBuf[VN_HEADER_GROUP1_OFF] = 0 ; //TODO
  pBuf[VN_HEADER_GROUP2_OFF] = 0 ; //TODO

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


