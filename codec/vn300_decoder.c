//
// Created by Aero on 2/2/17.
//

#include "vn300_decoder.h"
#include "vn300_msg_types.h"
#include "vn300_msg_int.h"

#include <stdlib.h>



#define VN300_GROUP_TIME  0
#define VN300_GROUP_IMU   1
#define VN300_GROUP_GPS   2
#define VN300_GROUP_ATT   3
#define VN300_GROUP_INS   4


vn300_decode_res decode_standard_msg(const vn300_msg_buf_wrap_t* in, vn300_standard_msg_t* out)
{
  if ((NULL == in) || (in->len < vn300_standard_message_length())) {
    return VN300_DECODE_EMPTY_INPUT;
  }

  if (VECTORNAV_HEADER_SYNC_BYTE != in->buf[VN_HEADER_SYNC_OFF] ){
    return VN300_DECODE_BAD_INPUT;
  }

  uint32_t idx = 1;


  //while < VN300_STANDARD_MSG_LEN


  return VN300_DECODE_OK;
}
