//
// Created by Aero on 2/2/17.
//

#include "vn300_decoder.h"
#include "vn300_msg_types.h"
#include "vn300_msg_int.h"

#include <stdlib.h>




//double VnUtil_extractDouble(const char* pos)
//{
//  double f;
//  uint64_t tmp;
//
//  memcpy(&tmp, pos, sizeof(double));
//  tmp = stoh64(tmp);
//  memcpy(&f, &tmp, sizeof(double));
//
//  return f;
//}

vn300_decode_res decode_standard_msg(const vn300_msg_buf_wrap_t* in, vn300_standard_msg_t* out)
{
  if ((NULL == in) || (in->len < vn300_standard_message_length())) {
    return VN300_DECODE_EMPTY_INPUT;
  }

  if (VECTORNAV_HEADER_SYNC_BYTE != in->buf[VN_HEADER_Sync] ){
    return VN300_DECODE_BAD_INPUT;
  }

  uint32_t idx = 1;


  //TODO decode


  return VN300_DECODE_OK;
}
