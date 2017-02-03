//
// Created by Aero on 2/2/17.
//

#ifndef VN300_DECODER_H
#define VN300_DECODER_H


#include "vn300_msg_types.h"


typedef enum {
    VN300_DECODE_OK,
    VN300_DECODE_EMPTY_INPUT,
    VN300_DECODE_BAD_INPUT,
    VN300_DECODE_FAIL
} vn300_decode_res;


vn300_decode_res decode_standard_msg(const vn300_msg_buf_wrap_t* buf, vn300_standard_msg_t* out);

#endif //VN300_DECODER_H
