//
// Created by Aero on 2/2/17.
//

#ifndef VN300_VN300_ENCODER_H
#define VN300_VN300_ENCODER_H


#include "vn300_msg_types.h"


typedef enum {
    VN300_ENCODE_OK,
    VN300_ENCODE_BAD_INPUT,
    VN300_ENCODE_FAIL
} vn300_encode_res;


vn300_encode_res encode_standard_msg(vn300_standard_msg_t* in, vn300_msg_buf_wrap_t* out);


#endif //VN300_VN300_ENCODER_H
