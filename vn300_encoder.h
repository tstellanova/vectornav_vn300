//
// Created by Aero on 2/2/17.
//

#ifndef VN300_VN300_ENCODER_H
#define VN300_VN300_ENCODER_H


#include "vn300_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VN300_ENCODE_OK,
    VN300_ENCODE_BAD_INPUT,
    VN300_ENCODE_FAIL
} vn300_encode_res;


/**
 *
 * @param in A standard message struct
 * @param out An encoded message buffer of the proper size. Allocate with @see vn300_alloc_msg_wrap
 * @return One of the encoding error codes
 */
vn300_encode_res vn300_encode_standard_msg(vn300_standard_msg_t* in, vn300_msg_buf_wrap_t* out);



#ifdef __cplusplus
}
#endif


#endif //VN300_VN300_ENCODER_H
