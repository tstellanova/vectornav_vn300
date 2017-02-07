//
// Created by Aero on 2/2/17.
//

#ifndef VN300_DECODER_H
#define VN300_DECODER_H


#include "vn300_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    VN300_DECODE_OK,
    VN300_DECODE_EMPTY_INPUT,
    VN300_DECODE_BAD_INPUT,
    VN300_DECODE_BAD_CRC,
    VN300_DECODE_FAIL
} vn300_decode_res;

/**
 *
 * @param buf A complete wrapped buffer with proper length (@see vn300_standard_message_length)
 * @param out
 * @return One of the decoding result codes
 */
vn300_decode_res vn300_decode_standard_msg(const vn300_msg_buf_wrap_t *buf, vn300_standard_msg_t *out);


#ifdef __cplusplus
}
#endif

#endif //VN300_DECODER_H
