//
// Created by Todd Stellanova on 2/7/17.
//

#ifndef VN300_COMMON_H
#define VN300_COMMON_H


#include "vn300_msg_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/// Byte used to indicate the start of a VectorNav binary message
#define VECTORNAV_HEADER_SYNC_BYTE  0xFA


vn300_msg_buf_wrap_t *vn300_alloc_msg_wrap(void);

void vn300_release_msg_wrap(vn300_msg_buf_wrap_t *wrap);


#ifdef __cplusplus
}
#endif

#endif //VN300_COMMON_H
