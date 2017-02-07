//
// Created by Todd Stellanova on 2/7/17.
//

#ifndef VN300_VN300_COMMON_H
#define VN300_VN300_COMMON_H


#include "vn300_msg_types.h"


vn300_msg_buf_wrap_t* vn300_alloc_msg_wrap(void);
void vn300_release_msg_wrap(vn300_msg_buf_wrap_t* wrap);




#endif //VN300_VN300_COMMON_H
