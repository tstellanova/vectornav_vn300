//
// Created by Todd Stellanova on 2/7/17.
//

#ifndef VN300_VN300_COMMON_H
#define VN300_VN300_COMMON_H


#include "vn300_msg_types.h"


vn300_msg_buf_wrap_t* vn300_alloc_msg_wrap(void);
void vn300_release_msg_wrap(vn300_msg_buf_wrap_t* wrap);

/**
 *
 * @return  The length of the payload for our preconfgured VN300 message
 */
uint32_t vn300_standard_payload_length(void);

/**
 *
 * @return  The total length of the standard preconfigured message sent by VN300
 */
uint32_t vn300_standard_message_length(void);


/**
 * Calculate VectorNav 16-bit CRC on data
 * VN‐300 uses the CRC16‐CCITT
 * @param data
 * @param length
 * @return  CRC16‐CCITT
 */
uint16_t vn_u16_crc(const uint8_t *data, uint32_t length);

/**
 * Calculate VectorNav 8-bit checksum on given data
 * @param data
 * @param length
 * @return 8 bit xor checksum
 */
uint8_t vn_u8_checksum(const uint8_t *data, uint32_t length);


#endif //VN300_VN300_COMMON_H
