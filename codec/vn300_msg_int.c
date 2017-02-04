//
// Created by Aero on 2/2/17.
//

#include <stdint.h>
#include "vn300_msg_int.h"

uint8_t vn_u8_checksum(const uint8_t *data, uint32_t length)
{
  uint8_t cksum = 0;
  for(uint32_t i=0; i<length; i++){
    cksum ^= data[i];
  }
  return cksum;
}

uint16_t vn_u16_crc(const uint8_t *data, uint32_t length)
{
  uint16_t crc = 0;
  for(uint32_t i=0; i<length; i++){
    crc  = (uint8_t)(crc >> 8) | (crc << 8);
    crc ^= data[i];
    crc ^= (uint8_t)(crc & 0xff) >> 4;
    crc ^= crc << 12;
    crc ^= (crc & 0x00ff) << 5;
  }
  return crc;
}



/**
 2D array to determine the payload length for a binary output packet.
  - The first index of the array is the group index.
  - The second index is the group field index.

 Both indices are assumed to be zero based.
 Max length of any field is 256.
 @see VN_Group_Index
 */
const uint8_t kVNGroupFieldLengths[VN_GROUP_COUNT][VN_GROUP_FIELD_COUNT] =
    {
        {8, 8, 8, 12, 16, 12, 24, 12, 12, 24, 20, 28, 2, 4, 8, 0}, //Group 1 (standard combo)
        {8, 8, 8, 2, 8, 8, 8, 4, 0, 0, 0, 0, 0, 0, 0, 0}, //Group (TIME)
        {2, 12, 12, 12, 4, 4, 16, 12, 12, 12, 12, 2, 40, 0, 0, 0}, //Group 3 (IMU)
        {8, 8, 2, 1, 1, 24, 24, 12, 12, 12, 4, 4, 0, 0, 0, 0}, //Group 4 (GPS)
        {2, 12, 16, 36, 12, 12, 12, 12, 12, 12, 28, 24, 0, 0, 0, 0}, // Group 5 (ATT)
        {2, 24, 24, 12, 12, 12, 12, 12, 12, 4, 4, 68, 64, 0, 0, 0}, // Group 6 (INS)
    };



//this defines the format of the standard payload
uint32_t vn300_standard_payload_length() {
  static uint32_t precalc_len = 0;
  if (0 == precalc_len) {
    precalc_len =
    // Group TIME
    kVNGroupFieldLengths[VN_GROUP_INDEX_TIME][VN_TIME_TimeGpsPps] +
    // Group IMU
    kVNGroupFieldLengths[VN_GROUP_INDEX_IMU][VN_IMU_AngularRate] +
    // Group GPS
    // Group ATT
    kVNGroupFieldLengths[VN_GROUP_INDEX_ATT][VN_ATT_YawPitchRoll] +
    kVNGroupFieldLengths[VN_GROUP_INDEX_ATT][VN_ATT_Quaternion] +
    // Group VN_GROUP_INDEX_INS
    kVNGroupFieldLengths[VN_GROUP_INDEX_INS][VN_INS_PosLla] +
    kVNGroupFieldLengths[VN_GROUP_INDEX_INS][VN_INS_PosEcef] +
    kVNGroupFieldLengths[VN_GROUP_INDEX_INS][VN_INS_VelBody] +
    kVNGroupFieldLengths[VN_GROUP_INDEX_INS][VN_INS_VelNed] +
    kVNGroupFieldLengths[VN_GROUP_INDEX_INS][VN_INS_PosU] +
    kVNGroupFieldLengths[VN_GROUP_INDEX_INS][VN_INS_VelU] +
    0;
  }

  return precalc_len;
}


uint32_t vn300_standard_message_length() {
  static uint32_t precalc_len = 0;
  if (0 == precalc_len) {
    uint32_t payload_len = vn300_standard_payload_length();
    precalc_len = VN_HEADER_PAYLOAD_OFF + payload_len + VN_CRC_LEN;
  }

  return precalc_len;
}

