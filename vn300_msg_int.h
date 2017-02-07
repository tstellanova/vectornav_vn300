//
// Created by Aero on 2/2/17.
//

#ifndef VN300_VN300_MSG_INT_H
#define VN300_VN300_MSG_INT_H



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


/**
 * Insert the standard msg header group fields in the given buffer.
 * This indicates which Groups have been selected,
 * and which Fields within those groups.
 * @param pBuf
 */
void vn_encode_standard_header_group_fields(uint8_t *pBuf);



/**
 * Our preconfigured VectorNav message format:
 *
 * sync  (uint8_t)
 * groups (uint8_t)
 * group field1...n (u16 each)
 * payload (n)
 * CRC (u16)
 */

#define VECTORNAV_HEADER_SYNC_BYTE  0xFA

#define VN_CRC_LEN  2

#define VN_GROUP_COUNT  6     /// Max number of Groups supported by VectorNav
#define VN_GROUP_FIELD_COUNT 16  /// Max number of fields supported within a Group

enum {
    VN_GROUP_INDEX_STD = 0,
    VN_GROUP_INDEX_TIME,
    VN_GROUP_INDEX_IMU,
    VN_GROUP_INDEX_GPS,
    VN_GROUP_INDEX_ATT,
    VN_GROUP_INDEX_INS,
} VN_Group_Index;


//Header contains selectors for four Groups
//Header is variable length depending on the number of groups active in the message
enum {
    VN_HEADER_Sync = 0,
    VN_HEADER_Groups = 1,
    //--- the following must be updated if we activate any new groups
            VN_HEADER_GroupFields_TIME = 2,
    VN_HEADER_GroupFields_IMU = 4,
    VN_HEADER_GroupFields_ATT = 6,
    VN_HEADER_GroupFields_INS = 8,
    VN_HEADER_Payload = 10
} VN_Header_Offset;


#define VN_HEADER_PAYLOAD_OFF (VN_HEADER_Payload)

/// The groups we've preconfigured to be active
#define VN300_SELECTED_GROUPS (\
    (1 << VN_GROUP_INDEX_TIME) & \
    (1 << VN_GROUP_INDEX_IMU) & \
    (1 << VN_GROUP_INDEX_ATT) & \
    (1 << VN_GROUP_INDEX_INS) \
)


// Binary Group 2 -- Time outputs
enum {
    VN_TIME_TimeStartup,
    VN_TIME_TimeGps,
    VN_TIME_GpsTow,
    VN_TIME_GpsWeek,
    VN_TIME_TimeSyncIn,
    VN_TIME_TimeGpsPps,
    VN_TIME_TimeUTC,
    VN_TIME_SyncInCnt,
    VN_TIME_RESV
} VN_Time_Group_Output;

#define VN300_TIME_SELECTED_FIELDS ( \
    (1 << VN_TIME_TimeGps) \
)

// Binary Group 3  -- IMU outputs
enum {
    VN_IMU_Status,
    VN_IMU_UncompMag,
    VN_IMU_UncompAccel,
    VN_IMU_UncompAngularRate, //VN_IMU_UncompGyro
    VN_IMU_Temp,
    VN_IMU_Pres,
    VN_IMU_DeltaTheta,
    VN_IMU_DeltaVel,
    VN_IMU_Mag,
    VN_IMU_Accel,
    VN_IMU_AngularRate,
    VN_IMU_SatFlags, //VN_IMU_SensSat
    VN_IMU_RESV
} VN_IMU_Group_Output;

#define VN300_IMU_SELECTED_FIELDS ( \
    (1 << VN_IMU_AngularRate) \
)

// Binary Group 4 -- GPS outputs
enum {
    VN_GPS_UTC,
    VN_GPS_Tow,
    VN_GPS_Week,
    VN_GPS_NumSats,
    VN_GPS_Fix,
    VN_GPS_PosLla,
    VN_GPS_PosEcef,
    VN_GPS_VelNed,
    VN_GPS_VelEcef,
    VN_GPS_PosU,
    VN_GPS_VelU,
    VN_GPS_TimeU,
    VN_GPS_Resv,
} VN_GPS_Group_Output;


// Binary Group 5 – Attitude Outputs
enum {
    VN_ATT_Reserved0,
    VN_ATT_YawPitchRoll,
    VN_ATT_Quaternion,
    VN_ATT_DCM,
    VN_ATT_MagNed,
    VN_ATT_AccelNed,
    VN_ATT_LinearAccelBody,
    VN_ATT_LinearAccelNed,
    VN_ATT_YprU,
    VN_ATT_Resv
} VN_ATT_Group_Output;

#define VN300_ATT_SELECTED_FIELDS ( \
    (1 << VN_ATT_YawPitchRoll) & \
    (1 << VN_ATT_Quaternion)  \
)

// Binary Group 6 – INS Outputs
enum {
    VN_INS_Status, // 2
    VN_INS_PosLla, // 24
    VN_INS_PosEcef, // 24
    VN_INS_VelBody, // 12
    VN_INS_VelNed, // 12
    VN_INS_VelEcef, // 12
    VN_INS_MagEcef, // 12
    VN_INS_AccelEcef, // 12
    VN_INS_LinearAccelEcef, //12
    VN_INS_PosU, // 4
    VN_INS_VelU, // 4
    VN_INS_Resv, //68 + 64
} VN_INS_GroupOutput;

#define VN300_INS_SELECTED_FIELDS ( \
    (1 << VN_INS_PosLla) &  \
    (1 << VN_INS_PosEcef) &  \
    (1 << VN_INS_VelBody) &  \
    (1 << VN_INS_PosU) &  \
    (1 << VN_INS_VelU) \
)


#endif //VN300_VN300_MSG_INT_H
