//
// Created by Aero on 2/2/17.
//

#ifndef VN300_VN300_MSG_INT_H
#define VN300_VN300_MSG_INT_H

/**
 *
 * @return  The length of the standard preconfigured message sent by VN300
 */
uint32_t vn300_standard_message_length() ;

// 8 bit xor checksum
uint8_t vn300_u8_checksum(uint8_t data[], uint32_t length);

//VN‐300 uses the CRC16‐CCITT
uint16_t vn300_u16_CRC(uint8_t data[], uint32_t length);


#define VECTORNAV_HEADER_SYNC_BYTE  0xFA

#define VN_HEADER_SYNC_OFF    0
#define VN_HEADER_GROUPS_OFF  1
#define VN_HEADER_GROUP1_OFF  2
#define VN_HEADER_GROUP1_LEN  2
#define VN_HEADER_GROUP2_OFF  4
#define VN_HEADER_GROUP2_LEN  2

#define VN_HEADER_PAYLOAD_OFF  6

#define VN_CRC_LEN  2


#define VN_GROUP_COUNT  6     /// number of Groups supported by VectorNav
#define VN_GROUP_FIELD_COUNT 16  /// max number of fields supported within a Group

enum {
    VN_GROUP_INDEX_STD = 0,
    VN_GROUP_INDEX_TIME,
    VN_GROUP_INDEX_IMU,
    VN_GROUP_INDEX_GPS,
    VN_GROUP_INDEX_ATT,
    VN_GROUP_INDEX_INS,
} VN_Group_Index;


// Binary Group 2 -- Time outputsx
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

#endif //VN300_VN300_MSG_INT_H
