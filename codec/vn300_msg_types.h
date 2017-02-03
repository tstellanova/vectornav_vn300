//
// Created by Aero on 2/2/17.
//

#ifndef VN300_VN300_MSG_TYPES_H_H
#define VN300_VN300_MSG_TYPES_H_H


#include <stdint.h>
#include <ctype.h>
#include <stdbool.h>



typedef uint16_t vn300_raw_status_t;

typedef float vn300_vel;
typedef double vn300_pos;
typedef vn300_pos vn300_pos3_t[3];
typedef vn300_vel vn300_vel3_t[3];

typedef vn300_vel3_t vn300_VelBody_t ;
typedef vn300_vel3_t vn300_VelNed_t ;
typedef vn300_pos3_t vn300_PosEcef_t ;
typedef vn300_pos3_t vn300_PosLla_t ;



typedef struct {
    uint8_t* buf;
    uint32_t len;
} vn300_msg_buf_wrap_t;

typedef struct {
    uint8_t mode; //Indicates the current mode of the INS filter:
    //0 = Not tracking. INS Filter is awaiting initialization.
    //1 = Aligning. INS Filter is dynamically aligning or aligning to GPS Compass solution.
    //2 = INS Filter is tracking and operating within specifications.

    bool gps_fix; //Indicates whether the GPS has a proper fix.
    bool gps_heading_ins; //Indicates if the INS is currently using the GPS compass heading solution.
    bool gps_compass; //Indicates if the GPS compass is operational and reporting a heading solution.
    bool imu_error;//true if IMU communication error is detected.
    bool mag_pres_error; //true if Magnetometer or Pressure sensor error is detected.
    bool gps_error; //true if GPS communication error is detected.

} vn300_ins_status_t;


typedef struct {
    vn300_ins_status_t ins_status;
    vn300_PosLla_t    pos_lla;
    vn300_PosEcef_t   pos_ecef;
    vn300_VelBody_t   vel_body;
    vn300_VelNed_t    vel_ned; //velocity in m/s
    vn300_vel         vel_uncertainty; // uncertainty (1 Sigma) in the current velocity estimate, in m/s.
    vn300_pos         pos_uncertainty; // uncertainty (1 Sigma) in the current position estimate, in meters.
} vn300_standard_msg_t;



#endif //VN300_VN300_MSG_TYPES_H_H
