//
// Created by Aero on 2/2/17.
//

#ifndef VN300_VN300_MSG_TYPES_H_H
#define VN300_VN300_MSG_TYPES_H_H


#include <stdint.h>
#include <ctype.h>
#include <stdbool.h>



typedef union
{
    float c[3];		//indexable

    /* Check if the compiler supports anonymous unions. */
#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) && defined(__GNUC__)
    struct
	{
		float x;	/**< X component. */
		float y;	/**< Y component. */
		float z;	/**< Z component. */
	};

	struct
	{
		float c0;	/**< Component 0. */
		float c1;	/**< Component 1. */
		float c2;	/**< Component 2. */
	};

#endif

} vn_vec3f;

/**
 * Represents a 3 component vector with an underlying data type of double
*/
typedef union
{
    double c[3];	/**< Indexable. */

    /* Check if the compiler supports anonymous unions. */
#if defined(__STDC_VERSION___) && (__STDC_VERSION__ >= 201112L) && defined(__GNUC__)

    struct
	{
		double x;	/**< The x component. */
		double y;	/**< The y component. */
		double z;	/**< The z component. */
	};

	struct
	{
		double c0;	/**< Component 0. */
		double c1;	/**< Component 1. */
		double c2;	/**< Component 2. */
	};

#endif

} vn_vec3d;



typedef uint16_t vn300_raw_status_t;

typedef float vn300_vel;
typedef double vn300_pos;
typedef vn_vec3d vn300_pos3_t;
typedef vn_vec3f vn300_vel3_t;


typedef vn300_vel3_t vn300_VelBody_t ;
typedef vn300_vel3_t vn300_VelNed_t ;
typedef vn300_pos3_t vn300_PosEcef_t ;
typedef vn300_pos3_t vn300_PosLla_t ;

typedef float vn_uncertainty;
typedef uint64_t vn_time_nanoseconds;


typedef struct {
    uint8_t* buf;
    uint32_t len;
} vn300_msg_buf_wrap_t;




typedef struct {
    vn_time_nanoseconds   gps_nanoseconds;
//    vn_vec3f              angular_rate;

    vn300_PosLla_t    pos_lla;
    vn300_PosEcef_t   pos_ecef;
    vn300_VelBody_t   vel_body;
    vn300_VelNed_t    vel_ned; //velocity in m/s

    vn_uncertainty    pos_uncertainty; // uncertainty (1 Sigma) in the current position estimate, in meters.
    vn_uncertainty    vel_uncertainty; // uncertainty (1 Sigma) in the current velocity estimate, in m/s.

} vn300_standard_msg_t;



#endif //VN300_VN300_MSG_TYPES_H_H
