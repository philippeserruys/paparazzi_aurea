/*
 * Paparazzi mcu0 $Id$
 *
 * Copyright (C) 2003  Pascal Brisset, Antoine Drouin
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

/** \file xsens.c
 * \brief Parser for the Xsens protocol
 */

#include "ins_module.h"
#include "ins_xsens.h"

#include <inttypes.h>

#include "generated/airframe.h"

#include "mcu_periph/sys_time.h"
#include "subsystems/datalink/downlink.h"
#include "messages.h"

#if USE_GPS_XSENS
#include "subsystems/gps.h"
#include "math/pprz_geodetic_wgs84.h"
#include "math/pprz_geodetic_float.h"
#include "subsystems/navigation/common_nav.h" /* needed for nav_utm_zone0 */
#endif

INS_FORMAT ins_x;
INS_FORMAT ins_y;
INS_FORMAT ins_z;

INS_FORMAT ins_vx;
INS_FORMAT ins_vy;
INS_FORMAT ins_vz;

INS_FORMAT ins_phi;
INS_FORMAT ins_theta;
INS_FORMAT ins_psi;

INS_FORMAT ins_p;
INS_FORMAT ins_q;
INS_FORMAT ins_r;

INS_FORMAT ins_ax;
INS_FORMAT ins_ay;
INS_FORMAT ins_az;

INS_FORMAT ins_mx;
INS_FORMAT ins_my;
INS_FORMAT ins_mz;

float ins_pitch_neutral;
float ins_roll_neutral;


void ahrs_init(void)
{
  ins_init();
}


#ifdef USE_IMU

#include "subsystems/imu.h"

void imu_init(void)
{
  ins_init();
}

void imu_periodic(void)
{
  ins_periodic_task();
}

#endif

//////////////////////////////////////////////////////////////////////////////////////////
//
//	XSens Specific
//

volatile uint8_t ins_msg_received;

#define XsensInitCheksum() { send_ck = 0; }
#define XsensUpdateChecksum(c) { send_ck += c; }

#define XsensSend1(c) { uint8_t i8=c; InsUartSend1(i8); XsensUpdateChecksum(i8); }
#define XsensSend1ByAddr(x) { XsensSend1(*x); }
#define XsensSend2ByAddr(x) { XsensSend1(*(x+1)); XsensSend1(*x); }
#define XsensSend4ByAddr(x) { XsensSend1(*(x+3)); XsensSend1(*(x+2)); XsensSend1(*(x+1)); XsensSend1(*x); }

#define XsensHeader(msg_id, len) { \
  InsUartSend1(XSENS_START); \
  XsensInitCheksum(); \
  XsensSend1(XSENS_BID); \
  XsensSend1(msg_id); \
  XsensSend1(len); \
}
#define XsensTrailer() { uint8_t i8=0x100-send_ck; InsUartSend1(i8); }

/** Includes macros generated from xsens_MTi-G.xml */
#include "xsens_protocol.h"


#define XSENS_MAX_PAYLOAD 254
uint8_t xsens_msg_buf[XSENS_MAX_PAYLOAD];


#define UNINIT        0
#define GOT_START     1
#define GOT_BID       2
#define GOT_MID       3
#define GOT_LEN       4
#define GOT_DATA      5
#define GOT_CHECKSUM  6

// FIXME Debugging Only
#ifndef DOWNLINK_DEVICE
#define DOWNLINK_DEVICE DOWNLINK_AP_DEVICE
#endif
#include "mcu_periph/uart.h"
#include "messages.h"
#include "subsystems/datalink/downlink.h"


uint8_t xsens_errorcode;
uint8_t xsens_msg_status;
uint16_t xsens_time_stamp;
uint16_t xsens_output_mode;
uint32_t xsens_output_settings;
float xsens_declination = 0;
float xsens_gps_arm_x = 0;
float xsens_gps_arm_y = 0;
float xsens_gps_arm_z = 0;


int8_t xsens_hour;
int8_t xsens_min;
int8_t xsens_sec;
int32_t xsens_nanosec;
int16_t xsens_year;
int8_t xsens_month;
int8_t xsens_day;

static uint8_t xsens_id;
static uint8_t xsens_status;
static uint8_t xsens_len;
static uint8_t xsens_msg_idx;
static uint8_t ck;
uint8_t send_ck;

struct LlaCoor_f lla_f;
struct UtmCoor_f utm_f;

volatile int xsens_configured = 0;

void ins_init( void ) {

  xsens_status = UNINIT;
  xsens_configured = 20;

  ins_pitch_neutral = INS_PITCH_NEUTRAL_DEFAULT;
  ins_roll_neutral = INS_ROLL_NEUTRAL_DEFAULT;

  xsens_msg_status = 0;
  xsens_time_stamp = 0;

  gps.nb_channels = 0;
}

void ins_periodic_task( void ) {
  if (xsens_configured > 0)
  {
     switch (xsens_configured)
     {
	case 20:
		/* send mode and settings to MT */
		XSENS_GoToConfig();
		//XSENS_SetOutputMode(xsens_output_mode);
		//XSENS_SetOutputSettings(xsens_output_settings);
		break;
	case 18:
		// Give pulses on SyncOut
		//XSENS_SetSyncOutSettings(0,0x0002);
		break;
	case 17:
		// 1 pulse every 100 samples
		//XSENS_SetSyncOutSettings(1,100);
		break;
	case 2:
		//XSENS_ReqLeverArmGps();
		break;

	case 6:
		//XSENS_ReqMagneticDeclination();
		break;

	case 13:
		//#ifdef AHRS_H_X
		//#pragma message "Sending XSens Magnetic Declination."
		//xsens_declination = atan2(AHRS_H_Y, AHRS_H_X);
		//XSENS_SetMagneticDeclination(xsens_declination);
		//#endif
		break;
	case 12:
		#ifdef GPS_IMU_LEVER_ARM_X
		#pragma message "Sending XSens GPS Arm."
		XSENS_SetLeverArmGps(GPS_IMU_LEVER_ARM_X,GPS_IMU_LEVER_ARM_Y,GPS_IMU_LEVER_ARM_Z);
		#endif
		break;
	case 10:
		{
		uint8_t baud = 1;
		XSENS_SetBaudrate(baud);
		}
		break;

	case 1:
		XSENS_GoToMeasurment();
		break;
     }
     xsens_configured--;
     return;
  }

  //RunOnceEvery(100,XSENS_ReqGPSStatus());
}

#include "estimator.h"

void handle_ins_msg( void) {


  // Send to Estimator (Control)
#ifdef XSENS_BACKWARDS
  EstimatorSetAtt((-ins_phi+ins_roll_neutral), (-ins_psi + RadOfDeg(180)), (ins_theta+ins_pitch_neutral));
  EstimatorSetRate(ins_p,-ins_q, -ins_r);
#else
  EstimatorSetAtt(ins_phi+ins_roll_neutral, -ins_psi, -ins_theta+ins_pitch_neutral);
  EstimatorSetRate(ins_p, ins_q, ins_r);
#endif

  // Position
  float gps_east = gps.utm_pos.east / 100.;
  float gps_north = gps.utm_pos.north / 100.;
  gps_east -= nav_utm_east0;
  gps_north -= nav_utm_north0;
  EstimatorSetPosXY(gps_east, gps_north);

  // Altitude and vertical speed
  float hmsl = gps.hmsl;
  hmsl /= 1000.0f;
  EstimatorSetAlt(hmsl);

  #ifndef ALT_KALMAN
  #warning NO_VZ
  #endif

  // Horizontal speed
  float fspeed = sqrt(ins_vx*ins_vx + ins_vy*ins_vy);
  if (gps.fix != GPS_FIX_3D)
  {
    fspeed = 0;
  }
  float fclimb = -ins_vz;
  float fcourse = atan2f((float)ins_vy, (float)ins_vx);
  EstimatorSetSpeedPol(fspeed, fcourse, fclimb);

  // Now also finish filling the gps struct for telemetry purposes
  gps.gspeed = fspeed * 100.;
  gps.speed_3d = (uint16_t)(sqrt(ins_vx*ins_vx + ins_vy*ins_vy + ins_vz*ins_vz) * 100);
  gps.course = fcourse * 1e7;

}

void parse_ins_msg( void ) {
  uint8_t offset = 0;
  if (xsens_id == XSENS_ReqLeverArmGpsAck_ID) {
    xsens_gps_arm_x = XSENS_ReqLeverArmGpsAck_x(xsens_msg_buf);
    xsens_gps_arm_y = XSENS_ReqLeverArmGpsAck_y(xsens_msg_buf);
    xsens_gps_arm_z = XSENS_ReqLeverArmGpsAck_z(xsens_msg_buf);

    DOWNLINK_SEND_IMU_MAG_SETTINGS(DefaultChannel, DefaultDevice,&xsens_declination,&xsens_declination,&xsens_gps_arm_x,&xsens_gps_arm_y,&xsens_gps_arm_z);
  }
  else if (xsens_id == XSENS_Error_ID) {
    xsens_errorcode = XSENS_Error_errorcode(xsens_msg_buf);
  }
#if USE_GPS_XSENS
  else if (xsens_id == XSENS_GPSStatus_ID) {
    gps.nb_channels = XSENS_GPSStatus_nch(xsens_msg_buf);
    gps.num_sv = 0;

    gps.last_fix_time = sys_time.nb_sec;

    uint8_t i;
    // Do not write outside buffer
    for(i = 0; i < Min(gps.nb_channels, GPS_NB_CHANNELS); i++) {
      uint8_t ch = XSENS_GPSStatus_chn(xsens_msg_buf,i);
      if (ch > gps.nb_channels) continue;
      gps.svinfos[ch].svid = XSENS_GPSStatus_svid(xsens_msg_buf, i);
      gps.svinfos[ch].flags = XSENS_GPSStatus_bitmask(xsens_msg_buf, i);
      gps.svinfos[ch].qi = XSENS_GPSStatus_qi(xsens_msg_buf, i);
      gps.svinfos[ch].cno = XSENS_GPSStatus_cnr(xsens_msg_buf, i);
      if (gps.svinfos[ch].flags > 0)
      {
        gps.num_sv++;
      }
    }
  }
#endif
  else if (xsens_id == XSENS_MTData2_ID) {
    for (offset=0;offset<xsens_len;) {
      uint8_t code1 = xsens_msg_buf[offset];
      uint8_t code2 = xsens_msg_buf[offset+1];
      int subpacklen = (int)xsens_msg_buf[offset+2];
      offset += 3;


      if (code1 == 0x10)
      {
        if (code2 == 0x20)
        {
          // Packet Counter
        }
      }
      else if (code1 == 0x20)
      {
        if (code2 == 0x30)
        {
          // Attitude Euler
          ins_phi   = XSENS_DATA_Euler_roll(xsens_msg_buf,offset) * M_PI / 180;
          ins_theta = XSENS_DATA_Euler_pitch(xsens_msg_buf,offset) * M_PI / 180;
          ins_psi   = XSENS_DATA_Euler_yaw(xsens_msg_buf,offset) * M_PI / 180;

          new_ins_attitude = 1;
          
        }
      }
      else if (code1 == 0x40)
      {
        if (code2 == 0x10)
        {
          // Delta-V
          ins_ax = XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset)*100.0f;
          ins_ay = XSENS_DATA_Calibrated_accY(xsens_msg_buf,offset)*100.0f;
          ins_az = XSENS_DATA_Calibrated_accZ(xsens_msg_buf,offset)*100.0f;
        }
      }
      else if (code1 == 0x80)
      {
        if (code2 == 0x20)
        {
          // Rate Of Turn
          ins_p = XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset) * M_PI / 180;
          ins_q = XSENS_DATA_Calibrated_accY(xsens_msg_buf,offset) * M_PI / 180;
          ins_r = XSENS_DATA_Calibrated_accZ(xsens_msg_buf,offset) * M_PI / 180;
        }
      }
      else if (code1 == 0x30)
      {
        if (code2 == 0x10)
        {
          // Baro Pressure
        }
      }
      else if (code1 == 0xE0)
      {
        if (code2 == 0x20)
        {
          // Status Word
          xsens_msg_status = XSENS_DATA_Status_status(xsens_msg_buf,offset+3);
#if USE_GPS_XSENS
          if (bit_is_set(xsens_msg_status,2)) gps.fix = GPS_FIX_3D; // gps fix
          else if (bit_is_set(xsens_msg_status,1)) gps.fix = 0x01; // efk valid
          else gps.fix = GPS_FIX_NONE;
#endif
        }
      }
      else if (code1 == 0x88)
      {
        if (code2 == 0xA0)
        {
          // SVINFO
        }
      }
      else if (code1 == 0x50)
      {
        if (code2 == 0x20)
        {
          // Altitude Elipsoid
          gps.utm_pos.alt = XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset)* 1000.0f;

	  // Compute geoid (MSL) height
          float hmsl;
	  WGS84_ELLIPSOID_TO_GEOID(lla_f.lat,lla_f.lon,hmsl);
          gps.hmsl =  gps.utm_pos.alt - (hmsl * 1000.0f);
        }
        else if (code2 == 0x40)
        {
          // LatLong
#ifdef GPS_LED
          LED_TOGGLE(GPS_LED);
#endif
          gps.last_fix_time = sys_time.nb_sec;
          gps.week = 0; // FIXME
          gps.tow = 1; // FIXME XSENS_DATA_RAWGPS_itow(xsens_msg_buf,offset) * 10;
          lla_f.lat = RadOfDeg(XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset));
          lla_f.lon = RadOfDeg(XSENS_DATA_Calibrated_accY(xsens_msg_buf,offset));

          // Set the real UTM zone 
          gps.utm_pos.zone = (DegOfRad(lla_f.lon)+180) / 6 + 1;

          utm_f.zone = nav_utm_zone0;
          // convert to utm
          utm_of_lla_f(&utm_f, &lla_f);
          // copy results of utm conversion
          gps.utm_pos.east = utm_f.east*100;
          gps.utm_pos.north = utm_f.north*100;
        }
      }
      else if (code1 == 0xD0)
      {
        if (code2 == 0x10)
        {
          // Velocity
          ins_vx = ((INS_FORMAT)XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset)) / 100.;
          ins_vy = ((INS_FORMAT)XSENS_DATA_Calibrated_accY(xsens_msg_buf,offset)) / 100.;
          ins_vz = ((INS_FORMAT)XSENS_DATA_Calibrated_accZ(xsens_msg_buf,offset)) / 100.;
          gps.ned_vel.x = XSENS_DATA_Calibrated_accX(xsens_msg_buf,offset);
          gps.ned_vel.y = XSENS_DATA_Calibrated_accY(xsens_msg_buf,offset);
          gps.ned_vel.z = XSENS_DATA_Calibrated_accZ(xsens_msg_buf,offset);
        }
      }

      offset += subpacklen;
    }


/*

        gps.pacc = XSENS_DATA_RAWGPS_hacc(xsens_msg_buf,offset) / 100;
        gps.sacc = XSENS_DATA_RAWGPS_sacc(xsens_msg_buf,offset) / 100;
        gps.pdop = 5;  // FIXME Not output by XSens
*/

/*
*/


/*
          ins_mx = XSENS_DATA_Calibrated_magX(xsens_msg_buf,offset);
          ins_my = XSENS_DATA_Calibrated_magY(xsens_msg_buf,offset);
          ins_mz = XSENS_DATA_Calibrated_magZ(xsens_msg_buf,offset);
*/


/*
        xsens_time_stamp = XSENS_DATA_TimeStamp_ts(xsens_msg_buf,offset);
#if USE_GPS_XSENS
        gps.tow = xsens_time_stamp;
#endif
*/

  }

}


void parse_ins_buffer( uint8_t c ) {
  ck += c;
  switch (xsens_status) {
  case UNINIT:
    if (c != XSENS_START)
      goto error;
    xsens_status++;
    ck = 0;
    break;
  case GOT_START:
    if (c != XSENS_BID)
      goto error;
    xsens_status++;
    break;
  case GOT_BID:
    xsens_id = c;
    xsens_status++;
    break;
  case GOT_MID:
    xsens_len = c;
    if (xsens_len > XSENS_MAX_PAYLOAD)
      goto error;
    xsens_msg_idx = 0;
    xsens_status++;
    break;
  case GOT_LEN:
    xsens_msg_buf[xsens_msg_idx] = c;
    xsens_msg_idx++;
    if (xsens_msg_idx >= xsens_len)
      xsens_status++;
    break;
  case GOT_DATA:
    if (ck != 0)
      goto error;
    ins_msg_received = TRUE;
    goto restart;
    break;
  }
  return;
 error:
 restart:
  xsens_status = UNINIT;
  return;
}
