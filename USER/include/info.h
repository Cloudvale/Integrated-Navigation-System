/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: info.h 10 2007-11-15 14:50:15Z xtimor $
 *
 */

/*! \file */

#ifndef __NMEA_INFO_H__
#define __NMEA_INFO_H__

#include "time.h"

#define NMEA_SIG_BAD        (0)
#define NMEA_SIG_LOW        (1)
#define NMEA_SIG_MID        (2)
#define NMEA_SIG_HIGH       (3)

#define NMEA_FIX_BAD        (1)
#define NMEA_FIX_2D         (2)
#define NMEA_FIX_3D         (3)

#define NMEA_MAXSAT         (12)
#define NMEA_SATINPACK      (4)
#define NMEA_NSATPACKS      (NMEA_MAXSAT / NMEA_SATINPACK)

#define NMEA_DEF_LAT        (5001.2621)
#define NMEA_DEF_LON        (3613.0595)

#ifdef  __cplusplus
extern "C" {
#endif

/**
 * Position data in fractional degrees or radians
 */
typedef struct _nmeaPOS
{
    double lat;         /**< Latitude */  //维度
    double lon;         /**< Longitude */ //经度

} nmeaPOS;

/**
 * Information about satellite
 * @see nmeaSATINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATELLITE
{
    int     id;         /**< Satellite PRN number */   //卫星PRN编号
    int     in_use;     /**< Used in position fix */   //用于定位
    int     elv;        /**< Elevation in degrees, 90 maximum */  //仰角，最大90
    int     azimuth;    /**< Azimuth, degrees from true north, 000 to 359 */  //方位角，与正北的度数 0-359
    int     sig;        /**< Signal, 00-99 dB */  //信号，0-99

} nmeaSATELLITE;

/**
 * Information about all satellites in view
 * @see nmeaINFO
 * @see nmeaGPGSV
 */
typedef struct _nmeaSATINFO
{
    int     inuse;      /**< Number of satellites in use (not those in view) */ //卫星使用数量
    int     inview;     /**< Total number of satellites in view */              //视图中的卫星数量
    nmeaSATELLITE sat[NMEA_MAXSAT]; /**< Satellites information */

} nmeaSATINFO;

/**
 * Summary GPS information from all parsed packets,
 * used also for generating NMEA stream
 * @see nmea_parse
 * @see nmea_GPGGA2info,  nmea_...2info
 */
typedef struct _nmeaINFO
{
    int     smask;      /**< Mask specifying types of packages from which data have been obtained */

    nmeaTIME utc;       /**< UTC of position */

    int     sig;        /**< GPS quality indicator (0 = Invalid; 1 = Fix; 2 = Differential, 3 = Sensitive) */ /*GPS质量指示符*/
    int     fix;        /**< Operating mode, used for navigation (1 = Fix not available; 2 = 2D; 3 = 3D) *//*GPGSA中的模式，固定,2D,3D*/

    double  PDOP;       /**< Position Dilution Of Precision */    /*位置精度因子*/
    double  HDOP;       /**< Horizontal Dilution Of Precision */  /*平面精度因子*/
    double  VDOP;       /**< Vertical Dilution Of Precision */    /*高程精度因子*/

   volatile double  lat;        /**< Latitude in NDEG - +/-[degree][min].[sec/60] */  //纬度
   volatile double  lon;        /**< Longitude in NDEG - +/-[degree][min].[sec/60] */ //经度
    double  elv;        /**< Antenna altitude above/below mean sea level (geoid) in meters */ //天线高度高于/低于平均海平面（大地水准面），单位为米
    double  speed;      /**< Speed over the ground in kilometers/hour */
    double  direction;  /**< Track angle in degrees True */ //轨道角度
    double  declination; /**< Magnetic variation degrees (Easterly var. subtracts from true course) */ //磁变化度（偏东方差减去真实航向）

	/*add-test*/
		double  EastVel;
		double  NorthVel;
		double  UpVel;
		double  TDOP;
		double  GDOP;
		double  Height;
		double  heading;
	
    nmeaSATINFO satinfo; /**< Satellites information */   //卫星信息

} nmeaINFO;

void nmea_zero_INFO(nmeaINFO *info);

#ifdef  __cplusplus
}
#endif

#endif /* __NMEA_INFO_H__ */
