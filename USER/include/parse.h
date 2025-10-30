/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: parse.h 4 2007-08-27 13:11:03Z xtimor $
 *
 */

#ifndef __NMEA_PARSE_H__
#define __NMEA_PARSE_H__

#include "sentence.h"

#ifdef  __cplusplus
extern "C" {
#endif

int nmea_pack_type(const char *buff, int buff_sz);
int nmea_find_tail(const char *buff, int buff_sz, int *res_crc);

int nmea_parse_GPGGA(const char *buff, int buff_sz, nmeaGPGGA *pack);
int nmea_parse_GPGSA(const char *buff, int buff_sz, nmeaGPGSA *pack);
int nmea_parse_GPGSV(const char *buff, int buff_sz, nmeaGPGSV *pack);
int nmea_parse_GPRMC(const char *buff, int buff_sz, nmeaGPRMC *pack);
int nmea_parse_GPVTG(const char *buff, int buff_sz, nmeaGPVTG *pack);

/*add-funciton*/
int nmea_parse_GLGSV(const char *buff, int buff_sz, nmeaGLGSV *pack);
int nmea_parse_GAGSV(const char *buff, int buff_sz, nmeaGAGSV *pack);
int nmea_parse_GQGSV(const char *buff, int buff_sz, nmeaGQGSV *pack);
int nmea_parse_KSXT(const char *buff, int buff_sz, nmeaKSXT *pack);
int nmea_parse_PSRDOPA(const char *buff, int buff_sz, nmeaPSRDOPA *pack);
int nmea_parse_GPHDT(const char *buff, int buff_sz, nmeaGPHDT *pack);
	
void nmea_GPGGA2info(nmeaGPGGA *pack, nmeaINFO *info);
void nmea_GPGSA2info(nmeaGPGSA *pack, nmeaINFO *info);
void nmea_GPGSV2info(nmeaGPGSV *pack, nmeaINFO *info);
void nmea_GPRMC2info(nmeaGPRMC *pack, nmeaINFO *info);
void nmea_GPVTG2info(nmeaGPVTG *pack, nmeaINFO *info);
void nmea_PSRDOPA2info(nmeaPSRDOPA *pack, nmeaINFO *info);

void nmea_KSXT2info(nmeaKSXT *pack, nmeaINFO *info);
void nmea_GPHDT2info(nmeaGPHDT *pack, nmeaINFO *info);

#ifdef  __cplusplus
}
#endif

#endif /* __NMEA_PARSE_H__ */
