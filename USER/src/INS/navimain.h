#ifndef __navimain_H__
#define __navimain_H__

extern struct STICA ica;
extern struct CSINS ins;
extern struct CEarth eth;
extern struct CIMU cimu;
extern struct Kalman15 Kal;
extern struct GNSSDATA GNSS;
extern struct Result Re1;

extern struct CVect3 pos;
extern struct CVect3 vn;
extern struct CVect3 SWg;
extern struct CVect3 SWa;
extern struct CVect3 Stdatt;
extern struct CVect3 Stdvn;
extern struct CVect3 Stdpos;
extern struct CVect3 Stdgyro;
extern struct CVect3 Stdacc;

extern struct CVect3 Gyrodata;
extern struct CVect3 Accdata;

extern struct Sensordata sen; //传感器参数
//extern struct Calibration Cal;//标定参数

extern double Pwm[3];
extern double Pvm[3];//静态误差补偿后数据


void Ininavimain(double Lati, double Longi, double High);
void navimain(double gyro_x_data, double gyro_y_data, double gyro_z_data, double accel_x_data, double accel_y_data, double accel_z_data, double Lat, double Long, double elv, uint8_t* DataRevision);


#endif
