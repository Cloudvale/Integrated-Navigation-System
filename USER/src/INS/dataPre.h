#ifndef __dataPre_H__
#define __dataPre_H__
#include "INS.h"
#include "navimain.h"

//extern const double Ka11;
//extern const double Ka12;
//extern const double Ka13;
//extern const double Ka21;
//extern const double Ka22;
//extern const double Ka23;
//extern const double Ka31;
//extern const double Ka32;
//extern const double Ka33;
//extern const double db1;
//extern const double db2;
//extern const double db3;//Accbias
//
//extern const double Kg11;
//extern const double Kg12;
//extern const double Kg13;
//extern const double Kg21;
//extern const double Kg22;
//extern const double Kg23;
//extern const double Kg31;
//extern const double Kg32;
//extern const double Kg33;
//extern const double eb1;
//extern const double eb2;
//extern const double eb3;//gyrobias


typedef struct _calibrationPara {
	double x_gyro_bias;
	double y_gyro_bias;
	double z_gyro_bias;
	double x_accel_bias;
	double y_accel_bias;
	double z_accel_bias;

	double gyroInstallErr_Inv11;
	double gyroInstallErr_Inv12;
	double gyroInstallErr_Inv13;
	double gyroInstallErr_Inv21;
	double gyroInstallErr_Inv22;
	double gyroInstallErr_Inv23;
	double gyroInstallErr_Inv31;
	double gyroInstallErr_Inv32;
	double gyroInstallErr_Inv33;

	double accelInstallErr_Inv11;
	double accelInstallErr_Inv12;
	double accelInstallErr_Inv13;
	double accelInstallErr_Inv21;
	double accelInstallErr_Inv22;
	double accelInstallErr_Inv23;
	double accelInstallErr_Inv31;
	double accelInstallErr_Inv32;
	double accelInstallErr_Inv33;
}__calibrationPara;


void IniCIMU(struct CIMU* cimu, double t, int s);
void Calibrate(struct Calibration* Cal, double* pwm, double* pvm, double gyro_x_data, double gyro_y_data, double gyro_z_data, double accel_x_data, double accel_y_data, double accel_z_data, double g);
void Sensordataassign(struct Sensordata* Sen, double pwm[3], double pvm[3], int nSamples);//
void getCalibration(struct Calibration* Cal);

#endif
