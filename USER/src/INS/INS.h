#ifndef __INS_H__
#define __INS_H__
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include "sys.h"


//#define PSINS_RMEMORY
#define GYRO_OUTPUT_ANGULAR_RATE_double//���������Ϊ����˫����
//#define GYRO_OUTPUT_ANGULAR_RATE_single//���������Ϊ���ʵ�����
#define IMU_Data_NoCalibration         //����������δ�궨
//#define INPUT_Heading_Information      //������Ϣд��
#define PSINS_MATRIX_MAX_DIM    15       //�������ά��
#define MMD		PSINS_MATRIX_MAX_DIM     //�������Ԫ����
#define MMD2	(MMD*MMD)                //�������Ԫ����




//����ȫ�ֱ���
extern const double wie; //������ת���ٶ�
extern const double g0; //g0
extern const double f; //�������
extern const double Re; //������뾶
extern const double Rp; //�������뾶
extern const double e2; //ƫ����ƽ��
extern const double e; //ƫ����
extern const double ppm;//ppm
extern const double d2r; //�ǻ�ת��
extern const double r2d; //����ת��
extern const int hur;//Сʱ
extern const int shur; //sqrt(hur)
extern const double b; //������ȡ�����
extern const double b1;
extern const double b2;
extern const double b3;
extern const double PI;
extern const double PI_2;
extern const double PI_4;
extern const double _2PI;
extern const double INF;     //�����
extern const double EPS;//����ֵ
extern const double dh2rs; //��ÿСʱת������ÿ��



//����ȫ�ֽṹ��
extern const struct CVect3 I31;
extern const struct CVect3 O31;
extern const struct CVect3 Ipos;          //��ά1����������������λλ������
extern const struct Quat qI;              //��λ��Ԫ��
extern const struct CMAT3 I33;
extern const struct CMAT3 O33;       //��ά��λ������ά�����



#define asinEx(x)		asin(Range(x, -1.0, 1.0))
#define acosEx(x)		acos(Range(x, -1.0, 1.0))//�����ȷ�����㷴���ҷ����������ڷ�Χ��


//�ṹ������*****//

struct CVect3 //��ά����
{
	double i, j, k;
};

struct CMAT3//��ά����
{
	double e00, e01, e02, e10, e11, e12, e20, e21, e22;
};

struct Quat//��Ԫ��
{
	double q0, q1, q2, q3;
};

struct CVect//��ά����
{
	int row, clm, rc;
	double dd[MMD];
};

struct CMat//��ά����
{
	int row, clm, rc;
	double dd[MMD2];
};

struct CEarth//�������
{
	double sl, cl, tl, sl2, s2l2, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;//γ��sin\cos\tan\sin2\sin2x2\Rm+h\Rn+h\λ��΢�־����з�����
	//	struct CVect3 pos, pos0, vn, vn0, wnie, wnen, wnin, gn, gcc; //λ�� ��һ��λ�� �ٶ� ��һ���ٶ� ������ת ǣ�����ٶ� �����������ٶ� �����ٶȷ���
	struct CVect3 pos, vn, wnie, wnen, wnin, gn, gcc; //λ�� ��һ��λ�� �ٶ� ��һ���ٶ� ������ת ǣ�����ٶ� �����������ٶ� �����ٶȷ���
	struct CMAT3 mpv, dotwninT;//λ�ø���΢�ַ��̾���
};


struct CIMU  //IMU��������
{
	int nSamples;//������
	double cimu_t;//����ʱ��
	bool singlefirst;//��������һ�μ���
	struct CVect3 phim, dvbm, wm_1, vm_1, The1, The2, V1, V2;//��Ч��תʸ��������������ǰһʱ�̽��������ٶ�����
	char rfu;//IMUָ��
};

struct Calibration//�궨����
{
	struct CMAT3 Ka, Kg;
	struct CVect3 db, eb;
};

struct Sensordata //����������
{
	double gyrodata[6];
	double accdata[6];
	int naviSampleCount;
};

struct CSINS  //����ʽ�ߵ�
{
	int x;//�����������ۼ�ȷ��ʱ��ļ���
	double heading;//ƫ����DDԺ
	double ts, nts, tk0, tk;//����ʱ�䣬����ʱ�䣬��ʼʱ��
	struct Quat qnb;  //��̬��Ԫ��
	struct CMAT3 Cnb, Cbn;//b->n�任����n->b�任����
	struct CVect3 wib, fb, fn, att;//bϵ�½��ٶȣ�bϵ�±�����nϵ�±���������״̬ת�ƾ����ã�����̬��
};

struct Kalman //12ά��׼
{
	struct CMat Fk, Qkf, Pk1, Pk, H, II;
	struct CMAT3 Maa, Mav, Mva, Mvv;
	struct CVect Xk, Zk, Q, Rkf, K, Pxrk;
	double nn;
	int nq, n, iter, nstep;
};

struct Kalman15 //12ά��׼
{
	struct CMat Fk, Qkf, Pk1, Pk, H, II;
	struct CMAT3 M1, Maa, Mav, Map, M2,M3, Mva, Mvv, Mvp, Mpv, Mpp;
	struct CVect Xk, Zk, Q, Rkf, K, Pxrk;
	double nn;
	int nq, n, iter, nstep;
};

struct GNSSDATA
{
 struct CVect3 GNSSpos; //λ��
 struct CVect3 GNSSvn;  //�ٶ�
 struct CVect3 GNSSerr;  //���
 struct CVect3 DOP;      //����˥������
 bool GNSSvalidity;     //������Ч
	//bool dataflag;       //���ݸ���
};
struct ICA //����ϵ�ֶ�׼
{
	double tk0, T2, T1;                     //��׼ʱ��       //��׼˫ʸ��ʱ��ѡȡ
	struct CIMU cimu;
	struct CMAT3 Cne, Cei, Cii, Cib, Cnb;//�����̬��
	struct Quat qib, qnb;                       //��Ԫ��
	struct CVect3 att0, Vi1, Vi2, Vib1, Vib2;  //��ʼ��̬�� iϵ�����ٶ�ʸ������ֵ����ʸ��ʵ��ֵ
	bool alignfinish, V1flag;                      //�ֶ�׼���� V1���ݲɼ�����
};

struct STICA  //����������ʽ��׼
{
	double t0, n;
	struct CVect3 G1, A1;
	struct CVect3 N1, W1, att0;
	struct CMAT3 Cnb;
	bool alignfinish;                      //�ֶ�׼���� V1���ݲɼ�����
};

struct PIICA // ������ʶ��׼
{
	struct CMAT3 PkE, PkN, KmulX;
	struct CVect3 Xk, Vn, KE, KN, ThetaE, ThetaN, U, Phi, atterr;
	double YkE, YkN, Lat, g;
	int count_num;
	bool alignfinish;
};

struct Result
{
	double Lati, Longi;
	float High;
	float VE, VN, VU;
	float pitch, roll, yaw;
	float fx, fy, fz;
	float gx, gy, gz;

};

//��������
void IniICA(struct ICA* ica, struct CVect3* pos0, double T, int s, double t);
void INSUpdate(struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, double* pwm, double* pvm);
void CoarAlign(struct ICA* ica, struct Calibration* Cal, double* pwm, double* pvm, double T);
void CIMUUpdate(struct CIMU* cimu, double* pwm, double* pvm);
void IniSTICA(struct STICA* ica, struct CVect3* pos);
void IniIns(struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct CVect3* att0, struct CVect3* vn0, struct CVect3* pos0, double tk0, double t, int n);
void CoarAlign0(struct STICA* ica, double* pwm, double* pvm, double  t, double T);
void IniKalman_D(struct Kalman* Kal, struct CVect3* pos, struct CVect3* Satt, struct CVect3* Svn, struct CVect3* Sgyro, struct CVect3* Sacc, struct CVect3* SWg, struct CVect3* SWa, double T);//��̬�ٶ�λ�ñ�׼����������Ʈ�ƣ��ӱ���ƫ�������ǡ��ӱ�������� �ߵ�����ʱ��
void Kal_TDupdate_D(struct Kalman* Kal, struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct Calibration* Cal, double* pwm, double* pvm);
void IniKalman_TD(struct Kalman15* Kal, struct CVect3* SWg, struct CVect3* SWa, struct CVect3* Stdatt, struct CVect3* Stdvn, struct CVect3* Stdpos, struct CVect3* Stdgyro, struct CVect3* Stdacc);
void Kal_INAV_UPdata(struct Kalman15* Kal, struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct GNSSDATA* GNSS,double* pwm, double* pvm, uint8_t* DataRevision);

struct CVect3* CV3protectmul1(struct CVect3* mul, struct CVect3* C, double a);
struct CVect3* operatormul3(struct CVect3* C1, struct CVect3* C2, struct CVect3* C);
struct CVect3* CV3add(struct CVect3* C1, struct CVect3* C2);
struct CVect3* CV3mul1(struct CVect3* C, double a);
struct CVect3* CV3protectadd(struct CVect3* C, struct CVect3* C1, struct CVect3* C2);


#endif
