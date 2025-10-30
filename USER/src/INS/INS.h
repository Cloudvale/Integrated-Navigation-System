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
#define GYRO_OUTPUT_ANGULAR_RATE_double//传感器输出为速率双子样
//#define GYRO_OUTPUT_ANGULAR_RATE_single//传感器输出为速率单子样
#define IMU_Data_NoCalibration         //传感器数据未标定
//#define INPUT_Heading_Information      //航向信息写入
#define PSINS_MATRIX_MAX_DIM    15       //矩阵最大维度
#define MMD		PSINS_MATRIX_MAX_DIM     //数组最大元素数
#define MMD2	(MMD*MMD)                //矩阵最大元素数




//声明全局变量
extern const double wie; //地球自转角速度
extern const double g0; //g0
extern const double f; //地球扁率
extern const double Re; //地球长轴半径
extern const double Rp; //地球短轴半径
extern const double e2; //偏心率平方
extern const double e; //偏心率
extern const double ppm;//ppm
extern const double d2r; //角弧转换
extern const double r2d; //弧角转换
extern const int hur;//小时
extern const int shur; //sqrt(hur)
extern const double b; //重力求取相关量
extern const double b1;
extern const double b2;
extern const double b3;
extern const double PI;
extern const double PI_2;
extern const double PI_4;
extern const double _2PI;
extern const double INF;     //无穷大
extern const double EPS;//零阈值
extern const double dh2rs; //度每小时转换弧度每秒



//声明全局结构体
extern const struct CVect3 I31;
extern const struct CVect3 O31;
extern const struct CVect3 Ipos;          //三维1向量、零向量、单位位置向量
extern const struct Quat qI;              //单位四元数
extern const struct CMAT3 I33;
extern const struct CMAT3 O33;       //三维单位矩阵、三维零矩阵



#define asinEx(x)		asin(Range(x, -1.0, 1.0))
#define acosEx(x)		acos(Range(x, -1.0, 1.0))//定义宏确保计算反余弦反正弦输入在范围内


//结构体声明*****//

struct CVect3 //三维向量
{
	double i, j, k;
};

struct CMAT3//三维矩阵
{
	double e00, e01, e02, e10, e11, e12, e20, e21, e22;
};

struct Quat//四元数
{
	double q0, q1, q2, q3;
};

struct CVect//多维向量
{
	int row, clm, rc;
	double dd[MMD];
};

struct CMat//多维矩阵
{
	int row, clm, rc;
	double dd[MMD2];
};

struct CEarth//地球参数
{
	double sl, cl, tl, sl2, s2l2, RMh, RNh, clRNh, f_RMh, f_RNh, f_clRNh;//纬度sin\cos\tan\sin2\sin2x2\Rm+h\Rn+h\位置微分矩阵中非零项
	//	struct CVect3 pos, pos0, vn, vn0, wnie, wnen, wnin, gn, gcc; //位置 上一次位置 速度 上一次速度 地球自转 牵连角速度 ，，重力加速度 慢变速度分量
	struct CVect3 pos, vn, wnie, wnen, wnin, gn, gcc; //位置 上一次位置 速度 上一次速度 地球自转 牵连角速度 ，，重力加速度 慢变速度分量
	struct CMAT3 mpv, dotwninT;//位置更新微分方程矩阵
};


struct CIMU  //IMU参数处理
{
	int nSamples;//子样数
	double cimu_t;//采样时间
	bool singlefirst;//单子样第一次计算
	struct CVect3 phim, dvbm, wm_1, vm_1, The1, The2, V1, V2;//等效旋转矢量、比力增量、前一时刻角增量、速度增量
	char rfu;//IMU指向
};

struct Calibration//标定参数
{
	struct CMAT3 Ka, Kg;
	struct CVect3 db, eb;
};

struct Sensordata //传感器数据
{
	double gyrodata[6];
	double accdata[6];
	int naviSampleCount;
};

struct CSINS  //捷联式惯导
{
	int x;//先验估计误差累计确定时间的计数
	double heading;//偏航角DD院
	double ts, nts, tk0, tk;//采样时间，更新时间，初始时间
	struct Quat qnb;  //姿态四元数
	struct CMAT3 Cnb, Cbn;//b->n变换矩阵，n->b变换矩阵
	struct CVect3 wib, fb, fn, att;//b系下角速度，b系下比力，n系下比力（构造状态转移矩阵用），姿态角
};

struct Kalman //12维对准
{
	struct CMat Fk, Qkf, Pk1, Pk, H, II;
	struct CMAT3 Maa, Mav, Mva, Mvv;
	struct CVect Xk, Zk, Q, Rkf, K, Pxrk;
	double nn;
	int nq, n, iter, nstep;
};

struct Kalman15 //12维对准
{
	struct CMat Fk, Qkf, Pk1, Pk, H, II;
	struct CMAT3 M1, Maa, Mav, Map, M2,M3, Mva, Mvv, Mvp, Mpv, Mpp;
	struct CVect Xk, Zk, Q, Rkf, K, Pxrk;
	double nn;
	int nq, n, iter, nstep;
};

struct GNSSDATA
{
 struct CVect3 GNSSpos; //位置
 struct CVect3 GNSSvn;  //速度
 struct CVect3 GNSSerr;  //误差
 struct CVect3 DOP;      //精度衰减因子
 bool GNSSvalidity;     //数据有效
	//bool dataflag;       //数据更新
};
struct ICA //惯性系粗对准
{
	double tk0, T2, T1;                     //对准时间       //对准双矢量时间选取
	struct CIMU cimu;
	struct CMAT3 Cne, Cei, Cii, Cib, Cnb;//相关姿态阵
	struct Quat qib, qnb;                       //四元数
	struct CVect3 att0, Vi1, Vi2, Vib1, Vib2;  //初始姿态角 i系下两速度矢量理论值，两矢量实测值
	bool alignfinish, V1flag;                      //粗对准结束 V1数据采集结束
};

struct STICA  //静基座解析式对准
{
	double t0, n;
	struct CVect3 G1, A1;
	struct CVect3 N1, W1, att0;
	struct CMAT3 Cnb;
	bool alignfinish;                      //粗对准结束 V1数据采集结束
};

struct PIICA // 参数辨识对准
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

//函数声明
void IniICA(struct ICA* ica, struct CVect3* pos0, double T, int s, double t);
void INSUpdate(struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, double* pwm, double* pvm);
void CoarAlign(struct ICA* ica, struct Calibration* Cal, double* pwm, double* pvm, double T);
void CIMUUpdate(struct CIMU* cimu, double* pwm, double* pvm);
void IniSTICA(struct STICA* ica, struct CVect3* pos);
void IniIns(struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct CVect3* att0, struct CVect3* vn0, struct CVect3* pos0, double tk0, double t, int n);
void CoarAlign0(struct STICA* ica, double* pwm, double* pvm, double  t, double T);
void IniKalman_D(struct Kalman* Kal, struct CVect3* pos, struct CVect3* Satt, struct CVect3* Svn, struct CVect3* Sgyro, struct CVect3* Sacc, struct CVect3* SWg, struct CVect3* SWa, double T);//姿态速度位置标准差、陀螺仪随机飘移，加表零偏，陀螺仪、加表随机游走 惯导更新时间
void Kal_TDupdate_D(struct Kalman* Kal, struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct Calibration* Cal, double* pwm, double* pvm);
void IniKalman_TD(struct Kalman15* Kal, struct CVect3* SWg, struct CVect3* SWa, struct CVect3* Stdatt, struct CVect3* Stdvn, struct CVect3* Stdpos, struct CVect3* Stdgyro, struct CVect3* Stdacc);
void Kal_INAV_UPdata(struct Kalman15* Kal, struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct GNSSDATA* GNSS,double* pwm, double* pvm, uint8_t* DataRevision);

struct CVect3* CV3protectmul1(struct CVect3* mul, struct CVect3* C, double a);
struct CVect3* operatormul3(struct CVect3* C1, struct CVect3* C2, struct CVect3* C);
struct CVect3* CV3add(struct CVect3* C1, struct CVect3* C2);
struct CVect3* CV3mul1(struct CVect3* C, double a);
struct CVect3* CV3protectadd(struct CVect3* C, struct CVect3* C1, struct CVect3* C2);


#endif
