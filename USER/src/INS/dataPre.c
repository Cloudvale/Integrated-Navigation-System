#include "dataPre.h"
#include "INS.h"

__calibrationPara calibrationPara;

//angle err        rad        

//scale factor     no units
//acc bias         g
//gyro bas         deg/s
//Ka Kg = inv(Ka) inv(Kg)





//1010标定参数001-3
//const double Ka11 = 1.00011670998236;
//const double Ka12 = -0.00102186365396967;
//const double Ka13 = -0.00179817203674419;
//const double Ka21 = 0.00930533520899094;
//const double Ka22 = 1.00033317750909;
//const double Ka23 = 0.00549491280202494;
//const double Ka31 = 0.000665748725208212;
//const double Ka32 = -0.0147816549502156;
//const double Ka33 = 1.00012622526644;


//const double db1 = 0.00281110417362657;
//const double db2 = -0.00411413162603780;
//const double db3 = -0.00331380314026669;//Accbias

//		
//		
//		
//const double Kg11 = 0.999735605725067;
//const double Kg12 = 0.00648537784123936;
//const double Kg13 = -0.00949350207485681;
//const double Kg21 = 0.00462696810729167;
//const double Kg22 = 0.999660934172751;
//const double Kg23 = -0.00453665859791050;
//const double Kg31 = -0.00143044146951526;
//const double Kg32 = -0.00515014329902745;
//const double Kg33 = 0.999511053509650;


//const double eb1 = 0.000222999;
//const double eb2 = 0.004214968;
//const double eb3 = 0.005354982;//gyrobias   ST

//const double eb1 = -0.002208539;
//const double eb2 = 0.006400707;
//const double eb3 = 0.007562651;//gyrobias  TR

////1009标定参数001-1
//const double Ka11 = 1.00078744660276;
//const double Ka12 = 0.00105561990262986;
//const double Ka13 = 0.0122061655607472;
//const double Ka21 = -0.00238615275177365;
//const double Ka22 = 1.00039435610904;
//const double Ka23 = 0.0206702043199928;
//const double Ka31 = -0.0150428486461152;
//const double Ka32 = -0.0149601404811995;
//const double Ka33 = 1.00015108402559;
//
//const double db1 = -0.0137352683139609;
//const double db2 = 0.0133388253820538;
//const double db3 = 0.0148255372238904;//Accbias
//
//const double Kg11 = 0.999685232768616;
//const double Kg12 = -0.00800074170987604;
//const double Kg13 = 0.0106934163157234;
//const double Kg21 = 0.0129859475819307;
//const double Kg22 = 1.00050030295136;
//const double Kg23 = 0.0158016647075474;
//const double Kg31 = -0.0197346854671154;
//const double Kg32 = -0.0155103198726123;
//const double Kg33 = 1.00080261492741;
//
//const double eb1 = -0.00745292657160785;
//const double eb2 = -0.000683109893512240;
//const double eb3 = 0.00368975339209650;//gyrobias

//0926标定数参数001-1
//const double Ka11 = 1.00112333429504;
//const double Ka12 = -4.60097504592455e-05;
//const double Ka13 = 0.0116494834827461;
//const double Ka21 = -0.00117968356690204;
//const double Ka22 = 1.00075862391766;
//const double Ka23 = 0.0208903319148215;
//const double Ka31 = -0.0147675767538990;
//const double Ka32 = -0.0150132409380054;
//const double Ka33 = 1.00076176594826;

//const double db1 = -0.0148259315302870;
//const double db2 = 0.0156881963200910;
//const double db3 = 0.0187248062888822;//Accbias

//const double Kg11 = 0.999159085610743;
//const double Kg12 = -0.00917687909582312;
//const double Kg13 = 0.0103217354563021;
//const double Kg21 = 0.0141761463025570;
//const double Kg22 = 1.00022574472365;
//const double Kg23 = 0.0158145908233200;
//const double Kg31 = -0.0193591460177037;
//const double Kg32 = -0.0155359737272307;
//const double Kg33 = 1.00044218996423;

//const double eb1 = -0.00119173;
//const double eb2 = -0.012473923;
//const double eb3 = -0.002916831;//gyrobias


//*****************标定***************************//
//标定参数赋值
//void getCalibration(struct Calibration* Cal)//标定参数获取
//{
//	Cal->Ka.e00 = Ka11;   Cal->Ka.e01 = Ka12;   Cal->Ka.e02 = Ka13;
//	Cal->Ka.e10 = Ka21;   Cal->Ka.e11 = Ka22;   Cal->Ka.e12 = Ka23;
//	Cal->Ka.e20 = Ka31;   Cal->Ka.e21 = Ka32;   Cal->Ka.e22 = Ka33;
//
//	Cal->Kg.e00 = Kg11;   Cal->Kg.e01 = Kg12;   Cal->Kg.e02 = Kg13;
//	Cal->Kg.e10 = Kg21;   Cal->Kg.e11 = Kg22;   Cal->Kg.e12 = Kg23;
//	Cal->Kg.e20 = Kg31;   Cal->Kg.e21 = Kg32;   Cal->Kg.e22 = Kg33;
//
//	Cal->db.i = db1;
//	Cal->db.j = db2;
//	Cal->db.k = db3;
//
//	Cal->eb.i = eb1;
//	Cal->eb.j = eb2;
//	Cal->eb.k = eb3;
//}

//void Sensordataassign(double gyrodata[6], double accdata[6], double pwm[3], double pvm[3], int nSamples, int* naviSampleCount)
void Sensordataassign(struct Sensordata* Sen, double pwm[3], double pvm[3], int nSamples)//导航传感器数据获取
{
	if (nSamples == 1) {
		Sen->gyrodata[0] = pwm[0];
		Sen->gyrodata[1] = pwm[1];
		Sen->gyrodata[2] = pwm[2];

		Sen->accdata[0] = pvm[0];
		Sen->accdata[1] = pvm[1];
		Sen->accdata[2] = pvm[2];
	}
	else {
		if (Sen->naviSampleCount == 0) {
			Sen->gyrodata[0] = pwm[0];
			Sen->gyrodata[1] = pwm[1];
			Sen->gyrodata[2] = pwm[2];

			Sen->accdata[0] = pvm[0];
			Sen->accdata[1] = pvm[1];
			Sen->accdata[2] = pvm[2];
			Sen->naviSampleCount++;
		}
		else {
			Sen->gyrodata[3] = pwm[0];
			Sen->gyrodata[4] = pwm[1];
			Sen->gyrodata[5] = pwm[2];

			Sen->accdata[3] = pvm[0];
			Sen->accdata[4] = pvm[1];
			Sen->accdata[5] = pvm[2];
			Sen->naviSampleCount++;
		}
	}
}

void Calibrate(struct Calibration* Cal, double* pwm, double* pvm, double gyro_x_data, double gyro_y_data, double gyro_z_data, double accel_x_data, double accel_y_data, double accel_z_data, double g)//静态误差补偿
{
	struct CVect3 f = { 0,0,0 };
	struct CVect3 w = { 0,0,0 };
	f.i = accel_x_data - Cal->db.i;
	f.j = accel_y_data - Cal->db.j;
	f.k = accel_z_data - Cal->db.k;
	w.i = gyro_x_data - Cal->eb.i;
	w.j = gyro_y_data - Cal->eb.j;
	w.k = gyro_z_data - Cal->eb.k;
	*(pvm) = Cal->Ka.e00 * f.i + Cal->Ka.e01 * f.j + Cal->Ka.e02 * f.k;
	*(pvm + 1) = Cal->Ka.e10 * f.i + Cal->Ka.e11 * f.j + Cal->Ka.e12 * f.k;
	*(pvm + 2) = Cal->Ka.e20 * f.i + Cal->Ka.e21 * f.j + Cal->Ka.e22 * f.k;

	*(pwm) = Cal->Kg.e00 * w.i + Cal->Kg.e01 * w.j + Cal->Kg.e02 * w.k;
	*(pwm + 1) = Cal->Kg.e10 * w.i + Cal->Kg.e11 * w.j + Cal->Kg.e12 * w.k;
	*(pwm + 2) = Cal->Kg.e20 * w.i + Cal->Kg.e21 * w.j + Cal->Kg.e22 * w.k;
	*pvm = *pvm * g; *pwm = *pwm * d2r;
	*(pvm + 1) = *(pvm + 1) * g; *(pwm + 1) = *(pwm + 1) * d2r;
	*(pvm + 2) = *(pvm + 2) * g; *(pwm + 2) = *(pwm + 2) * d2r;
}

//*********************惯性测量单元数据处理*********************//
void IniCIMU(struct CIMU* cimu, double t, int s)//惯性单元参数初始化
{
	cimu->nSamples = s;//字样数
	cimu->cimu_t = t;  //采样时间
	cimu->singlefirst = true; //单子样法第一次计算
	cimu->phim = cimu->dvbm = cimu->wm_1 = cimu->vm_1 = cimu->The1 = cimu->The2 = cimu->V1 = cimu->V2 = O31;//补偿后的等效旋转矢量、速度增量、保存前一次的角/速度增量
	//	cimu->rfu = 'ENU';
};

void CIMUUpdate(struct CIMU* cimu, double* pwm, double* pvm)//角度采样速度采样时间间隔pwm、pvm为n*3数组,ts子样单位时间//单双子样方法,
{

	// 速率转换增量
#ifdef GYRO_OUTPUT_ANGULAR_RATE_single
	for (int n = 0; n < 3; n++)
	{
		*(pwm + n) = *(pwm + n) * cimu->cimu_t;
		*(pvm + n) = *(pvm + n) * cimu->cimu_t;
	}
#endif
#ifdef GYRO_OUTPUT_ANGULAR_RATE_double
	for (int n = 0; n < 6; n++)
	{
		*(pwm + n) = *(pwm + n) * cimu->cimu_t;
		*(pvm + n) = *(pvm + n) * cimu->cimu_t;
	}
#endif
	//坐标系转换
	//若载体坐标系不是右前上，根据结构体中的指向char数组，将传感器数据转换为右前上坐标系。
	//待添加
	if (cimu->nSamples == 1)    //单子样算法
	{
		if (cimu->singlefirst == true)
		{
			cimu->wm_1.i = pwm[0]; cimu->wm_1.j = *(pwm + 1); cimu->wm_1.k = *(pwm + 2);
			cimu->vm_1.i = pvm[0]; cimu->vm_1.j = *(pvm + 1); cimu->vm_1.k = *(pvm + 2);//第一次计算赋予前一时刻矢量初值
			cimu->singlefirst = false;  //单子样第一次计算标志改变
		}
		struct CVect3 cm = O31; struct CVect3 sm = O31;
		struct CVect3 wmm = O31; struct CVect3 vmm = O31;
		cm = *CV3protectmul1(&cm, &cimu->wm_1, 0.0833333333333333);
		sm = *CV3protectmul1(&sm, &cimu->vm_1, 0.0833333333333333);//上时刻传感器数据乘1/12
		wmm.i = *pwm; wmm.j = *(pwm + 1); wmm.k = *(pwm + 2);
		vmm.i = *pvm; vmm.j = *(pvm + 1); vmm.k = *(pvm + 2);//当前时刻传感器数据
		//等效旋转矢量计算
		struct CVect3 Cm = cm; //防止数据篡改
		cimu->phim = *operatormul3(&Cm, &wmm, &cimu->phim);//cm×wmm
		cimu->phim = *CV3add(&cimu->phim, &wmm);           //wmm+cm×wmm
		//快变速度增量计算
		struct CVect3 Wmm = wmm; //防止数据篡改
		cimu->dvbm = *operatormul3(&Wmm, &vmm, &cimu->dvbm);//wmm×vmm
		cimu->dvbm = *CV3mul1(&cimu->dvbm, 0.5);           //0.5*wmm×vmm旋转效应补偿
		cm = *operatormul3(&cm, &vmm, &cm);//cm×vmm
		sm = *operatormul3(&sm, &wmm, &sm);//sm×wmm
		cimu->dvbm = *CV3add(&cimu->dvbm, &cm);
		cimu->dvbm = *CV3add(&cimu->dvbm, &sm);            //划桨误差补偿
		cimu->dvbm = *CV3add(&cimu->dvbm, &vmm);           //快变速度增量
		//更新前一时刻矢量
		cimu->wm_1.i = wmm.i; cimu->wm_1.j = wmm.j; cimu->wm_1.k = wmm.k;
		cimu->vm_1.i = vmm.i; cimu->vm_1.j = vmm.j; cimu->vm_1.k = vmm.k;//第一次计算赋予前一时刻矢量初值

	}
	else   //双子样算法
	{
		cimu->The1.i = *(pwm + 3 * (cimu->nSamples - 2));
		cimu->The1.j = *(pwm + 3 * (cimu->nSamples - 2) + 1);
		cimu->The1.k = *(pwm + 3 * (cimu->nSamples - 2) + 2);
		cimu->The2.i = *(pwm + 3 * (cimu->nSamples - 1));
		cimu->The2.j = *(pwm + 3 * (cimu->nSamples - 1) + 1);
		cimu->The2.k = *(pwm + 3 * (cimu->nSamples - 1) + 2);
		cimu->V1.i = *(pvm + 3 * (cimu->nSamples - 2));
		cimu->V1.j = *(pvm + 3 * (cimu->nSamples - 2) + 1);
		cimu->V1.k = *(pvm + 3 * (cimu->nSamples - 2) + 2);
		cimu->V2.i = *(pvm + 3 * (cimu->nSamples - 1));
		cimu->V2.j = *(pvm + 3 * (cimu->nSamples - 1) + 1);
		cimu->V2.k = *(pvm + 3 * (cimu->nSamples - 1) + 2);
		//等效旋转矢量计算
		struct CVect3 The = O31;
		The = *operatormul3(&cimu->The1, &cimu->The2, &The);
		The = *CV3mul1(&The, 0.6666666666666667);//圆锥误差补偿
		cimu->phim = *CV3protectadd(&cimu->phim, &The, &cimu->The1);
		cimu->phim = *CV3add(&cimu->phim, &cimu->The2);//等效旋转矢量
		//速度增量计算
		struct CVect3 cross1 = O31; struct CVect3 cross2 = O31;
		cross1 = *operatormul3(&cimu->The1, &cimu->V2, &cross1);
		cross2 = *operatormul3(&cimu->V1, &cimu->The2, &cross2);
		cross1 = *CV3add(&cross1, &cross2);
		cross1 = *CV3mul1(&cross1, 0.6666666666666667);   //划桨误差
		struct CVect3 sumT = O31; struct CVect3 sumV = O31;
		sumT = *CV3protectadd(&sumT, &cimu->The1, &cimu->The2);
		sumV = *CV3protectadd(&sumV, &cimu->V1, &cimu->V2);
		sumT = *operatormul3(&sumT, &sumV, &sumT);
		sumT = *CV3mul1(&sumT, 0.5);//旋转误差
		cimu->dvbm = *CV3protectadd(&cimu->dvbm, &cross1, &sumT);
		cimu->dvbm = *CV3add(&cimu->dvbm, &cimu->V1);
		cimu->dvbm = *CV3add(&cimu->dvbm, &cimu->V2);//快变增量
	}
}

