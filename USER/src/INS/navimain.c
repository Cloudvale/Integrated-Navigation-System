#include "dataPre.h"
#include "INS.h"
#include "navimain.h"


struct STICA ica;
struct CSINS ins;
struct CEarth eth;
struct CIMU cimu;
struct Kalman15 Kal;
struct GNSSDATA GNSS;
struct Result Re1;

struct CVect3 pos;
struct CVect3 vn;
struct CVect3 SWg = { 2.90900000000000e-05 ,2.90900000000000e-05 ,2.90900000000000e-05 };
struct CVect3 SWa = { 0.000555555555555556 ,0.000555555555555556 ,0.000555555555555556 };
struct CVect3 Stdatt = { 0.0872664625997165,0.0872664625997165, 0.0872664625997165 };
struct CVect3 Stdvn = {3, 3, 3};
struct CVect3 Stdpos = { 8.07963286148278e-07,8.07963286148278e-07 , 5 };
struct CVect3 Stdgyro = { 2.42406840554768e-06,2.42406840554768e-06,2.42406840554768e-06 };
struct CVect3 Stdacc = { 0.00146704875000000,0.00146704875000000,0.00146704875000000 };

struct Sensordata sen;   //����������
//struct Calibration Cal;//�궨����

double Pwm[3];
double Pvm[3];//��̬����������

void GNSSupdate(struct GNSSDATA* GNSS,double Lat,double Long, double elv)
{
	GNSS->GNSSpos.i = Lat*d2r;
	GNSS->GNSSpos.j = Long*d2r;
	GNSS->GNSSpos.k = elv;

	//Ԥ��
	GNSS->GNSSerr.i = 0.5;
	GNSS->GNSSerr.j = 0.5;
	GNSS->GNSSerr.k = 1;
	GNSS->DOP.i = 1.61592657229656e-07;
	GNSS->DOP.j = 1.61592657229656e-07;
	GNSS->DOP.k = 1;
}

void Ininavimain(double Lati,double Longi,double High)//γ�Ⱦ��ȸ߶�
{
	sen.naviSampleCount = 0;//������������ʼ��
	pos.i = Lati * d2r;
	pos.j = Longi * d2r;
	pos.k = High;//λ�ó�ʼ��
	vn.i = 0;
	vn.j = 0;
	vn.k = 0;//�ٶȳ�ʼ��
	IniSTICA(&ica, &pos);//����ʽ��׼��ʼ��
	//getCalibration(&Cal);//�궨������ȡ
	IniKalman_TD(&Kal, &SWg, &SWa, &Stdatt, &Stdvn, &Stdpos, &Stdgyro, &Stdacc);
}

void navimain(double gyro_x_data, double gyro_y_data, double gyro_z_data, double accel_x_data, double accel_y_data, double accel_z_data, double Lat, double Long, double elv, uint8_t* DataRevision)//deg/s   m/s^2   
{
	////���ݾ�̬����
	//double g = g0 * (1.0 + b * sin(pos.i) * sin(pos.i) - b1 * sin(2 * pos.i) * sin(2 * pos.i)) - b2 * pos.k;
	//Calibrate(&Cal, Pwm, Pvm, gyro_x_data, gyro_y_data, gyro_z_data, accel_x_data, accel_y_data, accel_z_data, g);
	Pwm[0] = gyro_x_data * d2r; Pvm[0] = accel_x_data;
	Pwm[1] = gyro_y_data * d2r; Pvm[1] = accel_y_data;
	Pwm[2] = gyro_z_data * d2r; Pvm[2] = accel_z_data;

	if (ica.alignfinish == false) {

		sen.gyrodata[0] = Pwm[0];
		sen.gyrodata[1] = Pwm[1];
		sen.gyrodata[2] = Pwm[2];

		sen.accdata[0] = Pvm[0];
		sen.accdata[1] = Pvm[1];
		sen.accdata[2] = Pvm[2];

		CoarAlign0(&ica, sen.gyrodata, sen.accdata, 0.005, 5);
		if (ica.alignfinish == true)
		{
			ica.att0.k = 0;
			IniIns(&ins, &eth, &cimu, &ica.att0, &vn, &pos, 0.0, 0.005, 2);//�ߵ��ṹ�� ����ṹ�� ��̬�����ṹ�� ���� �ٶ� λ�� ��ʼʱ�� ����ʱ�� ������
		}
	}
	else
	{
		Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
		if (sen.naviSampleCount >= 2) {
			
//			//if (GNSS.dataflag == true)
//			//{
//				GNSSupdate(&GNSS, Lat, Long, elv);
//			//}
			GNSSupdate(&GNSS, Lat, Long, elv);
			Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
			sen.naviSampleCount = 0;
			//�������
			Re1.Lati  = eth.pos.i*r2d;
			Re1.Longi = eth.pos.j*r2d;
			Re1.High  = (float)eth.pos.k;
			Re1.VE    = (float)eth.vn.i;
			Re1.VN    = (float)eth.vn.j;
			Re1.VU    = (float)eth.vn.k;
			Re1.pitch = (float)ins.att.i*r2d;
			Re1.roll  = (float)ins.att.j*r2d;
			Re1.yaw   = (float)ins.att.k*r2d;
			Re1.fx    = (float)ins.fb.i;
			Re1.fy    = (float)ins.fb.j;
			Re1.fz    = (float)ins.fb.k;
			Re1.gx    = (float)ins.wib.i*r2d;
			Re1.gy    = (float)ins.wib.j*r2d;
			Re1.gz    = (float)ins.wib.k*r2d;

		}
	}
}

//int main()
//{

//	
//	//��ʼ��
//	double Lati = 39.8184192000000;
//	double Longi = 116.292989800000;
//	double High = 46.0600000000000;
//	bool DataRevision;
//	Ininavimain(Lati, Longi, High);
//	ica.att0.i = -2.286 * d2r;
//	ica.att0.j = 0.666 * d2r;
//	ica.att0.k = 89.154 * d2r;
//	IniIns(&ins, &eth, &cimu, &ica.att0, &vn, &pos, 0.0, 0.005, 1);
//		 
//	//����������//�������ݸ���01
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	
//	//����������//�������ݸ���02
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���03
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���04
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;
//	

//	//����������//�������ݸ���05
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;
//	

//	//����������//�������ݸ���06
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���07
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���08
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���09
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���10
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���11
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;


//	//����������//�������ݸ���12
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;


//	//����������//�������ݸ���13
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���14
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���15
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;


//	//����������//�������ݸ���16
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���17
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���18
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���19
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	//����������//�������ݸ���20
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;


//	//����������//�������ݸ���21
//	Pwm[0] = -0.00999850000000000 * d2r; Pvm[0] = -0.0122980000000000;
//	Pwm[1] = 0.0320230000000000 * d2r; Pvm[1] = -0.548215000000000;
//	Pwm[2] = -0.00195300000000000 * d2r; Pvm[2] = 9.78999700000000;
//	Lati = 39.8184192000000;
//	Longi = 116.292989800000;
//	High = 46.0600000000000;
//	DataRevision = true;
//	//����
//	Sensordataassign(&sen, Pwm, Pvm, cimu.nSamples);
//	GNSSupdate(&GNSS, Lati, Longi, High);
//	Kal_INAV_UPdata(&Kal, &ins, &eth, &cimu, &GNSS, sen.gyrodata, sen.accdata, DataRevision);
//	sen.naviSampleCount = 0;

//	return 0;
//}