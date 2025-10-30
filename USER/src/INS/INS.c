#include "INS.h"
#include "dataPre.h"


//****************全局变量*************//
const struct CVect3 I31 = { 1, 1, 1 };
const struct CVect3 O31 = { 0,0,0 };
const struct CVect3 Ipos = { 1,1,1 };//三维1向量、零向量、单位位置向量
const struct Quat qI = { 1,0,0,0 };              //单位四元数
const struct CMAT3 I33 = { 1,0,0,0,1,0,0,0,1 };
const struct CMAT3 O33 = { 0,0,0,0,0,0,0,0,0 };       //三维单位矩阵、三维零矩阵

//全局变量定义
const double wie = 7.292115e-5;
const double g0 = 9.780325;
const double f = 0.003353;
const double Re = 6378136.5;
const double Rp = 6356755.0;
const double e2 = 0.006694757391;
const double e = 0.08182145;
const double ppm = 1.0e-6;
const int hur = 3600;
const int shur = 60;
const double b = 1 / 188.6;
const double b1 = 5.84e-06;
const double b2 = 3.08e-6;
const double b3 = 8.08e-9;
const double PI = 3.141592657539;             //圆周率
const double PI_2 = 1.5707963287695;
const double PI_4 = 0.78539816438475;
const double _2PI = 6.283185315078;
//const double sqrt2 = 1.414213562373095;	// sqrt(2) ...
//const double sqrt3 = 1.732050807568877;
//const double sqrt5 = 2.236067977499790;
//const double sqrt6 = 2.449489742783178;
//const double sqrt7 = 2.645751311064591;
//const double sqrt8 = 2.828427124746190;
const double INF = 3.402823466e+30;     //无穷大
const double EPS = 2.22044604925031e-16;//零阈值
const double d2r = 0.01745329;
const double r2d = 57.2957794;
const double dh2rs = 0.00000484813673;




//****************功能辅助函数***********//
double Range(double val, double minVal, double maxVal)//确保三角函数输入在范围内
{
	double res;
	if (val < minVal)
	{
		res = minVal;
	}
	else if (val > maxVal)
	{
		res = maxVal;
	}
	else
	{
		res = val;
	}
	return res;
}

double Atan2Ex(double y, double x)//计算反正切值
{
	double res;
	if ((x == 0) && (y == 0))//sign(x):x<0,output -1;x>0,output 1;x=0 output 0
	{
		res = 0.0;
	}
	else
	{
		res = atan2(y, x);//计算反正切
	}
	return res;
}


//****************三维向量CVect3相关运算函数***********//
struct CVect3* Initialize(struct CVect3* CC, double xyz)//初始化相同值
{
	CC->i = CC->j = CC->k = xyz;
	return CC;
}

struct CVect3* assignD(struct CVect3* CC, double xx, double yy, double zz)//赋不同值
{
	CC->i = xx;
	CC->j = yy;
	CC->k = zz;
	return CC;
}

struct CVect3* InitializeP(struct CVect3* CC, double* pdata)//数组指针初始化///////
{
	CC->i = *pdata++, CC->j = *pdata++, CC->k = *pdata;
	return CC;
}

struct CVect3* CV3add(struct CVect3* C1, struct CVect3* C2)//三维向量加法
{
	C1->i += C2->i;
	C1->j += C2->j;
	C1->k += C2->k;
	return C1;
}

struct CVect3* CV3protectadd(struct CVect3* C, struct CVect3* C1, struct CVect3* C2)//三维向量加法
{
	C->i = C1->i + C2->i;
	C->j = C1->j + C2->j;
	C->k = C1->k + C2->k;
	return C;
}

struct CVect3* CV3minus(struct CVect3* C1, struct CVect3* C2)//三维向量减法
{
	C1->i -= C2->i;
	C1->j -= C2->j;
	C1->k -= C2->k;
	return C1;
}

struct CVect3* CV3protectminus(struct CVect3* C, struct CVect3* C1, struct CVect3* C2)//三维向量减法
{
	C->i = C1->i - C2->i;
	C->j = C1->j - C2->j;
	C->k = C1->k - C2->k;
	return C;
}

struct CVect3* CV3mul1(struct CVect3* C, double a)//三维向量乘标量
{
	C->i = C->i * a;
	C->j = C->j * a;
	C->k = C->k * a;
	return C;
}

struct CVect3* CV3protectmul1(struct CVect3* mul, struct CVect3* C, double a)//三维向量乘标量
{
	mul->i = C->i * a;
	mul->j = C->j * a;
	mul->k = C->k * a;
	return mul;
}
/*
struct CVect3* CV3mul2(struct CVect3* C1, struct CMAT3* M)//三维行向量点乘三维矩阵
{
	C1->i = C1->i * M->e00 + C1->j * M->e10 + C1->k * M->e20;
	C1->j = C1->i * M->e01 + C1->j * M->e11 + C1->k * M->e21;
	C1->k = C1->i * M->e02 + C1->j * M->e12 + C1->k * M->e22;
	return C1;
}
*/

struct CVect3* operatormul3(struct CVect3* C1, struct CVect3* C2, struct CVect3* C)//三维向量叉乘  mul=C1*C2
{

	double i, j, k;
	i = C1->j * C2->k - C2->j * C1->k;
	j = C1->k * C2->i - C2->k * C1->i;
	k = C1->i * C2->j - C2->i * C1->j;
	C->i = i; C->j = j; C->k = k;
	return C;
}

struct CVect3* operatordiv(struct CVect3* CC, double a)//三维向量除标量
{
	CC->i = CC->i / a;
	CC->j = CC->j / a;
	CC->k = CC->k / a;
	return CC;
}

struct CVect3* operatorDiv(struct CVect3* C1, struct CVect3* C2)//三维向量对应元素相除
{
	C1->i = C1->i / C2->i;
	C1->j = C1->j / C2->j;
	C1->k = C1->k / C2->k;
	return C1;
}

struct CVect3* operatorNega(struct CVect3* C1)//向量取反
{
	C1->i = -1 * C1->i;
	C1->j = -1 * C1->j;
	C1->k = -1 * C1->k;
	return C1;
}

double norm(struct CVect3* C)//求模运算
{
	return sqrt(C->i * C->i + C->j * C->j + C->k * C->k);
}

double normXY(struct CVect3* C)//XY求模
{
	return sqrt(C->i * C->i + C->j * C->j);
}

struct CVect3* sqr(struct CVect3* C)//每一元素开方
{
	C->i = sqrt(C->i);
	C->j = sqrt(C->j);
	C->k = sqrt(C->k);
	return C;
}

struct CVect3* Pow(struct CVect3* C, struct CVect3* C1, int t)//每一分量求t次方
{
	C1->i = C1->j = C1->k = 1;
	for (int i = 0; i < t; i++)
	{
		C1->i = C1->i * C->i;
		C1->j = C1->j * C->j;
		C1->k = C1->k * C->k;
	}
	return C1;
}

double dot(struct CVect3* C1, struct CVect3* C2, double a)//两向量点乘
{
	a = (C1->i * C2->i + C1->j * C2->j + C1->k * C2->k);
	return a;
}

//****************三维矩阵CMAT3相关运算***********//

struct CMAT3* CM3Initialize(double xyz, struct CMAT3* C)//矩阵初始化/赋相同值
{
	C->e00 = C->e01 = C->e02 = C->e10 = C->e11 = C->e12 = C->e20 = C->e21 = C->e22 = xyz;
	return C;
};

struct CMAT3* CM3InitializeD(struct CMAT3* C, double xx, double xy, double xz, double yx, double yy, double yz, double zx, double zy, double zz)//矩阵赋不同值
{
	C->e00 = xx; C->e01 = xy; C->e02 = xz;
	C->e10 = yx; C->e11 = yy; C->e12 = yz;
	C->e20 = zx; C->e21 = zy; C->e22 = zz;
	return C;
};

struct CMAT3* CM3InitializeDD(double xx, double yy, double zz, struct CMAT3* C)//对角线元素赋值，其他元素为0
{
	C->e00 = xx; C->e01 = 0; C->e02 = 0;
	C->e10 = 0; C->e11 = yy; C->e12 = 0;
	C->e20 = 0; C->e21 = 0; C->e22 = zz;
	return C;
}

struct CMAT3* operatorCM3add(struct CMAT3* C1, struct CMAT3* C2)//矩阵加法
{
	C1->e00 += C2->e00; C1->e01 += C2->e01; C1->e02 += C2->e02;
	C1->e10 += C2->e10; C1->e11 += C2->e11; C1->e12 += C2->e12;
	C1->e20 += C2->e20; C1->e21 += C2->e21; C1->e22 += C2->e22;
	return C1;
}

struct CMAT3* operatorCM3minus(struct CMAT3* C1, struct CMAT3* C2)//矩阵减法
{
	C1->e00 -= C2->e00; C1->e01 -= C2->e01; C1->e02 -= C2->e02;
	C1->e10 -= C2->e10; C1->e11 -= C2->e11; C1->e12 -= C2->e12;
	C1->e20 -= C2->e20; C1->e21 -= C2->e21; C1->e22 -= C2->e22;
	return C1;
}

struct CMAT3* operatorCM3mul(struct CMAT3* C1, struct CMAT3* C2, struct CMAT3* M)//矩阵相乘
{
	struct CMAT3 MM;
	MM.e00 = C1->e00 * C2->e00 + C1->e01 * C2->e10 + C1->e02 * C2->e20;
	MM.e01 = C1->e00 * C2->e01 + C1->e01 * C2->e11 + C1->e02 * C2->e21;
	MM.e02 = C1->e00 * C2->e02 + C1->e01 * C2->e12 + C1->e02 * C2->e22;
	MM.e10 = C1->e10 * C2->e00 + C1->e11 * C2->e10 + C1->e12 * C2->e20;
	MM.e11 = C1->e10 * C2->e01 + C1->e11 * C2->e11 + C1->e12 * C2->e21;
	MM.e12 = C1->e10 * C2->e02 + C1->e11 * C2->e12 + C1->e12 * C2->e22;
	MM.e20 = C1->e20 * C2->e00 + C1->e21 * C2->e10 + C1->e22 * C2->e20;
	MM.e21 = C1->e20 * C2->e01 + C1->e21 * C2->e11 + C1->e22 * C2->e21;
	MM.e22 = C1->e20 * C2->e02 + C1->e21 * C2->e12 + C1->e22 * C2->e22;
	M->e00 = MM.e00; M->e01 = MM.e01; M->e02 = MM.e02;
	M->e10 = MM.e10; M->e11 = MM.e11; M->e12 = MM.e12;
	M->e20 = MM.e20; M->e21 = MM.e21; M->e22 = MM.e22;
	return M;
}

struct CMAT3* operatorCM3mulf(struct CMAT3* C, double a)//矩阵乘标量
{
	C->e00 = C->e00 * a; C->e01 = C->e01 * a; C->e02 = C->e02 * a;
	C->e10 = C->e10 * a; C->e11 = C->e11 * a; C->e12 = C->e12 * a;
	C->e20 = C->e20 * a; C->e21 = C->e21 * a; C->e22 = C->e22 * a;
	return C;
}

struct CVect3* CM3mulCV3(struct CMAT3* M1, struct CVect3* C1, struct CVect3* C)//矩阵乘三维矢量
{
	struct CVect3 CC;
	CC.i = M1->e00 * C1->i + M1->e01 * C1->j + M1->e02 * C1->k;
	CC.j = M1->e10 * C1->i + M1->e11 * C1->j + M1->e12 * C1->k;
	CC.k = M1->e20 * C1->i + M1->e21 * C1->j + M1->e22 * C1->k;
	C->i = CC.i; C->j = CC.j; C->k = CC.k;
	return C;
}

struct CMAT3* operatorCM3Nega(struct CMAT3* M1, struct CMAT3* M)//矩阵取反
{
	M->e00 = -M1->e00; M->e01 = -M1->e01; M->e02 = -M1->e02;
	M->e10 = -M1->e10; M->e11 = -M1->e11; M->e12 = -M1->e12;
	M->e20 = -M1->e20; M->e21 = -M1->e21; M->e22 = -M1->e22;
	return M;
}

struct CMAT3* operatorCM3tran(struct CMAT3* M1, struct CMAT3* M)//矩阵转置
{
	M->e00 = M1->e00; M->e01 = M1->e10; M->e02 = M1->e20;
	M->e10 = M1->e01; M->e11 = M1->e11; M->e12 = M1->e21;
	M->e20 = M1->e02; M->e21 = M1->e12; M->e22 = M1->e22;
	return M;
}

struct CMAT3* operatorCM3adj(struct CMAT3* M1, struct CMAT3* M)//求伴随矩阵
{
	M->e00 = (M1->e11 * M1->e22 - M1->e12 * M1->e21);
	M->e10 = -(M1->e10 * M1->e22 - M1->e12 * M1->e20);
	M->e20 = (M1->e10 * M1->e21 - M1->e11 * M1->e20);
	M->e01 = -(M1->e01 * M1->e22 - M1->e02 * M1->e21);
	M->e11 = (M1->e00 * M1->e22 - M1->e02 * M1->e20);
	M->e21 = -(M1->e00 * M1->e21 - M1->e01 * M1->e20);
	M->e02 = (M1->e01 * M1->e12 - M1->e02 * M1->e11);
	M->e12 = -(M1->e00 * M1->e12 - M1->e02 * M1->e10);
	M->e22 = (M1->e00 * M1->e11 - M1->e01 * M1->e10);
	return M;
}

double CM3det(struct CMAT3* M1, double a)//计算三维矩阵行列式
{
	a = M1->e00 * (M1->e11 * M1->e22 - M1->e12 * M1->e21) - M1->e01 * (M1->e10 * M1->e22 - M1->e12 * M1->e20) + M1->e02 * (M1->e10 * M1->e21 - M1->e11 * M1->e20);
	return a;
}

double* CM3trace(struct CMAT3* M1, double* a)//矩阵求迹
{
	*a = (M1->e00 + M1->e11 + M1->e22);
	return a;

}

struct CMAT3* CM3inv(struct CMAT3* M1, struct CMAT3* M)//矩阵求逆
{
	double a = 0;
	a = CM3det(M1, a);// M1的行列式数值
	if (a == 0)
	{
		printf("矩阵行列式为0不可逆");
	}
	double A = a;//行列式倒数
	A = 1 / A;
	struct CMAT3 M2;
	M2 = *operatorCM3adj(M1, &M2);//求伴随矩阵
	M = operatorCM3mulf(&M2, A);//求逆矩阵
	return M;
}

struct CVect3* diagCV3(struct CMAT3* M, struct CVect3* diag)//提取矩阵对角线元素生成三维向量
{
	diag->i = M->e00; diag->j = M->e11; diag->k = M->e22;
	return diag;
}

struct CMAT3* diagCM3(struct CVect3* C, struct CMAT3* diag)//三维向量构造对角矩阵
{
	double A = 0;
	diag = CM3Initialize(A, diag);
	diag->e00 = C->i; diag->e11 = C->j; diag->e22 = C->k;
	return diag;
}

struct CMAT3* askew(struct CVect3* CV, struct CMAT3* M)//三维向量反对称矩阵(A叉)
{
	double A = 0;
	M = CM3InitializeD(M, A, -CV->k, CV->j, CV->k, A, -CV->i, -CV->j, CV->i, A);
	return M;
}


//****************多维向量CVect相关运算***********//
struct CVect createCV(int row0, int clm0)//构造多维向量，1行n列或者n行1列
{
	struct CVect CV = { 0 };
	if (clm0 != 1 && row0 != 1)
	{
		printf("多维向量不合法");
	}
	else
		if (row0 > MMD && clm0 > MMD)
		{
			printf("多维向量长度超过阈值MMD");
		}
		else
		{
			CV.row = row0; CV.clm = clm0;
			CV.rc = CV.row * CV.clm;
		}
	return CV;
}

void SetCV(double a, struct CVect* CV1)//赋值相同值a
{
	for (int i = 0; i < CV1->rc; i++)
	{
		CV1->dd[i] = a;
	}
}

struct CVect* SetCVp(struct CVect* CV1, const double* pf)//使用数组赋值
{
	memcpy(CV1->dd, pf, CV1->rc * sizeof(double));
	return CV1;
}

/*struct CVect* SetCVD(struct CVect* CV1, double a, double a1, ...)//向量每一元素赋值
{
//	psinsassert(CV1->row <= MMD && CV1->clm <= MMD);
	va_list vl;
	va_start(vl, a);
	for (int i = 0, rc = CV1->row > CV1->clm ? CV1->row : CV1->clm; i < rc; i++)
	{
		if (a > 2 * INF) break;
		CV1->dd[i] = a;
		a = va_arg(vl, double);
	}
	va_end(vl);
	return CV1;
}*/

struct CVect* CVectadd(struct CVect* CV1, struct CVect* CV2, struct CVect* CV)//向量相加
{
	//	psinsassert(CV1->row == CV2->row && CV1->clm == CV2->clm);
	for (int i = 0; i < CV1->rc; i++)
	{
		CV->dd[i] = CV1->dd[i] + CV2->dd[i];
	}
	return CV;
}

struct CVect* CVectminus(struct CVect* CV1, struct CVect* CV2, struct CVect* CV)//向量相减
{
	//	psinsassert(CV1->row == CV2->row && CV1->clm == CV2->clm);
	for (int i = 0; i < CV1->rc; i++)
	{
		CV->dd[i] = CV1->dd[i] - CV2->dd[i];
	}
	return CV;
}

struct CVect* CVectmulscalar(struct CVect* CV1, double* sca)//多维向量乘标量/3*4ee3
{
	for (int i = 0, a = CV1->row > CV1->clm ? CV1->row : CV1->clm; i < a; i++)
	{
		CV1->dd[i] = *sca * CV1->dd[i];
	}
	return CV1;
}

struct CVect* CVectmulM(struct CVect* CV1, struct CMat* M, struct CVect* CV)//多维行向量乘矩阵
{
	//	psinsassert(CV1->clm == M->row);
	for (int i = 0; i < M->row; i++)
	{
		for (int j = 0; j < M->clm; j++)
		{
			CV->dd[j] += CV1->dd[i] * M->dd[j + M->clm * i];
		}
	}
	return CV;
}

struct CMat* CVectmulCV(struct CVect* CV1, struct CVect* CV2, struct CMat* M)//行向量乘列向量输出矩阵
{
	//	psinsassert(CV1->clm == CV2->row && CV1->row != 1);
	for (int i = 0; i < CV1->row; i++)
	{
		for (int j = 0; j < CV2->clm; j++)
		{
			M->dd[i * CV2->clm + j] = CV1->dd[i] * CV2->dd[j];
		}

	}
	return M;
}

double CVmulCVCon(struct CVect* CV1, struct CVect* CV2)//向量相乘输出double
{
	//	psinsassert(CV1->clm == CV2->row && CV1->row = CV2->clm = 1);
	double A = 0;
	for (int i = 0; i < CV1->clm; i++)
	{
		A += CV2->dd[i] * CV1->dd[i];
	}
	return A;
}

struct CVect* CVecttran(struct CVect* CV1)//转置
{
	int a = CV1->clm; CV1->clm = CV1->row; CV1->row = a;
	return CV1;
}

struct CVect* CVectdotmul(struct CVect* CV1, struct CVect* CV2)//对应元素相乘
{
	//	psinsassert(((CV1->clm > CV1->row) ? CV1->clm : CV1->row) == ((CV2->clm > CV2->row) ? CV2->clm : CV2->row));
	for (int i = 0; i < ((CV1->clm > CV1->row) ? CV1->clm : CV1->row); i++)
	{
		CV1->dd[i] = CV2->dd[i] * CV1->dd[i];
	}
	return CV1;
}

double operator(struct CVect* C1, int t)//提取第t个元素
{
	double A = C1->dd[t - 1];
	return A;
}

struct CVect* CVectabs(struct CVect* CV1)// 求绝对值
{
	for (int i = 0; i < CV1->rc; i++)
	{
		if (CV1->dd[i] < 0) { CV1->dd[i] = -CV1->dd[i]; }
	}
	return CV1;
}

//****************多维矩阵CMat相关运算***********//
struct CMat createCM(int row0, int clm0, double a)// 初始化，所有元素都为a
{
	struct CMat M = { 0 };
	M.clm = clm0; M.row = row0; M.rc = row0 * clm0;
	for (int i = 0; i < M.rc; i++)
	{
		M.dd[i] = a;
	}
	return M;
}

struct CMat* Setdiag(struct CMat* M, double a, ...)//方阵对角线元素赋值
{
	//	psinsassert(M->clm == M->row);
	va_list vl;
	va_start(vl, a);
	for (int i = 0, rc = M->row; i < rc; i++)
	{
		if (a > 2 * INF) break;
		M->dd[i * (M->clm + 1)] = a;
		a = va_arg(vl, double);
	}
	va_end(vl);
	return M;
}

void clear(struct CMat* M)//清零
{
	double* p = &M->dd[0];
	for (int i = 0; i < M->rc; i++)
	{
		*p = 0;
		p = p++;
	}
}
struct CMat* CMadd(struct CMat* M1, struct CMat* M2, struct CMat* M)//矩阵加法
{
	//	psinsassert(M1->row == M2->row && M1->clm = M2->clm);
	for (int i = 0; i < M1->rc; i++)
	{
		M->dd[i] = M1->dd[i] + M2->dd[i];
	}
	return M;
}

struct CMat* CMminus(struct CMat* M1, struct CMat* M2, struct CMat* M)//矩阵减法
{
	//	psinsassert(M1->row == M2->row && M1->clm = M2->clm);
	for (int i = 0; i < M1->rc; i++)
	{
		M->dd[i] = M1->dd[i] - M2->dd[i];
	}
	return M;
}

struct CMat* CVdiagCM(struct CVect* CV, struct CMat* M)//向量转换对角阵
{
	clear(M);
	int j = (CV->row > CV->clm) ? CV->row : CV->clm;
	for (int i = 0; i < j; i++)
	{
		M->dd[i * (M->clm + 1)] = CV->dd[i];
	}
	return M;
}

struct CMat* CMatmulcon(struct CMat* M, double a)//矩阵乘标量a
{
	for (int i = 0; i < M->rc; i++)
	{
		M->dd[i] = M->dd[i] * a;
	}
	return M;
}

struct CVect* Getrow(struct CMat* M, struct CVect* Row, int t)//取矩阵第t行
{
	Row->row = 1; Row->clm = M->clm;
	for (int i = 0; i < M->clm; i++)
	{
		Row->dd[i] = M->dd[t * M->clm + i];
	}
	return Row;
}

struct CVect* Getclm(struct CMat* M, struct CVect* CLM, int t)//取矩阵第t列
{
	CLM->row = M->row; CLM->clm = 1;
	for (int i = 0; i < M->row; i++)
	{
		CLM->dd[i] = M->dd[i * M->clm + t];
	}
	return CLM;
}

struct CVect* Getdiag(struct CMat* M1, struct CVect* CV)// 取对角线元素构造向量
{
	//	psinsassert(M1->clm == M1->row);
	for (int i = 0; i < M1->clm; i++)
	{
		CV->dd[i] = M1->dd[i * (M1->clm + 1)];
	}
	return CV;
}

struct CMat* CMatmulCM(struct CMat* M1, struct CMat* M2, struct CMat* M)//矩阵乘矩阵
{
	//	psinsassert(M1->clm == M2->row);
	struct CMat MM; MM.row = M1->row; MM.clm = M2->clm; MM.rc = MM.row * MM.clm;
	M->row = M1->row; M->clm = M2->clm; M->rc = M->row * M->clm;
	struct CVect Row; Row.row = 1; Row.clm = M1->clm; for (int r1 = 0; r1 < Row.clm; r1++) { Row.dd[r1] = 0; }//1*n多维行向量初始化
	struct CVect Clm; Clm.row = M2->row; Clm.clm = 1; for (int c1 = 0; c1 < Clm.row; c1++) { Clm.dd[c1] = 0; }//n*1多维列向量初始化
	for (int i = 0; i < MM.row; i++)
	{
		Row = *Getrow(M1, &Row, i);
		for (int j = 0; j < MM.clm; j++)
		{
			Clm = *Getclm(M2, &Clm, j);
			MM.dd[i * MM.clm + j] = CVmulCVCon(&Row, &Clm);
		}
	}
	for (int i = 0; i < MM.rc; i++)
	{
		M->dd[i] = MM.dd[i];
	}
	return M;
}

struct CMat* CMattran(struct CMat* M1, struct CMat* M)//转置
{
	//	psinsassert(M1->clm == M1->row);
	M->clm = M1->row; M->row = M1->clm;
	for (int i = 0; i < M->row; i++)
	{
		for (int j = 0; j < M->clm; j++)
		{
			M->dd[i * M->clm + j] = M1->dd[j * M1->clm + i];
		}
	}
	return M;
}
//struct CMat* CMattran1(struct CMat* M1, struct CMat* M)//普通矩阵转置
//{
//	struct CMat MM; MM.row = M1->clm; MM.clm = M1->row; MM.rc = M1->rc;
//	for (int i = 0; i < M->row; i++)
//	{
//		for (int j = 0; j < M->clm; j++)
//		{
//			M->dd[] = M1->dd[];
//		}
//	}
//}

struct CMat* CMatdetmul(struct CMat* M1, struct CMat* M2, struct CMat* M)//对应元素相乘
{
	//	psinsassert(M1->clm == M2->clm && M1->row == M2->row);
	M->row = M1->row; M->clm = M1->clm; M->rc = M1->rc;
	for (int i = 0; i < M1->rc; i++)
	{
		M->dd[i] = M1->dd[i] * M2->dd[i];
	}
	return M;
}

struct CVect* CMtmulCV(struct CMat* M, struct CVect* A, struct CVect* A1)//矩阵乘列向量
{
	//struct CVect Row; Row.row = 1; Row.clm = M->clm; for (int r1 = 0; r1 < Row.clm; r1++) { Row.dd[r1] = 0; }//1*n多维行向量初始化
	//struct CVect AA; AA = *A;
	//A1->clm = A->clm; A1->row = M->row; A1->rc = A1->clm * A1->row;
	//for (int i = 0; i < M->row; i++)
	//{
	//	Row = *Getrow(M, &Row, i);
	//	A1->dd[i] = CVmulCVCon(&Row, &AA);
	//}
	struct CVect AA; AA.row = A->row; AA.clm = 1; AA.rc = A->row;
	for (int j = 0; j < AA.rc; j++)
	{
		AA.dd[j] = A1->dd[j];
	}
	A1->row = M->row; A1->rc = A1->row;
	SetCV(0, A1);
	for (int i = 0; i < M->row; i++)
	{
		for (int ii = 0; ii < M->clm; ii++)
		{
			A1->dd[i] += M->dd[i * M->clm + ii] * AA.dd[ii];
		}
	}
	return A1;
}
//****************四元数Quat及姿态阵转换相关运算***********//
struct Quat* createQuat(struct Quat* Q, double qq0, double qq1, double qq2, double qq3)//创建四元数
{
	Q->q0 = qq0; Q->q1 = qq1; Q->q2 = qq2; Q->q3 = qq3;
	return Q;
}

struct CMAT3* a2mat(struct CVect3* CV, struct CMAT3* Cnb)//姿态角转换姿态阵（东北天右前上，312方式）
{
	double	si = sin(CV->i), ci = cos(CV->i),//俯仰角i;
		sj = sin(CV->j), cj = cos(CV->j),// 横滚角j;
		sk = sin(CV->k), ck = cos(CV->k);//方位角k;
	Cnb->e00 = cj * ck + si * sj * sk;	Cnb->e01 = ci * sk;	Cnb->e02 = sj * ck - si * cj * sk;
	Cnb->e10 = -cj * sk + si * sj * ck;	Cnb->e11 = ci * ck;	    Cnb->e12 = -sj * sk - si * cj * ck;
	Cnb->e20 = -ci * sj;				Cnb->e21 = si;		    Cnb->e22 = ci * cj;
	return Cnb;
}

struct CVect3* m2att(struct CMAT3* Cnb, struct CVect3* att)//姿态矩阵转换姿态角
{
	att->i = asinEx(Cnb->e21);
	att->j = Atan2Ex(-Cnb->e20, Cnb->e22);
	att->k = Atan2Ex(Cnb->e01, Cnb->e11);
	return att;
}

struct Quat* a2qua(const struct CVect3* CV, struct Quat* Q)//姿态角转换姿态四元数
{
	double si = sin(0.5 * CV->i), ci = cos(0.5 * CV->i), sj = (0.5 * CV->j), cj = (0.5 * CV->j), sk = (0.5 * CV->k), ck = (0.5 * CV->k);
	Q->q0 = ci * cj * ck - si * sj * sk;
	Q->q1 = si * cj * ck - ci * sj * sk;
	Q->q2 = ci * sj * ck + si * cj * sk;
	Q->q3 = ci * cj * sk + si * sj * ck;
	return Q;
}

struct CVect3* q2att(struct Quat* qnb, struct CVect3* C)//姿态四元数转化姿态角
{
	double	q11 = qnb->q0 * qnb->q0, q12 = qnb->q0 * qnb->q1, q13 = qnb->q0 * qnb->q2, q14 = qnb->q0 * qnb->q3,
		q22 = qnb->q1 * qnb->q1, q23 = qnb->q1 * qnb->q2, q24 = qnb->q1 * qnb->q3,
		q33 = qnb->q2 * qnb->q2, q34 = qnb->q2 * qnb->q3,
		q44 = qnb->q3 * qnb->q3;
	C->i = asinEx(2 * (q34 + q12));
	C->j = Atan2Ex(-2 * (q24 - q13), q11 - q22 - q33 + q44);
	C->k = Atan2Ex(2 * (q23 - q14), q11 - q22 + q33 - q44);//俯仰角i;横滚角j;方位角k;
	return C;
}

struct CVect3* q2rv(struct Quat* q, struct CVect3* CV)//四元数转换等效旋转矢量
{
	struct Quat dq = { 1,0,0,0 };

	dq = *createQuat(&dq, q->q0, q->q1, q->q2, q->q3);
	if (dq.q0 < 0) { dq.q0 = -dq.q0, dq.q1 = -dq.q1, dq.q2 = -dq.q2, dq.q3 = -dq.q3; }
	if (dq.q0 > 1.0) dq.q0 = 1.0;
	double n2 = acos(dq.q0), a;
	if (n2 > 1.0e-20)
	{
		a = 2.0 / (sin(n2) / n2);
	}
	else
	{
		a = 2.0;
	}
	CV->i = dq.q1 * a; CV->j = dq.q2 * a; CV->k = dq.q3 * a;
	return CV;
}

struct Quat* rv2q(struct CVect3* CV, struct Quat* Q)//等效旋转矢量转化四元数
{
#define F1	(   2 * 1)		// define: Fk=2^k*k! 
#define F2	(F1*2 * 2)
#define F3	(F2*2 * 3)
#define F4	(F3*2 * 4)
#define F5	(F4*2 * 5)
	double n2 = CV->i * CV->i + CV->j * CV->j + CV->k * CV->k, c, a;
	//	if (n2 < (PI / 180.0 * PI / 180.0))	// 0.017^2 
	if (n2 < 1.0e-8)
	{
		double n4 = n2 * n2;
		c = 1.0 - n2 * (1.0 / F2) + n4 * (1.0 / F4);//泰勒公式展开
		a = 0.5 - n2 * (1.0 / F3) + n4 * (1.0 / F5);
	}
	else
	{
		double n = sqrt(n2); double n_2 = n / 2.0;
		c = cos(n_2);
		a = sin(n_2) / n;
	}
	Q->q0 = c; Q->q1 = a * CV->i; Q->q2 = a * CV->j; Q->q3 = a * CV->k;
	return Q;
}


struct Quat* QuatmulQu(struct Quat* Q1, struct Quat* Q2, struct Quat* qtmp)//四元数相乘
{
	struct Quat Q;
	//Q.q0 = Q1->q0 * Q2->q0 - Q1->q1 * Q2->q1 - Q1->q2 * Q2->q2 - Q1->q3 * Q2->q3;
	//Q.q1 = Q1->q0 * Q2->q1 + Q1->q1 * Q2->q0 + Q1->q2 * Q2->q3 - Q1->q3 * Q2->q2;
	//Q.q2 = Q1->q0 * Q2->q2 + Q1->q2 * Q2->q0 + Q1->q3 * Q2->q1 - Q1->q1 * Q2->q3;
	//Q.q3 = Q1->q0 * Q2->q3 + Q1->q3 * Q2->q0 + Q1->q1 * Q2->q2 - Q1->q2 * Q2->q1;
	Q.q0 = Q1->q0 * Q2->q0 - Q1->q1 * Q2->q1 - Q1->q2 * Q2->q2 - Q1->q3 * Q2->q3;
	Q.q1 = Q1->q1 * Q2->q0 + Q1->q0 * Q2->q1 - Q1->q3 * Q2->q2 + Q1->q2 * Q2->q3;
	Q.q2 = Q1->q2 * Q2->q0 + Q1->q3 * Q2->q1 + Q1->q0 * Q2->q2 - Q1->q1 * Q2->q3;
	Q.q3 = Q1->q3 * Q2->q0 - Q1->q2 * Q2->q1 + Q1->q1 * Q2->q2 + Q1->q0 * Q2->q3;
	double n = sqrt(Q.q1 * Q.q1 + Q.q2 * Q.q2 + Q.q3 * Q.q3 + Q.q0 * Q.q0);
	Q.q0 = Q.q0 / n; Q.q1 = Q.q1 / n; Q.q2 = Q.q2 / n; Q.q3 = Q.q3 / n;
	qtmp->q0 = Q.q0; qtmp->q1 = Q.q1; qtmp->q2 = Q.q2; qtmp->q3 = Q.q3;
	return qtmp;
}

/*
struct CVect3 QuatmulCV(struct Quat* Q, struct CVect3* CV)//四元数乘矢量，坐标变换
{
	struct Quat qtmp;
	struct CVect3 vtmp;
	qtmp.q0 = -Q->q1 * CV->i - Q->q2 * CV->j - Q->q3 * CV->k;
	qtmp.q1 = Q->q0 * CV->i + Q->q2 * CV->k - Q->q3 * CV->j;
	qtmp.q2 = Q->q0 * CV->j + Q->q3 * CV->i - Q->q1 * CV->k;
	qtmp.q3 = Q->q0 * CV->k + Q->q1 * CV->j - Q->q2 * CV->i;//P*Q
	vtmp.i = -qtmp.q0 * Q->q1 + qtmp.q1 * Q->q0 - qtmp.q2 * Q->q3 + qtmp.q3 * Q->q2;
	vtmp.j = -qtmp.q0 * Q->q2 + qtmp.q2 * Q->q0 - qtmp.q3 * Q->q1 + qtmp.q1 * Q->q3;
	vtmp.k = -qtmp.q0 * Q->q3 + qtmp.q3 * Q->q0 - qtmp.q1 * Q->q2 + qtmp.q2 * Q->q1;
	return vtmp;
}
*/
struct Quat* tran(struct Quat* Q)//单位四元数转置/共轭
{
	Q->q1 = -Q->q1; Q->q2 = -Q->q2; Q->q3 = -Q->q3;
	return Q;
}

struct Quat* normlize(struct Quat* Q)//四元数归一化
{
	double nq = sqrt(Q->q0 * Q->q0 + Q->q1 * Q->q1 + Q->q2 * Q->q2 + Q->q3 * Q->q3);
	Q->q0 /= nq; Q->q1 /= nq; Q->q2 /= nq; Q->q3 /= nq;
	return Q;
}

struct Quat* SetYaw(struct Quat* Q1, double Yaw)//方位角设定为指定值
{
	struct CVect3 CV = { 0,0,0 };
	CV = *q2att(Q1, &CV);
	CV.k = Yaw;
	Q1 = a2qua(&CV, Q1);
	return Q1;
}

struct Quat* m2qua(struct CMAT3* Cnb, struct Quat* Q)//姿态阵转换四元数
{
	double q0, q1, q2, q3, qq4;
	if (Cnb->e00 >= Cnb->e11 + Cnb->e22)
	{
		q1 = 0.5 * sqrt(1 + Cnb->e00 - Cnb->e11 - Cnb->e22);  qq4 = 4 * q1;
		q0 = (Cnb->e21 - Cnb->e12) / qq4; q2 = (Cnb->e01 + Cnb->e10) / qq4; q3 = (Cnb->e02 + Cnb->e20) / qq4;
	}
	else if (Cnb->e11 >= Cnb->e00 + Cnb->e22)
	{
		q2 = 0.5 * sqrt(1 - Cnb->e00 + Cnb->e11 - Cnb->e22);  qq4 = 4 * q2;
		q0 = (Cnb->e02 - Cnb->e20) / qq4; q1 = (Cnb->e01 + Cnb->e10) / qq4; q3 = (Cnb->e12 + Cnb->e21) / qq4;
	}
	else if (Cnb->e22 >= Cnb->e00 + Cnb->e11)
	{
		q3 = 0.5 * sqrt(1 - Cnb->e00 - Cnb->e11 + Cnb->e22);  qq4 = 4 * q3;
		q0 = (Cnb->e10 - Cnb->e01) / qq4; q1 = (Cnb->e02 + Cnb->e20) / qq4; q2 = (Cnb->e12 + Cnb->e21) / qq4;
	}
	else
	{
		q0 = 0.5 * sqrt(1 + Cnb->e00 + Cnb->e11 + Cnb->e22);  qq4 = 4 * q0;
		q1 = (Cnb->e21 - Cnb->e12) / qq4; q2 = (Cnb->e02 - Cnb->e20) / qq4; q3 = (Cnb->e10 - Cnb->e01) / qq4;
	}
	double nq = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	Q->q0 = q0 / nq; Q->q1 = q1 / nq; Q->q2 = q2 / nq; Q->q3 = q3 / nq;
	return Q;
}

struct CMAT3* q2mat(struct Quat* qnb, struct CMAT3* Cnb)//四元数转换姿态矩阵
{
	double	q11 = qnb->q0 * qnb->q0, q12 = qnb->q0 * qnb->q1, q13 = qnb->q0 * qnb->q2, q14 = qnb->q0 * qnb->q3,
		q22 = qnb->q1 * qnb->q1, q23 = qnb->q1 * qnb->q2, q24 = qnb->q1 * qnb->q3,
		q33 = qnb->q2 * qnb->q2, q34 = qnb->q2 * qnb->q3,
		q44 = qnb->q3 * qnb->q3;
	Cnb->e00 = q11 + q22 - q33 - q44, Cnb->e01 = 2 * (q23 - q14), Cnb->e02 = 2 * (q24 + q13),
		Cnb->e10 = 2 * (q23 + q14), Cnb->e11 = q11 - q22 + q33 - q44, Cnb->e12 = 2 * (q34 - q12),
		Cnb->e20 = 2 * (q24 - q13), Cnb->e21 = 2 * (q34 + q12), Cnb->e22 = q11 - q22 - q33 + q44;
	return Cnb;
}
//矩阵求逆
struct CMat* LUinvMat(struct CMat* M, struct CMat* M1)//方阵
{
	struct CMat L; L.clm = M->clm; L.row = M->row; L.rc = L.clm * L.row;
	struct CMat U; U.clm = M->clm; U.row = M->row; U.rc = U.clm * U.row;
	for (int lu = 0; lu < L.rc; lu++)
	{
		U.dd[lu] = 0;
		if (lu % (L.row + 1) == 0)
		{
			L.dd[lu] = 1;
		}
		else
		{
			L.dd[lu] = 0;
		}
	}
	for (int i = 0; i < M->row; i++)//对角线元素循环
	{
		for (int j = 0; j < M->row - i; j++)//行循环
		{
			if (i == 0)
			{
				M1->dd[i * (M1->row + 1) + j] = M->dd[i * (M->row + 1) + j];
				U.dd[i * (M1->row + 1) + j] = M->dd[i * (M->row + 1) + j];
			}
			else
			{
				double sumC = 0;
				for (int ii = 0; ii < i; ii++)
				{
					sumC = sumC + M1->dd[i * (M1->row + 1) + j - M1->row * i + M->row * ii] * M1->dd[M1->row * i + ii];
				}
				M1->dd[i * (M1->row + 1) + j] = M->dd[i * (M->row + 1) + j] - sumC;
				U.dd[i * (M1->row + 1) + j] = M->dd[i * (M->row + 1) + j] - sumC;

			}
		}
		for (int k = 0; k < M->clm - i - 1; k++)//列循环
		{
			if (i == 0)
			{
				M1->dd[i * (M1->row + 1) + M1->row * (k + 1)] = M->dd[i * (M->row + 1) + M->row * (k + 1)] / M1->dd[i * (M1->row + 1)];
				L.dd[i * (M1->row + 1) + M1->row * (k + 1)] = M->dd[i * (M->row + 1) + M->row * (k + 1)] / M1->dd[i * (M1->row + 1)];
			}
			else
			{
				double sumR = 0;
				for (int kk = 0; kk < i; kk++)
				{
					sumR = sumR + M1->dd[M1->row * (i)+M1->row * (k + 1) + kk] * M1->dd[M1->row * i + i - i * M1->row + kk * M1->row];
				}
				M1->dd[i * (M1->row + 1) + (k + 1) * M1->row] = M->dd[i * (M->row + 1) + (k + 1) * M->row] - sumR;
				M1->dd[i * (M1->row + 1) + (k + 1) * M1->row] = M1->dd[i * (M1->row + 1) + (k + 1) * M1->row] / M1->dd[i * (M1->row + 1)];
				L.dd[i * (M1->row + 1) + (k + 1) * M1->row] = M1->dd[i * (M1->row + 1) + (k + 1) * M1->row];
			}
		}
	}
	//L求逆
	for (int i = 0; i < L.row; i++)
	{
		for (int j = 0; j <= i; j++)
		{
			if (i == j)
			{
				L.dd[i * L.row + j] = 1.0 / L.dd[i * L.row + j];
			}
			else
			{
				double Lsum = 0;
				for (int k = j; k < i; k++)
				{
					Lsum += L.dd[i * L.row + k] * L.dd[k * L.row + j];
				}
				L.dd[i * L.row + j] = -1.0 * Lsum / L.dd[i * L.row + i];
			}

		}
	}
	//U求逆
	for (int j = U.clm - 1; j >= 0; j--)
	{
		for (int i = j; i >= 0; i--)
		{
			if (i == j)
			{
				U.dd[i * U.row + j] = 1 / U.dd[i * U.row + j];
			}
			else
			{
				double Usum = 0;
				for (int k = j; k >= i + 1; k--)
				{
					Usum += U.dd[i * U.row + k] * U.dd[k * U.row + j];
				}
				U.dd[i * U.row + j] = -1.0 * Usum / U.dd[i * U.row + i];
			}
		}
	}
	//U^-1*L^-1
	M1 = CMatmulCM(&U, &L, M1);
	return M1;
}


//******************地球参数计算*******************//
/*
void CEarthUpdate(struct CEarth* Ear, double t)//上一时刻惯导速度位置更新地球参数， 惯导更新时间
{
	//外推法计算快变速度增量
	struct CVect3 P = O31; struct CVect3 V = O31;
	P = *CV3protectmul1(&P, &Ear->pos, 1.5); V = *CV3protectmul1(&V, &Ear->vn, 1.5);
	P = *CV3minus(&P, &Ear->pos0); V = *CV3minus(&V, &Ear->vn0);//外推法计算速度位置pos0为0.5*pos0
	Ear->pos0 = *CV3protectmul1(&Ear->pos0, &Ear->pos, 0.5);
	Ear->vn0 = *CV3protectmul1(&Ear->vn0, &Ear->vn, 0.5); //更新前一时刻0.5倍速度位置信息
	//地球半径参数外推
	Ear->sl = sin(P.i); Ear->cl = cos(P.i); Ear->tl = Ear->sl / Ear->cl; Ear->sl2 = Ear->sl * Ear->sl; Ear->s2l2 = sin(2 * P.i) * sin(2 * P.i);
	double sq = 1.0 - e2 * Ear->sl * Ear->sl, sq2 = sqrt(sq);
	double RN = Re / sq2;
	Ear->RNh = RN + P.k; Ear->RMh = (RN * (1 - e2) / sq2)+P.k;//计算Rmh Rnh
	//计算MPV矩阵
	Ear->f_RMh = 1.0 / Ear->RMh; Ear->f_RNh = 1.0 / Ear->RNh;
	Ear->clRNh = Ear->cl * Ear->RNh;  Ear->f_clRNh = 1.0 / Ear->clRNh;
	Ear->mpv.e01 = Ear->f_RMh;
	Ear->mpv.e10 = Ear->f_clRNh;
	Ear->mpv.e22 = 1.0;//位置更新系数矩阵Mpv外推
	//地球角速度相关外推
	Ear->wnie.i = 0.0; Ear->wnie.j = wie * Ear->cl; Ear->wnie.k = wie * Ear->sl;
	Ear->wnen.i = -V.j * Ear->f_RMh; Ear->wnen.j = V.i * Ear->f_RNh; Ear->wnen.k = Ear->wnen.j * Ear->tl;
	Ear->wnin = *CV3protectminus(&Ear->wnin, &Ear->wnie, &Ear->wnen);//角速度相关外推
	//重力外推
	Ear->gn.i = 0.0; Ear->gn.j = -b3 * P.k * sqrt(Ear->s2l2);
	Ear->gn.k = -g0 * (1.0 + b * Ear->sl2 - b1 * Ear->s2l2) - b2 * P.k;//重力加速度随高度纬度变化公式
	//慢变速度求取
	Ear->gcc.i = Ear->gn.i; Ear->gcc.k = Ear->gn.k; Ear->gcc.j = Ear->gn.j;
	struct CVect3 w = O31;
	w = *CV3protectadd(&w, &Ear->wnie, &Ear->wnin);
	w = *operatormul3(&w, &V, &w);
	Ear->gcc = *CV3minus(&Ear->gcc, &w);//有害加速度
	Ear->gcc = *CV3mul1(&Ear->gcc, t);//慢变速度
	Ear->dotwninT = *askew(&Ear->wnin, &Ear->dotwninT);
	Ear->dotwninT = *operatorCM3mulf(&Ear->dotwninT, -0.5*t);//-0.5*T*wnin,速度更新用
	//瞬时参数更新
	Ear->sl = sin(Ear->pos.i); Ear->cl = cos(Ear->pos.i); Ear->tl = Ear->sl / Ear->cl; Ear->sl2 = Ear->sl * Ear->sl; Ear->s2l2 = sin(2 * Ear->pos.i) * sin(2 * Ear->pos.i);
	sq = 1.0 - e2 * Ear->sl * Ear->sl, sq2 = sqrt(sq);
	RN = Re / sq2;
	Ear->RNh = RN + Ear->pos.k;  Ear->RMh = (RN * (1 - e2) / sq2) + Ear->pos.k;
	Ear->f_RMh = 1.0 / Ear->RMh; Ear->f_RNh = 1.0 / Ear->RNh; Ear->clRNh = Ear->cl * Ear->RNh; Ear->f_clRNh = 1.0 / Ear->clRNh;//地球参数
	Ear->wnie.j = wie * Ear->cl; Ear->wnie.k = wie * Ear->sl;
	Ear->wnen.i = -Ear->vn.j * Ear->f_RMh, Ear->wnen.j = Ear->vn.i * Ear->f_RNh; Ear->wnen.k = Ear->wnen.j * Ear->tl;
	Ear->wnin = *CV3protectminus(&Ear->wnin, &Ear->wnie, &Ear->wnen);//角速度计算
	Ear->gn.j = -b3 * Ear->pos.k * sqrt(Ear->s2l2);
	Ear->gn.k = -g0 * (1.0 + b * Ear->sl2 - b1 * Ear->s2l2) - b2 * Ear->pos.k;//重力加速度计算
}
*/
void CEarthUpdate(struct CEarth* Ear, double t)//上一时刻惯导速度位置更新地球参数， 惯导更新时间//不进行外推
{
	Ear->sl = sin(Ear->pos.i); Ear->cl = cos(Ear->pos.i); Ear->tl = Ear->sl / Ear->cl; Ear->sl2 = Ear->sl * Ear->sl; Ear->s2l2 = sin(2 * Ear->pos.i) * sin(2 * Ear->pos.i);
	double sq = 1.0 - e2 * Ear->sl * Ear->sl;
	double sq2 = sqrt(sq);
	double RN = Re / sq2;
	Ear->RNh = RN + Ear->pos.k;  Ear->RMh = (RN * (1 - e2) / sq2) + Ear->pos.k;
	Ear->f_RMh = 1.0 / Ear->RMh; Ear->f_RNh = 1.0 / Ear->RNh; Ear->clRNh = Ear->cl * Ear->RNh; Ear->f_clRNh = 1.0 / Ear->clRNh;//地球参数

	Ear->wnie.i = 0.0; Ear->wnie.j = wie * Ear->cl; Ear->wnie.k = wie * Ear->sl;
	Ear->wnen.i = -Ear->vn.j * Ear->f_RMh; Ear->wnen.j = Ear->vn.i * Ear->f_RNh; Ear->wnen.k = Ear->wnen.j * Ear->tl;
	Ear->wnin = *CV3protectadd(&Ear->wnin, &Ear->wnie, &Ear->wnen);//角速度计算

	Ear->gn.i = 0.0; Ear->gn.j = -b3 * Ear->pos.k * sqrt(Ear->s2l2);
	Ear->gn.k = -g0 * (1.0 + b * Ear->sl2 - b1 * Ear->s2l2) - b2 * Ear->pos.k;//重力加速度随高度纬度变化公式
	//慢变速度求取
	Ear->gcc.i = Ear->gn.i; Ear->gcc.k = Ear->gn.k; Ear->gcc.j = Ear->gn.j;
	struct CVect3 w = O31;
	w = *CV3protectadd(&w, &Ear->wnie, &Ear->wnin);
	w = *operatormul3(&w, &Ear->vn, &w);
	Ear->gcc = *CV3minus(&Ear->gcc, &w);//有害加速度	
	Ear->gcc = *CV3mul1(&Ear->gcc, t);//慢变速度
	//-0.5*T*wnin,速度更新用
	Ear->dotwninT = *askew(&Ear->wnin, &Ear->dotwninT);
	Ear->dotwninT = *operatorCM3mulf(&Ear->dotwninT, -0.5 * t);
	//计算MPV矩阵
	Ear->f_RMh = 1.0 / Ear->RMh; Ear->f_RNh = 1.0 / Ear->RNh;
	Ear->clRNh = Ear->cl * Ear->RNh;  Ear->f_clRNh = 1.0 / Ear->clRNh;
	Ear->mpv.e01 = Ear->f_RMh;
	Ear->mpv.e10 = Ear->f_clRNh;
	Ear->mpv.e22 = 1.0;//位置更新系数矩阵Mpv外推

}
/*
void IniCEarth(struct CEarth* Ear, struct CVect3* vn, struct CVect3* pos, double t)//地球参数初始化
{
	Ear->vn.i = Ear->vn0.i = vn->i;
	Ear->vn.j = Ear->vn0.j = vn->j;
	Ear->vn.k = Ear->vn0.k = vn->k;
	Ear->pos.i = Ear->pos0.i = pos->i;
	Ear->pos.j = Ear->pos0.j = pos->j;
	Ear->pos.k = Ear->pos0.k = pos->k;
	Ear->vn0 = *CV3mul1(&Ear->vn0, 0.5);
	Ear->pos0 = *CV3mul1(&Ear->pos0, 0.5);
	Ear->mpv = O33;
	CEarthUpdate(Ear, t);
}
*/
void IniCEarth(struct CEarth* Ear, struct CVect3* vn, struct CVect3* pos, double t)//地球参数初始化
{
	Ear->vn.i = vn->i;
	Ear->vn.j = vn->j;
	Ear->vn.k = vn->k;
	Ear->pos.i = pos->i;
	Ear->pos.j = pos->j;
	Ear->pos.k = pos->k;
	Ear->mpv = O33;
	CEarthUpdate(Ear, t);
}
/*
struct CVect3* posUpdate(struct CMAT3* C, struct CVect3* P,struct CVect3* V1, struct CVect3* V2, double T)//位置增量计算V1此时刻更新速度，V2上一时刻速度Ear->vn0,T惯导更新时间
{
	P = CV3protectadd(P, V1, V2);//vm+0.5*vm-1
	P = CV3add(P,V2); //vm+vm-1
	P = CM3mulCV3(C, P, P);
	P = CV3mul1(P, 0.5*T);
	return P;
}
*/
struct CVect3* posUpdate(struct CMAT3* C, struct CVect3* P, struct CVect3* V1, double T)//位置增量计算V1此时刻更新速度，T惯导更新时间//不外推
{
	P = CV3add(P, V1);
	P = CV3mul1(P, T);
	P = CM3mulCV3(C, P, P);
	return P;
}

//*****************捷联惯导算法**************************//

struct Quat qnbUpdate(struct Quat* Q, struct CVect3* wnin, struct CVect3* phim, double nts)  //姿态矩阵更新
{
	struct CVect3 C1 = { 0,0,0 };
	C1 = *CV3protectmul1(&C1, wnin, -1 * nts);
	struct Quat Q1 = { 1,0,0,0 };
	Q1 = *rv2q(&C1, &Q1);
	struct Quat Q2 = { 1,0,0,0 };
	Q2 = *rv2q(phim, &Q2);
	struct Quat Q3 = { 1,0,0,0 };
	Q3 = *QuatmulQu(&Q1, Q, &Q3);
	struct Quat Q4 = { 1,0,0,0 };
	Q4 = *QuatmulQu(&Q3, &Q2, &Q4);
	return Q4;
}

void IniIns(struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct CVect3* att0, struct CVect3* vn0, struct CVect3* pos0, double tk0, double t, int nSamples)//INS初始化 ins结构体 初始姿态四元数 初始速度 初始位置 初始时间 惯导采样时间 字样数
{
	ins->tk = ins->tk0 = tk0; ins->ts = t; ins->nts = ins->ts * nSamples;
	IniCEarth(eth, vn0, pos0, ins->nts);//地球参数初始化
	IniCIMU(cimu, t, nSamples);//惯性单元参数初始化
	ins->att.i = att0->i; ins->att.j = att0->j; ins->att.k = att0->k;
	ins->Cnb = *a2mat(&ins->att, &ins->Cnb);
	ins->qnb = *m2qua(&ins->Cnb, &ins->qnb);
	ins->wib = ins->fb = ins->fn = O31;
	ins->Cbn = *operatorCM3tran(&ins->Cnb, &ins->Cbn);
	ins->x = 0; ins->heading = 0.0;
}

void INSUpdate(struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, double* pwm, double* pvm)//捷联式惯导更新pwm\pvm多维向量，3*n,与子样数有关
{
	//时间更新
	ins->tk = ins->tk + ins->nts;
	//计算不可交换误差        
	CIMUUpdate(cimu, pwm, pvm);
	ins->fb = *CV3protectmul1(&ins->fb, &cimu->dvbm, 1 / ins->nts);
	ins->wib = *CV3protectmul1(&ins->wib, &cimu->phim, 1 / ins->nts);
	ins->fn = *CM3mulCV3(&ins->Cnb, &ins->fb, &ins->fn);
	//地球参数更新
	CEarthUpdate(eth, ins->nts);
	//速度更新
	struct CVect3 an;
	an = *CM3mulCV3(&ins->Cnb, &cimu->dvbm, &an);	//Cnb*快变速度增量
	struct CMAT3 CC = { 1,0,0,0,1,0,0,0,1 };
	eth->dotwninT = *operatorCM3add(&eth->dotwninT, &CC);
	an = *CM3mulCV3(&eth->dotwninT, &an, &an); //快变速度增量
	eth->vn = *CV3add(&eth->vn, &an);
	eth->vn = *CV3add(&eth->vn, &eth->gcc);
	//位置更新
	struct CVect3 P = { 0,0,0 };
	//	P = *posUpdate(&ins->eth.mpv,&P, &ins->eth.vn, &ins->eth.vn0, ins->nts);//位置增量计算
	P = *posUpdate(&eth->mpv, &P, &eth->vn, ins->nts);
	eth->pos = *CV3add(&eth->pos, &P);
	//姿态更新
	ins->qnb = qnbUpdate(&ins->qnb, &eth->wnin, &cimu->phim, ins->nts);//姿态四元数更新
	ins->Cnb = *q2mat(&ins->qnb, &ins->Cnb); //四元数转换姿态矩阵
	ins->att = *m2att(&ins->Cnb, &ins->att); //姿态矩阵转换欧拉角
	ins->Cbn = *operatorCM3tran(&ins->Cnb, &ins->Cbn); //转置姿态矩阵
	ins->x = ins->x + 1;
}


//*****************卡尔曼滤波算法**************************
void setCMAT3IntoCM(struct CMat* MM, struct CMAT3* M, int r, int c)//将33矩阵放入大矩阵行列对应位置中 放入r行c列位置
{
	double* p = &MM->dd[MM->clm * (r - 1) + (c - 1)];
	double* d = &M->e00;
	for (int i = 0; i < 3; i++)
	{
		*p = *d; p++; d++;
		*p = *d; p++; d++;
		*p = *d;
		p = p + MM->clm - 2;
		d++;
	}
}

//离散化
void Kaldiscretizing_D(struct CMat* Fk, struct CMat* II, int x, int n)//状态转移矩阵 惯导执行次数 转移矩阵非0维度数
{
	for (int i = 0; i < II->row; i++)
	{
		II->dd[i * II->clm + i] = 1;   //对角元素赋值1
	}

	for (int i = 0; i < Fk->clm*n; i++)
	{
		II->dd[i] = II->dd[i] + Fk->dd[i] * x;//状态转移矩阵离散化
	}
}

//SWg陀螺仪随机游走；SWa加标随机游走；
void IniKalman_TD(struct Kalman15* Kal, struct CVect3* SWg, struct CVect3* SWa, struct CVect3* Stdatt, struct CVect3* Stdvn, struct CVect3* Stdpos,struct CVect3* Stdgyro, struct CVect3* Stdacc)//15维分片导航初始化
{
	Kal->Fk.row = Kal->Fk.clm = 15; Kal->Fk.rc = 225; clear(&Kal->Fk);
	Kal->II.row = Kal->II.clm = 15; Kal->II.rc = 225; clear(&Kal->II);
	Kal->Q.row = 6; Kal->Q.clm = 1; Kal->Q.rc = 6;
	Kal->Q.dd[0] = SWg->i * SWg->i ; Kal->Q.dd[1] = SWg->j * SWg->j ; Kal->Q.dd[2] = SWg->k * SWg->k ;
	Kal->Q.dd[3] = SWa->i * SWa->i ; Kal->Q.dd[4] = SWa->j * SWa->j ; Kal->Q.dd[5] = SWa->k * SWa->k ;
	Kal->Qkf.row = Kal->Qkf.clm = 15; Kal->Qkf.rc = 225; clear(&Kal->Qkf);
	Kal->Pk1.row = Kal->Pk1.clm = 15; Kal->Pk1.rc = 225; clear(&Kal->Pk1);
	Kal->Pk.row = Kal->Pk.clm = 15; Kal->Pk.rc = 225; clear(&Kal->Pk);
	double* p = &Kal->Pk.dd[0];
	*p = Stdatt->i * Stdatt->i; p = p + 16;
	*p = Stdatt->j * Stdatt->j; p = p + 16;
	*p = Stdatt->k * Stdatt->k; p = p + 16;
	*p = Stdvn->i * Stdvn->i; p = p + 16;
	*p = Stdvn->j * Stdvn->j; p = p + 16;
	*p = Stdvn->k * Stdvn->k; p = p + 16;
	*p = Stdpos->i * Stdpos->i; p = p + 16;
	*p = Stdpos->j * Stdpos->j; p = p + 16;
	*p = Stdpos->k * Stdpos->k; p = p + 16;
	*p = Stdgyro->i * Stdgyro->i; p = p + 16;
	*p = Stdgyro->j * Stdgyro->j; p = p + 16;
	*p = Stdgyro->k * Stdgyro->k; p = p + 16;
	*p = Stdacc->i * Stdacc->i; p = p + 16;
	*p = Stdacc->j * Stdacc->j; p = p + 16;
	*p = Stdacc->k * Stdacc->k;             //p0赋值
	Kal->H.row = 3; Kal->H.clm = 15; Kal->H.rc = 45; clear(&Kal->H);
	Kal->H.dd[6] = 1; Kal->H.dd[22] = 1; Kal->H.dd[38] = 1;
	Kal->Xk.row = 15; Kal->Xk.clm = 1; Kal->Xk.rc = 15; SetCV(0.0, &Kal->Xk);
	Kal->Zk.row = 3; Kal->Zk.clm = 1; Kal->Zk.rc = 3; SetCV(0.0, &Kal->Zk);  //后续赋值
	Kal->Rkf.row = 3; Kal->Rkf.clm = 1; Kal->Rkf.rc = 3;
	Kal->K.row = Kal->Pxrk.row = 15; Kal->K.clm = Kal->Pxrk.clm = 1; Kal->K.rc = Kal->Pxrk.rc = 15; SetCV(0.0, &Kal->K); SetCV(0.0, &Kal->Pxrk);
	Kal->nq = 15; Kal->n = 0; Kal->nn = 0; Kal->iter = -2; Kal->nstep = 3;

}

void getObservations(struct CVect3* GNSSpos,struct CVect3* INSpos,struct CVect* Zk,struct CVect* Rkf,struct CVect3* GNSSerr,struct CVect3* DOP, uint8_t* DataRevision,bool* GNSSvalidity)//得到观测量
{
	if (*DataRevision == 1)
	{
		//位置观测
		Zk->dd[0] = INSpos->i - GNSSpos->i;
		Zk->dd[1] = INSpos->j - GNSSpos->j;
		Zk->dd[2] = INSpos->k - GNSSpos->k;
		//R阵
		Rkf->dd[0] = (GNSSerr->i * DOP->i) * (GNSSerr->i * DOP->i);
		Rkf->dd[1] = (GNSSerr->j * DOP->j) * (GNSSerr->j * DOP->j);
		Rkf->dd[2] = (GNSSerr->k * DOP->k) * (GNSSerr->k * DOP->k);

		*DataRevision = 0;
		*GNSSvalidity = true;
	}
	
}

void StateCompensation_TD(struct CVect* Xk, struct CMAT3* Cnb,struct Quat* q,struct CVect3* pos,struct CVect3* vn,struct CVect3* att)//状态补偿
{
	vn->i -= Xk->dd[3];
	vn->j -= Xk->dd[4];
	vn->k -= Xk->dd[5];
	pos->i -= Xk->dd[6];
	pos->j -= Xk->dd[7];
	pos->k -= Xk->dd[8];
	struct CVect3 ab = { Xk->dd[0],Xk->dd[1],Xk->dd[2] };
	struct Quat qq;
	qq = *rv2q(&ab, &qq);
	q = QuatmulQu(&qq, q, q);
	Cnb = q2mat(q, Cnb);
	att = m2att(Cnb, att);
	Xk->dd[0] = 0; Xk->dd[1] = 0; Xk->dd[2] = 0;
	Xk->dd[3] = 0; Xk->dd[4] = 0; Xk->dd[5] = 0;
	Xk->dd[6] = 0; Xk->dd[7] = 0; Xk->dd[8] = 0;
}

void KalsetFk_INAV(struct Kalman15* Kal, struct CSINS* ins, struct CEarth* eth, double T)//设置F阵   T 惯导更新时间
{
	//M1:[0 0 0;-Wnie(3) 0 0;Wnie(2) 0 0];
	Kal->M1.e00 = 0; Kal->M1.e01 = 0; Kal->M1.e02 = 0;
	Kal->M1.e10 = -eth->wnie.k * T; Kal->M1.e11 = 0; Kal->M1.e12 = 0;
	Kal->M1.e20 = eth->wnie.j * T; Kal->M1.e21 = 0; Kal->M1.e22 = 0;
	//M2:[0 0 vn(2)/((Rm+P(3))*(Rm+P(3)));0 0 -vn(1)/((Rn+P(3))*(Rn+P(3)));vn(1)/(cos(P(1))*cos(P(1))*(Rn+P(3))) 0 -vn(1)*tan(P(1))/((Rn+P(3))*(Rn+P(3)))];
	Kal->M2.e00 = 0; Kal->M2.e01 = 0; Kal->M2.e02 = eth->vn.j * eth->f_RMh * eth->f_RNh * T;
	Kal->M2.e10 = 0; Kal->M2.e11 = 0; Kal->M2.e12 = -eth->vn.i * eth->f_RNh * eth->f_RNh * T;
	Kal->M2.e20 = eth->vn.i * eth->f_clRNh * T / eth->cl; Kal->M2.e21 = 0; Kal->M2.e22 = -eth->vn.i * eth->f_RNh * eth->f_RNh * eth->tl * T;
	//M3:[0 0 0;-2*b3*P(3)*cos(2*P(1)) 0 -b3*sin(2*P(1));-g0*(b-4*b1*cos(2*P(1)))*sin(2*P(1)) 0 b2];
	Kal->M3.e00 = 0; Kal->M3.e01 = 0; Kal->M3.e02 = 0;
	Kal->M3.e10 = -2 * b3 * eth->pos.k * cos(2 * eth->pos.i) * T; Kal->M3.e11 = 0; Kal->M3.e12 = -1 * b3 * sin(2 * eth->pos.i) * T;
	Kal->M3.e20 = -1 * g0 * (b - 4 * b1 * cos(2 * eth->pos.i)) * sin(2 * eth->pos.i) * T; Kal->M3.e21 = 0; Kal->M3.e22 = b2 * T;
	//Maa
	Kal->Maa = *askew(&eth->wnin, &Kal->Maa);
	Kal->Maa = *operatorCM3mulf(&Kal->Maa, -T);
	//Mav:[0 -1/(Rm+P(3)) 0;1/(Rn+P(3)) 0 0;tan(P(1))/(Rn+P(3)) 0 0];
	Kal->Mav.e00 = 0; Kal->Mav.e01 = -1 * eth->f_RMh * T; Kal->Mav.e02 = 0;
	Kal->Mav.e10 = eth->f_RNh * T; Kal->Mav.e11 = 0; Kal->Mav.e12 = 0;
	Kal->Mav.e20 = eth->tl * eth->f_RNh * T; Kal->Mav.e21 = 0; Kal->Mav.e22 = 0;
	//Map:M1+M2
	Kal->Map.e00 = (Kal->M1.e00 + Kal->M2.e00) ;
	Kal->Map.e01 = (Kal->M1.e01 + Kal->M2.e01) ;
	Kal->Map.e02 = (Kal->M1.e02 + Kal->M2.e02) ;
	Kal->Map.e10 = (Kal->M1.e10 + Kal->M2.e10) ;
	Kal->Map.e11 = (Kal->M1.e11 + Kal->M2.e11) ;
	Kal->Map.e12 = (Kal->M1.e12 + Kal->M2.e12) ;
	Kal->Map.e20 = (Kal->M1.e20 + Kal->M2.e20) ;
	Kal->Map.e21 = (Kal->M1.e21 + Kal->M2.e21) ;
	Kal->Map.e22 = (Kal->M1.e22 + Kal->M2.e22) ;
	//Mva:dot(fn);
	Kal->Mva = *askew(&ins->fn, &Kal->Mva);
	Kal->Mva = *operatorCM3mulf(&Kal->Mva, T);
	//Mvv:dot(vn)*Mav - dot(2*Wnie+Wnen);
	struct CMAT3 w = O33;
	Kal->Mvv = *operatorCM3mul(askew(&eth->vn, &w), &Kal->Mav, &Kal->Mvv);
	struct CVect3 ww = O31;
	ww = *CV3protectadd(&ww, &eth->wnin, &eth->wnie);
	w = *askew(&ww, &w);
	w = *operatorCM3mulf(&w, T);
	Kal->Mvv = *operatorCM3minus(&Kal->Mvv, &w);
	//Mvp:(dot(vn)*(M2+M1))+(dot(vn)*M1)+M3;
	struct CMAT3 M;
	Kal->M1 = *operatorCM3add(&Kal->M1, &Kal->Map);
	M = *askew(&eth->vn, &M);
	Kal->Mvp = *operatorCM3mul(&M, &Kal->M1, &Kal->Mvp);
	Kal->Mvp = *operatorCM3add(&Kal->Mvp, &Kal->M3);
	//Mpv
	Kal->Mpv.e00 = 0;                Kal->Mpv.e01 = eth->f_RMh * T; Kal->Mpv.e02 = 0;
	Kal->Mpv.e10 = eth->f_clRNh * T; Kal->Mpv.e11 = 0;				Kal->Mpv.e12 = 0;
	Kal->Mpv.e20 = 0;				 Kal->Mpv.e21 = 0;				Kal->Mpv.e22 = T;
	//Mpp
	Kal->Mpp.e00 = 0;									   Kal->Mpp.e01 = 0;		Kal->Mpp.e02 = -eth->vn.j * eth->f_RMh * eth->f_RMh * T;
	Kal->Mpp.e10 = eth->vn.i * eth->tl * eth->f_clRNh * T; Kal->Mpp.e11 = 0;		Kal->Mpp.e12 = -eth->vn.i * eth->f_RNh * eth->f_clRNh * T;
	Kal->Mpp.e20 = 0;									   Kal->Mpp.e21 = 0;		Kal->Mpp.e22 = 0;
	//
	struct CMAT3 c;
	c.e00 = ins->Cnb.e00 * -T;
	c.e01 = ins->Cnb.e01 * -T;
	c.e02 = ins->Cnb.e02 * -T;
	c.e10 = ins->Cnb.e10 * -T;
	c.e11 = ins->Cnb.e11 * -T;
	c.e12 = ins->Cnb.e12 * -T;
	c.e20 = ins->Cnb.e20 * -T;
	c.e21 = ins->Cnb.e21 * -T;
	c.e22 = ins->Cnb.e22 * -T;
	//整合
	setCMAT3IntoCM(&Kal->Fk, &Kal->Maa, 1, 1);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mav, 1, 4);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Map, 1, 7);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mva, 4, 1);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mvv, 4, 4);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mvp, 4, 7);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mpv, 7, 4);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mpp, 7, 7);
	setCMAT3IntoCM(&Kal->Fk, &c, 1, 10);//-cnb*T
	c = *operatorCM3mulf(&c, -1);
	setCMAT3IntoCM(&Kal->Fk, &c, 4, 13);//cnb*T
}

void Kal_INAV_UPdata(struct Kalman15* Kal, struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct GNSSDATA* GNSS,double* pwm, double* pvm,uint8_t* DataRevision)//更新
{
	INSUpdate(ins, eth, cimu, pwm,pvm);
	for (int i = 0; i < Kal->nstep; i++)
	{
		if (Kal->iter <= -2)//构建F阵 获取观测量
		{
			getObservations(&GNSS->GNSSpos, &eth->pos, &Kal->Zk, &Kal->Rkf, &GNSS->GNSSerr, &GNSS->DOP, DataRevision, &GNSS->GNSSvalidity);
			KalsetFk_INAV(Kal, ins, eth, ins->nts);
		}
		else if(Kal->iter <= -1)//离散化
		{
			//Q离散化
			struct CMAT3 QkfG;
			struct CMAT3 Q = O33; 
			struct CMAT3 CnbT = *operatorCM3tran(&ins->Cnb, &CnbT);
			Q.e00 = Kal->Q.dd[0] * ins->nts; Q.e11 = Kal->Q.dd[1] * ins->nts; Q.e22 = Kal->Q.dd[2] * ins->nts;//gyro
			//C*Q*C*ins->x
			QkfG = *operatorCM3mul(&ins->Cnb, &Q, &QkfG);
			QkfG = *operatorCM3mul(&QkfG, &CnbT, &QkfG);
			QkfG = *operatorCM3mulf(&QkfG, ins->x);
			struct CMAT3 QkfA;
			Q.e00 = Kal->Q.dd[3] * ins->nts; Q.e11 = Kal->Q.dd[4] * ins->nts; Q.e22 = Kal->Q.dd[5] * ins->nts;//acc
			QkfA = *operatorCM3mul(&ins->Cnb, &Q, &QkfA);
			QkfA = *operatorCM3mul(&QkfA, &CnbT, &QkfA);
			QkfA = *operatorCM3mulf(&QkfA, ins->x);
			setCMAT3IntoCM(&Kal->Qkf, &QkfG, 1, 1);
			setCMAT3IntoCM(&Kal->Qkf, &QkfA, 4, 4);

			//离散化
			Kaldiscretizing_D(&Kal->Fk, &Kal->II, ins->x, 9); //状态转移矩阵 惯导执行次数 转移矩阵非0维度数
			ins->x = 0;
		}
		else if(Kal->iter <= 0)//先验估计
		{
			// 先验估计
			Kal->Xk = *CMtmulCV(&Kal->II, &Kal->Xk, &Kal->Xk);
		}
		else if(Kal->iter <= 10)//误差均方差矩阵更新II*PK
		{
			if (Kal->iter <= 9)
			{
				for (int j = 0; j < 15; j++)
				{
					for (int jj = 0; jj < 15; jj++)
					{
						Kal->Pk1.dd[(Kal->iter - 1) * Kal->Pk1.clm + j] += Kal->II.dd[(Kal->iter - 1) * Kal->II.clm + jj] * Kal->Pk.dd[jj * Kal->Pk.clm + j];
					}
				}
			}
			else
			{
				for (int j = 0; j < 15; j++)
				{
					Kal->Pk1.dd[(Kal->iter - 1) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter - 1) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter - 1) * Kal->Pk1.clm + j];
					Kal->Pk1.dd[Kal->iter * Kal->Pk1.clm + j] = Kal->II.dd[Kal->iter * (Kal->II.clm + 1)] * Kal->Pk.dd[Kal->iter * Kal->Pk1.clm + j];
					Kal->Pk1.dd[(Kal->iter + 1) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 1) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 1) * Kal->Pk1.clm + j];
					Kal->Pk1.dd[(Kal->iter + 2) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 2) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 2) * Kal->Pk1.clm + j];
					Kal->Pk1.dd[(Kal->iter + 3) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 3) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 3) * Kal->Pk1.clm + j];
					Kal->Pk1.dd[(Kal->iter + 4) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 4) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 4) * Kal->Pk1.clm + j];
				}
			}
		}
		else if (Kal->iter <= 18)//Pk1*II'+Qkf
		{
			
			if (Kal->iter <= 11)//第一行
			{
				for (int num = 0; num < 225; num++)
				{
					Kal->Pk.dd[num] = 0;
				}
				for (int kk = 0; kk < 15; kk++)
				{
					for (int k = 0; k < 15; k++)
					{
						
						//Kal->Pk.dd[kk] += Kal->Pk1.dd[kk] * Kal->II.dd[k * Kal->II.clm + kk];
						Kal->Pk.dd[(Kal->iter - 11) * Kal->Pk.clm + kk] += Kal->Pk1.dd[(Kal->iter - 11) * Kal->Pk1.clm + k] * Kal->II.dd[kk * Kal->II.clm + k];
					}
					//Kal->Pk.dd[kk] += Kal->Qkf.dd[kk];
					Kal->Pk.dd[(Kal->iter - 11) * Kal->Pk.clm + kk] += Kal->Qkf.dd[(Kal->iter - 11) * Kal->Pk1.clm + kk];
					//Kal->Pk.dd[kk * Kal->Pk.clm + 0] = Kal->Pk.dd[kk];
					Kal->Pk.dd[kk * Kal->Pk.clm + (Kal->iter - 11)] = Kal->Pk.dd[(Kal->iter - 11) * Kal->Pk.clm + kk];
				}

			}
			else
			{
				for (int kk = Kal->iter - 11; kk < 15; kk++)//clm
				{
					for (int k = 0; k < 15; k++ )
					{
						Kal->Pk.dd[(Kal->iter-11)*Kal->Pk.clm + kk] += Kal->Pk1.dd[(Kal->iter - 11) * Kal->Pk1.clm + k] * Kal->II.dd[kk*Kal->II.clm + k];

					}
					Kal->Pk.dd[(Kal->iter - 11) * Kal->Pk.clm + kk] += Kal->Qkf.dd[(Kal->iter - 11) * Kal->Pk1.clm + kk];
					Kal->Pk.dd[kk* Kal->Pk.clm + (Kal->iter - 11)] = Kal->Pk.dd[(Kal->iter - 11) * Kal->Pk.clm + kk];
				}
				for (int k1 =(26-Kal->iter); k1<15; k1++)//clm
				{
					for (int k2 = 0; k2 < 15; k2++)
					{
						Kal->Pk.dd[(26 - Kal->iter) * Kal->Pk.clm + k1] += Kal->Pk1.dd[(26 - Kal->iter)* Kal->Pk1.clm + k2] * Kal->II.dd[k1*Kal->II.clm + k2];
					}
					Kal->Pk.dd[(26 - Kal->iter) * Kal->Pk.clm + k1] += Kal->Qkf.dd[(26 - Kal->iter) * Kal->Pk.clm + k1];
					Kal->Pk.dd[k1*Kal->Pk.clm + (26 - Kal->iter)] = Kal->Pk.dd[(26 - Kal->iter) * Kal->Pk.clm + k1];
				}
			}
		}
		else if(Kal->iter <= 27)//序贯滤波后验估计
		{
			if (GNSS->GNSSvalidity == true)//卫星有效
			{

				if (Kal->iter % 3 == 1)
				{
					for (int k = 0; k < 15; k++)
					{
						for (int k1 = 0; k1 < 15; k1++)
						{
							Kal->Pxrk.dd[k] += Kal->Pk.dd[k * 15 + k1] * Kal->H.dd[(Kal->iter - 19 - Kal->n * 2) * Kal->H.clm + k1];
						}
					}
					for (int k = 0; k < 15; k++)
					{
						Kal->nn += Kal->H.dd[(Kal->iter - 19 - Kal->n * 2) * Kal->H.clm + k] * Kal->Pxrk.dd[k];
					}
					Kal->nn += Kal->Rkf.dd[Kal->n];
					for (int k = 0; k < 15; k++)
					{
						Kal->K.dd[k] = Kal->Pxrk.dd[k] / Kal->nn;
					}
					Kal->nn = 0;
				}
				else if (Kal->iter % 3 == 2)
				{
					double err = 0;
					for (int k = 0; k < 15; k++)
					{
						err += Kal->H.dd[(Kal->iter - 20 - Kal->n * 2) * Kal->H.clm + k] * Kal->Xk.dd[k];
					}
					//err = (Kal->Zk.dd[(Kal->iter - 2 * Kal->n - 16)] - err);
					err = (Kal->Zk.dd[Kal->n] - err);
					for (int k = 0; k < 15; k++)
					{
						Kal->Xk.dd[k] += Kal->K.dd[k] * err;
					}
				}
				else
				{
					for (int k = 0; k < 15; k++)
					{
						for (int k1 = 0; k1 < 15; k1++)
						{
							Kal->Pk.dd[k * 15 + k1] -= Kal->K.dd[k] * Kal->Pxrk.dd[k1];
						}
					}
					Kal->n++;
					SetCV(0.0, &Kal->Pxrk);
				}
				if (Kal->iter == 27)//误差补偿
				{
					//状态补偿
					StateCompensation_TD(&Kal->Xk, &ins->Cnb, &ins->qnb, &eth->pos, &eth->vn, &ins->att);
					//归零

					for (int i = 0; i < 225; i++)
					{
						Kal->Pk1.dd[i] = 0;
						Kal->II.dd[i] = 0;
					}
					Kal->iter = -3;
					Kal->n = 0;
					GNSS->GNSSvalidity = false;
				}
			}
			else
			{
				//先验归零
				Kal->iter = -3;
				Kal->n = 0;
				for (int i = 0; i < 225; i++)
				{
					Kal->Pk1.dd[i] = 0;
					Kal->II.dd[i] = 0;
				}
			}
			
		}
		Kal->iter++;
	}
}

	

//****************Kalman 精对准********************//
void KalsetFk_D(struct Kalman* Kal, struct CVect3* P, double T)//T:惯导更新时间
{
	Kal->Maa.e00 = 0;
	Kal->Maa.e01 = T * wie * sin(P->i);
	Kal->Maa.e02 = -1 * T * wie * cos(P->i);

	Kal->Maa.e10 = -1 * Kal->Maa.e01;
	Kal->Maa.e11 = 0;
	Kal->Maa.e12 = 0;
	Kal->Maa.e20 = -1 * Kal->Maa.e02;
	Kal->Maa.e21 = 0;
	Kal->Maa.e22 = 0;

	Kal->Mvv.e01 = 2 * Kal->Maa.e01; Kal->Mvv.e00 = 0; Kal->Mvv.e02 = 2 * Kal->Maa.e02;
	Kal->Mvv.e10 = 2 * Kal->Maa.e10; Kal->Mvv.e11 = Kal->Mvv.e12 = 0;
	Kal->Mvv.e20 = 2 * Kal->Maa.e20; Kal->Mvv.e21 = Kal->Mvv.e22 = 0;
	double Rn = Re / sqrt(1 - e * e * sin(P->i) * sin(P->i));
	double Rm = Rn * (1 - e * e) / (1 - e * e * sin(P->i) * sin(P->i));
	Kal->Mav.e01 = -1 * T / (Rm + P->k); Kal->Mav.e00 = Kal->Mav.e02 = 0;
	Kal->Mav.e10 = T / (Rn + P->k); Kal->Mav.e11 = Kal->Mav.e12 = 0;
	Kal->Mav.e20 = tan(P->i) * T / (Rn + P->k); Kal->Mav.e21 = Kal->Mav.e22 = 0;
	double gg = g0 * (1.0 + b * sin(P->i) * sin(P->i) - b1 * sin(2 * P->i) * sin(2 * P->i)) - b2 * P->k;
	Kal->Mva.e01 = -gg * T; Kal->Mva.e00 = Kal->Mva.e02 = 0;
	Kal->Mva.e10 = gg * T; Kal->Mva.e11 = Kal->Mva.e12 = 0;
	Kal->Mva.e20 = Kal->Mva.e21 = Kal->Mva.e22 = 0;
	setCMAT3IntoCM(&Kal->Fk, &Kal->Maa, 1, 1);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mav, 1, 4);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mva, 4, 1);
	setCMAT3IntoCM(&Kal->Fk, &Kal->Mvv, 4, 4);  //此部分在静基座中为常数，对准还需补充惯性器件误差造成的误差，Cnb为时变
}

void IniKalman_D(struct Kalman* Kal, struct CVect3* pos, struct CVect3* Satt, struct CVect3* Svn, struct CVect3* Sgyro, struct CVect3* Sacc, struct CVect3* SWg, struct CVect3* SWa, double T)//姿态速度位置标准差、陀螺仪随机飘移，加表零偏，陀螺仪、加表随机游走 惯导更新时间
{
	Kal->Fk.row = Kal->Fk.clm = 12; Kal->Fk.rc = 144; clear(&Kal->Fk);
	KalsetFk_D(Kal, pos, T);
	Kal->II.row = Kal->II.clm = 12; Kal->II.rc = 144; clear(&Kal->II);
	Kal->Q.row = 6; Kal->Q.clm = 1; Kal->Q.rc = 6;
	Kal->Q.dd[0] = SWg->i * SWg->i * T; Kal->Q.dd[1] = SWg->j * SWg->j * T; Kal->Q.dd[2] = SWg->k * SWg->k * T;
	Kal->Q.dd[3] = SWa->i * SWa->i * T; Kal->Q.dd[4] = SWa->j * SWa->j * T; Kal->Q.dd[5] = SWa->k * SWa->k * T;
	Kal->Qkf.row = Kal->Qkf.clm = 12; Kal->Qkf.rc = 144; clear(&Kal->Qkf);
	Kal->Pk1.row = Kal->Pk1.clm = 12; Kal->Pk1.rc = 144; clear(&Kal->Pk1);
	Kal->Pk.row = Kal->Pk.clm = 12; Kal->Pk.rc = 144; clear(&Kal->Pk);
	double* p = &Kal->Pk.dd[0];
	*p = Satt->i * Satt->i; p = p + 13;
	*p = Satt->j * Satt->j; p = p + 13;
	*p = Satt->k * Satt->k; p = p + 13;
	*p = Svn->i * Svn->i; p = p + 13;
	*p = Svn->j * Svn->j; p = p + 13;
	*p = Svn->k * Svn->k; p = p + 13;
	*p = Sgyro->i * Sgyro->i; p = p + 13;
	*p = Sgyro->j * Sgyro->j; p = p + 13;
	*p = Sgyro->k * Sgyro->k; p = p + 13;
	*p = Sacc->i * Sacc->i; p = p + 13;
	*p = Sacc->j * Sacc->j; p = p + 13;
	*p = Sacc->k * Sacc->k;

	Kal->H.row = 3; Kal->H.clm = 12; Kal->H.rc = 36; clear(&Kal->H);
	Kal->H.dd[3] = 1; Kal->H.dd[16] = 1; Kal->H.dd[29] = 1;
	Kal->Xk.row = 12; Kal->Xk.clm = 1; Kal->Xk.rc = 12; SetCV(0.0, &Kal->Xk);
	Kal->Zk.row = 3; Kal->Zk.clm = 1; Kal->Zk.rc = 3; SetCV(0.0, &Kal->Zk);  //后续赋值
	Kal->Rkf.row = 3; Kal->Rkf.clm = 1; Kal->Rkf.rc = 3; Kal->Rkf.dd[0] = Svn->i * Svn->i; Kal->Rkf.dd[1] = Svn->j * Svn->j; Kal->Rkf.dd[2] = Svn->k * Svn->k;
	Kal->K.row = Kal->Pxrk.row = 12; Kal->K.clm = Kal->Pxrk.clm = 1; Kal->K.rc = Kal->Pxrk.rc = 12; SetCV(0.0, &Kal->K); SetCV(0.0, &Kal->Pxrk);
	Kal->nq = 12; Kal->n = 0; Kal->nn = 0; Kal->iter = -1; Kal->nstep = 2;
}


//void Kal_TDupdate_D(struct Kalman* Kal, struct CSINS* ins, struct CEarth* eth, struct CIMU* cimu, struct Calibration* Cal, double* pwm, double* pvm)//时间分片滤波精对准
//{
//	INSUpdate(ins, eth, cimu, pwm, pvm);
//	for (int i = 0; i < Kal->nstep; i++)
//	{
//		if (Kal->iter <= -1)
//		{
//			Kal->Fk.dd[6] = -1 * ins->Cnb.e00 * ins->nts; Kal->Fk.dd[7] = -1 * ins->Cnb.e01 * ins->nts; Kal->Fk.dd[8] = -1 * ins->Cnb.e02 * ins->nts;
//			Kal->Fk.dd[18] = -1 * ins->Cnb.e10 * ins->nts; Kal->Fk.dd[19] = -1 * ins->Cnb.e11 * ins->nts; Kal->Fk.dd[20] = -1 * ins->Cnb.e12 * ins->nts;
//			Kal->Fk.dd[30] = -1 * ins->Cnb.e20 * ins->nts; Kal->Fk.dd[31] = -1 * ins->Cnb.e21 * ins->nts; Kal->Fk.dd[32] = -1 * ins->Cnb.e22 * ins->nts;

//			Kal->Fk.dd[45] = ins->Cnb.e00 * ins->nts; Kal->Fk.dd[46] = ins->Cnb.e01 * ins->nts; Kal->Fk.dd[47] = ins->Cnb.e02 * ins->nts;
//			Kal->Fk.dd[57] = ins->Cnb.e10 * ins->nts; Kal->Fk.dd[58] = ins->Cnb.e11 * ins->nts; Kal->Fk.dd[59] = ins->Cnb.e12 * ins->nts;
//			Kal->Fk.dd[69] = ins->Cnb.e20 * ins->nts; Kal->Fk.dd[70] = ins->Cnb.e21 * ins->nts; Kal->Fk.dd[71] = ins->Cnb.e22 * ins->nts;
//			//观测
//			Kal->Zk.dd[0] = eth->vn.i;
//			Kal->Zk.dd[1] = eth->vn.j;
//			Kal->Zk.dd[2] = eth->vn.k;
//			//Q离散化
//			Kal->Qkf.dd[0] = ins->Cnb.e00 * Kal->Q.dd[0] * ins->Cnb.e00 + ins->Cnb.e01 * Kal->Q.dd[1] * ins->Cnb.e01 + ins->Cnb.e02 * Kal->Q.dd[2] * ins->Cnb.e02; Kal->Qkf.dd[0]; Kal->Qkf.dd[0] = Kal->Qkf.dd[0] * ins->x;
//			Kal->Qkf.dd[1] = ins->Cnb.e00 * Kal->Q.dd[0] * ins->Cnb.e10 + ins->Cnb.e01 * Kal->Q.dd[1] * ins->Cnb.e11 + ins->Cnb.e02 * Kal->Q.dd[2] * ins->Cnb.e12; Kal->Qkf.dd[1]; Kal->Qkf.dd[1] = Kal->Qkf.dd[1] * ins->x;
//			Kal->Qkf.dd[2] = ins->Cnb.e00 * Kal->Q.dd[0] * ins->Cnb.e20 + ins->Cnb.e01 * Kal->Q.dd[1] * ins->Cnb.e21 + ins->Cnb.e02 * Kal->Q.dd[2] * ins->Cnb.e22; Kal->Qkf.dd[2]; Kal->Qkf.dd[2] = Kal->Qkf.dd[2] * ins->x;
//			Kal->Qkf.dd[12] = ins->Cnb.e10 * Kal->Q.dd[0] * ins->Cnb.e00 + ins->Cnb.e11 * Kal->Q.dd[1] * ins->Cnb.e01 + ins->Cnb.e12 * Kal->Q.dd[2] * ins->Cnb.e02; Kal->Qkf.dd[12]; Kal->Qkf.dd[12] = Kal->Qkf.dd[12] * ins->x;
//			Kal->Qkf.dd[13] = ins->Cnb.e10 * Kal->Q.dd[0] * ins->Cnb.e10 + ins->Cnb.e11 * Kal->Q.dd[1] * ins->Cnb.e11 + ins->Cnb.e12 * Kal->Q.dd[2] * ins->Cnb.e12; Kal->Qkf.dd[13]; Kal->Qkf.dd[13] = Kal->Qkf.dd[13] * ins->x;
//			Kal->Qkf.dd[14] = ins->Cnb.e10 * Kal->Q.dd[0] * ins->Cnb.e20 + ins->Cnb.e11 * Kal->Q.dd[1] * ins->Cnb.e21 + ins->Cnb.e12 * Kal->Q.dd[2] * ins->Cnb.e22; Kal->Qkf.dd[14]; Kal->Qkf.dd[14] = Kal->Qkf.dd[14] * ins->x;
//			Kal->Qkf.dd[24] = ins->Cnb.e20 * Kal->Q.dd[0] * ins->Cnb.e00 + ins->Cnb.e21 * Kal->Q.dd[1] * ins->Cnb.e01 + ins->Cnb.e22 * Kal->Q.dd[2] * ins->Cnb.e02; Kal->Qkf.dd[24]; Kal->Qkf.dd[24] = Kal->Qkf.dd[24] * ins->x;
//			Kal->Qkf.dd[25] = ins->Cnb.e20 * Kal->Q.dd[0] * ins->Cnb.e10 + ins->Cnb.e21 * Kal->Q.dd[1] * ins->Cnb.e11 + ins->Cnb.e22 * Kal->Q.dd[2] * ins->Cnb.e12; Kal->Qkf.dd[25]; Kal->Qkf.dd[25] = Kal->Qkf.dd[25] * ins->x;
//			Kal->Qkf.dd[26] = ins->Cnb.e20 * Kal->Q.dd[0] * ins->Cnb.e20 + ins->Cnb.e21 * Kal->Q.dd[1] * ins->Cnb.e21 + ins->Cnb.e22 * Kal->Q.dd[2] * ins->Cnb.e22; Kal->Qkf.dd[26]; Kal->Qkf.dd[26] = Kal->Qkf.dd[26] * ins->x;

//			Kal->Qkf.dd[39] = ins->Cnb.e00 * Kal->Q.dd[3] * ins->Cnb.e00 + ins->Cnb.e01 * Kal->Q.dd[4] * ins->Cnb.e01 + ins->Cnb.e02 * Kal->Q.dd[5] * ins->Cnb.e02; Kal->Qkf.dd[39]; Kal->Qkf.dd[39] = Kal->Qkf.dd[39] * ins->x;
//			Kal->Qkf.dd[40] = ins->Cnb.e00 * Kal->Q.dd[3] * ins->Cnb.e10 + ins->Cnb.e01 * Kal->Q.dd[4] * ins->Cnb.e11 + ins->Cnb.e02 * Kal->Q.dd[5] * ins->Cnb.e12; Kal->Qkf.dd[40]; Kal->Qkf.dd[40] = Kal->Qkf.dd[40] * ins->x;
//			Kal->Qkf.dd[41] = ins->Cnb.e00 * Kal->Q.dd[3] * ins->Cnb.e20 + ins->Cnb.e01 * Kal->Q.dd[4] * ins->Cnb.e21 + ins->Cnb.e02 * Kal->Q.dd[5] * ins->Cnb.e22; Kal->Qkf.dd[41]; Kal->Qkf.dd[41] = Kal->Qkf.dd[41] * ins->x;
//			Kal->Qkf.dd[51] = ins->Cnb.e10 * Kal->Q.dd[3] * ins->Cnb.e00 + ins->Cnb.e11 * Kal->Q.dd[4] * ins->Cnb.e01 + ins->Cnb.e12 * Kal->Q.dd[5] * ins->Cnb.e02; Kal->Qkf.dd[51]; Kal->Qkf.dd[51] = Kal->Qkf.dd[51] * ins->x;
//			Kal->Qkf.dd[52] = ins->Cnb.e10 * Kal->Q.dd[3] * ins->Cnb.e10 + ins->Cnb.e11 * Kal->Q.dd[4] * ins->Cnb.e11 + ins->Cnb.e12 * Kal->Q.dd[5] * ins->Cnb.e12; Kal->Qkf.dd[52]; Kal->Qkf.dd[52] = Kal->Qkf.dd[52] * ins->x;
//			Kal->Qkf.dd[53] = ins->Cnb.e10 * Kal->Q.dd[3] * ins->Cnb.e20 + ins->Cnb.e11 * Kal->Q.dd[4] * ins->Cnb.e21 + ins->Cnb.e12 * Kal->Q.dd[5] * ins->Cnb.e22; Kal->Qkf.dd[53]; Kal->Qkf.dd[53] = Kal->Qkf.dd[53] * ins->x;
//			Kal->Qkf.dd[63] = ins->Cnb.e20 * Kal->Q.dd[3] * ins->Cnb.e00 + ins->Cnb.e21 * Kal->Q.dd[4] * ins->Cnb.e01 + ins->Cnb.e22 * Kal->Q.dd[5] * ins->Cnb.e02; Kal->Qkf.dd[63]; Kal->Qkf.dd[63] = Kal->Qkf.dd[63] * ins->x;
//			Kal->Qkf.dd[64] = ins->Cnb.e20 * Kal->Q.dd[3] * ins->Cnb.e10 + ins->Cnb.e21 * Kal->Q.dd[4] * ins->Cnb.e11 + ins->Cnb.e22 * Kal->Q.dd[5] * ins->Cnb.e12; Kal->Qkf.dd[64]; Kal->Qkf.dd[64] = Kal->Qkf.dd[64] * ins->x;
//			Kal->Qkf.dd[65] = ins->Cnb.e20 * Kal->Q.dd[3] * ins->Cnb.e20 + ins->Cnb.e21 * Kal->Q.dd[4] * ins->Cnb.e21 + ins->Cnb.e22 * Kal->Q.dd[5] * ins->Cnb.e22; Kal->Qkf.dd[65]; Kal->Qkf.dd[65] = Kal->Qkf.dd[65] * ins->x;
//			//离散化
//			Kaldiscretizing_D(&Kal->Fk, &Kal->II, ins->x, 6); //状态转移矩阵 惯导执行次数 转移矩阵非0维度数
//			ins->x = 0;
//		}
//		else if (Kal->iter <= 0)
//		{
//			// 先验估计
//			Kal->Xk = *CMtmulCV(&Kal->II, &Kal->Xk, &Kal->Xk);
//		}
//		else if (Kal->iter <= 7)//II*PK
//		{
//			if (Kal->iter <= 6)
//			{
//				for (int j = 0; j < 12; j++)
//				{
//					for (int jj = 0; jj < 12; jj++)
//					{
//						Kal->Pk1.dd[(Kal->iter - 1) * Kal->Pk1.clm + j] += Kal->II.dd[(Kal->iter - 1) * Kal->II.clm + jj] * Kal->Pk.dd[jj * Kal->Pk.clm + j];
//					}
//				}
//			}
//			else
//			{
//				for (int j = 0; j < 12; j++)
//				{
//					Kal->Pk1.dd[(Kal->iter - 1) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter - 1) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter - 1) * Kal->Pk1.clm + j];
//					Kal->Pk1.dd[Kal->iter * Kal->Pk1.clm + j] = Kal->II.dd[Kal->iter * (Kal->II.clm + 1)] * Kal->Pk.dd[Kal->iter * Kal->Pk1.clm + j];
//					Kal->Pk1.dd[(Kal->iter + 1) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 1) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 1) * Kal->Pk1.clm + j];
//					Kal->Pk1.dd[(Kal->iter + 2) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 2) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 2) * Kal->Pk1.clm + j];
//					Kal->Pk1.dd[(Kal->iter + 3) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 3) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 3) * Kal->Pk1.clm + j];
//					Kal->Pk1.dd[(Kal->iter + 4) * Kal->Pk1.clm + j] = Kal->II.dd[(Kal->iter + 4) * (Kal->II.clm + 1)] * Kal->Pk.dd[(Kal->iter + 4) * Kal->Pk1.clm + j];
//				}
//			}
//		}
//		else if (Kal->iter <= 14)//II*Pk*II+Qk
//		{
//			for (int j1 = 0; j1 < 12; j1++)
//			{
//				double PP = 0;
//				//if ((j1 < Kal->iter - 8) && (Kal->iter != 14))
//				if ((j1 < Kal->iter - 8) && (Kal->iter < 14))
//				{
//					for (int j2 = 0; j2 < 12; j2++)
//					{
//						//Kal->Pk.dd[(20 - Kal->iter) * Kal->Pk.clm + 11 - j1] += Kal->Pk1.dd[(20 - Kal->iter) * Kal->Pk1.clm + j2] * Kal->II.dd[(11 - j1) * 12 + j2];
//						PP += Kal->Pk1.dd[(20 - Kal->iter) * Kal->Pk1.clm + j2] * Kal->II.dd[(11 - j1) * 12 + j2];
//					}
//					Kal->Pk.dd[(20 - Kal->iter) * Kal->Pk.clm + 11 - j1] = PP + Kal->Qkf.dd[(20 - Kal->iter) * Kal->Qkf.clm + 11 - j1];
//					Kal->Pk.dd[(11 - j1) * Kal->Pk.clm + 20 - Kal->iter] = Kal->Pk.dd[(20 - Kal->iter) * Kal->Pk.clm + 11 - j1];
//				}
//				else
//				{
//					for (int j2 = 0; j2 < 12; j2++)
//					{
//						//Kal->Pk.dd[(Kal->iter - 8) * Kal->Pk.clm + j1] += Kal->Pk1.dd[(Kal->iter - 8) * Kal->Pk.clm + j2] * Kal->II.dd[j1 * Kal->Pk.clm + j2];
//						PP += Kal->Pk1.dd[(Kal->iter - 8) * Kal->Pk.clm + j2] * Kal->II.dd[j1 * Kal->Pk.clm + j2];

//					}
//					Kal->Pk.dd[(Kal->iter - 8) * Kal->Pk.clm + j1] = PP + Kal->Qkf.dd[(Kal->iter - 8) * Kal->Qkf.clm + j1];
//					Kal->Pk.dd[j1 * 12 + (Kal->iter - 8)] = Kal->Pk.dd[(Kal->iter - 8) * Kal->Pk.clm + j1];
//				}
//			}
//		}
//		else if (Kal->iter <= 23)//序贯滤波
//		{
//			if (Kal->iter % 3 == 0)
//			{
//				for (int k = 0; k < 12; k++)
//				{
//					for (int k1 = 0; k1 < 12; k1++)
//					{
//						Kal->Pxrk.dd[k] += Kal->Pk.dd[k * 12 + k1] * Kal->H.dd[(Kal->iter - 15 - Kal->n * 2) * Kal->H.clm + k1];
//					}
//				}
//				for (int k = 0; k < 12; k++)
//				{
//					Kal->nn += Kal->H.dd[(Kal->iter - 15 - Kal->n * 2) * Kal->H.clm + k] * Kal->Pxrk.dd[k];
//				}
//				Kal->nn += Kal->Rkf.dd[Kal->n];
//				for (int k = 0; k < 12; k++)
//				{
//					Kal->K.dd[k] = Kal->Pxrk.dd[k] / Kal->nn;
//				}
//				Kal->nn = 0;
//			}
//			else if (Kal->iter % 3 == 1)
//			{
//				double err = 0;
//				for (int k = 0; k < 12; k++)
//				{
//					err += Kal->H.dd[(Kal->iter - 16 - Kal->n * 2) * Kal->H.clm + k] * Kal->Xk.dd[k];
//				}
//				//err = (Kal->Zk.dd[(Kal->iter - 2 * Kal->n - 16)] - err);
//				err = (Kal->Zk.dd[Kal->n] - err);
//				for (int k = 0; k < 12; k++)
//				{
//					Kal->Xk.dd[k] += Kal->K.dd[k] * err;
//				}
//			}
//			else
//			{
//				for (int k = 0; k < 12; k++)
//				{
//					for (int k1 = 0; k1 < 12; k1++)
//					{
//						Kal->Pk.dd[k * 12 + k1] -= Kal->K.dd[k] * Kal->Pxrk.dd[k1];
//					}
//				}
//				Kal->n++;
//				SetCV(0.0, &Kal->Pxrk);
//			}
//		}
//		else if (Kal->iter <= 24)
//		{
//			//         eth->vn.i=eth->vn.i-Kal->Xk.dd[3];
//					 //eth->vn.j=eth->vn.j-Kal->Xk.dd[4];
//					 //eth->vn.k=eth->vn.k-Kal->Xk.dd[5];
//					 //struct CVect3 ab = {Kal->Xk.dd[0],Kal->Xk.dd[1],Kal->Xk.dd[2]};
//					 //struct Quat qq;
//			//         qq=*rv2q(&ab,&qq);        
//			//         ins->qnb=*QuatmulQu(&qq, &ins->qnb, &ins->qnb);
//			//         ins->Cnb=*q2mat(&ins->qnb, &ins->Cnb);     
//			//         ins->att=*m2att(&ins->Cnb, &ins->att);
//					 //Kal->Xk.dd[0]=0;Kal->Xk.dd[1]=0;Kal->Xk.dd[2]=0;
//					 //Kal->Xk.dd[3]=0;Kal->Xk.dd[4]=0;Kal->Xk.dd[5]=0;

//			Kal->iter = -2;
//			Kal->n = 0;
//			for (int i = 0; i < 144; i++)
//			{
//				Kal->Pk1.dd[i] = 0;
//				Kal->II.dd[i] = 0;
//			}
//		}
//		Kal->iter++;
//	}
//}

void StateCompensation_D(struct CVect* Xk, struct CMAT3* Cnb, struct Quat* q, struct CVect3* vn, struct CVect3* att)//12维对准结果补偿
{
	vn->i -= Xk->dd[3];
	vn->j -= Xk->dd[4];
	vn->k -= Xk->dd[5];
	struct CVect3 ab = { Xk->dd[0],Xk->dd[1],Xk->dd[2] };
	struct Quat qq;
	qq = *rv2q(&ab, &qq);
	q = QuatmulQu(&qq, q, q);
	Cnb = q2mat(q, Cnb);
	att = m2att(Cnb, att);
}

//*****************对准算法**************************
void IniICA(struct ICA* ica, struct CVect3* pos0, double T, int s, double t)//粗对准结构体初始化 ica粗对准结构体 pos位置 gn重力 T粗对准时间 s字样数  采样时间
{
	ica->tk0 = 0;
	ica->T2 = T;
	ica->T1 = 0.5 * ica->T2;            //对准双矢量时间选取
	ica->Cne = O33; ica->Cne.e01 = 1; ica->Cne.e10 = -sin(pos0->i); ica->Cne.e12 = cos(pos0->i); ica->Cne.e20 = cos(pos0->i); ica->Cne.e22 = sin(pos0->i);
	ica->Cei = I33; ica->Cei.e00 = cos(wie * ica->T2); ica->Cei.e01 = sin(wie * ica->T2); ica->Cei.e10 = -sin(wie * ica->T2); ica->Cei.e11 = cos(wie * ica->T2);
	ica->Vi1 = O31; ica->Vi2 = O31;
	struct CVect3 gn = { 0,0,0 }; gn.k = -g0 * (1.0 + b * sin(pos0->i) * sin(pos0->i) - b1 * sin(2 * pos0->i) * sin(2 * pos0->i)) - b2 * pos0->k;
	ica->Vi1.i = -1 * gn.k * cos(pos0->i) * sin(wie * ica->T1) / wie;
	ica->Vi1.j = -1 * gn.k * cos(pos0->i) * (1 - cos(wie * ica->T1)) / wie;
	ica->Vi1.k = -1 * gn.k * sin(pos0->i) * ica->T1;
	ica->Vi2.i = -1 * gn.k * cos(pos0->i) * sin(wie * ica->T2) / wie;
	ica->Vi2.j = -1 * gn.k * cos(pos0->i) * (1 - cos(wie * ica->T2)) / wie;
	ica->Vi2.k = -1 * gn.k * sin(pos0->i) * ica->T2;
	ica->Cnb = ica->Cib = I33;
	ica->att0 = ica->Vib1 = ica->Vib2 = O31;   //初始姿态角
	ica->qib = *m2qua(&ica->Cib, &ica->qib); //四元数
	ica->qnb = *m2qua(&ica->Cnb, &ica->qnb);
	ica->Cii = O33;
	IniCIMU(&ica->cimu, t, s);
	ica->alignfinish = false;
	ica->V1flag = false;
}

//静基座解析式对准初始化
void IniSTICA(struct STICA* ica, struct CVect3* pos)//结构体 位置信息 惯导更新时间间隔 对准时间
{
	ica->t0 = 0; ica->n = 0;
	ica->A1.i = 0; ica->A1.j = 0; ica->A1.k = 0;
	ica->G1.i = 0; ica->G1.j = 0; ica->G1.k = 0;
	struct CVect3 gn;
	gn.i = 0;
	gn.j = 0;
	gn.k = g0 * (1.0 + b * sin(pos->i) * sin(pos->i) - b1 * sin(2 * pos->i) * sin(2 * pos->i)) - b2 * pos->k;//重力加速度计算
	ica->N1 = gn;
	struct CVect3 wn;
	wn.i = 0; wn.j = wie * cos(pos->i); wn.k = wie * sin(pos->i);
	ica->W1 = wn;
	ica->att0.i = 0; ica->att0.j = 0; ica->att0.k = 0;
	ica->Cnb = I33;
	ica->alignfinish = false;
}

void Twovecmea(struct CMAT3* Cnb, struct CVect3* V1, struct CVect3* V2, struct CVect3* G1, struct CVect3* G2)//双矢量求转换矩阵 V为目标系理论值，G为载体系实测值
{
	struct CVect3 A1, A2, A3, N1, N2, N3;
	A1 = *V1; A2 = *operatormul3(V1, V2, &A2); A3 = *operatormul3(&A2, V1, &A3);
	N1 = *G1; N2 = *operatormul3(G1, G2, &N2); N3 = *operatormul3(&N2, G1, &N3);//正交化
	double a1 = norm(&A1); double a2 = norm(&A2); double a3 = norm(&A3);
	double n1 = norm(&N1); double n2 = norm(&N2); double n3 = norm(&N3);
	A1 = *operatordiv(&A1, a1); A2 = *operatordiv(&A2, a2); A3 = *operatordiv(&A3, a3);
	N1 = *operatordiv(&N1, n1); N2 = *operatordiv(&N2, n2); N3 = *operatordiv(&N3, n3);//单位化
	Cnb->e00 = A1.i * N1.i + A2.i * N2.i + A3.i * N3.i;
	Cnb->e01 = A1.i * N1.j + A2.i * N2.j + A3.i * N3.j;
	Cnb->e02 = A1.i * N1.k + A2.i * N2.k + A3.i * N3.k;
	Cnb->e10 = A1.j * N1.i + A2.j * N2.i + A3.j * N3.i;
	Cnb->e11 = A1.j * N1.j + A2.j * N2.j + A3.j * N3.j;
	Cnb->e12 = A1.j * N1.k + A2.j * N2.k + A3.j * N3.k;
	Cnb->e20 = A1.k * N1.i + A2.k * N2.i + A3.k * N3.i;
	Cnb->e21 = A1.k * N1.j + A2.k * N2.j + A3.k * N3.j;
	Cnb->e22 = A1.k * N1.k + A2.k * N2.k + A3.k * N3.k;   //双矢量定姿
}
//解析式对准
void CoarAlign0(struct STICA* ica, double* pwm, double* pvm, double  t, double T)//T对准时间 t采样时间间隔
{
	//求和
	ica->G1.i += *(pwm + 0); ica->A1.i += *(pvm + 0);
	ica->G1.j += *(pwm + 1); ica->A1.j += *(pvm + 1);
	ica->G1.k += *(pwm + 2); ica->A1.k += *(pvm + 2);
	ica->t0 = ica->t0 + t;
	ica->n += 1.0;
	if (ica->t0 >= T)
	{
		ica->G1.i = ica->G1.i / ica->n; ica->G1.j = ica->G1.j / ica->n; ica->G1.k = ica->G1.k / ica->n;
		ica->A1.i = ica->A1.i / ica->n; ica->A1.j = ica->A1.j / ica->n; ica->A1.k = ica->A1.k / ica->n;
		Twovecmea(&ica->Cnb, &ica->N1, &ica->W1, &ica->A1, &ica->G1);
		ica->att0 = *m2att(&ica->Cnb, &ica->att0);
		ica->alignfinish = true;
	}
}

//惯性系粗对准
void CoarAlign(struct ICA* ica, struct Calibration* Cal, double* pwm, double* pvm, double T)//粗?对准的结构体 标定后的角/速度数据 惯导更新时间
{
	CIMUUpdate(&ica->cimu, pwm, pvm);
	struct Quat q;
	q = *rv2q(&ica->cimu.phim, &q);
	ica->qib = *QuatmulQu(&ica->qib, &q, &ica->qib);
	ica->Cib = *q2mat(&ica->qib, &ica->Cib);
	struct CVect3 Vib = *CM3mulCV3(&ica->Cib, &ica->cimu.dvbm, &Vib);
	ica->Vib2 = *CV3add(&ica->Vib2, &Vib);
	ica->tk0 = ica->tk0 + T;//时间更新
	if (ica->tk0 >= ica->T1 && ica->V1flag == false)
	{
		ica->Vib1 = ica->Vib2;
		ica->V1flag = true;
	}
	if (ica->tk0 >= ica->T2)
	{
		Twovecmea(&ica->Cii, &ica->Vi1, &ica->Vi2, &ica->Vib1, &ica->Vib2);
		ica->Cnb = *operatorCM3mul(&ica->Cne, &ica->Cei, &ica->Cnb);
		ica->Cnb = *operatorCM3mul(&ica->Cnb, &ica->Cii, &ica->Cnb);
		ica->Cnb = *operatorCM3mul(&ica->Cnb, &ica->Cib, &ica->Cnb);
		ica->qnb = *m2qua(&ica->Cnb, &ica->qnb);
		ica->att0 = *m2att(&ica->Cnb, &ica->att0);
		ica->alignfinish = true;
	}
}

void IniPIICA(struct PIICA* ica, struct CVect3* P)//参数辨识对准结果体初始化
{
	ica->PkE.e00 = 100; ica->PkE.e01 = 0; ica->PkE.e02 = 0;
	ica->PkE.e10 = 0; ica->PkE.e11 = 100; ica->PkE.e12 = 0;
	ica->PkE.e20 = 0; ica->PkE.e21 = 0; ica->PkE.e22 = 100;
	ica->PkN.e00 = 100; ica->PkN.e01 = 0; ica->PkN.e02 = 0;
	ica->PkN.e10 = 0; ica->PkN.e11 = 100; ica->PkN.e12 = 0;
	ica->PkN.e20 = 0; ica->PkN.e21 = 0; ica->PkN.e22 = 100;
	ica->Xk.i = 0; ica->Xk.j = 0; ica->Xk.k = 0;
	ica->Vn.i = 0; ica->Vn.j = 0; ica->Vn.k = 0;
	ica->KE.i = 0; ica->KE.j = 0; ica->KE.k = 0;
	ica->KN.i = 0; ica->KN.j = 0; ica->KN.k = 0;
	ica->ThetaE.i = ica->ThetaE.j = ica->ThetaE.k = 0;
	ica->ThetaN.i = ica->ThetaN.j = ica->ThetaN.k = 0;
	ica->KmulX.e00 = 0; ica->KmulX.e01 = 0; ica->KmulX.e02 = 0;
	ica->KmulX.e10 = 0; ica->KmulX.e11 = 0; ica->KmulX.e12 = 0;
	ica->KmulX.e20 = 0; ica->KmulX.e21 = 0; ica->KmulX.e22 = 0;
	ica->U.i = 0; ica->U.j = 0; ica->U.k = 0;
	ica->Phi.i = 0; ica->Phi.j = 0; ica->Phi.k = 0;
	ica->YkE = 0; ica->YkN = 0;
	ica->count_num = 0;
	ica->Lat = P->i;
	ica->g = g0 * (1 + 0.00530240 * sin(ica->Lat) * sin(ica->Lat) - 0.00000582 * sin(2 * ica->Lat) * sin(2 * ica->Lat));
	ica->alignfinish = false;
}

void ParameterIdentifie(struct PIICA* ica, struct CMAT3* CnbC, struct CVect3* Vb, double t)//递推最小二乘法计算
{
	ica->count_num++;
	//计算观测量
	double T = ica->count_num * t;
	ica->Xk.i = T; ica->Xk.j = T * T; ica->Xk.k = T * T * T;
	ica->YkE = ica->YkE + (CnbC->e00 * Vb->i + CnbC->e01 * Vb->j + CnbC->e02 * Vb->k);
	ica->YkN = ica->YkN + (CnbC->e10 * Vb->i + CnbC->e11 * Vb->j + CnbC->e12 * Vb->k);

	//计算增益
	double aE = (ica->Xk.i * ica->PkE.e00 + ica->Xk.j * ica->PkE.e10 + ica->Xk.k * ica->PkE.e20) * ica->Xk.i + (ica->Xk.i * ica->PkE.e01 + ica->Xk.j * ica->PkE.e11 + ica->Xk.k * ica->PkE.e21) * ica->Xk.j + (ica->Xk.i * ica->PkE.e02 + ica->Xk.j * ica->PkE.e12 + ica->Xk.k * ica->PkE.e22) * ica->Xk.k;
	double aN = (ica->Xk.i * ica->PkN.e00 + ica->Xk.j * ica->PkN.e10 + ica->Xk.k * ica->PkN.e20) * ica->Xk.i + (ica->Xk.i * ica->PkN.e01 + ica->Xk.j * ica->PkN.e11 + ica->Xk.k * ica->PkN.e21) * ica->Xk.j + (ica->Xk.i * ica->PkN.e02 + ica->Xk.j * ica->PkN.e12 + ica->Xk.k * ica->PkN.e22) * ica->Xk.k;
	aE = aE + 1; aN = aN + 1;
	ica->KE.i = ica->PkE.e00 * ica->Xk.i + ica->PkE.e01 * ica->Xk.j + ica->PkE.e02 * ica->Xk.k; ica->KE.i = ica->KE.i / aE;
	ica->KE.j = ica->PkE.e10 * ica->Xk.i + ica->PkE.e11 * ica->Xk.j + ica->PkE.e12 * ica->Xk.k; ica->KE.j = ica->KE.j / aE;
	ica->KE.k = ica->PkE.e20 * ica->Xk.i + ica->PkE.e21 * ica->Xk.j + ica->PkE.e22 * ica->Xk.k; ica->KE.k = ica->KE.k / aE;
	ica->KN.i = ica->PkN.e00 * ica->Xk.i + ica->PkN.e01 * ica->Xk.j + ica->PkN.e02 * ica->Xk.k; ica->KN.i = ica->KN.i / aE;
	ica->KN.j = ica->PkN.e10 * ica->Xk.i + ica->PkN.e11 * ica->Xk.j + ica->PkN.e12 * ica->Xk.k; ica->KN.j = ica->KN.j / aE;
	ica->KN.k = ica->PkN.e20 * ica->Xk.i + ica->PkN.e21 * ica->Xk.j + ica->PkN.e22 * ica->Xk.k; ica->KN.k = ica->KN.k / aE;

	//误差协方差计算
	//PkE = PkE - KKE * Xk * PkE;
	double* p = &ica->KmulX.e00;
	double* p1 = &ica->KE.i;
	double* p2 = &ica->Xk.i;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			*(p + 3 * i + j) = *(p1 + i) * *(p2 + j);
			*(p + 3 * i + j) = -1 * *(p + 3 * i + j);
		}
	}
	ica->KmulX.e00 = ica->KmulX.e00 + 1;
	ica->KmulX.e11 = ica->KmulX.e11 + 1;
	ica->KmulX.e22 = ica->KmulX.e22 + 1;
	ica->PkE = *operatorCM3mul(&ica->KmulX, &ica->PkE, &ica->PkE);
	//PkN = PkN - KKN * Xk * PkN;
	p = &ica->KmulX.e00;
	p1 = &ica->KN.i;
	p2 = &ica->Xk.i;
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			*(p + 3 * i + j) = *(p1 + i) * *(p2 + j);
			*(p + 3 * i + j) = -1 * *(p + 3 * i + j);
		}
	}
	ica->KmulX.e00 = ica->KmulX.e00 + 1;
	ica->KmulX.e11 = ica->KmulX.e11 + 1;
	ica->KmulX.e22 = ica->KmulX.e22 + 1;
	ica->PkN = *operatorCM3mul(&ica->KmulX, &ica->PkN, &ica->PkN);
	// 参数辨识
	//TheE = TheE + KKE * (YkE - Xk * TheE);
	double errE = ica->YkE - ica->Xk.i * ica->ThetaE.i - ica->Xk.j * ica->ThetaE.j - ica->Xk.k * ica->ThetaE.k;
	errE = ica->YkE - errE;
	ica->ThetaE.i += ica->KE.i * errE;
	ica->ThetaE.j += ica->KE.j * errE;
	ica->ThetaE.k += ica->KE.k * errE;
	//TheN = TheN + KKN * (YkN - Xk * TheN);
	double errN = ica->YkN - ica->Xk.i * ica->ThetaN.i - ica->Xk.j * ica->ThetaN.j - ica->Xk.k * ica->ThetaN.k;
	errN = ica->YkN - errN;
	ica->ThetaN.i += ica->KN.i * errN;
	ica->ThetaN.j += ica->KN.j * errN;
	ica->ThetaN.k += ica->KN.k * errN;
}

void PIICA_D(struct PIICA* ica, struct CSINS* ins, struct CIMU* cimu, struct Calibration* Cal, struct CMAT3* CnbC, struct Quat* QC, double* pvm, double* pwm, double t)//参数辨识法精对准程序 粗对准转换矩阵 增量形式数据，惯导采样时间, 对准时间，子样数
{


	struct CVect3 Vb = { *pvm,*(pvm + 1),*(pvm + 2) };
	Vb.i = Vb.i * t; Vb.j = Vb.j * t; Vb.k = Vb.k * t;
	ParameterIdentifie(ica, CnbC, &Vb, t);
	//参数分离
	ica->U.i = 2 * ica->ThetaN.j / ica->g;
	ica->U.j = -2 * ica->ThetaE.j / ica->g;
	ica->U.k = -(6 * ica->ThetaN.k / (ica->g * wie * cos(ica->Lat))) - (2 * ica->ThetaE.j * tan(ica->Lat) / ica->g);
	ica->Phi.i = ica->ThetaN.i / ica->g;
	ica->Phi.j = -1 * ica->ThetaE.i / ica->g;
	ica->Phi.k = ica->Phi.j * tan(ica->Lat) - ica->U.i / (wie * cos(ica->Lat));
	//失准角计算
	ica->atterr.i = ica->Phi.i + ica->U.i * ica->Xk.i + 0.5 * ica->Xk.j * wie * (ica->U.j * sin(ica->Lat) - ica->U.k * cos(ica->Lat));
	ica->atterr.j = ica->Phi.j + ica->U.j * ica->Xk.i - 0.5 * ica->Xk.j * wie * ica->U.i * sin(ica->Lat);
	ica->atterr.k = ica->Phi.k + ica->U.k * ica->Xk.i + 0.5 * ica->Xk.j * wie * ica->U.i * sin(ica->Lat);
	//姿态补偿
	struct Quat qq;
	qq = *rv2q(&ica->atterr, &qq);
	ins->qnb = *QuatmulQu(&qq, QC, &ins->qnb);
	ins->Cnb = *q2mat(&ins->qnb, &ins->Cnb);
	ins->att = *m2att(&ins->Cnb, &ins->att);
}
