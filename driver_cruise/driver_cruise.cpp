/*
	 WARNING !

	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "class_Visualization.h"
#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>

#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);
bool judgeDesert();

// Module Entry Point
extern "C" int driver_cruise(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}


/*
	 WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

//**********Global variables for vehicle states*********//
static float _midline[200][2];							//
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;//
static int _gearbox;									//
int Time = 0;
//******************************************************//


bool parameterSet = false;								//
void PIDParamSetter();									//


//******************************************************//
typedef struct Circle									//
{														//
	double r;											//
	int sign;											//
}circle;												//
//******************************************************//

//********************PID parameters*************************//
double kp_s;	//kp for speed							     //
double ki_s;	//ki for speed							     //
double kd_s;	//kd for speed							     //
double kp_d;	//kp for direction						     //
double ki_d;	//ki for direction					    	 //
double kd_d;	//kd for direction						     //
// Direction Control Variables						         //
double D_err;//direction error					             //
double D_errDiff = 0;//direction difference(Differentiation) //
double D_errSum = 0;//sum of direction error(Integration)      //
// Speed Control Variables								     //
circle c;												     //
double expectedSpeed;//      							     //
double curSpeedErr;//speed error   		                     //
double speedErrSum = 0;//sum of speed error(Integration)       //
int startPoint;											     //
int delta = 20;												 //
//***********************************************************//

//*******************Other parameters*******************//
const int topGear = 6;									//
double tmp;												//
bool flag = true;											//
double offset = 0;										//
double Tmp = 0;
double SpeedErr = 0;
//******************************************************//

//******************************Helping Functions*******************************//
// Function updateGear:															//
//		Update Gear automatically according to the current speed.				//
//		Implemented as Schmitt trigger.											//
void updateGear(int* cmdGear);													//
// Function constrain:															//
//		Given input, outputs value with boundaries.								//
double constrain(double lowerBoundary, double upperBoundary, double input);		//
// Function getR:																//
//		Given three points ahead, outputs a struct circle.						//
//		{radius:[1,500], sign{-1:left,1:right}									//
circle getR(float x1, float y1, float x2, float y2, float x3, float y3);		//
//******************************************************************************//

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm) {
	/* write your own code here */

	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}

cls_VISUAL cls_visual;
int nKey = 0;
char cKeyName[512];
time_t clock_begin = 0;
bool is_in_desert = 0;		// 1 refers to in the desert, 0 not
bool flag_is_in_desert_judged = 0;

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear) {
	static int judge = 0;
	if (parameterSet == false)		// Initialization Part
	{
		PIDParamSetter();
		clock_begin = clock();
	}
	else
	{
		// judge whether in the desert
		if (!flag_is_in_desert_judged)
		{
			if (_speed <= 60 && (clock() - clock_begin) / CLOCKS_PER_SEC <= 2)
			{
				judgeDesert();
			}
			else
			{
				flag_is_in_desert_judged = 1;
				is_in_desert = judgeDesert();
			}
		}

		// Speed Control
		/*
		You can modify the limited speed in this module
		Enjoy  -_-
		*/
		int m;
		if (is_in_desert)
		{
			m = constrain(80, 200, 0.002945 * _speed * _speed - 0.08891 * _speed + 60.511);
		}
		else
		{
			m = constrain(20, 200, 0.001894 * _speed * _speed + 0.3242 * _speed + 20);
		}
		circle c = getR(_midline[0][0], _midline[0][1], _midline[m / 2][0], _midline[m / 2][1], _midline[m][0], _midline[m][1]);


		if (is_in_desert)
		{
			expectedSpeed = -9.432e-07 * c.r * c.r * c.r + 0.0009511 * c.r * c.r + 0.04797 * c.r + 54.85;
			expectedSpeed = constrain(50, 150, expectedSpeed);
		}
		else
		{
			expectedSpeed = 4.289e-06 * c.r * c.r * c.r - 0.002539 * c.r * c.r + 0.6994 * c.r + 34.19;
			expectedSpeed = constrain(40, 180, expectedSpeed);
		}

		double Speediff;
		curSpeedErr = expectedSpeed - _speed;
		Speediff = _speed - SpeedErr;
		SpeedErr = _speed;
		speedErrSum = 0.1 * speedErrSum + curSpeedErr;
		if (curSpeedErr > 0)
		{

			if (abs(*cmdSteer) < 0.3)
			{
				if (!flag_is_in_desert_judged)
				{
					*cmdAcc = constrain(0.0, 0.8, kp_s * curSpeedErr / 2 + ki_s * speedErrSum + kd_s * Speediff + offset);
				}
				else
				{
					*cmdAcc = constrain(0.0, 0.4, kp_s * curSpeedErr / 2 + ki_s * speedErrSum + kd_s * Speediff + offset);
				}

				*cmdBrake = 0;
				if (is_in_desert == 1)
				{
					*cmdAcc = constrain(0, 0.25, kp_s * curSpeedErr / 2 + ki_s * speedErrSum + kd_s * Speediff + offset);
					*cmdBrake = 0;
				}
			}
			else if (abs(*cmdSteer) > 0.6)
			{
				*cmdAcc = 0 + offset;
				*cmdBrake = 0;
			}
			else
			{
				*cmdAcc = 0.005 + offset;
				*cmdBrake = 0;
			}

		}
		else if (curSpeedErr < 0)
		{
			*cmdBrake = constrain(0, 0.3, -kp_s * curSpeedErr - kd_s * Speediff - ki_s * speedErrSum);
			*cmdAcc = 0;
		}


		updateGear(cmdGear);

		/******************************************Modified by Yuan Wei********************************************/
		/*
		Please select a error model and coding for it here, you can modify the steps to get a new 'D_err',this is just a sample.
		Once you choose the error model, you can rectify the value of PID to improve your control performance.
		Enjoy  -_-
		*/
		// Direction Control		
		//set the param of PID controller
		kp_d = 15;
		ki_d = 0;
		kd_d = 27;
		if (is_in_desert)
		{
			kp_d = 12;
			ki_d = 0;
			kd_d = 22;
		}

		//std::fstream file;
		//FILE *fp;

		//get the error

		if (*cmdBrake == 1 && _acc < 19) judge++;
		else if (*cmdBrake == 1 && _acc > 20) judge--;
		judge = constrain(0, 80, judge);

		double distance, D_err1;
		int k = 50, k_dirt = 67;
		distance = _midline[0][0] * sqrt(_midline[0][1] * _midline[0][1] + _midline[0][0] * _midline[0][0]) / abs(_midline[0][0]);

		D_err = -atan2(k * distance, _speed) + _yaw;	//stanley method
		D_err1 = -atan2(_midline[3][0], _midline[3][1]);
		D_err = 0.8 * D_err + 0.2 * D_err1;
		D_errDiff = D_err - Tmp;
		D_errSum = D_errSum + D_err;
		Tmp = D_err;
		float T_err = sqrt(_midline[0][0] * _midline[0][0] + _midline[0][1] * _midline[0][1]) * _midline[0][0] / abs(_midline[0][0]);

		//set the error and get the cmdSteer
		*cmdSteer = constrain(-1, 1, kp_d * D_err + kd_d * D_errDiff);//stanley combined with pid;
		updateGear(cmdGear);

		//print some useful info on the terminal
		printf("c.r: %f \t", c.r);
		printf("speed: %f \t", _speed);
		printf("cmdSteer: %f \n", *cmdSteer);

#pragma region Wu
		/*cv::Mat im1Src = cv::Mat::zeros(cv::Size(400, 400), CV_8UC1);

		for (int i = 0; i < 200; i++)
			cv::circle(im1Src, cv::Point(200 + _midline[i][0] * 2, 400 - _midline[i][1] * 2), 2, cv::Scalar(100, 100, 100));
		sprintf_s(cKeyName, "Key: %c is pushed", nKey);
		cv::putText(im1Src, cKeyName, cv::Point(20, 50), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 255));
		cv::imshow("Path", im1Src);
		cls_visual.Fig1Y(5, 0, 150, 30, "CurrentV", _speed, "c.r", c.r);
		cls_visual.Fig2Y(3, 0, 150, 0, 1, 10, "CurrentV", _speed, "Acc", _acc, "+ACC", *cmdAcc);
		int tempKey = cv::waitKey(1);
		if (tempKey != -1)
			nKey = tempKey;*/
#pragma endregion
			/******************************************End by Yuan Wei********************************************/
	}
}

void PIDParamSetter()
{

	kp_s = 0.1;
	ki_s = 0.1;
	kd_s = 0.01;
	kp_d = 1.35;
	ki_d = 0.151;
	kd_d = 0.10;
	parameterSet = true;

}

void updateGear(int* cmdGear)
{
	if (_gearbox == 1)
	{
		if (_speed >= 60 && topGear > 1)
		{
			*cmdGear = 2;
		}
		else
		{
			*cmdGear = 1;
		}
	}
	else if (_gearbox == 2)
	{
		if (_speed <= 45)
		{
			*cmdGear = 1;
		}
		else if (_speed >= 112 && topGear > 2)
		{
			*cmdGear = 3;
		}
		else
		{
			*cmdGear = 2;
		}
	}
	else if (_gearbox == 3)
	{
		if (_speed <= 90)
		{
			*cmdGear = 2;
		}
		else if (_speed >= 152 && topGear > 3)
		{
			*cmdGear = 4;
		}
		else
		{
			*cmdGear = 3;
		}
	}
	else if (_gearbox == 4)
	{
		if (_speed <= 131)
		{
			*cmdGear = 3;
		}
		else if (_speed >= 197 && topGear > 4)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 4;
		}
	}
	else if (_gearbox == 5)
	{
		if (_speed <= 173)
		{
			*cmdGear = 4;
		}
		else if (_speed >= 250 && topGear > 5)
		{
			*cmdGear = 6;
		}
		else
		{
			*cmdGear = 5;
		}
	}
	else if (_gearbox == 6)
	{
		if (_speed <= 219)
		{
			*cmdGear = 5;
		}
		else
		{
			*cmdGear = 6;
		}
	}
	else if (_gearbox == -1)
	{
		if (_speed <= 1 && Time >= 4 && Time <= 300)
		{
			*cmdGear = -1;
		}
	}
	else
	{
		*cmdGear = 1;
	}
}

double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b * f - e * c) / (b * d - e * a);
	y = (d * c - a * f) / (b * d - e * a);
	r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	circle tmp = { r,sign };
	return tmp;
}

// 1 refers to in the desert, 0 not
bool judgeDesert()
{
	static int count = 0;
	if (_speed >= 45 && _speed <= 55)
	{
		if (_acc > 5 && _acc <= 10) count++;
		else if (_acc > 10) count--;
	}
	if (count > 0) return true;
	else return false;
}
