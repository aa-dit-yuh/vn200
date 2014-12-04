#include <stdio.h>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <vectornav.h>

/* Change the connection settings to your configuration. */
/* Remember to provide su permissions: sudo chmod 777 /dev/ttyUSB0 */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

/* Value of acceleration due to gravity used to nullify gravity compensation. */
const double g = 9.80665;
 
/* Number of iterations to calculate averages. */
const int iterAvg = 100;

/* Number of iterations to output. */
const int iterOut = 100;

using namespace std;

namespace vectornav{

	/* The variable object that connects to VN200. */
	Vn200 vn200;

	int i;

	VN_ERROR_CODE errorCode;
	
	/* Variable that stores the quaternion/ YPR. */
	VnQuaternion attitude;
	VnYpr ypr;
	
	/* Measurements obtained from IMU. */
	VnVector3 acceleration, magnetic ,angularRate;

	/* Rotation matrix C and offset vector B used to compensate acceleration. */
	VnVector3 B;
	VnMatrix3x3 C;
	
	/* Vector to store averages. */;
	VnVector3 avgAcc;
	VnYpr avgYpr;

	/* File pointer to output values. */
	FILE * fileOutPointer;
}