#include <vn200_node.hpp>

using namespace vectornav;

int main(int argc, char **argv)
{
	/* Try to connect to the VN200 IMU. */
	errorCode = vn200_connect(
		&vn200,
		COM_PORT,
		BAUD_RATE);

	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {
		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running 'sudo chmod 777 /dev/ttyUSB0'.\n");
		return 0;
	}

	/* Error encountered while connecting. Please reconnect. */
	if (errorCode != VNERR_NO_ERROR){
		printf("Error encountered when trying to connect to the sensor.\n");
		return 0;
	}

	/* Set cumulative initially to zero. */
	avgAcc.c0 = avgAcc.c1 = avgAcc.c2 = 0;
	avgYpr.yaw = avgYpr.pitch = avgYpr.roll = 0;

	/* Matrix C default to be Identity Matrix and vector B to be zero vector. */
	C.c00 = C.c11 = C.c22 = 1;
	C.c01 = C.c02 = C.c10 = C.c12 = C.c20 = C.c21 = 0;
	B.c0 = B.c1 = B.c2 = 0;


	/* Calculate averages at initial condition of rest. */
	for(i = 1; i <= iterAvg; i++){

		errorCode= vn200_getAcceleration(
			&vn200,
			&acceleration);

		errorCode= vn200_getYawPitchRoll(
			&vn200,
			&ypr);
		
		/* Calculate cumulatives. */
		avgYpr.yaw += ypr.yaw;
		avgYpr.pitch += ypr.pitch;
		avgYpr.roll += ypr.roll;
		
		avgAcc.c0 += acceleration.c0;
		avgAcc.c1 += acceleration.c1;
		avgAcc.c2 += acceleration.c2;

		usleep(50000);
	}

	/* Calculate averages. */
	avgAcc.c0 /=iterAvg;
	avgAcc.c1 /=iterAvg;
	avgAcc.c2 /=iterAvg;

	avgYpr.yaw /=iterAvg;
	avgYpr.pitch /=iterAvg;
	avgYpr.roll /=iterAvg;

	// fileOutPointer= fopen("data.dat","w");

	/* Start calibrated sensor measurement. */
	for (i = 0; i < iterOut; i++) {

		errorCode= vn200_getAcceleration(
			&vn200,
		 	&acceleration);		

		errorCode = vn200_getYawPitchRoll(
			&vn200,
			&ypr);
		
		/* Set offset vector B to be the averages. */
		B.c0 = avgAcc.c0;
		B.c1 = avgAcc.c1;
		B.c2 = avgAcc.c2;

		/* Set vector B in the acceleration compensation register. */
		vn200_setAccelerationCompensation(
			&vn200,
			C,
			B,
			true);

		/* Get calibrated accelerometer measurements. */
		errorCode = vn200_getQuaternionMagneticAccelerationAngularRate(
			&vn200,
			&attitude,
			&magnetic,
			&acceleration,
			&angularRate);

		ypr.yaw -= avgYpr.yaw;
		ypr.pitch -= avgYpr.pitch;
		ypr.roll -= avgYpr.roll;		

		/* Print calibrated gravity compensated measurements. */
		printf("IMU Solution:\n"
			"  Acceleration.x:         %+#7.2f\n"
			"  Acceleration.y:         %+#7.2f\n"
			"  Acceleration.z:         %+#7.2f\n",
			acceleration.c0,
			acceleration.c1,
			acceleration.c2);

		printf("IMU Solution:\n"
			"  Yaw:       			   %+#7.2f\n"
			"  Pitch:       		   %+#7.2f\n"
			"  Roll:        		   %+#7.2f\n",
			ypr.yaw,
			ypr.pitch,
			ypr.roll);

		/* We acheive a frequency of around 50 Hz. */
		usleep(10000);

		// fprintf(p,"%d %lf %lf %lf\n", i, acc.c0, acc.c1, acc.c2 - g);

		if (errorCode != VNERR_NO_ERROR){
			printf("Error encountered.\n");
			return 0;
		}
	}
	/* Close file that recorded all data measurements. */
	// fclose(p);
	
	/* Discconect vn200 device. */
	errorCode = vn200_disconnect(&vn200);
	if (errorCode != VNERR_NO_ERROR){
		printf("Error encountered when trying to disconnect from the sensor.\n");
		return 0;
	}
	
	return 0;
}
