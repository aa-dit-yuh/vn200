#include <stdio.h>
#include <unistd.h>
#include <vectornav.h>
/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;

int main()
{
	VN_ERROR_CODE errorCode;
	VnVector3 mag, acc, angRate;
	float temp, pressure;
	Vn200 vn200;
	errorCode = vn200_connect(
		&vn200,
		COM_PORT,
		BAUD_RATE);
	/* Make sure the user has permission to use the COM port. */
	if (errorCode == VNERR_PERMISSION_DENIED) {
		printf("Current user does not have permission to open the COM port.\n");
		printf("Try running again using 'sudo'.\n");
		return 0;
	}
	else if (errorCode != VNERR_NO_ERROR){
		printf("Error encountered when trying to connect to the sensor.\n");
		return 0;
	}
	while(1){
		/* Query the InsSolution register of the VN-200. Note this method of
		retrieving the attitude is blocking since a serial command will be
		sent to the physical sensor and this program will wait until a
		response is received. */
		vn200_getImuMeasurements(
			&vn200,
			&mag,
			&acc,
			&angRate,
			&temp,
			&pressure);
		printf("IMU Solution:\n"
			"Magnetic x: %lf\n"
			"Magnetic y: %lf\n"
			"Magnetic z: %lf\n"
			"Acceleration x: %lf\n"
			"Acceleration y: %lf\n"
			"Acceleration z: %lf\n"
			"Angular Rate x: %lf\n"
			"Angular Rate y: %lf\n"
			"Angular Rate z: %lf\n"
			"Temperature: %lf\n"
			"Pressure: %lf\n",
			mag.c0,
			mag.c1,
			mag.c2,
			acc.c0,
			acc.c1,
			acc.c2,
			angRate.c0,
			angRate.c1,
			angRate.c2,
			temp,
			pressure);
		printf("\n\n");
		sleep(1);
	}
	errorCode = vn200_disconnect(&vn200);
	if (errorCode != VNERR_NO_ERROR){
	printf("Error encountered when trying to disconnect from the sensor.\n");
	return 0;
	}
return 0;
}