#include <stdio.h>
#include <unistd.h>
#include <vectornav.h>
/* Change the connection settings to your configuration. */
const char* const COM_PORT = "//dev//ttyUSB0";
const int BAUD_RATE = 115200;
const double g = 9.80665;

void asyncDataListener(
	void* sender,
	VnDeviceCompositeData* data);

int main()
{
	int i;
	VN_ERROR_CODE errorCode;
	Vn200 vn200;
	VnQuaternion attitude;
	VnVector3 acc,mag,angRate,b;
	VnMatrix3x3 c;
	c.c00=c.c11=c.c22=1;
	c.c01=c.c02=c.c10=c.c12=c.c20=c.c21=0;	
	
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
	if (errorCode != VNERR_NO_ERROR){
		printf("Error encountered when trying to connect to the sensor.\n");
		return 0;
	}
	
	for (i = 0; i < 100; i++) {

		/* The library is handling and storing asynchronous data by itself.
		   Calling this function retrieves the most recently processed
		   asynchronous data packet. */
		vn200_getQuaternion(
			&vn200,
			&attitude);

		printf("IMU Solution:\n"
			"  Quaternion.x:           %+#7.2f\n"
			"  Quaternion.y:           %+#7.2f\n"
			"  Quaternion.z:           %+#7.2f\n"
			"  Quaternion.w:           %+#7.2f\n",
			attitude.x,
			attitude.y,
			attitude.z,
			attitude.w);

		b.c0 = 2 * (attitude.y * attitude.w - attitude.x * attitude.z);
		b.c1 = 2 * (attitude.x * attitude.y + attitude.z * attitude.w);
		b.c2 = attitude.x * attitude.x - attitude.y * attitude.y - attitude.z * attitude.z + attitude.w * attitude.w;


		printf("%lf %lf %lf\n",b.c0,b.c1,b.c2);

		vn200_setAccelerationCompensation(
			&vn200,
			c,
			b,
			true);

		vn200_getQuaternionMagneticAccelerationAngularRate(
			&vn200,
			&attitude,
			&mag,
		 	&acc,
			&angRate);

		printf("IMU Solution:\n"
			"  Quaternion.x:           %+#7.2f\n"
			"  Quaternion.y:           %+#7.2f\n"
			"  Quaternion.z:           %+#7.2f\n"
			"  Quaternion.w:           %+#7.2f\n"
			"  Acceleration.x:         %+#7.2f\n"
			"  Acceleration.y:         %+#7.2f\n"
			"  Acceleration.z:         %+#7.2f\n",
			attitude.x,
			attitude.y,
			attitude.z,
			attitude.w,
			acc.c0,
			acc.c1,
			acc.c2);

		sleep(1);
	}
	
	
	errorCode = vn200_disconnect(&vn200);
	if (errorCode != VNERR_NO_ERROR){
	printf("Error encountered when trying to disconnect from the sensor.\n");
	return 0;
	}
return 0;
}


// void asyncDataListener(void* sender, VnDeviceCompositeData* data)
// {
// 	VnVector3 b;
// 	VnMatrix3x3 c;
// 	c.c00=c.c11=c.c22=1;
// 	c.c01=c.c02=c.c10=c.c12=c.c20=c.c21=0;	
// 	b.c0 = 2 * (data->quaternion.y * data->quaternion.w - data->quaternion.x * data->quaternion.z);
// 	b.c1 = 2 * (data->quaternion.x * data->quaternion.y + data->quaternion.z * data->quaternion.w);
// 	b.c2 = data->quaternion.x * data->quaternion.x - data->quaternion.y * data->quaternion.y - data->quaternion.z * data->quaternion.z + data->quaternion.w * data->quaternion.w;


// 	printf("%lf %lf %lf\n",b.c0,b.c1,b.c2);

// 	vn200_setAccelerationCompensation(
// 		&vn200,
// 		c,
// 		b,
// 		true);



// 	printf("\n\n");
// }

