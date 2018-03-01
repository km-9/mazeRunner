#include "PCA9685.h"
#include <iostream>
#include <stdio.h>
//#include <ncurses.h>
//#include <thread>
#include <unistd.h>
#include <stdlib.h>
#include <cstddef>
#include <cstdio>
#include <ctime>
#include "rplidar.h"
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


using namespace rp::standalone::rplidar;
using namespace std;

int leftCount; int rightCount;
bool leftFollow = true;
double frontVal;
double leftVal;
double rightVal;

u_result capture_and_display(RPlidarDriver* drv){
	u_result ans;
	cout << "inside cap_and_disp" << endl;

	rplidar_response_measurement_node_t nodes[360*2];
	size_t count = _countof(nodes);

	// fetech extactly one 0-360 degrees' scan
	ans = drv->grabScanData(nodes, count);
	if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
			drv->ascendScanData(nodes, count);
			frontVal = 1;
			leftVal = 1;
			rightVal = 1;
					for (int pos = 0; pos < (int)count ; ++pos) {
									if (pos == 180){
											rightVal = nodes[pos].distance_q2/4.0f;
											cout << "Angle: "<< pos << " Dist: " << nodes[pos].distance_q2/4.0f;
										}
									if (pos == 90){
											leftVal = nodes[pos].distance_q2/4.0f;
											cout << "Angle: "<< pos << " Dist: " << nodes[pos].distance_q2/4.0f;
										}
									if (pos == 270){
											frontVal = nodes[pos].distance_q2/4.0f;
											cout << "Angle: "<< pos << " Dist: " << nodes[pos].distance_q2/4.0f;
									}
					}
			}
	 else {
			printf("error code: %x\n", ans);
	}
	return ans;
}

bool canLeft(){
	cout << "leftCall" << endl;
	cout << leftVal << endl;
  if(leftVal > 300){
//  if(leftCount < 1){
		cout << "canLeft" << endl;
    return true;
//  }
}
  return false;
}

bool canRight(){
cout << "rightCall" << endl;
cout << rightVal << endl;
if(rightVal > 300){
 //if(rightCount < 1){
	 cout << "canRight" << endl;
   return true;
 }
}
 return false;
}

bool canForward(){
		cout << "forwardCall" << endl;
		cout << frontVal << endl;
if(frontVal > 300){
	cout << "canForward" << endl;
 return true;
}
 return false;
}

int main (int argc, char const *argv[]) {
	cout << "inside main" << endl;
	PCA9685 pwm1;
	pwm1.init(1,0x40);

  PCA9685 pwm2;
	pwm2.init(1,0x40);

	pwm1.setPWMFreq(61);
  pwm2.setPWMFreq(61);
	usleep(1000 * 1000);
  pwm1.setPWM(0,0,0);
	pwm2.setPWM(1,0,0);
    int opt_com_baudrate = 115200;
    const char * opt_com_path = "/dev/ttyUSB0";
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);
		do {
        // try to connect
        if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
            break;
        }
        drv->startMotor();

        // take only one 360 deg scan and display the result as a histogram
        ////////////////////////////////////////////////////////////////////////////////
        if (IS_FAIL(drv->startScan( /* true */ ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
        {
            fprintf(stderr, "Error, cannot start the scan operation.\n");
            break;
        }

        if (IS_FAIL(capture_and_display(drv))) {
            fprintf(stderr, "Error, cannot grab scan data.\n");
            break;

        }
				cout << "end of lidar driver in main" << endl;
			}while(0);
while(true){
		capture_and_display(drv);
			if (canLeft()){
			//	capture_and_display(drv);
				pwm1.setPWM(0,0,600);
		    pwm2.setPWM(1,0,400);
			}
			if (canRight() && !canLeft() && rightVal < 400){
			//	capture_and_display(drv);
				pwm1.setPWM(0,0,400);
		    pwm2.setPWM(1,0,600);
			}
			while (canForward() && !canLeft()){
				//capture_and_display(drv);
				pwm1.setPWM(0,0,150);
		    pwm2.setPWM(1,0,600);
			}
}
    drv->stop();
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);
    return 0;
  }
