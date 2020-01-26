/** @file main.cpp
 *  @brief
 *
 *  @author Bill Merryman
 *  @bug No known bugs.
 *
 *  Created on: Dec 9, 2019
 *
 */

#include <iostream>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include "opencv2/highgui.hpp"

#include "motionManager.hpp"
#include "visionManager.hpp"

int main(int argc, char *argv[]) {

	if(argc < 10)
	{
		fprintf(stderr, "Usage: BioloidBeaglebone MLCaffeNamesFile MLProtoFile MLCaffeFile MLCaffeConfidence MLDarknetNamesFile MLDarknetCfgFile MLDarknetWeightsFile MLDarknetConfidence MLDarknetNMSThreshold");
		return -1;
	}

	int key = 0;

	const char *caffeNamesFile = argv[1];
	const char *protoFile = argv[2];
	const char *modelFile = argv[3];
	float caffeConfidence = atof(argv[4]);
	const char *darknetNamesFile = argv[5];
	const char *cfgFile = argv[6];
	const char *weightsFile = argv[7];
	float darknetConfidence = atof(argv[8]);
	float darknetNMSThreshold = atof(argv[9]);

	int usbfd = open("/dev/ttyACM0", O_RDWR| O_NONBLOCK | O_NDELAY);
	configurePort(usbfd);

	visionManagerInitialize(caffeNamesFile,
							protoFile,
							modelFile,
							caffeConfidence,
							darknetNamesFile,
							cfgFile,
							weightsFile,
							darknetConfidence,
							darknetNMSThreshold);

	while(key != 'x')
	{
		visionManagerProcess(key);
		key = cvWaitKey(25);
		if(key=='s')
		{
			transferFlashToMotionPage(usbfd, 2);
			executeMotionPage(usbfd, 2);
			key=' ';
		}
	}

}


