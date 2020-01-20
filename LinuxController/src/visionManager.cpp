/** @file visionManager.cpp
 *  @brief Functions for managing images/vision.
 *
 *
 *  @author Bill Merryman
 *  @bug No known bugs.
 *
 *  Created on: Dec 9, 2019
 *
 */

#include <iostream>
#include <fstream>
#include <unistd.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/dnn/shape_utils.hpp"
#include "visionManager.hpp"

using namespace std;
using namespace cv;
using namespace cv::dnn;

VideoCapture cap;

Mat displayImage;
Mat processingImage;

Net caffeNet;
Net darknetNet;

int imageProcessingType = 0;

Scalar thresholdLow = Scalar(60, 0, 100);
Scalar thresholdHigh = Scalar(200, 80, 255);
Rect thresholdROI = Rect(135, 95, 50, 50);

float caffeConfidence = 0.0;
float darknetConfidence = 0.0;
float darknetNonMaximaSuppressionThreshold = 0.0;

std::vector<std::string> caffeClasses;
std::vector<std::string> darknetClasses;

std::vector<cv::String> unconnectedOutputLayersNames;

void visionManagerInitialize(const char *caffeNamesFile,
				const char *prototxtFile,
				const char *caffemodelFile,
				float caffeConf,
				const char *darknetNamesFile,
				const char *cfgFile,
				const char *weightsFile,
				float darknetConf,
				float darknetNMSThreshold)
{
	CvSize inputSize;

	cap.open(0);

	caffeNet = cv::dnn::readNet(caffemodelFile, prototxtFile);
	darknetNet = cv::dnn::readNet(weightsFile, cfgFile);

	caffeConfidence = caffeConf;
	darknetConfidence = darknetConf;
	darknetNonMaximaSuppressionThreshold = darknetNMSThreshold;

	visionManagerInitializeCaffe();
	visionManagerInitializeDarknet();

	string nameLine;

	ifstream cnf(caffeNamesFile);
	while (getline(cnf, nameLine)) caffeClasses.push_back(nameLine);

	ifstream dnf(darknetNamesFile);
	while (getline(dnf, nameLine)) darknetClasses.push_back(nameLine);

	cvNamedWindow("Display_Image", CV_WINDOW_AUTOSIZE);
	cvNamedWindow("Processing_Image", CV_WINDOW_AUTOSIZE);
	setWindowTitle("Display_Image", "No Image Processing.");
	setWindowTitle("Processing_Image", "Not Used.");
}

void visionManagerInitializeCaffe()
{

}

void visionManagerInitializeDarknet()
{
        vector<int> unconnectedOutputLayersIndices = darknetNet.getUnconnectedOutLayers();
        vector<String> outputLayersNames = darknetNet.getLayerNames();
        unconnectedOutputLayersNames.resize(unconnectedOutputLayersIndices.size());
        for (size_t i = 0; i < unconnectedOutputLayersIndices.size(); ++i)
        unconnectedOutputLayersNames[i] = outputLayersNames[unconnectedOutputLayersIndices[i] - 1];
}

void visionManagerUninitialize()
{
	cvDestroyWindow("Display_Image");
	cvDestroyWindow("Processing_Image");
}

void visionManagerProcess(char key)
{
	if(key=='n')
	{
		imageProcessingType=0;
		setWindowTitle("Display_Image", "No Image Processing.");
		setWindowTitle("Processing_Image", "Not Used.");
	}
	if(key=='t')
	{
		imageProcessingType=1;
		setWindowTitle("Display_Image", "Process Image By Threshold");
		setWindowTitle("Processing_Image", "Image Moments");
	}
	if(key=='k')
	{
		imageProcessingType=2;
		setWindowTitle("Display_Image", "Capture color key for Threshold");
		setWindowTitle("Processing_Image", "Not Used.");
	}
	if(key=='c')
	{
		imageProcessingType=3;
		setWindowTitle("Display_Image", "Process Image By Caffe");
		setWindowTitle("Processing_Image", "Not Used.");
	}
	if(key=='d')
	{
		imageProcessingType=4;
		setWindowTitle("Display_Image", "Process Image By Darknet");
		setWindowTitle("Processing_Image", "Not Used.");
	}

	switch(imageProcessingType)
	{
		case 0:
			visionManagerProcessNone();
			break;
		case 1:
			visionManagerProcessThreshold();
			break;
		case 2:
			visionManagerCaptureThreshold();
			break;
		case 3:
			visionManagerProcessCaffe();
			break;
		case 4:
			visionManagerProcessDarknet();
			break;
	}

}

void visionManagerProcessNone()
{
	cap.read(displayImage);
	imshow("Display_Image", displayImage);
}

void visionManagerProcessThreshold()
{
	double area = 0;
	CvPoint position;
	char outputMessage[50];

	/*
	 * Not sure why, but in this version (Debian 9.5), trying to pass the
	 * displayImage and processingImage directly like in previous versions
	 * crashes the entire system! Cloning the images like this works fine...
	 */
	cap.read(displayImage);
	Mat display = displayImage.clone();
	Mat processing = displayImage.clone();

	inRange(display, thresholdLow, thresholdHigh, processing);
	cv::Moments moments = cv::moments(processing, false);
	area = moments.m00;
	if (area > 1000000)
	{
		position.x = moments.m10 / area;
		position.y = moments.m01 / area;
		sprintf(outputMessage, "pos: %d, %d", position.x, position.y);
		rectangle(display, cvPoint(position.x - 5, position.y - 5), cvPoint(position.x + 5, position.y + 5), cvScalar(0, 255, 0, 0), 1, 8, 0);
		putText(display, outputMessage, Point(position.x + 10, position.y + 5), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	}
	imshow("Display_Image", display);
	imshow("Processing_Image", processing);
}

void visionManagerCaptureThreshold()
{
	char outputMessage[64];
	double minR;
	double minG;
	double minB;
	double maxR;
	double maxG;
	double maxB;
	Mat bgr[3];

	cap.read(displayImage);
	split(displayImage(thresholdROI), bgr);
	minMaxLoc(bgr[0], &minR, &maxR);
	minMaxLoc(bgr[1], &minG, &maxG);
	minMaxLoc(bgr[2], &minB, &maxB);

	thresholdLow = Scalar(minR, minG, minB);
	thresholdHigh = Scalar(maxR, maxG, maxB);

	putText(displayImage, format("minR: %f", minR), Point(0, 10), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	putText(displayImage, format("minG: %f", minG), Point(0, 25), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	putText(displayImage, format("minB: %f", minB), Point(0, 40), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	putText(displayImage, format("maxR: %f", maxR), Point(0, 55), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	putText(displayImage, format("maxG: %f", maxG), Point(0, 70), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	putText(displayImage, format("maxB: %f", maxB), Point(0, 85), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	rectangle(displayImage, thresholdROI, Scalar(0, 255, 0), 1, 8, 0);

	imshow("Display_Image", displayImage);
	//imshow("Processing_Image", processingImage);
}

void visionManagerProcessCaffe()
{
	const Size resized(320, 240);
	Point position;

	char outputMessage[64];

	cap.read(displayImage);
	//resize(displayImage, processingImage, resized, 0, 0, CV_INTER_LINEAR);
	Mat blob = cv::dnn::blobFromImage(displayImage,
										0.007843f,
										resized,
										Scalar(127.5));
	caffeNet.setInput(blob);
	Mat detections = caffeNet.forward();
	for(int i = 0; i < detections.size[2]; i++)
	{
		int idxConf[4] = {0, 0, i, 2};
		float conf = detections.at<float>(idxConf);

		if(conf > caffeConfidence)
		{
			int idxCls[4] = {0, 0, i, 1};
			int cls = detections.at<float>(idxCls);

			int leftPercent[4] = {0, 0, i, 3};
			int topPercent[4] = {0, 0, i, 4};
			int widthPercent[4] = {0, 0, i, 5};
			int heightPercent[4] = {0, 0, i, 6};

			position.x = detections.at<float>(leftPercent) * displayImage.cols;
			position.y = detections.at<float>(topPercent) * displayImage.rows;
			int width = (detections.at<float>(widthPercent) * displayImage.cols) - position.x;
			int height = (detections.at<float>(heightPercent) * displayImage.rows) - position.y;

			Rect detection(position.x, position.y, width, height);
			rectangle(displayImage, detection, Scalar(0, 255, 0), 1, 8, 0);
			string label = format("%.2f", conf);
			if (!caffeClasses.empty())
			{
				CV_Assert(cls < (int)caffeClasses.size());
				label = caffeClasses[cls] + ":" + label;
			}
			putText(displayImage, label, Point(position.x, position.y + 10), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
		}
	}
	imshow("Display_Image", displayImage);
	//imshow("Processing_Image", processingImage);
}

void visionManagerProcessDarknet()
{
	const Size resized(192, 192);
	Point position;
	vector<int> classIds;
	vector<float> confidences;
	vector<Rect> boxes;

	char outputMessage[64];

	cap.read(displayImage);
	//resize(displayImage, processingImage, resized, 0, 0, CV_INTER_LINEAR);
	Mat blob = cv::dnn::blobFromImage(displayImage,
										0.007843f,
										resized,
										Scalar(127.5));

	darknetNet.setInput(blob);
	vector<Mat> outs;
	darknetNet.forward(outs, unconnectedOutputLayersNames);

    	for (size_t i = 0; i < outs.size(); ++i)
    	{
        	// Scan through all the bounding boxes output from the network and keep only the
        	// ones with high confidence scores. Assign the box's class label as the class
        	// with the highest score for the box.
        	float* data = (float*)outs[i].data;
        	for (int j = 0; j < outs[i].rows; ++j, data += outs[i].cols)
        	{
            		Mat scores = outs[i].row(j).colRange(5, outs[i].cols);
            		Point classIdPoint;
            		double confidence;
            		// Get the value and location of the maximum score
            		minMaxLoc(scores, 0, &confidence, 0, &classIdPoint);
            		if (confidence > darknetConfidence)
            		{
                		int centerX = (int)(data[0] * displayImage.cols);
                		int centerY = (int)(data[1] * displayImage.rows);
                		int width = (int)(data[2] * displayImage.cols);
                		int height = (int)(data[3] * displayImage.rows);
                		int left = centerX - width / 2;
                		int top = centerY - height / 2;

                		classIds.push_back(classIdPoint.x);
                		confidences.push_back((float)confidence);
                		boxes.push_back(Rect(left, top, width, height));
            		}
        	}
    	}

    	// Perform non maximum suppression to eliminate redundant overlapping boxes with
    	// lower confidences
    	vector<int> indices;
    	NMSBoxes(boxes, confidences, darknetConfidence, darknetNonMaximaSuppressionThreshold, indices);

    	for (size_t i = 0; i < indices.size(); ++i)
    	{
        	int idx = indices[i];
        	Rect box = boxes[idx];
    		rectangle(displayImage, Point(box.x, box.y), Point(box.x + box.width, box.y + box.height), Scalar(0, 255, 0), 1, 8, 0);
    		string label = format("%.2f", confidences[idx]);
    		if (!darknetClasses.empty())
    		{
        		CV_Assert(classIds[idx] < (int)darknetClasses.size());
        		label = darknetClasses[classIds[idx]] + ":" + label;
    		}
    		putText(displayImage, label, Point(box.x, box.y), CV_FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 2, 8, false);
	}
	imshow("Display_Image", displayImage);
	//imshow("Processing_Image", processingImage);
}

