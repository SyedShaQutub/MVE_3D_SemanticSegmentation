//
//                 INTEL CORPORATION PROPRIETARY INFORMATION
//
//    This software is supplied under the terms of a license agreement or
//    nondisclosure agreement with Intel Corporation and may not be copied
//    or disclosed except in accordance with the terms of that agreement.
//    Copyright (C) 2014 Intel Corporation. All Rights Reserved.
//
//    ## specifics
//
//    Mistl - Multi-camera Image STructure Library
//    Author: Jonas Scheer
//


#include 	"opencv2/opencv.hpp"
#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
#include	"mistl/OCV_Camera.h"
#include	"mistl/Error.h"
#include	<stdlib.h>
#include 	<stdio.h>
#include 	<string.h>



void
PrMsg() {
	std::cout<<"\nThis program tests a stereo calibration by performing a stereo rectification: \n";
	std::cout<<"usage: testCalib input-calibration-file_1 input-calibration-file_2 input_image_1 input_image_2 output_image_1 output_image_2 [concatinated_output_image] \n";
	std::cout<<"    input-calibration-file_1 and input-calibration-file_2 need to be *.cahv calibration files \n";
	std::cout<<"    optional concatinated_output_image parameter concatenates the rectified images and draws the epipolar lines\n";
	    exit(-1);
}




/*
 * a simple program that creates a hardcoded 
 * openCV camera and copies its parameters to
 * a mistel camera to check if the 
 * mistel camera is correctly initialized.
 */
int main(int argc, char** argv)
{
	//
	// hardcoded cam for testing
	//
	mistl::OCV_Camera oCam;

	double camMat[3][3];
	camMat[0][0] = 800;
	camMat[0][1] = 0;
	camMat[0][2] = 1920.0/2.0f;
	camMat[1][0] = 0;
	camMat[1][1] = 800;
	camMat[1][2] = 1080.0/2.0f;
	camMat[2][0] = 0;
	camMat[2][1] = 0;
	camMat[2][2] = 1;
	oCam.cameraMatrix = cv::Mat( 3, 3, cv::DataType<double>::type, camMat );

	double distMat[5];
	distMat[0] = 123;
	distMat[1] = -123;
	distMat[2] = 0.0;
	distMat[3] = 0.123f;
	distMat[4] = 0;
	oCam.distCoeffs = cv::Mat( 1, 5, cv::DataType<double>::type, distMat );

	double transMat[1][3];
	transMat[0][0] = 0;
	transMat[0][1] = 0;
	transMat[0][2] = 0;
	oCam.tvec = cv::Mat( 1, 3, cv::DataType<double>::type, transMat );

	double rotMat[3][3];
	rotMat[0][0] = 1;
	rotMat[0][1] = 0;
	rotMat[0][2] = 0;
	rotMat[1][0] = 0;
	rotMat[1][1] = 1;
	rotMat[1][2] = 0;
	rotMat[2][0] = 0;
	rotMat[2][1] = 0;
	rotMat[2][2] = 1;
	cv::Mat tmpRot = cv::Mat( 3, 3, cv::DataType<double>::type, rotMat );
	cv::Mat rot = cv::Mat( 3, 1, cv::DataType<double>::type);
	cv::Rodrigues(tmpRot, rot);
	oCam.rvec = rot;




	std::cout << "\n" << std::endl;
	// print openCV camera
	std::cout << "\openCV Camera:" << std::endl;
	oCam.Info();
	std::cout << "\openCV Camera translation:" << std::endl;
	std::cout << oCam.tvec << std::endl;
	std::cout << "\openCV Camera rotation:" << std::endl;
	std::cout << oCam.rvec << std::endl;
	std::cout << "\n" << std::endl;


	// copy openCV to Mistl
	mistl::Camera mistlCam;
	oCam.CopyTo(mistlCam);

	// print mistl camera
	std::cout << "\nMISTL Camera:" << std::endl;
	mistlCam.Info();
	std::cout << "\n" << std::endl;

}
