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
//#include 	<opencv2/core/core.hpp>
//#include 	<opencv2/highgui/highgui.hpp>
#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
#include    "mistl/WriteOBJ.h"
#include	"mistl/OCV_Camera.h"
#include	"mistl/Error.h"
#include	<stdlib.h>
#include 	<stdio.h>
#include 	<string.h>



void
PrMsg() {
	std::cout<<"\nThis program tests a stereo calibration by performing a stereo rectification: \n";
	std::cout<<"usage: testCalib input-calibration-file_1 input-calibration-file_2 input_image_1 input_image_2 output_image_1 output_image_2 [concatinated_output_image] \n";
	std::cout<<"    input-calibration-file_1 and input-calibration-file_2 need to be *.ycam or *.cahv calibration files \n";
	std::cout<<"    optional concatinated_output_image parameter concatenates the rectified images and draws the epipolar lines\n";
	    exit(-1);
}


int main(int argc, char** argv)
{

	if(argc<7) {
		PrMsg();
		return -1;
	}

	char* camFile1 = argv[1];
	char* camFile2 = argv[2];
	mistl::OCV_Camera oCam1;
	mistl::OCV_Camera oCam2;

	//
	// read data for camera 1.
	// use ycam or yaml file
	//

	try {

		mistl::Camera cam1;
		mistl::Camera cam2;

		ReadCamera(cam1, camFile1);
		oCam1.Set(cam1);

		ReadCamera(cam2, camFile2);
		oCam2.Set(cam2);
	}
	catch ( mistl::Error e) {
		std::cout<<"Error: "<<e.GetMsg()<<"\n";
		PrMsg();
		return -1;
	}

	std::cout<<"\n";


	//
	// read images with openCV
	//
	cv::Mat image1;
	cv::Mat image2;
	char* img1 = argv[3];
	char* img2 = argv[4];


	char* img1_rect = argv[5];
	char* img2_rect = argv[6];

	image1 = cv::imread(img1, CV_LOAD_IMAGE_COLOR);
	image2 = cv::imread(img2, CV_LOAD_IMAGE_COLOR);
	if(!image1.data)
	{
		 PrMsg();
		std::cout <<  "Could not open or find image: '" << img1 << "'" << std::endl ;
		return -1;
	}
	if(!image2.data)
	{
		PrMsg();
		std::cout <<  "Could not open or find image: '" << img2 << "'" << std::endl ;
		return -1;
	}
	cv::Size imageSize(image1.cols, image1.rows);
	cv::Mat image_rect(imageSize.height, imageSize.width*2, CV_32FC3);
	cv::Mat image1_rect(imageSize.height, imageSize.width, CV_32FC3);
	cv::Mat image2_rect(imageSize.height, imageSize.width, CV_32FC3);




	//
	// stereo rectofy
	// (first we compute the relative rotation and translation)
	//
	// performe matrix multiplication: rotMat2 * (rotMat1)^(-1) 	to compute relRot
	cv::Mat mRot1(3, 3, cv::DataType<double>::type);
	cv::Mat mRot2(3, 3, cv::DataType<double>::type);
	cv::Rodrigues(oCam1.rvec, mRot1);
	cv::Rodrigues(oCam2.rvec, mRot2);
	cv::Mat tmpRot1_Inv = mRot1.inv();
	cv::Mat relRot = mRot2 * tmpRot1_Inv;							// relative rotation matrix between the coordinate systems of the first and the second cameras.

	// transpose vectors for relative translation
	double t1[3][1];
	t1[0][0] = oCam1.tvec.at<double>(0, 0);
	t1[1][0] = oCam1.tvec.at<double>(0, 1);
	t1[2][0] = oCam1.tvec.at<double>(0, 2);
	cv::Mat transp1(3, 1, cv::DataType<double>::type, t1);
	double t2[3][1];
	t2[0][0] = oCam2.tvec.at<double>(0, 0);
	t2[1][0] = oCam2.tvec.at<double>(0, 1);
	t2[2][0] = oCam2.tvec.at<double>(0, 2);
	cv::Mat transp2(3, 1, cv::DataType<double>::type, t2);
	cv::Mat relTransl = transp2 - (relRot*transp1);					//  relative translation


	cv::Mat R1,R2,P1,P2,Q;
	cv::Rect_<int>* roi1 = new cv::Rect_<int>;
	cv::Rect_<int>* roi2 = new cv::Rect_<int>;
	float alpha = 1.0f;
	cv::stereoRectify(oCam1.cameraMatrix, oCam1.distCoeffs, oCam2.cameraMatrix, oCam2.distCoeffs, imageSize,
			relRot, relTransl, R1, R2, P1, P2, Q, 0, alpha, imageSize, roi1, roi2);


	// create undistortion maps and remap image data
	cv::Mat cam1Map1 = cvCreateMat( imageSize.height, imageSize.width, CV_32FC1 );
	cv::Mat cam1Map2 = cvCreateMat( imageSize.height, imageSize.width, CV_32FC1 );
	cv::initUndistortRectifyMap(oCam1.cameraMatrix, oCam1.distCoeffs, R1, P1, imageSize, CV_32FC1, cam1Map1, cam1Map2);
	cv::remap(image1, image1_rect, cam1Map1, cam1Map2, cv::INTER_LANCZOS4, cv::BORDER_TRANSPARENT);
	cv::rectangle(image1_rect, *roi1, cv::Scalar(255, 180, 0));

	cv::Mat cam2Map1 = cvCreateMat( imageSize.height, imageSize.width, CV_32FC1 );
	cv::Mat cam2Map2 = cvCreateMat( imageSize.height, imageSize.width, CV_32FC1 );
	cv::initUndistortRectifyMap(oCam2.cameraMatrix, oCam2.distCoeffs, R2, P2, imageSize, CV_32FC1, cam2Map1, cam2Map2);
	cv::remap(image2, image2_rect, cam2Map1, cam2Map2, cv::INTER_LANCZOS4, cv::BORDER_TRANSPARENT);
	cv::rectangle(image2_rect, *roi2, cv::Scalar(255, 180, 0));



	//
	// writing rectified images
	//
	cv::vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
	compression_params.push_back(100);
	cv::imwrite(img1_rect, image1_rect, compression_params);
	cv::imwrite(img2_rect, image2_rect, compression_params);




	// concatenate rectified images to a single one
	// with epipolar lines
	if(argc > 7) {
		cv::Rect_<int>* roi3 = new cv::Rect_<int>;
		roi3->x = roi2->x + imageSize.width;
		roi3->y = roi2->y;
		roi3->width = roi2->width;
		roi3->height = roi2->height;
		// concat rectified images into a single one
		cv::Mat rectImage = cvCreateMat( imageSize.height, imageSize.width*2, CV_32FC3 );
		cv::hconcat(image1_rect, image2_rect, rectImage);

		// draw lines in concated image
		int lines = 60;
		for(int i=0; i<lines; i++ ) {
			int r = (i%3) == 0 ? 255 : 0;
			int g = (i%3) == 1 ? 255 : 0;
			int b = (i%3) == 2 ? 255 : 0;
			cv::line(rectImage, cv::Point(0, (imageSize.height/lines) * i), cv::Point(imageSize.width*2, (imageSize.height/lines) * i),
					cv::Scalar(r, g, b));
		}
		cv::rectangle(rectImage, *roi1, cv::Scalar(255, 180, 0), 3);
		cv::rectangle(rectImage, *roi3, cv::Scalar(255, 180, 0), 3);
		cv::imwrite(argv[7], rectImage, compression_params);
	}
}
