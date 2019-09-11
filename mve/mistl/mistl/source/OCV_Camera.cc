

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
//    Author: Oliver Grau
//


#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
#include	<stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include	"mistl/OCV_Camera.h"



namespace mistl {
  


void	
ReadCameraYAML( mistl::OCV_Camera	&cam, const char *fn)
{
//	cv::FileStorage fs2(fn, cv::FileStorage::READ);
//
//	fs2["Intrinsics"] >> cam.cameraMatrix;
//	if(!fs2["tvec"].empty()) fs2["tvec"] >> cam.tvec;
//	if(!fs2["rvec"].empty()) fs2 ["rvec"] >> cam.rvec;
//	fs2["Distortion"] >> cam.distCoeffs;




	cv::FileStorage fs2(fn, cv::FileStorage::READ);

		fs2["Intrinsics"] >> cam.cameraMatrix;
		if(!fs2["tvec"].empty()) fs2["tvec"] >> cam.tvec;
		if(!fs2["rvec"].empty()) fs2 ["rvec"] >> cam.rvec;
		// reading rotation matrix instead of rotation vector
		// and using openCV to transform it to a matrix
		if(!fs2["rmat"].empty()) {
			cv::Mat rotMat = cv::Mat(3,3,CV_64F,0.0);
			fs2 ["rmat"] >> rotMat;
			cv::Rodrigues(rotMat, cam.rvec);
		}
		fs2["Distortion"] >> cam.distCoeffs;

		// read camera resolution
		cv::Mat resolution = cv::Mat(1,3,CV_16U,0.0);
		fs2["resolution"] >> resolution;
		cam.nx = resolution.at<int>(0, 0);
		cam.ny = resolution.at<int>(0, 1);

		// read camera pixel size
		cv::Mat pixel_size = cv::Mat(1,2,CV_64F,0.0);
		fs2["pixel_size"] >> pixel_size;


		// old
	//	cam.sx = pixel_size.at<double>(0, 0);
	//	cam.sy = pixel_size.at<double>(0, 1);
	//	cam.f = cam.cameraMatrix.at<double>(0, 0);
		// new
		double sx = pixel_size.at<double>(0, 0);
		cam.AdjustF(sx);
}


void
WriteCameraYAML( mistl::OCV_Camera       &cam, const char *fn)
{
//        cv::FileStorage fs2(fn, cv::FileStorage::WRITE);
//        fs2 << "Intrinsics" << cam.cameraMatrix;
//        fs2 << "tvec" << cam.tvec;
//        fs2 << "rvec" << cam.rvec;
//        fs2 << "Distortion" << cam.distCoeffs;
////           fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;




	double fx = cam.cameraMatrix.at<double>(0, 0);
			double fy = cam.cameraMatrix.at<double>(1, 1);
	        cv::FileStorage fs2(fn, cv::FileStorage::WRITE);
	        fs2 << "Intrinsics" << cam.cameraMatrix;
	        fs2 << "tvec" << cam.tvec;

	        cv::Mat rotMat = cv::Mat(3,3,CV_64F,0.0);
	        cv::Rodrigues(cam.rvec, rotMat);
	//        fs2 << "rvec" << cam.rvec;
	        fs2 << "rmat" << rotMat;

	        fs2 << "Distortion" << cam.distCoeffs;
	//           fs << "cameraMatrix" << cameraMatrix << "distCoeffs" << distCoeffs;

	        // store camera resolution
	        cv::Mat resolution = cv::Mat(1,3,CV_32S,0.0);
	        resolution.at<int>(0, 0) = cam.nx;
			resolution.at<int>(0, 1) = cam.ny;
			resolution.at<int>(0, 2) = 1;
			fs2 << "resolution" << resolution;

			// store camera pixel size
			cv::Mat pixel_size = cv::Mat(1,2,CV_64F,0.0);
			pixel_size.at<double>(0, 0) = cam.sx;
			pixel_size.at<double>(0, 1) = cam.sy * fy / fx;
			fs2 << "pixel_size" << pixel_size;

}

}

