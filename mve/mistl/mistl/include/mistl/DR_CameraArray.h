

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


#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
//#include	"mistl/OCV_Camera.h"
#include	<stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include	<string>

#ifndef __MISTL_DR_CameraArray_H__
#define __MISTL_DR_CameraArray_H__


namespace mistl {

#ifdef _MSC_VER
#pragma warning( disable : 4244 ) // conversion from 'const double' to 'float', possible loss of data
#endif
  

 


/*! \class DR_CameraArray DR_CameraArray.h
  \brief Camera Array of 3 
*/
class	DR_CameraArray {
public:
  DR_CameraArray() 
  {
    refCam = 2;
    calibration_version = 3;
    camType="DavisReef";
    calibration_type="FACTORY";
    number_of_cameras=3;
    number_of_views=18;
    rms_reprojection_error=3.6730694222960875e-01;


    camera.clear();
    camera.push_back( mistl::Camera() );
    camera.push_back( mistl::Camera() );
    camera.push_back( mistl::Camera() );
  }

  DR_CameraArray(mistl::DR_CameraArray &cam_arr) {
    Copy(cam_arr);
  }
  
  mistl::DR_CameraArray& operator=(const mistl::DR_CameraArray& other) { Copy(other); return *this;}
  
  void Copy( const mistl::DR_CameraArray &cam_arr ) {
	  refCam = cam_arr.refCam;
	  calibration_version = cam_arr.calibration_version;
	  camType = cam_arr.camType;
	  calibration_type = cam_arr.calibration_type;
	  number_of_cameras = cam_arr.number_of_cameras;
	  number_of_views = cam_arr.number_of_views;
	  rms_reprojection_error = cam_arr.rms_reprojection_error;

	  cameraMatrix = cam_arr.cameraMatrix;
	  distCoeffs = cam_arr.distCoeffs;
	  tvec = cam_arr.tvec;
	  rotmat = cam_arr.rotmat;
          
          for(unsigned i=0; i<number_of_cameras; ++i)
            this->camera[i].Copy(cam_arr.camera[i]);
  }

public:
//   mistl::Camera      camera[3];
  std::vector<mistl::Camera>    camera;

  // properties from the DR camera file
  unsigned refCam;
  int calibration_version;
  std::string camType;
  std::string calibration_type;
  /*const*/ unsigned number_of_cameras;
  int number_of_views;
  double rms_reprojection_error;
//   unsigned NumberOfCameras() const { return 3;}
  
  // these are set after read
    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;
    cv::Mat tvec;
    cv::Mat rotmat;

};

void	ReadCameraYAML( mistl::DR_CameraArray	&cam, const char *fn, bool set_only_k1k2 = false );

void WriteCameraYAML( mistl::DR_CameraArray &cam, const char *fn);

}

#endif
