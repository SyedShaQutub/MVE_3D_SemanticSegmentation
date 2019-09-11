

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

#ifndef __MISTL_UnDistortImage_incl
#define __MISTL_UnDistortImage_incl


#include        "mistl/Camera.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>


namespace mistl {




/**
 * @ingroup MISTLCFunction
 * @brief Undistort an image and compute adjust camera parameter.
 * @author O. Grau
*/
void UnDistortImage( 
  const mistl::Camera & incam,
  const cv::Mat & inimage,
  const mistl::Camera & outcam,
  cv::Mat & outimage
) ;



}

#endif