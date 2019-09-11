

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
//       @author Oliver Grau
//


#ifndef __MISTL_FEATURETRACKER_H__
#define __MISTL_FEATURETRACKER_H__


// #include        "mistl/Vector3.h"
// #include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include        "mistl/KLTracker.h"
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {



/*!
  \class FeatureTracker FeatureTracker.h
  \brief Tracking class using OpenCV KLT
  \author Oliver Grau, Intel 2014
*/

class FeatureTracker : public KLTracker
{
public:
  FeatureTracker();
  
  
  virtual void  Track(   
              const cv::Mat & target,
              const std::vector<cv::Point2f> &cornersA,
              std::vector<cv::Point2f> &cornersB,
              std::vector<uchar> &status,
              std::vector<float> error
        );
  
 
  
  ////////////
public:
 
};



}

#endif


