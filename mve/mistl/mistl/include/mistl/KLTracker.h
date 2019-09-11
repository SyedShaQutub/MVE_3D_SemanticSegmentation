

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


#ifndef __MISTL_KLTRACKER_H__
#define __MISTL_KLTRACKER_H__


// #include        "mistl/Vector3.h"
// #include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {

  
/*!
  \class KLTPyramid KLTracker.h
  \brief Pyramid class using OpenCV KLT
  \author Oliver Grau, Intel 2014
*/

class KLTPyramid 
{
public:
  KLTPyramid();
  ~KLTPyramid();
};

/*!
  \class KLTracker KLTracker.h
  \brief Tracking class using OpenCV KLT
  \author Oliver Grau, Intel 2014
*/

class KLTracker 
{
public:
  KLTracker();
  
  
  void  Track(   
              const cv::Mat & target,
              const std::vector<cv::Point2f> &cornersA,
              std::vector<cv::Point2f> &cornersB,
              std::vector<uchar> &status,
              std::vector<float> error
        );
  
  void FilterOutliers( 
      const std::vector<cv::Point2f> &cornersA,
      const std::vector<cv::Point2f> &cornersB,
      std::vector<uchar> &status,
      double  maxdist = -1.0,
      cv::Mat *fundamental_matrix =0 );
  
  //! Set reference image
  void        SetRefImage( cv::Mat refimage ) { refimage_ptr= refimage; }
  
  //! Helper: draw features and track into image
  // scaleX and scaleY are the scale factors of image 2, in
  // which cornersB have been detected, with respect to image 1
  // in which cornersA were detected.
  void MarkMatches( 
        cv::Mat & imgC,
      const std::vector<cv::Point2f> &cornersA,
      const std::vector<cv::Point2f> &cornersB,
      const std::vector<uchar> &status,
      float scaleX = 1.0f,
      float scaleY = 1.0f
              ) ;
              
  
  
  ////////////
public:
  cv::Mat refimage_ptr;
  int winsize;
  int maxlvl ;
};



}

#endif


