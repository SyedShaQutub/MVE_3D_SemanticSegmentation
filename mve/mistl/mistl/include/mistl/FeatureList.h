

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


#ifndef __MISTL_FeatureList_H__
#define __MISTL_FeatureList_H__


#include        "mistl/Vector3.h"
#include        "mistl/Log.h"
#include        "mistl/Camera.h"
#include        "mistl/Track.h"
#include        "mistl/MultiCamTracking.h"
#include        "ReadCoordinateFile.h"
#include 		"mistl/FeatureValidation.h"
#include        "mistl/Quality.h"
#include        "mistl/Histogram.h"
#include        "Quality.h"
#include        "KLTracker.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
//#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {

enum FeatureDetectionAlgo
{
	FDA_UNKNOWN,
	FDA_AKAZE,
	FDA_FAST_AND_FREAK
};

/*!
  \class FeatureList FeatureList.h
  \brief Class for multicamera tracking
    \author Oliver Grau, Intel 2014
*/
class FeatureList
{
public:
  FeatureList(const mistl::Camera &icamera)
	{
		camera = icamera;
  }

  FeatureList(const std::vector<cv::KeyPoint> &ikeypoints, const cv::Mat &idescriptors, const mistl::Camera &icamera)
	{
    keypoints = ikeypoints;
    descriptors = idescriptors;
    camera = icamera;
  }
  
  /*! \brief detects features
   *
   * 		Detects features using either AKAZE or FAST & FREAK
   */
  void Detect(const cv::Mat & target, FeatureDetectionAlgo algo);
  
  /*! \brief Feature matcher for two FeatureLists
   *
   * 		Matches this FeatureList object against the one
   * 		stated as parameter.
   */
  void Match(mistl::FeatureList fList, float ransacOutlierThreshold);
             
  void ConvertToPointList(std::vector<cv::Point2f> &cornersA, std::vector<uchar> &statusA );

protected:
  
	void DetectAKAZE(const cv::Mat & target);
	void DetectFASTandFREAK(const cv::Mat & target);

  /*! \brief sorts an vector of keypoints depending on their response value
  *		Quicksort algorithm for sorting keypoints.
  *		Keypoints with high response value reside at the beginning of the vector,
  *		keypoints with lower value will reside at the end of the vector.
  */
  void sortKeyPoints(int left, int right);

public:

  void init2dHistogram(unsigned horizontalBins, unsigned vericalBins, unsigned featuresPerBin);

  void init3dHistogram(unsigned depthBins, unsigned featuresPerBin);

 /*! \brief filter method for cv::keyPoints based on histograms
  *
  *		filter method
  */
  bool retainBestKeypoints(const cv::Mat &image, const mistl::Camera cam);
  
  std::vector<cv::KeyPoint> keypoints;
  cv::Mat descriptors;

	mistl::Camera camera;               //!< Camera

  std::vector<cv::DMatch> matches;		//!< List of matches when calling Match() with another FeatureList
  std::vector<cv::Point2f> plist;
  std::vector<uchar> status;

  mistl::Histogram     histogram3d;
  mistl::Histogram     histogram2d;		//!< histogram that is created after keypoint detection.
  unsigned maxKeyPointsPer2DBin;
  unsigned maxKeyPointsPerDepthBin;
};

} 

#endif


