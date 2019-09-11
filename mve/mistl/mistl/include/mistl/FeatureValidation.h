

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
//       @author Jonas Scheer
//


#ifndef __MISTL_FEATURE_VALIDATION_H__
#define __MISTL_FEATURE_VALIDATION_H__


 #include        "mistl/Track.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {

/**
 *
 */
class FeatureValidation
{
public:
	FeatureValidation();
	~FeatureValidation();

	/**
	 *
	 */
	bool sufficientFeaturesFound();

	/*
	 *
	 */
	void validateFeatureDistribution();

	/*! \brief adds features for all cameras
	 *	Adds a set of features for all the cameras.
	 *	If we have 3 cameras for example then featureTracks should contain 3 mistl::Track
	 */
	void addFeatures(std::vector<mistl::Track> featureTracks	//! vector containing one feature track for each camera
			);

	/**
	 * adds
	 */
//	void addFeatures(std::vector<mistl::TrackingPointList>);

	/* ! \ brief sets the width of the images that are used
	 *
	 */
	void setWidth(int imgWidth, 								//! image width
			int camId											//! id of camera
			);


	/* ! \brief sets the height of the images that are used
	 *
	 */
	void setHeight(int imgHeight,								//! image height
			int camId											//! id of camera
			);


	/* ! \brief sets width and height of the images that are used
	 *
	 */
	void setDim(int imgWidth,									//! image width
			int imgHeight,										//! image height
			int camId											//! camera id
			);

	/*
	 * initializes the used cameras with a list of corresponding images
	 * to set the width and height of each camera. This method will
	 * be executed only once. To reinitialize the Feature validation,
	 * first set the 'inititalized' attribute to false.
	 */
	void initCameras(const std::vector<cv::Mat >   &imglist);

	/* ! \brief sets the amount of cameras used
	 *
	 */
	void setCamCount(int cameras);

	/* ! \brief prints a 2D histogram
	 *
	 *
	 */
	void printHistogram();

	bool inititalized;

	int cameras;

	std::vector<cv::Size*> camDimensions;

private:

	//
	// sequence of tracking data that belongs toghether.
	// Each sequence element has N tracking objects (with N = no. Cameras)
	// The tracking objects of one camera contain the features found
	// in the corresponding image.
	// featurePoints[0] holds matched feature of the the N cameras from scene 0
	// featurePoints[1] holds matched feature of the the N cameras from scene 1
	// featurePoints[2] holds matched feature of the the N cameras from scene 2
	//
//	std::vector< std::vector<TrackingPointList> > trackingFeatures;

	std::vector<mistl::Track> featureTraks;

	/**
	 * amount of horizontal and vertical subdivions of the image
	 * for creating tjhe feature distribution histogram (histgram2d).
	 */
	int verticalBins;
	int horizontalBins;
	std::vector< std::vector<std::vector<int> > > histograms2d;

	//! each 2d bin is related to
	double minDepth;	//! minimal depth in meters for which histograms are build
	double maxDepth;	//! maximum depth in meters for which histograms are build
	int depthBins;
};

}
#endif
