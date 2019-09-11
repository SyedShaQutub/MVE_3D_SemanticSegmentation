
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

#include        "mistl/Error.h"
#include        "mistl/FeatureValidation.h"
#include        "mistl/Line3.h"
#include 		<math.h>
#include <vector>
#include 	"opencv2/opencv.hpp"
#include <stdio.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>


namespace mistl {

FeatureValidation::FeatureValidation() {

	inititalized = false;
	cameras = 0;
	verticalBins = 3;
	horizontalBins = 4;
 	minDepth = 0;
 	maxDepth = 10;
 	depthBins = 4;
}

FeatureValidation::~FeatureValidation() {

	this->camDimensions.clear();
}

bool FeatureValidation::sufficientFeaturesFound() {

	return true;
}

void FeatureValidation::validateFeatureDistribution() {

}

void FeatureValidation::setWidth(int imgWidth, int camId) {

}

void FeatureValidation::setHeight(int imgHeight, int camId) {

}

void FeatureValidation::setDim(int imgWidth, int imgHeight, int camId) {

}

void FeatureValidation::setCamCount(int cameras){

}

void FeatureValidation::printHistogram() {

	std::cout << "Feature-histograms: \n\n " << std::endl;

	for(int i=0; i<cameras; i++) {

		int featCount = 0;
		for(int k=0; k<verticalBins; k++) {
			for(int j=0; j<horizontalBins; j++) {

				featCount = featCount + histograms2d[i][k][j];
			}
		}

		std::cout << "camera " << i << ": " << featCount << std::endl;
		for(int k=0; k<verticalBins; k++) {
			for(int j=0; j<horizontalBins; j++) {

				printf ("%*d", 8, histograms2d[i][k][j]);
			}
			std::cout << std::endl;
		}
	}
}

void FeatureValidation::addFeatures(std::vector<mistl::Track> featureTracks) {

	MISTL_ASSERT(featureTracks.size() == cameras, "amount of TrackingPointLists does not match amount of cameras");


//	trackingFeatures.push_back(features);

	for(int i=0; i<(int)featureTracks.size(); i++) {

		MISTL_ASSERT(featureTracks[i].plist.size() == featureTracks[0].plist.size(), "Feature Tracks do not have the same amount of features.");

		mistl::Track camTrack = featureTracks[i];
		int featCount = 0;
		for(int j=0; j<(int)camTrack.plist.size(); j++) {
			cv::Point2f featurePos = camTrack.plist[j];

			int imgWidth = camDimensions[i]->width;
			int imgHeight = camDimensions[i]->height;
			if( camTrack.status.at(j) != 0 && featurePos.x >= 0 && featurePos.y >= 0 &&
					featurePos.x < imgWidth && featurePos.y < imgHeight) {
				float horzBinSize = (float)imgWidth / (float)horizontalBins;
				float vertBinSize = (float)imgHeight / (float)verticalBins;
				float tmpFeaturePosX = featurePos.x ;
				float tmpFeaturePosY = featurePos.y ;

				int horzBin = (int) floor(tmpFeaturePosX / horzBinSize);
				int vertBin = (int) floor(tmpFeaturePosY / vertBinSize);

//				std::cout << "cam " << i << std::endl;
//				std::cout << "horzBin " << horzBin << std::endl;
//				std::cout << "vertBin " << vertBin << std::endl;
//				std::cout << "horzBinSize " << horzBinSize << std::endl;
//				std::cout << "vertBinSize " << vertBinSize << std::endl;
//				std::cout << "featurePos.x " << featurePos.x << std::endl;
//				std::cout << "featurePos.y " << featurePos.y << std::endl;
	//			std::cout << "tmpFeaturePosX " << tmpFeaturePosX << std::endl;
	//			std::cout << "tmpFeaturePosY " << tmpFeaturePosY << std::endl;

//				if(camTrack.status.at(j) != 0)
//					std::cout << "camTrack.pidlist[j] " << j << std::endl;


				MISTL_ASSERT(tmpFeaturePosX >= 0, "Feature x-position may not be nagetive.");
				MISTL_ASSERT(tmpFeaturePosY >= 0, "Feature y-position may not be nagetive.");


				histograms2d[i][vertBin][horzBin]++;
				featCount++;
			}
		}
		std::cout << "cam " << i << ": " << featCount << " features added." << std::endl;
	}
}

void FeatureValidation::initCameras(const std::vector<cv::Mat > &camImgList) {

	if(inititalized)
		return;

	cameras = (int)camImgList.size();

	for(int i=0; i<cameras; i++) {
		cv::Mat tmpImg= camImgList[i];

		cv::Size *camSize = new cv::Size();
		camSize->width = tmpImg.cols;
		camSize->height = tmpImg.rows;

		camDimensions.push_back(camSize);

		std::vector< std::vector<int> > histogram2d;
		histogram2d.reserve(verticalBins);
		for(int k=0; k<verticalBins; k++) {
			std::vector<int> horzBin;
			horzBin.reserve(horizontalBins);
			for(int j=0; j<horizontalBins; j++) {
				horzBin.push_back(0);
			}
			histogram2d.push_back(horzBin);
		}
		histograms2d.push_back(histogram2d);
	}

	inititalized = true;
}


}
