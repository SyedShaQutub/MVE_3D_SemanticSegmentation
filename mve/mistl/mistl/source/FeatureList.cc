

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
//	 @author Jonas Scheer
//

#include "mistl/FeatureList.h"
#include "mistl/Error.h"
#include "mistl/Log.h"
#include "mistl/ProjectionMapping.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include <vector>
#include <list>
#include <cctype>

#include <AKAZE.h>

#define FAST_INITIAL_THRESHOLD      20
#define FAST_MINIMUM_THRESHOLD      4
#define FAST_MAXIMUM_THRESHOLD      100

#define FAST_MINIMUM_FEATURES       800
#define FAST_MAXIMUM_FEATURES       1200
#define FAST_GRID_SIZE              4
#define FAST_NO_ITERATIONS          20

#define CORNER_SUBPIX_WINDOWS_SIZE  4
#define CORNER_SUBPIX_NO_ITERATIONS 30

#define AKAZE_DTHRESHOLD            0.0005f

namespace mistl
{

bool mistl::FeatureList::retainBestKeypoints(const cv::Mat &image, const mistl::Camera cam)
{
#ifdef BDEBUG
	std::cout << "\nKeypoint filtering: \n\n" << std::endl;
#endif

	unsigned maxKeyPointsPerBin = maxKeyPointsPer2DBin;
	std::vector< cv::KeyPoint > retainedPoints;
	cv::Mat tmpMat(cv::Size(descriptors.cols, 1), descriptors.type() );
	cv::Mat retainedDescriptors(cv::Size(descriptors.cols, 1), descriptors.type() );

	mistl::ProjectionMapping mapper;
	mapper.CopyCamera(cam);
	mapper.SetRange(mapper.cam.GetNx(), mapper.cam.GetNy(), 10.);
	mapper.SetBinning(histogram3d);

	mistl::MappingLinear linmapper;
	linmapper.SetRange(image.size().width, image.size().height, 0.);
	linmapper.SetBinning(histogram2d);

	// sort keypoints
	sortKeyPoints(0, keypoints.size()-1);


#ifdef BDEBUG
	int kk = 1;
	for(int k=0; k<keypoints.size(); k++
	{
		if(k > 0)
			MISTL_ASSERT( keypoints[k].response >= keypoints[k-1].response , "sorting was not successfull.");
	}
#endif

	retainedPoints.push_back(keypoints[0]);
	descriptors.row(0).copyTo(retainedDescriptors.row(0));

	std::cout << "\n" << std::endl;
	for(unsigned i=1; i<(unsigned)keypoints.size(); i++)
	{
		if(retainedPoints.size() >= (histogram2d.Nx()*histogram2d.Ny()*histogram2d.Nz()*maxKeyPointsPerBin))
		{
			// all histogram-bins are full
			break;
		}

		bool featureAdded = false;
		unsigned addedPoints2D = 0;
		unsigned addedPoints3D = 0;
    float px,py;
    unsigned ix,iy,iz;
		px = keypoints[i].pt.x;
		py = keypoints[i].pt.y;

		linmapper.Map(px ,py, 0.f, ix, iy, iz);
		if(histogram2d.Get(ix,iy,iz) >= maxKeyPointsPerBin)
		{
			// histogram-bin (ix,iy,iz) is full
			continue;
		}

		histogram2d.Add(ix,iy,iz);
		addedPoints2D++;
		retainedPoints.push_back(keypoints[i]);

		descriptors.row(i).copyTo(tmpMat.row(0));
		/*std::cout << "\n\nBefore: " << std::endl;
		std::cout << "descriptors.rows(" << i <<"): " << descriptors.row(i) << "\n" << std::endl;
		std::cout << "retainedDescriptors.rows(" << retainedDescriptors.rows-1 <<"): " << retainedDescriptors.row(retainedDescriptors.rows-1) << "\n" << std::endl;*/

		cv::vconcat(retainedDescriptors, tmpMat, retainedDescriptors);
		/*std::cout << "After: " << std::endl;
		std::cout << "retainedDescriptors.rows(" << retainedDescriptors.rows-1 <<"): " << retainedDescriptors.row(retainedDescriptors.rows-1) << "\n" << std::endl;
		std::cout << "\nadd feature: " << i << " with response: " << keypoints[i].response << std::endl;*/
	}

//#ifdef BDEBUG
	histogram2d.Info();
//#endif

	keypoints = retainedPoints;
	descriptors = retainedDescriptors;

	std::cout << "Histogram - keyPoints: " << keypoints.size() << std::endl;
	std::cout << "Histogram - descriptors: " << descriptors.rows << std::endl;

	return true;
}


void mistl::FeatureList::sortKeyPoints(int left, int right)
{
	int i = left, j = right;
	cv::KeyPoint tmp;
	cv::KeyPoint pivot = keypoints[(left + right) / 2];

	//std::cout << "descriptors.rows: " << descriptors.rows << "; descriptors.cols: " << descriptors.cols << "; descriptors.type:" << descriptors.type() << std::endl;
  cv::Mat tmpMat(cv::Size(descriptors.cols, 1), descriptors.type());
  //std::cout << "tmpMat.rows: " << tmpMat.rows << "; tmpMat.cols: " << tmpMat.cols << "; tmpMat.type:" << tmpMat.type() << std::endl;/    std::cout << "KeyPoints: " << keypoints.size() << std::endl;
	/* partition */
  while (i <= j)
	{
    while (keypoints[i].response > pivot.response)
			i++;
	  while (keypoints[j].response < pivot.response)
			j--;

	  if (i <= j)
		{
			tmp = keypoints[i];
			keypoints[i] = keypoints[j];
			keypoints[j] = tmp;
			/*std::cout << "\n\nBefore: " << std::endl;
			std::cout << "descriptors.rows(i): " << descriptors.row(i) << "\n" << std::endl;
			std::cout << "descriptors.rows(j): " << descriptors.row(j) << "\n" << std::endl;*/
			descriptors.row(i).copyTo(tmpMat.row(0));
			descriptors.row(j).copyTo(descriptors.row(i));
			tmpMat.row(0).copyTo(descriptors.row(j));
			/*std::cout << "After: " << std::endl;
			std::cout << "descriptors.rows(i): " << descriptors.row(i) << "\n" << std::endl;
			std::cout << "descriptors.rows(j): " << descriptors.row(j) << "\n" << std::endl;*/
			i++;
			j--;
	  }
	}

	/* recursion */
	if (left < j)
		sortKeyPoints(left, j);
	if (i < right)
		sortKeyPoints(i, right);

	return;
}

void mistl::FeatureList::Detect(const cv::Mat & target, FeatureDetectionAlgo algo)
{
	switch(algo)
	{
	case FDA_AKAZE:
		DetectAKAZE(target);
		break;
	case FDA_FAST_AND_FREAK:
		DetectFASTandFREAK(target);
		break;
	default:
		/* FIXME - bail for invalid algo */
		break;
	}
}

void mistl::FeatureList::DetectAKAZE(const cv::Mat & target)
{
	AKAZEOptions options;

#ifdef BDEBUG
	options.verbosity = true;
#endif
		
	options.dthreshold = AKAZE_DTHRESHOLD;
	options.img_width = target.cols;
	options.img_height = target.rows;
	libAKAZE::AKAZE detector(options);

	// Convert the image to float to extract features
	cv::Mat target_32;
	target.convertTo(target_32, CV_32F, 1.0 / 255.0, 0);

	detector.Create_Nonlinear_Scale_Space(target_32);
	detector.Feature_Detection(keypoints);
	detector.Compute_Descriptors(keypoints, descriptors);

// debug
#ifdef BDEBUG
  cv::Mat visimg;
			//debug
	std::cout<< "Fxt cam:"<<i<<"\n";
	for(unsigned j = 0; j < keypoints.size(); j++)
		std::cout<< "\t"<< j<<"\t:"<<keypoints.at(j).pt <<"\n";


	target.copyTo(visimg);
	for(unsigned j = 0; j < keypoints.size(); j++)
		cv::circle(visimg, keypoints[j].pt, 2, cv::Scalar(0,0,255), 1);

	std::string fn =  "trackpt_" + std::to_string(seq_no) + "_" +  std::to_string(i) + ".png";
	cv::imwrite(fn.c_str(), visimg);
#endif
  return;
}

void mistl::FeatureList::DetectFASTandFREAK(const cv::Mat & target)
{
	std::vector<cv::KeyPoint> initial_keypoints;
	std::vector<cv::Point2f> points;

	cv::Ptr<cv::FastAdjuster> adjuster(new cv::FastAdjuster(FAST_INITIAL_THRESHOLD, true, FAST_MINIMUM_THRESHOLD, FAST_MAXIMUM_THRESHOLD));
	cv::DynamicAdaptedFeatureDetector detector(adjuster, FAST_MINIMUM_FEATURES, FAST_MAXIMUM_FEATURES, FAST_NO_ITERATIONS);
	
	for (int i = 0; i < FAST_GRID_SIZE; i++)
	{
		for (int j = 0; j < FAST_GRID_SIZE; j++)
		{
			cv::Range row_range((i * target.rows) / FAST_GRID_SIZE, ((i + 1) * target.rows) / FAST_GRID_SIZE);
      cv::Range col_range((j * target.cols) / FAST_GRID_SIZE, ((j + 1) * target.cols) / FAST_GRID_SIZE);

      cv::Mat sub_target = target(row_range, col_range);
			vector<cv::KeyPoint> sub_keypoints;
			detector.detect(sub_target, sub_keypoints);

			for(vector<cv::KeyPoint>::iterator it = sub_keypoints.begin(), end = sub_keypoints.end(); it != end; it++)
			{
				it->pt.x += col_range.start;
				it->pt.y += row_range.start;
			}
			initial_keypoints.insert(initial_keypoints.end(), sub_keypoints.begin(), sub_keypoints.end());
		}
	}


	points.resize(initial_keypoints.size());
	for (int j = 0; j < initial_keypoints.size(); j++)
	{
		points[j].x = initial_keypoints[j].pt.x;
		points[j].y = initial_keypoints[j].pt.y;
	}

	cornerSubPix(target, points, cv::Size(CORNER_SUBPIX_WINDOWS_SIZE, CORNER_SUBPIX_WINDOWS_SIZE), cv::Size(-1, -1),
		cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, CORNER_SUBPIX_NO_ITERATIONS, 0.1));

	// Make sure this remains unique
	for (int k = 0; k < points.size() - 1; k++)
	{
		if (points[k].x < 0 || points[k].y < 0)
			continue;

		for (int l = k + 1; l < points.size(); l++)
		{
			if ((fabs(points[k].x - points[l].x) < 1.0f) && (fabs(points[k].y - points[l].y) < 1.0f))
				points[l] = cv::Point2f(-1.0f, -1.0f);
		}
	}

	int c = 0;

	keypoints.resize(initial_keypoints.size());
	for (int j = 0; j < initial_keypoints.size(); j++)
	{
		if (points[j].x > 0 && points[j].y > 0)
		{
			keypoints[c] = initial_keypoints[j];
			keypoints[c].pt = points[j];
			c++;
		}
	}

	keypoints.resize(c);

	// Now, run FREAK and describe the features
	cv::FREAK freak(false, false, 8.0f, 2);
	freak.compute(target, keypoints, descriptors);
}

void mistl::FeatureList::ConvertToPointList(std::vector<cv::Point2f> &cornersA, std::vector<uchar> &statusA)
{
	cornersA = plist;
	statusA = status;
}

void mistl::FeatureList::Match(mistl::FeatureList fList, float ransacOutlierThreshold)
{
	cv::BFMatcher matcher(cv::NORM_HAMMING);
	std::vector<std::vector<cv::DMatch> > rawmatches;
	matcher.knnMatch(fList.descriptors, descriptors, rawmatches, 2);

	matches.resize(rawmatches.size());
	plist.resize(rawmatches.size());
	status.resize(rawmatches.size());

	std::vector<cv::Point2f> hkpt1(rawmatches.size()), hkpt2(rawmatches.size());
	std::vector<int> hidx(rawmatches.size());
	int c = 0;

	for (int j = 0; j < matches.size(); j++)
	{
		cv::DMatch dmatch = rawmatches[j][0];
		int q = rawmatches[j][0].queryIdx;
			
		if (rawmatches[j][0].distance < 0.8f * rawmatches[j][1].distance)
		{
			int t = rawmatches[j][0].trainIdx;
			plist[q] = keypoints[t].pt;
			status[q] = 255;

			fList.camera.UnDistort(fList.keypoints[q].pt.x, fList.keypoints[q].pt.y, hkpt1[c].x, hkpt1[c].y);
			camera.UnDistort(keypoints[t].pt.x, keypoints[t].pt.y, hkpt2[c].x, hkpt2[c].y);
			hidx[c] = j;
			c++;
		}
		else
		{
			plist[q] = cv::Point2f(0.0f, 0.0f);
			status[q] = 0;
		}
	}

	hkpt1.resize(c);
	hkpt2.resize(c);
	hidx.resize(c);

	// Use RANSAC - but just to filter out the marginal outliers and not create any bias
	std::vector<uchar> mask;
	cv::Mat H = cv::findHomography(hkpt1, hkpt2, CV_RANSAC, ransacOutlierThreshold, mask);

	for (int j = 0; j < mask.size(); j++)
	{
		if (!mask[j])
		{
			int q = rawmatches[hidx[j]][0].queryIdx;
			status[q] = 0;
		}
	}
}

void mistl::FeatureList::init2dHistogram(unsigned horizontalBins, unsigned vericalBins, unsigned featuresPerBin)
{
  histogram2d.SetSize(horizontalBins, vericalBins, 1);
  maxKeyPointsPer2DBin = featuresPerBin;
}

void mistl::FeatureList::init3dHistogram(unsigned depthBins, unsigned featuresPerBin)
{
	histogram3d.SetSize(1, 1, depthBins);
	maxKeyPointsPerDepthBin = featuresPerBin;
}

}
