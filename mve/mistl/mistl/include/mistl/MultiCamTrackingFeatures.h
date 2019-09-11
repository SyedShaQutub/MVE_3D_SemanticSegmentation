

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


#ifndef __MISTL_MultiCamTrackingFeatures_H__
#define __MISTL_MultiCamTrackingFeatures_H__


#include        "mistl/Vector3.h"
#include        "mistl/Log.h"
#include        "mistl/Camera.h"
#include        "mistl/Track.h"
#include        "mistl/MultiCamTracking.h"
#include        "ReadCoordinateFile.h"
#include 		"mistl/FeatureValidation.h"
#include        "Quality.h"
#include        "KLTracker.h"
#include        "FeatureList.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {





/*!
  \class MultiCamTrackingFeatures MultiCamTrackingFeatures.h
  \brief Class for multicamera tracking
    \author Oliver Grau, Intel 2014
*/

class MultiCamTrackingFeatures
{
public:
  MultiCamTrackingFeatures();
  ~MultiCamTrackingFeatures();
  
  
  
  //! Clear tracking data
  void  Clear();
  
  /** \brief Find iniatal 2d point to track
   * 
   * Initialize list of 2d points to track. This is implmented by calling cv::goodFeaturesToTrack. 
   */
//   void FindFeaturesToTrack( const cv::Mat &refimg, 
//                            mistl::TrackingPointList &tplist,
//                 int maxCorners, //!< max number of corner features to extract in first image
//                   double qualityLevel, //!< quality level see: cv::goodFeaturesToTrack
//                   double minDistance  //!< min. distance between features see: cv::goodFeaturesToTrack
//   );
  
 
  /** \brief Compute corner features in first image 
   * 
   */
  void PrepareTracking( const std::vector<cv::Mat >   &imglist   //!< list with input images. Need to be same size and type 8-bit (grey scale)
   );
  
  /** \brief find correspondences in other images 
   * 
   * Perform tracking between images. The forst image is used as reference. 
   * The method calls Clear() and then FindFeaturesToTrack() with the parameters in maxCorners, qualityLevel, minDistance
   */
  void InfraMatching( 
   );
  
  
  //! Triangulate 2d correspondences and return median projection error (min. of three cameras)
  float ReprojectionTest(unsigned *n_ptr = 0);
  
  /*! \brief Create 3d points 
   * 
   * It is assumed this method is called at the first frame (seq==0).
   */
  void GeneratePoints ( 
                        bool triangulationinit=true,  //!< use camera (1st + 2nd) to generate 3d points, otherwise back-project with radius len
                        float pointdistance = 3.0       //!< radius for back projection
       );
  /*! \brief Prune feature points without correspondences
   * 
   * This method compacts the track list by removing points without correspondences. 
   * It is assumed this method is called at the first frame (seq==0).
   */
  void PrunePoints (  );
    
    //! Return track for camera,frame
  mistl::Track &Track( unsigned camid, unsigned seqid ) {
    MISTL_ASSERT( camtracklist.size(), "MultiCamTrackingFeatures::Track: camtracklist not initalised");
    MISTL_ASSERT( camid<camtracklist.size(), "MultiCamTrackingFeatures::Track: camid out of range");
    MISTL_ASSERT( seqid<camtracklist[camid].tracklist.size(), "MultiCamTrackingFeatures::Track:tracklist seq error");
    return camtracklist[camid].tracklist[seqid];
  }
  const mistl::Track &Track( unsigned camid, unsigned seqid ) const {
    MISTL_ASSERT( camtracklist.size(), "MultiCamTrackingFeatures::Track: camtracklist not initalised");
    MISTL_ASSERT( camid<camtracklist.size(), "MultiCamTrackingFeatures::Track: camid out of range");
    MISTL_ASSERT( seqid<camtracklist[camid].tracklist.size(), "MultiCamTrackingFeatures::Track:tracklist seq error");
    return camtracklist[camid].tracklist[seqid];
  }  
  //!< write correspondences into set of coordinate files 
  void WriteCoordinateFile( const char *patt = "coordinate-cam%02d.coord",   //!< printf pattern to generate filenames
                            bool multiple_file_per_camera = true,            //!< write one coordinate file per camera
                            unsigned id_offset = 0      //!< point id/label offset
            ) const ;
  	// refactored to FeatureList
//  void removeFPMatchDistRatio(const std::vector<std::vector<cv::DMatch> > &knnMatches, std::vector< cv::DMatch > &outMatches);
  
  //!< Generate a list of 3d to 2D correspondences
  void  GenerateCoordinateList( unsigned camera_no, //!< camera
                                std::vector<mistl::CoordinateListEntry> &clist 
                              ) const;
  
  void Info(bool fulllist=true) const;

  
  void ComputeQuality( 
                      mistl::TrackQuality  & q   //!< Return quality of tracking
                    ) const;

  //! Write parameters to yaml-file stream
  void Store( cv::FileStorage &fs );
  //! Read parameters from yaml-file stream
  void ReStore( cv::FileNode &tm ) ;
    
  bool  DebugMode() const { return mistl::DebugMode(); }
  /////////////////////////////
public:
//   bool verbose;
  std::vector<mistl::Camera>          camlist;        //!< list of cameras. cam[refcam] is reference
  std::vector<mistl::Vector3f>      P3list;           //!< 3d points (assumed static)
  std::vector<MultiCamTrackingBase>   camtracklist;   //!< tracks per camera
  
  std::vector<mistl::FeatureList>  fdlist;      //!< Store keypoints and descriptors
  
  
  mistl::KLTracker   tracker; //!< this is used with cam[refcam] as reference
  
  unsigned refcam;      //!< reference camera index
  unsigned      tracksizex, tracksizey; //!< use this size for correspondence analysis
  // parameters used by FindFeaturesToTrack()
  int maxCorners; //!< max number of corner features to extract in reference image
  float featurefactor;   //!< extract more features in non-reference images 
  
  // the following parameters are not used in feature matching 
  double qualityLevel; //!< quality level see: cv::goodFeaturesToTrack
  double minDistance;
  bool  filteroutliers; //!< Filter outliers by f-matrix analysis
  int featureLevels; //!< extract feature
  unsigned winsize;     //!< Window size for KLT tracker
  unsigned      minnumberoffeatures;    //!< minimal number of features to achive good quality
//  float m_distanceRatio;			// refactored to FeatureList
  
  int useHistFeatureMatching;
  int maxKeyPointsPer2DBin;
  int maxKeyPointsPerDepthBin;
  int binsDepth;
  int bins2D;

  mistl::TrackingPointList   trackdata;
  std::vector<cv::Mat >   scaledimglist;       // keep list of input image for debugging (mark matches)
  std::vector<mistl::Vector2f> scalelist;
  
  bool useFeatureValidation;
	
	std::string featureDetectionAlgorithm;
	float featureRansacOutlierThreshold;

private:
};


//! Count active points, i.e. status!=0
// unsigned ActivePoints( const std::vector<uchar> &status );
  

}

#endif


