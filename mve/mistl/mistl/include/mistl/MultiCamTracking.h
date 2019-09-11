

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


#ifndef __MISTL_MultiCamTracking_H__
#define __MISTL_MultiCamTracking_H__


#include        "mistl/Vector3.h"
#include        "mistl/Log.h"
#include        "mistl/Camera.h"
#include        "mistl/Track.h"
#include        "ReadCoordinateFile.h"
#include 		"mistl/FeatureValidation.h"
#include        "Quality.h"
#include        "KLTracker.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {


/*!
  \class MultiCamTrackingBase MultiCamTracking.h
  \brief Data container for multicamera tracking
  \author Oliver Grau, Intel 2014
*/

class MultiCamTrackingBase 
{
public:
//     MultiCamTracking();
//     ~MultiCamTracking();
    
    MultiCamTrackingBase() {}
    MultiCamTrackingBase( const MultiCamTrackingBase &t) { (*this)=t; }
    mistl::MultiCamTrackingBase& operator=(const MultiCamTrackingBase &t) { 
      tracklist=t.tracklist;
      return *this;
    }
    
    std::vector<Track>  tracklist;      //!< track per frame
    
};




/*!
  \class TrackingPoint MultiCamTracking.h
  \brief tracking pont
  \author Oliver Grau, Intel 2014
*/

class TrackingPointMeta 
{
public:
  TrackingPointMeta() { id=fcount=0; first=last=0; conf= .0f; hier=0;}
  TrackingPointMeta(unsigned inhier) { id=fcount=0; first=last=0; conf= .0f; hier=inhier;}
public:
  unsigned  id;
  unsigned fcount;       //!< counter how often found
  unsigned hier;
  unsigned first, last;
  float conf;
};


/*!
 * \class TrackingPointList MultiCamTracking.h
 * \brief tracking pont list
*/
class TrackingPointList 
{
public:
  void Copy( const std::vector<cv::Point2f> &iplist, unsigned hier=0 ) {
    Clear();
    Add( iplist, hier);
  }
  void Add( const std::vector<cv::Point2f> &iplist, unsigned hier=0 ) {
//     plist = iplist;
    double scale = (1 << hier);  
    std::cout <<"TPL::Copy h:"<<hier<<" sc:"<<scale<<" n:"<< iplist.size()<<std::endl;
    for(unsigned id=0; id<iplist.size(); ++id) {
        plist.push_back( iplist[id]*scale );
        status.push_back( 255 );
        metalist.push_back( TrackingPointMeta(hier) );

        imgFeatures[iplist[id].x*scale][iplist[id].y*scale] = (int)plist.size()-1;
//        std::cout <<"plist.size(): " << plist.size() << "; status.size(): " << status.size() <<std::endl;
        MISTL_ASSERT(plist.size() == status.size(), "vectors do not have same size");
        MISTL_ASSERT(metalist.size() == status.size(), "vectors do not have same size");
      }
  }
  void Clear() {
	  metalist.clear();
	  plist.clear();
	  status.clear();

	  for(int i=0; i<imgDimX; i++) {
	    for(int j=0; j<imgDimY; j++) {
	      imgFeatures[i][j] = -1;
	    }
	  }
  }
  
  // ####################################################
  // This methods seem to have nasty side-effects? It looks it should two separate methods SetImageSize() + BuildFeatureTable (or somethng)
  // The 'table' is only used in MultiCamTracking.cc to find stable feature, so should be local in that method!!!!
  // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
  void setImgSize(int x, int y) {

	  imgDimX = x;
	  imgDimY = y;
	  imgFeatures.reserve(imgDimX);
	  for(int i=0; i<imgDimX; i++) {
		  std::vector<int> col;
		  for(int j=0; j<imgDimY; j++) {
			  col.push_back(-1);
		  }
		  imgFeatures.push_back(col);
	  }
  }

  void    PointList( std::vector<cv::Point2f> & list, std::vector< unsigned > & idlist = *((std::vector< unsigned >*)0) ) const {
    list.clear();
    if( &idlist ) idlist.clear();
    for(unsigned id=0; id<plist.size(); ++id) {
      if( ! status.at(id)) break;
      list.push_back( plist.at(id) );
      if( &idlist ) idlist.push_back( id );
    }
  } 
public:
  std::vector<TrackingPointMeta>  metalist;
  std::vector<cv::Point2f> plist;
  std::vector<uchar> status;
  std::vector< std::vector<int> > imgFeatures;
  int imgDimX;
  int imgDimY;

};




/*!
  \class MultiCamTracking MultiCamTracking.h
  \brief Class for multicamera tracking
    \author Oliver Grau, Intel 2014
*/

class MultiCamTracking
{
public:
  MultiCamTracking();
  ~MultiCamTracking();
  
  
  
  //! Clear tracking data
  void  Clear();
  
  /** \brief Find iniatal 2d point to track
   * 
   * Initialize list of 2d points to track. This is implmented by calling cv::goodFeaturesToTrack. 
   */
  void FindFeaturesToTrack( const cv::Mat &refimg, 
                           mistl::TrackingPointList &tplist,
                int maxCorners, //!< max number of corner features to extract in first image
                  double qualityLevel, //!< quality level see: cv::goodFeaturesToTrack
                  double minDistance  //!< min. distance between features see: cv::goodFeaturesToTrack
  );
  
 
  /** \brief Compute corner features in first image and find correspondences in other images by KLT
   * 
   * Perform tracking between images. The forst image is used as reference. 
   * The method calls Clear() and then FindFeaturesToTrack() with the parameters in maxCorners, qualityLevel, minDistance
   */
  void InfraTracking( const std::vector<cv::Mat >   &imglist   //!< list with input images. Need to be same size and type 8-bit (grey)
//                       mistl::TrackQuality  & q   //!< Return quality of tracking
                    );
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
    MISTL_ASSERT( camtracklist.size(), "MultiCamTracking::Track: camtracklist not initalised");
    MISTL_ASSERT( camid<camtracklist.size(), "MultiCamTracking::Track: camid out of range");
    MISTL_ASSERT( seqid<camtracklist[camid].tracklist.size(), "MultiCamTracking::Track:tracklist seq error");
    return camtracklist[camid].tracklist[seqid];
  }
  const mistl::Track &Track( unsigned camid, unsigned seqid ) const {
    MISTL_ASSERT( camtracklist.size(), "MultiCamTracking::Track: camtracklist not initalised");
    MISTL_ASSERT( camid<camtracklist.size(), "MultiCamTracking::Track: camid out of range");
    MISTL_ASSERT( seqid<camtracklist[camid].tracklist.size(), "MultiCamTracking::Track:tracklist seq error");
    return camtracklist[camid].tracklist[seqid];
  }  
  //!< write correspondences into set of coordinate files 
  void WriteCoordinateFile( const char *patt = "coordinate-cam%02d.coord",   //!< printf pattern to generate filenames
                            bool multiple_file_per_camera = true,            //!< write one coordinate file per camera
                            unsigned id_offset = 0      //!< point id/label offset
            ) const ;

  
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
  std::vector<mistl::Camera>          camlist;        //!< list of cameras. cam[0] is reference
  std::vector<mistl::Vector3f>      P3list;           //!< 3d points (assumed static)
  std::vector<MultiCamTrackingBase>   camtracklist;   //!< tracks per camera
  mistl::KLTracker   tracker; //!< this is used with cam[refcam] as reference
  unsigned refcam;      //!< reference camera index
  unsigned      tracksizex, tracksizey; //!< use this size for correspondence analysis
  // parameters used by FindFeaturesToTrack()
  int m_maxCorners; //!< max number of corner features to extract in first image
  double m_qualityLevel; //!< quality level see: cv::goodFeaturesToTrack
  double m_minDistance;
  bool  filteroutliers; //!< Filter outliers by f-matrix analysis
  int featureLevels; //!< extract feature
  unsigned winsize;     //!< Window size for KLT tracker
  unsigned      minnumberoffeatures;    //!< minimal number of features to achive good quality
  
  mistl::TrackingPointList   trackdata;

  bool useFeatureValidation;

private:
  mistl::FeatureValidation featureValidation;
};


//! Count active points, i.e. status!=0
unsigned ActivePoints( const std::vector<uchar> &status );
  

}

#endif


