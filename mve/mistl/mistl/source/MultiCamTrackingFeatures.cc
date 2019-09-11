

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
//       
//


#include        "mistl/Error.h"

#include        "mistl/KLTracker.h"
#include        "mistl/MultiCamTrackingFeatures.h"
#include        "mistl/MultiCamTracking.h"
#include        "mistl/Line3.h"
#include        "mistl/Histogram.h"
#include        "mistl/ProjectionMapping.h"
#include        "mistl/Log.h"
#include        <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>
#include <string>
#include 		"opencv2/opencv.hpp"
#include 		"opencv2/core/core.hpp"

// ndk compiler does not support std::to_string
#ifdef ANDROID
#include "mistl/to_string.h"
#else
#define		to_string	std::to_string
#endif

namespace mistl {
  
MultiCamTrackingFeatures :: MultiCamTrackingFeatures()
{
//   verbose=true;
  filteroutliers=true;
  maxCorners = 1000;
  qualityLevel= 0.01; //!< quality level see: cv::goodFeaturesToTrack
  minDistance=10;
  featureLevels = 0;
  winsize= 15;
  minnumberoffeatures=40;
  refcam=0;
  tracksizex=tracksizey=0;
//   m_distanceRatio=0.8f;		// removed to FeatureList
  featurefactor= 2.f;

  useHistFeatureMatching = 0;
  maxKeyPointsPer2DBin = 40;
  maxKeyPointsPerDepthBin = 30;
  binsDepth = 3;
  bins2D = 3;

	featureDetectionAlgorithm = "AKAZE";
}

MultiCamTrackingFeatures :: ~MultiCamTrackingFeatures()
{
  Clear();
}

unsigned seq_no=0;

//
//
void MultiCamTrackingFeatures :: Info(bool fulllist) const
{
	if (DebugMode()) {
		std::cout << "MultiCamTrackingFeatures  refcam:" << refcam << " no of cameras:" << camtracklist.size();
		std::cout << "\t3-points:" << P3list.size() << "\n";
		if (fulllist) {
			for (unsigned j = 0; j < camtracklist.size(); ++j) {
				std::cout << "cam";
				if ((j + 1) < 10)
					std::cout << '0'; 
        std::cout << j + 1 << "\tpoints: ";
				for (unsigned i = 0; i < camtracklist[j].tracklist.size(); ++i) {
					const mistl::Track &track = Track(j, i);
					if (i) std::cout << ',';
					std::cout << track.ActivePoints() << '(' << track.plist.size() << ')';
				}
				std::cout << "\n";
			}
		}
	}
}





//
//
void MultiCamTrackingFeatures :: ComputeQuality(mistl::TrackQuality  & q) const
{

//   unsigned n;   //!< number of total correspondences
//   unsigned ncam;   //!< number of cameras
//   unsigned min_n;       //!< lowest number of correspondences in one camera
  q.match=0.0f; // not used at the moment
  q.n=0;
  q.ncam = (unsigned)camtracklist.size();
  q.min_n = 0;

  for(unsigned j=0; j<camtracklist.size(); ++j) {
    for(unsigned i=0; i<camtracklist[j].tracklist.size(); ++i) {
      const mistl::Track &track = Track(j,i);
      unsigned n=track.ActivePoints();
      q.n += n;
      if( n>0 && q.min_n==0) q.min_n=n;
      if(n<q.min_n) q.min_n=n;
    }
  }   
  //
  // compute quality
  //
  if(q.min_n < minnumberoffeatures) {
    std::cout<<"MCT::ComputeQuality q.min_n < minnumberoffeatures \n";
    q.quality = 0.0f;
    q.confidence = 0.0f;
    return;
  }
  unsigned t = maxCorners*q.ncam; // max expected correspondences
  
  float q1 = static_cast<float>(q.n) / static_cast<float>(t);
  // need more tweaking
  q.quality = 1.0f;
  q.confidence = q1*2.0;
  if(q.confidence >1.0f) q.confidence=1.0f;
}





void 
MultiCamTrackingFeatures :: Clear()
{
  camtracklist.clear();
  scaledimglist.clear();
  scalelist.clear();
}


void 
MultiCamTrackingFeatures :: PrepareTracking( const std::vector<cv::Mat >   &imglist )
{
  if (DebugMode()) std::cout << "InitTracks ilist:" << imglist.size() << " refcam:" << refcam << "\n";
  
  Clear();
  
  MISTL_ASSERT( imglist.size()>1 /*&& (imglist.size()==icamlist.size())*/, "MultiCamTrackingFeatures :: InitTracks error image/camera list don't match or empty");

  
  
  // generate entries
  for(unsigned i=0; i<imglist.size(); ++i) {
    if(tracksizex>0 && tracksizey>0) {
      float fx,fy;
      fx = static_cast<double>(tracksizex) / static_cast<double>(imglist.at(i).size().width);
      fy = static_cast<double>(tracksizey) / static_cast<double>(imglist.at(i).size().height);
      scalelist.push_back( mistl::Vector2f(fx,fy) );
	  if (DebugMode()) std::cout << "Resize image (" << imglist.at(i).size() << " -> " << cv::Size(tracksizex, tracksizey) << "\n";
      cv::Mat target;
      cv::resize( imglist.at(i), target , cv::Size(tracksizex, tracksizey), 0.0, 0.0, CV_INTER_CUBIC );
      scaledimglist.push_back(target);
#ifdef BDEBUG      
      std::string   fn =  "resized_" + to_string(seq_no) + "_" +  to_string(i) + ".png"  ;
      cv::imwrite( fn.c_str(), target );
#endif
    } else {
      cv::Mat cpimg;
      imglist.at(i).copyTo(cpimg);
      scaledimglist.push_back(cpimg); //imglist.at(i));
#ifdef BDEBUG      
       std::string   fn =  "resized_" + to_string(seq_no) + "_" +  to_string(i) + ".png"  ;
      cv::imwrite( fn.c_str(), cpimg );
#endif
    }

    camtracklist.push_back( MultiCamTrackingBase() );
  }
  
  
//   mistl::MultiCamTrackingBase  &base = camtracklist[0];
  
  fdlist.clear();
  
  // Get feature matching algorithm
	FeatureDetectionAlgo algo = FDA_UNKNOWN;

	if (!featureDetectionAlgorithm.compare("AKAZE"))
		algo = FDA_AKAZE;
	else if (!featureDetectionAlgorithm.compare("FAST_AND_FREAK"))
		algo = FDA_FAST_AND_FREAK;
  
	MISTL_ASSERT(algo != FDA_UNKNOWN, "Invalid feature detector!");
	MISTL_ASSERT(featureRansacOutlierThreshold > 0, "Invalid feature RANSAC outlier threshold!");
	std::cout << "Using feature detection algorithm: " << featureDetectionAlgorithm << endl;
	std::cout << "Feature matching ransac outlier threshold: " << featureRansacOutlierThreshold << endl;

  // build keypoints and descriptors
   for (unsigned int i = 0; i < scaledimglist.size(); i++) {
    if (!scaledimglist[i].data) continue; // avoid invalid, empty images

		mistl::FeatureList newFeatureList(camlist[i]);
		newFeatureList.Detect(scaledimglist[i], algo);
    if(i==refcam && useHistFeatureMatching>0) {
      newFeatureList.init2dHistogram(bins2D, bins2D, maxKeyPointsPer2DBin);
      newFeatureList.init3dHistogram(binsDepth, maxKeyPointsPerDepthBin);
      newFeatureList.retainBestKeypoints(scaledimglist[i], camlist[i]);
    }
//     newFeatureList.ComputeDescriptors(scaledimglist[i]);
    if (DebugMode()) std::cout << "cam-" <<i << " has " << newFeatureList.keypoints.size() << " features\n";
    fdlist.push_back( newFeatureList );
   }
  
}

void 
MultiCamTrackingFeatures :: InfraMatching(  )
{
  if (DebugMode()) std::cout << "InfraMatching  refcam:" << refcam << "\n";
  
  for (unsigned int i = 0; i < fdlist.size(); i++) {
    mistl::MultiCamTrackingBase  &base = camtracklist.at(i);
    base.tracklist.push_back( mistl::Track() );
    mistl::Track &track = base.tracklist.back();
    
    if (i==refcam) {
      
        std::vector<cv::Point2f> plist;
        for( unsigned j=0; j<fdlist.at(i).keypoints.size(); ++j) {
          
          plist.push_back(  fdlist.at(i).keypoints[j].pt );
        }
        track.plist = plist ;
        track.status.clear();
        
#ifdef BDEBUG
     //debug
      std::cout<< "Ref cam:\n";
#endif
    
        for( unsigned j=0; j<plist.size(); ++j) {
          track.status.push_back(255);
  
#ifdef BDEBUG
          
          //debug
          std::cout<< "\t"<< j<<"\t:"<<track.plist.at(j)<<" s:"<<(int)track.status.at(j)<<"\n";
#endif
        }

      continue; 
    }
    
    
//     mistl::MultiCamTrackingFeaturesBase  &base = camtracklist[i];
//     base.tracklist.push_back( mistl::Track() );
//     mistl::Track &track = base.tracklist.back();
//     
//     tracker.Track( scaledimglist[i], trackdata.plist, track.plist, track.status, track.error );
    
    // matching descriptors
	fdlist.at(i).Match(fdlist.at(refcam), featureRansacOutlierThreshold);
    track.plist=fdlist.at(i).plist;
    track.status=fdlist.at(i).status;


#ifdef BDEBUG 
          //debug
      std::cout<< "cam:"<<i<<"\n";
        for( unsigned j=0; j<plist.size(); ++j) {
          std::cout<< "\t"<< j<<"\t:"<<track.plist.at(j)<<" s:"<< (int)track.status.at(j)<<"\n";
        }
    
    // drawing the results
    cv::namedWindow("matches", 1);
    cv::Mat img_matches;
    cv::drawMatches( scaledimglist.at(refcam), fdlist.at(refcam).keypoints, scaledimglist.at(i), fdlist.at(i).keypoints, matches, img_matches);
    cv::imshow("matches", img_matches);
    cv::waitKey(0);
#endif
      
      
//       cv::Mat fundamental_matrix;
//       if(filteroutliers) 
//         tracker.FilterOutliers( trackdata.plist, track.plist, track.status , 2.0, &fundamental_matrix);
    
  }

  //
  // scaling of features, due to different image resolutions
  //
  if(scalelist.size()!=0) {
    float fx,fy;
    for (unsigned int i = 0; i < scaledimglist.size(); i++) {
      fx=scalelist.at(i).X();
      fy=scalelist.at(i).Y();
      mistl::MultiCamTrackingBase  &base = camtracklist[i];
      mistl::Track &track = base.tracklist.back();
     
	  if (DebugMode()) std::cout << " rescaling track-cam " << i << " fx,fy:" << 1.0 / fx << "," << 1.0 / fy << "\n";
    
      for (unsigned int j = 0; j < track.plist.size(); j++) {
        if (track.status.at(j) == 0) {
            continue;
        }
//        std::cout<< "scale " << cornersB[i];
        track.plist.at(j).x /= fx;        
        track.plist.at(j).y /= fy;
//        std::cout<< " -> " << cornersB[i]<<std::endl;

      }

    }
  }
  seq_no++;
  
}


void 
MultiCamTrackingFeatures :: PrunePoints (  )
{
  // 
  // only considers first track for now!!!!!
  // It is assumed this method is called at the first frame (seq==0)
  //
  MISTL_ASSERT( camtracklist.size()>0, "MultiCamTrackingFeatures::GeneratePoints:needs at least one image/camera");
  
  
  std::vector<uchar> allstatus;
  const unsigned len = (unsigned)Track(0,0).plist.size();
  for(unsigned i=0; i<len; ++i) {
    allstatus.push_back(0);
  }
  
  // only keep if all camera show feature
  if(true) {
    
      for(unsigned i=0; i<len; ++i) {
        bool v=true;

        for(unsigned j=0; j<camtracklist.size(); ++j) 
          if(Track(j,0).status.at(i)==0) v=false;
      
        if(!v) {
//           std::cout<<"Clear "<<i<<"\n";
          for(unsigned j=0; j<camtracklist.size(); ++j) 
            Track(j,0).status.at(i)=0;
        }
    }
  }
  
  
  for(unsigned j=1; j<camtracklist.size(); ++j) {
    for(unsigned i=0; i<len; ++i) 
      if(Track(j,0).status.at(i)>0) allstatus[i] = 1;
  }
  if (DebugMode()) std::cout << "GeneratePoints allstatus:" << mistl::ActivePoints(allstatus) << "\n";
//   unsigned maxpoints=mistl::ActivePoints(allstatus);
  
  
//     std::cout<<"GeneratePoints prune\n";
    
    for(unsigned j=0; j<camtracklist.size(); ++j) {
      std::vector<uchar> newstatus;
      std::vector<cv::Point2f>      newplist;
      mistl::Track &track = Track(j,0);
      for(unsigned i=0; i<len; ++i) {
        if(allstatus[i]!=0) {
          newplist.push_back( track.plist.at(i) );
          newstatus.push_back( track.status.at(i) );
        }
      }    

      track.plist = newplist;
      track.status = newstatus;
    }


  
}


float 
MultiCamTrackingFeatures :: ReprojectionTest (  unsigned *n_ptr   )
{
    
  bool hascameras=true;
  if(camlist.size()!=camtracklist.size()) {
    hascameras=false;
    for(unsigned j=1; j<camtracklist.size(); ++j)
      camlist.push_back( mistl::Camera());
  }

//   std::vector<mistl::Vector3f>      plist;           //!< 3d points (assumed static)
  std::vector<float>      sqdistlist;           //!< 3d points (assumed static)

  // generate points
  mistl::Track &track1 = Track( refcam,0);
  
  if ( DebugMode() ) {
    std::cout<<"ReprojectionTest\n";
  }
      
  if( n_ptr ) *n_ptr = 0;
  
  if(camtracklist.size()<3  || !hascameras ) return 0.f; 
    
  unsigned c1,c2;
  
  c1=0;
  c2=1;
  if(refcam==0) { ++c1; ++c2;}
  if(refcam==1) {  ++c2;}
    
  mistl::Track &track = Track(c1,0);
  mistl::Track &track2 = Track(c2,0);


  for(unsigned i=0; i<track.plist.size(); ++i) {
    if(track.status.at(i)==0) {
//         plist.push_back( mistl::Vector3f());
      continue;
    }
    mistl::Line3      lin1( camlist[refcam].C(), camlist[refcam].C() + camlist[refcam].LineOfSight( track1.plist[i].x, track1.plist[i].y ));
    mistl::Line3      lin2( camlist[c1].C(), camlist[c1].C() + camlist[c1].LineOfSight( track.plist[i].x, track.plist[i].y ));
    double tau;
    mistl::Vector3f P = lin1.Intersection( lin2, tau);
    float depth=camlist[refcam].A().Dot(P);
//      std::cout<< "img: " << c1 << ";feature: "<< i << " " << track.plist[i] <<" <-> " << track1.plist[i] << "\t"<<P<<"\ttau:"<<tau<< " depth:"<<depth<<"\n";

    
    if( depth>0.0f) {
//       plist.push_back( P );
      float x,y,rx,ry;
      camlist[c2].Project(P,x,y);
      rx = x-track2.plist[i].x;
      ry = y-track2.plist[i].y;

      sqdistlist.push_back( rx*rx+ry*ry );
    }
  }
  if(sqdistlist.size()<1) return 0.f;
  if( n_ptr ) *n_ptr = (unsigned)sqdistlist.size();
  std::sort(sqdistlist.begin(), sqdistlist.end());
  return  sqrtf(sqdistlist.at( sqdistlist.size()/2 )) ;

}


void 
MultiCamTrackingFeatures :: GeneratePoints (  bool triangulate, float pointlen  )
{
    
  bool hascameras=true;
  if(camlist.size()!=camtracklist.size()) {
    hascameras=false;
    for(unsigned j=1; j<camtracklist.size(); ++j)
      camlist.push_back( mistl::Camera());
  }
  
  P3list.clear();
  // generate points
  mistl::Track &track1 = Track( refcam,0);
  
  if ( DebugMode() ) {
    std::cout<<"GeneratePoints("<<triangulate<<","<<pointlen<<")\n";
    camlist[0].Info();
  }
  // initialise points 
  for(unsigned i=0; i<track1.plist.size(); ++i) {
    mistl::Vector3f P = camlist[refcam].C() + camlist[refcam].LineOfSight( track1.plist[i].x, track1.plist[i].y ) * pointlen;
    P3list.push_back( P );
  }

      
  if(camtracklist.size()==1 || !triangulate || !hascameras ) return; 
  
  // overwrite points 
  
  for(unsigned j=0; j<camtracklist.size(); ++j) {
    if(j==refcam) continue;
    
    mistl::Track &track = Track(j,0);

    std::cout << "\n\nTRACK " << j << "\n\n" << std::endl;

    for(unsigned i=0; i<track.plist.size(); ++i) {
      if(track.status.at(i)==0) continue;

      mistl::Line3      lin1( camlist[refcam].C(), camlist[refcam].C() + camlist[refcam].LineOfSight( track1.plist[i].x, track1.plist[i].y ));
      mistl::Line3      lin2( camlist[j].C(), camlist[j].C() + camlist[j].LineOfSight( track.plist[i].x, track.plist[i].y ));
      double tau;
      mistl::Vector3f P = lin1.Intersection( lin2, tau);
      float depth=camlist[refcam].A().Dot(P);
//      std::cout<< "img: " << j << ";feature: "<< i << " " << track.plist[i] <<" <-> " << track1.plist[i] << "\t"<<P<<"\ttau:"<<tau<< " depth:"<<depth<<"\n";

      P3list.at(i) = P;
      
      if( depth<0.0f) {
    	track.status.at(i)=0;
      }

#ifndef WIN32
          // ted
          // andrdoid compiler didn't like this
          // changed to isinf()
      //if( isinff(P[0]) || isinff(P[1]) || isinff(P[2]) ) {
      if (std::isinf(P[0]) || std::isinf(P[1]) || std::isinf(P[2])) {
#else
      #include <float.h>
      if (!_finite(P[0]) || _finite(P[1]) || _finite(P[2])) {
#endif
        track.status.at(i)=0;
		    track1.status.at(i)=0;
      }
    }
    break; // only one camera pair
  }  
}


void  MultiCamTrackingFeatures :: GenerateCoordinateList( unsigned j, std::vector<mistl::CoordinateListEntry> &clist ) const
{
    clist.clear();
#ifdef BDEBUG
    std::cout<<"GenerateCoordinateList "<<camtracklist[j].tracklist.size()<<"\n";
#endif   
    for(unsigned i=0; i<camtracklist[j].tracklist.size(); ++i) {
      const mistl::Track &track = Track(j,i);
      MISTL_ASSERT(track.plist.size()==P3list.size(), "MultiCamTrackingFeatures :: GenerateCoordinateList: 3point list doesn't match track list");
      for (unsigned k = 0; k < track.plist.size(); ++k){
        if (track.status.at(k) != 0) {
          mistl::CoordinateListEntry itm;
          
          itm.id = i;
          itm.P[0] = P3list[k].X();
          itm.P[1] = P3list[k].Y();
          itm.P[2] = P3list[k].Z();
          itm.ip[0] = track.plist[k].x;
          itm.ip[1] = track.plist[k].y;
          itm.pointid = k;
          clist.push_back(itm);
        }
      }
    }
  
}


void
MultiCamTrackingFeatures :: WriteCoordinateFile( const char *patt, bool multiple_file_per_camera, unsigned id_offset  ) const 
{
  int strlenPatt = (int)strlen(patt);
  char* fn = new char[strlenPatt + 200];
    
  for(unsigned j=0; j<camtracklist.size(); ++j) {
    sprintf(fn,patt,j,0);
    std::ofstream fo( fn);

    MISTL_ASSERT(!fo.fail(), "MultiCamTrackingFeatures :: WriteCoordinateFile: error opening file for writing");
    
    std::cout<<"Write coord-file:"<<fn<<"\n";
    fo << "#Coordinate list file\n#frame-id       Px Py Pz        ix iy   label\n";
    for(unsigned i=0; i<camtracklist[j].tracklist.size(); ++i) {
      const mistl::Track &track = Track(j,i);
//       MISTL_ASSERT(track.plist.size()==P3list.size(), "MultiCamTrackingFeatures :: WriteCoordinateFile: 3point list doesn't match track list");
      bool pr_3d = track.plist.size()==P3list.size();
      for (unsigned k = 0; k < track.plist.size(); ++k){
        if (track.status.at(k) != 0) {
          if( pr_3d ) {
            fo << i << '\t' << P3list[k].X() << ' ' << P3list[k].Y() << ' ' << P3list[k].Z();
          } else 
            fo << i << '\t' << "0.0 0.0 0.0";
          fo << '\t' << track.plist[k].x << ' ' << track.plist[k].y << '\t' << k+id_offset << '\n';
        }
      }
    }
  }

  delete[] fn;
//   return P3list.size();
}

//! Count active points, i.e. status!=0
// unsigned ActivePoints( const std::vector<uchar> &status )  {
//       unsigned r=0;
//       for(unsigned i=0; i<status.size(); ++i) if(status[i]!=0) ++r;
//       return r;
// }
// 


void MultiCamTrackingFeatures::Store( cv::FileStorage &fs )
{      
  
  MISTL_ASSERT( fs.isOpened(), "RunCalibration::Store: error opening parameter file for writing");


  fs << "maxCorners" << maxCorners;
  fs << "qualityLevel" << qualityLevel;
//   fs << "verbose" << static_cast<int>(verbose);
  fs << "filteroutliers" << static_cast<int>(filteroutliers);
  fs << "minDistance" << minDistance;
  fs << "winsize" << static_cast<int>(winsize);
  fs << "featureLevels" << featureLevels;
  fs << "useHistFeatureMatching" << useHistFeatureMatching;
  fs << "2dFeaturesPerBin" << maxKeyPointsPer2DBin;
  fs << "depthFeaturesPerBin" << maxKeyPointsPerDepthBin;
  fs << "depthBins" << binsDepth;
  fs << "2dBins" << bins2D;
	fs << "featureDetectionAlgorithm" << featureDetectionAlgorithm;
//   fs << "" << ;
}

//! Read parameters from file
void MultiCamTrackingFeatures::ReStore( cv::FileNode &tm )
{
//   SetDefaults();
  if(tm.empty()) return;
  int v;
  v = tm["filteroutliers"]; filteroutliers= v!=0;
//   v = tm["verbose"];  verbose= v!=0;
  qualityLevel = tm["qualityLevel"]  ;
  maxCorners = tm["maxCorners"]  ;
  minDistance = tm["minDistance"]  ;
  if(tm["featureLevels"].type()==cv::FileNode::INT) {featureLevels = tm["featureLevels"];}
  if(tm["useHistFeatureMatching"].type()==cv::FileNode::INT) {v = tm["useHistFeatureMatching"]; useHistFeatureMatching=static_cast<unsigned>(v);}
  if(tm["2dFeaturesPerBin"].type()==cv::FileNode::INT) {v = tm["2dFeaturesPerBin"]; maxKeyPointsPer2DBin=static_cast<unsigned>(v);}
  if(tm["depthFeaturesPerBin"].type()==cv::FileNode::INT) {v = tm["depthFeaturesPerBin"]; maxKeyPointsPerDepthBin=static_cast<unsigned>(v);}
  if(tm["depthBins"].type()==cv::FileNode::INT) {v = tm["depthBins"]; binsDepth=static_cast<unsigned>(v);}
  if(tm["2dBins"].type()==cv::FileNode::INT) {v = tm["2dBins"]; bins2D=static_cast<unsigned>(v);}
  if(tm["winsize"].type()==cv::FileNode::INT) {v = tm["winsize"]; winsize=static_cast<unsigned>(v);}
  if(tm["minnumberoffeatures"].type()==cv::FileNode::INT) {v = tm["minnumberoffeatures"]; minnumberoffeatures=static_cast<unsigned>(v);}
  if(tm["tracksizex"].type()==cv::FileNode::INT) {v = tm["tracksizex"]; tracksizex=static_cast<unsigned>(v);}
  if(tm["tracksizey"].type()==cv::FileNode::INT) {v = tm["tracksizey"]; tracksizey=static_cast<unsigned>(v);}
  if(tm["featureDetectionAlgorithm"].type()==cv::FileNode::STRING) {featureDetectionAlgorithm = (std::string) tm["featureDetectionAlgorithm"];}
  if (tm["featureRansacOutlierThreshold"].type() == cv::FileNode::FLOAT) { featureRansacOutlierThreshold = tm["featureRansacOutlierThreshold"]; }
}

}