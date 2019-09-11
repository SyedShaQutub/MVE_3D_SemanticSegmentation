

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
#include        "mistl/MultiCamTracking.h"
#include        "mistl/Line3.h"
#include        <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <cmath>

namespace mistl {
  
MultiCamTracking :: MultiCamTracking()
{
//   verbose=true;
  filteroutliers=true;
  m_maxCorners = 1000;
  m_qualityLevel= 0.01; //!< quality level see: cv::goodFeaturesToTrack
  m_minDistance=10;
  featureLevels = 0;
  winsize= 15;
  minnumberoffeatures=40;
  refcam=0;
  tracksizex=tracksizey=0;
  useFeatureValidation = false;

}

MultiCamTracking :: ~MultiCamTracking()
{
}



//
//
void MultiCamTracking :: Info(bool fulllist) const
{
  std::cout<<"MultiCamTracking  refcam:"<< refcam << " no of cameras:"<<camtracklist.size();
  std::cout<<"\t3-points:"<< P3list.size()<<"\n";
  if(fulllist) {
    for(unsigned j=0; j<camtracklist.size(); ++j) {
      std::cout<<"cam"; if((j+1)<10)std::cout<<'0';std::cout<<j+1<<"\tpoints: ";
      for(unsigned i=0; i<camtracklist[j].tracklist.size(); ++i) {
        const mistl::Track &track = Track(j,i);
        if(i) std::cout<<',';
        std::cout<<track.ActivePoints()<<'('<<track.plist.size()<<')';
      }
      std::cout<<"\n";
    }
  }
      
}


//
//
void MultiCamTracking :: ComputeQuality(mistl::TrackQuality  & q) const
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
    q.confidence = 0.0f;\
    return;
  }
  unsigned t = m_maxCorners*q.ncam; // max expected correspondences
  
  float q1 = static_cast<float>(q.n) / static_cast<float>(t);
  // need more tweaking
  q.quality = 1.0f;
  q.confidence = q1*2.0;
  if(q.confidence >1.0f) q.confidence=1.0f;
}

 

void MultiCamTracking :: FindFeaturesToTrack( const cv::Mat &refimg, 
                                      mistl::TrackingPointList &tplist,
                    int maxCorners, //!< max number of corner features to extract in first image
                      double qualityLevel, //!< quality level see: cv::goodFeaturesToTrack
                      double minDistance  //!< min. distance between features see: cv::goodFeaturesToTrack
  )
{
  // hack!!
   featureLevels = 4;
  
  std::cout<<"FindFeaturesToTrack : " << featureLevels;
  cv::Mat nxtimg;
  
  std::vector<cv::Point2f> plist;
  
  cv::goodFeaturesToTrack( refimg, plist, maxCorners, qualityLevel, minDistance);
  tplist.Clear();
  
  // Store size of reference image to point list to allow re-scaling
  int imgWidth = refimg.size().width;
  int imgHeight = refimg.size().height;
  if(tplist.imgDimX <= 0 || tplist.imgDimY <= 0) {
    tplist.setImgSize(imgWidth, imgHeight);
  }
  
  tplist.Copy( plist );
  std::cout<<"FindFeaturesToTrack i:"<< refimg.size() << " q:"<< qualityLevel <<" max:"<<maxCorners<< " found:"<< plist.size();
  std::cout << "  tp:"<< tplist.plist.size() <<std::endl;
  
  if(featureLevels) {
    nxtimg=refimg;
    unsigned h_maxCorners= maxCorners/4;
    unsigned min_dist = minDistance;
    for(int i=1; i<=featureLevels; ++i) {
      cv::pyrDown(nxtimg,nxtimg);
      
      std::vector<cv::Point2f> plist;
      cv::goodFeaturesToTrack( nxtimg, plist, h_maxCorners, qualityLevel,min_dist );

      // clear nearby features
//#ifndef _WIN32
      int searchsize = min_dist/2;
      int deleted = 0;
      for(unsigned j=0; j<plist.size(); j++) {

    	  double scale = (1 << i);

    	  int x = int(plist[j].x * scale);
    	  int y = int(plist[j].y * scale);
    	  MISTL_ASSERT( x>=0, " x coordinate may not be negative.");
    	  MISTL_ASSERT( y>=0, " y coordinate may not be negative.");

    	  int tRow = y-searchsize >= 0 ? y-searchsize : 0;						// define top row of the search window
    	  int bRow = y+searchsize < imgHeight ? y+searchsize : imgHeight-1;		// define bottom row of the search window
    	  int lSide = x-searchsize >= 0 ? x-searchsize : 0;						// define left side of the search window
    	  int rSide = x+searchsize < imgWidth ? x+searchsize : imgWidth-1;		// define right side of the search window

    	  for(int n=tRow; n<=bRow; n++) {
    		 for(int m=lSide; m<=rSide; m++) {
    			 int feature = tplist.imgFeatures[m][n];
    			 if(feature >= 0) {

    				 tplist.status[feature] = 0;
    				 tplist.imgFeatures[m][n] = -1;
    				 deleted++;
    			 }
    		 }
    	  }
      }
//#endif

      tplist.Add( plist, i );
      h_maxCorners= h_maxCorners/4;
      if(!h_maxCorners) break;
      min_dist /= 2;
      std::cout<<"   h:"<< i << " feat:"<<  plist.size() << std::endl;
    }

    // delete all features with status 0 (features in lower res images were found next by)
    std::vector<TrackingPointMeta>  tmpMetalist;
    std::vector<cv::Point2f> tmpPlist;
    std::vector<uchar> tmpStatus;
    for(int i=0; i<(int)tplist.status.size(); i++) {
      MISTL_ASSERT( tplist.plist[i].x>=0, " x coordinate may not be negative.");
      MISTL_ASSERT( tplist.plist[i].y>=0, " y coordinate may not be negative.");

      if(tplist.status[i] != 0) {
    	  tmpPlist.push_back(tplist.plist[i]);
    	  tmpStatus.push_back(tplist.status[i]);
    	  tmpMetalist.push_back(tplist.metalist[i]);
      }
    }
    tplist.Clear();
    tplist.plist = tmpPlist;
    tplist.status = tmpStatus;
    tplist.metalist = tmpMetalist;
   
  }
}

void 
MultiCamTracking :: Clear()
{
  camtracklist.clear();

}


void 
MultiCamTracking :: InfraTracking( const std::vector<cv::Mat >   &imglist/*,  mistl::TrackQuality  & q*/ )
{
	if (DebugMode()) std::cout << "InitTracks ilist:" << imglist.size() /*<< " clist:"<<icamlist.size()*/ << "\n";
  
  Clear();
  
  MISTL_ASSERT( imglist.size()>1 /*&& (imglist.size()==icamlist.size())*/, "MultiCamTracking :: InitTracks error image/camera list don't match or empty");

  // initializing feature validation
  if(useFeatureValidation) {
	  featureValidation.initCameras(imglist);
  }

  //
  std::vector<cv::Mat >   scaledimglist;
  std::vector<mistl::Vector2f> scalelist;
  
  
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
    } else {
      scaledimglist.push_back(imglist.at(i));
    }

    camtracklist.push_back( MultiCamTrackingBase() );
  }
  
  
  mistl::MultiCamTrackingBase  &base = camtracklist[0];
  
  //
  // find features to track
  //
  FindFeaturesToTrack( scaledimglist[0], trackdata, m_maxCorners, m_qualityLevel, m_minDistance);
  
  if(DebugMode()) std::cout<< "Found " <<trackdata.plist.size() << " features to track"<<std::endl;
  
  // save features for first cam
  base.tracklist.push_back( mistl::Track( trackdata.plist ));
  
  tracker.SetRefImage( scaledimglist[0] );
  
  // set winsize of tracker
  tracker.winsize = winsize;                       
  
  for (unsigned int i = 1; i < scaledimglist.size(); i++) {
    mistl::MultiCamTrackingBase  &base = camtracklist[i];
    base.tracklist.push_back( mistl::Track() );
    mistl::Track &track = base.tracklist.back();
    
    if (scaledimglist[i].data){  // avoid invalid, empty images
      tracker.Track( scaledimglist[i], trackdata.plist, track.plist, track.status, track.error );
    
      cv::Mat fundamental_matrix;
      if(filteroutliers) 
        tracker.FilterOutliers( trackdata.plist, track.plist, track.status , 2.0, &fundamental_matrix);
    }
  }
  
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
  
}


void 
MultiCamTracking :: PrunePoints (  )
{
  // 
  // only considers first track for now!!!!!
  // It is assumed this method is called at the first frame (seq==0)
  //
  MISTL_ASSERT( camtracklist.size()>0, "MultiCamTracking::GeneratePoints:needs at least one image/camera");
  
  
    
    
  
  std::vector<uchar> allstatus;
  const unsigned len = (unsigned)Track(0,0).plist.size();
  for(unsigned i=0; i<len; ++i) {
    allstatus.push_back(0);
  }
  
  if(true) {
    
      for(unsigned i=0; i<len; ++i) {
        bool v=true;

        for(unsigned j=1; j<camtracklist.size(); ++j) 
          if(Track(j,0).status.at(i)==0) v=false;
      
        if(!v) {
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


    // add found features to Feature validation after filtering outliers.
    if(useFeatureValidation && featureValidation.inititalized) {
      std::vector<mistl::Track> tmpValidationTracks;
	  for (unsigned int i = 0; i < camtracklist.size(); i++) {
		mistl::Track &tmpTrack = Track(0,0);
		mistl::Track &track = Track(i,0);
		MISTL_ASSERT( track.plist.size() == tmpTrack.plist.size() , "cameras do not have an equal number of features");
		tmpValidationTracks.push_back( track );
	  }
	  featureValidation.addFeatures(tmpValidationTracks);
	  featureValidation.printHistogram();
    }
  
}

void 
MultiCamTracking :: GeneratePoints (  bool triangulate, float pointlen  )
{
    
  bool hascameras=true;
  if(camlist.size()!=camtracklist.size()) {
    hascameras=false;
    for(unsigned j=1; j<camtracklist.size(); ++j)
      camlist.push_back( mistl::Camera());
  }
  
  P3list.clear();
  // generate points
  mistl::Track &track1 = Track(0,0);
  
  if ( DebugMode() ) {
    std::cout<<"GeneratePoints("<<triangulate<<","<<pointlen<<")\n";
    camlist[0].Info();
  }
  // initialise points 
  for(unsigned i=0; i<track1.plist.size(); ++i) {
    mistl::Vector3f P = camlist[0].C() + camlist[0].LineOfSight( track1.plist[i].x, track1.plist[i].y ) * pointlen;
    P3list.push_back( P );
  }

      
  if(camtracklist.size()==1 || !triangulate || !hascameras ) return; 
  
  // overwrite points 
  

  if (DebugMode()) {
	  std::cout << "INFO \n\n\n\n" << std::endl;
	  std::cout << "camtracklist.size() " << camtracklist.size() << std::endl;
	  for (unsigned j = 0; j < camtracklist.size(); ++j) {
		  std::cout << "camtracklist[" << j << "].tracklist.size() = " << camtracklist[j].tracklist.size() << std::endl;
		  for (unsigned i = 0; i < camtracklist[j].tracklist.size(); i++) {
			  std::cout << "camtracklist[" << j << "].tracklist[" << i << "].plist.size() = " << camtracklist[j].tracklist[i].plist.size() << std::endl;
		  }
	  }
	  std::cout << "INFO END \n\n\n\n" << std::endl;
  }

  for(unsigned j=1; j<camtracklist.size(); ++j) {
	mistl::Track &track = Track(j,0);

	if (DebugMode()) std::cout << "\n\nTRACK " << j << "\n\n" << std::endl;

    for(unsigned i=0; i<track.plist.size(); ++i) {
      if(track.status.at(i)==0) continue;

      mistl::Line3      lin1( camlist[0].C(), camlist[0].C() + camlist[0].LineOfSight( track1.plist[i].x, track1.plist[i].y ));
      mistl::Line3      lin2( camlist[j].C(), camlist[j].C() + camlist[j].LineOfSight( track.plist[i].x, track.plist[i].y ));
      double tau;
      mistl::Vector3f P = lin1.Intersection( lin2, tau);
      float depth=camlist[0].A().Dot(P);
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
  }  
}


void  MultiCamTracking :: GenerateCoordinateList( unsigned j, std::vector<mistl::CoordinateListEntry> &clist ) const
{
    clist.clear();
#ifdef BDEBUG
    std::cout<<"GenerateCoordinateList "<<camtracklist[j].tracklist.size()<<"\n";
#endif   
    for(unsigned i=0; i<camtracklist[j].tracklist.size(); ++i) {
      const mistl::Track &track = Track(j,i);
      MISTL_ASSERT(track.plist.size()==P3list.size(), "MultiCamTracking :: GenerateCoordinateList: 3point list doesn't match track list");
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
MultiCamTracking :: WriteCoordinateFile( const char *patt, bool multiple_file_per_camera, unsigned id_offset  ) const 
{
  int strlenPatt = (int)strlen(patt);
  char* fn = new char[strlenPatt + 200];
    
  for(unsigned j=0; j<camtracklist.size(); ++j) {
    sprintf(fn,patt,j,0);
    std::ofstream fo( fn);

    MISTL_ASSERT(!fo.fail(), "MultiCamTracking :: WriteCoordinateFile: error opening file for writing");
    
    std::cout<<"Write coord-file:"<<fn<<"\n";
    fo << "#Coordinate list file\n#frame-id       Px Py Pz        ix iy   label\n";
    for(unsigned i=0; i<camtracklist[j].tracklist.size(); ++i) {
      const mistl::Track &track = Track(j,i);
//       MISTL_ASSERT(track.plist.size()==P3list.size(), "MultiCamTracking :: WriteCoordinateFile: 3point list doesn't match track list");
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
unsigned ActivePoints( const std::vector<uchar> &status )  {
      unsigned r=0;
      for(unsigned i=0; i<status.size(); ++i) if(status[i]!=0) ++r;
      return r;
}



void MultiCamTracking::Store( cv::FileStorage &fs )
{      
  
  MISTL_ASSERT( fs.isOpened(), "RunCalibration::Store: error opening parameter file for writing");


  fs << "maxCorners" << m_maxCorners;
  fs << "qualityLevel" << m_qualityLevel;
//   fs << "verbose" << static_cast<int>(verbose);
  fs << "filteroutliers" << static_cast<int>(filteroutliers);
  fs << "minDistance" << m_minDistance;
  fs << "winsize" << static_cast<int>(winsize);
  fs << "featureLevels" << featureLevels;
//   fs << "" << ;
}

//! Read parameters from file
void MultiCamTracking::ReStore( cv::FileNode &tm )
{
//   SetDefaults();
  if(tm.empty()) return;
  int v;
  v = tm["filteroutliers"]; filteroutliers= v!=0;
//   v = tm["verbose"];  verbose= v!=0;
  m_qualityLevel = tm["qualityLevel"]  ;
  m_maxCorners = tm["maxCorners"]  ;
  m_minDistance = tm["minDistance"]  ;
  featureLevels = tm["featureLevels"];
  if(tm["winsize"].type()==cv::FileNode::INT) {v = tm["winsize"]; winsize=static_cast<unsigned>(v);}
  if(tm["minnumberoffeatures"].type()==cv::FileNode::INT) {v = tm["minnumberoffeatures"]; minnumberoffeatures=static_cast<unsigned>(v);}
  if(tm["tracksizex"].type()==cv::FileNode::INT) {v = tm["tracksizex"]; tracksizex=static_cast<unsigned>(v);}
  if(tm["tracksizey"].type()==cv::FileNode::INT) {v = tm["tracksizey"]; tracksizey=static_cast<unsigned>(v);}

}

  
}
