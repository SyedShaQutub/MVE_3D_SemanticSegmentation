

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


#ifndef __MISTL_Track_H__
#define __MISTL_Track_H__


#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {
  
unsigned ActivePoints( const std::vector<uchar> &status );

/*!
  \class Track Track.h
  \brief Data container for camera tracking
  \author Oliver Grau, Intel 2014
*/
class   Track 
{
public:
  Track() {}
  Track( const std::vector<cv::Point2f>      &iplist ) { 
    plist=iplist;
    FillStatus();
  }
  Track( const Track &t) { (*this)=t; }
  mistl::Track& operator= (const Track &t) { 
    plist=t.plist;
    pidlist=t.pidlist;
    seqid=t.seqid;
    status=t.status;
    error=t.error;
    return *this;
  }
  void Clear() {
    plist.clear();
    pidlist.clear();
    status.clear();
    error.clear();
  }
  void FillStatus( ) {
    for(unsigned j=0; j<plist.size(); ++j) {
      status.push_back( 1 );
      error.push_back( 0.0 );
    }
  }
  
  unsigned ActivePoints(   ) const { return mistl::ActivePoints(status); }
  ///////////////////////////
public:
  //! coordinate entries
  std::vector<cv::Point2f>      plist;
  //! point identifiers. Needs to have same length as plist
  std::vector<unsigned>         pidlist;
  std::vector<uchar> status;
    std::vector<float> error;
    unsigned      seqid;
};



}

#endif


