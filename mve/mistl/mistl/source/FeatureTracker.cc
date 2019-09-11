

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

#include        "mistl/FeatureTracker.h"
#include        <vector>

namespace mistl {

  unsigned ActivePoints( const std::vector<uchar> &status );
  
FeatureTracker::FeatureTracker()
{
}


    
 
void  
FeatureTracker::Track(      const cv::Mat & target,
                       const std::vector<cv::Point2f> &cornersA,
                       std::vector<cv::Point2f> &cornersB,
                      std::vector<uchar> &status,
                       std::vector<float> error
          ) {
  
  
  
  std::cout<<"FeatureTracker::Track( "<<  refimage_ptr.size() << " target:"<< target.size()<< " active points:"
   <<cornersA.size()<< " winsize:"<<winsize<<"\n";
   
  
  std::cout<<" found points:"    <<ActivePoints(status)<<"\n";
}


   
  
}
