

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

#include        "mistl/KLTracker.h"
#include        <vector>

namespace mistl {

  unsigned ActivePoints( const std::vector<uchar> &status );
  
KLTracker::KLTracker()
{
    winsize = 11;
     maxlvl = 5;
//     refimage_ptr = 0;
}


    
 
void  
KLTracker::Track(      const cv::Mat & target,
                       const std::vector<cv::Point2f> &cornersA,
                       std::vector<cv::Point2f> &cornersB,
                      std::vector<uchar> &status,
                       std::vector<float> error
          ) {
  
  
  
  std::cout<<"KLTracker::Track( "<<  refimage_ptr.size() << " target:"<< target.size()<< " active points:"
   <<cornersA.size()<<  " winsize:" << winsize<<"\n";
   
   bool resized=false;
   cv::Mat  rtarget;
   double fx,fy;
   
  
  if( (refimage_ptr.size().height != target.size().height) || (refimage_ptr.size().width !=  target.size().width) ) {
    cv::resize( target, rtarget, refimage_ptr.size(), 0.0, 0.0, CV_INTER_CUBIC );
    resized = true;
    fx = static_cast<double>(target.size().width) / static_cast<double>(refimage_ptr.size().width);
    fy = static_cast<double>(target.size().height) / static_cast<double>(refimage_ptr.size().height);
    
   std::cout<<"KLTracker::Track RESIZE !!!!!!!\n";
  } else 
    rtarget = target;

     
  
  calcOpticalFlowPyrLK( refimage_ptr, rtarget, cornersA, cornersB, status, error,
            cv::Size(winsize, winsize), maxlvl);
  
  if(resized) { 
    std::cout<<"  resized: KLTracker::Track -  "<<   rtarget.size()<< " fx,fy:"<<fx<<","<<fy <<"\n";
    for (unsigned int i = 0; i < cornersB.size(); i++) {
        if (status[i] == 0 /*|| error[i] > 0*/) {
            continue;
        }
//        std::cout<< "scale " << cornersB[i];
        cornersB[i].x *= fx;        
        cornersB[i].y *= fy;
//        std::cout<< " -> " << cornersB[i]<<std::endl;

    }
  }
  
  std::cout<<" found points:"    <<ActivePoints(status)<<"\n";
}


    
void KLTracker::FilterOutliers( 
        const std::vector<cv::Point2f> &cornersA,
        const std::vector<cv::Point2f> &cornersB,
        std::vector<uchar> &status,
        double  maxdist /*= -1.0*/,
        cv::Mat *fundamental_matrix /*=0 */) {

        std::vector<cv::Point2f> fl1;
        std::vector<cv::Point2f> fl2;   
        int idx=0;
        

        
        for (std::vector<uchar>::iterator it = status.begin() ; it != status.end(); ++it, ++idx) 
                if( *it ) {
                        fl1.push_back( cornersA[idx ] );
                        fl2.push_back( cornersB[idx ] );
                }

        // do not continue if there are no points. opencv crashes if findFundamentalMat is used with no points.
        if(( fl1.size() == 0) || (fl2.size() == 0)){
          return;
        }

        std::vector<uchar> states;
        cv::Mat fmatrix = cv::findFundamentalMat(fl1, fl2, cv::FM_RANSAC, 3, 0.99, states);
        
        // different indices here 
        idx=0;
        int sidx=0;
        for (std::vector<uchar>::iterator it = status.begin() ; it != status.end(); ++it, ++idx) 
                if( *it ) {
                        *it = states[sidx];
//                      if(*it && maxdist>0.0) {
//                              if( mistl::DistanceF(fmatrix, cornersA[idx], cornersB[idx]) > maxdist ) *it=0;
//                      }
                        ++sidx;
                }

        if(fundamental_matrix) *fundamental_matrix = fmatrix;
}



void KLTracker::MarkMatches( 
        cv::Mat & imgC,
        const std::vector<cv::Point2f> &cornersA,
        const std::vector<cv::Point2f> &cornersB,
        const std::vector<uchar> &status,
        float scaleX,
	    float scaleY
                ) {
   
  int j = 0;
    for (unsigned int i = 0; i < cornersB.size(); i++) {
        if (status[i] == 0 /*|| error[i] > 0*/) {
            continue;
        }

        cv::Point2f cornerB = cornersB[i];
        cornerB.x = cornerB.x / scaleX;
        cornerB.y = cornerB.y / scaleY;

        cv::circle( imgC, cornerB, 2, cv::Scalar(0,0,255), 1);
        cv::line(imgC, cornersA[i], cornerB, cv::Scalar(255, 0, 0));
        ++j;
    }
}
  
}
