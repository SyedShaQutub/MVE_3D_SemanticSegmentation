

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
//	 @author Oliver Grau
//


#include	"mistl/Error.h"
#include	"mistl/UnDistortImage.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <string>
#include <string.h>


namespace mistl {
  
void UnDistortImage( 
  const mistl::Camera & incam,
  const cv::Mat & inimage,
  const mistl::Camera & outcam,
  cv::Mat & outimage
)
{
//   MISTL_ASSERT( (inimage.depth()==1 || inimage.depth()==3), "UnDistortImage: inimage must be depth 1 or 3");
//   MISTL_ASSERT( inimage.type()== ), "UnDistortImage: inimage must be depth 1 or 3");

//   outcam.Copy(incam);
//   outcam.SetK3(0.0f);
//   outcam.SetK5(0.0f);
//   
  unsigned nx,ny,n;
  nx=inimage.cols;
  ny=inimage.rows;
  n=inimage.depth();
  outimage.create(ny,nx,inimage.type() );
  
  for(unsigned y=0; y<ny; ++y)
    for(unsigned x=0; x<nx; ++x) {
      float udx,udy;
      float ddx,ddy;
      
//       udx = static_cast<float> (x );
//       udy = static_cast<float> (y );
//       incam.Distort( udx,udy, ddx,ddy );
      
      udx = static_cast<float> (x - incam.GetPx());
      udy = static_cast<float> (y - incam.GetPy());
      udx = udx*outcam.GetSx()/outcam.GetF();
      udy = udy*outcam.GetSy()/outcam.GetF();
      incam.DistortICC( udx,udy, ddx,ddy );
      ddx = ddx*incam.GetF()/incam.GetSx() + incam.GetPx();
      ddy = ddy*incam.GetF()/incam.GetSy() + incam.GetPy();

      // hack only nn interpolation
//      std::cout<<y<<","<<x<<" -> "<<ddy <<","<<ddx<<"\n";
      // insert padding value
      if(ddx<0.0f || ddx>=nx || ddy<0.0f || ddy>=ny ) {
//       std::cout<<"padding "<<y<<","<<x<<"\n";
        if(inimage.type()==CV_8U) outimage.at<uchar>(y,x) = 0;
        else { outimage.at<cv::Vec3b>(y, x) = cv::Vec3b(); }
      
      } else {
        unsigned ox= static_cast<unsigned> (ddx);
        unsigned oy= static_cast<unsigned> (ddy);
        if(inimage.type()==CV_8U) outimage.at<uchar>(y,x) = inimage.at<uchar>(oy,ox);
        else { outimage.at<cv::Vec3b>(y, x) = inimage.at<cv::Vec3b>(oy, ox); }
         
      }
    }
  
}


	
}
