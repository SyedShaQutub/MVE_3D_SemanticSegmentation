
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


#ifndef __MISTL_DISTORTION_H__
#define __MISTL_DISTORTION_H__

#include	"math.h"
#include	"mistl/Vector2.h"
#include        "mistl/Vector3.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>

using namespace std;

namespace mistl {


/*!
  \class Distortion Distortion.h
  \brief Distortion class
*/

class Distortion 
{
public:
//   Distortion(float rr3 = 0.0f, float rr5 = 0.0f )  {
//     dmat=cv::Mat(1,5, CV_64F, double(0));
//     SetK1(rr3);
//     SetK2(rr5);
//   }
  Distortion( )  {
    dmat=cv::Mat(1,5, CV_64F, double(0));
  }
  void Copy( const mistl::Distortion &c ) {
//     r=c.r;
//     p=c.p;
    c.dmat.copyTo(dmat);
  }
  
  //! Access distortion coefficients
//   float K(unsigned i) const { return r[i]; }
//   float &K(unsigned i)  { return r[i]; }
  
  const cv::Mat & D() const { return dmat; }
  
  float GetK1() const { return dmat.at<double>(0,0); } //!< Returns the radial distortion, term K3 in [m-2]
  float GetK2() const { return dmat.at<double>(0,1); } //!< Returns the radial distortion, term K5 in [m-4]
  void  SetK1(float v) { dmat.at<double>(0,0)=(double)v; }           //!< Set radial distortion k3 in [m-2]
  void  SetK2(float v) { dmat.at<double>(0,1)=(double)v; }           //!< Set radial distortion k5 in [m-4]

  //! Distort 2-coordinates 
  void
  DistortICC( const mistl::Vector2f &i, //!< Input coordinates in [pel]
            mistl::Vector2f &out  //!< distorted coordinates in [pel] 
  ) const {
      DistortICC(i.X(),i.Y(), out.X(), out.Y() );
  }
  
  //! Distortion method for image coordinates without center point shift
  virtual void
  DistortICC( float x, float y, //!< Input coordinates in [pel]
            float &outx, float &outy  //!< distorted coordinates in [pel] 
  ) const {
        const float rd = (x * x + y * y);
        const float scale = 1.0f + GetK1() * rd + GetK2() * rd * rd; // + r.Z() * rd * rd *rd *rd ;
        outx = x * scale;
        outy = y * scale;
  }

  
  virtual void
  UnDistortICC( float x, float y, //!< Distorted input coordinates in [pel]
            float &outx, float &outy  //!< undistorted coordinates in [pel] 
  ) {
        const float rd = sqrtf(x * x + y * y);
        float scale;
        float r;
        
        try {
          r = SolveInvEq(rd); 
        }     catch ( mistl::Error e) {
            outx=x; outy=y; return;
        }
        const float rSquared = r*r;
        scale = 1.0f + rSquared * GetK1() + rSquared*rSquared * GetK2(); // + rSquared*rSquared *rSquared*rSquared * GetK3() ;
        outx = x / scale;
        outy = y / scale;
  }

  void Info() const {
    cout << " k1: " << GetK1() << " k2: "<< GetK2(); // << " k3: " << GetK3();
  }
  

  // Not yet implmented for three coeficients !!!!!!!!!!!!!!!!!
  float   SolveInvEq( float r, unsigned max_iterations = 50 ) const {
    float delta;
    float rn = r; 
    unsigned i = max_iterations;
    do {
     delta = (r - rn - GetK1() * rn * rn * rn - GetK2() * rn * rn * rn * rn * rn) /
             (1.0f + 3.0f * GetK1() * rn * rn + 5.0f * GetK2() * rn * rn * rn * rn);
     rn = rn + delta;
    } while ((fabsf(delta) > 1e-8) && i--);
    MISTL_ASSERT( i!=0, "Distortion: computation of iterative function did not converge");
    return rn;
  }
  
protected: 
  //////////////////////////////////
  cv::Mat  dmat;
  
//   mistl::Vector3f r;    //!< radial distortion coeficients
  // tangential distortion not yet supported
//   mistl::Vector2f p; //!< tangential distortion coeficients
        
        
};

}

#endif
