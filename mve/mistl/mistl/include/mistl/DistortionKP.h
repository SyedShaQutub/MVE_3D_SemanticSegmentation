
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


#ifndef __MISTL_DISTORTIONKP_H__
#define __MISTL_DISTORTIONKP_H__

#include	"math.h"
#include	"mistl/Distortion.h"
#include	"mistl/Vector2.h"
#include  "mistl/Vector3.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ForceInline.h"

using namespace std;

namespace mistl {

/*!
  \class DistortionKP Distortion.h
  \brief Distortion class with 3 radial and 2 tangential parameters
*/


class DistortionKP : public Distortion
{
  public:
    DistortionKP();

    void Copy( const mistl::DistortionKP &c ) {
	  mistl::Distortion::Copy(c);
  //     r=c.r;
  //     p=c.p;
    }  
   
    finline float GetK3() const { return dmat.at<double>(0, 4); }      //!< Returns the radial distortion, term K7 in [m-6]
    finline float GetP1() const { return dmat.at<double>(0, 2); }      //!< Returns the tangential distortion, term P1
    finline float GetP2() const { return dmat.at<double>(0, 3); }      //!< Returns the tangential distortion, term P1
  
    finline void SetK1(float v) { dmat.at<double>(0, 0) = (double)v; lookUpTableNeedsRebuilding = true; }  //!< Set radial distortion k3 in [m-2]
    finline void SetK2(float v) { dmat.at<double>(0, 1) = (double)v; lookUpTableNeedsRebuilding = true; }  //!< Set radial distortion k5 in [m-4]
    finline void SetK3(float v) { dmat.at<double>(0, 4) = (double)v; lookUpTableNeedsRebuilding = true; }  //!< Set radial distortion k7 in [m-6]
    finline void SetP1(float v) { dmat.at<double>(0, 2) = (double)v; lookUpTableNeedsRebuilding = true; }  //!< Set tangential distortion 1     
    finline void SetP2(float v) { dmat.at<double>(0, 3) = (double)v; lookUpTableNeedsRebuilding = true; }  //!< Set tangential distortion 2        
    
    void CreateInverseLookupTable();

     //! Distortion method for image coordinates without center point 
    void DistortICC(float x, float y, //!< Input coordinates in [pel]
      float &outx, float &outy        //!< distorted coordinates in [pel]
      ) const;

    void UnDistortICC(float x, float y, //!< Distorted input coordinates in [pel]
      float &outx, float &outy          //!< undistorted coordinates in [pel] 
      );
  

    void Info() const {
      const float epsilon = 1e-20f;
  
      cout << " k1: " << GetK1() << " k2: " << GetK2();
      if(fabsf( GetK3()) > epsilon) 
        cout << " k3: " << GetK3();

      if(fabsf( GetP1()) > epsilon) 
        cout << " p1: " << GetP1();

      if(fabsf( GetP2()) > epsilon) 
        cout << " p2: " << GetP2();
    }  

  protected: 
    float maxDiameterLookupTable;
    vector<vector<vector<float> > > lookUpTable;
    bool use_lut;
    int numEntriesLookupTable;
    bool lookUpTableNeedsRebuilding;
  };

}

#endif
