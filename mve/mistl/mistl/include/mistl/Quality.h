
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
//    Author: Oliver Grau
//


#ifndef  Quality_incl_
#define  Quality_incl_


#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"




#include "opencv2/core/core.hpp"



namespace mistl {
  

/*! \class Quality Quality.h
  \brief Base class for quality and confidence
  \author O.Grau
*/
class Quality  {
public:           
  Quality() {}
  ~Quality(){}

public:
  //! Compute quality and confidence
//   virtual void Compute() =0;
  
  void  Info() const;
  
  float  GetQuality() const { 
   if(quality<0.0f) return 0.0f;
   if(quality>1.0f) return 1.0f;
   return quality;}
  float  GetConfidence() const { 
   if(confidence<0.0f) return 0.0f;
   if(confidence>1.0f) return 1.0f;
   return confidence;}

  float  quality;
  float  confidence;

 
private:
 
};


/*! \class TrackQuality Quality.h
  \brief  Class for quality and confidence of tracking results
  \author O.Grau
*/
class TrackQuality : public Quality {
public:           
  TrackQuality(){}
  ~TrackQuality(){}
  
  void  Info() const;
  
//   virtual void Compute();
  
 
public:

  float match;
  unsigned n;   //!< number of total correspondences
  unsigned ncam;   //!< number of cameras
  unsigned min_n;       //!< lowest number of correspondences in one camera

private:
 
};



}
#endif
