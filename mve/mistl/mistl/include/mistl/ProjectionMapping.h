
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


#ifndef  ProjectionMapping_incl_
#define  ProjectionMapping_incl_


#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/Camera.h"
#include        "mistl/Mapping.h"




namespace mistl {
  

/*! \class ProjectionMapping ProjectionMapping.h
  \brief Mapping of 3D points onto a restricted histogramm volume
  \author O.Grau
*/


class ProjectionMapping : public mistl::MappingLinear {
public:           
  ProjectionMapping() {
  }
  ~ProjectionMapping(){}

  // use default = operator
//   void Copy( const mistl::ProjectionMapping& other ) {
//     dx=other.dx;
//     dy=other.dy;
//     dz=other.dz;
//   }
//   mistl::ProjectionMapping& operator=(const mistl::ProjectionMapping& other) { Copy(other); return *this;}


  
//    void  Info() const;
  
  void  CopyCamera( const mistl::Camera &icam ) {
    cam.Copy(icam);
  }
  
  
  
  
  
  void MapProj( mistl::Vector3f  v,
            unsigned & outx,unsigned & outy, unsigned & outz
  )  const  {
    float x,y,z;
    cam.Project( v, x,y,z );
//     if(fabsf(z)>1e-10) z = 1.0f / z;
//     else z=1e10;
    Map( x,y,z, outx, outy, outz );
  }
  
public:
 
  
 
// protected:
  mistl::Camera cam;

};



}
#endif
