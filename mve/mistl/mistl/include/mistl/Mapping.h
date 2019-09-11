
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


#ifndef  Mapping_incl_
#define  Mapping_incl_


#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/Histogram.h"




namespace mistl {
  

/*! \class MappingLinear Mapping.h
  \brief Base class for histogramm mapping functions
  \author O.Grau
*/


class MappingLinear {
public:           
  MappingLinear() {
    dx=dy=dz=0;    
    rx=ry=rz=1.0f;

  }
  ~MappingLinear(){}

  // use default = operator
//   void Copy( const mistl::MappingLinear& other ) {
//     dx=other.dx;
//     dy=other.dy;
//     dz=other.dz;
//   }
//   mistl::MappingLinear& operator=(const mistl::MappingLinear& other) { Copy(other); return *this;}


  
//    void  Info() const;
  
  void SetBinning( unsigned idx,unsigned idy, unsigned idz=1 ) {
    dx=idx;
    dy=idy;
    dz=idz;
  }
  
  void SetBinning( const mistl::Histogram &hist ) {
    SetBinning( hist.Nx(), hist.Ny(),hist.Nz() );
  }
  
  void SetRange( float irx,float iry, float irz=1 ) {
    rx=irx;
    ry=iry;
    rz=irz;
  }
  
  
  void Map(float x,float y, float z,
            unsigned & outx,unsigned & outy, unsigned & outz
  )  const  { 
    MISTL_ASSERT( dx && dy && dz, "Mapper: must set binning != 0");
    
    int v= floor( x/rx * (float)dx );
    if(v<0) outx=0;
    else outx=v;
    if(outx>=dx) outx=dx-1;
    v= floor( y/ry * (float)dy );
    if(v<0) outy=0;
    else outy=v;
    if(outy>=dy) outy=dy-1;
    if (rz != 0.0f){
            v = floor(z / rz * (float)dz);
    }
    else {
      v=0;
//             std::cout << "Divide by 0 in Map(). Exit" << std::endl;
//             exit(1);
    }
    if(v<0) outz=0;
    else outz=v;
    if(outz>=dz) outz=dz-1;
  }
  
public:
 
  
 
protected:
  
  float rx,ry,rz;
  unsigned dx,dy,dz;

};



}
#endif
