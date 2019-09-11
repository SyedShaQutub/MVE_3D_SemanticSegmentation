
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


#ifndef  Histogram_incl_
#define  Histogram_incl_


#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"




#include "opencv2/core/core.hpp"



namespace mistl {
  

/*! \class Histogram Histogram.h
  \brief Base class for histogramms
  \author O.Grau
*/


class Histogram  {
public:           
  Histogram() {n=0;}
  ~Histogram(){}

  void Copy( const mistl::Histogram& other ) {
    dx=other.dx;
    dy=other.dy;
    dz=other.dz;
    data=other.data;
    n=other.n;
  }
  mistl::Histogram& operator=(const mistl::Histogram& other) { Copy(other); return *this;}

  void Clear() {
    for(unsigned i=0; i<dx*dy*dz; ++i) data.at( i )=0; 
    n=0;
  }
  
  void  Info() const {
     std::cout<< "Histogram ("<<Nx()<<","<<Ny()<<","<<Nz()<<") :\n";
     for(unsigned z=0; z<Nz();++z) {
       for(unsigned y=0; y<Ny();++y) {
         std::cout<<" (y:"<<y<<"): ";
         for(unsigned x=0; x<Nx();++x)
           std::cout<< Get(x,y,z) <<"\t";
       }
       std::cout<<"\n";
     }
  }
  
  unsigned  N() const { return n;}
  unsigned Nx() const { return dx; }
  unsigned Ny() const { return dy; }
  unsigned Nz() const { return dz; }

  float MeanOccupancy( ) const {
    float r=0.f;
    for(unsigned x=0; x<data.size();++x)
      r += (float)data[x];
    if(data.size()) r /= (float)data.size();
    return r;
  }
  unsigned MedianOccupancy( ) const {
    if(!data.size()) return 0;
    std::vector<unsigned>    d = data;
    std::sort (d.begin(), d.end() );
    unsigned i= (unsigned)d.size()/2;
    return d.at(i);
  }
  
  unsigned MinOccupancy( ) const {
    if(!data.size()) return 0;
    unsigned v= data[0];
    for(unsigned x=0; x<data.size();++x)
      if(v>data[x]) v=data[x];
    return v;
  }
  
  unsigned MaxOccupancy( ) const {
    unsigned v= 0;
    for(unsigned x=0; x<data.size();++x)
      if(v<data[x]) v=data[x];
    return v;
  }

  //! Fk(n) function
  float Fk( unsigned n_, unsigned k, float lambda ) const {
    float r = (float)n_/(float)N() - lambda/(float)k ;
    if(r<0.f) return 0.f;
    return 1.f;
  }
  
  
  
  void SetSize( unsigned idx,unsigned idy, unsigned idz=1 ) {
    dx=idx;
    dy=idy;
    dz=idz;
    data.resize( dx*dy*dz, 0 );
  }
  
  void Add( unsigned x,unsigned y, unsigned z=1 )    { 
      ++(data.at( x + y*dx + z*dx*dy )); 
      ++n;
  }
  
  
  unsigned Get(unsigned x,unsigned y, unsigned z=1 )  const  { 
     return data.at( x + y*dx + z*dx*dy ); 
  }
public:
  //! Compute quality and confidence
//   virtual void Compute() =0;
  
  
 
protected:
  
  unsigned n;
  unsigned dx,dy,dz;
  std::vector<unsigned>    data;
};



}
#endif
