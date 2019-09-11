
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



#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>
#include <fstream>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/RunCalibration.h"
#include        "mistl/ProjectionMapping.h"

#ifdef ANDROID
#include "mistl/to_string.h"
#else
#define		to_string	std::to_string
#endif


namespace mistl {

void RunCalibration::Judge3DHistogram(  float &near, float &mid, float &far )  const 
{
//   near = static_cast<float>(histogram3d.Get(0,0,0 ));
//   near /= 0.1; 
//   if(near>1.0f) near=1.0f;
//                             
//   mid = static_cast<float>(histogram3d.Get(0,0,1 ));
//   mid /= 0.1; 
//   if(mid>1.0f) mid=1.0f;
//                            
//   far = static_cast<float>(histogram3d.Get(0,0,2 ));
//   far /= 0.1; 
//   if(far>1.0f) far=1.0f;

	//printf("lambda=%f\n", lambda);
  
  near = histogram3d.Fk( histogram3d.Get(0,0,0 ), 3, lambda );
  mid = histogram3d.Fk( histogram3d.Get(0,0,1 ), 3, lambda );
  far = histogram3d.Fk( histogram3d.Get(0,0,2 ), 3, lambda );
}            


float RunCalibration::Judge2DHistogram(   )  const 
{
  float h=1.f;
  for(unsigned j=0; j<histogram2d.Ny(); ++j) {
    unsigned sum=0;
    for(unsigned i=0; i<histogram2d.Nx(); ++i) {
      sum += histogram2d.Get(i,j,0);
    }
    h += histogram2d.Fk( sum, 6, lambda );
  }
  for(unsigned j=0; j<histogram2d.Nx(); ++j) {
    unsigned sum=0;
    for(unsigned i=0; i<histogram2d.Ny(); ++i) {
      sum += histogram2d.Get( j,i,0);
    }
    h += histogram2d.Fk( sum, 6, lambda );
  }
  return h / 6.0f;
}

void RunCalibration::Compute3DHistogram(   )  
{
  // write points
  std::vector<mistl::Vector3f> plist;
  std::map<unsigned,unsigned> index_translate;

  cal.GetPointList(plist, index_translate);
  
  histogram3d.SetSize( 1,1,mtrack.binsDepth );
  histogram2d.SetSize( mtrack.bins2D,mtrack.bins2D,1 );

  mistl::ProjectionMapping     mapper;
  mapper.CopyCamera( camlist.at(mtrack.refcam) );
  mapper.SetRange( mapper.cam.GetNx(), mapper.cam.GetNy(), 10. );
  mapper.SetBinning( histogram3d );
  
  mistl::MappingLinear     linmapper;
  linmapper.SetRange( mapper.cam.GetNx(), mapper.cam.GetNy(), 0. );
  linmapper.SetBinning( histogram2d );
  
  for(unsigned i=0; i<plist.size(); ++i) {
    
    float px,py,pz;
    mapper.cam.Project(plist[i],px,py,pz );
//     std::cout << "Map "<<plist[i] << " -> " << px<<","<<py << ","<<pz << "\tbin: [";
    
    unsigned ix,iy,iz;
    mapper.MapProj( plist[i], ix,iy,iz );
//     std::cout << ix<<","<<iy<<","<<iz<<"]\n";
    histogram3d.Add(ix,iy,iz );
    
    linmapper.Map( px,py, 0.f , ix,iy,iz );
    histogram2d.Add(ix,iy,iz );
  }
 
  if (DebugMode()) {
	  histogram3d.Info();
	  histogram2d.Info();
  }
  
}


}
