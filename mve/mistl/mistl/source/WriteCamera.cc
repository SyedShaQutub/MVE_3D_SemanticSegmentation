

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
#include	"mistl/Camera.h"
#include	"mistl/OCV_Camera.h"
#include        "mistl/YAML_IO.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>


namespace mistl {
  

const unsigned version=3;

void WriteCameraCAHV( const mistl::Camera &cam, const char *fn  )
{
	 std::ofstream fo(fn);

  MISTL_ASSERT(!fo.fail(), "error opening file for writing");


  if(version==3) 
    fo << ";CAHV 1.3\n";
  else
    fo << ";CAHV 1.2\n";
  fo << "; Camera-type: Camera\n";

 
  mistl::Vector3f c,a,h,v;


  c = cam.C()*1000.0;
  a = cam.A();
  h = cam.H0()*(cam.GetF()/cam.GetSx()) + cam.A()*(cam.GetCenterPointShiftX()/ cam.GetSx());
  v = cam.V0()*(cam.GetF()/cam.GetSy()) + cam.A()*(cam.GetCenterPointShiftY()/ cam.GetSy());
  fo << c.X() <<" "<<c.Y()<<" "<<c.Z() <<'\n';  
  fo << a.X() <<" "<<a.Y()<<" "<<a.Z()<< '\n';
  fo << h.X()<<" "<<h.Y()<<" "<<h.Z()<< '\n';    
  fo << v.X()<<" "<<v.Y()<<" "<<v.Z()<< '\n'; 
  fo << cam.GetK1() << " "<<cam.GetK2();
  if(version==3) fo << " "<<cam.GetK3(); 
  fo << '\n'; 
  fo << cam.GetSx()*1000.0 << " "<< cam.GetSy()*1000.0 << '\n';
  fo << cam.GetNx() << " "<< cam.GetNy()<< '\n';


}




void WriteCamera( const mistl::Camera &cam, const char *fn  )
{	
  const char *cp = strrchr(fn,'.');
  if(cp == nullptr) {
	  std::cout << "output file must be of type: *.cahv, *.ycam or *.yml\n" << std::endl;
	  exit(1);
  }

  if(!strcmp(cp, ".cahv")) WriteCameraCAHV(cam,fn);
  else if(!strcmp(cp, ".ycam")) WriteCameraYAML(cam,fn);
  else if(!strcmp(cp, ".yml")) {
	  mistl::OCV_Camera oCam;
	  oCam.Set(cam);
//	  oCam.AdjustF(1e-5);   // guess work here: what is the actual sensor pixel size?
	  WriteCameraYAML(oCam, fn);
  }
  else MISTL_ASSERT(false,"WriteCamera: unknown file extension - no write module to delegate");
}
	
}
