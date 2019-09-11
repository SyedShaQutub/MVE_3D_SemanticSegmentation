

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


#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
#include	<stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"
#include	"mistl/OCV_Camera.h"
#include <string.h>
#include	"mistl/Error.h"





int main(int argc, char** argv)
{
  printf("Hello world! My name is Mistl!\n");
   
  mistl::Vector3f	a,b,c;
 
  
  mistl::OCV_Camera mcam;
  
  ReadCameraYAML( mcam,"scam00.yml");
  mcam.Info();
  std::cout<<"\n";
   

    mistl::Camera	cam;
//   ReadCamera(cam,"cam_03.cahv");
  mcam.AdjustF(1e-5);	// guess work here: what is the actual sensor pixel size?
  mcam.nx=1288;
  mcam.ny=968;
//   std::cout<<"convert to mistl-cam..\n";
  mcam.CopyTo(cam);
  cam.Info();
  std::cout<<"\n";
  // set position of camera 
  cam.SetC( mistl::Vector3f( 0.4f, 0.5f, 1.3f));
  
  mistl::WriteCamera(cam,"t.ycam");
  
  mistl::Vector2f	px;
  float x,y;
  c=mistl::Vector3f(1,2,3);
  cam.Project(c,x,y);
  std::cout<<c<<" -> "<<x<<" "<<y<<"\n";
  
  cv::Point2f p;
  cv::Point3f P(c.X(),c.Y(),c.Z());
  mcam.Project( P, p);
  std::cout<<"OpenCV: "<< P<<" -> "<<p<<"\n";
  
  // Test copy cam to OpenCV camera
  mistl::OCV_Camera ocvcam;
  ocvcam.Set(cam);
  WriteCameraYAML(ocvcam,"o.yml");
  std::cout<<"OCV_Camera pose:"<< ocvcam.rvec << ocvcam.tvec << "\n";
  
  mistl::Camera ncam;
  ocvcam.CopyTo(ncam);
  std::cout<<"The MISTL camera position:"<< ncam.C()<<"\n";
  
  

}