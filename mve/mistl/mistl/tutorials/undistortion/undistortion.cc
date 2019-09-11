

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
#include        "mistl/UnDistortImage.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include	<stdlib.h>
#include <stdio.h>



void
PrMsg() {
    std::cout<<"usage: undistortion  distorted-camera-file image-file out-image [in-undistorted-camera]\n";
    exit(-1);
}


int main(int argc, char** argv)
{
  printf("Hello world! My name is Mistl!\n");
   
 
  if(argc!=5 && argc!=4) PrMsg();
  
  mistl::Camera	cam;
  std::cout<<"m1\n";
  try {
      ReadCamera(cam,argv[1]);
  } 
  catch ( mistl::Error e) {
      std::cout<<"Error: "<<e.GetMsg()<<"\n";
      return -1;
  }
  
  std::cout<<"m2\n";
  cam.Info();std::cout<<"\n";
  
  cv::Mat img = cvLoadImage(argv[2], CV_LOAD_IMAGE_UNCHANGED);
  
  std::cout<<"img:"<<img.size()<<" depth:"<<img.depth()<< " type:"<<img.type()<<"\n";
  
  cv::Mat uimg;
  mistl::Camera ucam;
  
  if(argc==5) {
    try {
        ReadCamera(ucam,argv[4]);
    } 
    catch ( mistl::Error e) {
        std::cout<<"Error: "<<e.GetMsg()<<"\n";
        return -1;
    }
  } else {
    ucam.Copy(cam);
    ucam.SetK3(0.0);
    ucam.SetK5(0.0);
    // hack
    ucam.SetF( cam.GetF()*0.8 );
  }
  
  UnDistortImage( cam, img, ucam, uimg );
  
  cv::imwrite(argv[3] , uimg );
} 