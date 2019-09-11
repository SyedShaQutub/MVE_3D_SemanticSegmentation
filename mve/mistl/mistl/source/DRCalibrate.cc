
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
//    Author: 
//



#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>


#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/DRCalibrate.h"
#include        "mistl/MultiCamTracking.h"



namespace mistl {

  
DRCalibrate::DRCalibrate() {
  SetDefaults();
}

unsigned DRCalibrate::MapCameraIndex( unsigned n) const 
{
  if(n<camArray.refCam) 
    return n+1;

  return 0; // SHOULD 0 BE DEFAULT CASE IF THE IF-CONDITION ABOVE DOESNT APPLY?
}

DRCalibrate::~DRCalibrate() {
}

// Call RunCalibration::SetInitialCameraParameterList() with cameras from in_camArray.
// !!! The cameras will be reordered, as some Mistl classes (Feature exctraction) assume cam 0 is the reference camera
// This is currently a work-around !!!
void DRCalibrate::SetInitialCameraParameterList( const mistl::DR_CameraArray & in_camArray ) {

  camArray=in_camArray;
  mtrack.refcam=in_camArray.refCam;
  
  std::vector<mistl::Camera>    tmpCamlist;
#ifdef SWAPCAMS
  tmpCamlist.push_back( camArray.camera[camArray.refCam] );
  for(unsigned i=0; i<camArray.number_of_cameras; ++i) {
      if(i==camArray.refCam) continue;
      tmpCamlist.push_back( camArray.camera[i] );
  }
#else
  for(unsigned i=0; i<camArray.number_of_cameras; ++i) tmpCamlist.push_back( camArray.camera[i] );
#endif
  RunCalibration::SetInitialCameraParameterList(tmpCamlist);
}


void DRCalibrate::GetInitialCameras( mistl::DR_CameraArray & out_camArray ) const {
        out_camArray = camArray;
	std::vector<mistl::Camera>    tmpCamlist;
	RunCalibration::GetInitialCameras(tmpCamlist);

        MISTL_ASSERT( tmpCamlist.size()==camArray.number_of_cameras, "DRCalibrate::GetInitialCameras: unexpected number of cameras");

        
#ifdef SWAPCAMS
        
	out_camArray.camera[camArray.refCam].Copy(tmpCamlist[0]);
        
        std::cout<<"###\ncam-ref:"<< camArray.refCam<<"\n";
        tmpCamlist[0].Info();
        
        unsigned j=0;
	for(unsigned i=1;  i<camArray.number_of_cameras; ++i,++j) {
		if(j!=camArray.refCam) {
                  out_camArray.camera[j] .Copy( tmpCamlist[i]);
                  
             std::cout << " "<<i << "->" << j <<"\n";
             tmpCamlist[i].Info();
                }
	}
#else
        for(unsigned i=0;  i<camArray.number_of_cameras; ++i) {
            out_camArray.camera[i] .Copy( tmpCamlist[i]);
        }
#endif 
}


CalibrationCodeFlag DRCalibrate::Tracking( const std::vector<cv::Mat> & framelist, unsigned seq ) {

#ifdef DEBUG
  if(DebugMode()) {
    std::cout << "%%%%\n%%%%\n - DRCalibrate::Tracking: ref-cam: "<< camArray.refCam << " (no of cams:"<<camArray.number_of_cameras<<") ";
    for(unsigned i=0; i<framelist.size(); ++i) {
      std::cout << " cam-"<<i<<" "<< framelist[i].size();
    }
    std::cout << "\n";
  }
#endif

    
	std::vector<cv::Mat> tmpFrameList;
#ifdef SWAPCAMS

	tmpFrameList.push_back( framelist[camArray.refCam] );
	  for(unsigned i=1; i<camArray.number_of_cameras; ++i) {
		if(i==camArray.refCam) {
			tmpFrameList.push_back( framelist[0] );
			continue;
		}

		tmpFrameList.push_back( framelist[i] );
	}

        return RunCalibration::Tracking(tmpFrameList, seq);
#else
        return RunCalibration::Tracking(framelist, seq);
#endif
}





}
