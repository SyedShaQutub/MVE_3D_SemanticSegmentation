
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


#ifndef  DRCalibrate_incl_
#define  DRCalibrate_incl_

#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        "mistl/DR_CameraArray.h"
#include        <stdlib.h>
#include <stdio.h>
#include        "mistl/RunCalibration.h"

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/ReadCoordinateFile.h"
#include        "mistl/MultiCameraCalibration.h"
#include        "mistl/MultiCamTracking.h"



#include "opencv2/core/core.hpp"



namespace mistl {
  

/*! \class DRCalibrate DRCalibrate.h
  \brief Convenience class to run calibration from one set of images
  \author O.Grau
*/
class DRCalibrate : public RunCalibration {
public:           
  DRCalibrate();
  ~DRCalibrate();

 
  
//   //! Write parameters to yaml-file stream
//   void Store( cv::FileStorage &fs );
//   //! Read parameters from yaml-file stream
//   void ReStore( cv::FileNode &tm ) ;
  
    virtual unsigned MapCameraIndex( unsigned n) const ;

public:

  //! rearranges frames appropriate to the camera list where the reference camera was set to the 0th position
  CalibrationCodeFlag  Tracking( const std::vector<cv::Mat>       & framelist, //!< list of image
                                    unsigned      seq_id=0          //!< Frame number of image set. This will be copied to 'seq_no' 

                    );

  //! Set inital camera parameters
  void SetInitialCameraParameterList( const mistl::DR_CameraArray &camArray ) ;

  //! Get copy of initial camera objects
  void GetInitialCameras( mistl::DR_CameraArray &camArray ) const;

 
private:
   mistl::DR_CameraArray camArray;
 
};




}
#endif
