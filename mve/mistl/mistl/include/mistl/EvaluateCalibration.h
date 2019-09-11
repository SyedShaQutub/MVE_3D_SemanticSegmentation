
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


#ifndef  EvaluateCalibration_incl_
#define  EvaluateCalibration_incl_

#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/ReadCoordinateFile.h"
#include        "mistl/MultiCameraCalibration.h"
#include        "mistl/RunCalibration.h"


#include "opencv2/core/core.hpp"

#include "dlm/scene/Scene.h"


// forward declarations
// namespace dlm {
//   class Scene ;
// }

namespace mistl {
  

/*! \class EvaluateCalibration EvaluateCalibration.h
 * \brief Evalutate calibration data
 * \author O.Grau
*/
class EvaluateCalibration {
public:           
  EvaluateCalibration();
  ~EvaluateCalibration();

 

  
  /*! \brief Calibrate camera from one set of images. 
   * 
   * Runs BuildData() and CameraCalibration(). Returns 0 if calibration was succesfull or a error code otherwise
   */
  void  Evaluate( mistl::MultiCameraCalibration &rcal, double &evaluation, double &certainty );

 
  
  //! Write parameters to yaml-file stream
//   void Store( cv::FileStorage &fs );
  //! Read parameters from yaml-file stream
//   void ReStore( cv::FileNode &tm ) ;
  
  
public:
  
  std::vector<double>  err_per_camera;
 


private:
  
  
};




}
#endif