
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


#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/EvaluateCalibration.h"
#include        "mistl/MultiCamTracking.h"
#include        "mistl/RunCalibration.h"



namespace mistl {

  
EvaluateCalibration::EvaluateCalibration() {

}



EvaluateCalibration::~EvaluateCalibration() {
}


void  EvaluateCalibration::Evaluate( mistl::MultiCameraCalibration &rcal, double &evaluation, double &certainty  )
{
  
  // triavial evaluation
  unsigned n;
  evaluation = rcal.EvaluateN(n);
  certainty = 0.0;
  if(n>8) certainty = 0.2;
  if(n>40) certainty = 0.5;
  if(n>80) certainty = 1.0;

}



}
