

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
//       @author Oliver Grau
//


#ifndef __MISTL_TextureMap_H__
#define __MISTL_TextureMap_H__


#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

namespace mistl {
  

/*!
  \class TextureMap TextureMap.h
  \brief texture map class
  \author Oliver Grau, Intel 2014
*/


// currently a place holder needs to get an image attached
class   TextureMap 
{
public:
  
  ///////////////////////////
public:
  
  std::string   filename;
};



}

#endif


