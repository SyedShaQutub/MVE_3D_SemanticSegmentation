

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
#include	"mistl/Transform3.h"
#include	<stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"

#ifndef __MISTL_YAML_H__
#define __MISTL_YAML_H__


namespace mistl {
 /**
 * @ingroup MISTLCFunction
 * @brief Write vector to YAML stream
 * @author O.Grau
**/ 
void    WriteVector( cv::FileStorage &fs, const mistl::Vector3f &v );

 /**
 * @ingroup MISTLCFunction
 * @brief Read vector from YAML stream
 * @author O.Grau
**/ 
mistl::Vector3f ReadVector( cv::FileNode &fs, const char *nam );


/**
 * @ingroup MISTLCFunction
 * @brief Write camera parameters to YAML file
 * @param cam camera object
 * @param fn file name
 * @author O.Grau
**/
void WriteCameraYAML( const mistl::Camera &cam, const char *fn  );

/**
 * @ingroup MISTLCFunction
 * @brief Read camera parameters from YAML file
 * @param cam camera object
 * @param fn file name
 * @author O.Grau
**/
void ReadCameraYAML( mistl::Camera &cam, const char *fn  );

  
/**
 * @ingroup MISTLCFunction
 * @brief Write Transform3f parameters to YAML file
 * @param tr Transform3 object
 * @param fn file name
 * @param id frame or sequence id
 * @author O.Grau
**/
void WriteTransform3YAML( const mistl::Transform3f &tr, const char *fn, unsigned id=0  );

/**
 * @ingroup MISTLCFunction
 * @brief Read Transform3f parameters from YAML file
 * @param tr Transform3 object
 * @param fn file name
 * @param id_p return sequence id 
 * @author O.Grau
**/
void ReadTransform3YAML( mistl::Transform3f &tr, const char *fn, unsigned *id_p = 0  );


}



#endif
