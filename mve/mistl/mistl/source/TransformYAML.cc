

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
#include	"mistl/Transform3.h"
#include	"mistl/YAML_IO.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "opencv2/core/core.hpp"


namespace mistl {
  



void WriteTransform3YAML( const mistl::Transform3f &tr, const char *fn, unsigned id  )
{
  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  MISTL_ASSERT( fs.isOpened(), "WriteCameraYAML: error opening file" );

  fs << "Transform3" << "{" ;
    fs << "Id" << static_cast<int>(id);
    fs << "T"; WriteVector( fs, tr.t);
    fs << "R";
    fs << tr.rot.cvMat();
  fs << "}";
}

void ReadTransform3YAML( mistl::Transform3f &tr, const char *fn, unsigned *id_p  )
{
  cv::FileStorage fs(fn, cv::FileStorage::READ);
  MISTL_ASSERT( fs.isOpened(), "WriteCameraYAML: error opening file" );
  
  cv::FileNode tm = fs["Transform3"];
  if(id_p) 
    *id_p = static_cast<unsigned>((int)tm["Id"]);
  tr.t = mistl::ReadVector( tm,"T");
  cv::Mat cvmat;
  tm["R"] >> cvmat;
  tr.rot = mistl::Matrix3x3f(cvmat);

    
};

  

	
}
