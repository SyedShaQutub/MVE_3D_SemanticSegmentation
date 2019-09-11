
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


#ifndef __MISTL_AuxFuncs_H__
#define __MISTL_AuxFuncs_H__

#include "stdio.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>

#ifdef ANDROID
#include "mistl/to_string.h"
#endif

namespace mistl {


std::vector<std::string>
SplitStr( const std::string &input );


std::string     ReplaceDirName( const std::string & ipath,  const std::string &dirnam );

std::string     ImageName( const std::string & ipath );

bool fileCheck(const char* const fileName);

bool fileExists(const char* const fileName);


    
}

#endif
