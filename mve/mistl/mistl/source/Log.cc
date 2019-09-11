
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



#include        "mistl/Log.h"
#include <cstdio>
#include <iostream>
#include <sys/stat.h>

#if defined(_WIN32)
#include <direct.h>
#endif


namespace mistl {

bool    debugmode = false;
std::string logdir;
std::string logfn;


bool
DebugMode() {return debugmode;}

void SetDebugMode( bool v) { debugmode=v; }

void SetLogDirectory( const std::string dir ) { 
  logdir=dir;

#if defined(_WIN32)
  _mkdir(logdir.c_str());
#else
  mkdir( logdir.c_str() ,  0777);
#endif
}



const std::string       GetLogDirectory()
{
  return logdir;
}


void    SetLogFile( const std::string fn )
{
  logfn=fn;
  // error FIXME: The return value of function 'freopen' is required to be utilized.
  freopen(fn.c_str(),"w",stdout);
}




}

