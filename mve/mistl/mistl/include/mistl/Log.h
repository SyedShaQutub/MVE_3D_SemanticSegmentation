
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
//	 
//

// #include	<string.h>
#include	<cstring>
#include        <iostream>


#ifndef __MISTL_LOG_H__
#define __MISTL_LOG_H__



namespace mistl {

 /**
 * @ingroup MISTLCFunction
 * @brief Return debug mode flag
**/ 
bool DebugMode();

 /**
 * @ingroup MISTLCFunction
 * @brief Set debug mode flag
**/ 
void SetDebugMode( bool v);

 /**
 * @ingroup MISTLCFunction
 * @brief Set log directory
 * 
 * This should be use for log-files and debug data. The function tries to created the directory.
**/ 
void SetLogDirectory( const std::string dir );


/**
 * @ingroup MISTLCFunction
 * @brief Return log directory
**/ 
const std::string       GetLogDirectory();

 /**
 * @ingroup MISTLCFunction
 * @brief Set log file
 * 
 * In the current implementation all output to stdout/std::cout is redirected into the log-file.
**/ 
void    SetLogFile( const std::string dir );



}

#endif