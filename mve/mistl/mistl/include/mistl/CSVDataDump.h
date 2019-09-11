
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


#ifndef  CSVDataDump_incl_
#define  CSVDataDump_incl_


#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/RunCalibration.h"


#include <iostream>
#include <fstream>

#include "opencv2/core/core.hpp"



namespace mistl {
  

/*! \class CSVDataDump CSVDataDump.h
  \brief Class to generate a HTML visulization of calibration results
  \author O.Grau
*/
class CSVDataDump  {
public:           
  CSVDataDump();
  ~CSVDataDump();

public:
  //! Open file to dump data in. Sets separator according file extension: .txt=' ', .csv=','
  void Open( const std::string fn );
  
  //!
  void AddEntry( const RunCalibration &cal );

  void Close();
  
  bool addheader;
  char  separator;
 
protected:
  void WriteHeader();

  
  unsigned imgcount;
  std::string dir;
  std::string htmlfn;
  std::ofstream htmlfile;
};




}
#endif
