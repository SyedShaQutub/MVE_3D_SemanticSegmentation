
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


#ifndef  NiceDataDump_incl_
#define  NiceDataDump_incl_


#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/RunCalibration.h"


#include <iostream>
#include <fstream>

#include "opencv2/core/core.hpp"



namespace mistl {
  

/*! \class NiceDataDump NiceDataDump.h
  \brief Class to generate a HTML visulization of calibration results
  \author O.Grau
*/
class NiceDataDump  {
public:           
  NiceDataDump();
  ~NiceDataDump();

public:
  //! Set directory path to dump data in.
  void OpenDumpDir( const std::string dir );
  
  //!
  void AddEntry( const cv::Mat &refimg, mistl::RunCalibration &cal );
  
//                  mistl::CalibrationCode is_in_calib, //!< 0=out of cal, 1=unknown, 2=in calibration
//                  bool good_for_recalib, 
//                  float quality,   
//                  float confidence 
                 
//                );
  
  void Close();
  
//   void  Info() const;
  
  float img_scale;
  unsigned images_per_row;
 
 
protected:
  void EmphText( const std::string &text, const std::string color ) ;
  void EmphThr( const std::string &text, float v ) ;


  void WriteHeader();
  void WriteFooter();
  
  unsigned imgcount;
  std::string dir;
  std::string htmlfn;
  std::ofstream htmlfile;
};




}
#endif
