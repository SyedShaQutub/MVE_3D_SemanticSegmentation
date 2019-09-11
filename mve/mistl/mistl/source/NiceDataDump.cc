

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


#include        "mistl/Error.h"
#include        "mistl/Log.h"

#include        "mistl/NiceDataDump.h"
#include        <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>

// ndk compiler does not support std::to_string
#ifdef ANDROID
#include "mistl/to_string.h"
#else
#define		to_string	std::to_string
#endif

namespace mistl {

  

NiceDataDump::NiceDataDump()
{
  img_scale = 0.05f;
  images_per_row = 5;
}


NiceDataDump:: ~NiceDataDump()
{
  Close();
  
}

// use namespace std;


void NiceDataDump::EmphText( const std::string &text, const std::string color ) 
{
  htmlfile << "<font color = \"#" << color << "\">" << text << "</font>\n";  
}


void NiceDataDump::EmphThr( const std::string &text, float v ) 
{
  if(v>0.5)   EmphText( text,"00ff00");
  else EmphText( text,"ff0000");
}


void 
NiceDataDump::OpenDumpDir( const std::string dir_ )
{
  
  htmlfn = dir_ + "/" + "!OpenMe.html";
  htmlfile.open ( htmlfn );
  
  WriteHeader();
  imgcount=0;
}
  
void 
NiceDataDump::AddEntry( const cv::Mat &refimg,  mistl::RunCalibration &cal

//                 mistl::CalibrationCode is_in_calib, 
//                         bool good_for_recalib, 
//                         float quality,
//                         float confidence,
                      )
{  


  cv::Mat thumbnail;
  cv::Size size( (int)floorf(img_scale*refimg.cols+0.5), (int)floorf(img_scale*refimg.rows+0.5) );
  cv::resize( refimg, thumbnail, size ); //, 0.0, 0.0, CV_INTER_CUBIC );
  std::string fbase= "thumb_"+ to_string(imgcount)  + ".jpg";

  std::string   fn = mistl::GetLogDirectory()+ "/" + fbase;
  cv::imwrite( fn.c_str(), thumbnail);

  htmlfile << "<td>\n";
  htmlfile << " <IMG SRC=\"" << fbase << "\"><br /> \n";
  htmlfile << imgcount << " " ;
  
//   if(is_in_calib==mistl::InCalibration)
//     htmlfile << "<font color = \"#00FF00\">In</font>\n";  
//   else if (is_in_calib==mistl::OutOfCalibration)
//     htmlfile << "<font color = \"#FF0000\">Out</font>\n";  \
//   else
//     htmlfile << "<font color = \"#808080\">Unknown</font>\n"; 
    
  htmlfile << " ";
  
//   if(good_for_recalib)
//     htmlfile << "<font color = \"#0080dd\">Good</font>\n";  
//   else
//     htmlfile << "<font color = \"#801010\">Bad</font>\n";  
  
  
  if( (cal.q.fin_n<(unsigned int)cal.minimum_n) | (cal.q.trackrc!= mistl::Success) ) {
    EmphThr( "N", 0.f );
    EmphThr( "M", 0.f );
    EmphThr( "F", 0.f );
  } else {
    float near,mid,far;
    cal.Judge3DHistogram(  near,mid,far );
    
    EmphThr( "N", near );
    EmphThr( "M", mid );
    EmphThr( "F", far );
  }
  
  
  htmlfile << " q:"<< cal.q.GetQuality() << " c:"<< cal.q.GetConfidence();
  
//   htmlfile << " # " << fbase;
  htmlfile << "</td>\n";
  if( ( (imgcount+1)%images_per_row)==0) htmlfile << "</tr>\n";  
  
  imgcount++;
}

  
void NiceDataDump::Close()
{
  WriteFooter();
  htmlfile.close();
}



void 
NiceDataDump::WriteHeader()
{
  htmlfile << "<HEAD>\n<TITLE>Adaptive Calibration Page</TITLE>\n</HEAD>\n";
  htmlfile << "<BODY BGCOLOR=\"WHITE\">\n<H1>Adaptive Calibration</H1>\n";
  
//   htmlfile << "<style>\n";
//   body {
//     background-color: yellow;
// }
// 
// </style>
  
  htmlfile << "<table border=\"1\" align=\"center\" width=\"90%\">";
  htmlfile << "<caption>Picture Gallery</caption>";
  htmlfile << "<tr>";
// <td></td>
// <td></td>
// <td></td>
// </tr>
// <tr>
// <td></td>
// <td></td>
// <td></td>
}

void 
NiceDataDump::WriteFooter()
{
  htmlfile << "</tr>\n";
  htmlfile << "</table>\n";
  htmlfile << "\n</body>\n";
}

  
}
