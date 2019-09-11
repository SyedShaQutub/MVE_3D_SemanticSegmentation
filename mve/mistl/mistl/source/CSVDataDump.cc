

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

#include        "mistl/CSVDataDump.h"
#include        <vector>
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>


namespace mistl {

#ifndef M_PI
  #define M_PI 3.1415926535897932384626433832795;
#endif
  

CSVDataDump::CSVDataDump()
{
  addheader= false; //true;
  separator=' ';
}


CSVDataDump:: ~CSVDataDump()
{
  Close();
  
}

// use namespace std;

void 
CSVDataDump::Open( const std::string fn )
{
  
//   htmlfn = dir + "/" + "results.csv";
  if(fn.substr(fn.find_last_of(".") + 1) == "txt") 
    separator=' ';
  else
    separator=',';
  htmlfile.open ( fn );
  
  WriteHeader();
  imgcount=0;
}


/* Generate one data entry
(numbers starting from 1 !!!!

1  image set number (sequence id)
2  tracking quality
3  tracking quality confidence
4  calibration quality
5  calibration confidence
6  quick point match
7  initial number of points
8  residual error of 3d point assembly
9  calibration residual error
10 number of equations (2d matches) evaluated

[11 .. (camera 0)]
11 focal length in [mm]
12 center point shift x in [pel]
13 center point shift y in [pel]
14 camera rotation in [deg]
15 distortion k1
16 distortion k2
17 distortion k3

[18.. camera 1]

[25.. camera 2]

32 near 3d confidence
33 mid 3d confidence
34 far 3d confidence

35 3d hist N
36 near 
37 mid
38 far

39 2d hist N
40-48 2d histogram

49 Difference to reference camera

50 Outliers removed in BA
51 number of 3d keypoints (approximation)

*/

void 
CSVDataDump::AddEntry( const RunCalibration &cal )
{  

  htmlfile << imgcount;
  // 2,..
  htmlfile << separator<< cal.q.trackquality.GetQuality()<<separator<< cal.q.trackquality.GetConfidence();
  // 4,5
  htmlfile << separator<< cal.q.GetQuality()<<separator<< cal.q.GetConfidence();
  // stats for quality (6..)
  htmlfile << separator<< cal.q.initialmatch;
  htmlfile << separator<< cal.q.initial_n;
  htmlfile << separator<< cal.q.p3dmatch;
  htmlfile << separator<< cal.q.finmatch;
  htmlfile << separator<< cal.q.fin_n;
  // cam data (11..
  mistl::Vector3f v(0., 0., 1.); // = cal./*new*/camlist.at(2).A();
  for(unsigned i=0; i<3;++i) {
    htmlfile << separator<< cal./*new*/camlist.at(i).GetF()*1000.0f ;
    htmlfile << separator<< cal./*new*/camlist.at(i).GetCenterPointShiftX()/cal./*new*/camlist.at(i).GetSx();
    htmlfile << separator<< cal./*new*/camlist.at(i).GetCenterPointShiftY()/cal./*new*/camlist.at(i).GetSy();
    float r= v.Dot(cal./*new*/camlist.at(i).A());
    r = acos (r) * 180.0 / M_PI;
    htmlfile << separator<< r;
    htmlfile << separator<< cal./*new*/camlist.at(i).GetK1();
    htmlfile << separator<< cal./*new*/camlist.at(i).GetK2();
    htmlfile << separator<< cal./*new*/camlist.at(i).GetK3();
  }
 
  float near,mid,far;
  cal.Judge3DHistogram ( near,mid,far );
  htmlfile << separator<< near << separator << mid << separator << far;
  
  // 35
  htmlfile << separator<< cal.histogram3d.N() ;
    htmlfile << separator<< cal.histogram3d.Get(0,0,0);
    htmlfile << separator<< cal.histogram3d.Get(0,0,1);
    htmlfile << separator<< cal.histogram3d.Get(0,0,2);
  // 39
  htmlfile << separator<< cal.histogram2d.N() ;
  for(unsigned i=0; i<cal.histogram2d.Ny();++i)
    for(unsigned j=0; j<cal.histogram2d.Nx();++j)
      htmlfile << separator<< cal.histogram2d.Get(j,i,0);
    
  // 49
  float r = cal.DistanceToRef();
  htmlfile << separator<< r;
  
  // 66
  htmlfile << separator<< cal.outliers;
    htmlfile << separator<<  cal.q.fin_n / cal.camlist.size();

  htmlfile << "\n";
  
  imgcount++;
}

  
void CSVDataDump::Close()
{
  htmlfile.close();
}



void 
CSVDataDump::WriteHeader()
{
  if(addheader) {
    htmlfile << "frame,tracking-quality,tracking-confidence,calibration-quality,calibration-confidence,cam0-f,cam0-cs-x,cam0-cs-y";
    htmlfile << "\n";
  }
  
}



  
}
