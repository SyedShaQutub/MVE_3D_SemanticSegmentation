

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
#include	"mistl/Camera.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "opencv2/core/core.hpp"


namespace mistl {
  
void    WriteVector( cv::FileStorage &fs, const mistl::Vector3f &v )
{
//   fs <<  "{" << "x" << v.X() << "y" << v.Y() << "z" << v.Z() << "}";
//   fs << "[" << v.X()  << v.Y()  << v.Z() << "]";
  std::vector<double> vec;
  vec.push_back(v.X());
  vec.push_back(v.Y());
  vec.push_back(v.Z());

  fs << vec;
}

mistl::Vector3f ReadVector( cv::FileNode &fs, const char *nam )
{
  cv::FileNode tl = fs[nam];
  MISTL_ASSERT(tl.type() == cv::FileNode::SEQ && tl.size() == 3, "ReadVector: wrong type or length");
  return mistl::Vector3f( (double)tl[0], (double)tl[1],(double)tl[2] );
}



void WriteCameraYAML( const mistl::Camera &cam, const char *fn  )
{
  cv::FileStorage fs(fn, cv::FileStorage::WRITE);
  MISTL_ASSERT( fs.isOpened(), "WriteCameraYAML: error opening file" );

  const float epsilon = 0.0f;
  
  fs << "Camera" << "{" ;
    fs << "Name" << cam.name;
    fs << "C"; WriteVector( fs, cam.C());
    fs << "A"; WriteVector( fs, cam.A());
    fs << "Up"; WriteVector( fs, cam.Up());
    fs << "F" << cam.GetF();
    fs << "Sx" << cam.GetSx();
    fs << "Sy" << cam.GetSy();
    fs << "K1" << cam.GetK1();
    fs << "K2" << cam.GetK2();
    if(fabsf( cam.GetK3()) > epsilon)  fs << "K3" << cam.GetK3();
    if(fabsf( cam.GetP1()) >epsilon)  fs << "P1" << cam.GetP1();
    if(fabsf( cam.GetP2()) >epsilon)  fs << "P2" << cam.GetP2();
    fs << "Nx" << cam.GetNx();
    fs << "Ny" << cam.GetNy();
    fs << "CenterPointShiftX" << cam.GetCenterPointShiftX();
    fs << "CenterPointShiftY" << cam.GetCenterPointShiftY();
  fs << "}";
}

void ReadCameraYAML( mistl::Camera &cam, const char *fn  )
{
  cv::FileStorage fs(fn, cv::FileStorage::READ);
  MISTL_ASSERT( fs.isOpened(), "WriteCameraYAML: error opening file" );
  
  cv::FileNode tm = fs["Camera"];
  
    cam.name = (std::string)tm["Name"];

    cam.SetF(tm["F"]);
    cam.SetK1(tm["K1"]);
    cam.SetK2(tm["K2"]);
    cam.SetK3(tm["K3"]);
    
    if(tm["P1"].type()==cv::FileNode::FLOAT) cam.SetP1(tm["P1"]);
    if(tm["P2"].type()==cv::FileNode::FLOAT) cam.SetP2(tm["P2"]);
    
    cam.SetC( ReadVector(tm,"C"));
    mistl::Vector3f a= ReadVector(tm,"A");
    mistl::Vector3f up= ReadVector(tm,"Up");
    cam.SetOrientation(a,up);
    float sx = tm["Sx"];
    float sy = tm["Sy"];
    unsigned nx = static_cast<unsigned>((int)tm["Nx"]);
    unsigned ny = static_cast<unsigned>((int)tm["Ny"]);
    cam.SetTarget( nx,ny, sx,sy );
    cam.SetCenterPointShiftX( tm["CenterPointShiftX" ] );
    cam.SetCenterPointShiftY( tm["CenterPointShiftY" ] );
};

  

	
}
