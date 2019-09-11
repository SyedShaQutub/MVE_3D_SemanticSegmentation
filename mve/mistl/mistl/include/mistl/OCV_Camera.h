

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
#include	<stdlib.h>
#include <stdio.h>
#include "opencv2/opencv.hpp"

#ifndef __MISTL_OCV_Camera_H__
#define __MISTL_OCV_Camera_H__


namespace mistl {

#ifdef _MSC_VER
#pragma warning( disable : 4244 ) // conversion from 'const double' to 'float', possible loss of data
#endif
  

class	OCV_Camera {
public:
  OCV_Camera() 
    : cameraMatrix(3,3,CV_64F), distCoeffs(1,5,CV_64F)
  {
    nx=ny=100;
    sx=sy=1;
    rvec= cv::Mat(3,1,CV_64F,0.0); //rvec.at<double>(0,0)=0; rvec.at<double>(1,0)=0; rvec.at<double>(2,0)=0; 
    tvec= cv::Mat(3,1,CV_64F,0.0); //tvec.at<double>(0,0)=0; tvec.at<double>(1,0)=0; tvec.at<double>(2,0)=0; 
  }

  cv::Mat cameraMatrix;
  cv::Mat	distCoeffs;
  cv::Mat rvec,tvec;

  unsigned	nx,ny;  //!< image size

  // these are redundant and can be compute with AdjustF(), AdjustS()
  double		sx,sy;  //!< pixel size
  double		f;      //!< calibration constant (aka focal length)

  void Info() const {
          std::cout << "mat-type:"<< cameraMatrix.type() <<"\n";
          std::cout << "Intrinsics:" << cameraMatrix <<"\n";
          std::cout << "rvec:" << rvec <<"\n";
          std::cout << "tvec:" << tvec <<"\n";
          std::cout << "Distortion:" << distCoeffs <<"\n";
  }
  //! Use this function to compute f and pixel ratio from one pixel length
  void	AdjustF( double s) {
    double fx= cameraMatrix.at<double>(0,0);
    double fy= cameraMatrix.at<double>(1,1);
    f = fx*s;
    sx = s;
    sy = fy/fx * s;
  }
  //! Use this function to compute pixel size and ratio from focal length
  void	AdjustS( float fin) {
    double fx= cameraMatrix.at<double>(0,0);
    double fy= cameraMatrix.at<double>(1,1);
    f = fin;
    sx = f/fx;
    sy = f/fy;
  }

  //! Copy parameters to Mistl camera
  void CopyTo( mistl::Camera	&cam ) const {
    float csx, csy;
    float px = cameraMatrix.at<double>(0, 2);
    float py = cameraMatrix.at<double>(1, 2);
          
    float k1 = distCoeffs.at<double>(0, 0);
    float k2 = distCoeffs.at<double>(0, 1);
    float p1 = distCoeffs.at<double>(0, 2);
    float p2 = distCoeffs.at<double>(0, 3);
    float k3 = distCoeffs.at<double>(0, 4);
    
    csy=(py- 0.5f*(ny-1) ) * sy;
    csx=(px- 0.5f*(nx-1) ) * sx;

    cam.SetTarget(nx,ny, sx,sy );
    MISTL_ASSERT( f != 0.0f, "invalid focal length. f=0");
    cam.SetF( f );
    cam.SetK1(k1);
    
    // tangential coefficients not yet supported!
//     cam.SetP1(p1);
//     cam.SetP2(p2);
    cam.SetK2(k2);
    cam.SetK3(k3);
    cam.SetCenterPointShiftX(csx);
    cam.SetCenterPointShiftY(csy);
    MISTL_ASSERT( cam.GetF() != 0.0f, "invalid focal length. f=0");

//    std::cout<<"CopyTo - rvec:"<< rvec <<std::endl;
    cv::Mat cvmat;
    if(rvec.cols == 3 && rvec.rows == 3) {	// yaml reader gives a 3x3 matrix, other a 1x3 vector

    	MISTL_ASSERT(rvec.type() == CV_64F, "converting cv::Mat has wrong type");
		MISTL_ASSERT((rvec.rows == 3) && (rvec.cols == 3), "converting cv::Mat has wrong size");

    	cvmat = rvec;
    }
    else {
    	cv::Rodrigues(rvec,cvmat);
    }
    mistl::Matrix3x3f rot( cvmat);

//    std::cout<<"con t: "<<"  tvec:"<<tvec;
    mistl::Vector3f     t( tvec ); 
//         std::cout<<" copy t-in:"<<t<<" tvec r,c:"<<tvec.rows<<","<<tvec.cols;
    t = rot.Inverse() * t;
        
//         std::cout<<"copy t:"<<t<< "  rvec:"<<rvec<<" cvmat:"<<cvmat<<"\n";
//         std::cout<<"copy rot.Inverse()"<<rot.Inverse()<<"\n";
        
    cam.SetC( t* -1.0f);
    mistl::Vector3f     a( rot(2,0), rot(2,1), rot(2,2) );
    mistl::Vector3f     up ( -rot(1,0), -rot(1,1), -rot(1,2) );
    cam.SetOrientation(a,up);
    
  }

  // Set parameters from MISTL camera
  void Set( const mistl::Camera       &cam ) {
    cameraMatrix= cv::Mat::zeros(3,3,CV_64F);          
    cameraMatrix.at<double>(0,2)=cam.GetPx();
    cameraMatrix.at<double>(1,2)=cam.GetPy();
    cameraMatrix.at<double>(2,2)=1.0;
    f=cam.GetF();
    sx=cam.GetSx();
    sy=cam.GetSy();
    nx=cam.GetNx();
    ny=cam.GetNy();
    cameraMatrix.at<double>(0,0)= f/sx;
    cameraMatrix.at<double>(1,1)= f/sy;
          
    distCoeffs= cv::Mat::zeros(1,5,CV_64F);
    distCoeffs.at<double>(0,0)=cam.GetK1();
    distCoeffs.at<double>(0,1)=cam.GetK2();
//     distCoeffs.at<double>(0,2)=cam.GetP1();
//     distCoeffs.at<double>(0,3)=cam.GetP2();
    distCoeffs.at<double>(0,4)=cam.GetK3();
          
    //  convert tvec+rvec
	  cv::Mat cvmat= cam.GetPose().rot.cvMat();
	  cv::Rodrigues(cvmat,rvec);
          
	  mistl::Vector3f     t(cam.C()*-1.0);
    std::cout<<"t-in:"<<t<<"\n";

    mistl::Matrix3x3f poseRotationMatrix = cam.GetPose().rot;
    t = poseRotationMatrix * t;
	  tvec= t.cvMat();
//  std::cout<<"set t:"<<t<<"  tvec:"<<tvec<< "  rvec:"<<rvec<<"\n";
  }
        
	void    Project( const cv::Point3f &P, cv::Point2f &p) const {
          std::vector<cv::Point3f> op;
          std::vector<cv::Point2f> ip;
          op.push_back(P);
          ip.push_back(cv::Point2f());
          cv::projectPoints( op, rvec,tvec, cameraMatrix, distCoeffs, ip );
          p = ip[0];
        }
};

void	ReadCameraYAML( mistl::OCV_Camera	&cam, const char *fn);

void WriteCameraYAML( mistl::OCV_Camera &cam, const char *fn);

}

#endif
