

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
#include        "mistl/DR_CameraArray.h"
#include        "mistl/Log.h"
#include "stdio.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include "opencv2/core/core.hpp"

// ndk compiler does not support std::to_string
#ifdef ANDROID
#include "mistl/to_string.h"
#endif

namespace mistl {


//
// needs to be implemented .....
void WriteCameraYAML( mistl::DR_CameraArray &cam, const char *fn  )
{
  const char *cp = strrchr(fn,'.');

    if(!cp){
      std::cout << "ReadCameraYAML(DR_CameraArray): Did not find a . in file name: " << fn << std::endl;
      return;
    }

    if(!strcmp(cp, ".ycam")) {

      char *w = new char[strlen(fn)+10];
      for(unsigned i=0; i<3; ++i) {
        sprintf(w,fn,i);
        std::cout<<"Try to read camera:"<<w<<"\n";
        mistl::WriteCamera( cam.camera[i], w);
      }
      delete [] w;
    } else {
      cv::FileStorage fs(fn, cv::FileStorage::WRITE);
      MISTL_ASSERT( fs.isOpened(), "WriteCameraYAML: error opening file" );


      // writing DR properties
      fs << "calibration_version" << cam.calibration_version;
      fs << "calibration_type" << cam.calibration_type;
      fs << "number_of_cameras" << static_cast<int>(cam.number_of_cameras);
      fs << "number_of_views" << cam.number_of_views;
      fs << "rms_reprojection_error" << cam.rms_reprojection_error;
      fs << "camera_type"<< cam.camType;

      fs << "camera_calibrations" << "{" ;

      for(unsigned i=0; i<3; i++) {
    	// prepare an "iterative string": camera_0 or camera_1 or camera_2 ...		// TODO: use sprintf
    	char camNamePrefix[30] = "camera_";
    	char integer_string[3];
    	sprintf(integer_string, "%d", i);
    	std::strcat(camNamePrefix, integer_string);
    	fs << camNamePrefix << "{";



    	//
    	// initialize and transform mistl camera data so it can be written
    	// in DR format
    	//

    	// initialize camera Id
    	fs << "camera_id" << static_cast<int>(i);
    	if(i == cam.refCam) {			// states if the camera is the reference camera
    		fs << "reference" << 1;
    	}
    	else {
    		fs << "reference" << 0;
    	}
    	
    	
    	// !!!!!!!!!!!!! "position" not stored !!!!!!!!!!!!!!
    	// Is this needed then it has to be carried over from input file !!!!
//     	fs << "position" << "[:";		// TODO: store position when reading dr-file
//     	fs << 0 << 0;
//     	fs << "]" ;


    	// transform intrinsic camera data
    	cv::Mat K= cv::Mat::zeros(3,3,CV_64F);
    	K.at<double>(0,2)=cam.camera[i].GetPx();
    	K.at<double>(1,2)=cam.camera[i].GetPy();
    	K.at<double>(2,2)=1.0;
    	float f=cam.camera[i].GetF();
    	float sx=cam.camera[i].GetSx() * 1.0e3 ;  // mistl camera keeps everything in [m]. DR pixel size is in [mm]??
    	float sy=cam.camera[i].GetSy() * 1.0e3 ;
    	int nx=cam.camera[i].GetNx();
    	int ny=cam.camera[i].GetNy();
    	K.at<double>(0,0)= f / cam.camera[i].GetSx();
    	K.at<double>(1,1)= f / cam.camera[i].GetSy();		

    	// transforme distortion data
    	cv::Mat D = cv::Mat::zeros(1,5,CV_64F);
    	D.at<double>(0,0)=cam.camera[i].GetK1();
    	D.at<double>(0,1)=cam.camera[i].GetK2();
    	D.at<double>(0,2)=cam.camera[i].GetP1();
    	D.at<double>(0,3)=cam.camera[i].GetP2();
    	D.at<double>(0,4)=cam.camera[i].GetK3();

    	// transforme rotation data
    	cv::Mat rvec=cam.camera[i].GetPose().rot.cvMat();

    	// transform translation  data
    	cv::Mat tmpTvec;
    	cv::Mat tvec= cv::Mat(1,3,CV_64F,0.0);
        
        // MISTL camera use [m], DR [mm]
        // If DR parameters are used in OpenCV manner, then DR-C might be a translation not center of camera
        // then apply negative sign!!!

//         mistl::Vector3f     t(cam.camera[i].C()* -1000.0f);
        mistl::Vector3f     t(cam.camera[i].C()* 1000.0f);
        
    	mistl::Matrix3x3f poseRotationMatrix = cam.camera[i].GetPose().rot;
//     	t = poseRotationMatrix * t;
    	tmpTvec= t.cvMat();
    	cv::transpose(tmpTvec ,tvec);

        // populate matrices from DR that are not used in mistl
        cv::Mat H_L= cv::Mat::eye(3,3,CV_64F);
        cv::Mat H= cv::Mat::eye(3,3,CV_64F);
        cv::Mat H2= cv::Mat::eye(3,3,CV_64F);
        cv::Mat H2_L= cv::Mat::eye(3,3,CV_64F);
        cv::Mat rectifying_K_L= cv::Mat::eye(3,3,CV_64F);
        cv::Mat rectifying_K= cv::Mat::eye(3,3,CV_64F);
        cv::Mat rectifying_transform= cv::Mat::eye(3,3,CV_64F);
        cv::Mat rectifying_camera_position= cv::Mat::zeros(3,1,CV_64F);
        
        
    	//
    	// write transformed camera data to DR file
    	//
		fs << "position" << "[:";			// position was missing here
                //hack
                if(i==0) 
                  fs << -43.5f << -37.5f;
                else if(i==1) 
                  fs << -43.5f << 37.5f;
                else 
                  fs << 0.f << 0.f;
                
		fs << "]";

        fs << "resolution" << "[:";
        fs << nx << ny << 1;
        fs << "]" ;

        fs << "pixel_size" << "[:";
        fs << sx << sy;
        fs << "]" ;

    	fs << "K" << K;
    	fs << "D" << D;
    	fs << "R" << rvec;
    	fs << "C" << tvec;

        fs << "H_L" << H_L;
        fs << "H" << H;
        fs << "H2" << H2;
        fs << "H2_L" << H2_L;
        fs << "rectifying_K_L" << rectifying_K_L;
        fs << "rectifying_K" << rectifying_K;
        fs << "rectifying_transform" << rectifying_transform;
        fs << "rectifying_camera_position" << rectifying_camera_position;
        



    	fs << "}";
      }

      fs << "}";

      fs.release();
    }
 
}


bool fileExistsYAML(const char* const fileName){
  FILE* testFile = fopen(fileName, "r");

  // check if the file opened.
  if (!testFile){
    return false;
  }

  fclose(testFile);

  return true;
}

//
// This is a dummy  at the moment !!!!
///
void ReadCameraYAML( mistl::DR_CameraArray &cam, const char *fn , bool set_only_k1k2  )
{
 const char *cp = strrchr(fn,'.');

  if(!cp){
    std::cout << "ReadCameraYAML(DR_CameraArray): Did not find a . in file name: " << fn << std::endl;
    return;
  }

//  if (!fileExistsYAML(fn)){
//    std::cout << "ReadCameraYAML. File not found: " << fn << std::endl;
//    exit(1);
//  }
  
  if(!strcmp(cp, ".ycam")) {
  
    char *w = new char[strlen(fn)+10];
    for(unsigned i=0; i<3; ++i) {
      sprintf(w,fn,i);
      std::cout<<"Try to read camera:"<<w<<"\n";

      mistl::ReadCamera( cam.camera[i], w);
    }
    delete [] w;
    // !!!!!!!!!!! hack to ensure local ycam readin works!
    cam.refCam=2;
  } else {

	cv::FileStorage fs(fn, cv::FileStorage::READ);


	// read DR prperties
        int v;
	cam.calibration_version = fs["calibration_version"];
// 	std::string tmpString = (std::string) fs["calibration_type"];
// 	cam.calibration_type = tmpString.c_str();
        if( fs["calibration_type"].type()==cv::FileNode::STRING)
          cam.calibration_type = (std::string)fs["calibration_type"];
        else
          cam.calibration_type = "unknown";
	 v= fs["number_of_cameras"]; cam.number_of_cameras= static_cast<unsigned>(v);
	cam.number_of_views = fs["number_of_views"];
	cam.rms_reprojection_error = fs["rms_reprojection_error"];
	if( fs["camera_type"].type()==cv::FileNode::STRING) cam.camType = (std::string)fs["camera_type"];



	// prepare data structures for openCV yaml reader
	cv::FileNode nodeTmp2;
	cv::FileNode nodeTmp;
	cv::FileNodeIterator it;
	cv::FileNode node = fs["camera_calibrations"];

        if(node.empty() ) 
          throw  mistl::Error("ReadCameraYAML: no calibration node found in file");   
        
	for(int i=0; i<3; i++) {

		// prepare an "iterative string": camera_0 or camera_1 or camera_2 ...
// 		char camNamePrefix[30] = "camera_";
// 		char integer_string[3];
// 		sprintf(integer_string, "%d", i);
// 		std::strcat(camNamePrefix, integer_string);
#ifdef ANDROID
		std::string camNamePrefix = "camera_" + to_string(i);
#else
		std::string camNamePrefix = "camera_"+std::to_string(i);
#endif                

		// initializing data structures for reading the yaml data
		/*cv::Mat*/ cam.cameraMatrix = cv::Mat(3,3,CV_64F);
		/*cv::Mat*/ cam.distCoeffs = cv::Mat(1,5,CV_64F);
// 		cv::Mat rvec= cv::Mat(3,1,CV_64F,0.0);
		/*cv::Mat*/ cam.tvec= cv::Mat(3,1,CV_64F,0.0);

		unsigned	nx,ny;
		float		px, py;
		double		sx = 0.0, sy = 0.0;
		double		fx,fy;  // calibration constant in [pel]
		float 		csx, csy;
		int camId = -1;


		nodeTmp = node[camNamePrefix];
		camId = nodeTmp["camera_id"];
		bool isRefCam;
		nodeTmp["reference"] >> isRefCam;
		if(isRefCam) {
			cam.refCam = camId;
		}

		//
		// write data to preinitialized data-structures
		//
		nodeTmp["K"] >> cam.cameraMatrix;
		nodeTmp["D"] >> cam.distCoeffs;
		nodeTmp["R"] >> cam.rotmat;
		nodeTmp["C"] >> cam.tvec;

		// read resolution
		nodeTmp2 = nodeTmp["resolution"];
		if (nodeTmp2.type() != cv::FileNode::SEQ)
		{
			std::cerr << "can not read resolutionfor cam: " << i <<" strings is not a sequence! FAIL" << std::endl;
			return ;
		}
		it = nodeTmp2.begin();
		int resX = *it++;
		int resY = *it;
		nx  = resX;
		ny = resY;

		// read pixel_size
		nodeTmp2 = nodeTmp["pixel_size"];
		if (nodeTmp2.type() != cv::FileNode::SEQ)
		{
// 			std::cerr << "pixel_size: strings is not a sequence! FAIL" << std::endl;
// 			return ;
                  sx=sy= 1.f;
		} else {
                  it = nodeTmp2.begin();
                  double pixel_size_X = *it++;
                  double pixel_size_Y = *it;
                  sx = pixel_size_X * 1.0e-3 ;  // mistl camera keeps everything in [m]. DR pixel size is in [mm]
                  sy = pixel_size_Y * 1.0e-3 ;
                }

		// initialize remaining values for setting mist cam more easily
                fx = cam.cameraMatrix.at<double>(0, 0);
                fy = cam.cameraMatrix.at<double>(1, 1);     //!!!!!!!!!!!!!!!!! DR is not coherent here, there is a pixel size
                                                        // that defines the pixel aspect ratio and it is not equal to the 
                                                        // fx/fy ratio !!!!!!!!!!!!!!
                if( fabsf( fx-fy ) > 1e-10 ) 
                  sy = sx * fx/fy;
                
                px = cam.cameraMatrix.at<double>(0, 2);
		py = cam.cameraMatrix.at<double>(1, 2);
		csy=(py- 0.5f*(ny-1) ) * sy;
		csx=(px- 0.5f*(nx-1) ) * sx;


		//
		// writing data to mistl cam
		//

		cam.camera[camId].SetTarget(nx,ny, sx,sy );
                cam.camera[camId].name = camNamePrefix;

		// writing intrinsics
//		double sx = pixel_size_X;
//		cam.camera[camId].ocv_cam.AdjustF(sx);
		MISTL_ASSERT( fx != 0.0f, "invalid focal length. f=0");
		cam.camera[camId].SetF( fx * sx );
		cam.camera[camId].SetK1(cam.distCoeffs.at<double>(0, 0));
		cam.camera[camId].SetK2(cam.distCoeffs.at<double>(0, 1));
                
                // tangential coefficients not yet supported
                if(!set_only_k1k2) {
                  cam.camera[camId].SetP1(cam.distCoeffs.at<double>(0, 2));
                  cam.camera[camId].SetP2(cam.distCoeffs.at<double>(0, 3));
                  
                  cam.camera[camId].SetK3(cam.distCoeffs.at<double>(0, 4));
                } else {
                  cam.camera[camId].SetP1(0.f);
                  cam.camera[camId].SetP2(0.f);
                  cam.camera[camId].SetK3(0.f);
                }
                
		cam.camera[camId].SetCenterPointShiftX(csx);
		cam.camera[camId].SetCenterPointShiftY(csy);
		MISTL_ASSERT( cam.camera[camId].GetF() != 0.0f, "invalid focal length. f=0");

		
// 		if(rvec.cols == 3 && rvec.rows == 3) {	// yaml reader gives a 3x3 matrix, other a 1x3 vector
// 
// 			MISTL_ASSERT(rvec.type() == CV_64F, "converting cv::Mat has wrong type");
// 			MISTL_ASSERT((rvec.rows == 3) && (rvec.cols == 3), "converting cv::Mat has wrong size");
// 			cvmat = rvec;
// 		}
// 		else {
// 			cv::Rodrigues(rvec,cvmat);
// 		}
		mistl::Matrix3x3f rot( cam.rotmat);
		mistl::Vector3f     camcenter( cam.tvec );
// 		t = rot.Inverse() * t;

                // MISTL camera use [m], DR [mm]
                // If DR parameters are used in OpenCV manner, then DR-C might be a translation not center of camera
                // then apply negative sign!!!
//                 cam.camera[camId].SetC( camcenter* -0.001f);
                cam.camera[camId].SetC( camcenter* 0.001f);
                
		mistl::Vector3f     a( rot(2,0), rot(2,1), rot(2,2) );
		mistl::Vector3f     up ( -rot(1,0), -rot(1,1), -rot(1,2) );
		cam.camera[camId].SetOrientation(a,up);



                if(mistl::DebugMode()) {
                  std::cout << "Camera: " << camId << " read: \n" << std::endl;
                  cam.camera[camId].Info();
                  std::cout << "\n" << std::endl;
                }
	}
  }
  
  
};

  

	
}
