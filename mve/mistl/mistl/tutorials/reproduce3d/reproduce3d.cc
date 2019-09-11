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
//    Author: Jonas Scheer
//


//#include 	"opencv2/opencv.hpp"
//#include 	<opencv2/core/core.hpp>
//#include 	<opencv2/highgui/highgui.hpp>
//#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
#include	"mistl/Line3.h"
#include <math.h>
//#include    "mistl/WriteOBJ.h"
//#include	"mistl/OCV_Camera.h"
#include	"mistl/Error.h"
#include	<stdlib.h>
#include 	<stdio.h>
#include 	<string.h>


using namespace std;

void
PrMsg() {
//	std::cout<<"\nThis program tests a stereo calibration by performing a stereo rectification: \n";
//	std::cout<<"usage: testCalib input-calibration-file_1 input-calibration-file_2 input_image_1 input_image_2 output_image_1 output_image_2 [concatinated_output_image] \n";
//	std::cout<<"    input-calibration-file_1 and input-calibration-file_2 need to be *.ycam or *.cahv calibration files \n";
//	std::cout<<"    optional concatinated_output_image parameter concatenates the rectified images and draws the epipolar lines\n";

	std::cout<<"    error\n";
	exit(-1);
}


int main(int argc, char** argv)
{
	cout<<"2D -> 3D Projection " << endl;

	if( /*(argc<4 && argv[3]=="-file") || */ (argc<7) ) {
		PrMsg();
		return -1;
	}

	// read in two cameras
	mistl::Camera cam1;
	mistl::Camera cam2;
	int x1;
	int x2;
	int y1;
	int y2;
	try {

		char* camFile1 = argv[1];
		char* camFile2 = argv[2];

		ReadCamera(cam1, camFile1);
		ReadCamera(cam2, camFile2);
	}
	catch ( mistl::Error e) {
		std::cout<<"Error: can not read cameras - "<<e.GetMsg()<<"\n";
		PrMsg();
		return -1;
	}


	// read input 2d-coordinates for projection to 3d
	if(argv[3] == "-file") {
		std::cout<<"file\n";
	}
	else {
		char* inputCoord1_x = argv[3];
		char* inputCoord1_y = argv[4];

		char* inputCoord2_x = argv[5];
		char* inputCoord2_y = argv[6];

		x1 = atoi(inputCoord1_x);
		x2 = atoi(inputCoord2_x);
		y1 = atoi(inputCoord1_y);
		y2 = atoi(inputCoord2_y);
		cout << "projecting values (" << x1  << ", " << y1 << "); (" << x2 << ", " << y2 << ") -> ";
	}

	mistl::Line3      lin1( cam1.C(), cam1.C() + cam1.LineOfSight( x1, y1 ));
	mistl::Line3      lin2( cam2.C(), cam2.C() + cam2.LineOfSight( x2, y2 ));
	double tau;
	mistl::Vector3f P = lin1.Intersection( lin2, tau);
	float depth=cam1.A().Dot(P);
    std::cout<< "P1: (" << P[0] << ", "<< P[1] << ", " << P[2] << ") \t"<< "\n";

    /*
     * test for back projection
     */
    mistl::Vector2f backProj_cam1;
    mistl::Vector2f backProj_cam2;
    mistl::Vector2f tmpPixel_cam1 = mistl::Vector2f(x1, y1);
    mistl::Vector2f tmpPixel_cam2 = mistl::Vector2f(x2, y2);
    cam1.Project(P, backProj_cam1);
    cam2.Project(P, backProj_cam2);
    cout << "backProjection cam1: " << backProj_cam1 << "\t error: " << (tmpPixel_cam1 - backProj_cam1).Magnitude() << "\n" ;
    cout << "backProjection cam2: " << backProj_cam2 << "\t error: " << (tmpPixel_cam2 - backProj_cam2).Magnitude() << "\n" ;


  if( argc>= 11) {
	  char* inputCoord1_x = argv[7];
	  char* inputCoord1_y = argv[8];
	  char* inputCoord2_x = argv[9];
	  char* inputCoord2_y = argv[10];

	  x1 = atoi(inputCoord1_x);
	  x2 = atoi(inputCoord2_x);
	  y1 = atoi(inputCoord1_y);
	  y2 = atoi(inputCoord2_y);
	  cout << "\nprojecting values (" << x1  << ", " << y1 << "); (" << x2 << ", " << y2 << ") ->";

	  mistl::Line3  lin3( cam1.C(), cam1.C() + cam1.LineOfSight( x1, y1 ));
	  mistl::Line3  lin4( cam2.C(), cam2.C() + cam2.LineOfSight( x2, y2 ));

	  mistl::Vector3f P2 = lin3.Intersection( lin4, tau);
	  float depth=cam1.A().Dot(P2);
	  std::cout<< "P2: (" << P2[0] << ", "<< P2[1] << ", " << P2[2] << ") \t" <<"\n";

	  tmpPixel_cam1 = mistl::Vector2f(x1, y1);
	  tmpPixel_cam2 = mistl::Vector2f(x2, y2);
	  cam1.Project(P2, backProj_cam1);
	  cam2.Project(P2, backProj_cam2);
	  cout << "backProjection cam1: " << backProj_cam1 << "\t error: " << (tmpPixel_cam1 - backProj_cam1).Magnitude() << "\n" ;
	  cout << "backProjection cam2: " << backProj_cam2 << "\t error: " << (tmpPixel_cam2 - backProj_cam2).Magnitude() << "\n" ;


	  cout << "\n\ndistance P1-P2: " << (P-P2).Magnitude() << "\n" ;
  }

}
