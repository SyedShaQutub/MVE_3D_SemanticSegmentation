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


#include 	"opencv2/opencv.hpp"
#include 	<opencv2/core/core.hpp>
#include 	<opencv2/highgui/highgui.hpp>
#include	"mistl/Vector3.h"
#include	"mistl/Camera.h"
#include	"mistl/Line3.h"
#include 	"mistl/KLTracker.h"
#include 	"mistl/MultiCamTracking.h"
#include 	<math.h>
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

	std::cout<<"reproduce_ui -c camera_file_1 camera_file_2 -i image_1 image_2 \n\n";
	exit(-1);
}


int clickX = -1;
int clickY = -1;
int clickX2 = -1;
int clickY2 = -1;

void resetValues() {
	clickX = -1;
	clickY = -1;
	clickX2 = -1;
	clickY2 = -1;

	cout << "reset values \n" << endl;
}


int reproduce3d(const mistl::Camera &cam1, const mistl::Camera &cam2, const mistl::Vector2f &P1, mistl::Vector2f const &P1sec,
		const mistl::Vector2f &P2, const mistl::Vector2f &P2sec) {

	float x1 = P1.X();
	float y1 = P1.Y();
	float x2 = P1sec.X();
	float y2 = P1sec.Y();

	cout << "projecting values (" << x1  << ", " << y1 << "); (" << x2 << ", " << y2 << ") -> ";


	mistl::Line3      lin1( cam1.C(), cam1.C() + cam1.LineOfSight( x1, y1 ));
	mistl::Line3      lin2( cam2.C(), cam2.C() + cam2.LineOfSight( x2, y2 ));
	double tau;
	mistl::Vector3f p1 = lin1.Intersection( lin2, tau);
	float depth=cam1.A().Dot(p1);
//	cam2.Info();
//	std::cout<< "\nlinA: " << lin1.A() << "\t"<< "\n";
//	std::cout<< "linB: " << lin1.B() << "\t"<< "\n";
//	std::cout<< "\nlinA: " << lin2.A() << "\t"<< "\n";
//	std::cout<< "linB: " << lin2.B() << "\t"<< "\n\n";
//	std::cout<< "P1: (" << p1[0] << ", "<< p1[1] << ", " << p1[2] << ") \t"<< "\n";



	/*
	 * test for back projection
	 */
	mistl::Vector2f backProj_cam1;
	mistl::Vector2f backProj_cam2;
	mistl::Vector2f tmpPixel_cam1 = mistl::Vector2f(x1, y1);
	mistl::Vector2f tmpPixel_cam2 = mistl::Vector2f(x2, y2);
	cam1.Project(p1, backProj_cam1);
	cam2.Project(p1, backProj_cam2);
	cout << "backProjection cam1: " << backProj_cam1 << "\t error: " << (tmpPixel_cam1 - backProj_cam1).Magnitude() << "\n" ;
	cout << "backProjection cam2: " << backProj_cam2 << "\t error: " << (tmpPixel_cam2 - backProj_cam2).Magnitude() << "\n" ;


	x1 = P2.X();
	y1 = P2.Y();
	x2 = P2sec.X();
	y2 = P2sec.Y();
	cout << "\nprojecting values (" << x1  << ", " << y1 << "); (" << x2 << ", " << y2 << ") ->";

	mistl::Line3  lin3( cam1.C(), cam1.C() + cam1.LineOfSight( x1, y1 ));
	mistl::Line3  lin4( cam2.C(), cam2.C() + cam2.LineOfSight( x2, y2 ));

	mistl::Vector3f p2 = lin3.Intersection( lin4, tau);
	depth=cam1.A().Dot(p2);
	std::cout<< "P2: (" << p2[0] << ", "<< p2[1] << ", " << p2[2] << ") \t" <<"\n";

	tmpPixel_cam1 = mistl::Vector2f(x1, y1);
	tmpPixel_cam2 = mistl::Vector2f(x2, y2);
	cam1.Project(p2, backProj_cam1);
	cam2.Project(p2, backProj_cam2);
	cout << "backProjection cam1: " << backProj_cam1 << "\t error: " << (tmpPixel_cam1 - backProj_cam1).Magnitude() << "\n" ;
	cout << "backProjection cam2: " << backProj_cam2 << "\t error: " << (tmpPixel_cam2 - backProj_cam2).Magnitude() << "\n" ;


	cout << "\n\ndistance P1-P2: " << (p1-p2).Magnitude() << "\n" ;

}


const cv::Scalar hiercolortable[] = {
    cv::Scalar(255,64,64),
    cv::Scalar(0,64,255),
    cv::Scalar(0,255,64),
    cv::Scalar(255,127,64),
    cv::Scalar(127,255,127)
};



void onMouse(int event, int x, int y, int flags, void* data) {

//	if (event == cv::EVENT_RBUTTONUP) {
//		cv::destroyAllWindows();
//		return;
//	}

	if (event != cv::EVENT_LBUTTONDOWN) {
		return;
	}

	cv::Point2f point = *((cv::Point2f*)data);
	point.x = x;
	point.y = y;
	cout << "Point: (" << x << ", " << y << ") \n";
	if(clickX >= 0 && clickY >= 0) {
		clickX2 = x;
		clickY2 = y;
	}
	else {
		clickX = x;
		clickY = y;
	}
}


int main(int argc, char** argv)
{

	if( /*(argc<4 && argv[3]=="-file") || */ (argc<5) ) {
		PrMsg();
		return -1;
	}

	cout<<"\ninteraktive 2D -> 3D Projection " << endl;


	char* camFile1;
	char* camFile2;
	char* camFile3;
	char* camFile4;
	mistl::Camera cam1;
	mistl::Camera cam2;
	mistl::Camera cam3;
	mistl::Camera cam4;
	bool useFourCamFiles = false;

	cv::Size winSize(1200, 900);
	mistl::Vector2f scale(1.0f, 1.0f);
	cv::Mat cImg1;
	cv::Mat cImg2;

	bool writeImage = true;

	//
	// parse comandline parameters
	//
	int aa=1;
	for(;;) {

		if(aa >= argc)
			break;

		if( *argv[aa]!='-') break;
		if(!strcmp(argv[aa],"-i")) {
			++aa; if(aa>=argc) PrMsg();
			cImg1 = cvLoadImage(argv[aa], CV_LOAD_IMAGE_UNCHANGED);
			++aa; if(aa>=argc) PrMsg();
			cImg2 = cvLoadImage(argv[aa], CV_LOAD_IMAGE_UNCHANGED);
			++aa;
		} else
		if(!strcmp(argv[aa],"-c")) {
			++aa; if(aa>=argc) PrMsg();
			camFile1 = argv[aa];
			++aa; if(aa>=argc) PrMsg();
			camFile2 = argv[aa];
			++aa;
		}  else
		if(!strcmp(argv[aa],"-c1")) {
			++aa; if(aa>=argc) PrMsg();
			camFile1 = argv[aa];
			++aa; if(aa>=argc) PrMsg();
			camFile2 = argv[aa];
			++aa;
		}  else
		if(!strcmp(argv[aa],"-c2")) {
			++aa; if(aa>=argc) PrMsg();
			camFile3 = argv[aa];
			++aa; if(aa>=argc) PrMsg();
			camFile4 = argv[aa];
			++aa;
			useFourCamFiles = true;
		}  else
		if(!strcmp(argv[aa],"-s")) {
			++aa; if(aa>=argc) PrMsg();
			winSize.width = atof(argv[aa]);
			++aa; if(aa>=argc) PrMsg();
			winSize.width = atof(argv[aa]);
			++aa;
		} else {
			PrMsg();
		}

	}


	//
	// read cameras
	//
	try {

		ReadCamera(cam1, camFile1);
		ReadCamera(cam2, camFile2);

		if(useFourCamFiles) {
			ReadCamera(cam3, camFile3);
			ReadCamera(cam4, camFile4);
		}
	}
	catch ( mistl::Error e) {
		std::cout<<"Error: can not read cameras - "<<e.GetMsg()<<"\n";
		PrMsg();
		return -1;
	}



	//
	// read the two given images
	//
	cv::Mat img1;
	cv::Mat img2;
    if(! cImg1.data )                              // Check for invalid input
    {
      cout << "Could not open or find the image" <<  endl ;
      return -1;
    }
    cv::cvtColor(cImg1, img1, CV_RGB2GRAY);

	if(! cImg2.data )                              // Check for invalid input
	{
	  cout << "Could not open or find the image" << endl ;
	  return -1;
	}
	cv::cvtColor(cImg2, img2, CV_RGB2GRAY);









	mistl::MultiCamTracking mtrack;
	vector<cv::Mat> img_ptr_list;
    vector<cv::Mat> Cimg_ptr_list;
    vector<char*> camlistParameters;

    mtrack.camlist.push_back(cam1);
    mtrack.camlist.push_back(cam2);
    img_ptr_list.push_back(img1);
	img_ptr_list.push_back(img2);
	Cimg_ptr_list.push_back(cImg1);
	Cimg_ptr_list.push_back(cImg2);

	try {
	    mtrack.InfraTracking( img_ptr_list);
	    mtrack.Info();

	    cout << "Mark " << mtrack.trackdata.plist.size() << " features" << endl;
	    scale.X() = (float)Cimg_ptr_list[0].size().width / (float)winSize.width;
	    scale.Y() = (float)Cimg_ptr_list[0].size().height / (float)winSize.height;
//	    cv::resize(Cimg_ptr_list[0], Cimg_ptr_list[0], winSize, cv::INTER_LANCZOS4);

	    for (unsigned int i = 0; i < mtrack.trackdata.plist.size(); i++) {
			int hier=mtrack.trackdata.metalist.at(i).hier;
			if(hier<0 || hier>4) {
			  // ouch!!
			  cout <<" - hier(archy) out of range (" <<hier<< ") !!!" << endl;
			  continue;
			}
			cv::Scalar col=hiercolortable[hier];
	//        cout << "M-h: " << hier << "  " << mtrack.trackdata.plist[i]<< " col: " << col << endl;
			cv::Point2f tmpPoint = mtrack.trackdata.plist[i];
//			tmpPoint.x = tmpPoint.x / scale.X();
//			tmpPoint.y = tmpPoint.y / scale.Y();
			cv::circle( Cimg_ptr_list[0], tmpPoint, 2, col, 1);
	    }

		for(unsigned  j=1; j<img_ptr_list.size(); ++j ) {
			float scaleX = (float)Cimg_ptr_list[j].size().width / (float)Cimg_ptr_list[0].size().width;
			float scaleY = (float)Cimg_ptr_list[j].size().height / (float)Cimg_ptr_list[0].size().height;

			mtrack.tracker.MarkMatches(  Cimg_ptr_list[0], mtrack.Track(0,0).plist, mtrack.Track(j,0).plist,
					mtrack.Track(j,0).status, scaleX, scaleY );
	    }

		if(writeImage) {
			cv::vector<int> compression_params;
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(100);
			cv::imwrite("output.jpg", Cimg_ptr_list[0], compression_params);
		}

		cv::resize(Cimg_ptr_list[0], Cimg_ptr_list[0], winSize, cv::INTER_LANCZOS4);
		unsigned jj=0;
		char name[80];
		sprintf(name,"window cam-%02d", jj+1) ;
		cv::namedWindow(name, 1);
		cv::moveWindow(name, 50, 50);
		cv::imshow(name, Cimg_ptr_list[0]);
		cv::Point2f data;
		cv::setMouseCallback(name, onMouse, &data);
//		cv::setMouseCallback()
	}
	catch ( mistl::Error e) {
	    cout << "Error: " << e.GetMsg() << endl;
	    return -1;
	}


	for(;;) {
		int pressedKey = cv::waitKey(0);
		if(pressedKey == 27) {
			cv::destroyAllWindows();
			return 0;
		}

		if(pressedKey == 32) {

			if(clickX<0 || clickY<0 || clickX2<0 || clickY2<0) {

				resetValues();
				continue;
			}
			else {
				/*
				 * start caluculation
				 */

				MISTL_ASSERT(mtrack.Track(0,0).plist.size() == mtrack.Track(1,0).plist.size(), "wrong amount of feature points");

				int searchWinSize = 13*2;
				mistl::Vector2f v1(0, 0);
				mistl::Vector2f v2(0, 0);
				mistl::Vector2f v3(0, 0);
				mistl::Vector2f v4(0, 0);

				cv::Point2f clickPoint1(((float)clickX)*scale.X(), ((float)clickY)*scale.Y());
				cv::Point2f clickPoint2(((float)clickX2)*scale.X(), ((float)clickY2)*scale.Y());
				vector<cv::Point2f> trackData1 = mtrack.Track(0,0).plist;
				vector<cv::Point2f> trackData2 = mtrack.Track(1,0).plist;
				bool p1Found = false, p2Found = false;
				for(int pCount=0; pCount<trackData1.size(); pCount++) {
					cv::Point2f tmpPoint = trackData1[pCount];

					if(tmpPoint.x >= clickPoint1.x-(searchWinSize/2) && tmpPoint.x <= clickPoint1.x+(searchWinSize/2) &&
							tmpPoint.y >= clickPoint1.y-(searchWinSize/2) && tmpPoint.y <= clickPoint1.y+(searchWinSize/2)) {

						// point found
						v1 = mistl::Vector2f(tmpPoint.x, tmpPoint.y);
						v2 = mistl::Vector2f(trackData2[pCount].x, trackData2[pCount].y);

						p1Found = true;
					}
				}

				for(int pCount=0; pCount<trackData1.size(); pCount++) {
					cv::Point2f tmpPoint = trackData1[pCount];

					if(tmpPoint.x >= clickPoint2.x-(searchWinSize/2) && tmpPoint.x <= clickPoint2.x+(searchWinSize/2) &&
							tmpPoint.y >= clickPoint2.y-(searchWinSize/2) && tmpPoint.y <= clickPoint2.y+(searchWinSize/2)) {

						// point found
						v3 = mistl::Vector2f(tmpPoint.x, tmpPoint.y);
						v4 = mistl::Vector2f(trackData2[pCount].x, trackData2[pCount].y);

						p2Found = true;
					}
				}

				if(p1Found && p2Found) {
					reproduce3d(cam1, cam2, v1, v2, v3, v4);
				}

				resetValues();
			}
		}

		if(pressedKey == 114) {

			resetValues();
		}

	}


	return 0;


}
