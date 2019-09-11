

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
#include        "mistl/WriteOBJ.h"
#include	<stdlib.h>
#include <stdio.h>
#include        "mistl/ReadCoordinateFile.h"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include        "mistl/YAML_IO.h"
#include        "mistl/DR_CameraArray.h"
#include        "mistl/DRCalibrate.h"
#include        "mistl/Error.h"

#ifdef _MSC_VER
  #pragma warning( disable : 4996 ) // '*' was declared deprecated
#endif

void
PrMsg() {
    std::cout<<"usage: DRcamera [options] camera-file\n";
    std::cout<<"\t-vis outfile\n";
    std::cout<<"\t-p (file) read-in 3d-points (use with -ov and -wov to plot points into image)\n";
    std::cout<<"\t-o printf style pattern for output-camera (e.g. camera%02d.ycam)\n";
    std::cout<<"\t-ov (file) overlay image to plot points \n";
    std::cout<<"\t-wov (file)  write image to plot points \n";
    std::cout<<"\t-seq (int)  sequence id to plot points \n";
    std::cout<<"\t-otrans (file)  object transform to plot points \n";
    std::cout<<"\t-plot2d\tplot 2D image coordinates (use with -ov and -wov to plot points into image)\n";
  exit(-1);
}


int main(int argc, char** argv)
{
  printf("Hello world! My name is Mistl!\n");
   
  mistl::Vector3f	P,c;
  double	d=0;
  float len=10.0;

 
  
  int aa=1;
  char *visfn= (char *)0;
  char *ofn= (char *)0;
  char *ovfn= (char *)0;
  char *pfn= (char *)0;
  char *wovfn= (char *)0;
  char *otransfn= (char *)0;
  int seq=0;
  bool  pr_imagefeatures=false;
  bool drFormat = false;

  if(aa>=argc) {PrMsg(); return -1;}
  for(;aa<argc;) {
    std::cout<<"Test (aa="<<aa<<"):"<<argv[aa]<<"\n";
    
    if( argv[aa][0]!='-') break;
    if(!strcmp(argv[aa],"-seq")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      seq= atoi(argv[aa]);
      ++aa;
    }
    if(!strcmp(argv[aa],"-otrans")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      otransfn=argv[aa];
      ++aa;
    }
    if(!strcmp(argv[aa],"-vis")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      visfn=argv[aa];
      ++aa;
    }
    if(!strcmp(argv[aa],"-convdr")) {
    	drFormat = true;
	  ++aa;
	}
    if(!strcmp(argv[aa],"-wov")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      wovfn=argv[aa];
      ++aa;
    }
    if(!strcmp(argv[aa],"-ov")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      ovfn=argv[aa];
      ++aa;
    }
    if(!strcmp(argv[aa],"-o")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      ofn=argv[aa];
      ++aa;
    }
    if(!strcmp(argv[aa],"-p")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      pfn=argv[aa];
      ++aa;
    }
    if(!strcmp(argv[aa],"-plot2d")) {
      ++aa;
      if(aa>=argc){PrMsg(); return -1;}
      pr_imagefeatures=true;
    }
  }
  if(aa>=argc) {PrMsg(); return -1;}
  
  mistl::DR_CameraArray	camarry;
  std::cout<<"m1\n";
  try {
	  
          mistl::ReadCameraYAML(camarry,argv[aa]);
	  
  } 
  catch ( mistl::Error e) {
      std::cout<<"Error: "<<e.GetMsg()<<"\n";
      return -1;
  }
  

/*  

  std::cout<<"m2\n";
  cam.Info();std::cout<<"\n";
  
  
  std::cout<<std::endl;
  std::cout<<"Pose:"<< cam.GetPose().rot << cam.GetPose().t <<std::endl<<std::endl;
  
  
  
  mistl::Matrix3x3f     mat = cam.GetPose().rot.Inverse() ;
  mistl::Vector3f       tinv = mat * (cam.C() * -1.0);
  std::cout<<"Pose-1:"<< mat << tinv <<std::endl<<std::endl;
  
  mat = cam.GetPose().rot;
  tinv = (mat * cam.C() );
  tinv = tinv  * -1.0;
  std::cout<<"T-inv 2:"<< tinv <<std::endl<<std::endl;
  
  
  mat = cam.GetPose().rot.T();
  tinv = tinv  * -1.0;
  tinv = mat * tinv;
  std::cout<<"T-inv -inv:"<< tinv <<std::endl<<std::endl;*/
  

  for(unsigned i=0; i<camarry.number_of_cameras; ++i) {
    camarry.camera[i].Info();
    std::cout<<"\n";
    
    mistl::Matrix3x3f mat,mat2;
    mat = camarry.camera[i].GetPose().rot;
    
    std::cout<<"Rot:\n"<<mat<<"\n\n";
     Mul( mat, mat.T(), mat2);
    std::cout<<"r x rT:\n"<<mat2<<"\n\n";
    
  }

  mistl::Camera cam = camarry.camera[camarry.refCam];
  
  mistl::Vector2f	p1;
//   mistl::Vector2f       px(100.0f,100.0f);
  mistl::Vector2f       px(0.0f,0.0f);
  float x,y;
  
  P= cam.C() + cam.LineOfSight(px)*len;
  cam.Project(P,p1);
  std::cout<<px<<" >> "<< P<<" -> "<<p1<<  " distance:"<< (p1-px).Magnitude()<<"\n";
  
  P= mistl::Vector3f(1.0,1.0,1.0 );
  cam.Project(P,p1);
  std::cout<<px<<"Project "<< P<<" -> "<<p1 <<"\n";
  
  ////
  const float r=100.0f;
  cam.Distort( r, 0 , x,y );
  float s = sqrt( x*x + y*y);
  float invs = cam.SolveInvEq( s );
  
  
  std::cout << "Test distortion: r:"<<r<< " dist:"<<s << "  inv:"<<invs ;
  std::cout << "\n";
  float ux,uy;
  cam.UnDistort(x,y,ux,uy);
  std::cout << "\t"<<r<<",0 -> "<< x<<","<<y<< " un-> "<<ux<<","<<uy<<"\n";

  
//   std::cout<<"m3\n";
  float delta=0.000000001f;
  for(int i=0; i<10000000; ++i) {
	  
      c[0] += delta;
      cam.Project(c,x,y);
      d+= x;
      
  }
  printf("result:%g\n",d);
  
  // write an obj file of the camera
  if(visfn) {
    mistl::TriangularMesh *tp = VisualizeCamera( cam, 0.05f );
    tp->Info();
    WriteOBJ( *tp, visfn);
  }
  
  bool  pr_label=true;
  
  
  if(pfn) {
    std::vector<mistl::CoordinateListEntry> clist ;
    ReadCoordinateFile( pfn, clist ) ;
    
    std::cout<<"Read "<<clist.size()<<" points\n";
    
    cv::Mat img;
    
    if(ovfn) {
      img = cvLoadImage( ovfn, CV_LOAD_IMAGE_UNCHANGED);
      std::cout<<"Read image:"<<ovfn<< " size:"<<img.size()<<"\n";
    }
    else   img.create( cam.GetNy(),cam.GetNx(),CV_8UC3 );
    
    mistl::Transform3f tr;
    tr.Clear();
    if(otransfn)
     mistl::ReadTransform3YAML( tr, otransfn );
    std::cout<<"R:"<<tr.rot<<" t:"<<tr.t<<"\n";
    
    for(unsigned i=0; i<clist.size(); ++i) {
      if(clist[i].id != seq) continue;
      
      cv::Point2f cornerB ;
      P[0]=clist[i].P[0];
      P[1]=clist[i].P[1];
      P[2]=clist[i].P[2];

      P=tr.Trans(P);
      float x,y,z;
      cam.Project(P,x,y,z);
      p1[0]=x;p1[1]=y;
      
        cornerB.x =p1[0];
        cornerB.y =p1[1];

        if(z<0.0) {
          cv::circle( img, cornerB, 2, cv::Scalar(255,0,0), 1);
          std::cout<<"point "<<i<<" ("<<P<<") is behind camera\n";
        } else {
          cv::circle( img, cornerB, 2, cv::Scalar(0,255,0), 1);
        }
        
        if(pr_label) {
          char w[40];
          sprintf(w,"%ld", clist[i].pointid );
//         std::cout<<"Id:"<<w<<"\n";
          cv::putText( img, w, cornerB, CV_FONT_HERSHEY_COMPLEX, 0.5,
           cv::Scalar(255,255,255));
        }
        
        // plot image coordinates
        if(pr_imagefeatures) {
          cornerB.x =clist[i].ip[0];
          cornerB.y =clist[i].ip[1];;
          cv::circle( img, cornerB, 2, cv::Scalar(255,0,255), 1);
          if(pr_label) {
            char w[40];
            sprintf(w,"%ld", clist[i].pointid );
//         std::cout<<"Id:"<<w<<"\n";
            cv::putText( img, w, cornerB, CV_FONT_HERSHEY_COMPLEX, 0.5,
            cv::Scalar(255,128,255));
          }
        }
    }
    
    
    if(wovfn) {
      if(!strcmp(wovfn,"-")) {
        cv::namedWindow( "Display window" ); //, WINDOW_AUTOSIZE );// Create a window for display.
        cv::imshow( "Display window", img );                   // Show our image inside it.
  
        cv::waitKey(0); 
      } else {
        std::cout<<"Write image:"<< wovfn<<"\n";
        cv::imwrite( wovfn , img );
      }
    }
  }

  
  if(ofn) {

	// output DR format
	mistl::DRCalibrate drCalibration;
	drCalibration.SetInitialCameraParameterList(camarry);



	std::cout<<"\n\nCAMERAS: "<<std::endl;
	std::vector<mistl::Camera> cams;
	drCalibration.GetCameras(cams);
	cams[0].Info(); std::cout<<"\n: "<<std::endl;
	cams[1].Info(); std::cout<<"\n: "<<std::endl;
	cams[2].Info(); std::cout<<"\n: "<<std::endl;

	mistl::Vector3f c0_before = cams[0].C();
	mistl::Vector3f c1_before = cams[1].C();
	mistl::Vector3f c2_before = cams[2].C();
	mistl::Vector3f a0_before = cams[0].A();
	mistl::Vector3f a1_before = cams[1].A();
	mistl::Vector3f a2_before = cams[2].A();

	if(!drFormat) {
		drCalibration.NormScaleCameras();
	}

	std::cout<<"\n\n"<<std::endl;
	std::vector<mistl::Camera> cams2;
	drCalibration.GetCameras(cams2);
	cams2[0].Info(); std::cout<<"\n: "<<std::endl;
	cams2[1].Info(); std::cout<<"\n: "<<std::endl;
	cams2[2].Info(); std::cout<<"\n: "<<std::endl;

	mistl::Vector3f c0_after = cams2[0].C();
	mistl::Vector3f c1_after = cams2[1].C();
	mistl::Vector3f c2_after = cams2[2].C();
	mistl::Vector3f a0_after = cams2[0].A();
	mistl::Vector3f a1_after = cams2[1].A();
	mistl::Vector3f a2_after = cams2[2].A();


	// some assertions to check if the applied normnalization was correct
	// instead checking == we use >= and <= with some small eps value due to float computation errors
	float eps_val = 0.00001f;
	// relative position of cameras should not change
	MISTL_ASSERT((c0_before-c1_before).Magnitude() >= ( (c0_after-c1_after).Magnitude() - eps_val), "normalization failed" );
	MISTL_ASSERT((c0_before-c1_before).Magnitude() <= ( (c0_after-c1_after).Magnitude() + eps_val), "normalization failed" );
	MISTL_ASSERT((c0_before-c2_before).Magnitude() >= ( (c0_after-c2_after).Magnitude() - eps_val), "normalization failed" );
	MISTL_ASSERT((c0_before-c2_before).Magnitude() <= ( (c0_after-c2_after).Magnitude() + eps_val), "normalization failed" );
	MISTL_ASSERT((c1_before-c2_before).Magnitude() >= ( (c1_after-c2_after).Magnitude() - eps_val), "normalization failed" );
	MISTL_ASSERT((c1_before-c2_before).Magnitude() <= ( (c1_after-c2_after).Magnitude() + eps_val), "normalization failed" );

	// cameras should look in the same direction as ref camera
	a0_before = a0_before.Normalized();
	a1_before = a1_before.Normalized();
	a2_before = a2_before.Normalized();
	a0_after = a0_after.Normalized();
	a1_after = a1_after.Normalized();
	a2_after = a2_after.Normalized();
	MISTL_ASSERT( a0_before.Dot(a2_before) >= 0, "normalization failed" );	// before cams look in same direction
	MISTL_ASSERT( a1_before.Dot(a2_before) >= 0, "normalization failed" );
	MISTL_ASSERT( a0_after.Dot(a2_after) >= 0, "normalization failed" );	// after cams look in same direction
	MISTL_ASSERT( a1_after.Dot(a2_after) >= 0, "normalization failed" );




//	MISTL_ASSERT((c0_before-c1_before).Magnitude() == ( (c0_after-c1_after).Magnitude() + 5.0f), "normalization failed" );


//	std::vector<mistl::Camera> normCams;
	drCalibration.GetInitialCameras(camarry);
//	mistl::DR_CameraArray norm_camarry;

//	for(unsigned i=0; i<normCams.size(); ++i) {
//	  norm_camarry.camera[i] = normCams[i];

//	  char fn[80];
//	  sprintf(fn,ofn,i);
//	  try {
//		mistl::WriteCamera( camarry.camera[i], fn );
//		mistl::WriteCameraYAML(norm_camarry, fn);
//	  }
//	  catch ( mistl::Error e) {
//		std::cout<<"Error: "<<e.GetMsg()<<"\n";
//		return -1;
//	  }
//	}


	char fn[80];
	sprintf(fn,ofn);
//	std::cout << "writing cam: " << fn << std::endl;

	if(drFormat) {
		for(unsigned i=0; i<camarry.number_of_cameras; ++i) {
		  camarry.camera[i].Info();
		  std::cout<<"\n";
		  char fn[80];
		  sprintf(fn,ofn,i);
		  try {
			mistl::WriteCamera( camarry.camera[i], fn );
		  }
		  catch ( mistl::Error e) {
			std::cout<<"Error: "<<e.GetMsg()<<"\n";
			return -1;
		  }
		}
	}
	else {
		mistl::WriteCameraYAML(camarry, fn);
	}

  }
}
