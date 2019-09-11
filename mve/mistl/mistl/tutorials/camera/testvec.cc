

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
// #include	"mistl/Camera.h"
#include	<stdlib.h>
#include <stdio.h>


int main(int argc, char** argv)
{
  printf("Hello world! My name is Mistl!\n");
   
  mistl::Vector3f	a,b,c;
  double	d=0;

 
  
  
//   mistl::Camera	cam;
//   std::cout<<"m1\n";
//   ReadCamera(cam,"cam_03.cahv");
//   std::cout<<"m2\n";
//   cam.Print();
//   
//   mistl::Vector2f	px;
//   float x,y;
//   c=mistl::Vector3f(1,2,3);
//   cam.Project(c,x,y);
//   std::cout<<c<<" -> "<<x<<" "<<y<<"\n";
//   
//   
  std::cout<<"m3\n";
  double delta=0.000000001;
  for(int i=0; i<1000000000; ++i) {
//       a[0] = i;
//       a[1] = i;
//       a[2] = i;
	  
	  	 a[0] = i;
	 a[1] = i;
         a[2] = i;
	 b[0] = i;
	 b[1] = i;
         b[2] = i;

  // DANIEL: commented out because error : dentifier "dot" is undefined
  //d += dot(a,b);

	  
//       c.x += delta;
//       cam.Project(c,x,y);
//       d+= x;
      
  }
  printf("result:%g\n",d);
}