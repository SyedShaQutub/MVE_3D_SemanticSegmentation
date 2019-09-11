

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

// #include	"Space/Vector3.h"

#include	"mistl/Vector3.h"
#include	<stdlib.h>
#include <stdio.h>


mistl::Vector3f	MyFunc( float a )
{
	return mistl::Vector3f(a,a,a);
}


int main(int argc, char** argv)
{
  printf("Hello world! My name is Mistl!\n");
   
  mistl::Vector3f	a,b,c;
// 	Vector3dC	a,b,c;
  double	d=0;

	b=mistl::Vector3f(0,0,0);
	c=b - MyFunc(2.2);

  std::cout<<"m3\n";
  double delta=0.000000001;
  for(int i=0; i<1000000000; ++i) {
	  
	a[0] = i;
	 a[1] = i;
         a[2] = i;
	 b[0] = i;
	 b[1] = i;
         b[2] = i;
	 d += a.Dot(b);
	 
      
  }
  printf("result:%g\n",d);
}