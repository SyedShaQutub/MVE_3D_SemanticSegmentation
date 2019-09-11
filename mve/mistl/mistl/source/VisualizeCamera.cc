

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
#include	"mistl/TriangularMesh.h"
#include        "mistl/Camera.h"



namespace mistl {

  
mistl::TriangularMesh *
CreateBox( const mistl::Vector3f &Center, float size )
{
  mistl::TriangularMesh        *t1 = new mistl::TriangularMesh();

  mistl::Vector3f  v1;

  v1 = Center-mistl::Vector3f(size*0.5f,size*0.5f,size*0.5f) ;

  t1->Add(   v1  );
  t1->Add( v1+mistl::Vector3f(size,0.0f,0.0f)  );
  t1->Add( v1+mistl::Vector3f(size,size,0.0f)  );
  t1->Add( v1+mistl::Vector3f(0.0f,size,0.0f)  );

  v1 = Center +mistl::Vector3f(-size*0.5f,-size*0.5f,size*0.5f) ;
  t1->Add(   v1  );
  t1->Add( v1+mistl::Vector3f(size,0.0f,0.0f)  );
  t1->Add( v1+mistl::Vector3f(size,size,0.0f)  );
  t1->Add( v1+mistl::Vector3f(0.0f,size,0.0f)  );
 

  t1->Add( 0, 2, 1); //bottom
  t1->Add( 2, 3, 1);

  t1->Add( 6, 4, 5); //top
  t1->Add( 6, 5, 7);

  t1->Add( 6, 2, 3); // back
  t1->Add( 3, 7, 6);

  t1->Add( 4, 0, 1); //front
  t1->Add( 5, 4, 1);

  // ri
  t1->Add( 5, 1, 2);
  t1->Add( 2, 6, 5);
  
  t1->Add( 0, 4, 3);
  t1->Add( 3, 4, 7);
  

  

  return t1;
}

mistl::TriangularMesh *
VisualizeCamera( const mistl::Camera &cam, float size )
{
  mistl::TriangularMesh        *t1 = new mistl::TriangularMesh();

  mistl::Vector3f  h1,v1;

  v1 = cam.H0()*(size* -0.5f) ;

  t1->Add( cam.C() + v1  );
  t1->Add( cam.C() + v1+cam.H0()*size );
  t1->Add( cam.C() + v1+cam.A()*size  );
  t1->Add( cam.C() + v1+cam.A()*size+cam.H0()*size  );

  h1 = cam.V0() * -size;
  t1->Add( cam.C() + h1+v1  );
  t1->Add( cam.C() + h1+v1+cam.H0()*size  );
  t1->Add( cam.C() + h1+v1+cam.A()*size  );
  t1->Add( cam.C() + h1+v1+cam.A()*size+cam.H0()*size  );

  t1->Add( 0, 2, 1);
  t1->Add( 2, 3, 1);

  t1->Add( 6, 4, 5);
  t1->Add( 6, 5, 7);

  t1->Add( 6, 2, 0);
  t1->Add( 4, 6, 0);

  // back
  t1->Add( 4, 0, 1);
  t1->Add( 5, 4, 1);

  // ri
  t1->Add( 5, 1, 3);
  t1->Add( 7, 5, 3);

  // pryramid
  t1->Add( cam.C() + v1+cam.A()*size + cam.H0()*size*0.5f+cam.V0() * -size*0.5f  );
  t1->Add( cam.C() + v1+cam.A()*2.0f*size );
  t1->Add( cam.C() + v1+cam.A()*2.0f*size+ cam.H0()*size );
  t1->Add( cam.C() + v1+h1+cam.A()*2.0f*size );
  t1->Add( cam.C() + v1+h1+cam.A()*2.0f*size+ cam.H0()*size );

  t1->Add( 8, 9, 10);
  t1->Add( 8, 10, 12);
  t1->Add( 8, 12, 11);
  t1->Add( 8, 11, 9);

  t1->Add( 11, 10, 9);
  t1->Add( 11, 12, 10);

  
  
  return t1;
}

}
