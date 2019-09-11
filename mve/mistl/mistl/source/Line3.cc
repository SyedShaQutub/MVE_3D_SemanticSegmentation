


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

//
//      
//
//      based on some code from: Oliver Grau, MAY-1994
//


#include        "mistl/Error.h"
#include	"mistl/Vector3.h"
#include	"mistl/Line3.h"

namespace mistl {

Line3 :: Line3(  )
{
	a = mistl::Vector3f(0,0,0);
	b = mistl::Vector3f(0,0,1);
}


Line3 :: Line3( const mistl::Vector3f &p1, const mistl::Vector3f &p2 )
{
	Set(p1,p2);
}


Line3 :: Line3( const mistl::Line3 &pl )
{
	a = pl.A();
	b = pl.B();
}

Line3 :: ~Line3(  )
{
}

int	Line3 :: Set( const mistl::Vector3f &p1, const mistl::Vector3f &p2 )
{
	MISTL_ASSERT(  ((p1-p2).Magnitude() > 1e-38),"mistl::Line3 :: Set(p1,p2) : parameters give no regular line\n");

	a = p1;
	b = p2-p1;
	return 1;
}

// ted
// android compiler doesn't like const mistl::Vector3f &_
// Set line parameter A+B
#if 0
int Line3::SetAB( const mistl::Vector3f &_A, const mistl::Vector3f &_B)
{
	
	//MISTL_ASSERT( _B.Magnitude() > 1e-38, "mistl::Line3 :: SetAB(A,B) : parameters give no regular line\n");
	//a=_A ;
	//b=_B ;
	return 1 ;
}
#endif



mistl::Line3& Line3 :: operator=(const mistl::Line3 &pl )
{
	a = pl.A();
	b = pl.B();
	return *this;
}


mistl::Vector3f Line3 :: FootPointofPerpendicular( const mistl::Vector3f & p )
{
	float	t0,b2;

	t0 = (p-A()).Dot(B());
	b2 = B().Dot(B());
	MISTL_ASSERT(!(b2>1e-38) ,"mistl::Line3 :: FootPointofPerpendicular : Can't calculate\n");
          
        t0 = t0 / b2;
	return A()+B()*t0;
}


mistl::Vector3f
Line3 :: Intersection( const mistl::Line3 &l, double &tauret )
{
	double	*tp = &tauret;


  float tau,gamma;
  mistl::Vector3f	l1p,l1r;
  mistl::Vector3f	l2p,l2r;
  mistl::Vector3f	intersect;

  l1p = A();
  l1r = B().Normalized();
  l2p = l.A();
  l2r = (l.B()).Normalized();

  /*
  cout << "l1p: "<< l1p <<"\n";
  cout << "l1r: "<< l1r <<"\n";
  cout << "l2p: "<< l2p <<"\n";
  cout << "l2r: "<< l2r <<"\n";
  */

  tau   = (l2p.Dot(l2r)) * (l1r.Dot(l2r)); //product(line2->punkt,line2->richtung)*
          			//product(line1->richtung,line2->richtung);

  tau += (l2r.Dot(l2r)) * (l1p.Dot(l1r) - l2p.Dot(l1r));

  tau -= (l1p.Dot(l2r)) * (l2r.Dot(l1r));

  tau /= ((l1r.Dot(l2r)) * (l1r.Dot(l2r)) - (l1r.Dot(l1r)) * (l2r.Dot(l2r)));

  gamma = tau* (l1r.Dot(l2r)) + l1p.Dot(l2r) - l2p.Dot(l2r);

  gamma/= l2r.Dot(l2r);

  intersect = ( (l1p+l2p) + (l1r*tau) + (l2r*gamma)) * 0.5f;


	if(tp) *tp = tau;
	return intersect;
}

}
