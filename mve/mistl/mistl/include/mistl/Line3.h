#ifndef		_Line3_incl
#  define	_Line3_incl


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
//	based on some code from: Oliver Grau, MAY-1994
//

#include        <vector>
#include        "Vector3.h"


namespace mistl {

/**
	\class  Line3 Line3.h
	\brief Line3 class. 
	Creates a line in 3D space given by one point on the line, \em A, and a vector for the direction, \em B, 
	satisfying the equation <em> X = A() + r * B() </em>.
	\author Oliver Grau
	\version 1.1
*/


class	Line3  {
	public:
        /** Create a line. Note that until parameters are set otherwise
		  the default after creation is: <em> X = (0,0,0)T + r* (0,0,1)T </em> */
		Line3( );

		/** Create a line and set it to the straight line that runs through the given points. */
		Line3( const mistl::Vector3f &p1, const mistl::Vector3f &p2);

		//! Copy constructor
		Line3( const Line3 &istr );

		/** Destructor */
		~Line3( );

		// access methods

		/** Returns point \em A from the parametric form of the
		  line:	<em> X = A() + r * B() </em>. */
		mistl::Vector3f  A() const { return a; }

		/** Returns vector \em B from the parametric form of the
		  line:	<em> X = A() + r * B() </em>. */
		mistl::Vector3f  B() const { return b; }

		/** Returns the point on the line perpendicular from p. */
		mistl::Vector3f FootPointofPerpendicular( const mistl::Vector3f & p );

		// update methods

		/** Set line equal to l. */
		Line3& operator=(const Line3 &l );

		/** Set line so that it runs through p1 and p2. */
		int Set( const mistl::Vector3f &p1, const mistl::Vector3f &p2);
		
		/** Set line parameter A+B  */
		int SetAB( const mistl::Vector3f &A, const mistl::Vector3f &B);

		/** Compute the point of intersection of two lines. Returns point between the closest points on
			the lines if these do not intersect at all. If tau is given then the distance of the
			intersecting point to the first start point of this line is returned.
		*/
		mistl::Vector3f Intersection( const Line3 &l, double &tau = *((double *)0) );

                

	private:
		/** Point \em A in the equation <em> X = A() + r * B() </em> */
		mistl::Vector3f	a;
		/** Vector \em B in the equation <em> X = A() + r * B() </em> */
		mistl::Vector3f	b;

};


  
}

#endif
