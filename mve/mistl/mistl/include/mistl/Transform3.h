
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


#ifndef __MISTL_TRANS3_H__
#define __MISTL_TRANS3_H__

//#include	"mistl/Transform3_embree.h"

#include	"mistl/Vector3.h"
#include	"mistl/Matrix3x3.h"

namespace mistl {


/*!
  \class Transform3f Transform3.h
  \brief 3D transformation class (rotation+translation)
*/

template<typename Ty> 
class Transform3
{
	
	public:
		mistl::Matrix3x3f	rot;	//!< Rotation matrix
		Vector3<Ty>	t;	//!< translation 
		
// 		mistl::Vector3<Ty> operator*(const mistl::Vector3<Ty> &v) const {
//                   return Trans(v);
//                 }
		//! apply transformation
		mistl::Vector3<Ty>	Trans( const mistl::Vector3<Ty> v)  {
			mistl::Vector3<Ty> x;
			x=rot*v;
			return x+t;
		}
		
		mistl::Transform3<Ty>& operator=(const mistl::Transform3<Ty>& other) { Copy(other); return *this;}
		void Copy( const Transform3<Ty> &other) { rot=other.rot; t=other.t;}
		
		mistl::Vector3<Ty> RotationVx() const { 
			return mistl::Vector3<Ty>( this->rot[0][0], this->rot[0][1],this->rot[0][2] );
		}
		mistl::Vector3<Ty> RotationVy() const { 
			return mistl::Vector3<Ty>( this->rot[1][0], this->rot[1][1],this->rot[1][2] );
		}
		mistl::Vector3<Ty> RotationVz() const { 
			return mistl::Vector3<Ty>( this->rot[2][0], this->rot[2][1],this->rot[2][2] );
		}
		void	SetRotationVx(	mistl::Vector3<Ty> v ) {
			this->rot[0][0]=v.X();
			this->rot[0][1]=v.Y();
			this->rot[0][2]=v.Z();
		}
		void	SetRotationVy(	mistl::Vector3<Ty> v ) {
			this->rot[1][0]=v.X();
			this->rot[1][1]=v.Y();
			this->rot[1][2]=v.Z();
		}
		void	SetRotationVz(	mistl::Vector3<Ty> v ) {
			this->rot[2][0]=v.X();
			this->rot[2][1]=v.Y();
			this->rot[2][2]=v.Z();
		}
		
		//! Set rotation around axis
		void SetRotation(const mistl::Vector3<Ty> &axis, double angle) {
                 if (axis.Length() > .0f)
                    {
                      mistl::Vector3<Ty> axisdir=axis.Normalized();

                      double s=sin(angle);
                      double c=cos(angle);
                      double k= 1.0 -c;
                      double symm,skew;

                      this->rot[0][0] = k*axisdir[0]*axisdir[0] + c;
                      this->rot[1][1] = k*axisdir[1]*axisdir[1] + c;
                      this->rot[2][2] = k*axisdir[2]*axisdir[2] + c;

                      symm = k*axisdir[1]*axisdir[0];
                      skew = s*axisdir[2];
                      this->rot[0][1] = symm - skew;
                      this->rot[1][0] = symm + skew;

                      symm = k*axisdir[2]*axisdir[0];
                      skew = s*axisdir[1];
                      this->rot[0][2] = symm + skew;
                      this->rot[2][0] = symm - skew;

                      symm = k*axisdir[2]*axisdir[1];
                      skew = s*axisdir[0];
                      this->rot[1][2] = symm - skew;
                      this->rot[2][1] = symm + skew;
                    }
                }

		
		// Set transform to neutral (rot=identity, trans=nil)
		void Clear() {
                  rot=rot.Identity();
                  t=mistl::Vector3<Ty>(0,0,0);
                }
                
		//! Return inverse transformation
		mistl::Transform3<Ty>   Inverse() const {
                  mistl::Transform3<Ty> t;
                  t.rot = (*this).rot.T();
                  t.t =  (t.rot * (*this).t) * -1.0;
                  return t;
                }
};


typedef Transform3<float> Transform3f;
typedef Transform3<double> Transform3d;

}


#endif