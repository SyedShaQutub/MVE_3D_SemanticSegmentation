
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


#ifndef __MISTL_VEC3_H__
#define __MISTL_VEC3_H__

#include	"mistl/Error.h"


#include <Core/FixedVector.h>
#include <assert.h>
#include <cmath>

#ifdef      USE_OPENCV
// #include "opencv2/opencv.hpp"
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"

#endif

namespace mistl {

const double nearlyzero = 1e-12;

#ifdef _MSC_VER
#pragma warning( disable : 4244 ) // conversion from 'const double' to 'float', possible loss of data
#endif

template<typename Ty> 
class Vector3: public FixedVectorC<Ty, 3>
{
public:
    Vector3 ()
    { 
        this->m_data[0] = 0.0;
        this->m_data[1] = 0.0;
        this->m_data[2] = 0.0;
    }
    // Default constructor, sets elements to zero

    Vector3 (Ty x, Ty y, Ty z)
    {
       this->m_data[0] = x;
       this->m_data[1] = y;
       this->m_data[2] = z;
    }
    // Data constructor.

    Vector3(const FixedVectorC<Ty, 3>& v)
    : FixedVectorC<Ty, 3>(v)
    { }
    // Base constructor.

#ifdef      USE_OPENCV
      Vector3( const cv::Mat &cvmat ) {
        MISTL_ASSERT( cvmat.type() == CV_64F, "Vector3 converting cv::Mat has wrong type");
        if( cvmat.rows == 3 && cvmat.cols == 1) 
          for(int i=0; i<3; ++i) {
            this->m_data[i] =cvmat.at<double>(i,0);
//             std::cout<<" this->m_data["<<i<<"]="<<cvmat.at<double>(i,0)<<"\n";
          }
        else if( cvmat.rows == 1 && cvmat.cols == 3) 
          for(int i=0; i<3; ++i) {
//             std::cout<<" this->m_data["<<i<<"]="<<cvmat.at<double>(0,i)<<"\n";
          this->m_data[i] = cvmat.at<double>(0, i);
          }
        else
          MISTL_ASSERT( false, "Vector3 converting cv::Mat has wrong size");
      }
      Vector3( const cv::Point3i &p) { 
        this->m_data[0] = p.x;
        this->m_data[1] = p.y;
        this->m_data[2] = p.z; }
      Vector3( const cv::Point3f &p) { 
        this->m_data[0] = p.x;
        this->m_data[1] = p.y;
        this->m_data[2] = p.z; }
      Vector3( const cv::Point3d &p) { 
        this->m_data[0] = p.x;
        this->m_data[1] = p.y;
        this->m_data[2] = p.z; }
      cv::Mat  cvMat() const {
	cv::Mat	cvmat(1,3,CV_64F);
	for(int i=0; i<3; ++i)
	  cvmat.at<double>(0,i) = this->m_data[i];
	return cvmat;
      }
#endif	
public:
    // Access

    Ty &X()
    { return this->m_data[0]; }
    Ty X() const
    { return this->m_data[0]; }
    // Access first component

    Ty &Y()
    { return this->m_data[1]; }
    Ty Y() const
    { return this->m_data[1]; }
    // Access second component

    Ty &Z()
    { return this->m_data[2]; }
    Ty Z() const
    { return this->m_data[2]; }
    // Access third component

public:
    // Vector operations

	Ty Length() const {return this->Magnitude();}
	
    //!     Return the cross product of this vector and the 'v'.
    inline Vector3<Ty> Cross(const Vector3<Ty> & v)  const
    {
        return Vector3<Ty>( this->m_data[1] * v[2] - this->m_data[2] * v[1] , -(this->m_data[0] * v[2] - this->m_data[2] * v[0]), this->m_data[0] * v[1] - this->m_data[1] * v[0] );
    }
	

    Vector3<Ty> Normalized() const { 
      float len = this->Magnitude();
      if (len > nearlyzero) len= 1.0f/len;
      else len=1.0f;
      
      return  Vector3<Ty>(this->m_data[0] * len,
          this->m_data[1] * len,
          this->m_data[2] * len);
    }
    void Normalize() { 
       Ty len = this->Magnitude();
       if (len > nearlyzero)
       {
          this->m_data[0] /= len;
          this->m_data[1] /= len;
          this->m_data[2] /= len;
       }
    }
};

typedef mistl::Vector3<float> Vector3f;
typedef mistl::Vector3<double> Vector3d;
typedef mistl::Vector3<int> Vector3i;
typedef mistl::Vector3<unsigned> Vector3u;

Vector3f Cross(const Vector3f & a, const Vector3f & v)  ;
//     {
//         return Vector3f( a.Y() * v[2] - a.Z() * v[1] , -(a.X() * v[2] - a.Z() * v[0]), a.X() * v[1] - a.Y() * v[0] );
//     }
}

#endif