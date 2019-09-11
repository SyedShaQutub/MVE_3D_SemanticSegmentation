
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


#ifndef __MISTL_VEC2_H__
#define __MISTL_VEC2_H__

#include <Core/FixedVector.h>
#include <assert.h>
#include <cmath>

#ifdef      USE_OPENCV
#include "opencv2/core/core_c.h"
#include "opencv2/core/core.hpp"
#endif

namespace mistl {



template<typename T> 
class Vector2: public FixedVectorC<T, 2>
{
public:
    Vector2()
    { 
        this->m_data[0] = 0.0;
        this->m_data[1] = 0.0;
    }
    // Default constructor, sets elements to zero

    Vector2(T x, T y)
    {
       this->m_data[0] = x;
       this->m_data[1] = y;
    }
    // Data constructor.

    Vector2(const FixedVectorC<T, 2>& v)
    : FixedVectorC<T, 2>(v)
    { }
    // Base constructor.

#ifdef      USE_OPENCV
      Vector2( const cv::Point2i &p) { 
        this->m_data[0] = p.x;
        this->m_data[1] = p.y;
      }
      Vector2( const cv::Point2f &p) { 
        this->m_data[0] = p.x;
        this->m_data[1] = p.y;
      }
      Vector2( const cv::Point2d &p) { 
        this->m_data[0] = p.x;
        this->m_data[1] = p.y;
      }
#endif  
	
public:
    // Access

    T &X()
    { return this->m_data[0]; }
    T X() const
    { return this->m_data[0]; }
    // Access first component

    T &Y()
    { return this->m_data[1]; }
    T Y() const
    { return this->m_data[1]; }
    // Access second component

   

public:
    // Vector operations

	T Length() {return this->Magnitude();}


    inline void Normalize(double nearlyzero = 1e-12)
    { 
       double len = this->Magnitude();
       if (len > nearlyzero)
       {
          this->m_data[0] /= len;
          this->m_data[1] /= len;
       }
    }
};

typedef mistl::Vector2<float> Vector2f;
typedef mistl::Vector2<double> Vector2d;

}

#endif