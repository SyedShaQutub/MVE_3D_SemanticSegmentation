
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


#ifndef __MISTL_VEC4_H__
#define __MISTL_VEC4_H__

//#include	"mistl/Vector3_embree.h"


#include <Core/FixedVector.h>
#include <assert.h>
#include <cmath>

namespace mistl {


template<typename Ty> 
class Vector4: public FixedVectorC<Ty, 4>
{
public:
    Vector4 ()
    { 
        this->m_data[0] = 0.0;
        this->m_data[1] = 0.0;
        this->m_data[2] = 0.0;
        this->m_data[3] = 0.0;
    }
    // Default constructor, sets elements to zero

    Vector4 (Ty x, Ty y, Ty z, Ty w)
    {
       this->m_data[0] = x;
       this->m_data[1] = y;
       this->m_data[2] = z;
       this->m_data[3] = w;
    }
    // Data constructor.

//     Vector4(const FixedVectorC<Ty, 4>& v)
//     : FixedVectorC<T, 4>(v)
//     { }
    // Base constructor.

	
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
  
    Ty &W()
    { return this->m_data[3]; }
    Ty W() const
    { return this->m_data[3]; }
    // Access third component

public:
    // Vector operations


    inline void Normalize(double nearlyzero = 1e-12)
    { 
       double len = this->Magnitude();
       if (len > nearlyzero)
       {
          this->m_data[0] /= len;
          this->m_data[1] /= len;
          this->m_data[2] /= len;
          this->m_data[3] /= len;
       }
    }
};

typedef mistl::Vector4<float> Vector4f;
typedef mistl::Vector4<double> Vector4d;

}

#endif