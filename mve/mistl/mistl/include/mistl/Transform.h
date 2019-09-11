
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


#include	"math/vec3.h"

namespace mistl {


/*!
  \class Transform Transform.h
  \brief 3D transformation class (rotation+translation)
*/
template<typename T> 
class Vector3 : public embree::Vec3<T>
{
};


typedef Vector3<float> Vector3f;
typedef Vector3<double> Vector3d;

}

#endif