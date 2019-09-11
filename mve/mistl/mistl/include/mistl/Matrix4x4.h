
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

#ifndef __MISTL_MATRIX44_H__
#define __MISTL_MATRIX44_H__

#include <Core/FixedMatrix.h>
#include <mistl/Vector3.h>
#include <iostream>
#ifdef      USE_OPENCV
#include "opencv2/core/core.hpp"
#endif

namespace mistl {		

class Matrix4x4f: public FixedMatrixC<float,4,4>
{
public:
// Constructors
	
	Matrix4x4f()
	{
    	this->m_data[0][0] = 0.0; this->m_data[0][1] = 0.0; this->m_data[0][2] = 0.0;
    	this->m_data[1][0] = 0.0; this->m_data[1][1] = 0.0; this->m_data[1][2] = 0.0;
    	this->m_data[2][0] = 0.0; this->m_data[2][1] = 0.0; this->m_data[2][2] = 0.0;
	}
	// Default constructor sets the elements to zero
	
  Matrix4x4f(float R00, float R01, float R02,
             float R10, float R11, float R12,
             float R20, float R21, float R22)
	{
    	this->m_data[0][0] = R00; this->m_data[0][1] = R01; this->m_data[0][2] = R02;
    	this->m_data[1][0] = R10; this->m_data[1][1] = R11; this->m_data[1][2] = R12;
    	this->m_data[2][0] = R20; this->m_data[2][1] = R21; this->m_data[2][2] = R22;
	}
    // Construct from elements of the matrix	
  Matrix4x4f(const FixedMatrixC<float, 4, 4>& R) : FixedMatrixC<float, 4, 4>(R){}
   
  //! return value, no check on coordinates
  float operator ()(int row, int col) const { return this->m_data[row][col]; }

  mistl::Vector3f operator*(mistl::Vector3f v){
    mistl::Vector3f returnVector(this->m_data[0][0] * v.X() + this->m_data[0][1] * v.Y() + this->m_data[0][2] * v.Z(),
                                 this->m_data[1][0] * v.X() + this->m_data[1][1] * v.Y() + this->m_data[1][2] * v.Z(),
                                 this->m_data[2][0] * v.X() + this->m_data[2][1] * v.Y() + this->m_data[2][2] * v.Z());
      
    return returnVector;
  }

#ifdef      USE_OPENCV
  Matrix4x4f(const cv::Mat &cvmat) {
    std::cout << "Matrix4x4 (cvmat) type:" << cvmat.type() << " \t" << cvmat.rows << "," << cvmat.cols << "\n";

    MISTL_ASSERT(cvmat.type() == CV_64F, "converting cv::Mat has wrong type");
    MISTL_ASSERT((cvmat.rows == 4) && (cvmat.cols == 4), "converting cv::Mat has wrong size");

    for (int i = 0; i < 4; ++i){
      for (int j = 0; j < 4; ++j){
        this->m_data[i][j] = cvmat.at<double>(i, j);
      }
    }
  }

  cv::Mat cvMat() const {
    cv::Mat	cvmat(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i){
      for (int j = 0; j < 4; ++j){
        cvmat.at<double>(i, j) = this->m_data[i][j];
      }
    }

    return cvmat;
  }
#endif
   

public:
// Static functions
	
	static Matrix4x4f Identity()
	{
    return FixedMatrixC<float, 4, 4>::Identity();
	}

public:
// Matrix operations
	
   
};

}    // namespace mistl 

#endif 
