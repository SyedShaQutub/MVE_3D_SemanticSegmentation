
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

#ifndef __MISTL_MATRIX33_H__
#define __MISTL_MATRIX33_H__

#include <Core/FixedMatrix.h>
#include <mistl/Vector3.h>
#include <iostream>
#ifdef      USE_OPENCV
#include "opencv2/core/core.hpp"
#endif

namespace mistl {		

#ifdef _MSC_VER
#pragma warning( disable : 4244 ) // conversion from 'const double' to 'float', possible loss of data
#endif

class Matrix3x3f: public FixedMatrixC<float,3,3>
{
public:
// Constructors
	
	Matrix3x3f()
	{
    	this->m_data[0][0] = 0.0; this->m_data[0][1] = 0.0; this->m_data[0][2] = 0.0;
    	this->m_data[1][0] = 0.0; this->m_data[1][1] = 0.0; this->m_data[1][2] = 0.0;
    	this->m_data[2][0] = 0.0; this->m_data[2][1] = 0.0; this->m_data[2][2] = 0.0;
	}
	// Default constructor sets the elements to zero
	
  Matrix3x3f(float R00, float R01, float R02,
             float R10, float R11, float R12,
             float R20, float R21, float R22)
	{
    	this->m_data[0][0] = R00; this->m_data[0][1] = R01; this->m_data[0][2] = R02;
    	this->m_data[1][0] = R10; this->m_data[1][1] = R11; this->m_data[1][2] = R12;
    	this->m_data[2][0] = R20; this->m_data[2][1] = R21; this->m_data[2][2] = R22;
	}
    // Construct from elements of the matrix	
  Matrix3x3f(const FixedMatrixC<float, 3, 3>& R) : FixedMatrixC<float, 3, 3>(R){}
   
  //! return value, no check on coordinates
  float operator ()(int row, int col) const { return this->m_data[row][col]; }

  mistl::Vector3f operator*(mistl::Vector3f v){
    mistl::Vector3f returnVector(this->m_data[0][0] * v.X() + this->m_data[0][1] * v.Y() + this->m_data[0][2] * v.Z(),
                                 this->m_data[1][0] * v.X() + this->m_data[1][1] * v.Y() + this->m_data[1][2] * v.Z(),
                                 this->m_data[2][0] * v.X() + this->m_data[2][1] * v.Y() + this->m_data[2][2] * v.Z()); 
    return returnVector;
  }
  
//   mistl::Matrix3x3f operator*(const mistl::Matrix3x3f &v) const {
//     mistl::Matrix3x3f r; //assume it is pre-initalized with zeros
//     unsigned i, j, k;
//     for(i = 0; i < 3; i++)
//             for(j = 0; j < 3; j++) 
//                     for(k = 0; k < 3; k++)
//                             r[i][j] +=  (*this)[i][k] *  v[k][j];
//     return r;
//   }
  
//   mistl::Matrix3x3f Identity () const{
//     mistl::Matrix3x3f r; //assume it is pre-initalized with zeros
//     unsigned i;
//     for(i = 0; i < 3; i++)
//           r[i][i]= 1.f;
//     return r;
//   }
 
#ifdef      USE_OPENCV
  Matrix3x3f(const cv::Mat &cvmat) {
//     std::cout << "Matrix3x3 (cvmat) type:" << cvmat.type() << " \t" << cvmat.rows << "," << cvmat.cols << "\n";

    MISTL_ASSERT(cvmat.type() == CV_64F, "converting cv::Mat has wrong type");
    MISTL_ASSERT((cvmat.rows == 3) && (cvmat.cols == 3), "converting cv::Mat has wrong size");

    for (int i = 0; i < 3; ++i){
      for (int j = 0; j < 3; ++j){
        this->m_data[i][j] = cvmat.at<double>(i, j);
      }
    }
  }

  cv::Mat cvMat() const {
    cv::Mat	cvmat(3, 3, CV_64F);
    for (int i = 0; i < 3; ++i){
      for (int j = 0; j < 3; ++j){
        cvmat.at<double>(i, j) = this->m_data[i][j];
      }
    }

    return cvmat;
  }
#endif
   

public:
// Static functions
	
	static Matrix3x3f Identity()
	{
    return FixedMatrixC<float, 3, 3>::Identity();
	}

public:
// Matrix operations
	
    //! Return determinant
    float Det() const {
      const float a = (*this)(0, 0);
      const float b = (*this)(0, 1);
      const float c = (*this)(0, 2);
      const float d = (*this)(1, 0);
      const float e = (*this)(1, 1);
      const float f = (*this)(1, 2);
      const float g = (*this)(2, 0);
      const float h = (*this)(2, 1);
      const float i = (*this)(2, 2);
      return a*(e*i-f*h) - b*(i*d-f*g) + c*(d*h-e*g);
    }

    //! Return inverse of matrix
    Matrix3x3f Inverse() const {
      const float a = (*this)(0, 0);
      const float b = (*this)(0, 1);
      const float c = (*this)(0, 2);
      const float d = (*this)(1, 0);
      const float e = (*this)(1, 1);
      const float f = (*this)(1, 2);
      const float g = (*this)(2, 0);
      const float h = (*this)(2, 1);
      const float i = (*this)(2, 2);
      const float det = this->Det();
// std::cout<< *this <<" det:"<<det<<"\n";
      return Matrix3x3f(  
        (e*i-f*h)/det, -(b*i-c*h)/det, (b*f-c*e)/det,
        -(d*i-f*g)/det, (a*i-c*g)/det, -(a*f-c*d)/det,
        (d*h-e*g)/det, -(a*h-b*g)/det, (a*e-b*d)/det );
    }	
};

}    // namespace mistl 

#endif 
