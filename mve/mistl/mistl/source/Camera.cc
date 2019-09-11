

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

#include	"mistl/Camera.h"



namespace mistl {
  



void
Camera ::SetCAHV( const mistl::Vector3f ci, const mistl::Vector3f ai, const mistl::Vector3f hi, 
			  const mistl::Vector3f vi,
			  int  width, int  height, float isx, float isy  )
{
	SetC(ci);
	SetTarget(width,height, isx,isy);

	const float tmp_csx = ai.Dot(hi);
	const float tmp_csy = ai.Dot(vi);
	mistl::Vector3f v0(vi - ai*(tmp_csy));
	const float len = v0.Length();                 // is this in mm ????
	if (len > 0.0f)
	    v0 = v0 * -1.0f/len;

	SetOrientation(ai,v0);
	SetF( len*isy );
	SetCenterPointShiftX( tmp_csx * isx );
	SetCenterPointShiftY( tmp_csy * isy );
}

// #ifdef  USE_DLM
// dlm::Camera         Camera ::Intrinsics() const
// {
// }
// 
// dlm::Transformation Camera ::Extrinsics() const
// {
// }
// 
// #endif

float Distance( const mistl::Camera &cam1,  //!< reference camera
               const mistl::Camera &cam2 ,  //!< camera 2
                float scene_scale,
                bool omit_distortions
             )
{
  double r=0.0;
  double v;
  
  // pose
  v = (cam1.C()-cam2.C()).SumOfSqr() * scene_scale; r+=v;
std::cout << "C:"<<v<<" ";
  v = (cam1.A()-cam2.A()).SumOfSqr();r+=v;
std::cout << "A:"<<v<<" ";
  v= (cam1.H0()-cam2.H0()).SumOfSqr();r+=v;
std::cout << "H:"<<v<<" ";

  v= (cam1.V0()-cam2.V0()).SumOfSqr();r+=v;
std::cout << "V:"<<v<<" ";

  // 
  v = cam1.GetF()-cam2.GetF();
  v= v*v* 300.0*300.0; r+=v; // fixed norm
std::cout << "F:"<<v<<" ";
  v = cam1.GetPx()-cam2.GetPx();
  v= v*v * 0.01*0.01; r+=v;
std::cout << "Px:"<<v<<" ";
  v = cam1.GetPy()-cam2.GetPy();
  v= v*v * 0.01*0.01; r+=v;
std::cout << "Py:"<<v<<" ";

  //
  if(!omit_distortions) {
double rkk=r;
  v = cam1.GetK1()-cam2.GetK1();
  r += v*v* 100.0;
  v = cam1.GetK2()-cam2.GetK2();
  r += v*v* 100.0;
  v = cam1.GetK3()-cam2.GetK3();
  r += v*v* 100.0;
  v = cam1.GetP1()-cam2.GetP1();
  r += v*v* 100.0* 100.0;
  v = cam1.GetP2()-cam2.GetP2();
  r += v*v* 100.0* 100.0;
std::cout << "KP:"<< r-rkk;
  }
  std::cout <<"\n";
  return sqrt( r );

}

float Distance( const std::vector<mistl::Camera> &cam1list,  //!< reference camera
               const std::vector<mistl::Camera> &cam2list,   //!< camera 2
                float scene_scale,          //!< Scene scale
                bool omit_distortions
             )
{
  std::cout << "Distance("<<cam1list.size()<<","<<cam2list.size()<<")\n";
  
  MISTL_ASSERT( cam1list.size()==cam2list.size(), "Distance(cam1list,cam2list): lists have different lengths");
  float r=0.f;
  for(unsigned i=0; i<cam1list.size(); ++i)
    r += Distance( cam1list[i], cam2list[i], scene_scale, omit_distortions );
  return r;
}


}
