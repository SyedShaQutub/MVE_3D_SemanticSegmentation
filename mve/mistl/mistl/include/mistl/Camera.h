
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


#ifndef __MISTL_CAMERA_H__
#define __MISTL_CAMERA_H__

#include	"mistl/Precision.h"

#include	"mistl/Vector3.h"
#include	"mistl/Vector2.h"
#include	"mistl/Transform3.h"
#include	"mistl/Distortion.h"
#include	"mistl/DistortionKP.h"
#include    "mistl/Error.h"

#ifdef  USE_DLM
#include "dlm/entity/Camera.h"
#include "dlm/entity/Transformation.h"
#endif

namespace mistl {


/*!
  \class Camera Camera.h
  \brief Camera class
  \author O.Grau
*/

class Camera : public mistl::DistortionKP
{
public:
  Camera() /*: mistl::Distortion(0.0f, 0.0f)*/ {
          csx=csy=0.0f;
          f=1.0f;
          nx=ny=100;
          sx=sy=1e-6f;
          
//                 this->SetC( mistl::Vector3f (0,0,0));
//                 this->SetOrientation( mistl::Vector3f (0,0,-1), mistl::Vector3f (0,1,0) );
          pose.t=mistl::Vector3f (0.0f,0.0f,0.0f);
          SetA(mistl::Vector3f (0.0f,0.0f,-1.0f)); 
          SetUp(mistl::Vector3f (0.0f,1.0f,0.0f)); 
          UpdateCache();
          
  }
  Camera( const mistl::Camera &c ) {
    Copy(c);
  }
  
  void Copy( const mistl::Camera &c ) {
    pose = c.pose;
    up_vec = c.up_vec;
    csx = c.csx;
    csy = c.csy;
    f = c.f;
    nx=c.nx; ny=c.ny; 
    sx=c.sx; sy=c.sy;
    name=c.name;
    camId=c.camId;
    isReferenceCamera=c.isReferenceCamera;
    mistl::DistortionKP::Copy(c);
    UpdateCache();
  }
  mistl::Camera& operator=(const mistl::Camera& other) { Copy(other); return *this;}
  
  
  const mistl::Transform3f operator =( const mistl::Transform3f &tt) {  pose=tt; UpdateCache(); return pose; }
  
  //! Return center point (position) of camera in [m]3
  const mistl::Vector3f C() const { return pose.t; }
  
  const mistl::Vector3f H0() const { return pose.RotationVx(); }
  const mistl::Vector3f V0() const { return pose.RotationVy(); }
  const mistl::Vector3f A() const { return pose.RotationVz(); }
  const mistl::Vector3f H() const { return h_vec; }
  const mistl::Vector3f V() const { return v_vec; }
  const mistl::Vector3f Up() const { return up_vec; }
  int	GetNx() const {return nx;}
  int	GetNy() const {return ny;}				//!< Return image height in [pel]	
  float	GetF() const { return f; }				//!< Return f-constant (focal length) in [m]
  float	GetCenterPointShiftX() const { return csx; }		//!< Center point shift in [m]
  float	GetCenterPointShiftY() const { return csy; }
  float	GetSx() const { return sx; }                             //!< Return pixel size in [m]
  float	GetSy() const { return sy; }
	
  /** \brief Get camera pose (external orientation)
   * 
   * This is currently not a true transformation, since the translational part is C (rather than -((rot-1)*C) ) !
   */
  const mistl::Transform3f &GetPose() const {return pose;}
  

  
    
  //! Set position of camera
  void	SetC( const mistl::Vector3f &c ) { pose.t=c; UpdateCache(); }
  //! Set camera orientation
  void	SetOrientation( const mistl::Vector3f &a, const mistl::Vector3f &up ) { 
          SetA(a); SetUp(up); UpdateCache();
  }
  //! Set camera pose (postion and orientation)
  void    SetPose( const mistl::Transform3f     & posein ) {
    mistl::Vector3f na,nup;
    na= posein.RotationVz();
    nup= posein.RotationVy()* -1.0;
    SetOrientation(na,nup);
    SetC(posein.t);
  }
  //! Set image sensor details
  void	SetTarget( 
              unsigned idx, //!< width 
              unsigned idy, //!< height
              float isx,         //!< pixel width in [m]
              float isy          //!< pixel height in [m]
                    ) {
          nx=idx; ny=idy; sx=isx; sy=isy; UpdateCache();
  }
	
  //! Set calibration constant (aka focal length) in [m]
  void SetF(float nf) {
          f = nf;
          UpdateCache();
  }

  //! Center point shift in [m]
  void SetCenterPointShiftY(float cpy)	{
          csy = cpy;
          UpdateCache();
  }
  //! Center point shift in [m]
  void 	SetCenterPointShiftX(float cpx)	{
          csx = cpx;
          UpdateCache();
  }

  //! x-component of principal point in [pel]
  float GetPx() const {
          return cx;
  }

  //! y-component of principal point in [pel]
  float GetPy() const {
          return cy;
  }

  void	Info() const { 
		  // ted VS 2012 didn't like this:
		    std::cout << "camera \"" << name << "\": " << C() << " a:" << A() << " up:" << Up() << "\n  \\";
		  //std::cout << "camera \\ <name> \\ " <<C()<< " a:"<<A()<<" up:"<<Up()<<"\n  \\";

          std::cout <<" nx,ny:"<<GetNx()<<','<<GetNy();
          std::cout <<" sx,sy:"<<GetSx()<<','<<GetSy();
          std::cout <<" f:"<<GetF();
          std::cout << " Px,y:"<< GetPx()<<','<<GetPy() ;
          std::cout << " CSx,y:"<< GetCenterPointShiftX()/GetSx()<<','<<GetCenterPointShiftY()/GetSy() ;
          mistl::DistortionKP::Info();
// 		std::cout << "\n";
  }
  
  //#########################################
  
  //! Distortion method for image coordinates 
  void  Distort ( const float x, const float y, //!< Input coordinates in [pel]
            float &outx, float &outy  //!< distorted coordinates in [pel] 
  ) const {
      float udx = (x-GetPx())/fx;
      float udy = (y-GetPy())/fy;
      float ddx,ddy;
      DistortICC( udx,udy, ddx,ddy );
      outx=fx*ddx+cx;
      outy=fy*ddy+cy;
  }
  //! Undistortion of image coordinates
  void
  UnDistort( const float x, const float y, //!< Distorted input coordinates in [pel]
            float &outx, float &outy  //!< undistorted coordinates in [pel] 
  ) {
      float ddx = (x-GetPx())/fx;
      float ddy = (y-GetPy())/fy;
      float udx,udy;
      UnDistortICC( ddx,ddy, udx,udy );
      outx=fx*udx+cx;
      outy=fy*udy+cy;
  }
  //! Projection  
  void ProjectICC( const mistl::Vector3f &X, float &x, float &y, float &z = *((float *)0) ) const {
          mistl::Vector3f p_c = X - C();
          float divisor = p_c.Dot( A());

          // check against 1e-30, not DBL_MIN because the result can be still smaller than divisor
          if( divisor < 1e-15f  ) {
                  divisor = 1e-15f;

          }
          // radial-distortion-free projection:
          x = p_c.Dot( H0()) / divisor;
          y = p_c.Dot( V0()) / divisor;
          if( &z != 0) z=divisor;
  }
  //! Projection 
  void Project( const mistl::Vector3f &X, //!< 3d point
                float &x, //!< image coordinate in [pel]
                float &y,  //!< image coordinate in [pel]
                float &z = *((float *)0) //!< depth in [m]
              ) const {
          mistl::Vector3f p_c = X - C();
          float divisor = p_c.Dot( A());

          //std::cout << "P-C : "<<p_c<<" z:"<<z<<"\n";
          //std::cout << "h: "<<h_vec<< "\tv: "<<v_vec<<"\n";

          // check against 1e-30, not DBL_MIN because the result can be still smaller than divisor
          if( divisor < 1e-15f  ) {
                  divisor = 1e-15f;

          }
//              float x,y;
          // radial-distortion-free projection:
          x = p_c.Dot( H0()) / divisor;
          y = p_c.Dot( V0()) / divisor;
          // x,y are without units now (in ICC), apply ICC-Distortion ala OpenCV
          DistortICC(x,y, x,y);
          
          // bring x,y into image domain (units:pel)
          x=fx*x+cx;
          y=fy*y+cy;
          if( &z != 0) z=divisor;
  }        
  void Project( const mistl::Vector3f &X, mistl::Vector2f &px) const {
          Project(X, px.X(), px.Y() );
  }
  
  //! Return line of sight (direction)
  mistl::Vector3f LineOfSight( float px, //!< x-coordinate in [pel]
                               float py //!< y-coordinate in [pel]
                             ) {
    mistl::Vector3f S, S1, S2;
    
    MISTL_ASSERT( (fx != 0),"invalid value for fx: fx=0\n");
    MISTL_ASSERT( (fy != 0),"invalid value for fx: fy=0\n");

    float x = (px-cx)/fx;  // convert to unit-less image coordinates
    float y = (py-cy)/fy;
    UnDistortICC( x,y, x,y);
    
    S1 = A() * x - H0();
    S2 = A() * y - V0();
    S = S1.Cross(S2);

    S.Normalize();
    return S;
  }
  mistl::Vector3f LineOfSight( const mistl::Vector2f &px) {
      return LineOfSight(px.X(), px.Y());
  }  
  
  //! Convert from CAHV notation
  void SetCAHV( const mistl::Vector3f ci, const mistl::Vector3f ai, const mistl::Vector3f hi, 
                    const mistl::Vector3f vi,
                int  dx, int  dy, float isx, float isy
              );

  //! Camera name
  std::string   name;

  //! flag that idicates if the camera will be used as reference camera
  // OLG: That should not be in here!! Will be taken out in later version - do not use!
  bool isReferenceCamera;

  //! camera ID
  int camId;

protected:
  void 	SetH0(mistl::Vector3f v)  { v.Normalize(); pose.SetRotationVx(v); }
  void	SetV0(mistl::Vector3f v)  { v.Normalize(); pose.SetRotationVy(v); }
  void	SetA(mistl::Vector3f v)  {  v.Normalize(); pose.SetRotationVz(v); }
  void	SetUp(mistl::Vector3f v)  {  v.Normalize(); up_vec=v; }
  
  
  void UpdateCache() {
    SetV0( up_vec * -1.0 );
    SetH0( V0().Cross( A() ) );
    h_vec = H0() * (GetF()/GetSx()) + A() * GetPx();        // not sure we need this any more?
    v_vec = V0() * (GetF()/GetSy()) + A() * GetPy();
    fx= GetF()/GetSx();
    fy= GetF()/GetSy();
    cx= 0.5f * (GetNx()-1) + GetCenterPointShiftX()/GetSx();
    cy= 0.5f * (GetNy()-1) + GetCenterPointShiftY()/GetSy();
  }
  
  // camera parameters
  mistl::Transform3f	pose;
  mistl::Vector3f	h_vec,v_vec;
  
  int 	nx;	// image target width
  int	ny;	// image target height
  
  mistl::Vector3f	up_vec;
  float	sx,sy;	// Pixel size in the image plane
  float	csx, csy;	// center point shift
  float	f;	// focal length
  float   fx;     // premultiplied  in [pel]
  float   fy;     // premultiplied  
  float   cx,cy;  // premultiplied  in [pel]

	
};



/**
 * @ingroup MISTLCFunction
 * @brief Read camera parameters from file
 * @param cam camera object
 * @param fn file name
 * @author O.Grau
**/
void ReadCamera( mistl::Camera &cam, const char *fn  );


/**
 * @ingroup MISTLCFunction
 * @brief Write camera parameters to file
 * @param cam camera object
 * @param fn file name
 * @author O.Grau
**/
void WriteCamera( const mistl::Camera &cam, const char *fn  );



/**
 * @ingroup MISTLCFunction
 * @brief Compare two cameras. Returns a numerical distance value
 * @author O.Grau
**/
float Distance( const mistl::Camera &cam1,  //!< reference camera
               const mistl::Camera &cam2,   //!< camera 2
                float scene_scale = 8.f ,        //!< Scene scale
                bool omit_distortions = true
             );


float Distance( const std::vector<mistl::Camera> &cam1list,  //!< reference camera
               const std::vector<mistl::Camera> &cam2list,   //!< camera 2
                float scene_scale = 8.f,         //!< Scene scale
                bool omit_distortions = true
             );


class TriangularMesh;

/**
 * @ingroup MISTLCFunction
 * @brief Generate camera visualisation as 3d mesh
 * @param cam camera object
 * @param size of camera
 * @author O.Grau
**/
mistl::TriangularMesh *
VisualizeCamera(const mistl::Camera &cam, float size);

  

}


#endif
