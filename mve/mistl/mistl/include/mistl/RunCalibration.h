
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


#ifndef  RunCalibration_incl_
#define  RunCalibration_incl_

#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/ReadCoordinateFile.h"
#include        "mistl/MultiCameraCalibration.h"
#include        "mistl/MultiCamTracking.h"
#include        "mistl/MultiCamTrackingFeatures.h"
#include        "mistl/Log.h"
#include        "mistl/Quality.h"
#include        "mistl/Histogram.h"


#include "opencv2/core/core.hpp"

#include "dlm/scene/Scene.h"


namespace mistl {
  
enum CalibrationCode { 
        Success = 0, Failed, NotEnoughFeatures, /*UnCoherentCorresponcences,*/ InCalibration, OutOfCalibration
};
typedef CalibrationCode CalibrationCodeFlag;
 
  
class CalibrationQuality  : public Quality {
public:
//    virtual void Compute();
  
 
public:

  mistl::TrackQuality   trackquality;
  mistl::CalibrationCodeFlag  trackrc;
  
  float initialmatch;   //!< res.error after point triangulation
  unsigned initial_n;
  float p3dmatch;       //!< res.error after point BA
  unsigned p3d_n;
  float finmatch;          //!< final BA residual error
  unsigned fin_n;
};



 

/*! \class RunCalibration RunCalibration.h
  \brief Convenience class to run calibration from one set of images
  \author O.Grau
*/
class RunCalibration {
public:           
  RunCalibration();
  virtual ~RunCalibration();

  //! Set default parameters
  void SetDefaults();
  
  //! Set inital camera parameters
  void SetInitialCameraParameterList( const std::vector<mistl::Camera> & camlist ) ;

  //! Initialize calibration. 
  void Init(  );

  //! Get copy of initial camera objects. This method is redundant and returns GetCameras()
  void GetInitialCameras( std::vector<mistl::Camera> & camlist ) const;
  
  //! Get copy of estimated camera objects. 
  void GetCameras( std::vector<mistl::Camera> & camlist ) const;

   //! Set reference cameras. This is only needed to test calibration results against a reference calibration and to check parameter boundaries
  void SetReferenceCameras( const std::vector<mistl::Camera> & camlist ) ;
  
  //! Return distance of calibration to reference 
  float DistanceToRef() const;
  
 //! Bring refence camera to orgin and zero-rotation and scale the positions of the remaining cameras
  void NormScaleCameras( double scale = 1.0, mistl::Transform3f *tr = ( mistl::Transform3f *)0);
  
  /*! \brief Calibrate camera from one set of images. 
   * 
   * Runs BuildData() and CameraCalibration(). Returns 0 if calibration was succesfull or a error code otherwise
   */
  CalibrationCodeFlag  Calibrate(  );

  /*! Build scene data. 
   * 
   * This call builds the data for the BA. It then does a first pass with fixed cameras to triangulate 3D scene points.
   * The methods returns 0 if data is ok. Otherwise if there is not sufficient data to to a BA for calibration an error
   * code is returned, which can be converted into a message with CalibrationCode().
   */
  CalibrationCodeFlag  BuildData() ;
  
  //! Returns 0 if calibration was succesfull or a error code otherwise
  CalibrationCodeFlag CameraCalibration();
  
  //! Establish correspondences between a set of cameras. Parameters and results of the tracking are store in mtrack.
  CalibrationCodeFlag  Tracking( const std::vector<cv::Mat>       & framelist, //!< list of images      
                  unsigned      seq_id=0          //!< Frame number of image set. This will be copied to 'seq_no' 
                  );
  
  //! translate Calibrate() return code to string
  const char *CalibrationCode (CalibrationCodeFlag n) const;
  
  //! Write parameters to yaml-file stream
  void Store( cv::FileStorage &fs );
  //! Read parameters from yaml-file stream
  void ReStore( cv::FileNode &tm ) ;
  
  bool  DebugMode() const { return mistl::DebugMode(); }
  
  //! Returns filename to generate temp log and debug information in log-directory
  std::string   DebugLogFile( const std::string fn ) const {
    return mistl::GetLogDirectory() + "/" + fn ;
  }
  
  //! Debugging function. Writes data to files in log directory
  void DumpData( const std::string tag ,
                 bool write_3dpoints,              //!< write 3d point list
                 bool write_2dlist                 //!< write 2d coordinate list
               ) ;
  
  //! Compute a 3d histogram
  void Compute3DHistogram();
  //! Compute confidence values for 3d histogram bins
  void Judge3DHistogram(  float &near, float &mid, float &far )  const;
   //! Compute confidence values for 3d histogram bins
  float Judge2DHistogram(  )  const;
 
  //! Used to re-order camera indices (E.g. for DR where reference camera is not index=0
//   virtual unsigned MapCameraIndex( unsigned n) const ;
  
  //! Compute quality of calibration. Automatically called by CameraCalibration()
  void ComputeQuality();
  
  //! Quick test to determine whether a multi-camera system is still in calibration. Need to be called after Tracking()
  mistl::CalibrationCodeFlag ReprojectionTest( );
  
  //! Check that estimated parameters are in boundaries (max_rot,..). Returns false if outside boundaries
  bool BoundaryCheck();
  
public:
  mistl::CalibrationQuality     q;
  
  //! This list contains results of the calibration after successfull call of Calibrate()
  std::vector<mistl::Camera>  camlist;
  //! Tracking object (contains tracking results)
  mistl::MultiCamTrackingFeatures      mtrack;
  //! Calibration object
  mistl::MultiCameraCalibration cal;
  
  // Parameter
  bool  outlier_removal;
  unsigned estimate_dist;
  float outlier_max_err;        //!< used to prune out points after generation
  float outlier_max_err2;       //!< used to prune out points after BA of all parameters
  bool verbose;
  bool triangulationinit;
  float pointdistance;          //!< Initial distance of 3D points generated from Tracker 
  std::vector<OptimizeCameraFlags>     est_pos;         //!< enable position estimation per camera
  std::vector<OptimizeCameraFlags>     est_rot; 	//!< enable rotation estimation per camera
  std::vector<OptimizeCameraFlags>     est_cps; 	//!< enable center point shift estimation per camera
  std::vector<OptimizeCameraFlags>     est_f; 	//!< enable focal length estimation per camera
  std::vector<int>     est_k;   //!< enable radial distortion k1 estimation per camera
  std::vector<bool>     est_p;   //!< enable tangential distortion  estimation per camera

  // boundaries for rejecting parameter estimations that are too far off the start parameters
  std::vector<float>     max_rot;  //!< Check rotation against start parameters. (See SetReferenceCameras() )

  unsigned      seq_no; //!< This is typically the frame number of an image set. It is set by Tracking()
  
  
  float reprojectionthr;      //!< Threshold for re-projection test (see ReprojectionTest() )
  
  // Parameters for quality estimation
  float residualref;
  float residualoffset;
  int minimum_n;
  float lambda;
  int outliers;
  
  // solving strategies
  bool  rotation_preference;    //!< If true then try to solve for rotation only in first stage solve
  //
  //
  //------------------
  // Auxillary stuff
  //------------------
  //
  
//   std::string   feedbackwindow; //!< register OpenCV window to allow visual feedback 
  cv::Mat       visrefimage;    //!< reference image for visual feedback
  bool  visual_feedback;        //!< Write out images with correspondences found in tracking.
  bool create_coord_files;      //!< Write out 2d correspondences found in tracking.
  
  unsigned point_id_offset;     //!< This is an offset that is increased every frame by the number of 3D scene points

  mistl::Histogram     histogram3d;
  mistl::Histogram     histogram2d;
  
  //! Reference cameras - this is for test purposes to compare differences to the calibration results
  std::vector<mistl::Camera>  refcamlist;
private:
  
  
};




}
#endif
