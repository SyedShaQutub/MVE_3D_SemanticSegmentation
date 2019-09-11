
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


#ifndef  MultiCameraCalibration_incl_
#define  MultiCameraCalibration_incl_

#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/ReadCoordinateFile.h"
#include        "mistl/Transform3.h"

#include "dlm/scene/Scene.h"


// forward declarations
// namespace dlm {
//   class Scene ;
// }

namespace mistl {
  
  
class   TrackInfo {
public:
  unsigned camid;
  unsigned seq;
  std::vector<mistl::CoordinateListEntry> clist;
};


// enum OptimizePose { NoParameter = 0,  OptimizeRotationEuler = 0x7, OptimizePan = 0x1, OptimizeTilt = 0x2, OptimizeRoll = 0x4, 
//                       OptimizeX = 0x8, OptimizeY = 0x10,  OptimizeZ = 0x20, OptimizePosition = 0x38 };
enum OptimizeFlags { 
        NoParameter = 0,  OptimizeRotationEuler = 0x7, OptimizePan = 0x1, OptimizeTilt = 0x2, OptimizeRoll = 0x4, 
        OptimizeX = 0x8, OptimizeY = 0x10,  OptimizeZ = 0x20, OptimizePosition = 0x38,
        OptimizeF = 0x40, 
        CenterpointShift = 0x180, CenterpointShiftX = 0x80, CenterpointShiftY = 0x100,
        K3K5Terms = 0x600, //!< Estimate radial distortion terms K3 and K5
        K3Term = 0x200, //!< Estimate radial distortion term K3
        K5Term = 0x400, //!< Estimate radial distortion term K5
        Tangentialdistortion = 0x800, //!< Estimate tangential distortion terms p1+p2
        Intrinsics = OptimizeF | CenterpointShift | K3K5Terms
  };
  
typedef OptimizeFlags OptimizeCameraFlags;
  
class MultiCameraCalibration {
public:
  
                      
    MultiCameraCalibration();
    ~MultiCameraCalibration();
    void Clear();
    //! Add/create a new camera
    void        AddCamera( const mistl::Camera        &cam, bool isfixed);
    
    //! Add localised (2d) point data for a particular camera and frame
    void        AddTrack( unsigned camid,  //! camera id
                          unsigned seq, //!< frame/sequence id
                          const std::vector<mistl::CoordinateListEntry> &clist, 
                          bool optimze_points=false 
                        );
    //! Call the solver
    float Solve();
    //! Return number of cameras
    unsigned    NCamera() const ;
    //! Get copy of camera parameters
    void GetCamera( unsigned camid,  mistl::Camera        &cam) ; //const;
    //! Set camera parameters 
    void SetCamera( unsigned camid, const mistl::Camera        &cam);
    
    //! Get localised (2d) point data for a particular camera and frame
    void        GetTrack( unsigned camid,  //! camera id
                          unsigned seq, //!< frame/sequence id
                          std::vector<mistl::CoordinateListEntry> &clist
                        );

    //! Set estimation of distortion (0:disable, 1..3). Only k1,..,k3 radial distortions are currently supported.
    void EstimateK( unsigned camid,     //!< camera id
                    unsigned k_no       //!< number of radial terms (0,..,3)
                  );
    
    //! Get list of all object (3D) points
//     void GetPointList( std::vector<mistl::Vector3f> &plist ) ;
                       
     //! Get list of all object (3D) points
    void GetPointList( std::vector<mistl::Vector3f> &plist, std::map<unsigned,unsigned> &index_translate,
                       mistl::Transform3f *tr = ( mistl::Transform3f *)0, 
                       double scale = 1.0 ) ;
                      
    //! Get list of (2D) points 
    void Get2DPointList( unsigned camid, unsigned seqid, 
                         const std::map<unsigned,unsigned> &index_translate,
                          std::vector<mistl::Vector2f> &plist, 
                         std::vector<unsigned> &indexlist );
    
    /*! \brief Get object transformation 
     * 
     * Returns the object transform of frame-id 'seq'. This id must exist otherwise a error is raised.
     */
    mistl::Transform3f GetTransform( unsigned seq ) const;
    
    //! Returns list of sequence ids.
    std::vector<unsigned> GetSeqIdList() const;
    
    /*! Return number of transformation objects.
     * 
     * The sequence ids do not need to be in linear order. Use GetSeqIdList() to get a list of all ids.
     */
    unsigned NoOfTransformations() const { return (unsigned)transseq_lookup.size(); }
    
    //! Activate/disable optimization of transformation object
    void OptimizeTransformObjects( unsigned seq, bool opt ) ;
    
    //! Enable/disable Estimation of tangential distortions
    void EstimateTangentialDist( unsigned camid, bool val );

    //! Set flags for camera optimization
    void OptimizeCamera( unsigned cam,  int opt );
    
     //! Set flags for camera optimization
    int  GetOptimizeCameraFlag( unsigned cam );
    
    //! Remove outliers
    unsigned RemoveOutlier( float max_delta, bool isVerbose);
    
    // kept in for reference
//     unsigned RemoveOutlierOld( float max_delta, bool isVerbose);

    // misc
    dlm::Vector3d   dlmVector( const mistl::Vector3f &v) const { return dlm::Vector3d( v.X(), v.Y(), v.Z() ); }
    dlm::Vector2d   dlmVector2D( const mistl::Vector2f &v) const { return dlm::Vector2d( v.X(), v.Y() ); }
    dlm::Quatd      Orientation(const mistl::Camera        &cam) const;

    void Store( ) ;
    
    void  UpdateCameras() ;
    
    /*! \brief Evaluate data includes some debug tests. 
     * 
     * More informative than EvaluateN(),  but slower.
     */
    double Evaluate(bool isVerbose=false,          //!< if true then print a number of statistics
                    cv::Mat    *errmatrix=0,         //!< Return all errors per track(lines) and camera(columns), matrix type CV_64F
                    cv::Mat    *countmatrix=0        //!< Return count of feature per track(lines) and camera(columns), matrix type CV_16U
                   );    
    
    //! Evaluate data. Returns residual error normalized to samples (number of 2d observations)
    double EvaluateN(  unsigned & n //!< number of 2d observations (tracked points)
                    );

    //! Scale data - Not yet implmented!!!
    void Scale( float s);
    
    void SetVerbose(bool v) { verbose=v; }
    bool GetVerbose() const { return verbose;}
private:
  bool verbose;
  dlm::Scene       scene;
  std::vector<mistl::Camera>    camlist;
  std::map<unsigned, dlm::ORepId> lookup;
  std::map<std::pair<unsigned,unsigned>, dlm::FTrkId> lookupFTrk;
  std::map<unsigned, dlm::TSeqId>      transseq_lookup;
//   unsigned seq_no;      //!< kept to enforce continous sequence ids
  //
  // this information is redundant and is used for evaluation
  std::vector<mistl::TrackInfo>  tracklist;
};




}
#endif