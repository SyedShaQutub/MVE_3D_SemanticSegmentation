
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



#include        "mistl/Vector3.h"
#include        "mistl/Camera.h"
#include        <stdlib.h>
#include <stdio.h>

#include "../../dlm/scene/SceneAssistant.inline.h"

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/ReadCoordinateFile.h"
#include        "mistl/MultiCameraCalibration.h"
#include "dlm/scene/Scene.h"
#include "dlm/optimizer/Optimizer.h"

#ifdef ANDROID
#include "dlmVoodooIO/dlmVoodooIO.h"
#else
#ifdef __linux__
#include "dlmVoodooIO/dlmVoodooIO.h"
#else
#include "dlmVoodooIO/dlmVoodooIO.h"
#endif
#endif


namespace mistl {



void        
MultiCameraCalibration::AddCamera( const mistl::Camera        &cam, bool isfixed )
{
  unsigned  camid=NCamera();
  
  // dlm cameras is formed by: ImageRepresentation (IRep) is used with references to: Camera (CRep), Distortion(DRep) 
  // and a transformation sequence (tSeq) with transformations (tRep)
  
  // central object is the IRep all other objects are linked to it
  dlm::IRepId const iRepId = scene.CreateIRep();
  if(verbose) std::cout<<" AddCamera("<<camid<<"):"<< iRepId.Get() <<"\n";
  
    //------------------------------------------------------------------------------

  dlm::CRepId const cRepId = scene.CreateCRep();        // camera rep, i.e. intrinsics
  dlm::DRepId const dRepId = scene.CreateDRep();        // distortion
  dlm::TSeqId const tSeqId = scene.CreateTSeq();        // transformation sequence - will have one object
  dlm::TRepId const tRepId = scene.CreateTRep();        // transformation object that will take the camera pose
  dlm::FSetId const fSetId = scene.CreateFSet();        // all (2d) feature points will be added into this container

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  scene.Assign(iRepId, cRepId);
  scene.Assign(iRepId, dRepId);
  scene.Assign(iRepId, tSeqId);
  scene.Assign(iRepId, fSetId);

  scene.Append(tSeqId, tRepId);

 
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  dlm::DRep& dRep = scene.Get(dRepId);

//   dRep.K().At(0) = cam.GetK3(); //camera.RadialDistortion().At(0);
//   dRep.K().At(1) = cam.GetK5(); //camera.RadialDistortion().At(1);

  if( isfixed ) {
      scene.Get(tRepId).SetOptimize(false);
  }
  
   // set defaults
  dRep.SetOptimize(false);
 
//   dRep.OptimizeK(0) = true;
//   dRep.OptimizeK(1) = true;

  camlist.push_back(cam);
  
  SetCamera(NCamera()-1, cam);
}




void        
MultiCameraCalibration::AddTrack( unsigned camid, unsigned seq, const std::vector<mistl::CoordinateListEntry> &clist, bool optimze_points )
{
  // make sure continous seq ids
//   MISTL_ASSERT( seq<=seq_no, "MultiCameraCalibration::AddTrack: seq id must be continous");
  

  
  if(verbose) std::cout<<"AddTrack cam:"<<camid<<  "  seq:"<<seq << " opt:"<<optimze_points <<"\n";
  
  dlm::IRep const& iRep = scene.Get(dlm::IRepId(camid));
  dlm::TSeqId tSeqId;
  
  
  // make a copy of the input data
  // this is redundant and might be only usefull for debugging. 
  mistl::TrackInfo      d;
  d.clist = clist;
  d.camid = camid;
  d.seq = seq;
  
  // make sure there is a transformation sequence for each new frame number (seq)
  //   the transfomation relates to the object (3D) points
  if (transseq_lookup.find(seq) == transseq_lookup.end()) {
    // create transformation sequence
    tSeqId = scene.CreateTSeq();
    // add one transformation
    dlm::TRepId tRepId = scene.CreateTRep();
    scene.Append( tSeqId, tRepId);
    
    if(verbose) std::cout<<"\tAddTrack new seq:"<<seq  <<"\n";
//        if(optimze_points) 
//          scene.Get(tRepId).SetOptimize(false);
//        else
         scene.Get(tRepId).SetOptimize(true);
       scene.Get(tRepId).Invert() = true;
       
       transseq_lookup[seq] = tSeqId;
  } else {
       tSeqId = transseq_lookup.find(seq)->second;
  }
       
  //
  // for all (2D) points in track list
  int auto_pid = 0;
  
  for(unsigned i=0; i< clist.size(); ++i) {
    auto const& point = clist.at(i);
    int pid = point.pointid;
    
    // skip if not assigned to a object (3D) point
    if (pid < 0) { 
      pid = auto_pid++;
//       continue; 
      
    }
    
//     std::cout<<pid<<" "; for(unsigned k=0; k<3; ++k) std::cout<< " "<<point.P[k];
//     std::cout<<"\n";
     
     dlm::ORepId oRepId;
     // create new object point if not already in lookup table
     // the object points will be set by the value from the coordinate list (e.g. from chart calibration)
     if (lookup.find(pid) == lookup.end()) {
       
       oRepId = scene.CreateORep();             
       
       lookup[pid] = oRepId;
       
       dlm::ORep& oRep = scene.Get(oRepId);
       
       oRep.X() = 1000.0 * point.P[0];         // set coordinates in [mm] 
       oRep.Y() = 1000.0 * point.P[1];
       oRep.Z() = 1000.0 * point.P[2];
       
       oRep.SetOptimize(optimze_points);         // disable optimisation by default: indented for chart or known landmark points
//        oRep.OptimizeZ() = false;
    } else {
      oRepId = lookup.find(pid)->second;
    }
    
    // generate key: sequence(=frame)+pointid
    auto entry = std::make_pair(seq, pid); // sequence id
    
    // the FTrk 'track' in DLM is the basic vehicle to expand from and links object (3d) points with transformations
    // here we make sure we have a unique FTrk for each camera and sequence. 
    dlm::FTrkId fTrkId;
    // look up Ftrack for each unique sequence/feature point and create track if not found
    if (lookupFTrk.find(entry) == lookupFTrk.end()) {
      // create track and assign object point and transfomation sequence
      fTrkId = scene.CreateFTrk();
      scene.Assign(fTrkId, oRepId);
      scene.Assign(fTrkId, tSeqId);
      lookupFTrk[entry] = fTrkId;
    } else {
      fTrkId = lookupFTrk.find(entry)->second;
    }
    
    // create feature point and link it up
    dlm::FRepId /*const*/ fRepId = scene.CreateFRep(iRep.FSet());
    scene.Assign(iRep.FSet(), fRepId, fTrkId);
    
    // store fRepId in clist copy - for debugging!!!!!
    auto /*const*/& dp = d.clist.at(i);
    dp.fid = fRepId.Get();
    
    // #########################
//     float ny=(float)camlist.at(camid).GetNy();
    // copy value
    dlm::Vector2d v = dlm::Vector2d(point.ip[0], /*ny-*/point.ip[1]);
    scene.Get(iRep.FSet(), fRepId) = v;
  }
  // make a copy of the input data
  // this is redundant and might be only usefull for debugging. 
  tracklist.push_back( d );
}

void        
MultiCameraCalibration::GetTrack( unsigned camid,  //! camera id
                          unsigned seq, //!< frame/sequence id
                          std::vector<mistl::CoordinateListEntry> &clist
                        )
{
  clist.clear();
  // for all tracks
  for(unsigned i=0; i<tracklist.size(); ++i ) {
    if(verbose)
      std::cout<<"Get Track "<<i<<"\n"; //====================\n";
   
    if(tracklist[i].camid==camid && tracklist[i].seq==seq) {
      clist=tracklist[i].clist;
      return;
    }
  }
}



}
