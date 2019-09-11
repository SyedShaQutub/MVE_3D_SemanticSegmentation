
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

  
MultiCameraCalibration::MultiCameraCalibration() {
  verbose=false;
//   seq_no=0;
}


MultiCameraCalibration::~MultiCameraCalibration() {
}

unsigned    
MultiCameraCalibration::NCamera() const {

  return scene.IRepS().Range();
}

void
MultiCameraCalibration::Clear()
{
    
}

std::vector<unsigned> 
MultiCameraCalibration::GetSeqIdList() const
{
  std::vector<unsigned> r;
  for(auto iterator = transseq_lookup.begin(); iterator != transseq_lookup.end(); iterator++) {
    // iterator->first = key
    // iterator->second = value
    // Repeat if you also want to iterate through the second map.
    r.push_back( iterator->first );
  }
  std::sort(r.begin(), r.end());
  return r;
}


void
MultiCameraCalibration::OptimizeTransformObjects( unsigned seq, bool opt ) 
{
  MISTL_ASSERT( transseq_lookup.find(seq) != transseq_lookup.end(), "MultiCameraCalibration::OptimizeTransformObjects unknown seq id" );

  dlm::TSeqId const tSeqId = transseq_lookup.find(seq)->second;
  auto it = scene.Get(tSeqId).cbegin();
  
  scene.Get(*it).SetOptimize(opt); 
}


int  MultiCameraCalibration::GetOptimizeCameraFlag( unsigned camid )
{
  dlm::IRep & iRep = scene.Get(dlm::IRepId(camid));

  // assumes the following are created!!
  dlm::CRepId const cRepId = iRep.CRep();     
//   dlm::DRepId const dRepId = iRep.DRep();
  dlm::TSeqId const tSeqId = iRep.TSeq();
  
   
  dlm::TSeq const& tSeq = scene.Get(tSeqId);
  // extrinsics
  auto it = tSeq.cbegin();
  int opt_flags=0;
  
  if(scene.Get(*it) .OptimizePan ())   opt_flags |= mistl::OptimizePan;  
  if(scene.Get(*it) .OptimizeTilt ())   opt_flags |= mistl::OptimizeTilt;//  PLEASE CHANGE THIS CODE TO SOMETHING THAT  DOESNT GENERATE WARNINGS !!!!!!!!!!!!!!!!!
  if(scene.Get(*it) .OptimizeRoll () )   opt_flags |= mistl::OptimizeRoll;
  if(scene.Get(*it) .OptimizeX ())   opt_flags |= mistl::OptimizeX;
  if(scene.Get(*it) .OptimizeY ())   opt_flags |= mistl::OptimizeY;
  if(scene.Get(*it) .OptimizeZ ())   opt_flags |= mistl::OptimizeZ;

  if(scene.Get(cRepId).OptimizeF())   opt_flags |= mistl::OptimizeF;
  if(scene.Get(cRepId).OptimizePPOX())   opt_flags |= mistl::CenterpointShiftX;
  if(scene.Get(cRepId).OptimizePPOY() )   opt_flags |= mistl::CenterpointShiftY;
  
  return opt_flags;
}



void        
MultiCameraCalibration::OptimizeCamera( unsigned camid,  int opt_flags )
{
//   dlm::Scene::IRepStorage const& iRepS = scene.IRepS();

//   dlm::IRepId const iRepId = iRepS.At(camid);
  dlm::IRep & iRep = scene.Get(dlm::IRepId(camid));

  // assumes the following are created!!
  dlm::CRepId const cRepId = iRep.CRep();     
//   dlm::DRepId const dRepId = iRep.DRep();
  dlm::TSeqId const tSeqId = iRep.TSeq();
  
   
  dlm::TSeq const& tSeq = scene.Get(tSeqId);
  // extrinsics
  auto it = tSeq.cbegin();
  
  scene.Get(*it) .OptimizePan ()  =  opt_flags & mistl::OptimizePan;  //  warning C4800: 'int' : forcing value to bool 'true' or 'false' (performance warning)
  scene.Get(*it) .OptimizeTilt ()  =  opt_flags & mistl::OptimizeTilt;//  PLEASE CHANGE THIS CODE TO SOMETHING THAT  DOESNT GENERATE WARNINGS !!!!!!!!!!!!!!!!!
  scene.Get(*it) .OptimizeRoll ()  =  opt_flags & mistl::OptimizeRoll;
  scene.Get(*it) .OptimizeX ()  =  opt_flags & mistl::OptimizeX;
  scene.Get(*it) .OptimizeY ()  =  opt_flags & mistl::OptimizeY;
  scene.Get(*it) .OptimizeZ ()  =  opt_flags & mistl::OptimizeZ;

  scene.Get(cRepId).OptimizeF() = opt_flags & mistl::OptimizeF;
  scene.Get(cRepId).OptimizePPOX()  = opt_flags & mistl::CenterpointShiftX;
  scene.Get(cRepId).OptimizePPOY() = opt_flags & mistl::CenterpointShiftY;
  
  
//       scene.Get(tRepId).SetOptimize(false);

  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
//   dlm::DRep& dRep = scene.Get(dRepId);
// 
//   dRep.K().At(0) = cam.GetK3(); 
//   dRep.K().At(1) = cam.GetK5(); 

}

mistl::Transform3f
MultiCameraCalibration::GetTransform( unsigned seq ) const
{
  MISTL_ASSERT( transseq_lookup.find(seq) != transseq_lookup.end(), "MultiCameraCalibration::GetTransform unknown seq id" );

  dlm::TSeqId const tSeqId = transseq_lookup.find(seq)->second;
  auto it = scene.Get(tSeqId).cbegin();
  
//   std::cout<<"tSeqId - len:" << scene.Get(tSeqId).size() << "\n";
  
  dlm::Transformation const t = scene.Get(*it); //.Inverse(); // only one transform ? --> Assumption here
 
  mistl::Transform3f    mtrans;
  
  dlm::Matrix4d m = t.Matrix();
  mtrans.rot=mistl::Matrix3x3f( m.At(0,0), m.At(0,1), m.At(0,2),
                              m.At(1,0), m.At(1,1), m.At(1,2),
                              m.At(2,0), m.At(2,1), m.At(2,2) );
  mtrans.t = mistl::Vector3f( m.At(0,3), m.At(1,3),m.At(2,3) ) * 0.001f ;

  return mtrans;
}


dlm::Quatd 
MultiCameraCalibration::Orientation(const mistl::Camera        &cam) const
{
  dlm::Matrix3d rotationMatrix;

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  dlm::MF::SetCol(rotationMatrix, 0, dlmVector(cam.H0()) );
  dlm::MF::SetCol(rotationMatrix, 1, dlmVector(cam.V0()) );
  dlm::MF::SetCol(rotationMatrix, 2, dlmVector(cam.A())  );

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  return dlm::QF::FromRotation(rotationMatrix);
}


void        
MultiCameraCalibration::SetCamera( unsigned camid, const mistl::Camera        &cam)
{
//   dlm::Scene::IRepStorage const& iRepS = scene.IRepS();

//   dlm::IRepId const iRepId = iRepS.At(camid);
  dlm::IRep & iRep = scene.Get(dlm::IRepId(camid));

  // assumes the following are created!!
  dlm::CRepId const cRepId = iRep.CRep();     
  dlm::DRepId const dRepId = iRep.DRep();
  dlm::TSeqId const tSeqId = iRep.TSeq();
  
//   dlm::Transformation *tp = & scene.Get(tSeqId)[0];
//   dlm::TRepId const tRepId = scene.Get(tSeqId); // TRep();
   
  dlm::TSeq const& tSeq = scene.Get(tSeqId);
 
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  dlm::Camera intrinsics;

  intrinsics.F  () = 1000.0f * cam.GetF  ();
  intrinsics.PPO() = dlm::Vector2d(1000.0f*cam.GetCenterPointShiftX(), 1000.0f*cam.GetCenterPointShiftY() );   
  scene.Get(cRepId) = intrinsics;
  
  //
  iRep.PixelSize() = dlm::Vector2d( 1000.0f*cam.GetSx(), 1000.0f*cam.GetSy());
  iRep.Size() = dlm::Vector2u( cam.GetNx(), cam.GetNy());
  iRep.Origin() = dlm::IRep::LowerLeft; // should be (??): iRep.Origin() = dlm::IRep::UpperLeft;
  iRep.PixelCenter() = dlm::IRep::Zero;
 
  // extrinsics
  auto it = tSeq.cbegin();
  scene.Get(*it) = dlm::Transformation( Orientation(cam), dlmVector(cam.C()*1000.0f ), true) ;   //cam.Extrinsics();
//   mistl::Matrix3x3f mat = cam.GetPose().rot;
//   mistl::Vector3f tinv = (mat * cam.C() );
//   tinv = tinv  * -1000.0f;
//   scene.Get(*it) = dlm::Transformation( Orientation(cam), dlmVector(tinv ), true) ;   //cam.Extrinsics();

//   std::cout<<"SetCamera "<<camid<<" is-C:"<< cam.C() << " tinv:"<<tinv<<std::endl;
  
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  if (iRep.HasDRep()) {
    dlm::DRep& dRep = scene.Get(dRepId);

    dRep.K().At(0) = cam.GetK1(); 
    dRep.K().At(1) = cam.GetK2(); 
    dRep.K().At(2) = cam.GetK3();

    //  set p / tangential coefficients 
    dRep.P().At(0) = cam.GetP1();
    dRep.P().At(1) = cam.GetP2();
  } else {
      MISTL_ASSERT( false, "SetCamera: no drep found!");
  }


 

}

void
MultiCameraCalibration::GetCamera( unsigned camid,  mistl::Camera        &cam ) 
{
  dlm::IRep & iRep = scene.Get(dlm::IRepId(camid));
   // assumes the following are created!!
  dlm::CRepId const cRepId = iRep.CRep();     
  dlm::DRepId const dRepId = iRep.DRep();
  dlm::TSeqId const tSeqId = iRep.TSeq();
  
  dlm::TSeq const& tSeq = scene.Get(tSeqId);
 
  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  dlm::Camera intrinsics = scene.Get(cRepId);
  // extrinsics
  auto it = tSeq.cbegin();
  
  dlm::Transformation const t = scene.Get(*it);  //dlm::SA::Collapse(scene, iRep.TSeq());  // ???????
  
  dlm::Matrix3d const rotation = dlm::QF::Rotation(t.Rotation());
  dlm::Vector3d const v0 = dlm::MF::GetCol(rotation, 1);
  dlm::Vector3d const a  = dlm::MF::GetCol(rotation, 2);
  
  
  cam.SetOrientation( mistl::Vector3f(a.X(),a.Y(),a.Z()) , mistl::Vector3f( v0.X(), v0.Y(), v0.Z())* -1.0f );
  
//   mistl::Matrix3x3f mat = cam.GetPose().rot.T();
  mistl::Vector3f tinv ( t.Translation().X(), t.Translation().Y(), t.Translation().Z() );
  tinv = tinv* 0.001f;
//   tinv = mat * tinv;
  cam.SetC( tinv );
//   std::cout<<"GetCamera "<<camid<<" C:"<<tinv<<std::endl;
  
  cam.SetF( intrinsics.F  () / 1000.0f );
  cam.SetCenterPointShiftX( intrinsics.PPO().X() / 1000.0f );
  cam.SetCenterPointShiftY( intrinsics.PPO().Y() / 1000.0f );
  cam.SetTarget( iRep.Size().X(),  iRep.Size().Y(), iRep.PixelSize().X()*0.001f, iRep.PixelSize().Y()*0.001f );
  
  if (iRep.HasDRep()) {
        dlm::DRep const& dRep = scene.Get(dRepId);
#ifdef BDEBUG
        if(verbose) 
          std::cout<<"GetCamera k1:"<< dRep.OptimizeK(0) << " k2:" << dRep.OptimizeK(1) << " k3:" << dRep.OptimizeK(2)<< "\n";
#endif
        cam.SetK1( dRep.K().At(0) );
        cam.SetK2( dRep.K().At(1) );
        cam.SetK3( dRep.K().At(2) );

        cam.SetP1( dRep.P().At(0) );
        cam.SetP2( dRep.P().At(1) );
  } else {
    cam.SetK1(0.0f);
    cam.SetK2(0.0f);
    cam.SetK3(0.0f);
    cam.SetP1(0.0f);
    cam.SetP2(0.0f);
  }
        
}

void
MultiCameraCalibration::UpdateCameras() 
{
  for(unsigned i=0; i<camlist.size(); ++i ) {
    GetCamera( i, camlist[i] );
  }
}



void MultiCameraCalibration::EstimateK( unsigned camid, unsigned k_no )
{
  MISTL_ASSERT(k_no<4, "EstimateK: only k1,..,k3 radial distortions supported");
  
    dlm::IRep & iRep = scene.Get(dlm::IRepId(camid));
   // assumes the following are created!!
    dlm::DRepId const dRepId = iRep.DRep();
    
    dlm::DRep & dRep = scene.Get(dRepId);
    
    // we only support radial distortions k1..k3
    for(unsigned k=0; k<3; ++k) {
        bool f = ( k<k_no );
        dRep.OptimizeK(k) = f;
    }

#ifdef BDEBUG
    // debug output
    std::cout<<"EstimateK cam:"<<camid<<" Count:"<< dRep.Count() <<" = ";
    for(unsigned k=0; k<6; ++k) std::cout<< dRep.OptimizeK(k)<<" ";
    std::cout<<"\n";
#endif
}

void MultiCameraCalibration::EstimateTangentialDist( unsigned camid, bool val=true )
{
  
    dlm::IRep & iRep = scene.Get(dlm::IRepId(camid));
   // assumes the following are created!!
    dlm::DRepId const dRepId = iRep.DRep();
    
    dlm::DRep & dRep = scene.Get(dRepId);
    dRep.SetOptimizeP(val);
    
  

#ifdef BDEBUG
    // debug output
    std::cout<<"Estimate-P cam:"<<camid<<" Count:"<< dRep.Count() <<" = ";
    std::cout<<"\n";
#endif
}



void 
MultiCameraCalibration::GetPointList( std::vector<mistl::Vector3f> &plist, std::map<unsigned,unsigned> &index_translate, 
                                      mistl::Transform3f *tr , double scale)  {
  plist.clear();
  
  for(std::map<unsigned, dlm::ORepId>::iterator iter = lookup.begin(); iter != lookup.end(); ++iter) {
//     unsigned k =  iter->first;
    dlm::ORepId oRepId = iter->second;
    if(!scene.IsValid( oRepId )) {
//       std::cout << "GetPointList invalid oRepId!!!!\n";
      continue;
    }
    mistl::Vector3f v;
    
    dlm::ORep& oRep = scene.Get(oRepId);
    v=mistl::Vector3f( oRep.X(), oRep.Y() , oRep.Z() );
    
    v = v*0.001f;
    if(tr) v = tr->Trans(v);
    v = v * scale;
//     std::cout<<k<<" :"<<v<<"\n";
    index_translate[ oRepId.Get() ] = (unsigned int)plist.size();
    plist.push_back(v);
  }
}

void 
MultiCameraCalibration::Get2DPointList( unsigned camid, unsigned seqid, const std::map<unsigned,unsigned> &index_translate,
                                        std::vector<mistl::Vector2f> &plist, std::vector<unsigned> &indexlist ) {
  plist.clear();
  indexlist.clear();

//   return ;
  //
  // 3d point indices are not adjusted, as there might be gaps because of outlier removal!!!
  // !!!!!
  
  
//   dlm::IRepId const iRepId = dlm::IRepId(camid); // camera
  dlm::IRep const& iRep = scene.Get(dlm::IRepId(camid));// camera
  
  // far all 3d points
  for(std::map<unsigned, dlm::ORepId>::iterator iter = lookup.begin(); iter != lookup.end(); ++iter) {
    unsigned pointid =  iter->first;
    dlm::ORepId oRepId = iter->second;
    if(!scene.IsValid( oRepId )) {
  //       std::cout << "GetPointList invalid oRepId!!!!\n";
      continue;
    }
    dlm::FTrkId fTrkId;
    // look up Ftrack for each unique sequence/feature point 
    auto entry = std::make_pair(seqid, pointid); // sequence id
    if (lookupFTrk.find(entry) == lookupFTrk.end()) {
      continue;
    }
    fTrkId = lookupFTrk.find(entry)->second;
    dlm::FTrk  & fTrk = scene.Get(fTrkId);
    
    
    dlm::FSetId fSetId = iRep.FSet();
    dlm::FRepId fRepId;
    
    if (fTrk.GetFRep(fSetId, fRepId))
    {
      dlm::FRep const& fRep = scene.Get(fSetId, fRepId);
      dlm::Vector2d v = fRep;
      if( index_translate.find(pointid) == index_translate.end() ) {
        std::cout << "Get2DPointList : invalid index : "<<pointid<<"\n";
      } else {
        plist.push_back( mistl::Vector2f( v.X() , v.Y() ) );
        unsigned npid = index_translate.find(pointid)->second;
        indexlist.push_back( npid );
      }
    }
    else
    {
      continue;
    }
    
    
  }
}


//
// This method is not yet implmented !!!!!!!!!
//
void MultiCameraCalibration::Scale( float s)
{
  // scale all points
  for(std::map<unsigned, dlm::ORepId>::iterator iter = lookup.begin(); iter != lookup.end(); ++iter) {
//     unsigned k =  iter->first;
    dlm::ORepId oRepId = iter->second;
    if(!scene.IsValid( oRepId )) {
//       std::cout << "GetPointList invalid oRepId!!!!\n";
      continue;
    }
  
//       dlm::ORep& oRep = scene.Get(oRepId);

  }
}


void
MultiCameraCalibration::Store()
{
  if(verbose) std::cout << scene.Get(dlm::IRepId(0)).PixelSize().X() << " " << scene.Get(dlm::IRepId(0)).PixelSize().Y() << std::endl;
  
//   try {
  dlmVoodooIO::Store(
        scene,
        "%02d.cahv",
        "%02d.pnt",
        true);
//   } 
//   catch ( dlm::DomainError const& e) {
//     std::cout << "DomainError: " << e.File() << " " << e.Line() << std::endl;
//   }

    
}




}
