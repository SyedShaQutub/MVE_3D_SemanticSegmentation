
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

double MultiCameraCalibration::EvaluateN(  unsigned & n
//                     ,                       bool isVerbose
                    )
{
    //----------------------------------------------------------------------------
  using namespace dlm;
  //----------------------------------------------------------------------------

  dlm::uint64 samples = 0;

  FloatType cumulativeSquareError = static_cast<FloatType>(0.0);

  for (dlm::uint64 i = 0; i < scene.IRepS().Range(); ++i)
  {
    IRepId const iRepId = IRepId(i);
    
    if (!scene.IsValid(iRepId)) { continue; }
    
    IRep const& iRep = scene.Get(iRepId);               // equivalent of a camera
    FSet const& fSet = scene.Get(iRep.FSet());          // all the 2d feature points of one camera
    
    // loop over all 2d feature points assigned to one camera
    for (dlm::uint64 j = 0; j < fSet.Range(); ++j)
    {
      if (!fSet.Active(j)      ) { continue; }
      if (!fSet.At(j).HasFTrk()) { continue; }
      
      // determine 3d point from Fset and Frep id
      dlm::Vector2d  ipp= dlm::SA::Project(scene, iRep.FSet(), FRepId(j) );
      
      dlm::Vector2d dlmp = iRep.ToImg(ipp);
      dlm::Vector2d const distance = fSet.At(j) - dlmp; //iRep.ToImg(scene.Project(iRep.FSet(), FRepId(j)));
      
// std::cout<<" -> proj of point "<<" dlm:"<< dlmp.X() <<" " << ipp.Y() << " f:" << fSet.At(j).X()  << " " <<fSet.At(j).Y() <<"\n";

      FloatType const squareError = distance.Dot(distance);
      
      cumulativeSquareError += squareError;
      
      ++samples;
    }
  }
  
  FloatType const meanSquareError = cumulativeSquareError / static_cast<FloatType>(samples);
  
  FloatType const rootMeanSquareError = sqrt(meanSquareError);

  n = samples;
  return rootMeanSquareError;
}




unsigned 
MultiCameraCalibration::RemoveOutlier( float max_delta, bool isVerbose)
{
    //----------------------------------------------------------------------------
  using namespace dlm;
  //----------------------------------------------------------------------------

  dlm::uint64 outliers = 0;

  FloatType sq_err = max_delta*max_delta;
  
//   FloatType cumulativeSquareError = static_cast<FloatType>(0.0);

  for (dlm::uint64 i = 0; i < scene.IRepS().Range(); ++i)
  {
    IRepId const iRepId = IRepId(i);
    
    if (!scene.IsValid(iRepId)) { continue; }
    
    IRep const& iRep = scene.Get(iRepId);
    FSet const& fSet = scene.Get(iRep.FSet());
    
    for (dlm::uint64 j = 0; j < fSet.Range(); ++j)
    {
      if (!fSet.Active(j)      ) { continue; }
      if (!fSet.At(j).HasFTrk()) { continue; }
      
      FRep const& fRep = scene.Get(iRep.FSet(), FRepId(j));
      dlm::Vector3d point = dlm::SA::Transform(scene, fRep.FTrk());
    
//     std::cout<<"\nSA P:"<<point.X()<<','<< point.Y()<<','<<point.Z()<<"\t";

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // apply transformations
      if (iRep.HasTSeq())
      {
        point = dlm::SA::Transform(point, scene, iRep.TSeq());
  //     std::cout<<"T:"<<point.X()<<','<< point.Y()<<','<<point.Z()<<"\t";
      }
    
      if(point.Z()<0.0) {
		if (isVerbose) std::cout << " !Depth < 0\n";
        scene.RemoveFTrk(iRep.FSet(), FRepId(j) );
        ++outliers;
        continue;
      }
      
      dlm::Vector2d  ipp= dlm::SA::Project(scene, iRep.FSet(), FRepId(j) );
      dlm::Vector2d dlmp = iRep.ToImg(ipp);
      dlm::Vector2d const distance = fSet.At(j) - dlmp; //iRep.ToImg(scene.Project(iRep.FSet(), FRepId(j)));
      
// std::cout<<" -> proj of point "<<" dlm:"<< dlmp.X() <<" " << ipp.Y() << " f:" << fSet.At(j).X()  << " " <<fSet.At(j).Y() <<"\n";

      FloatType const squareError = distance.Dot(distance);
      
      // remove outlier
      if( squareError > sq_err ) {
//      std::cout<< "squareError > sq_err remove FRepId:"<<j<<"\n";
        scene.RemoveFTrk(iRep.FSet(), FRepId(j) );
        ++outliers;
      }
    }
  }
  
//   FloatType const meanSquareError = cumulativeSquareError / static_cast<FloatType>(outliers);
//   
//   FloatType const rootMeanSquareError = sqrt(meanSquareError);

  return outliers;
}




double 
MultiCameraCalibration::Evaluate(bool isVerbose, cv::Mat *errmatrix , cv::Mat *countmatrix )
{
//   double d=0;
  double dd=0;
  unsigned no=0;
  std::vector<mistl::Vector3f> plist;
  std::map<unsigned,unsigned> index_translate;
  
  std::cout<<"GetPointList\n";
  GetPointList( plist, index_translate) ;
  
  UpdateCameras();
  
  unsigned no_of_seqids = NoOfTransformations();
  // check if errmatrix is non-zero object
  if( (errmatrix)!=0 ) {
   *errmatrix = cv::Mat(no_of_seqids, (int)camlist.size(), CV_64F);
    for (unsigned i = 0; i < no_of_seqids; ++i){
      for (unsigned j = 0; j < camlist.size(); ++j){
        errmatrix->at<double>(i, j) = 0.0;
      }
    }
  }
  // check if countmatrix is non-zero object
  if( (countmatrix)!=0 ) {
   *countmatrix = cv::Mat(no_of_seqids, (int)camlist.size(), CV_16U);
    for (unsigned i = 0; i < no_of_seqids; ++i){
      for (unsigned j = 0; j < camlist.size(); ++j){
        countmatrix->at<unsigned short>(i, j) = 0;
      }
    }
  }

  // build map of seq ids
  std::map<unsigned, unsigned>      seq_lookup;
  std::vector<unsigned> seqlist = GetSeqIdList();
  for(unsigned i=0; i<seqlist.size(); ++i) 
    seq_lookup[seqlist[i]] = i;
        
  std::map<unsigned,mistl::Transform3f> tmap;
  for(std::map<unsigned, dlm::TSeqId>::iterator iter = transseq_lookup.begin(); iter != transseq_lookup.end(); ++iter) {
    unsigned seq= iter->first;
    mistl::Transform3f tr = GetTransform(seq);
    tmap[seq]=tr;
    if(isVerbose) std::cout<<"Transformation "<<seq<<"\t"<<tr.t<<" r:"<<tr.rot<<"\n";
  }
  if(isVerbose)
      std::cout<<"\n====================\n";
  
  // use for statistics
  std::vector<double> camd;
  for(unsigned k=0; k<camlist.size(); ++k) camd.push_back(0.0);
  
  // for all tracks
  for(unsigned i=0; i<tracklist.size(); ++i ) {
    if(isVerbose)
      std::cout<<"Track "<<i<<"\n"; //====================\n";
    
    // use for statistics
    double trackd = 0;
    
    
    for(unsigned j=0; j<tracklist[i].clist.size(); ++j ) {
      auto const& point = tracklist[i].clist.at(j);
      if(point.pointid<0 || point.disabled) continue;
      
      mistl::Vector2f   pp,p(point.ip[0], point.ip[1]);
      const mistl::Camera &cam = camlist.at(tracklist[i].camid);
      
      // !!!! pointid is not valid anymore if oRep were removed, e.g.  by RemoveOutlier
      mistl::Vector3f   P0= (plist.at(point.pointid)) ;    

      // 
      mistl::Vector3f   P= tmap[point.id].Trans(P0) ;
//       float depth= cam.A().Dot(P);

      float x,y,z;
      cam.Project( P,x,y,z );
      pp.X()=x;
      pp.Y()=y;
      if(z<0.0) {
        std::cout<<" !depth "<<z<< " of point:"<<P<<"\n";
//         continue;
      }     
      
      float r;
      
      dlm::uint64 fid = point.fid;
      dlm::FRepId  fRepId(fid);
      dlm::IRep & iRep = scene.Get(dlm::IRepId(tracklist[i].camid));
      dlm::Vector2d  ipp= dlm::SA::Project(scene, iRep.FSet(), fRepId );
      dlm::Vector2d dlmp = iRep.ToImg(ipp);
        
      r= (dlmp-dlm::Vector2d(point.ip[0], point.ip[1])).Magnitude();
      dd += r*r;
      
      //
      // this tests that mistl projection delivers same coordinate than DLM
      // this is for debugging and might be removed in production code!!!!
      //
      double mag,diff;
      mag=pp.Magnitude();
      diff=(pp-mistl::Vector2f(dlmp.X(),dlmp.Y())).Magnitude();
      // if this fires then something is wrong!!!!
      if( diff > mag*1e-3) {
        std::cout<<" -> proj of point cam:"<<tracklist[i].camid<<" "<<P<<"mistl:"<<pp<<" dlm:"<<mistl::Vector2f(dlmp.X(),dlmp.Y())<<"\n";
        if (iRep.HasDRep())
          {
            dlm::DRepId const dRepId = iRep.DRep();
            dlm::DRep const& dRep = scene.Get(dRepId);
            std::cout<< "DRep k:("; 
            for(unsigned k=0; k<3; ++k) {
              std::cout<< dRep.K().At(k);
              if(k!=2) std::cout<<",";
            }
            std::cout<< ") p1:"<< dRep.P().At(0) <<" p2:"<< dRep.P().At(1) <<"\n";
            cam.Info(); std::cout<< "\n";
          }
      }
      
      trackd += r*r;
      camd.at(tracklist[i].camid) += r*r;
      
     
      if( (errmatrix)!=0 ) {
        unsigned lin = seq_lookup.find(tracklist[i].seq)->second; 
        errmatrix->at<double>(lin, tracklist[i].camid) += r*r;
        if( (countmatrix)!=0 ) countmatrix->at<unsigned short>(lin, tracklist[i].camid) ++;
      }
      
      ++no;
    }
    
    
    if(isVerbose)
      std::cout<<" error:" << sqrt(trackd) << "\n";
  } // for all tracks
  
  
  if(isVerbose) {
    std::cout<<"---------- Per camera:\n";
    for(unsigned k=0; k<camlist.size(); ++k) std::cout<<k<< "\td:"<< sqrt(camd.at(k))<<"\n";
    double nd;
    if(no) nd=dd/no;
    else nd=dd;
    nd=sqrt(nd);
    std::cout<<"  total:"<<sqrt(dd)<<"\t norm:"<< nd <<" n:"<<no<<"\n";
  }
//   std::cout<<"\ndlm-eval:"<< sqrt(dd)<<"\n";

  
  
  return sqrt(dd);
}




#ifdef notdef
unsigned 
MultiCameraCalibration::RemoveOutlierOld( float max_delta, bool isVerbose)
{
  unsigned outliers=0;
  std::vector<mistl::Vector3f> plist;
  GetPointList( plist ) ;
  
  std::map<unsigned,mistl::Transform3f> tmap;
  for(std::map<unsigned, dlm::TSeqId>::iterator iter = transseq_lookup.begin(); iter != transseq_lookup.end(); ++iter) {
    unsigned seq= iter->first;
    mistl::Transform3f tr = GetTransform(seq);
    tmap[seq]=tr;
    std::cout<<"Transformation "<<seq<<"\t"<<tr.t<<" r:"<<tr.rot<<"\n";
  }
  if(isVerbose)
      std::cout<<"\nRemoveOutlier maxerr:"<<max_delta<<" ====================\n";
  
  
  for(unsigned i=0; i<tracklist.size(); ++i ) {
    if(isVerbose)
      std::cout<<"Track "<<i; //<<"\n====================\n";
   
    
    for(unsigned j=0; j<tracklist[i].clist.size(); ++j ) {
      auto & point = tracklist[i].clist.at(j);
      if(point.pointid<0 || point.disabled) continue;
      
      mistl::Vector2f   pp,p(point.ip[0], point.ip[1]);
      const mistl::Camera &cam = camlist.at(tracklist[i].camid);
      mistl::Vector3f   P0= (plist.at(point.pointid)) ;
      
      // 
      mistl::Vector3f   P= tmap[point.id].Trans(P0) ;
      float depth= cam.A().Dot(P);

      cam.Project( P,pp );
   
      dlm::FRepId  fRepId(point.fid);
      dlm::IRep & iRep = scene.Get(dlm::IRepId(tracklist[i].camid));
      dlm::Vector2d  ipp= dlm::SA::Project(scene, iRep.FSet(), fRepId );
      dlm::Vector2d dlmp = iRep.ToImg(ipp);
        
      float r= (dlmp-dlm::Vector2d(point.ip[0], point.ip[1])).Magnitude();
      
      
      if( r > max_delta  || depth<0.0) {
        // remove
        point.disabled = true;
//         scene.Delete(iRep.FSet(), fRepId); //FRepId(j));
        
        
//         FRep& fRep = scene.Get( fRepId); 
//          FTrkId const old = fRep.FTrk();
// 
//         scene.Get(old).RemoveFRep(fSetId);
        
          // this removes track (rRep + fTrk) 
          // it also removes oRep if no reference to it exists!!!!!
          scene.RemoveFTrk(iRep.FSet(), fRepId);
     
        
        ++outliers;

      }
      
//       if( (pp-mistl::Vector2f(dlmp.X(),dlmp.Y())).Magnitude()> 1e-3) 
//         std::cout<<"\n!proj of point "<<P<<"mistl:"<<pp<<" dlm:"<<mistl::Vector2f(dlmp.X(),dlmp.Y())<<"\n";
      
    }
    if(isVerbose)
      std::cout<<"Outliers:" << outliers << "\n";
  }
  return outliers;

}
#endif


float 
MultiCameraCalibration::Solve()
{
      dlm::Optimizer optimizer;
  
      optimizer.Pack(scene);
      optimizer.Optimize();
      optimizer.Unpack(scene);
      
  return 0;
}






}
