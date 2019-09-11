
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
#include <fstream>

#include <string.h>
#include        "mistl/Error.h"
#include        "mistl/RunCalibration.h"
#include        "mistl/MultiCamTracking.h"
#include "mistl/WriteOBJ.h"

#ifdef ANDROID
#include "mistl/to_string.h"
#else
#define		to_string	std::to_string
#endif


namespace mistl {


 

  
const cv::Scalar hiercolortable[] = {
    cv::Scalar(255,64,64),
    cv::Scalar(0,64,255),
    cv::Scalar(0,255,64),
    cv::Scalar(255,127,64),
    cv::Scalar(127,255,127)
};


RunCalibration::RunCalibration() {
  SetDefaults();
  point_id_offset=0;
  histogram3d.SetSize( 1,1,3 );
  histogram2d.SetSize( 3,3,1 );
}


void RunCalibration::SetDefaults()
{
  verbose=false;
  seq_no=0;
  outlier_removal=true;
  estimate_dist=2;
  outlier_max_err=5.0f;
  outlier_max_err2=3.0f;
  visual_feedback = true;
  create_coord_files = false;
  pointdistance= 4.0f;
  residualref= 1.6f;
  residualoffset= 1.0;
  minimum_n= 100;
  lambda = 0.2f;
  reprojectionthr = 1.6f;
  rotation_preference=true;
}


RunCalibration::~RunCalibration() {
}



void RunCalibration::Store( cv::FileStorage &fs )
{      
  
  MISTL_ASSERT( fs.isOpened(), "RunCalibration::Store: error opening parameter file for writing");


  fs << "visual_feedback" << static_cast<int>(visual_feedback);
  fs << "create_coord_files" << static_cast<int>(create_coord_files);
  fs << "outlier_removal" << static_cast<int>(outlier_removal);
  fs << "verbose" << static_cast<int>(verbose);
  fs << "estimate_dist" << static_cast<int>(estimate_dist);
  fs << "triangulationinit" << static_cast<int>(triangulationinit);
  fs << "outlier_max_err" << outlier_max_err;
   fs << "outlier_max_err2" << outlier_max_err2;
   
   fs << "residualref" << residualref;
  fs << "residualoffset" <<  residualoffset;
  fs << "minimum_n" <<  minimum_n;
  fs << "lambda" <<  lambda;
  fs << "reprojectionthr" <<  reprojectionthr;
 fs << "pointdistance" << pointdistance;
  fs << "MultiCamTracking" << "{";
     mtrack.Store(fs);
  fs << "}";
//   fs << "" << ;
}

//! Read parameters from file
void RunCalibration::ReStore( cv::FileNode &tm )
{
  cv::FileNodeIterator it;
  cv::FileNode node;
  SetDefaults();
  if(tm.empty()) return;
  int v;
  v = tm["visual_feedback"]; visual_feedback= v!=0;
  v = tm["create_coord_files"]; create_coord_files= v!=0;
  v = tm["outlier_removal"]; outlier_removal= v!=0;
  v = tm["verbose"];  verbose= v!=0;
  v = tm["triangulationinit"];  triangulationinit= v!=0;
  v = tm["estimate_dist"]; estimate_dist= static_cast<unsigned>(v);
  outlier_max_err = tm["outlier_max_err"]  ;
  outlier_max_err2 = tm["outlier_max_err2"]  ;
  pointdistance = tm["pointdistance"]  ;
  
  // solving strategies
  if(tm["rotation_preference"].type()==cv::FileNode::INT) {
    v = tm["rotation_preference"];
//     std::cout<<"rotation_preference:"<<v<<"\n";
    rotation_preference=v>0;}

    if(tm["minimum_n"].type()==cv::FileNode::INT) {v = tm["minimum_n"]; minimum_n=static_cast<unsigned>(v);}
    if(tm["residualref"].type()==cv::FileNode::FLOAT) {residualref = tm["residualref"]; }
    if(tm["residualoffset"].type()==cv::FileNode::FLOAT) {residualoffset = tm["residualoffset"]; }
    if(tm["lambda"].type()==cv::FileNode::FLOAT) { lambda = tm["lambda"]; }
    if(tm["reprojectionthr"].type()==cv::FileNode::FLOAT) { reprojectionthr = tm["reprojectionthr"]; }

  node=tm["est_pos"];
  if(!node.empty() && node.type() == cv::FileNode::SEQ) {
    for(it = node.begin(); it!=node.end(); it++) {
        int val=*it;

        mistl::OptimizeFlags value = (val==1) ? mistl::OptimizeFlags::OptimizePosition : mistl::OptimizeFlags::NoParameter;
        est_pos.push_back(value);
    }
  }
  else {
	  for(int cameras=0; cameras<3; cameras++)
		  est_pos.push_back(mistl::OptimizeFlags::NoParameter);
  }

	node=tm["est_rot"];
	if(!node.empty() && node.type() == cv::FileNode::SEQ) {
		for(it = node.begin(); it!=node.end(); it++) {
			int val=*it;

			mistl::OptimizeFlags value = (val==1) ? mistl::OptimizeFlags::OptimizeRotationEuler : mistl::OptimizeFlags::NoParameter;
			est_rot.push_back(value);
		}
	}
	else {
		for(int cameras=0; cameras<3; cameras++)
			est_rot.push_back(mistl::OptimizeFlags::NoParameter);
	}
       
	node=tm["est_cps"];
	if(!node.empty() && node.type() == cv::FileNode::SEQ) {
		for(it = node.begin(); it!=node.end(); it++) {
			int val=*it;

			mistl::OptimizeFlags value = (val==1) ? mistl::OptimizeCameraFlags::CenterpointShift : mistl::OptimizeFlags::NoParameter;
			est_cps.push_back(value);
		}
	}
	else {
		for(int cameras=0; cameras<3; cameras++)
			est_cps.push_back(mistl::OptimizeFlags::NoParameter);
	}

	node=tm["est_f"];
	if(!node.empty() && node.type() == cv::FileNode::SEQ) {
		for(it = node.begin(); it!=node.end(); it++) {
			int val=*it;

			mistl::OptimizeFlags value = (val==1) ? mistl::OptimizeCameraFlags::OptimizeF : mistl::OptimizeFlags::NoParameter;
			est_f.push_back(value);
		}
	}
	else {
		for(int cameras=0; cameras<3; cameras++)
			est_f.push_back(mistl::OptimizeFlags::NoParameter);
	}

	node=tm["est_k"];
	if(!node.empty() && node.type() == cv::FileNode::SEQ) {
		for(it = node.begin(); it!=node.end(); it++) {
			int val=*it;
			est_k.push_back(val);
		}
	}
	else {
		for(int cameras=0; cameras<3; cameras++)
			est_k.push_back( 0 );
	}

        node=tm["est_p"];
        if(!node.empty() && node.type() == cv::FileNode::SEQ) {
                for(it = node.begin(); it!=node.end(); it++) {
                        int val=*it;
                        est_p.push_back(val!=0);
                }
        }
        else {
                for(int cameras=0; cameras<3; cameras++)
                        est_p.push_back( false );
        }
        
        // boundaries
        node=tm["max_rot"];
        std::cout<<"max_rot:";
        if(!node.empty() && node.type() == cv::FileNode::SEQ) {
                for(it = node.begin(); it!=node.end(); it++) {
                        float val=*it;
                        max_rot.push_back(val);
                      std::cout<<" "<<val;
                }
        } else {
                for(int cameras=0; cameras<3; cameras++) {
                        max_rot.push_back(1e38);
                        std::cout<<" "<<1e38;
                }
        }
        std::cout<<"\n";
 
  cv::FileNode trnode = tm["MultiCamTracking" ];
  if(trnode.empty()) mtrack = MultiCamTrackingFeatures();
  else mtrack.ReStore(trnode);
}


void RunCalibration:: SetInitialCameraParameterList( const std::vector<mistl::Camera> & incamlist ) 
{
  camlist = incamlist; // this is the 'working copy' updated by the calibration later
}


void RunCalibration:: Init(  )
{
  
  // this should be all done externally
  point_id_offset=0;
  q.quality=0.0;
  q.confidence=0.0;
}


// Redundant method just kept in for comaptiblity!
void RunCalibration::GetInitialCameras( std::vector<mistl::Camera> & outcamlist ) const
{  
   GetCameras(outcamlist);
}


//! Get copy of  camera objects
void RunCalibration::GetCameras( std::vector<mistl::Camera> & outcamlist ) const
{

  outcamlist = camlist;
}

void RunCalibration::NormScaleCameras( double scale, mistl::Transform3f *tr_p )
{
  mistl::Transform3f tr,inv;
  mistl::Vector3f	cref;

  tr = camlist.at(mtrack.refcam).GetPose();
  cref=tr.t;
  tr.t = mistl::Vector3f(.0f,.0f,.0f);
  inv=tr.Inverse();

#ifdef DEBUG  
  std::cout<<"NormScaleCameras ref.rot:"<<   tr.rot<<"\n"<<"inv:"<<inv.rot<<"\n";
#endif  
 
  for(unsigned i=0; i<camlist.size(); ++i) {
    mistl::Transform3f pose,npose;

    pose=camlist.at(i).GetPose();

    pose.t = pose.t - cref;
#ifdef BDEBUG
  std::cout<<i<< "\t"<<   pose.rot<<"\n";
  camlist.at(i).Info();
#endif  
    if(i==mtrack.refcam)
      npose.rot = pose.rot.Identity();
    else {
    	Mul( pose.rot, inv.rot, npose.rot);
    	npose.t = inv.Trans(pose.t);
    	npose.t = npose.t * scale;
    }

//       npose.rot = pose.rot * inv.rot;

#ifdef BDEBUG  
  std::cout<<"n:\t"<<   npose.rot<<"\n";
#endif
    camlist.at(i).SetPose(npose);
#ifdef DEBUG  
  camlist.at(i).Info();
  std::cout<<"\n------\n";
#endif
  }
  if(tr_p) *tr_p = inv; 
}


static const char *msg[] = { "Success", "Failed", "Not enough 2D features", 
  "In Calibration", "Out of Calibration"
};


const char * RunCalibration::CalibrationCode (CalibrationCodeFlag n) const
{
   return msg[n];
}

CalibrationCodeFlag
RunCalibration::ReprojectionTest( )
{
 // fast reprojection test
      unsigned nreptest;
      float reperr = mtrack.ReprojectionTest ( &nreptest );
      if(DebugMode()) {
        std::cout << "ReprojectionTest - " << reperr << " n:"<<nreptest << "  Threshold:"<< reprojectionthr <<"\n";
      }
      
      if( reperr > reprojectionthr )
        return mistl::Failed;
      
      return mistl::Success;
}


//! Calibrate camera from one set of images. Returns true if calibration was succesfull
CalibrationCodeFlag  
RunCalibration::Tracking( const std::vector<cv::Mat>       & framelist, //!< list of images
                          unsigned      seq_id
                   )
{
  seq_no = seq_id;
  if (DebugMode()) std::cout << "\n\n###############\nCamera tracking frame:"<< seq_no <<" set-size:" << framelist.size();
  //std::cout << " ccfiles:" << create_coord_files<< "\n" ;
#ifdef DEBUG
  if(DebugMode()) {
    for(unsigned i=0; i<framelist.size(); ++i) {
      std::cout << " cam-"<<i<<" "<< framelist.at(i).size();
    }
    std::cout << "\n";
  }
#endif

  // copy initial cameras
  mtrack.camlist = this->camlist;
  
  mtrack.PrepareTracking( framelist );
  mtrack.InfraMatching(  );


  mtrack.Info();
  
  if(DebugMode() && visual_feedback) {
    cv::Mat outFrame;
    cv::cvtColor( framelist.at(mtrack.refcam) , outFrame ,CV_GRAY2RGB);
      visrefimage = outFrame;
      cv::vector<int> compression_params;
      compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
      compression_params.push_back(100);

      // mark features in ref image				// not needed ?? check MarkMatches
      for (unsigned int i = 0; i < mtrack.trackdata.plist.size(); i++) {
            int hier=mtrack.trackdata.metalist.at(i).hier;
            if(hier<0 || hier>4) {
            // ouch!!
              continue;
            }
            cv::Scalar col=hiercolortable[hier];
            cv::circle( visrefimage, mtrack.trackdata.plist.at(i), 2, col, 1);
      }
      // write out grayscale images 
      for(unsigned  j=0; j<framelist.size(); ++j ) {
        // std::cout<<"vis-track "<<j<<std::endl;
        // compensate image re-size
        if(j==mtrack.refcam) continue;
        float wx,wy;
        if(mtrack.tracksizex>0.f) {
          wx=mtrack.tracksizex;
          wy=mtrack.tracksizey;
        } else {
          wx=(float)framelist.at(mtrack.refcam).size().width;
          wy=(float)framelist.at(mtrack.refcam).size().height;
        }
//         float scaleX = (float)framelist[j].size().width / wx;
//         float scaleY = (float)framelist[j].size().height / wy;
        float scaleX = wx/(float)visrefimage.size().width  ;
        float scaleY = wy/(float)visrefimage.size().height ;
        mtrack.tracker.MarkMatches(  visrefimage, mtrack.Track(mtrack.refcam,0).plist, mtrack.Track(j,0).plist, mtrack.Track(j,0).status,
//                                      1.0f,1.0f );
                                     scaleX, scaleY );
		std::string   fn = DebugLogFile("track_" + to_string(seq_no) + "_" + to_string(j) + ".jpg");
        cv::imwrite( fn.c_str(), framelist[j], compression_params);
      }


      // visualize 2D-Histogram bins:
      if(mtrack.useHistFeatureMatching && mtrack.bins2D > 0) {
    	  for(int i=1; i<mtrack.bins2D; i++) {
    		  float coord = (visrefimage.size().width / mtrack.bins2D) *i;
    		  cv::line(visrefimage, cv::Point2f(coord, 0), cv::Point2f(coord, visrefimage.size().height), cv::Scalar(0, 255, 0), 2);
    	  }
    	  for(int i=1; i<mtrack.bins2D; i++) {
    	   	  float coord = (visrefimage.size().height / mtrack.bins2D) *i;
    		  cv::line(visrefimage, cv::Point2f(0, coord), cv::Point2f(visrefimage.size().width, coord), cv::Scalar(0, 255, 0), 2);
		  }
      }

      // write images with markers
	  std::string   fn = DebugLogFile("track_" + to_string(seq_no) + "_" + to_string(mtrack.refcam) + ".jpg");
	  cv::imwrite( fn.c_str(), visrefimage, compression_params);

  }
  
  if(verbose) std::cout<<"PrunePoints().."<<std::endl;
  mtrack.PrunePoints();
  
  //
  // compute quality
  mtrack.ComputeQuality( q.trackquality );
  if(verbose){
    q.trackquality.Info();
    std::cout << "GeneratePoints().." << std::endl;
  }

#ifdef BDEBUG   
  if(create_coord_files) {
	    std::string   fn = DebugLogFile( "track_" + to_string(seq_no) + "_cam%02d.coord"  );
		mtrack.WriteCoordinateFile( fn.c_str(), true );
  }
#endif
  
  mtrack.GeneratePoints(triangulationinit, pointdistance);

  if(verbose) {
    std::cout<<"Points after GeneratePoints():\n";
    mtrack.Info();
  }
  
  if(create_coord_files) {
	  std::string   fn = DebugLogFile("genpoints_" + to_string(seq_no) + "_cam%02d.coord");
	mtrack.WriteCoordinateFile( fn.c_str(), true, point_id_offset );
  }
  
  point_id_offset += (unsigned)mtrack.P3list.size();
  
  if( q.trackquality.GetQuality() <0.001 || q.trackquality.GetConfidence() <0.001 ) 
    q.trackrc = mistl::NotEnoughFeatures;
  else
    q.trackrc =  mistl::Success;
  return q.trackrc;
}


  //! Calibrate camera from tracking results. Returns true if calibration was succesfull
CalibrationCodeFlag  
RunCalibration::Calibrate(   )
{
  q.quality=0.0;
  q.confidence=0.0;
  if (DebugMode())
	  DumpData("Init",false,false);
  BuildData();
  if (DebugMode())
	  DumpData("Build", true, true);
  CalibrationCodeFlag r;
  r=CameraCalibration();
  if (DebugMode())
	  DumpData("Calib", true, true);
  return mistl::Success;       // DANIEL: should this really always return success or "r" instead?
}


CalibrationCodeFlag  RunCalibration::BuildData()
{
  // reset quality stats
  q.quality=0.0;
  q.confidence=0.0;
  
  q.initialmatch=0.0f;   //!< res.error after point triangulation
  q.initial_n=0;
  q.p3dmatch=0.0f;       //!< res.error after point BA
  q.p3d_n=0;
  q.finmatch=0.0f;          //!< final BA residual error
  q.fin_n=0;
  
  if(DebugMode() || verbose) std::cout << "\n\n\nSTART build data \n" << std::endl;

  // propagate verbose flag 
  cal.SetVerbose( verbose);
  
  // copy data into calibration object
  for(unsigned camid=0; camid<camlist.size(); ++camid) {
    if (DebugMode()) std::cout << "GenerateCoordinateList(" << camid << ").." << std::endl;
    std::vector<mistl::CoordinateListEntry> clist;
    mtrack.GenerateCoordinateList( camid, clist );
    cal.AddCamera( camlist.at(camid), false /*camid==0*/ );
    
    if(DebugMode() || verbose) 
      std::cout << "\nCamera:"<<camid<<" has "<< mistl::MaxSequence(clist)+1<<" tracks\n";
    
    for(int i=0; i<= mistl::MaxSequence(clist); ++i) {
      std::vector<mistl::CoordinateListEntry> seqlist = FilterSequence( clist, i);
      if(seqlist.size()>0) {
        cal.AddTrack( camid, i, seqlist, true );
#ifdef BDEBUG
        if(DebugMode() || verbose)
          std::cout << "Added track seq:"<<i<<"\n"; 
#endif
      }
    }
  }
  
  
  // ######################################
  // first phase
  //

  // fix all transformation object
  std::vector<unsigned> olist = cal.GetSeqIdList();
  for(unsigned i=0; i<olist.size() ; ++i)
    cal.OptimizeTransformObjects(olist.at(i),false);


  // fix all cameras
  for(unsigned i=0; i<cal.NCamera(); ++i) {
    if(DebugMode() || verbose) std::cout<<"fix camera "<<i<<"\n";
    cal.OptimizeCamera( i, mistl::NoParameter );
    cal.EstimateK( i, 0 );
  }


  unsigned n;
  double d,err;


  cal.UpdateCameras();
  err=cal.EvaluateN(n);
  
  q.initialmatch=err;
  q.initial_n=n;
  
  if(DebugMode() || verbose) std::cout<<"initial Eval ("<<n<<"):"<<err<<"\n";


  d=cal.Solve();

  cal.UpdateCameras();
  err=cal.EvaluateN(n );
  q.p3dmatch=err;
  q.p3d_n=n;

#ifdef BDEBUG
  if(DebugMode()) {
    std::string   fn = DebugLogFile("points_" + to_string(seq_no) + ".coord");
    std::ofstream fo( fn );

    std::vector<mistl::Vector3f> plist;
    std::map<unsigned,unsigned> index_translate;

    cal.GetPointList(plist, index_translate);
    fo<<"#Coordinate list file\n#frame-id       Px Py Pz        ix iy   label\n";
    for(unsigned i=0; i<plist.size(); ++i)
      fo<<seq_no << " " << plist[i]<< " 0 0 "<< to_string(i)<<"\n";
  }
#endif 

  if(DebugMode() || verbose) std::cout<<"Exit BuildData - Eval ("<<n<<"):"<<err<<"\n";


  return mistl::Success;
}

CalibrationCodeFlag  
RunCalibration::CameraCalibration ()
{
  // ######################################
  // second phase
  //
  unsigned n;  
  double d,err;
  
  // enable estimation of intrinsics of all cameras
  
  for(unsigned i=0; i<cal.NCamera(); ++i) {
    int opt_flag;
    
    if(rotation_preference && est_rot.at(i))
      opt_flag= est_rot.at(i);
    else
      opt_flag= est_rot.at(i)   |
          est_f.at(i)           |
          est_cps.at(i) |
          est_pos.at(i);
       
    if (DebugMode()) std::cout<<"rotation_preference:"<<rotation_preference <<" opt_flag:"<< opt_flag<<"\n";
    
    cal.OptimizeCamera( i,opt_flag );
    if(!rotation_preference) {
      cal.EstimateK( i, est_k.at(i) );
      cal.EstimateTangentialDist( i, est_p.at(i) );
    }
  }

  if (DebugMode()) std::cout<<"#############\n::::: 2nd solve\n";
  
  if(outlier_removal) {
//    unsigned no = cal.RemoveOutlier( outlier_max_err, true );
	unsigned no = cal.RemoveOutlier(outlier_max_err, DebugMode());

    
    if(DebugMode() || verbose) std::cout<<"Outlier:" << no <<"\n";
  }
    
  d=cal.Solve();
  
  if (DebugMode()) std::cout << "-- update\n";
  
  cal.UpdateCameras();
  err=cal.EvaluateN(n);
  if (DebugMode()) std::cout << " Eval (" << n << "):" << err << "\n";
  
  // tigher outlier removal
  if(outlier_removal) {
    //unsigned no = cal.RemoveOutlier( outlier_max_err2, true );
	unsigned no = cal.RemoveOutlier(outlier_max_err2, DebugMode());
	if (DebugMode() || verbose) std::cout << "Outlier:" << no << "\n";
  }
  
  if(outlier_removal | rotation_preference) {
    
    // enable estimation of intrinsics of all cameras
  
    for(unsigned i=0; i<cal.NCamera(); ++i) {
      cal.OptimizeCamera( i,
        est_rot.at(i)     |
            est_f.at(i)           |
            est_cps.at(i) |
            est_pos.at(i)
      );
      cal.EstimateK( i, est_k.at(i) );
      cal.EstimateTangentialDist( i, est_p.at(i) );
    }

    d=cal.Solve();
    if(DebugMode() || verbose) std::cout<<"-- update\n";
    err=cal.EvaluateN(n);
    if(DebugMode() || verbose) std::cout<<" Eval ("<<n<<"):"<<err<<"\n";
  }
  
  q.finmatch=err;
  q.fin_n=n;
  
  // copy cameras
  camlist.clear();
  for(unsigned int i=0; i<cal.NCamera(); ++i) {
    mistl::Camera cam;
    cal.GetCamera(i,cam );
    
    if(DebugMode() || verbose) {
      std::cout<<"Cam:"<<i<<"  ";
      cam.Info(); std::cout<<"\n";
    }
    camlist.push_back(cam);
  }
  
  // ted
  //if (true) {
   if (DebugMode()) {
    std::string   fn = DebugLogFile("finalpoints_" + to_string(seq_no) + ".coord");
    std::ofstream fo( fn );

    std::vector<mistl::Vector3f> plist;
    std::map<unsigned,unsigned> index_translate;
    cal.GetPointList(plist, index_translate);
    fo<<"#Coordinate list file\n#frame-id       Px Py Pz        ix iy   label\n";
    for(unsigned i=0; i<plist.size(); ++i)
      fo<<seq_no << " " <<plist.at(i)<< " 0 0 "<< to_string(i)<<"\n";
    
     // write obj file
    std::string   pofn = DebugLogFile("finalpoints_" + to_string(seq_no) + ".obj");
      
    mistl::TriangularMesh tp;
    for(unsigned i=0; i<plist.size(); ++i)
      tp.AddBox(plist.at(i), 0.02f );
    try { 
      mistl::WriteOBJ( tp, pofn.c_str() );
    } 
    catch ( mistl::Error e) {
      cout << "Error: " << e.GetMsg() << endl;
    }
  }
  
  ComputeQuality();
  
  return mistl::Success;
}

bool
RunCalibration::BoundaryCheck() {
  // test rotation
  for(unsigned i=0; i<refcamlist.size(); ++i) {
    float r;
    r = acos(refcamlist.at(i).A().Dot(camlist.at(i).A())) * 57.2957795131;
    std::cout<<"BoundaryCheck rot(A) cam:"<<i <<" ref.a:"<<refcamlist.at(i).A()<<"\tcam.a:"<<camlist.at(i).A()
      << " r:"<<r<<"\n";
    if(r>max_rot.at(i)) {
      if (DebugMode()) std::cout<<"BoundaryCheck rot(A) cam:"<<i<<" out of bound ("<<max_rot.at(i)<<")\n";
      return false;
    }
    r = acos(refcamlist.at(i).Up().Dot(camlist.at(i).Up())) * 57.2957795131;
    if(r>max_rot.at(i)) {
      if (DebugMode()) std::cout<<"BoundaryCheck rot(Up) cam:"<<i<<" out of bound ("<<max_rot.at(i)<<")\n";
      return false;
    }

    
  }
  
  return true;
}


void RunCalibration::ComputeQuality()
{
  // boundary check - this has priority 
  if( !BoundaryCheck() ) {
    q.quality = 0.0f;
    q.confidence = 1.0f;
  } else {
  
    Compute3DHistogram();
    
    float r4,r = q.finmatch;
    float a4,a = residualref;
    a4=a*a;
    r= (r - residualoffset );
    r4=r*r;
    
    q.quality = 1.0 - r ; //(r4/a4);
    if(q.quality<0.0) q.quality=0.0;

    if (DebugMode()) std::cout << "RunCalibration::ComputeQuality: finmatch:" << q.finmatch << " fin_n:" << q.fin_n;
  //   std::cout<< " r4:"<<r4 ;
    
    if( q.fin_n<(unsigned int)minimum_n) q.confidence = 0.f;
    else {
      float near,mid,far;
      Judge3DHistogram ( near,mid,far );
      float comb3d = near*mid*far;
      
      float comb2d = Judge2DHistogram();
      
      r= static_cast<float>(q.fin_n );
      float min = static_cast<float>(minimum_n);
      float no_of_p = ( (r-min) / min);
      if(no_of_p > 1.0f) no_of_p = 1.0f;
      
      // try mean
  //    q.confidence = no_of_p * ( 2.0/3.0*comb3d + comb2d/3.0) ;

          // ted
          // add weights for individual 3D partitions to allow comparison vs. multical set
          float alpha = 1.0f / 3.0f;
          float beta = (1.0f - alpha) / 3.0f;
          q.confidence = no_of_p * (beta*near + beta*mid + beta*far + comb2d*alpha);    
    }
  } 
  
  if (DebugMode()) std::cout << "\tq:" << q.quality << " c:" << q.confidence << "\n";
  
}


using namespace std;

void RunCalibration::DumpData( const std::string tag, bool write_points,
                                bool write_2dlist
                          )  
{
  // write points
  std::vector<mistl::Vector3f> plist;
  std::map<unsigned,unsigned> index_translate;

  cal.GetPointList(plist, index_translate);
  
//   std::cout<<"DumpData "<<tag<<" wp:"<<write_points<<" w2d:"<<write_2dlist<<" no of p:"<<plist.size()<<"\n";
  
  if(write_points) {
    std::string fn = DebugLogFile(tag + "_points_" + to_string(seq_no) + ".pl");
    std::ofstream fo(fn);
    fo<<"#3d pointlist\n3\n"<<plist.size()<<"\n";
    for(unsigned i=0; i<plist.size(); ++i)
      fo<<plist.at(i)<<"\n";
  }
                            
  // write cameras
  for(unsigned i=0; i</*new*/camlist.size(); ++i) {

	  std::string fn = DebugLogFile(tag + "_" + to_string(seq_no) + "_cam_0" + to_string(i) + ".ycam");

	  mistl::WriteCamera( camlist.at(i), fn.c_str() );
        
        // write 2d coordinates
        if(write_2dlist) {
          std::string fn = DebugLogFile(tag + "_" + to_string(seq_no) + "_cam_0" + to_string(i) + ".coord");
          std::vector<mistl::Vector2f> p2dlist;
          std::vector<unsigned> indexlist;
          // the points are stored as seq:=0
          cal.Get2DPointList( i, /*seq_no*/ 0, index_translate, p2dlist, indexlist );
          std::ofstream fo(fn);
//   std::cout<<"\tDumpData 2DPointList "<<tag<<" no of p:"<<p2dlist.size()<<"\n";
//      std::cout<<"Trying to write 2d point file:"<< fn<< " fs:"<<fo.is_open() << "\n";

          fo<<"#Coordinate list file\n#frame-id       Px Py Pz        ix iy   label\n";
          for(unsigned k=0; k<p2dlist.size(); ++k)
                fo<< seq_no << " " << plist.at( indexlist.at(k) ) <<"\t" <<  p2dlist.at(k) << "\t" << indexlist.at(k) << "\n";
      }  
        
  }
}

float RunCalibration::DistanceToRef() const
{
  // set postion to zero, as it is moved by rotation back to reference postion
  return mistl::Distance( camlist, refcamlist, 0.f );
}

void RunCalibration::SetReferenceCameras( const std::vector<mistl::Camera> & camlist ) 
{
  refcamlist=camlist;
}


}
