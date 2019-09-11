

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


#include	"mistl/Error.h"
#include	"mistl/Camera.h"
#include	"mistl/OCV_Camera.h"
#include	"mistl/DR_CameraArray.h"
#include        "mistl/YAML_IO.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

using namespace std;

namespace mistl {
  


void
ReadCameraCAHV( mistl::Camera &cam, const char *fn  )
{
	 std::ifstream filestream(fn);

  MISTL_ASSERT(!filestream.fail(), "error opening camera file for reading");

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  // check header type and version
  std::string line;
  filestream >> line;

//   std::cout<<"RRRRR:"<<line<<"\n";
  MISTL_ASSERT(!line.compare(";CAHV"), "file not of type CAHV");

  filestream >> line;
//   std::cout<<"RRRRR-vers:"<<line<<"\n";
  int version = 0; 
  if(!line.compare("1.1")) version=1;
  else if(!line.compare("1.2")) version=2;
  else if(!line.compare("1.3")) version=3;
 else
	MISTL_ASSERT(true, "unknown version of CAHV file");

  getline(filestream, line); //discard remains of first line

  // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
  mistl::Vector3f c,a,h,v;
  float	r3=0.0f,r5=0.0f, r7=0.0f, sx,sy;
  int dx,dy;

  unsigned mode = 0;

//   std::cout<<"ReadCamera "<<fn<<" vers:"<<version<<"\n";
  
  while (!filestream.eof())
  {
    MISTL_ASSERT(!filestream.fail(), "error reading CAHV file");

    getline(filestream, line);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // get rid of comment lines (prefixed with ';')
    if (!line.substr(0, 1).compare(";")) { continue; }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    std::istringstream stream(line);

    switch (mode)
    {
	case 0: { stream >> c.X()>>c.Y()>>c.Z();  c *= 1.0f/1000.0f ;                      } break;
	case 1: { stream >> a.X()>>a.Y()>>a.Z();  
// 		std::cout<<"a:"<<a;
		a.Normalize(); 		
// 		std::cout<<" norm:"<<a<<"\n";
	} break;
	case 2: { stream >> h.X()>>h.Y()>>h.Z();                         } break;
	case 3: { stream >> v.X()>>v.Y()>>v.Z();                         } break;
	case 4: 
        {
          stream >> r3;       
          if(version>=2)  {
            stream >> r5;    
//             std::cout<<"r5:"<<r5<<"\n";
          }
          if(version==3)  {
            stream >> r7;    
//             std::cout<<"r7:"<<r7<<"\n";
          }
          
        } break;
	case 5: { stream >> sx;    stream >> sy;    sx  *= 1.0f/1000.0f; sy *= 1.0f/1000.0f;     } break;
	case 6: { stream >> dx;    stream >> dy;            } break;
    }

    ++mode;


  }
  cam.SetCAHV( c,a,h,v, dx,dy, sx,sy );
  cam.SetK1(r3);
  if(version>=2) cam.SetK2(r5);
  if(version==3) cam.SetK3(r7);

}

void ReadCamera(  mistl::Camera &cam, const char *fn  )
{       
  if(!fn){
    cout << "ReadCamera: invalid filename." << endl;
    return;
  }
  if(fn[0] == 0){
    cout << "ReadCamera: invalid filename." << endl;
    return;
  }

  const char *cp = strrchr(fn,'.');

  if(!cp){
    cout << "ReadCamera: Did not find a . in file name: " << fn << endl;
    return;
  }
  
  if(!strcmp(cp, ".cahv")) 
    ReadCameraCAHV(cam,fn);
  else if(!strcmp(cp, ".ycam")) 
    ReadCameraYAML(cam,fn);
  else if(!strcmp(cp, ".yml")) {
	  mistl::OCV_Camera oCam;
	  ReadCameraYAML(oCam, fn);
//	  oCam.AdjustF(1e-5);   // guess work here: what is the actual sensor pixel size?
//	  oCam.Info();
	  oCam.CopyTo(cam);
  }
  else
    MISTL_ASSERT(false,"ReadCamera: unknown file extension - no reader module to delegate");
}
	
}
