#ifndef		_TriangularMesh_incl
#  define	_TriangularMesh_incl


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

//
//	
//
//	based on some code from: Oliver Grau, MAY-1994
//

#include        <vector>
#include        "Vector3.h"
#include        "Vector2.h"
#include        "mistl/Error.h"

#include <sstream>
#include <fstream>
#include <string>

namespace mistl {

class TextureMap;

/**
	\class  TriangularMesh TriangularMesh.h
	\brief TriangularMesh class. 
	Creates a triangular mesh in 3D space 
	\author Oliver Grau
	\version 1.1
*/


class	TriangularMesh  {
public:

  TriangularMesh( ) {
    map = 0;
  }
  ~TriangularMesh( ) {
    
  }

//   TriangularMesh& operator=(const TriangularMesh &tm ) ;
    
  //! Add triangle from three points
  void  Add( const mistl::Vector3f &p1, const mistl::Vector3f &p2, const mistl::Vector3f &p3) {
    unsigned no=(unsigned)plist.size();
    plist.push_back(p1);
    plist.push_back(p2);
    plist.push_back(p3);
    trilist.push_back(mistl::Vector3u(no,no+1,no+2));
  }
  //! Add triangle from three indices
  void  Add( unsigned i1, unsigned i2, unsigned i3 ) {
    MISTL_ASSERT( (i1<plist.size())&&(i2<plist.size())&&(i3<plist.size()), "TriangularMesh::Add: indices out of range");
    trilist.push_back(mistl::Vector3u(i1,i2,i3));
  }

  //! Add vertex f
  void  Add( const mistl::Vector3f &p ) {     plist.push_back(p); }

  void  Info() const {
      std::cout << "TriangularMesh points:"<<plist.size()<< " triangles:"<<trilist.size();
      if(normallist.size()) std::cout << " normals:"<<normallist.size();
      if(txtlist.size()) std::cout << " texture-coordinate:"<<txtlist.size();
      std::cout << "\n";
  }
  
  void AddBox( const mistl::Vector3f &Center, float size )
  {
    mistl::Vector3f  v1;

    
    v1 = Center-mistl::Vector3f(size*0.5f,size*0.5f,size*0.5f) ;
    unsigned no= (unsigned)this->plist.size();
    
    this->Add(   v1  );
    this->Add( v1+mistl::Vector3f(size,0.0f,0.0f)  );
    this->Add( v1+mistl::Vector3f(size,size,0.0f)  );
    this->Add( v1+mistl::Vector3f(0.0f,size,0.0f)  );

    v1 = Center +mistl::Vector3f(-size*0.5f,-size*0.5f,size*0.5f) ;
    this->Add(   v1  );
    this->Add( v1+mistl::Vector3f(size,0.0f,0.0f)  );
    this->Add( v1+mistl::Vector3f(size,size,0.0f)  );
    this->Add( v1+mistl::Vector3f(0.0f,size,0.0f)  );
  
//     std::cout<<Center<<" v1:"<<v1<< " p1:"<< plist[no+1] <<"\n"; 
    
    this->Add( no+0, no+2, no+1); //bottom
    this->Add( no+2, no+0, no+3);

    this->Add( no+6, no+4, no+5); //top
    this->Add( no+6, no+7, no+4);

    this->Add( no+6, no+2, no+3); // back
    this->Add( no+3, no+7, no+6);

    this->Add( no+4, no+0,no+ 1); //front
    this->Add( no+5, no+4, no+1);

    // ri
    this->Add( no+5, no+1, no+2);
    this->Add( no+2, no+6, no+5);
    
    this->Add( no+0, no+4, no+3);
    this->Add( no+3, no+4, no+7);
  }
  
  bool HasTextureMap() const { return map!= 0; }
                
public:
  
  std::string   name;
  
  std::vector<mistl::Vector3f>  plist;
  std::vector<mistl::Vector3u>  trilist;
  //
  std::vector<mistl::Vector2f>  txtlist;        //!< Texture bindings
  std::vector<mistl::Vector3f>  normallist;
  
  mistl::TextureMap     *map;
  

};

mistl::TriangularMesh *
CreateBox( const mistl::Vector3f &Center, float size );
  
}

#endif
