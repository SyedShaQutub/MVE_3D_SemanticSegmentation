

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

#ifndef __MISTL_WriteOBJ_incl
#define __MISTL_WriteOBJ_incl


#include        "mistl/TriangularMesh.h"



namespace mistl {




/**
 * @ingroup MISTLCFunction
 * @brief Write 3D model  to OBJ file.
 * @param mesh 3D mesh data to write
 * @param fn (path &) filename
 * @param Y_axis_up If true then rotate coordinate so that Y-axis is upwards
 * @author O. Grau
*/
void WriteOBJ( const mistl::TriangularMesh &mesh, const char *fn, bool Y_axis_up=false ) ;


void WriteOBJ( FILE *fp, const mistl::TriangularMesh &mesh, bool Y_axis_up=false ) ;

void WriteOBJ( const std::vector<mistl::Vector3f>  &plist, const char *fn, float size, bool Y_axis_up=false ) ;


}

#endif