

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

#ifndef __MISTL_ReadCoordinateFile_incl
#define __MISTL_ReadCoordinateFile_incl

#include        "mistl/Vector3.h"
#include        "mistl/Vector2.h"
#include        <vector>

// #include        "dlm/scene/Identifiers.h"
// #include "dlm/scene/Scene.h"


namespace mistl {


/*!
  \class CoordinateListEntry ReadCoordinateFile.h
  \brief CoordinateListEntry class
*/
class CoordinateListEntry {
  
public:
 
  
public:
        int id;                 //!< object or frame id
        double P[3];            //!< point3 coordinate
        double ip[2];           //!< image coordinate
//         char label[21];
        long   pointid;        //!< id of feature point / point3, set to -1 if not assigned
        // -----
        unsigned    fid;            //<! (2d) feature id, this is not supported the coordinate files, purpose: debugging
        bool    disabled;               //<! flag if taken out by outlier detection

};

void AddList( std::vector<mistl::CoordinateListEntry> &clist,
              const std::vector<mistl::Vector3f> &p3list,
                unsigned camid, unsigned seqid, 
                const std::vector<mistl::Vector2f> &p2list, 
                const std::vector<unsigned> &indexlist,
                unsigned offset = 0
            );

unsigned        ReadCoordinateFile( const char *fn, std::vector<mistl::CoordinateListEntry> &clist ) ;

int    
MaxSequence( const std::vector<mistl::CoordinateListEntry> &clist);

std::vector<mistl::CoordinateListEntry>
FilterSequence( const std::vector<mistl::CoordinateListEntry> &clist, int seq ) ;



}

#endif