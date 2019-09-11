

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


#include        "mistl/ReadCoordinateFile.h"


#include        "mistl/Error.h"
#include        <stdlib.h>
#include        <stdio.h>

namespace mistl {

void AddList( std::vector<mistl::CoordinateListEntry> &clist,
              const std::vector<mistl::Vector3f> &p3list,
                unsigned camid, unsigned seqid, 
                const std::vector<mistl::Vector2f> &p2list, 
                const std::vector<unsigned> &indexlist,
                unsigned offset )
{
  MISTL_ASSERT( p2list.size()==indexlist.size(), "CoordinateListEntry :: AddList 2d-list and index list have different sizes");
  for(unsigned i=0; i<p2list.size(); ++i) {
    mistl::CoordinateListEntry itm;
    const mistl::Vector3f &P = p3list.at( indexlist[i] );
    itm.P[0]=P[0]; itm.P[1]=P[1]; itm.P[2]=P[2]; 
    itm.ip[0]=p2list[i].X(); itm.ip[1]=p2list[i].Y(); 
    itm.id=seqid;
    itm.pointid=  offset + indexlist[i];
    itm.disabled = false;
    clist.push_back( itm );
  }
}




unsigned       ReadCoordinateFile( const char *fn, std::vector<mistl::CoordinateListEntry> &clist, mistl::Vector3i *imagesize ) {
        clist.clear();
        FILE *fp = fopen(fn,"r");
//      std::cout<<" opening file:"<<fn<<" :"<< (fp!=0) << "\n";
        MISTL_ASSERT( fp!=0, "error opening file for reading coordinates");
        
        if(imagesize) {imagesize->X()= -1;imagesize->Y()=- 1;}
        char buf[201];
        for(;;) {
                if(!fgets(buf,200,fp)) break;
                if(!strncmp(buf,"#target-size:",13) && imagesize) {
                   int i=sscanf(buf+14,"%d %d", & imagesize->X(), & imagesize->Y() );
                   std::cout<<"ReadCoordinateFile imagesize:"<<imagesize->X() <<","<< imagesize->Y() <<"\n";
                }
                if(buf[0]=='#') continue;
//              std::cout<<"> "<<buf;
                mistl::CoordinateListEntry itm;
//                 int i=sscanf(buf,"%d %lf %lf %lf %lf %lf %20s", & itm.id, itm.P, itm.P+1, itm.P+2, itm.ip, itm.ip+1, itm.label );
                int i=sscanf(buf,"%d %lf %lf %lf %lf %lf %ld", & itm.id, itm.P, itm.P+1, itm.P+2, itm.ip, itm.ip+1, &itm.pointid );
//              std::cout<<"sscanf:"<<i<<"\n";
                MISTL_ASSERT( (i==6)||(i==7), "error reading coordinates");
//                 if(i==6) itm.label[0]='\0';
                if(i==6) itm.pointid = -1;
                itm.disabled = false;
                clist.push_back( itm );
        }
        fclose(fp);
        return (unsigned)clist.size();
}

int    
MaxSequence( const std::vector<mistl::CoordinateListEntry> &clist) {
  int max= -1;
  for(unsigned i=0; i<clist.size(); ++i) {
//     std::cout<<i<<"\t"<<clist[i].id<<"\n";
    if( clist[i].id > max ) max = clist[i].id;
  }
  return max;
}


std::vector<mistl::CoordinateListEntry>
FilterSequence( const std::vector<mistl::CoordinateListEntry> &clist, int seq ) {
  std::vector<mistl::CoordinateListEntry> outclist;
  for(unsigned i=0; i<clist.size(); ++i) 
    if( clist[i].id == seq ) outclist.push_back(clist[i]);
  return outclist;
}




}
