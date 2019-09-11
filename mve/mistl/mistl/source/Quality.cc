

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
//       @author Oliver Grau
//


#include        "mistl/Error.h"

#include        "mistl/Quality.h"
#include        <vector>

namespace mistl {

void  
Quality::Info() const
{
  std::cout<< "Quality: "<< GetQuality() << " Confidence:"<< GetConfidence() << "\n";
}



// TrackQuality
  
// void 
// TrackQuality::Compute()
// {
//   quality = 0.0f;
//   confidence = 0.0f;
// }

 
void 
TrackQuality::Info() const
{
  std::cout<< "TrackQuality: ";
  std::cout<< " n:"<<n<< " min_n:" << min_n << " ncam:"<< ncam;
  std::cout<< "Quality: "<< GetQuality() << " Confidence:"<< GetConfidence() << "\n";
}



  
}
