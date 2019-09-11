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
//	 @author Oliver Grau, Daniel Pohl
//

#include	"mistl/DistortionKP.h"

namespace mistl {

  DistortionKP::DistortionKP() : mistl::Distortion() {
    maxDiameterLookupTable = 2.0f; // from center it looks maxDiameterLookupTable/2 (=> radius) into all directions
    numEntriesLookupTable = 2000;
    lookUpTableNeedsRebuilding = true;

    use_lut = false; //true;
  }

  void DistortionKP::CreateInverseLookupTable(){  
    if(!use_lut) 
      return;
    
    cout << "DistortionKP::CreateInverseLookupTable()" << endl;

    // init array if this is the first time
    if((int)lookUpTable.size() == 0){
      lookUpTable.resize(numEntriesLookupTable);
      for(int i = 0; i < numEntriesLookupTable; ++i){
        lookUpTable[i].resize(numEntriesLookupTable);
        for(int j = 0; j < numEntriesLookupTable; ++j){
          lookUpTable[i][j].resize(2);
        }
      }
    }
    
    const float stepSize = maxDiameterLookupTable / (float)numEntriesLookupTable;
    const float offsetToShiftFromFirstQuadrantToCenter = numEntriesLookupTable / 2.0f;
    

    for(int intX = 0; intX < numEntriesLookupTable; intX++){
      for(int intY = 0; intY < numEntriesLookupTable; intY++){
        const float x = (intX - offsetToShiftFromFirstQuadrantToCenter) * stepSize;
        const float y = (intY - offsetToShiftFromFirstQuadrantToCenter) * stepSize;

        const float xSquared = x * x;
        const float ySquared = y * y;

        const float rd = (xSquared + ySquared);
        const float rdSquared = rd * rd;
        const float scale = 1.0f + GetK1() * rd + GetK2() * rdSquared + GetK3() * rdSquared * rdSquared;

        const float xTimesY = x*y;
        lookUpTable[intX][intY][0] = x * scale + (2.0f * GetP1() * xTimesY + GetP2() * (rdSquared + 2.0f * xSquared));
        lookUpTable[intX][intY][1] = y * scale + (GetP1() * (rdSquared + 2.0f * ySquared) + 2.0f * GetP2() * xTimesY);

        //cout << "[" << intX << "][" << intY << "][0] = " << lookUpTable[intX][intY][0] << "; [1] = " << lookUpTable[intX][intY][1] << endl;
      }      
    }

    lookUpTableNeedsRebuilding = false;
  }

  void DistortionKP::DistortICC(float x, float y, float &outx, float &outy) const {
    const float xSquared = x * x;
    const float ySquared = y * y;
    const float rd = (xSquared + ySquared);
    const float rdSquared = rd * rd;
    const float scale = 1.0f + GetK1() * rd + GetK2() * rdSquared + GetK3() * rdSquared * rd;

    const float xTimesY = x*y;
    outx = x * scale + (2.0f * GetP1() * xTimesY + GetP2() * (rdSquared + 2.0f * xSquared));
    outy = y * scale + (GetP1() * (rdSquared + 2.0f * ySquared) + 2.0f * GetP2() * xTimesY);

    //cout << "DistortICC: x = " << x << ", y = " << y << " to outx = " << outx << ", outy = " << outy << endl;
  }


  void DistortionKP::UnDistortICC(float x, float y, float& outx, float &outy) {
    if(!use_lut) 
      return Distortion::UnDistortICC(x,y, outx,outy );  

    if(lookUpTableNeedsRebuilding){
      CreateInverseLookupTable();
    }
    
    int closestX = 0;
    int closestY = 0;
    float minDistance = 99999999.9f;
    
    // find match with minimal distance to the entry from the table
    for(int intX = 0; intX < numEntriesLookupTable; intX++){
      for(int intY = 0; intY < numEntriesLookupTable; intY++){
        const float differenceVectorX = x - lookUpTable[intX][intY][0];
        const float differenceVectorY = y - lookUpTable[intX][intY][1];

        const float distance = sqrtf(differenceVectorX * differenceVectorX + differenceVectorY * differenceVectorY);
        if(distance < minDistance){
          minDistance = distance;
          closestX = intX;
          closestY = intY;
        }
      }
    }

    float debugClosestX = lookUpTable[closestX][closestY][0];
    float debugClosestY = lookUpTable[closestX][closestY][1];

    const float stepSize = maxDiameterLookupTable / (float)numEntriesLookupTable;
    const float offsetToShiftFromFirstQuadrantToCenter = numEntriesLookupTable / 2.0f;

    outx = (closestX - offsetToShiftFromFirstQuadrantToCenter) * stepSize;
    outy = (closestY - offsetToShiftFromFirstQuadrantToCenter) * stepSize;

    cout << "UnDistortICC: x = " << x << ", y = " << y << " to outx = " << outx << ", outy = " << outy << endl;
  }
	
}
