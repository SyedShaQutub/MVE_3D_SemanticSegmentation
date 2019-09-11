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
//    Author: 
//

#include "stdio.h"
#include <iostream>
#include <vector>
#include <string>
#include <sstream>


#include        "mistl/AuxilaryFunctions.h"



using namespace std;

namespace mistl {

std::vector<std::string>
SplitStr( const std::string &input )
{
  std::vector<std::string> ret;

  std::istringstream ss(input);
  std::string token;

  while(std::getline(ss, token, ',')) {
      ret.push_back(token);
  }
  return ret;
}


std::string     ReplaceDirName( const std::string & ipath,  const std::string &dirnam )
{  
  std::string path = ipath;
  std::string patt = "$datadir";
  size_t start_pos = path.find(patt);
  if(start_pos == std::string::npos)
        return ipath;
  

  path.replace(start_pos, patt.length(), dirnam);
  
  return path;
}

std::string     ImageName( const std::string & ipath )
{
  std::string path = ipath;
  std::string patt = ".jpg";
  size_t start_pos = path.find(patt);
  if(start_pos == std::string::npos)
        return ipath;
  

  return path.substr(0, start_pos);
  
}




bool fileCheck(const char* const fileName){
  FILE* testFile = fopen(fileName, "w");

  // check if the file opened.
  if (!testFile){
    return false;
  }

  fclose(testFile);

  return true;
}

bool fileExists(const char* const fileName){
  FILE* testFile = fopen(fileName, "r");

  // check if the file opened.
  if (!testFile){
    return false;
  }

  fclose(testFile);

  return true;
}



}

