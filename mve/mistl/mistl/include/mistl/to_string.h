

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
//    Author: ted painter				
//

#ifndef __MISTL_to_string_H__
#define __MISTL_to_string_H__

// andrdoid compiler doesn't support std::to_string
template <typename T>
std::string to_string(T value)
{
	std::ostringstream os;
	os << value;
	return os.str();
}

#endif
