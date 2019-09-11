
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



#include        "mistl/Error.h"




namespace mistl {



Error::Error(const Error &ee)
{
	e_msg = new char[strlen(ee.e_msg)+1];
	strcpy(e_msg, ee.e_msg);
}

Error::Error(const char *msg, const char *msg2)
		: e_msg(NULL)
{
	if(msg)
		SetMsg(msg, msg2);
}

Error::~Error()
{
	if(e_msg) {
		delete[] e_msg;
		e_msg = NULL;
	}
}

void
Error::SetMsg(const char *msg, const char *msg2)
{
	if(e_msg) {
		delete[] e_msg;
		e_msg = NULL;
	}
	std::size_t len = strlen(msg)+1;
	if(msg2)
		len += strlen(msg2);
	e_msg = new char[len];
	strcpy(e_msg, msg);
	if(msg2)
		strcat(e_msg, msg2);
}



}

