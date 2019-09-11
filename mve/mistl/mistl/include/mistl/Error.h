
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

// #include	<string.h>
#include	<cstring>
#include        <iostream>


#ifndef __MISTL_EXCEPTION_H__
#define __MISTL_EXCEPTION_H__

#ifdef _MSC_VER
  #pragma warning(disable: 4996) // '*' was declared deprecated
#endif

namespace mistl {

/*! \brief error message class
**
**
*/
class Error
{
public:
	Error(const Error &ee);
	Error(const char *msg=0, const char *msg2=0);
	virtual ~Error();
	void SetMsg(const char *msg, const char *msg2=0);
	inline const char *GetMsg() const { return e_msg; }

  Error &operator =(const Error &ee){
    e_msg = new char[strlen(ee.e_msg) + 1];
    strcpy(e_msg, ee.e_msg); 
    return *this;
  }
protected:
	char	*e_msg;
};


#ifdef DEBUG
#define MISTL_ASSERT(expression, msg) \
if (!(expression))                        \
{                                         \
  std::cout << "MISTL_ASSERT line:"<< __LINE__ << " in "<< __FILE__ << " expr: " << #expression << "\n"; \
  throw  mistl::Error(msg);          \
}

#else
#define MISTL_ASSERT(expression, msg) \
if (!(expression))                        \
{                                         \
  throw  mistl::Error(msg);          \
}
#endif


}

#endif
