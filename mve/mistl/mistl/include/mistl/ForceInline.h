#ifndef _FORCEINLINE_H
#define _FORCEINLINE_H

#ifndef finline
  #ifdef WIN32
    #define finline __forceinline
  #else
    #define finline inline __attribute__((always_inline)) 
  #endif
#endif


#endif