#ifndef COMMON_H
#define COMMON_H

//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( description, error )	\
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
         "*** Triclops Error '%s' at line %d :\n\t%s\n", \
         triclopsErrorToString( error ), \
         __LINE__, \
         description );	\
      printf( "Press any key to exit...\n" ); \
      getchar(); \
      exit( 1 ); \
   } \
}

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

struct ImageContainer
{
    FC2::Image unprocessed[2];
    FC2::Image bgru[2];
    FC2::Image bgr[2];
    FC2::Image mono[2];
    FC2::Image packed;
};

enum IMAGE_SIDE
{
        RIGHT = 0, LEFT
};




#endif
