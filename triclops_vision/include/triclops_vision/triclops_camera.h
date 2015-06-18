#ifndef TRICLOPS_CAMERA_H
#define TRICLOPS_CAMERA_H

#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <pcl_ros/point_cloud.h>

#include "triclops_vision/typedefs.h"

//TODO make this object oriented program

//
// Macro to check, report on, and handle Triclops API error codes.
//
// TODO get rid of this macro and make it a function
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
     "ERROR: %s reported %s.\n", \
     function, \
     triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

struct ImageContainer
{
    FC2::Image unprocessed[2];
    FC2::Image bgru[2];
    FC2::Image mono[2];
    FC2::Image packed;
};

enum IMAGE_SIDE
{
    RIGHT = 0, LEFT
};

// configue camera to capture image
int configureCamera( FC2::Camera &camera );

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera     & camera,
                             TriclopsContext & triclops );

// capture image from connected camera
int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage );

// convert image to BRGU
int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );

// convert image to BRG
int convertToBGR( FC2::Image & image, FC2::Image & convertedImage );

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage,
                           ImageContainer   & imageContainer,
                           TriclopsInput    & colorData,
                           TriclopsInput    & stereoData);

// carry out stereo processing pipeline
int doStereo( TriclopsContext const & triclops,
               TriclopsInput  const & stereoData,
               TriclopsImage16      & depthImage );

// save 3d points generated from stereo processing
int save3dPoints( FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData );
// save 3d points generated from stereo processing
int get3dPoints(  PointCloud       & returnPoints,
                  FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData);


#endif // TRICLOPS_CAMERA_H
