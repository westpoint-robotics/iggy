
//=============================================================================
// stereoto3dpoints
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written to a file. 
//
// This point file can be viewed with PGRView under windows.
//
//=============================================================================

#include "triclops.h"

#include "fc2triclops.h"
#include <pcl_ros/point_cloud.h>
#include "triclops_vision/typedefs.h"


#include <stdio.h>
#include <stdlib.h>
#include "triclops_vision/common.h"
#include "triclops_vision/common.h"

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
                           TriclopsInput    & stereoData );

// carry out stereo processing pipeline
int doStereo( TriclopsContext const & triclops, 
               TriclopsInput  const & stereoData,
               TriclopsImage16      & depthImage );

int gets3dPoints( FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData,
                  PointCloud      & returnedPoints);

// save 3d points generated from stereo processing
int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData );

