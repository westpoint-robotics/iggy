#ifndef TRICLOPS_OPENCV_H
#define TRICLOPS_OPENCV_H

#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <pcl_ros/point_cloud.h>

#include "triclops_vision/typedefs.h"
#include <cv_bridge/cv_bridge.h>

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(FC2::Image & bgrImage,
                           cv::Mat & cvImage);
int convertTriclops2Opencv( TriclopsInput & bgrImage,
                           cv::Mat & cvImage);
int convertTriclops2Opencv( TriclopsImage & bgrImage,
                           cv::Mat & cvImage);

int convertTriclops2Opencv( TriclopsColorImage & bgrImage,
                           cv::Mat & cvImage);

int convertTriclops2Opencv( TriclopsImage16 & bgrImage,
                           cv::Mat & cvImage);

int convertOpencv2Triclops( cv::Mat & cvImage,
                             TriclopsColorImage & bgrImage);

// convert an Opencv into a triclops color image
int convertOpencv2Triclops( cv::Mat & cvImage,
                             FC2::Image & bgrImage);
#endif // TRICLOPS_OPENCV_H

/*FC2::Image
   Image(
       unsigned int    rows,
       unsigned int    cols,
       unsigned int    stride,
       unsigned char*  pData,
       unsigned int    dataSize,
       unsigned int    receivedDataSize,
       PixelFormat     format,
       BayerTileFormat bayerFormat = NONE );

typedef struct TriclopsImage
{
  // The number of rows in the image.
  int   nrows;
  // The number of columns in the image.
  int   ncols;
  // This is the pitch, or row increment for the image.  Data must be
  // contiguous within each row, and the rows must be equally spaced.
  // Rowinc indicates the number of bytes between the beginning of a
  // row and the beginning of the following row.
  int   rowinc;
  // The area for the pixel data.  This must be numRows * numCols bytes
  // large.
  unsigned char*	data;

*/
