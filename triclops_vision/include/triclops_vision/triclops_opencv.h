#ifndef TRICLOPS_OPENCV_H
#define TRICLOPS_OPENCV_H

#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <pcl_ros/point_cloud.h>

#include "triclops_vision/typedefs.h"

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(FC2::Image & bgrImage,
                           cv::Mat & cvImage);

// convert an Opencv into a triclops color image
//int convertOpencv2Triclops( FC2::Camera     & camera,
 //                            TriclopsContext & triclops );

#endif // TRICLOPS_OPENCV_H
