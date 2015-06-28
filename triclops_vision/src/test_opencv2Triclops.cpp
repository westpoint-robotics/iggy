#include <triclops.h>
#include <fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "triclops_vision/typedefs.h"
#include "triclops_vision/sto3dpoints.h"
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/LineFilter.h"

/* This is a program that tests the triclops camera driver and the triclops opencv code.
 * This test displays both the right and left camera images in the opencv highgui after
 * converting the camera triclops images into a opencv image.
 */

TriclopsError     te;

bool SHOW_OPENCV = true;

int main(int  argc, char **argv)
{
  TriclopsInput triclopsColorInput, triclopsMonoInput;
  TriclopsContext triclops;

  FC2::Camera camera;
  FC2::Image grabbedImage;

  camera.Connect();
  // configure camera
  if ( configureCamera( camera ) )
  {
      return EXIT_FAILURE;
  }

  // generate the Triclops context
  if ( generateTriclopsContext( camera, triclops ) )
  {
      return EXIT_FAILURE;
  }

  // Par 1 of 2 for grabImage method
  FC2::Error fc2Error = camera.StartCapture();
  if (fc2Error != FC2::PGRERROR_OK)
  {
      return FC2T::handleFc2Error(fc2Error);
  }

  // Get the camera info and print it out
  FC2::CameraInfo camInfo;
  fc2Error = camera.GetCameraInfo( &camInfo );
  if ( fc2Error != FC2::PGRERROR_OK )
  {
      std::cout << "Failed to get camera info from camera" << std::endl;
      return false;
  }
  else
  {
      ROS_INFO(">>>>> CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d \n", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber);
  }
  // Container of Images used for processing
  ImageContainer imageContainer;

    // Part 2 of 2 for grabImage method
    // this image contains both right and left images
    fc2Error = camera.RetrieveBuffer(&grabbedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    // generate triclops inputs from grabbed image
    if ( generateTriclopsInput( grabbedImage, imageContainer, triclopsColorInput, triclopsMonoInput ))
        {
                    return EXIT_FAILURE;
        }

    // output image disparity image with subpixel interpolation
    TriclopsImage16 disparityImage16;

    // carry out the stereo pipeline
    if ( doStereo( triclops, triclopsMonoInput, disparityImage16 ) )
    {
                return EXIT_FAILURE;
    }
    PointCloud points;
    //cv::vector<cv::Vec4i> lines;
    // publish the point cloud containing 3d points
    if ( gets3dPoints(grabbedImage, triclops, disparityImage16, triclopsColorInput, points) )
    {
        return EXIT_FAILURE;
    }
    else
    {
      ROS_INFO("Succesful getting 3dpoints");
    }


}
