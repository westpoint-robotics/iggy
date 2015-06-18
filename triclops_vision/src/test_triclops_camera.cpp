#include <triclops.h>
#include <fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include "triclops_vision/typedefs.h"
#include "triclops_vision/triclops_camera.h"
#include "triclops_vision/triclops_opencv.h"


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
      std::printf("CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d \n", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber);
  }
  printf("here now1");
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    // grab image from camera.
    // this image contains both right and left images
      printf("here now1");
    fc2Error = camera.RetrieveBuffer(&grabbedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }
    // Container of Images used for processing
    printf("here now2");
    ImageContainer imageContainer;
    cv::Mat      leftImage;
    cv::Mat      rightImage;
    // generate triclops inputs from grabbed image
    if (generateTriclopsInput(grabbedImage, imageContainer, triclopsColorInput, triclopsMonoInput))
    {
        return EXIT_FAILURE;
    }
    printf("here now3");
    convertTriclops2Opencv(imageContainer.bgru[LEFT], leftImage);
    convertTriclops2Opencv(imageContainer.bgru[RIGHT], rightImage);
    cv::imshow("Image Right", rightImage);
    cv::imshow("Image Left", leftImage);
    cv::waitKey(3);
    ros::spinOnce();
    loop_rate.sleep();
  }

}
