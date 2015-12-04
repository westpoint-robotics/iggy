#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "triclops_vision/typedefs.h"
#include "triclops_vision/image_publisher.h"


CameraSystem::CameraSystem{

  TriclopsInput triclopsColorInput, triclopsMonoInput;
  TriclopsContext triclops;

  FC2::Camera camera; // namespace FC2 --> instantiating Camera
  FC2::Image grabbedImage;

  camera.Connect();
  // configure camera - Identifies what camera is being used?
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

  ros::init(argc, argv, "talker");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("/triclops/points", 0);

  // Container of Images used for processing
  ImageContainer imageContainer;

  image_transport::ImageTransport it(nh);
  image_transport::Publisher image_pub_left= it.advertise("camera/left/rgb", 1);
  image_transport::Publisher image_pub_right= it.advertise("camera/right/rgb", 1);

  while (ros::ok())
  {
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


    // function call from image_publisher.cpp

    ImagePublisher imagePublisher(grabbedImage, imageContainer, image_pub_left,image_pub_right);  

}




