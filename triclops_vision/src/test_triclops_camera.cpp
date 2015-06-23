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
#include "triclops_vision/triclops_camera.h"
#include "triclops_vision/triclops_opencv.h"

/* This is a program that tests the triclops camera driver and the triclops opencv code.
 * This test displays both the right and left camera images in the opencv highgui after
 * converting the camera triclops images into a opencv image.
 */

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
  ros::Publisher pc2_pub = nh.advertise<sensor_msgs::PointCloud2>("points", 0);

  // grab image from camera.
  // this image contains both right and left images
  fc2Error = camera.RetrieveBuffer(&grabbedImage);
  if (fc2Error != FC2::PGRERROR_OK)
  {
      return FC2T::handleFc2Error(fc2Error);
  }
  // Container of Images used for processing
  ImageContainer imageContainer;
  cv::Mat      leftImage;
  cv::Mat      rightImage;
  // generate triclops inputs from grabbed image
  if (generateTriclopsInput(grabbedImage, imageContainer, triclopsColorInput, triclopsMonoInput))
  {
      return EXIT_FAILURE;
  }
  convertTriclops2Opencv(imageContainer.bgru[LEFT], leftImage);
  convertTriclops2Opencv(imageContainer.bgru[RIGHT], rightImage);
  FC2::Image convertedImage;
  convertOpencv2Triclops( rightImage, convertedImage);
  ROS_INFO("cvRows %d,cvCol %d, format %d", convertedImage.GetRows(),convertedImage.GetRows(), convertedImage.GetPixelFormat() );

  //const char * convertedFilename = "converted.pgm";



  while (ros::ok())
  {
    // grab image from camera.
    // this image contains both right and left images
    fc2Error = camera.RetrieveBuffer(&grabbedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }
    // Container of Images used for processing
    //ImageContainer imageContainer;
    //cv::Mat      leftImage;
    //cv::Mat      rightImage;
    // generate triclops inputs from grabbed image
    if (generateTriclopsInput(grabbedImage, imageContainer, triclopsColorInput, triclopsMonoInput))
    {
        return EXIT_FAILURE;
    }
    if (SHOW_OPENCV){
      convertTriclops2Opencv(imageContainer.bgru[LEFT], leftImage);
      convertTriclops2Opencv(imageContainer.bgru[RIGHT], rightImage);
      cv::imshow("Image Right", rightImage);
      cv::imshow("Image Left", leftImage);
      cv::waitKey(3);
      }
    // start of getting 3d point cloud
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
    if ( get3dPoints(points, grabbedImage, triclops, disparityImage16, triclopsColorInput) )
    {
        return EXIT_FAILURE;
    }
    else
    {
        points.header.frame_id="bumblebee2";
        // Problem with time format in PCL see: http://answers.ros.org/question/172241/pcl-and-rostime/
        //ros::Time time_st = ros::Time::now ();
        //points.header.stamp = time_st.toNSec()/1e3;
        //pcl_conversion::toPCL(ros::Time::now(), point_cloud_msg->header.stamp);
        pc2_pub.publish(points);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

}
