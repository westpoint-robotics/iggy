//#include <triclops.h>
//#include <fc2triclops.h>
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
#include "triclops_vision/vision_3d.h"
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/image_publisher.h"

// ADDED to allow publishing of raw images. TODO clean this up. This is messy solution.
#include <image_transport/image_transport.h>

/* This is a program that tests the triclops camera driver and the triclops opencv code.
 * This test displays both the right and left camera images in the opencv highgui after
 * converting the camera triclops images into a opencv image.
 */
/* TEMP
TriclopsError     te;
Vision3D vs;

bool SHOW_OPENCV = true;

int main(int  argc, char **argv)
{
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



    // Messy solution to publish the images in ROS ----- STARTS HERE  ---------------------------------------------------------
    //TODO Clean up this code and put in OO paradigm
    /*
 Inputs: grabbedImage, imageContainer

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                            grabbedImage,
                            true /*assume little endian*/      /*
                            unprocessedImage[RIGHT],
                            unprocessedImage[LEFT]);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
      {
          return FC2T::handleFc2TriclopsError(fc2TriclopsError,
                                       "unpackUnprocessedRawOrMono16Image");
      }// FlyCapture2::Image& unprocessedImage[RIGHT];

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    //DML get left and right image into opencv Mat
    for ( int i = 0; i < 2; ++i )
    {
        if ( convertToBGR(unprocessedImage[i], imageContainer.bgr[i]) )
        {
            return 1;
        }
    }
    // convert images to OpenCV Mat
    cv::Mat      leftImage;
    cv::Mat      rightImage;

    //ROS_INFO(">>>>> Data Size Bytes: %d \n",imageContainer.bgr[LEFT].GetDataSize());
    unsigned int rowBytes = (double)imageContainer.bgr[LEFT].GetDataSize()/(double)imageContainer.bgr[LEFT].GetRows();
    //ROS_INFO(">>>>> ROW Bytes: %d rows %d cols %d\n",rowBytes,imageContainer.bgr[LEFT].GetRows(), imageContainer.bgr[LEFT].GetCols());
    leftImage = cv::Mat(imageContainer.bgr[LEFT].GetRows(), imageContainer.bgr[LEFT].GetCols(), CV_8UC3, imageContainer.bgr[LEFT].GetData(),rowBytes);
    rowBytes = (double)imageContainer.bgr[RIGHT].GetDataSize()/(double)imageContainer.bgr[RIGHT].GetRows();
    rightImage = cv::Mat(imageContainer.bgr[RIGHT].GetRows(), imageContainer.bgr[RIGHT].GetCols(), CV_8UC3, imageContainer.bgr[RIGHT].GetData(),rowBytes);

    // Uncomment the below 3 lines to have opencv display the images
    //cv::imshow("Image Right", rightImage);
    //cv::imshow("Image Left", leftImage);
    //cv::waitKey(3);

    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
    image_pub_left.publish(msg);
    msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightImage).toImageMsg();
    image_pub_right.publish(msg);

//Outputs: None
*/

    // Messy solution to publish the images in ROS ----- END HERE -----------------------------------------------------------------

    // output image disparity image with subpixel interpolation

/* TEMP    
    TriclopsImage16 disparityImage16;

    // carry out the stereo pipeline
    if ( vs.doStereo( triclops, triclopsMonoInput, disparityImage16 ) )
    {
                return EXIT_FAILURE;
    }

    PointCloud points;
    // publish the point cloud containing 3d points
    if ( vs.gets3dPoints(grabbedImage, triclops, disparityImage16, triclopsColorInput, points) )
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
