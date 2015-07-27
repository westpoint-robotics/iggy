#include <ros/ros.h>
#include <ros/time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <triclops_vision/typedefs.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <triclops_vision/images_into_triclops.h>

using namespace cv;

void ImagesIntoTriclops::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    //ROS_INFO("Got a right one");
    combined_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    ROS_INFO("combined image %d,%d",combined_image.cols,combined_image.rows);
    cv::imshow("Combined Image", combined_image);
    cv::waitKey(3);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not converted from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  //ros::NodeHandle nh;
  ROS_INFO("Before creating the class");
  ImagesIntoTriclops iit;
  ROS_INFO("After creating the class");

  //image_transport::Subscriber sub = it.subscribe("/camera/right/image_raw", 1, imageCallback);
  //image_transport::Subscriber sub = it.subscribe("/camera/left/color", 1, imageCallback);
  ros::spin();
}
