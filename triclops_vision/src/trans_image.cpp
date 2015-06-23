
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "triclops_vision/LineFilter.h"
#include "triclops_vision/triclops_camera.h"
#include "triclops_vision/triclops_opencv.h"

LineFilter lf;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat filtered_image;
    cv::vector<cv::Vec4i> lines;
    cv::Mat orig_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    // Orig image size is 1024 768
    lf.findLines(orig_image, filtered_image, lines);
    lf.displayOriginal();
    lf.displayCanny();
    lf.displayCyan();
    lf.displayHough();

  }

  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not converted from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  //image_transport::Subscriber sub = it.subscribe("/camera/right/image_raw", 1, imageCallback);
  image_transport::Subscriber sub = it.subscribe("/camera/left/color", 1, imageCallback);
  ros::spin();
}

