#include "triclops.h"
#include "fc2triclops.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "triclops_vision/LineFilter.h"

LineFilter lf;
FC2::Image bgruImage;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat orig_image;
    cv::Mat filtered_image;
    orig_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv:Mat
    bgruImage.nrows = orig_image
    bgruImage.ncols = orig_image
    bgruImage.blue = orig_image
    bgruImage.red = orig_image
    bgruImage.green = orig_image
    bgruImage.rowinc = orig_image


    // convert Left image to OpenCV Mat TODO Find a better way to do this.
    unsigned int rowBytes = (double)bgrImage[LEFT].GetReceivedDataSize()/(double)bgrImage[LEFT].GetRows();
    leftImageBGR = cv::Mat(bgrImage[LEFT].GetRows(), bgrImage[LEFT].GetCols(), CV_8UC3, bgrImage[LEFT].GetData(),rowBytes);

    // Orig image size is 1024 768
    lf.findLines(orig_image, filtered_image);
    lf.displayOriginal();
    lf.displayCanny();
    lf.displayBlue();
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
