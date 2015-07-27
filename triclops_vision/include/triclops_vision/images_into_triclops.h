#ifndef IMAGES_INTO_TRICLOPS_H
#define IMAGES_INTO_TRICLOPS_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <triclops_vision/typedefs.h>
using namespace cv;
class ImagesIntoTriclops
{
public:
  //! Constructor.
  //ImagesIntoTriclops();
    ImagesIntoTriclops()
    {
        ROS_INFO("Constructor start");
        //Topic you want to publish
        pub_pc2_ = n_.advertise<sensor_msgs::PointCloud2>("points", 0);

        //Topic you want to subscribe
        sub_image_ = n_.subscribe("/camera/combined/color", 10, &ImagesIntoTriclops::imageCallback, this);
        ROS_INFO("Constructor end");
    }

  //! Destructor.
  //~ImagesIntoTriclops();

  //! Publish the message.
  void publishMessage(ros::Publisher *pub_message);

  //! Callback function for subscriber.
  void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    cv::Mat combined_image; // original left image from subscriber
    cv::Mat left_image; // original right image from subscriber
    cv::Mat right_image; // original right image from subscriber
    cv::vector<cv::Vec4i> lines; // List of white lines
    ros::NodeHandle n_;
    ros::Publisher pub_pc2_;
    ros::Subscriber sub_image_;
};

#endif // IMAGES_INTO_TRICLOPS_H
