/*
  Author: Stuart Baker
  Revision Date: 2013-06-02
  File Name: image_transform.cc
  Description: A ROS nodelet to create a pointcloud from a given image.
                     Key assumptions:
                      - the camera is fixed in space (its transform doesn't change)
                      - the surface beneath the camera can be approximated as a flat plane
*/

// General C++ includes
#include <string>
#include <math.h>
#include <stdio.h>

// General ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Dynamic reconfigure includes
// #include <dynamic_reconfigure/server.h>
// #include <dynamic_reconfigure/config_tools.h>
// #include <dynamic_reconfigure/ConfigDescription.h>
// #include <dynamic_reconfigure/ParamDescription.h>
// #include <dynamic_reconfigure/Group.h>
// #include <dynamic_reconfigure/config_init_mutex.h>
// #include <line_filter/ImageTransformConfig.h>

// Defines
#define _USE_MATH_DEFINES

namespace enc = sensor_msgs::image_encodings;
namespace igvc_image_pipeline
{
  class ImageTransform : public nodelet::Nodelet
  {
    public:
      ImageTransform() {}
      ~ImageTransform() {}

    private:
      ros::NodeHandle nh_;
      ros::NodeHandle private_nh_;

      boost::shared_ptr<image_transport::ImageTransport> it_;
      image_transport::Subscriber image_sub_;
      image_transport::Publisher image_pub_;

      cv_bridge::CvImagePtr image_out_ptr_;

      // dynamic_reconfigure::Server<line_filter::ImageTransformConfig> reconfigure_server_;
      // dynamic_reconfigure::Server<line_filter::ImageTransformConfig>::CallbackType f_;

      double x_offset_, x_meter_pixel_, y_meter_pixel_;
      int image_height_, image_width_;
      cv::Mat image_transform_;

      virtual void onInit();
      void computeTransform(void);
      // void configCb(line_filter::ImageTransformConfig&, uint32_t);
      void imageCb(const sensor_msgs::ImageConstPtr&);
  }; // class ImageTransform
} // namespace igvc_image_pipeline

/*

Split for header

*/

// #include <image_transform.h>
#include <pluginlib/class_list_macros.h>

namespace igvc_image_pipeline
{
  void ImageTransform::onInit()
  {
    // Establish the nodehandles
    nh_ = getNodeHandle();
    private_nh_ = getNodeHandle();

    computeTransform();

    // Setup the image transport subscriber
    it_.reset(new image_transport::ImageTransport(nh_));
    image_pub_ = it_->advertise("transform_out", 1);
    image_sub_ = it_->subscribe("camera/rgb/image_color", 1, &ImageTransform::imageCb, this);

    // Setup the dynamic reconfigure server
    // f_ = boost::bind(&ImageTransform::configCb, this, _1, _2);
    // reconfigure_server_.setCallback(f_);

  }  // On init function

  void ImageTransform::computeTransform(void)
  {
    double x_image_distance_, y_image_distance_, x_image_min_, x_image_max_, y_image_near_max_,
                y_image_near_min_, y_image_far_max_, y_image_far_min_;

    // These values are all in the ROS coordinate frame: x forward, y left, z up
    // Set the base variables
    image_height_ = 480;//768;
    image_width_ = 640;//1024;

    // Calculate the x-direction values
    x_image_min_ = 0.266667;//0.701675;
    x_image_max_ = 2.5400;//4.9149;
    x_image_distance_ = x_image_max_ - x_image_min_;
    x_offset_ = 0.266667;//0.701675;
    x_meter_pixel_ = x_image_distance_ / image_height_;

    // Calculate the y-direction values
    y_image_far_max_ = 1.3462;//3.24485;
    y_image_far_min_ = -1.3462;//-3.1623;
    y_image_distance_ = y_image_far_max_ - y_image_far_min_;
    y_meter_pixel_ = y_image_distance_ / image_width_;

    y_image_near_max_ = 0.2032;//1.03505;
    y_image_near_min_ = -0.2032;//-0.94615;

    cv::Point2f src[4], dst[4];

    // Standard image coordinate from: origin at upper-left, +x to the right, +y down
    // Points are TL, TR, BL, BR
    src[0].x = 0;
    src[0].y = 0;
    src[1].x = image_width_;
    src[1].y = 0;
    src[2].x = 0;
    src[2].y = image_height_;
    src[3].x = image_width_;
    src[3].y = image_height_;

    dst[0].x = 0;
    dst[0].y = 0;
    dst[1].x = image_width_;
    dst[1].y = 0;
    dst[2].x = image_width_ / 2 - floor(y_image_near_max_ / y_meter_pixel_);
    dst[2].y = image_height_;
    dst[3].x = image_width_ / 2 + floor(-1.0 * y_image_near_min_ / y_meter_pixel_);
    dst[3].y = image_height_;

    image_transform_ = getPerspectiveTransform(src, dst);
  }


// Callback for the dynamic reconfigure server
  // void ImageTransform::configCb(line_filter::ImageTransformConfig &config, uint32_t level)
  // {

  // }  // Dynamic reconfigure callback

// Callback for the image pipeline
  void ImageTransform::imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    image_out_ptr_.reset (new cv_bridge::CvImage);

    // Create a local pointer to the source image
    cv_bridge::CvImageConstPtr source_image_ptr;
    try
    {
      source_image_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Transform the image to fix the camera perspective
    warpPerspective(source_image_ptr->image, image_out_ptr_->image,
                              image_transform_, source_image_ptr->image.size(), cv::INTER_LINEAR);

    image_out_ptr_->encoding = source_image_ptr->encoding;
    image_out_ptr_->header = source_image_ptr->header;
    image_pub_.publish(image_out_ptr_->toImageMsg());
  }  // Image callback
}  // Namespace igvc_image_pipeline

PLUGINLIB_DECLARE_CLASS(igvc_image_pipeline,
                                          ImageTransform,
                                          igvc_image_pipeline::ImageTransform,
                                          nodelet::Nodelet);