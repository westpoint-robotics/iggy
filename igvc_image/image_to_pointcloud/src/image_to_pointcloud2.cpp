/*
  Author: Stuart Baker
  Revision Date: 2013-06-02
  File Name: image_to_pointcloud.cc
  Description: A ROS nodelet to create a pointcloud from a given image.
                     Key assumptions:
                      - the camera is fixed in space (its transform doesn't change)
                      - the surface beneath the camera can be approximated as a flat plane
*/

// General C++ includes
#include <string>
#include <math.h>
#include <stdio.h>
#include <stdint.h>
// General ROS includes
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <tf/transform_listener.h>

// OpenCV includes
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// PCL includes
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Dynamic reconfigure includes
// #include <dynamic_reconfigure/server.h>
// #include <dynamic_reconfigure/config_tools.h>
// #include <dynamic_reconfigure/ConfigDescription.h>
// #include <dynamic_reconfigure/ParamDescription.h>
// #include <dynamic_reconfigure/Group.h>
// #include <dynamic_reconfigure/config_init_mutex.h>
// #include <line_filter/ImageToPointCloudConfig.h>

// Defines
#define _USE_MATH_DEFINES

namespace enc = sensor_msgs::image_encodings;
class ImageToPointCloud //: public nodelet::Nodelet
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
public:
    ImageToPointCloud():
    min_height_(0.0),
    max_height_(0.15),
    angle_min_(-M_PI/2),
    angle_max_(M_PI/2),
    angle_increment_(M_PI/180.0/2.0),
    scan_time_(1.0/30.0),
    range_min_(0.45),
    range_max_(10.0),
    output_frame_id_("/base_footprint"),
    it_(nh_)
    {
      // Setup the subscribers and publishers
      scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("out",1);
      
      
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 > ("cloud", 1);

      // Setup the image transport subscriber
      //it_.reset(new image_transport::ImageTransport(nh_));
      image_sub_ = it_.subscribe("/stereo/transform_out", 5, &ImageToPointCloud::imageCb, this);

      computeTransform();

      // Setup the dynamic reconfigure server
      // f_ = boost::bind(&ImageToPointCloud::configCb, this, _1, _2);
      // reconfigure_server_.setCallback(f_);

      // Set the frames and grab the transform

      output_cloud_.header.frame_id = "/odom";
    }
    ~ImageToPointCloud() {}

  private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    ros::Publisher scan_pub_;
    ros::Publisher cloud_pub_;

    // dynamic_reconfigure::Server<line_filter::ImageToPointCloudConfig> reconfigure_server_;
    // dynamic_reconfigure::Server<line_filter::ImageToPointCloudConfig>::CallbackType f_;

    tf::Vector3 origin_;

    boost::mutex connect_mutex_;

    PointCloud output_cloud_;
    int n;
    double x_offset_, x_meter_pixel_, y_meter_pixel_;
    int image_height_, image_width_;
    cv::Mat image_transform_;
    
    sensor_msgs::PointCloud2 out;
    sensor_msgs::LaserScan output;
    
    double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_, range_min_sq_;
    std::string output_frame_id_;
    
/*

Split for header

*/



  void computeTransform(void)
  {
    double x_image_distance_, y_image_distance_, x_image_min_, x_image_max_, y_image_near_max_,
		y_image_near_min_, y_image_far_max_, y_image_far_min_;

    // These values are all in the ROS coordinate frame: x forward, y left, z up
    // Set the base variables
    image_height_ = /*480;*/768;
    image_width_ = /*640;*/1024;

    // Calculate the x-direction values
    x_image_min_ = /*0.266667;*/0.701675;
    x_image_max_ = /*2.5400;*/4.9149;
    x_image_distance_ = x_image_max_ - x_image_min_;
    x_offset_ = /*0.266667;*/0.701675;
    x_meter_pixel_ = x_image_distance_ / image_height_;

    // Calculate the y-direction values
    y_image_far_max_ = /*1.3462;*/3.24485;
    y_image_far_min_ = /*-1.3462;*/-3.1623;
    y_image_distance_ = y_image_far_max_ - y_image_far_min_;
    y_meter_pixel_ = y_image_distance_ / image_width_;

    y_image_near_max_ = /*0.2032;*/1.03505;
    y_image_near_min_ = /*-0.2032;*/-0.94615;

  }


  int convert_to_scan(pcl::PointXYZ point)
  {
      
      const float &x = -1.0 * point.y;
      const float &y = point.z;
      const float &z = point.x;
      
      if ( std::isnan(x) || std::isnan(y) || std::isnan(z) )
      {
        ROS_INFO("rejected for nan in point(%f, %f, %f)\n", x, y, z);
        return -1;
      }
      
      if (-y > max_height_ || -y < min_height_)
      {
        ROS_INFO("rejected for height %f not in range (%f, %f)\n", y, min_height_, max_height_);
        return -1;
      }

      double range_sq = z*z+x*x;
      if (range_sq < range_min_sq_) {
        ROS_INFO("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range_sq, range_min_sq_, x, y, z);
        return -1;
      }
      
      double angle = -atan2(x, z);
      if (angle < output.angle_min || angle > output.angle_max)
      {
        ROS_INFO("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        return -1;
      }
      int index = (angle - output.angle_min) / output.angle_increment;
      
      if (output.ranges[index] * output.ranges[index] > range_sq)
        output.ranges[index] = sqrt(range_sq);
      
      return 0;
   } // convert_to_scan
  
  // Callback for the dynamic reconfigure server
  // void ImageToPointCloud::configCb(line_filter::ImageToPointCloudConfig &config, uint32_t level)
  // {

  // }  // Dynamic reconfigure callback

  // Callback for the image pipeline
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
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
    
    //create laserScan
    output.header.frame_id = output_frame_id_; // Set output frame. Point clouds come from "optical" frame, scans come from corresponding mount frame
    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;
    
    
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);
    output.ranges.assign(ranges_size, output.range_max + 1.0);
    
    // Set the limits for iterating and the cycle throught the image to create a point cloud
    image_height_ = source_image_ptr->image.rows;
    image_width_ = source_image_ptr->image.cols;
    for (int i = 0; i < image_height_; ++i)
    {
      for (int j = 0; j < image_width_; ++j)
      {
	
	int pixel= source_image_ptr->image.at<int>(i, j);
	if (pixel==255)
	{
	  createPoint(i, j);
	}
      }
    }
    // Publish the generated point cloud and then clear it out for the next cycle
    
    //pcl_conversions::toPCL(output_cloud_.header); <-- this didnt work
    //publish point cloud
    pcl::toROSMsg(output_cloud_, out);
    out.header.stamp = ros::Time::now();
    cloud_pub_.publish(out);
    //publish laserScan
    output.header.stamp = ros::Time::now();
    scan_pub_.publish(output);
    
    //clear data structures
    output_cloud_.points.clear();
    output_cloud_.width =0;
    output_cloud_.height =0;
    

  }  // Image callback

  // Function to create a point cloud point from a given pixel
  void createPoint(int i, int j)
  {
    pcl::PointXYZ pixel_point;
    float x = x_offset_ + (image_height_ - i) * x_meter_pixel_;
    float y = (image_width_ / 2 - j) * y_meter_pixel_;
    // Calculate the z distance
    if (!(x==0.0 && y==0.0))
    {
      pixel_point.z = 0;

    // Calculate the x distance
    pixel_point.x = x;

    // Calculate the y distance
    pixel_point.y = y;
    }
    //add point to pointcloud
    output_cloud_.push_back(pixel_point);
    //add point to laserscan
    convert_to_scan(pixel_point);
  }  // Create point function
  
  
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_to_pointcloud");
  ImageToPointCloud ipc;
  ros::spin();
  return 0;
}
// PLUGINLIB_DECLARE_CLASS(igvc_image_pipeline,
//                                           ImageToPointCloud,
//                                           igvc_image_pipeline::ImageToPointCloud,
//                                           nodelet::Nodelet);