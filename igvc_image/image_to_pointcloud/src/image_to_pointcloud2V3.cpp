/*
  Author: Joseph Salmento & Stuart Baker
  Revision Date: 2014-03-22
  File Name: image_to_pointcloud.cc
  Description: A ROS node to create a pointcloud from a given image.
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
		//laserscan variables with () go here
		it_(nh_)
		{
		  // Setup the subscribers and publishers
		  cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2 > ("cloud", 1);

		  // Setup the image transport subscriber
		  image_sub_ = it_.subscribe("/stereo/transform_out", 5, &ImageToPointCloud::imageCb, this);

		  computeTransform();

		  // Set the frames
		  output_cloud_.header.frame_id = "/odom";
		}
    ~ImageToPointCloud() {}

  private:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    ros::Publisher cloud_pub_;

	// Declare node variables
    tf::Vector3 origin_;
    boost::mutex connect_mutex_;
    PointCloud output_cloud_;
    double x_offset_, x_meter_pixel_, y_meter_pixel_;
    int image_height_, image_width_;
    cv::Mat image_transform_;
    sensor_msgs::PointCloud2 out;
/*

Split for header

*/
  //Calculates the variable valuables for a given image.
  void computeTransform(void)
  {
    double x_image_distance_, y_image_distance_, x_image_min_, x_image_max_, y_image_near_max_,
		y_image_near_min_, y_image_far_max_, y_image_far_min_;

    // These values are all in the ROS coordinate frame: x forward, y left, z up
    // Set the base variables if using an Bumblebee2 on IGGE
    image_height_ = 768;
    image_width_ = 1024;
	
    // Calculate the x-direction values
    x_image_min_ = 0.701675;
    x_image_max_ = 4.9149;
    x_image_distance_ = x_image_max_ - x_image_min_;
    x_offset_ = 0.701675;
    x_meter_pixel_ = x_image_distance_ / image_height_;

    // Calculate the y-direction values
    y_image_far_max_ = 3.24485;
    y_image_far_min_ = -3.1623;
    y_image_distance_ = y_image_far_max_ - y_image_far_min_;
    y_meter_pixel_ = y_image_distance_ / image_width_;

    y_image_near_max_ = 1.03505;
    y_image_near_min_ = -0.94615;
  }

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
        
    // Set the limits for iterating and the cycle throught the image to create a point cloud
    image_height_ = source_image_ptr->image.rows;
    image_width_ = source_image_ptr->image.cols;
	
	//#pragama ... enables the for  loops to run in parallel
    for (int i = 0; i < image_height_; ++i)
    {
      for (int j = 0; j < image_width_; ++j)
      {
		int pixel= source_image_ptr->image.at<int>(i, j);
		
		//if pixel is white create a point
		if (pixel==255)
		{
			createPoint(i, j);
		}
      }
    }
    
	// Publish the generated point cloud and then clear it out for the next cycle 
    pcl::toROSMsg(output_cloud_, out);
    out.header.stamp = ros::Time::now();
    cloud_pub_.publish(out);
    
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
  }// Create point function
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_to_pointcloud");
  ImageToPointCloud ipc;
  ros::spin();
  return 0;
}
// The code below is used to change node into a nodelete.
// PLUGINLIB_DECLARE_CLASS(igvc_image_pipeline,
//                                           ImageToPointCloud,
//                                           igvc_image_pipeline::ImageToPointCloud,
//                                           nodelet::Nodelet);