/*
Copyright 2011, 2012 Massachusetts Institute of Technology

This work is sponsored by the Department of the Air Force under Air Force
Contract #FA8721-05-C-0002. Opinions, interpretations, conclusions and
recommendations are those of the authors and are not necessarily endorsed by the
United States Government.

MODIFIED extensively by CDT Jorge Figueroa-Cecco. 4 MAR 2015. SIMPLE IS BETTER!
*/

#include <ros/ros.h>
#include <sys/time.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud2.h>

#include "structured_segmentation/ground_segmenter.h"


namespace structured_segmentation
{

// Function called on initialization of the nodelet
void GroundSegmenter::onInit(void)
{
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Output topics for ground and non-ground publication
  output_ground_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("output_ground", 0);
  output_not_ground_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("output_not_ground", 0);

  // Input topic subscription
  input_ = private_nh_.subscribe("input", 3, &GroundSegmenter::segment, this, ros::TransportHints().tcpNoDelay(true));
}

void
GroundSegmenter::segment(const PointCloudPtr &input)
{
 input_cloud_ptr_ = input;

 PointCloud ground, not_ground;

 for(size_t i=0; i < input->width; i++){ // CDT Jorge Figueroa-Cecco uses this function to segment the input pointcloud into a ground and not_ground pointclouds. 4 MAR 2015
   PointT point = input->at(i);
   if (point.z < -1.1) {
     //point.z = 0.0;
     ground.push_back(point);
   }
   else {
     //point.z = 0.0;
     not_ground.push_back(point);
   }
 }
  
  ground.header.stamp = input->header.stamp;
  ground.header.frame_id = input->header.frame_id;
  output_ground_pub_.publish(ground);
  not_ground.header.stamp = input->header.stamp;
  not_ground.header.frame_id = input->header.frame_id;
  output_not_ground_pub_.publish(not_ground);
}

};  // End of GroundSegmenter class

PLUGINLIB_DECLARE_CLASS(structured_segmentation, ground_segmenter,
                        structured_segmentation::GroundSegmenter,
                        nodelet::Nodelet)
