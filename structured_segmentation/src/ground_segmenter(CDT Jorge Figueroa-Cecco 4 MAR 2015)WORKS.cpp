/*
Copyright 2011, 2012 Massachusetts Institute of Technology

This work is sponsored by the Department of the Air Force under Air Force
Contract #FA8721-05-C-0002. Opinions, interpretations, conclusions and
recommendations are those of the authors and are not necessarily endorsed by the
United States Government.
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

#include <pcl_ros/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl/common/norms.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "structured_segmentation/typedefs.h"
#include "structured_segmentation/ground_segmenter.h"


struct float_xyz
{
  float x;
  float y;
  float z;
};

typedef struct float_xyz float_xyz;

namespace structured_segmentation
{
void
GroundSegmenter::initialize(void)
{
  // Pull from onInit
}


// Function called on initialization of the nodelet
void GroundSegmenter::onInit(void)
{
  printf("THIS IS A TEST1");
  nh_ = getNodeHandle();
  private_nh_ = getPrivateNodeHandle();

  // Output topics for ground and non-ground publication
  output_ground_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("output_ground", 0);
  output_not_ground_pub_ = private_nh_.advertise<sensor_msgs::PointCloud2>("output_not_ground", 0);

  // Input topic subscription
  input_ = private_nh_.subscribe("input", 3, &GroundSegmenter::segment, this, ros::TransportHints().tcpNoDelay(true));

  // server_ = new dynamic_reconfigure::Server<structured_segmentation::GroundSegmenterConfig>(private_nh_);
  // server_->setCallback(boost::bind(&GroundSegmenter::onConfigure, this, _1, _2));

  // private_nh_.getParam("display", display_);
  // private_nh_.getParam("mingraddist", mingraddist_);
  // mingraddist_sqr_ = mingraddist_ * mingraddist_;
  // private_nh_.getParam("maxgrad", maxgrad_);
  // private_nh_.getParam("maxdh", maxdh_);

  if (display_)
  {
    cv::namedWindow("Gradient Image", CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
    cvResizeWindow("Gradient Image", 500, 64);
    cv::namedWindow("Ground Image", CV_WINDOW_NORMAL | CV_WINDOW_FREERATIO | CV_GUI_NORMAL);
    cvResizeWindow("Ground Image", 500, 64);
    cvStartWindowThread();
  }
}

// void
// GroundSegmenter::onConfigure(structured_segmentation::GroundSegmenterConfig& config, uint32_t level)
// {
//   mingraddist_ = config.mingraddist;
//   mingraddist_sqr_ = mingraddist_ * mingraddist_;
//   maxgrad_ = config.maxgrad;
//   maxdh_ = config.maxdh;
// }

void
GroundSegmenter::segment(const PointCloudPtr &input)
{
  input_cloud_ptr_ = input;
  boost::shared_ptr<cv::Mat> gradient_matrix(new cv::Mat(input->height, input->width, CV_32F));
  boost::shared_ptr<cv::Mat> ground_matrix(new cv::Mat(input->height, input->width, CV_8U));
  *ground_matrix = cv::Mat::zeros(input->height, input->width, CV_8U);

  // // Calculate gradient field
  // calculateGradientField(gradient_matrix);
  // calculateGround(gradient_matrix, ground_matrix);

  // if(display_)
  // {
  //   cv::imshow("Gradient Image", *gradient_matrix);
  //   cv::imshow("Ground Image", *ground_matrix);
  // }

 PointCloud ground, not_ground;

 // for(size_t i=0; i < input->width; i++)
 //  for(size_t j=0; j < input->height; j++)
 //  {
 //    if(ground_matrix->at<uint8_t>(j,i))
 //      ground.push_back(input->at(i,j));
 //    else
 //      not_ground.push_back(input->at(i,j));file:///home/igvc/Desktop/velodyne_w_people.pcap
 //  }

 for(size_t i=0; i < input->width; i++){ // CDT Jorge Figueroa-Cecco uses this function to segment the input pointcloud into a ground and not_ground pointclouds. 4 MAR 2015
   PointT point = input->at(i);
   if (point.z < -1.0) {
     //point.z = 0.0;
     ground.push_back(point);
   }
   else {
     //point.z = 0.0;
     not_ground.push_back(point);
   }
 }
/* This is from the original code. This causes an error complaining that our pointcloud is a 1D PC, instead of a 2D. We can access width, but accessing height creates this error because height is default set to 1 because it is an unordered pointcloud. - CDT Jorge Figueroa-Cecco 4 MAR 2015.
 for(size_t i=0; i < input->width; i++)
  for(size_t j=0; j < input->height; j++)
  {
    PointT point = input->at(i,j);
    if(point.z > -1.0)
    {
      point.z = 0.0;
      ground.push_back(point);
    // else
    //   not_ground.push_back(input->at(i,j));
    }
  }*/
  
  ground.header.stamp = input->header.stamp;
  ground.header.frame_id = input->header.frame_id;
  output_ground_pub_.publish(ground);
  not_ground.header.stamp = input->header.stamp;
  not_ground.header.frame_id = input->header.frame_id;
  output_not_ground_pub_.publish(not_ground);
}

void
GroundSegmenter::calculateGradientField(boost::shared_ptr<cv::Mat> &gradient_matrix)
{
  #pragma omp parallel for
  for (unsigned int ring = 0; ring < input_cloud_ptr_->height; ++ring)
  {
    for (unsigned int index = 0; index < input_cloud_ptr_->width; ++index)
    {
      float grad = calcGrad(ring, index);
      gradient_matrix->at<float>(ring, index) = grad;
    }
  }
}

void
GroundSegmenter::getPoint(int ring, int index, PointT& point)
{
  // Assumes that ring == height, index == width
  if(ring >= static_cast<int>(input_cloud_ptr_->height))
  {
    point.x = std::numeric_limits<float>::quiet_NaN();
    point.y = std::numeric_limits<float>::quiet_NaN();
    point.z = std::numeric_limits<float>::quiet_NaN();
    point.intensity = std::numeric_limits<float>::quiet_NaN();
    return;
  }
  else if(ring < 0)
  {
    point.x = std::numeric_limits<float>::quiet_NaN();
    point.y = std::numeric_limits<float>::quiet_NaN();
    point.z = std::numeric_limits<float>::quiet_NaN();
    point.intensity = std::numeric_limits<float>::quiet_NaN();
    return;
  }

  if(index >= static_cast<int>(input_cloud_ptr_->width))
    index -= input_cloud_ptr_->width;
  else if(index < 0)
    index += input_cloud_ptr_->width;
  point = input_cloud_ptr_->at(index, ring);
}

float
GroundSegmenter::calcGrad(size_t ring, size_t index)
{
  float gradient_N = 0.0f;
  float gradient_S = 0.0f;
  float gradient_E = 0.0f;
  float gradient_W = 0.0f;
  size_t dist_dims = 3;
  size_t max_search = 40;

  PointT point;
  getPoint(ring, index, point);

  // Look for the closest point greater than mingraddist away,
  // moving up through the rings.
  for(int i = 1; i < max_search; i++)
  {
    PointT point_n;
    // If this point is outside of the top ring, just set gradient to 0 and move on.
    if(static_cast<int>(ring + i) > input_cloud_ptr_->height-1)
    {
      gradient_N = 0.0f;
      i = input_cloud_ptr_->height;
    }
    getPoint(ring + i, index, point_n);
    if(!isnan(point_n.x) && !isnan(point_n.y) && !isnan(point_n.z))
    {
        float dist = pcl::L2_Norm_SQR(point.data, point_n.data, dist_dims);
        if(dist > mingraddist_sqr_)
        {
          gradient_N = fabs(point_n.z - point.z)/(pcl::L2_Norm(point.data, point_n.data,2));
          i = input_cloud_ptr_->height;
        }
    }
  }

  if(gradient_N > maxgrad_)
    return gradient_N;

  // Roving down through the rings.
  for(int i = 1; i < max_search; i++)
  {
    PointT point_n;
    // If this point is inside the bottom ring, just set the gradient to 0 and move on.
    if((static_cast<int>(ring) - i) < 0)
    {
      gradient_S = 0.0f;
      i = input_cloud_ptr_->height;
    }
    getPoint(ring - i, index, point_n);
    if(!isnan(point_n.x) && !isnan(point_n.y) && !isnan(point_n.z))
    {
        float dist = pcl::L2_Norm_SQR(point.data, point_n.data, dist_dims);
        if(dist > mingraddist_sqr_)
        {
          gradient_S = fabs(point_n.z - point.z)/(pcl::L2_Norm(point.data, point_n.data,2));
          i = input_cloud_ptr_->height;
        }
    }
  }

  if(gradient_S > maxgrad_)
    return gradient_S;

  // Moving clockwise around a ring.
  for(int i = 1; i < max_search; i++)
  {
    PointT point_n;
    getPoint(ring, index + i, point_n);
    if(!isnan(point_n.x) && !isnan(point_n.y) && !isnan(point_n.z))
    {
        float dist = pcl::L2_Norm_SQR(point.data, point_n.data, dist_dims);
        if(dist > mingraddist_sqr_)
        {
          gradient_E = fabs(point_n.z - point.z)/(pcl::L2_Norm(point.data, point_n.data,2));
          i = input_cloud_ptr_->width;
        }
    }
  }

  if(gradient_E > maxgrad_)
    return gradient_E;

  // Moving counterclockwise around a ring.
  for(int i = 1; i < max_search; i++)
  {
    PointT point_n;
    getPoint(ring, index - i, point_n);

    if(!isnan(point_n.x) && !isnan(point_n.y) && !isnan(point_n.z))
    {
        float dist = pcl::L2_Norm_SQR(point.data, point_n.data, dist_dims);
        if(dist > mingraddist_sqr_ )
        {
          gradient_W = fabs(point_n.z - point.z)/(pcl::L2_Norm(point.data, point_n.data,2));
          i = input_cloud_ptr_->width;
        }
    }
  }

  // Return the maximum of the 4 calculated gradients.
  float maxgradient = std::max(std::max(gradient_E, gradient_W), std::max(gradient_S, gradient_N));
  return std::min(maxgradient, 1.0f);
}

void
GroundSegmenter::calculateGround(const boost::shared_ptr<cv::Mat> &gradient_matrix,
                                boost::shared_ptr<cv::Mat> &ground_matrix)
{

  float last_ground_cw = input_cloud_ptr_->at(0,0).z;
  float last_ground_ccw = input_cloud_ptr_->at(0,0).z;

  for (size_t index = 0; index < input_cloud_ptr_->width; index++)
  {
    float grad = gradient_matrix->at<float>(0, index);
    // Check to make sure that it's under max gradient
    if(grad <= maxgrad_)
    {
      if(input_cloud_ptr_->at(index, 0).z < -1.3)
        ground_matrix->at<uint8_t>(0, index) = 255;
    }
  }


  for(size_t ring = 1; ring < input_cloud_ptr_->height; ring++)
  {
    for(size_t index = 0; index < input_cloud_ptr_->width; index++)
    {
      float grad_cw = gradient_matrix->at<float>(ring, index);
      size_t inv_index = input_cloud_ptr_->width - index - 1;
      float grad_ccw = gradient_matrix->at<float>(ring, inv_index);

      if(grad_cw <= maxgrad_)
      {
        if(ground_matrix->at<uint8_t>(ring-1, index) &&
           fabs(input_cloud_ptr_->at(index,ring-1).z-input_cloud_ptr_->at(index,ring).z) <= maxdh_)
        {
          ground_matrix->at<uint8_t>(ring, index) = 255;
          last_ground_cw = input_cloud_ptr_->at(index, ring).z;
        }
        else if(ground_matrix->at<uint8_t>(ring-6, index) ||
                ground_matrix->at<uint8_t>(ring-4, index) ||
                ground_matrix->at<uint8_t>(ring-2, index) ||
                ground_matrix->at<uint8_t>(ring-1, index-1) ||
                ground_matrix->at<uint8_t>(ring-1, index+1) ||
                ground_matrix->at<uint8_t>(ring, index-1)   ||
                ground_matrix->at<uint8_t>(ring, index+1))
        {
          if(fabs(input_cloud_ptr_->at(index,ring).z - last_ground_cw) <= maxdh_)
          {
            ground_matrix->at<uint8_t>(ring, index) = 255;
            last_ground_cw = input_cloud_ptr_->at(index,ring).z;
          }
        }
      }
      if(grad_ccw <= maxgrad_)
      {
        if(ground_matrix->at<uint8_t>(ring-1, inv_index) &&
           fabs(input_cloud_ptr_->at(inv_index,ring-1).z-input_cloud_ptr_->at(inv_index,ring).z) <= maxdh_)
        {
          ground_matrix->at<uint8_t>(ring, inv_index) = 255;
          last_ground_ccw = input_cloud_ptr_->at(inv_index, ring).z;
        }
        else if(ground_matrix->at<uint8_t>(ring-4, inv_index) ||
                ground_matrix->at<uint8_t>(ring-6, inv_index) ||
                ground_matrix->at<uint8_t>(ring-2, inv_index) ||
                ground_matrix->at<uint8_t>(ring-1, inv_index-1) ||
                ground_matrix->at<uint8_t>(ring-1, inv_index+1) ||
                ground_matrix->at<uint8_t>(ring, inv_index-1)   ||
                ground_matrix->at<uint8_t>(ring, inv_index+1))
        {
          if(fabs(input_cloud_ptr_->at(inv_index,ring).z - last_ground_ccw) <= maxdh_)
          {
            ground_matrix->at<uint8_t>(ring, inv_index) = 255;
            last_ground_ccw = input_cloud_ptr_->at(inv_index,ring).z;
          }
        }
      }

    }
  }
}

};  // End of GroundSegmenter class

PLUGINLIB_DECLARE_CLASS(structured_segmentation, ground_segmenter,
                        structured_segmentation::GroundSegmenter,
                        nodelet::Nodelet)
