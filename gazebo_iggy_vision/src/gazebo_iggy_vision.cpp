#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>

#include <algorithm>
#include <assert.h>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>

#include "gazebo_iggy_vision/gazebo_iggy_vision.h"

#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <gazebo/sensors/SensorTypes.hh>

#include <sensor_msgs/point_cloud2_iterator.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


#include <tf/tf.h>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(GazeboIggyVision)

////////////////////////////////////////////////////////////////////////////////
// Constructorpcl::PointXYZRGB
GazeboIggyVision::GazeboIggyVision()
{
  this->point_cloud_connect_count_ = 0;
  this->depth_info_connect_count_ = 0;
  this->last_depth_image_camera_info_update_time_ = common::Time(0);
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboIggyVision::~GazeboIggyVision()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboIggyVision::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {

    ROS_INFO("!!!!  Loading Iggy Vision Plugin  !!!!");
  DepthCameraPlugin::Load(_parent, _sdf);

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  // copying from DepthCameraPlugin into GazeboRosCameraUtils
  this->parentSensor_ = this->parentSensor;
  this->width_ = this->width;
  this->height_ = this->height;
  this->depth_ = this->depth;
  this->format_ = this->format;
  this->camera_ = this->depthCamera;

  // using a different default
  if (!_sdf->HasElement("imageTopicName"))
    this->image_topic_name_ = "ir/image_raw";
  if (!_sdf->HasElement("cameraInfoTopicName"))
    this->camera_info_topic_name_ = "ir/camera_info";
    ROS_INFO("!!!!#########  Width,height,depth,format:%d,%d,%d,%s  ########!!!!",this->width, this->height, this->depth, this->format.c_str());
    // Width,height,depth,format:640,480,3,B8G8R8
  // point cloud stuff
  if (!_sdf->HasElement("pointCloudTopicName"))
    this->point_cloud_topic_name_ = "points";
  else
    this->point_cloud_topic_name_ = _sdf->GetElement("pointCloudTopicName")->Get<std::string>();

  // depth image stuffpcl::PointXYZRGB
  if (!_sdf->HasElement("depthImageTopicName"))
    this->depth_image_topic_name_ = "depth/image_raw";
  else
    this->depth_image_topic_name_ = _sdf->GetElement("depthImageTopicName")->Get<std::string>();

  if (!_sdf->HasElement("depthImageCameraInfoTopicName"))
    this->depth_image_camera_info_topic_name_ = "depth/camera_info";
  else
    this->depth_image_camera_info_topic_name_ = _sdf->GetElement("depthImageCameraInfoTopicName")->Get<std::string>();

  if (!_sdf->HasElement("pointCloudCutoff"))
    this->point_cloud_cutoff_ = 0.4;
  else
    this->point_cloud_cutoff_ = _sdf->GetElement("pointCloudCutoff")->Get<double>();

  if (!_sdf->HasElement("linesOnly"))
    this->lines_only_ = false;
  else
    this->lines_only_ = _sdf->GetElement("linesOnly")->Get<bool>();

  load_connection_ = GazeboRosCameraUtils::OnLoad(boost::bind(&GazeboIggyVision::Advertise, this));
  GazeboRosCameraUtils::Load(_parent, _sdf);
}

void GazeboIggyVision::Advertise()
{
  ros::AdvertiseOptions point_cloud_ao =
    ros::AdvertiseOptions::create<sensor_msgs::PointCloud2 >(
      this->point_cloud_topic_name_,1,
      boost::bind( &GazeboIggyVision::PointCloudConnect,this),
      boost::bind( &GazeboIggyVision::PointCloudDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->point_cloud_pub_ = this->rosnode_->advertise(point_cloud_ao);

  ros::AdvertiseOptions depth_image_ao =
    ros::AdvertiseOptions::create< sensor_msgs::Image >(
      this->depth_image_topic_name_,1,
      boost::bind( &GazeboIggyVision::DepthImageConnect,this),
      boost::bind( &GazeboIggyVision::DepthImageDisconnect,this),
      ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_pub_ = this->rosnode_->advertise(depth_image_ao);

  ros::AdvertiseOptions depth_image_camera_info_ao =
    ros::AdvertiseOptions::create<sensor_msgs::CameraInfo>(
        this->depth_image_camera_info_topic_name_,1,
        boost::bind( &GazeboIggyVision::DepthInfoConnect,this),
        boost::bind( &GazeboIggyVision::DepthInfoDisconnect,this),
        ros::VoidPtr(), &this->camera_queue_);
  this->depth_image_camera_info_pub_ = this->rosnode_->advertise(depth_image_camera_info_ao);
}



////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboIggyVision::PointCloudConnect()
{
  this->point_cloud_connect_count_++;
  (*this->image_connect_count_)++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboIggyVision::PointCloudDisconnect()
{
  this->point_cloud_connect_count_--;
  (*this->image_connect_count_)--;
  if (this->point_cloud_connect_count_ <= 0)
    this->parentSensor->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboIggyVision::DepthImageConnect()
{
  this->depth_image_connect_count_++;
  this->parentSensor->SetActive(true);
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboIggyVision::DepthImageDisconnect()
{
  this->depth_image_connect_count_--;
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void GazeboIggyVision::DepthInfoConnect()
{
  this->depth_info_connect_count_++;
}
////////////////////////////////////////////////////////////////////////////////
// Decrement count
void GazeboIggyVision::DepthInfoDisconnect()
{
  this->depth_info_connect_count_--;
}


////////////////////////////////////////////////////////////////////////////////
// Detect White Lines
void GazeboIggyVision::DetectWhiteLines(const unsigned char *_image,
                   unsigned int _width, unsigned int _height,
                   unsigned int _depth, const std::string &_format)
{
    // Width,height,depth,format:640,480,3,B8G8R8
    int byteWidth = _width*_depth;
    //this->cvImage = cv::Mat(_height, _width, CV_8UC3,  (char*)_image, byteWidth);
    //this->filtered_image = wl_filter.findLines(cvImage);

    //wl_filter.displayThreshold();
    //wl_filter.displayCyan();
    //wl_filter.filterControl();
    //cv::waitKey(3);        
 }


////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboIggyVision::OnNewDepthFrame(const float *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
  if (this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ <= 0 &&
        this->depth_image_connect_count_ <= 0 &&
        (*this->image_connect_count_) <= 0)
    {
      this->parentSensor->SetActive(false);
    }
    else
    {
      if (this->point_cloud_connect_count_ > 0)
        this->FillPointdCloud(_image);

      if (this->depth_image_connect_count_ > 0)
        this->FillDepthImage(_image);
    }
  }
  else
  {
    if (this->point_cloud_connect_count_ > 0 ||
        this->depth_image_connect_count_ <= 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboIggyVision::OnNewRGBPointCloud(const float *_pcd,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  this->depth_sensor_update_time_ = this->parentSensor->GetLastUpdateTime();
  if (!this->parentSensor->IsActive())
  {
    if (this->point_cloud_connect_count_ > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if (this->point_cloud_connect_count_ > 0)
    {
      this->lock_.lock();
      this->point_cloud_msg_.header.frame_id = this->frame_name_;
      this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
      this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
      this->point_cloud_msg_.width = this->width;
      this->point_cloud_msg_.height = this->height;
      this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

      sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg_);
      pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
      pcd_modifier.resize(_width*_height);

      point_cloud_msg_.is_dense = true;

      sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
      sensor_msgs::PointCloud2Iterator<float> iter_rgb(point_cloud_msg_, "rgb");

      for (unsigned int i = 0; i < _width; i++)
      {
        for (unsigned int j = 0; j < _height; j++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
        {
          unsigned int index = (j * _width) + i;
          *iter_x = _pcd[4 * index];
          *iter_y = _pcd[4 * index + 1];
          *iter_z = _pcd[4 * index + 2];
          *iter_rgb = _pcd[4 * index + 3];
          if (i == _width /2 && j == _height / 2)
          {
            uint32_t rgb = *reinterpret_cast<int*>(&(*iter_rgb));
            uint8_t r = (rgb >> 16) & 0x0000ff;
            uint8_t g = (rgb >> 8)  & 0x0000ff;
            uint8_t b = (rgb)       & 0x0000ff;
            std::cerr << (int)r << " " << (int)g << " " << (int)b << "\n";
          }
        }
      }

      this->point_cloud_pub_.publish(this->point_cloud_msg_);
      this->lock_.unlock();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////// =====================================================================
// Update the controller
void GazeboIggyVision::OnNewImageFrame(const unsigned char *_image,
    unsigned int _width, unsigned int _height, unsigned int _depth,
    const std::string &_format)
{
  if (!this->initialized_ || this->height_ <=0 || this->width_ <=0)
    return;

  //Convert to opencv image and store a member variable to the class
  // Width,height,depth,format:640,480,3,B8G8R8
  int byteWidth = _width*_depth;
  this->cvinit_image = cv::Mat(_height, _width, CV_8UC3,  (char*)_image, byteWidth);

  //ROS_ERROR("camera_ new frame %s %s",this->parentSensor_->Name().c_str(),this->frame_name_.c_str());
  this->sensor_update_time_ = this->parentSensor->GetLastUpdateTime();

  if (!this->parentSensor->IsActive())
  {
    if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run 1 frame after activate
      this->parentSensor->SetActive(true);
  }
  else
  {
    if ((*this->image_connect_count_) > 0)
      this->PutCameraData(_image);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put camera data to the interface
void GazeboIggyVision::FillPointdCloud(const float *_src)
{
  this->lock_.lock();

  this->point_cloud_msg_.header.frame_id = this->frame_name_;
  this->point_cloud_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->point_cloud_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;
  this->point_cloud_msg_.width = this->width;
  this->point_cloud_msg_.height = this->height;
  this->point_cloud_msg_.row_step = this->point_cloud_msg_.point_step * this->width;

  // Find white lines width,height,step 640,480,20480
  //ROS_INFO("width,height,step %d,%d,%d",this->width,this->height,this->point_cloud_msg_.row_step);

  ///copy from depth to point cloud message
  if(lines_only_)  
  {
    //FillPointCloudHelperWhiteAlt(this->point_cloud_msg_,
    FillPointCloudHelperWhite(this->point_cloud_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );
  }
  else
  {
    FillPointCloudHelper(this->point_cloud_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );
  }

  this->point_cloud_pub_.publish(this->point_cloud_msg_);

  this->lock_.unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Put depth image data to the interface
void GazeboIggyVision::FillDepthImage(const float *_src)
{
  this->lock_.lock();
  // copy data into image
  this->depth_image_msg_.header.frame_id = this->frame_name_;
  this->depth_image_msg_.header.stamp.sec = this->depth_sensor_update_time_.sec;
  this->depth_image_msg_.header.stamp.nsec = this->depth_sensor_update_time_.nsec;

  ///copy from depth to depth image message
  FillDepthImageHelper(this->depth_image_msg_,
                 this->height,
                 this->width,
                 this->skip_,
                 (void*)_src );

  this->depth_image_pub_.publish(this->depth_image_msg_);

  this->lock_.unlock();
}

// Fill depth information for White Lines only ====================================================================================
bool GazeboIggyVision::WhiteLineTester()
{
  // Get a list of pixels that are on the white lines
    std::vector<cv::Point2i> pixels = wl_filter.getPointsOnLines();
    cv::Mat cvImage = cv::Mat(480,640, CV_8UC3, cv::Scalar(0, 0, 0));

    for (int i = 0; i < pixels.size(); i++) {
        int pixelX = pixels[i].y;
        int pixelY = pixels[i].x;
        cvImage.at<cv::Vec3b>(pixelX,pixelY)=cv::Vec3b(255,255,255);
    }
    cv::Mat disImage;
    cv::resize(cvImage, disImage, cv::Size(400, 300));
    cv::imshow("Lines Image", disImage);
    cv::waitKey(3);
    return true;
}

// Fill depth information for White Lines only ====================================================================================
bool GazeboIggyVision::FillPointCloudHelperWhiteAlt(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{

    pcl::PointCloud<pcl::PointXYZRGB> whiteLinePoints;
    // Get a list of pixels that are on the white lines
    std::vector<cv::Point2i> pixels = wl_filter.getPointsOnLines();

    // Specifies if all the data in points is finite (true), or if certain points might contain Inf/NaN values (false).
    point_cloud_msg.is_dense = true;

    float* toCopyFrom = (float*)data_arg;
    int index = 0;

    double hfov = this->parentSensor->GetDepthCamera()->GetHFOV().Radian();
    double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));
    pcl::PointXYZRGB point;
    //  ROS_INFO("width,heigth,rowArg,colArg %d,%d,%d,%d",this->width,this->height,rows_arg,cols_arg);
    //  width,heigth,rowArg,colArg 640,480,480,640 --- rows/x/i  and cols/y/j
    // convert depth to point cloud
    for (int j=0; j<rows_arg; j++){
        double pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
        for (int i=0; i<cols_arg; i++) {
            double yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
            double depth = toCopyFrom[index++];

            point.x = depth * tan(yAngle);
            point.y = depth * tan(pAngle);
            if(depth > this->point_cloud_cutoff_)
            {
                point.z    = depth;
            }
            else //point in the unseeable range
            {
                point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN ();
                point_cloud_msg.is_dense = false;
            }

            // put image color data for each point
            uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));
            if (this->image_msg_.data.size() == rows_arg*cols_arg*3)
            {
                // color
                cv::Point2i value = cv::Point2i(i,j);
                if (std::find(pixels.begin(), pixels.end(),value)!=pixels.end()){
                point.r = 0;
                point.g = 255;
                point.b = 255;
                }
                else
                {
                point.x = 0.0;
                point.y = 0.0;
                point.z = 0.0;
                point.b = image_src[i*3+j*cols_arg*3+0];
                point.g = image_src[i*3+j*cols_arg*3+1];
                point.r = image_src[i*3+j*cols_arg*3+2];
                }
            }
            else
            {
                point.r = 255;
                point.g = 255;
                point.b = 255;
            }
        whiteLinePoints.push_back(point);
        }
    }

    pcl::toROSMsg(whiteLinePoints,point_cloud_msg) ;
    point_cloud_msg.header.frame_id="depth_camera_optical";
    point_cloud_msg.header.stamp=ros::Time::now();

    return true;
}

// Fill depth information for White Lines only ====================================================================================
bool GazeboIggyVision::FillPointCloudHelperWhite(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
    if ((this->cvinit_image.rows*this->cvinit_image.rows) == 0){
        return false;
    }

    cv::Mat cvImage = wl_filter.findLines(this->cvinit_image);

//    cv::Mat disImage;
//    cv::resize(cvImage, disImage, cv::Size(400, 300));
//    cv::imshow("Lines Image", disImage);
//    cv::waitKey(3);

  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(rows_arg*cols_arg);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");

  point_cloud_msg.is_dense = true;

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

    double hfov = this->parentSensor->GetDepthCamera()->GetHFOV().Radian();
  double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));

  // convert depth to point cloud
  for (uint32_t j=0; j<rows_arg; j++)
  {
    double pAngle;
    if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i<cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

         //  kkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk
      cv::Vec3b pix_color = cvImage.at<cv::Vec3b>(j,i);
      if (  pix_color == cv::Vec3b(255,255,0)){
      double depth = toCopyFrom[index];       

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x      = depth * tan(yAngle);
      *iter_y      = depth * tan(pAngle);
      if(depth > this->point_cloud_cutoff_)
      {
        *iter_z    = depth;
      }
      else //point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == rows_arg*cols_arg*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*cols_arg*3+0];
        iter_rgb[1] = image_src[i*3+j*cols_arg*3+1];
        iter_rgb[2] = image_src[i*3+j*cols_arg*3+2];
      }
      else if (this->image_msg_.data.size() == rows_arg*cols_arg)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*cols_arg];
        iter_rgb[1] = image_src[i+j*cols_arg];
        iter_rgb[2] = image_src[i+j*cols_arg];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
}
else{
     *iter_x = *iter_y = *iter_z = iter_rgb[0] = iter_rgb[1] = iter_rgb[2] = 0;
}
      index++;
    }
  }

  return true;
}

// Fill depth information
bool GazeboIggyVision::FillPointCloudHelper(
    sensor_msgs::PointCloud2 &point_cloud_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  sensor_msgs::PointCloud2Modifier pcd_modifier(point_cloud_msg);
  pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
  pcd_modifier.resize(rows_arg*cols_arg);

  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_rgb(point_cloud_msg_, "rgb");

  point_cloud_msg.is_dense = true;

  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  double hfov = this->parentSensor->GetDepthCamera()->GetHFOV().Radian();
  double fl = ((double)this->width) / (2.0 *tan(hfov/2.0));
  ROS_INFO("width,heigth,rowArg,colArg %d,%d,%d,%d",this->width,this->height,rows_arg,cols_arg);
  // convert depth to point cloud
  for (uint32_t j=0; j<rows_arg; j++)
  {
    double pAngle;
    if (rows_arg>1) pAngle = atan2( (double)j - 0.5*(double)(rows_arg-1), fl);
    else            pAngle = 0.0;

    for (uint32_t i=0; i<cols_arg; i++, ++iter_x, ++iter_y, ++iter_z, ++iter_rgb)
    {
      double yAngle;
      if (cols_arg>1) yAngle = atan2( (double)i - 0.5*(double)(cols_arg-1), fl);
      else            yAngle = 0.0;

      double depth = toCopyFrom[index++];

      // in optical frame
      // hardcoded rotation rpy(-M_PI/2, 0, -M_PI/2) is built-in
      // to urdf, where the *_optical_frame should have above relative
      // rotation from the physical camera *_frame
      *iter_x      = depth * tan(yAngle);
      *iter_y      = depth * tan(pAngle);
      if(depth > this->point_cloud_cutoff_)
      {
        *iter_z    = depth;
      }
      else //point in the unseeable range
      {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN ();
        point_cloud_msg.is_dense = false;
      }

      // put image color data for each point
      uint8_t*  image_src = (uint8_t*)(&(this->image_msg_.data[0]));
      if (this->image_msg_.data.size() == rows_arg*cols_arg*3)
      {
        // color
        iter_rgb[0] = image_src[i*3+j*cols_arg*3+0];
        iter_rgb[1] = image_src[i*3+j*cols_arg*3+1];
        iter_rgb[2] = image_src[i*3+j*cols_arg*3+2];
      }
      else if (this->image_msg_.data.size() == rows_arg*cols_arg)
      {
        // mono (or bayer?  @todo; fix for bayer)
        iter_rgb[0] = image_src[i+j*cols_arg];
        iter_rgb[1] = image_src[i+j*cols_arg];
        iter_rgb[2] = image_src[i+j*cols_arg];
      }
      else
      {
        // no image
        iter_rgb[0] = 0;
        iter_rgb[1] = 0;
        iter_rgb[2] = 0;
      }
    }
  }

  return true;
}

// Fill depth information
bool GazeboIggyVision::FillDepthImageHelper(
    sensor_msgs::Image& image_msg,
    uint32_t rows_arg, uint32_t cols_arg,
    uint32_t step_arg, void* data_arg)
{
  image_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  image_msg.height = rows_arg;
  image_msg.width = cols_arg;
  image_msg.step = sizeof(float) * cols_arg;
  image_msg.data.resize(rows_arg * cols_arg * sizeof(float));
  image_msg.is_bigendian = 0;

  const float bad_point = std::numeric_limits<float>::quiet_NaN();

  float* dest = (float*)(&(image_msg.data[0]));
  float* toCopyFrom = (float*)data_arg;
  int index = 0;

  // convert depth to point cloud
  for (uint32_t j = 0; j < rows_arg; j++)
  {
    for (uint32_t i = 0; i < cols_arg; i++)
    {
      float depth = toCopyFrom[index++];

      if (depth > this->point_cloud_cutoff_)
      {
        dest[i + j * cols_arg] = depth;
      }
      else //point in the unseeable range
      {
        dest[i + j * cols_arg] = bad_point;
      }
    }
  }
  return true;
}

void GazeboIggyVision::PublishCameraInfo()
{
  ROS_DEBUG("publishing default camera info, then depth camera info");
  GazeboRosCameraUtils::PublishCameraInfo();

  if (this->depth_info_connect_count_ > 0)
  {
    this->sensor_update_time_ = this->parentSensor_->GetLastUpdateTime();
    common::Time cur_time = this->world_->GetSimTime();
    if (cur_time - this->last_depth_image_camera_info_update_time_ >= this->update_period_)
    {
      this->PublishCameraInfo(this->depth_image_camera_info_pub_);
      this->last_depth_image_camera_info_update_time_ = cur_time;
    }
  }
}

}
