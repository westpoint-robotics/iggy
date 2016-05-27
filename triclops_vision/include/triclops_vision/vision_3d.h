
//=============================================================================
// stereoto3dpoints
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written to a file. 
//
// This point file can be viewed with PGRView under windows.
//
//=============================================================================


#ifndef VISION_3D_H
#define VISION_3D_H

#include <stdio.h>
#include <stdlib.h>
#include "triclops/triclops.h"
#include "triclops/fc2triclops.h"
#include <pcl_ros/point_cloud.h>
#include "triclops_vision/typedefs.h"
#include "triclops_vision/common.h"
#include "triclops_vision/common.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/camera_system.h"
#include <pcl_ros/point_cloud.h>
#include <image_transport/image_transport.h>


/**
 * @brief The Vision3D class. This class implements a filter and creates a point cloud from input
 *
 */

class Vision3D
{
    public:
        Vision3D(int argc, char **argv, CameraSystem *camera);
        virtual ~Vision3D();
        void run();

        // reference to the outside camera system
        CameraSystem* camerasystem;

        // generate triclops input necessary to carry out stereo processing
        int generateTriclopsInput( FC2::Image const & grabbedImage, 
                           ImageContainer   & imageContainer,
                           TriclopsInput    & colorData,
                           TriclopsInput    & stereoData );

        // carry out stereo processing pipeline
        int doStereo( TriclopsContext const & triclops, 
               TriclopsInput  const & stereoData,
               TriclopsImage16      & depthImage );

        int producePointCloud( cv::Mat const &disparity,
                  cv::Mat const &maskImage,
                  cv::Mat const &redImage,
                  cv::Mat const &blueImage,
                  PointCloud      &returnedLines,
                  PointCloud      &returnedRed,
                  PointCloud      &returnedBlue);

        // save 3d points generated from stereo processing
        int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData );


        //void visionCallBackRGBRight(const sensor_msgs::ImageConstPtr& msg);
        //void visionCallBackRGBLeft(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackDisparity(const sensor_msgs::ImageConstPtr& msg);
        //void visionCallBackFilteredRight(const sensor_msgs::ImageConstPtr& msg);
        //void visionCallBackFilteredRightRed(const sensor_msgs::ImageConstPtr& msg);
        //void visionCallBackFilteredRightBlue(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackFilteredLeft(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackFilteredLeftRed(const sensor_msgs::ImageConstPtr& msg);
        void visionCallBackFilteredLeftBlue(const sensor_msgs::ImageConstPtr& msg);
        void produceDisparity(cv::Mat left, cv::Mat right, cv::Mat *result);
        PointCloud lineCloud, redCloud, blueCloud;


    protected:


    private:
        int numDisp;
        int blockSize;
        cv::Mat filteredLeft;
        cv::Mat filteredLeftRed;
        cv::Mat filteredLeftBlue;
        cv::Mat filteredRight;
        cv::Mat filteredRightRed;
        cv::Mat filteredRightBlue;
        cv::Mat imageLeft;
        cv::Mat imageRight;
        cv::Mat disparityImage;
        image_transport::Subscriber subcamfilteredright;
        image_transport::Subscriber subcamfilteredrightred;
        image_transport::Subscriber subcamfilteredrightblue;
        image_transport::Subscriber subcamfilteredleft;
        image_transport::Subscriber subcamfilteredleftred;
        image_transport::Subscriber subcamfilteredleftblue;

        image_transport::Subscriber subcamrgbright;
        image_transport::Subscriber subcamrgbleft;  
        image_transport::Subscriber subcamdisp; 
        ros::Publisher lineCloudPublisher;
        ros::Publisher blueCloudPublisher;
        ros::Publisher redCloudPublisher;
	ros::Rate *loop_rate;
	ros::NodeHandle *nh;
	ros::AsyncSpinner *async_wheels;

};

#endif // VISION_3D_H

