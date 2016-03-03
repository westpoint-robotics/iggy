
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

#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "triclops_vision/typedefs.h"

#include "triclops_vision/vision_3d.h"
#include "triclops_vision/line_filter.h"

Vision3D::Vision3D(int argc, char **argv, CameraSystem *camera) {
    ros::init(argc,argv,"linefilter");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    this->pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("/vision3D/points", 0);

    this->subcamdisp = it.subscribe("/camera/disparity", 1, &Vision3D::visionCallBackDisparity, this);
    this->subcamfilteredright = it.subscribe("/camera/right/linefiltered", 0, &Vision3D::visionCallBackFilteredRight, this);
    this->subcamfilteredleft = it.subscribe("/camera/left/linefiltered", 0, &Vision3D::visionCallBackFilteredLeft, this);

    // TODO: Do we need these?
    //this->subcamrgbright = it.subscribe("/camera/right/rgb", 0, &Vision3D::visionCallBackRGBRight, this);
    //this->subcamrgbleft = it.subscribe("/camera/left/rgb", 0, &Vision3D::visionCallBackRGBLeft, this);

    this->numDisp = 16*5;
    this->blockSize = 21;
    this->camerasystem = camera;

    /* Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow("Disparity Control", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar( "Number Disparities", "Disparity Control", &(this->numDisp), 100*5);
    cv::createTrackbar( "Block Size", "Disparity Control", &(this->blockSize), 253);
    */
}

Vision3D::~Vision3D()
{
    cvDestroyAllWindows();
}

void Vision3D::visionCallBackDisparity(const sensor_msgs::ImageConstPtr& msg)
{
    this->disparityImage = cv_bridge::toCvShare(msg, "mono16")->image;
}

void Vision3D::visionCallBackFilteredRight(const sensor_msgs::ImageConstPtr& msg)
{
    this->filteredRight = cv_bridge::toCvShare(msg, "mono8")->image;
}

void Vision3D::visionCallBackFilteredLeft(const sensor_msgs::ImageConstPtr& msg)
{
    this->filteredLeft = cv_bridge::toCvShare(msg, "mono8")->image;
}

void Vision3D::visionCallBackRGBRight(const sensor_msgs::ImageConstPtr& msg)
{
    this->imageRight = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(this->imageRight, this->imageRight, CV_BGR2GRAY);
}

void Vision3D::visionCallBackRGBLeft(const sensor_msgs::ImageConstPtr& msg)
{
    this->imageLeft = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::cvtColor(this->imageLeft, this->imageLeft, CV_BGR2GRAY);
}

int Vision3D::producePointCloud(  cv::Mat const &disparityImage,
                cv::Mat const &maskImage,
                PointCloud      & returnedPoints)
{
    float            x, y, z;
    int              i, j, k;
    unsigned char    *disparityRow, disparity;
    unsigned char    *maskRow, mask;

    //printf("[!] Searching through image at %p for obstacles...\n", &maskImage);
    for ( i = 0; i < disparityImage.rows; i++ )
    {
        disparityRow = disparityImage.data + ( i * disparityImage.step );
        maskRow = maskImage.data + ( i * maskImage.step );
        for ( j = 0; j < disparityImage.cols; j++ )
        {
            disparity = disparityRow[j];
            mask = maskRow[j];

            // do not run invalid points
            if ( disparity < 0xFF )
            {
                //if (mask != 0)
                //  printf("MASK: %d @ (%i,%i)\n", mask, i, j);        
            
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( this->camerasystem->triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                PointT point;
                //only fil out for points that are cyan
                if (mask == 0xFF)
                {
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    point.r = disparity;
                    point.g = disparity;
                    point.b = disparity;
                    returnedPoints.push_back(point);
                }
            }
        }
    }
    
    return 0;
}

void Vision3D::run()
{     
    producePointCloud(this->disparityImage, this->filteredLeft, this->cloud);
    this->cloud.header.frame_id = "map";
    this->pointCloudPublisher.publish(this->cloud);
    this->cloud.clear();
    ros::spinOnce();
}


