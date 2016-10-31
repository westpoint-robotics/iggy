
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
    this->nh = new ros::NodeHandle();
    image_transport::ImageTransport it(*(this->nh));

    this->lineCloudPublisher = this->nh->advertise<sensor_msgs::PointCloud2>("/vision3D/lines", 1);
    this->redCloudPublisher = this->nh->advertise<sensor_msgs::PointCloud2>("/vision3D/red", 1);
    this->blueCloudPublisher = this->nh->advertise<sensor_msgs::PointCloud2>("/vision3D/blue", 1);

    this->subcamdisp = it.subscribe("/camera/disparity", 1, &Vision3D::visionCallBackDisparity, this);
    //this->subcamfilteredright = it.subscribe("/camera/right/linefiltered", 0, &Vision3D::visionCallBackFilteredRight, this);
    //this->subcamfilteredrightred = it.subscribe("/camera/right/redFilter", 0, &Vision3D::visionCallBackFilteredRightRed, this);
    //this->subcamfilteredrightblue = it.subscribe("/camera/right/blueFilter", 0, &Vision3D::visionCallBackFilteredRightBlue, this);
    this->subcamfilteredleft = it.subscribe("/camera/left/linefiltered", 1, &Vision3D::visionCallBackFilteredLeft, this);
    this->subcamfilteredleftred = it.subscribe("/camera/left/redFilter", 1, &Vision3D::visionCallBackFilteredLeftRed, this);
    this->subcamfilteredleftblue = it.subscribe("/camera/left/blueFilter", 1, &Vision3D::visionCallBackFilteredLeftBlue, this);


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
    //printf("Copying disparity over...\n");
    this->disparityImage = cv_bridge::toCvCopy(msg, "mono16")->image;
}

/*void Vision3D::visionCallBackFilteredRight(const sensor_msgs::ImageConstPtr& msg)
{
    this->filteredRight = cv_bridge::toCvCopy(msg, "mono8")->image;
}

void Vision3D::visionCallBackFilteredRightRed(const sensor_msgs::ImageConstPtr& msg)
{
    this->filteredRightRed = cv_bridge::toCvCopy(msg, "mono8")->image;
}

void Vision3D::visionCallBackFilteredRightBlue(const sensor_msgs::ImageConstPtr& msg)
{
    this->filteredRightBlue = cv_bridge::toCvCopy(msg, "mono8")->image;
}*/

void Vision3D::visionCallBackFilteredLeft(const sensor_msgs::ImageConstPtr& msg)
{
    //printf("Copying left line filtered\n");
    this->filteredLeft = cv_bridge::toCvCopy(msg, "mono8")->image;
}

void Vision3D::visionCallBackFilteredLeftRed(const sensor_msgs::ImageConstPtr& msg)
{
    //printf("Copying left line red filtered\n");
    this->filteredLeftRed = cv_bridge::toCvCopy(msg, "mono8")->image;
}

void Vision3D::visionCallBackFilteredLeftBlue(const sensor_msgs::ImageConstPtr& msg)
{
    //printf("Copying left line blue filtered\n");
    this->filteredLeftBlue = cv_bridge::toCvCopy(msg, "mono8")->image;
}

/*void Vision3D::visionCallBackRGBRight(const sensor_msgs::ImageConstPtr& msg)
{
    this->imageRight = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::cvtColor(this->imageRight, this->imageRight, CV_BGR2GRAY);
}

void Vision3D::visionCallBackRGBLeft(const sensor_msgs::ImageConstPtr& msg)
{
    this->imageLeft = cv_bridge::toCvCopy(msg, "bgr8")->image;
    cv::cvtColor(this->imageLeft, this->imageLeft, CV_BGR2GRAY);
}*/

int Vision3D::producePointCloud(  cv::Mat const &disparityImage,
                cv::Mat const &maskImage,
                cv::Mat const &redImage,
                cv::Mat const &blueImage,
                PointCloud      &linePoints,
                PointCloud      &redPoints,
                PointCloud      &bluePoints)
{
    float            x, y, z;
    int              i=0, j=0, k;
    //unsigned char   disparity;
    //unsigned char    mask;

    unsigned short   disparity;
    unsigned char    mask, red, blue;
    //Resolution Checking will follow

    if (redImage.cols <= 0 || blueImage.cols <= 0 || maskImage.cols <= 0 || disparityImage.cols <= 0) {
        //printf("[!] Not enough cols, leaving...\n");
        return -1;
    }

   //printf("[!] Searching through image at %p for obstacles..mask %d,%d,%d,%d,%d and dispar %d,%d,%d,%d,%d\n", &redImage, redImage.cols, redImage.rows,int(redImage.step), redImage.channels(), int(redImage.elemSize()), disparityImage.cols,disparityImage.rows,int(disparityImage.step),disparityImage.channels(),int(disparityImage.elemSize()) );
    for ( i = 0; i < disparityImage.rows; i++ )
    {
        for ( j = 0; j < disparityImage.cols; j++ )
        {
	    if (i%2 && j%2) continue;

            //disparity = disparityRow[j];
            //mask = maskRow[j];
            disparity = disparityImage.at<unsigned short>(i,j);
            mask = maskImage.at<unsigned char>(i,j);
            red = redImage.at<unsigned char>(i,j);
            blue = blueImage.at<unsigned char>(i,j);

            //convert from disparity to pointcloud
            triclopsRCD16ToXYZ( this->camerasystem->triclops, i, j, disparity, &x, &y, &z );

            // look at points within a range
            PointT pointLine, pointRed, pointBlue;
            //only fil out for points that are cyan
            if (mask != 0 && i > 200)
            {
                //std::cout << "mask and disparity: " << int(mask) << " and " << disparity << std::endl;
                pointLine.x = z;
                pointLine.y = -x;
                pointLine.z = -y;
                pointLine.r = 0x00;
                pointLine.g = 0x00;
                pointLine.b = 0x00;
                linePoints.push_back(pointLine);
            }

            if (red != 0)
            {
                //std::cout << "mask and disparity: " << int(mask) << " and " << disparity << std::endl;
                pointRed.x = z;
                pointRed.y = -x;
                pointRed.z = -y;
                pointRed.r = 0xFF;
                pointRed.g = 0x00;
                pointRed.b = 0x00;
                redPoints.push_back(pointRed);
            }

            if (blue != 0)
            {
                //std::cout << "mask and disparity: " << int(mask) << " and " << disparity << std::endl;
                pointBlue.x = z;
                pointBlue.y = -x;
                pointBlue.z = -y;
                pointBlue.r = 0x00;
                pointBlue.g = 0x00;
                pointBlue.b = 0xFF;
                bluePoints.push_back(pointBlue);
            }
            mask = 0;
            red = 0;
            blue = 0;
        }
    }
    
    return 0;
}

void Vision3D::run()
{
    //printf("Producing point cloud\n");
    producePointCloud(this->disparityImage, this->filteredLeft, this->filteredLeftRed, this->filteredLeftBlue, this->lineCloud, this->redCloud, this->blueCloud);
    this->lineCloud.header.frame_id = "bumblebee2";
    this->redCloud.header.frame_id = "bumblebee2";
    this->blueCloud.header.frame_id = "bumblebee2";
    this->lineCloudPublisher.publish(this->lineCloud);
    this->redCloudPublisher.publish(this->redCloud);
    this->blueCloudPublisher.publish(this->blueCloud);
    this->lineCloud.clear();
    this->redCloud.clear();
    this->blueCloud.clear();
}











