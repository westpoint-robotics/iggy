#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "triclops_vision/line_filter.h"
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/image_publisher_single.h"

//Create imagecontainer for moving images
ImageContainer imageContainerL;
cv::Mat cImageL; // An OpenCV version of the rectified stereo image from the camera
cv::Mat filtered_imageL; // The image with detected white lines painted cyan

//Create imagecontainer for moving images
ImageContainer imageContainerR;
cv::Mat cImageR; // An OpenCV version of the rectified stereo image from the camera
cv::Mat filtered_imageR; // The image with detected white lines painted cyan

/**
 * @brief LineFilter::LineFilter Used to find white lines in OpenCv Images.
 *
 */
LineFilter::LineFilter(int argc, char** argv)
{
  this->thresh_val=1; // 225
  this->erosion_size=1; // 2
  this->h_rho=1; // 1
  this->h_theta=180; // 180
  this->h_thresh=30; // 40
  this->h_minLineLen=20; // 20
  this->h_maxLineGap=7; // 30

  // Create control sliders that allow tunning of the parameters for line detection
  cv::namedWindow("ControlView", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar( "Threshold Value", "ControlView", &thresh_val, 255);
  cv::createTrackbar( "Erosion Size", "ControlView", &erosion_size, 25);
  cv::createTrackbar( "h_rho", "ControlView", &h_rho, 25);
  cv::createTrackbar( "h_theta", "ControlView", &h_theta, 360);
  cv::createTrackbar( "h_thresh", "ControlView", &h_thresh, 255);
  cv::createTrackbar( "minLineLen", "ControlView", &h_minLineLen, 250);
  cv::createTrackbar( "maxLineGap", "ControlView", &h_maxLineGap, 250);

  //Start ROS
  ros::init(argc,argv,"linefilter");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);

  //Create publishers
  this->image_pub_filtered_left = it.advertise("/camera/left/linefiltered", 1);
  this->image_pub_filtered_right = it.advertise("/camera/right/linefiltered", 1);

  //Creation of Subscribers, which use callback functions to execute transform and republishing upon receipt of data.
  this->subcamleft = it.subscribe("/camera/left/rgb", 0, &LineFilter::imageCallbackL, this);
  this->subcamright = it.subscribe("/camera/right/rgb", 0, &LineFilter::imageCallbackR, this); 

  //ROS loop that causes the system to keep moving.
  
}

LineFilter::~LineFilter()
{
    cvDestroyAllWindows();
}

void LineFilter::run() {
    ros::spinOnce();
}

//Executable for linefilter when called by launch file, will subscribe to camera left and right nodes, and will publish filtered images for each.

void LineFilter::imageCallbackL(const sensor_msgs::ImageConstPtr& msg)
{
    //Pull subscribed data inside this callback, formatting for linefilter use based on original vision_3d code
    cImageL = cv_bridge::toCvShare(msg, "bgr8")->image;

    //Execute filtration, map to new image filtered image
    LineFilter::findLines(cImageL, filtered_imageL, this->lines);

    sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_imageL).toImageMsg();
    outmsg->header.frame_id="bumblebee2";
    outmsg->header.stamp = ros::Time::now();

    /*DEBUG
    cv::imshow("Filter Left", filtered_imageL);
    cv::waitKey(10);*/

    this->image_pub_filtered_left.publish(outmsg);
}

void LineFilter::imageCallbackR(const sensor_msgs::ImageConstPtr& msg)
{
    
    //Pull subscribed data inside this callback, formatting for linefilter use based on original vision_3d code
    cImageR = cv_bridge::toCvShare(msg, "bgr8")->image;
    //Execute filtration, map to new image filtered image
    LineFilter::findLines(cImageR, filtered_imageR, this->lines);

    sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage(std_msgs::Header(), "mono8", filtered_imageR).toImageMsg();
    outmsg->header.frame_id="bumblebee2";
    outmsg->header.stamp = ros::Time::now();

    /*DEBUG
    cv::imshow("Filter Right", filtered_imageR);
    cv::waitKey(10);*/

    this->image_pub_filtered_right.publish(outmsg);
}

/**
 * @brief LineFilter::findLines This function finds the white lines in the src_image
 * @param src_image the original image to find white lines in
 * @param rtrn_image the original image with cyan lines drawn where the white lines were detected
 * @param lines a vector of start and end points for each line found
 *
 *  It Uses the following algorithm to find white lines:
 *     1. turn image into grayscale
 *     2. blur the image
 *     3. run it through a threshold filter using THRESH_TO_ZERO mode
 *     4. run it through an erosion filter
 *     5. run it through a Canny edge detector
 *     6. finally, take this processed image and find the lines using Probabilistic Hough Transform HoughLinesP
 */
void LineFilter::findLines(const cv::Mat &src_image, cv::Mat &rtrn_image, cv::vector<cv::Vec4i> &lines)
{
    this->original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(src_image, this->gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::GaussianBlur(this->gray_image, this->blur_image, cv::Size(7,7), 0.0, 0.0, cv::BORDER_DEFAULT);

    // Threshold the image
    cv::threshold(this->blur_image, this->thresh_image, this->thresh_val, 1, cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
    cv::erode(this->thresh_image, this->eroded_image, element);

    // Canny edge detection
    cv::Canny(this->eroded_image, this->canny_image, 50, 250, 3);

    // Prevent any divide by zero errors
    // TODO Find it if there is a better way to do avoid these values being zero.
    if (this->h_rho == 0)
    {
        this->h_rho =1;
    }
    if (this->h_theta == 0)
    {
        this->h_theta =1;
    }
    if (this->h_thresh == 0)
    {
        this->h_thresh =1;
    }

    // Find the Hough lines
    cv::HoughLinesP(this->canny_image, lines, this->h_rho, (CV_PI/this->h_theta), this->h_thresh, this->h_minLineLen, this->h_maxLineGap);
    this->hough_image = cv::Mat::zeros(canny_image.size(), canny_image.type());
    this->cyan_image = src_image.clone();

    // Draw the Hough lines on the image
    for( int i =0; i< lines.size(); i++)
    {
        line(this->hough_image, cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]), cv::Scalar(255,255,0),3,8);
        line(this->cyan_image, cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]), cv::Scalar(255,255,0),5,8);
    }

    // Return the original image with detected white lines drawn in cyan
    //changed to only return hough_image
    this->cyan_image = this->hough_image;
    rtrn_image = this->cyan_image;
}

/**
 * @brief LineFilter::findPointsOnLines. Finds the x,y coordinates of each point on line defined by an start and end point.
 * @param cImage The image that lines exist in
 * @param lines A list of lines defined by start and end points
 * @param pixels A list of pixels that are on the lines.
 */
void LineFilter::findPointsOnLines(const cv::Mat &cImage, const cv::vector<cv::Vec4i> &lines, std::vector<cv::Point2i> &pixels)
{
  cv::Point pt1;
  cv::Point pt2;
  for ( int i = 0; i < lines.size(); i++)
    {
      pt1.x = lines[i][0];
      pt1.y = lines[i][1];
      pt2.x = lines[i][2];
      pt2.y = lines[i][3];
      cv::LineIterator it(cImage, pt1, pt2, 8);
      for(int j = 0; j < it.count; j++, ++it){
          pixels.push_back(cv::Point2i(it.pos().x,it.pos().y));
      }
    }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Original image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayOriginal()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(this->original_image, disImage, cv::Size(400,300));
    cv::imshow("Original Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Grayscale image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayGrayScale()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(this->gray_image, disImage, cv::Size(400,300));
    cv::imshow("Grayscale Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Blurred image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayBlurred()
{
  try
  {
    cv::Mat disImage;
    cv::resize(this->blur_image, disImage, cv::Size(400,300));
    cv::imshow("Blurred Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayThreshold()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(this->thresh_image, disImage, cv::Size(400,300));
    cv::imshow("Threshold Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Eroded image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayEroded()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(this->eroded_image, disImage, cv::Size(400,300));
    cv::imshow("Eroded Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Canny Edge image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayCanny()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(this->canny_image, disImage, cv::Size(400,300));
    cv::imshow("Canny Edge Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Hough Lines image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayHough()
{
  try
  {
    cv::Mat disImage;
    cv::resize(this->hough_image, disImage, cv::Size(400,300));
    cv::imshow("Hough Lines Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

/**
 * @brief LineFilter::displayOriginal Use OpenCV imShow to display the Cyan Lined image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void LineFilter::displayCyan()
{
  try
  {
    cv::Mat disImage;
    cv::resize(this->cyan_image, disImage, cv::Size(400,300));
    cv::imshow("Blue Lines Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}


