#include <string>
#include <math.h>
#include <stdio.h>

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace enc = sensor_msgs::image_encodings;

/* Try this algorithm: 
(source: http://stackoverflow.com/questions/16665742/a-good-approach-for-detecting-lines-in-an-image)
    1. Grab image from webcam
    2. turn into grayscale
    3. blur the image
    4. Run it through a threshold filter using THRESH_TO_ZERO mode
    5. run it through an erosion filter
    6. run it through a Canny edge detector
    7. finally, take this processed image and find the lines using Probabilistic Hough Transform HoughLinesP
*/

// Line Finding Node for IGVC modified by MAJ Larkin(DML) on 8JAN2015.
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  int thresh_val;
  int erosion_size;
  int h_rho;
  int h_theta;
  int h_thresh; 
  int h_minLineLen;
  int h_maxLineGap;

  ImageConverter(): // Jorge's prefered values (15 JAN 2015)
    it_(nh_),
    thresh_val(222), // 225
    erosion_size(5), // 2
    h_rho(1), // 1
    h_theta(180), // 180
    h_thresh(50), // 40
    h_minLineLen(20), // 20
    h_maxLineGap(7) // 30

  {
    image_pub_ = it_.advertise("filter_out", 1);
    image_sub_ = it_.subscribe("/camera/right/image_raw", 1, &ImageConverter::imageCb, this);

    cv::namedWindow("Input Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Output Image", CV_WINDOW_AUTOSIZE);
    
    cv::createTrackbar( "Threshold Value", "Input Image", &thresh_val, 255);
    cv::createTrackbar( "Erosion Size", "Input Image", &erosion_size, 25);
    cv::createTrackbar( "h_rho", "Output Image", &h_rho, 25);
    cv::createTrackbar( "h_theta", "Output Image", &h_theta, 360);
    cv::createTrackbar( "h_thresh", "Output Image", &h_thresh, 255);
    cv::createTrackbar( "minLineLen", "Output Image", &h_minLineLen, 250);
    cv::createTrackbar( "maxLineGap", "Output Image", &h_maxLineGap, 250);

  }

  ~ImageConverter()
  {
    cv::destroyWindow("Input Image");
    cv::destroyWindow("Output Image");
    cvDestroyAllWindows();
  }


/*

Split for header

*/
  

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr original_ptr;
    
    // convert the ROS msg into an openCV image
    try
    {
      original_ptr = cv_bridge::toCvShare(msg, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat gray_image;
    cv::Mat blur_image;
    cv::Mat thresh_image;
    cv::Mat eroded_image;
    cv::Mat canny_image;
    cv::Mat hough_image;
    
    // Convert the BGR image to Gray scale
    cvtColor(original_ptr->image, gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::GaussianBlur(gray_image, blur_image, cv::Size(7,7), 0.0, 0.0, cv::BORDER_DEFAULT);

    // Threshold the image
    cv::threshold(blur_image, thresh_image, thresh_val, 1, cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
    cv::erode(thresh_image, eroded_image, element);

    // Canny edge detection
    cv::Canny(eroded_image, canny_image, 50, 250, 3);

    // Find the Hough lines
    cv::vector<cv::Vec4i> lines;
    if (h_rho == 0) {h_rho =1;}
    if (h_theta == 0) {h_theta =1;}
    if (h_thresh == 0) {h_thresh =1;}
    cv::HoughLinesP(canny_image, lines, h_rho, (CV_PI/h_theta), h_thresh, h_minLineLen, h_maxLineGap);       
    hough_image = cv::Mat::zeros(canny_image.size(), canny_image.type());
    
    // Draw the Hough lines on the image
    for( size_t i =0; i< lines.size(); i++)
    {
      line(hough_image, cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]), cv::Scalar(255,255,0),3,8);
    }

    // Show the images in a window for debug purposes
    cv::imshow("Input Image", gray_image);
    cv::imshow("Output Image", hough_image);
    cv::waitKey(3);

    //publish the final product
    cv_bridge::CvImage out;
    out.header = original_ptr->header;
    out.encoding = original_ptr->encoding;
    out.image = hough_image;
    
    // publish the output image    
    image_pub_.publish(out.toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_filter");
  ImageConverter ic;
  ros::spin();
  return 0;
}