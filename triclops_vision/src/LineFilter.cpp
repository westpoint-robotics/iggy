#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "triclops_vision/LineFilter.h"

LineFilter::LineFilter():
    thresh_val(222), // 225
    erosion_size(5), // 2
    h_rho(1), // 1
    h_theta(180), // 180
    h_thresh(30), // 40
    h_minLineLen(20), // 20
    h_maxLineGap(7) // 30
{
  // Create control sliders that allow tunning of the parameters for line detection
  cv::namedWindow("ControlView", CV_WINDOW_AUTOSIZE);
  cv::createTrackbar( "Threshold Value", "ControlView", &thresh_val, 255);
  cv::createTrackbar( "Erosion Size", "ControlView", &erosion_size, 25);
  cv::createTrackbar( "h_rho", "ControlView", &h_rho, 25);
  cv::createTrackbar( "h_theta", "ControlView", &h_theta, 360);
  cv::createTrackbar( "h_thresh", "ControlView", &h_thresh, 255);
  cv::createTrackbar( "minLineLen", "ControlView", &h_minLineLen, 250);
  cv::createTrackbar( "maxLineGap", "ControlView", &h_maxLineGap, 250);
}

LineFilter::~LineFilter()
{
    cvDestroyAllWindows();
}

void LineFilter::findLines(cv::Mat &src_image, cv::Mat &rtrn_image, cv::vector<cv::Vec4i> &lines)
{
    original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(src_image, gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::GaussianBlur(gray_image, blur_image, cv::Size(7,7), 0.0, 0.0, cv::BORDER_DEFAULT);

    // Threshold the image
    cv::threshold(blur_image, thresh_image, thresh_val, 1, cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ), cv::Point( erosion_size, erosion_size ) );
    cv::erode(thresh_image, eroded_image, element);

    // Canny edge detection
    cv::Canny(eroded_image, canny_image, 50, 250, 3);

    // Prevent any divide by zero errors
    // TODO Find it if there is a better way to do avoid these values being zero.
    if (h_rho == 0)
    {
        h_rho =1;
    }
    if (h_theta == 0)
    {
        h_theta =1;
    }
    if (h_thresh == 0)
    {
        h_thresh =1;
    }

    // Find the Hough lines
    //cv::vector<cv::Vec4i> lines;

    cv::HoughLinesP(canny_image, lines, h_rho, (CV_PI/h_theta), h_thresh, h_minLineLen, h_maxLineGap);
    hough_image = cv::Mat::zeros(canny_image.size(), canny_image.type());
    cyan_image = src_image.clone();

    // Draw the Hough lines on the image
    for( int i =0; i< lines.size(); i++)
    {
        line(hough_image, cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]), cv::Scalar(255,255,0),3,8);
        line(cyan_image, cv::Point(lines[i][0],lines[i][1]),cv::Point(lines[i][2],lines[i][3]), cv::Scalar(255,255,0),5,8);
//        char numstr[40];
//        sprintf(numstr, "Line%d s: %d,%d e: %d,%d", i, int(lines[i][0]),int(lines[i][1]),int(lines[i][2]),int(lines[i][3]));
//        putText(cyan_image, numstr, cv::Point(10,40*i+30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
    }

    // Return the final image with just the white lines in it
    rtrn_image = cyan_image;
}



// Use OpenCV imShow to display the Original image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayOriginal()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(original_image, disImage, cv::Size(400,300));
    cv::imshow("Original Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Grayscale image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayGrayScale()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(gray_image, disImage, cv::Size(400,300));
    cv::imshow("Grayscale Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Blurred image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayBlurred()
{
  try
  {
    cv::Mat disImage;
    cv::resize(blur_image, disImage, cv::Size(400,300));
    cv::imshow("Blurred Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Threshold image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayThreshold()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(thresh_image, disImage, cv::Size(400,300));
    cv::imshow("Threshold Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Eroded image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayEroded()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(eroded_image, disImage, cv::Size(400,300));
    cv::imshow("Eroded Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Canny Edge image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayCanny()
{
  try
  {
    // Show the images in a window for debug purposes
    cv::Mat disImage;
    cv::resize(canny_image, disImage, cv::Size(400,300));
    cv::imshow("Canny Edge Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Hough Lines image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayHough()
{
  try
  {
    cv::Mat disImage;
    cv::resize(hough_image, disImage, cv::Size(400,300));
    cv::imshow("Hough Lines Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}

// Use OpenCV imShow to display the Hough Lines image in a window
// This function reduces the size of the picture to 400x300
void LineFilter::displayCyan()
{
  try
  {
    cv::Mat disImage;
    cv::resize(cyan_image, disImage, cv::Size(400,300));
    cv::imshow("Blue Lines Image", disImage);
    cv::waitKey(3);
  }
  catch (cv::Exception& e)
  {
      const char* err_msg = e.what();
      std::cout << "exception caught: " << err_msg << std::endl;
  }
}
