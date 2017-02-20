#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "gazebo_iggy_vision/whiteline_filter.h"

#include <dynamic_reconfigure/server.h>
//#include <gazebo_iggy_vision/LinefilterConfig.h>

WhitelineFilter::WhitelineFilter()
{
    this->thresh_val = 183; // 203
    this->erosion_size = 1; // 2
    this->h_rho = 1; // 1
    this->h_theta = 180; // 180
    this->h_thresh = 5; // 40
    this->h_minLineLen = 10; // 20
    this->h_maxLineGap = 25; // 30
    this->lower_limit=0;
    this->upper_limit=255;

    //dynamic_reconfigure::Server<gazebo_iggy_vision::line_filter_paramsConfig>::CallbackType cb;
    //cb = boost::bind(&WhitelineFilter::configCallback, this, _1, _2);
    //dr_srv_.setCallback(cb);
}

//void WhitelineFilter::configCallback(gazebo_iggy_vision::LinefilterConfig &config, uint32_t level)
//{
//    ROS_INFO("Reconfigure for white line filter started.")
//    // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
//    this->thresh_val=config.groups.filter.thresh_val_param;
//    this->erosion_size=config.groups.filter.erosion_size_param;
//    this->h_rho=config.groups.hough.h_rho_param;
//    this->h_theta=config.groups.hough.h_theta_param;
//    this->h_thresh=config.groups.hough.h_thresh_param;
//    this->h_minLineLen=config.groups.hough.h_minLineLen_param;
//    this->h_maxLineGap=config.groups.hough.h_maxLineGap_param;
//    this->lower_limit=config.groups.hough.lowerLimit_param;
//    this->upper_limit=config.groups.hough.upperLimit_param;
//} // end configCallback()

/**
 * @brief WhitelineFilter::findLines This function finds the white lines in the
 * src_image
 * @param src_image the original image to find white lines in
 * @param rtrn_image the original image with cyan lines drawn where the white
 * lines were detected
 * @param lines a vector of start and end points for each line found
 *
 *  It Uses the following algorithm to find white lines:
 *     1. turn image into grayscale
 *     2. blur the image
 *     3. run it through a threshold filter using THRESH_TO_ZERO mode
 *     4. run it through an erosion filter
 *     5. run it through a Canny edge detector
 *     6. finally, take this processed image and find the lines using
 * Probabilistic Hough Transform HoughLinesP
 */
cv::Mat WhitelineFilter::findLines(const cv::Mat& src_image)
{
    this->original_image = src_image;
    // Convert the BGR image to Gray scale
    cvtColor(src_image, this->gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::GaussianBlur(this->gray_image, this->blur_image, cv::Size(7, 7), 0.0, 0.0,
        cv::BORDER_DEFAULT);

    // Threshold the image
    cv::threshold(this->blur_image, this->thresh_image, this->thresh_val, 1,
        cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::erode(this->thresh_image, this->eroded_image, element);

    // Canny edge detection
    cv::Canny(this->eroded_image, this->canny_image, 50, 250, 3);

    // Prevent any divide by zero errors
    if (this->h_rho <= 0) {
        this->h_rho = 1;
    }
    if (this->h_theta <= 0) {
        this->h_theta = 1;
    }
    if (this->h_thresh <= 0) {
        this->h_thresh = 1;
    }

    // Find the Hough lines
    cv::HoughLinesP(this->canny_image, lines, this->h_rho,
        (CV_PI / this->h_theta), this->h_thresh, this->h_minLineLen,
        this->h_maxLineGap);
    this->hough_image = cv::Mat::zeros(canny_image.size(), CV_8UC1);
    this->cyan_image = src_image.clone();

    // Draw the Hough lines on the image
    for (int i = 0; i < lines.size(); i++) {
        line(this->hough_image, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), 255, 3, 8);
        line(this->cyan_image, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(255, 255, 0), 5, 8);
    }
    for (int i=0; i < hough_image.rows;i++){
        for (int j=0; j<hough_image.cols;j++){
            if (i <= this->upper_limit){
                hough_image.at<unsigned char>(i,j)=0;
            }
            else if (i >= this->lower_limit){
                hough_image.at<unsigned char>(i,j)=0;
            }
        }
    }
    return cyan_image;
}

/**
 * @brief whiteline_filter::getPointsOnLines. Finds the x,y coordinates of each
 * point on line defined by an start and end point.
 * @param cImage The image that lines exist in
 * @param lines A list of lines defined by start and end points
 * @param pixels returns a list of pixels that are on the lines.
 */
std::vector<cv::Point2i> WhitelineFilter::getPointsOnLines()
{
    cv::Point pt1;
    cv::Point pt2;
    std::vector<cv::Point2i> pixels;

    for (int i = 0; i < this->lines.size(); i++) {
        pt1.x = this->lines[i][0];
        pt1.y = this->lines[i][1];
        pt2.x = this->lines[i][2];
        pt2.y = this->lines[i][3];
        cv::LineIterator it(this->original_image, pt1, pt2, 8);
        for (int j = 0; j < it.count; j++, ++it) {
            pixels.push_back(cv::Point2i(it.pos().x, it.pos().y));
        }
    }
    return pixels;
}

void WhitelineFilter::filterControl()
{
    // Create control sliders that allow tunning of the parameters for line detection
    cv::namedWindow("ControlView", CV_WINDOW_AUTOSIZE);
    cv::createTrackbar("Threshold Value", "ControlView", &this->thresh_val, 255);
    cv::createTrackbar("Erosion Size", "ControlView", &this->erosion_size, 25);
    cv::createTrackbar("h_rho", "ControlView", &this->h_rho, 25);
    cv::createTrackbar("h_theta", "ControlView", &this->h_theta, 360);
    cv::createTrackbar("h_thresh", "ControlView", &this->h_thresh, 255);
    cv::createTrackbar("minLineLen", "ControlView", &this->h_minLineLen, 250);
    cv::createTrackbar("maxLineGap", "ControlView", &this->h_maxLineGap, 250);
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Original image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayOriginal()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->original_image, disImage, cv::Size(400, 300));
        cv::imshow("Original Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Grayscale image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayGrayScale()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->gray_image, disImage, cv::Size(400, 300));
        cv::imshow("Grayscale Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Blurred image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayBlurred()
{
    try {
        cv::Mat disImage;
        cv::resize(this->blur_image, disImage, cv::Size(400, 300));
        cv::imshow("Blurred Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayThreshold()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->thresh_image, disImage, cv::Size(400, 300));
        cv::imshow("Threshold Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Eroded image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayEroded()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->eroded_image, disImage, cv::Size(400, 300));
        cv::imshow("Eroded Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Canny Edge image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayCanny()
{
    try {
        // Show the images in a window for debug purposes
        cv::Mat disImage;
        cv::resize(this->canny_image, disImage, cv::Size(400, 300));
        cv::imshow("Canny Edge Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Hough Lines image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayHough()
{
    try {
        cv::Mat disImage;
        cv::resize(this->hough_image, disImage, cv::Size(400, 300));
        cv::imshow("Hough Lines Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Cyan Lined image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void WhitelineFilter::displayCyan()
{
    try {
        cv::Mat disImage;
        cv::resize(this->cyan_image, disImage, cv::Size(400, 300));
        cv::imshow("Blue Lines Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}
