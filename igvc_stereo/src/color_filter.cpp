#include <iostream>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/gpu/gpu.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "color_filter.h"

ColorFilter::ColorFilter()
{
	this->max_delt = 15;
	this->min_val = 150;
	this->R_H_Max = 40; // 203
	this->R_H_Min = 230;
	this->R_S_Max = 255;
	this->R_S_Min = 190;
	this->R_V_Max = 255;
	this->R_V_Min = 102;
	this->B_H_Max = 200;
	this->B_H_Min = 150;
	this->B_S_Max = 255;
	this->B_S_Min = 150;
	this->B_V_Max = 255;
	this->B_V_Min = 50;

	this->thresh_val = 200; // 203
	this->erosion_size = 3; // 2
	this->h_rho = 1; // 1
	this->h_theta = 180; // 180
	this->h_thresh = 30; // 40
	this->h_minLineLen = 21; // 20
	this->h_maxLineGap = 20; // 30
	this->lower_limit=118;
	this->upper_limit=250;


	dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig>::CallbackType cb;
	cb = boost::bind(&ColorFilter::configCallback, this, _1, _2);
	dr_srv_.setCallback(cb);
}

void ColorFilter::configCallback(igvc_stereo::color_filter_paramsConfig &config, uint32_t level)
{
  // Set class variables to new values. They should match what is input at the dynamic reconfigure GUI.
	max_delt = config.groups.lines.max_delt_param;
	min_val = config.groups.lines.min_val_param;

	R_H_Max = config.groups.flags.R_H_Max_param; // 203
	R_H_Min = config.groups.flags.R_H_Min_param;
	R_S_Max = config.groups.flags.R_S_Max_param;
	R_S_Min = config.groups.flags.R_S_Min_param;
	R_V_Max = config.groups.flags.R_V_Max_param;
	R_V_Min = config.groups.flags.R_V_Min_param;
	B_H_Max = config.groups.flags.B_H_Max_param;
	B_H_Min = config.groups.flags.B_H_Min_param;
	B_S_Max = config.groups.flags.B_S_Max_param;
	B_S_Min = config.groups.flags.B_S_Min_param;
	B_V_Max = config.groups.flags.B_V_Max_param;
	B_V_Min = config.groups.flags.B_V_Min_param;

	thresh_val=config.groups.filter.thresh_val_param;
	erosion_size=config.groups.filter.erosion_size_param;
	h_rho=config.groups.hough.h_rho_param;
	h_theta=config.groups.hough.h_theta_param;
	h_thresh=config.groups.hough.h_thresh_param;
	h_minLineLen=config.groups.hough.h_minLineLen_param;
	h_maxLineGap=config.groups.hough.h_maxLineGap_param;
	lower_limit=config.groups.hough.lowerLimit_param;
	upper_limit=config.groups.hough.upperLimit_param;
} // end configCallback()

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
cv::gpu::GpuMat ColorFilter::findLines(const cv::Mat& src_image)
{
    this->original_image.upload(src_image);
    // Convert the BGR image to Gray scale
    cv::gpu::cvtColor(this->original_image, this->gray_image, CV_BGR2GRAY);

    // Reduce resolution of image
    cv::gpu::GaussianBlur(this->gray_image, this->blur_image, cv::Size(7, 7), 0.0, 0.0,
        cv::BORDER_DEFAULT);

    // Threshold the image
    cv::gpu::threshold(this->blur_image, this->thresh_image, this->thresh_val, 1,
        cv::THRESH_TOZERO);

    // Erode the image
    cv::Mat element = getStructuringElement(
        cv::MORPH_ELLIPSE, cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
        cv::Point(erosion_size, erosion_size));
    cv::gpu::erode(this->thresh_image, this->eroded_image, element);

    // Canny edge detection TODO: make Canny values dynamic
    //cv::gpu::Canny(this->eroded_image, this->canny_image, 50, 250, 3);

    return this->eroded_image;
    /*/ Prevent any divide by zero errors
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

    return hough_image;*/
}


int ColorFilter::rgb2hsv(uint32_t rgb){
	uint8_t B = (rgb >> 16) & 0x0000ff;
	uint8_t G = (rgb >> 8) & 0x0000ff;
	uint8_t R = (rgb) & 0x0000ff;
	double r = (double)R/255;
	double g = (double)G/255;
	double b = (double)B/255;
	//printf("r: %d, g: %d, b: %d\n", R, G, B);
	double min, max, delta, h, s, v;
	int H, S, V;

	min = r < g ? r : g;
	min = min < b ? min : b;
	
	max = r > g ? r : g;
	max = max > b ? max : b;

	v = max;
	delta = max - min;
	if (max == 0) return 0;
	if (delta == 0) ++delta; 
	s = delta/max;

	if (r == max) h = (g - b)/delta;
	else if (g == max) h = 2.0 + (b - r)/delta;
	else if (b == max) h = 4.0 + (r - g)/delta;

	h *= 60;

	if (h < 0) h = h + 360.0;
	h = h*255.0/360.0;
	H = int(h); S = int(s*255); V = int(v*255);
	//printf("h: %d, s: %d, v: %d\n", H, S, V);

	return (H << 16)|(S << 8)|(V);
}

bool ColorFilter::checkWhite(float rgb) {
	uint32_t color_uint = *(uint32_t*) & rgb;
	unsigned char* color_uchar = (unsigned char*) &color_uint;
	color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
	uint8_t B = (color_uint >> 16) & 0x0000ff;
	uint8_t G = (color_uint >> 8) & 0x0000ff;
	uint8_t R = (color_uint) & 0x0000ff;
	int max, min, mid, delt;

	max = B > G ? B : G;
	max = max > R ? max : R;

	min = B < G ? B : G;
	min = min < R ? min : R;
	
	if (B >= G && B <= R) mid = B;
	else if (G <= B && G >= R) mid = G;
	else mid = R;

	delt = (max-mid) > (mid-min) ? (max-mid) : (mid-min);
	return (delt <= this->max_delt && mid > this->min_val);
}

bool ColorFilter::checkRed(float rgb) {
	uint32_t color_uint = *(uint32_t*) & rgb;
	unsigned char* color_uchar = (unsigned char*) &color_uint;
	color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
	int hsv = rgb2hsv(uint32_t(color_uint));
	int h, s, v;
	h = (hsv >> 16) & 0x0000ff;
	s = (hsv >> 8) & 0x0000ff;
	v = (hsv) & 0x0000ff;

	return ((h < this->R_H_Max || h > this->R_H_Min) && s > R_S_Min && v > R_V_Min);
}

bool ColorFilter::checkBlu(float rgb) {
	uint32_t color_uint = *(uint32_t*) & rgb;
	unsigned char* color_uchar = (unsigned char*) &color_uint;
	color_uint = ((uint32_t) color_uchar[0] << 16 | (uint32_t) color_uchar[1] << 8 | (uint32_t) color_uchar[2]);
	int hsv = rgb2hsv(uint32_t(color_uint));
	int h, s, v;
	h = (hsv >> 16) & 0x0000ff;
	s = (hsv >> 8) & 0x0000ff;
	v = (hsv) & 0x0000ff;

	return (h < this->B_H_Max && h > this->B_H_Min && s > B_S_Min && v > B_V_Min);
}

/**
 * @brief whiteline_filter::displayOriginal Use OpenCV imShow to display the
 *Original image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayOriginal()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->original_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
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
 *Threshold image in a window
 *
 * This function reduces the size of the picture to 400x300
 */
void ColorFilter::displayThreshold()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->thresh_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
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
void ColorFilter::displayEroded()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->eroded_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
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
void ColorFilter::displayCanny()
{
    try {
        // Show the images in a window for debug purposes
	cv::Mat orgImage;
	this->canny_image.download(orgImage);
        cv::Mat disImage;
        cv::resize(orgImage, disImage, cv::Size(400, 300));
        cv::imshow("Canny Edge Image", disImage);
        cv::waitKey(3);
    }
    catch (cv::Exception& e) {
        const char* err_msg = e.what();
        std::cout << "exception caught: " << err_msg << std::endl;
    }
}


