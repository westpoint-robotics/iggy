#ifndef COLOR_FILTER_H
#define COLOR_FILTER_H

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
// Dynamic reconfigure includes.
#include <dynamic_reconfigure/server.h>
// Auto-generated from cfg/ directory.
#include <igvc_stereo/color_filter_paramsConfig.h>

class ColorFilter {
public:
	ColorFilter();

	//! Callback function for dynamic reconfigure server.
	void configCallback(igvc_stereo::color_filter_paramsConfig &config, uint32_t level);
	cv::Mat findLines(const cv::Mat& src_image);

	int rgb2hsv(uint32_t rgb);
	bool checkRed(float rgb);
	bool checkBlu(float rgb);
	bool checkWhite(float rgb);

	void displayOriginal();
	void displayThreshold();
	void displayEroded();
	void displayCanny();

private:
	//! Dynamic reconfigure server.
	dynamic_reconfigure::Server<igvc_stereo::color_filter_paramsConfig> dr_srv_;

	// 2016 Includes
	std::vector<cv::Vec4i> lines;
	std::vector<cv::Point2i> pixels;
	int thresh_val; // The threshold value used to identify white in the image
	int erosion_size; // The  size of our kernel to erode the image with
	int h_rho; // Distance resolution of the accumulator in pixels (hough transform)
	int h_theta; // Angle resolution of the accumulator in radians (hough transform)
	int h_thresh; // Accumulator threshold parameter. Only those lines are returned that get enough votes (hough transform)
	int h_minLineLen; // Line segments shorter than that are rejected (hough transform)
	int h_maxLineGap; // Maximum allowed gap between points on the same line to link them (hough transform)
	cv::Mat original_image; // The original source image
	cv::Mat gray_image; // The image after it is converted to grayscale
	cv::Mat blur_image; // The image after it a blur effect is applied
	cv::Mat thresh_image; // The image after the colors are categorized by defined threshold values
	cv::Mat eroded_image; // The image after an erosion filter is applied
	cv::Mat canny_image; // The image after canny edge detection is complete
	cv::Mat hough_image; // The image with the hugh line transform is applied
	cv::Mat cyan_image; // The image after all detected white lines are drawn in cyan color
	int lower_limit; // The image after all detected white lines are drawn in cyan color
	int upper_limit; // The image after all detected white lines are drawn in cyan color

	int max_delt;
	int min_val;
	int R_H_Max; // 203
	int R_H_Min;
	int R_S_Max;
	int R_S_Min;
	int R_V_Max;
	int R_V_Min;
	int B_H_Max;
	int B_H_Min;
	int B_S_Max;
	int B_S_Min;
	int B_V_Max;
	int B_V_Min;
};

#endif // WHITELINE_FILTER_H


