#ifndef LINE_FILTER_H
#define LINE_FILTER_H
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/image_publisher.h"
#include "triclops_vision/common.h"

/**
 * @brief The LineFilter class. This class identifies white lines in a OpenCv image
 *
 */
class LineFilter
{
    public:
        LineFilter(int argc, char** argv);
        virtual ~LineFilter();
        void run();

        void findLines(const cv::Mat &src_image, cv::Mat &rtrn_image, cv::vector<cv::Vec4i> &lines);
        void findPointsOnLines(const cv::Mat &cImage, const cv::vector<cv::Vec4i> &lines, std::vector<cv::Point2i> &pixels);

        void displayOriginal();
        void displayGrayScale();
        void displayBlurred();
        void displayThreshold();
        void displayEroded();
        void displayCanny();
        void displayHough();
        void displayCyan();

        void returnCyan(cv::Mat cyanImage)
        { cyanImage = cyan_image;}
        void imageCallbackL(const sensor_msgs::ImageConstPtr& msgL);
        void imageCallbackR(const sensor_msgs::ImageConstPtr& msgR);
        cv::vector<cv::Vec4i> lines;
        

    protected:
    private:
        cv::Mat original_image;   // The original source image
        cv::Mat gray_image;       // The image after it is converted to grayscale
        cv::Mat blur_image;       // The image after it a blur effect is applied
        cv::Mat thresh_image;     // The image after the colors are categorized by defined threshold values
        cv::Mat eroded_image;     // The image after an erosion filter is applied
        cv::Mat canny_image;      // The image after canny edge detection is complete
        cv::Mat hough_image;      // The image with the hugh line transform is applied
        cv::Mat cyan_image;       // The image after all detected white lines are drawn in cyan color

        int thresh_val;           // The threshold value used to identify white in the image
        int erosion_size;         // The  size of our kernel to erode the image with
        int h_rho;                // Distance resolution of the accumulator in pixels (hough transform)
        int h_theta;              // Angle resolution of the accumulator in radians (hough transform)
        int h_thresh;             // Accumulator threshold parameter. Only those lines are returned that get enough votes (hough transform)
        int h_minLineLen;         // Line segments shorter than that are rejected (hough transform)
        int h_maxLineGap;         // Maximum allowed gap between points on the same line to link them (hough transform)

        image_transport::Publisher image_pub_filtered_right;
        image_transport::Publisher image_pub_filtered_left;

        image_transport::Subscriber subcamright;
        image_transport::Subscriber subcamleft;

	ros::Rate *loop_rate;
	ros::AsyncSpinner *async_wheels;
	
};

#endif // LINE_FILTER_H
