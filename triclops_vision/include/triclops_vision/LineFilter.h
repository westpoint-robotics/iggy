#ifndef LINEFILTER_H
#define LINEFILTER_H
/* Using this algorithm:
(source: http://stackoverflow.com/questions/16665742/a-good-approach-for-detecting-lines-in-an-image)
    1. Grab image from webcam
    2. turn into grayscale
    3. blur the image
    4. Run it through a threshold filter using THRESH_TO_ZERO mode
    5. run it through an erosion filter
    6. run it through a Canny edge detector
    7. finally, take this processed image and find the lines using Probabilistic Hough Transform HoughLinesP
*/

class LineFilter
{
    public:
        LineFilter();
        virtual ~LineFilter();
        void findLines(cv::Mat &src_image, cv::Mat &rtrn_image, cv::vector<cv::Vec4i> &lines);
        void displayOriginal();
        void displayGrayScale();
        void displayBlurred();
        void displayThreshold();
        void displayEroded();
        void displayCanny();
        void displayHough();
        void displayBlue();


    protected:
    private:
        cv::Mat original_image;
        cv::Mat gray_image;
        cv::Mat blur_image;
        cv::Mat thresh_image;
        cv::Mat eroded_image;
        cv::Mat canny_image;
        cv::Mat hough_image;
        cv::Mat blue_image;

        int thresh_val;
        int erosion_size;
        int h_rho;
        int h_theta;
        int h_thresh;
        int h_minLineLen;
        int h_maxLineGap;

};

#endif // LINEFILTER_H
