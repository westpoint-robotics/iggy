#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include "triclops_vision/typedefs.h"
#include "triclops_vision/vision_3d.h"
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/image_publisher.h"
#include "triclops_vision/camera_system.h"
#include <image_transport/image_transport.h>


ImagePublisherS::ImagePublisherS(FC2::Image grabbedImage, ImageContainer imageContainer, image_transport::Publisher *image_pub_main){

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                            grabbedImage,
                            true /*assume little endian*/,
                            unprocessedImage[main]);
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
      {
          FC2T::handleFc2TriclopsError(fc2TriclopsError,"unpackUnprocessedRawOrMono16Image");
      }// FlyCapture2::Image& unprocessedImage[RIGHT];

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    //DML get main image into opencv Mat
    for ( int i = 0; i < 2; ++i )
    {
        if ( convertToBGR(unprocessedImage[i], imageContainer.bgr[i]) )
        {     1;     }
    }
    // convert images to OpenCV Mat
    cv::Mat      mainImage;


    //ROS_INFO(">>>>> Data Size Bytes: %d \n",imageContainer.bgr[main].GetDataSize());
    unsigned int rowBytes = (double)imageContainer.bgr[main].GetDataSize()/(double)imageContainer.bgr[main].GetRows();
    //ROS_INFO(">>>>> ROW Bytes: %d rows %d cols %d\n",rowBytes,imageContainer.bgr[main].GetRows(), imageContainer.bgr[main].GetCols());
    mainImage = cv::Mat(imageContainer.bgr[main].GetRows(), imageContainer.bgr[main].GetCols(), CV_8UC3, imageContainer.bgr[main].GetData(),rowBytes);


    // Uncomment the below 3 lines to have opencv display the images
    //cv::imshow("Image Right", rightImage);
    //cv::imshow("Image main", mainImage);
    //cv::waitKey(3);


    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mainImage).toImageMsg();
    image_pub_main->publish(msg);

}




