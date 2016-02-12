#ifndef IMAGE_PUBLISHER_SINGLE_H
#define IMAGE_PUBLISHER_SINGLE_H
#include <stdio.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "triclops_vision/vision_3d.h"
#include "triclops_vision/common.h"

#include <image_transport/image_transport.h>


/**
 * @brief The ImagePublisher class. More details coming soon!
 *
 */
class ImagePublisherS
        //Document this code with UML diagram generator online
        //Have diagram of desired outcome.
{
    public:

        ImagePublisherS(FC2::Image grabbedImage, ImageContainer imageContainer, image_transport::Publisher *image_pub_main);

    private:
        cv::Mat      mainImage;
};

#endif // IMAGE_PUBLISHER_SINGLE_H


