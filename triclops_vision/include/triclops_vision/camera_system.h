#ifndef CAMERA_SYSTEM_H
#define CAMERA_SYSTEM_H

#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>

#include "triclops_vision/typedefs.h"
#include "triclops_vision/common.h"
#include "triclops_vision/line_filter.h"

class CameraSystem {
    public:
        CameraSystem(int argc, char** argv);
        // configue camera to capture image
        void run();
        int configureCamera( FC2::Camera &camera );
        // generate Triclops context from connected camera
        int generateTriclopsContext( FC2::Camera     & camera,
                                     TriclopsContext & triclops );
        // capture image from connected camera
        int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage );
        // convert image to BRGU
        int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );
        // convert image to BRG
        int convertToBGR( FC2::Image & image, FC2::Image & convertedImage );
        // generate triclops input necessary to carry out stereo processing
        int generateTriclopsInput( FC2::Image const & grabbedImage,
                                   ImageContainer   & imageContainer,
                                   TriclopsInput    & colorData,
                                   TriclopsInput    & stereoData);
        // generare triclops input
        int generateTriclopsInput( FC2::Image const & grabbedImage,
                                   ImageContainer   & imageCont,
                                   TriclopsInput    & triclopsInput );
        // carry out stereo processing pipeline
        int doStereo( TriclopsContext const & triclops,
                       TriclopsInput  const & stereoData,
                       TriclopsImage16      & depthImage );
        // save 3d points generated from stereo processing
        int save3dPoints( FC2::Image      const & grabbedImage,
                          TriclopsContext const & triclops,
                          TriclopsImage16 const & disparityImage16,
                          TriclopsInput   const & colorData );
        // save 3d points generated from stereo processing
        int get3dPoints(  PointCloud       & returnPoints,
                          FC2::Image      const & grabbedImage,
                          TriclopsContext const & triclops,
                          TriclopsImage16 const & disparityImage16,
                          TriclopsInput   const & colorData);

        // color process the image and convert to monochrome
        int convertColorToMonoImage( FC2::Image & colorImage,
                                     FC2::Image & monoImage );
        TriclopsContext triclops;
        
        private:
            FC2::Camera camera;
            FC2::Image grabbedImage;
            TriclopsInput color;
            TriclopsInput mono;
            TriclopsImage16 disparityImageTriclops;
            cv::Mat disparityImageCV;
            image_transport::Publisher image_pub_left;
            image_transport::Publisher image_pub_right;
            image_transport::Publisher image_pub_disparity;
	    ros::Rate *loop_rate;
            
};

#endif // CAMERA_SYSTEM_H
