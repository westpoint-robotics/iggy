//=============================================================================
// Copyright © 2004 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// stereoto3dpoints
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written to a file.
//
// This point file can be viewed with PGRView under windows.
//
//=============================================================================
/*
    Modified to add ros by DML on 7MAY2015

    Tested on 08MAY2015 at full speed and averaged publishing 33.5 clouds/sec then
    throttled back to 20 clouds/sec.

    TODO This code is a mess. Use as a prototype and design a clean solution implementing OOP.
*/
#include "triclops.h"

#include "fc2triclops.h"

#include <stdio.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "triclops_vision/typedefs.h"
#include <image_transport/image_transport.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
     "ERROR: %s reported %s.\n", \
     function, \
     triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

bool SHOW_OPENCV_VIDEO = false;

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

struct ImageContainer
{
    FC2::Image unprocessed[2];
    FC2::Image bgru[2];
    FC2::Image mono[2];
    FC2::Image packed;
};

enum IMAGE_SIDE
{
    RIGHT = 0, LEFT
};

// configue camera to capture image
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
                           TriclopsInput    & stereoData,
                           cv::Mat          & leftImageBGR,
                           cv::Mat          & rightImageBGR);

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
                  TriclopsInput   const & colorData );

int main(int  argc, char **argv)
{
    ros::init(argc, argv, "triclopsStereo");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub_left= it.advertise("camera/left/color", 1);
    image_transport::Publisher image_pub_right= it.advertise("camera/right/color", 1);
    ros::Rate loop_rate(30);

    TriclopsInput triclopsColorInput, triclopsMonoInput;
    TriclopsContext triclops;

    FC2::Camera camera;
    FC2::Image grabbedImage;

    camera.Connect();
    // configure camera
    if ( configureCamera( camera ) )
    {
        return EXIT_FAILURE;
    }
    // generate the Triclops context
    if ( generateTriclopsContext( camera, triclops ) )
    {
        return EXIT_FAILURE;
    }

    FC2::Error fc2Error = camera.StartCapture();
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    // Get the camera info and print it out
    FC2::CameraInfo camInfo;
    fc2Error = camera.GetCameraInfo( &camInfo );
    if ( fc2Error != FC2::PGRERROR_OK )
    {
        std::cout << "Failed to get camera info from camera" << std::endl;
        return false;
    }
    else
    {
        ROS_INFO("CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber);
    }

    while (ros::ok())
    {
        //ROS_INFO("Top of the while loop ---------------");
        // grab image from camera.
        // this image contains both right and left images
        fc2Error = camera.RetrieveBuffer(&grabbedImage);
        if (fc2Error != FC2::PGRERROR_OK)
        {
            return FC2T::handleFc2Error(fc2Error);
        }

        // Container of Images used for processing
        ImageContainer imageContainer;
        cv::Mat      leftImage;
        cv::Mat      rightImage;
        // generate triclops inputs from grabbed image
        if (generateTriclopsInput(grabbedImage, imageContainer, triclopsColorInput, triclopsMonoInput, leftImage, rightImage))
        {
            return EXIT_FAILURE;
        }
        if (SHOW_OPENCV_VIDEO)
        {
            cv::imshow("Image Right", rightImage);
            cv::imshow("Image Left", leftImage);
            cv::waitKey(3);
         }

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
        image_pub_left.publish(msg);
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightImage).toImageMsg();
        image_pub_right.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
    // Close the camera and disconnect
    camera.StopCapture();
    camera.Disconnect();

    // Destroy the Triclops context
    TriclopsError te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );

    return 0;
}


int configureCamera( FC2::Camera & camera )
{
    FC2T::ErrorType fc2TriclopsError;
    FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    fc2TriclopsError = FC2T::setStereoMode( camera, mode );
    if ( fc2TriclopsError )
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    return 0;
}


int grabImage ( FC2::Camera & camera, FC2::Image& grabbedImage )
{
    FC2::Error fc2Error = camera.StartCapture();
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = camera.RetrieveBuffer(&grabbedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}


int generateTriclopsContext( FC2::Camera     & camera,
                             TriclopsContext & triclops )
{
    FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    FC2T::ErrorType fc2TriclopsError;
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
                                            "getContextFromCamera");
    }

    return 0;
}

int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int convertToBGR( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGR, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int generateTriclopsInput( FC2::Image const & grabbedImage,
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput,
                            TriclopsInput   & triclopsMonoInput,
                            cv::Mat         & leftImageBGR, // TODO Fix this is a work around to get openCV images available to the big while loop
                            cv::Mat         & rightImageBGR)
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError te;

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                            grabbedImage,
                            true /*assume little endian*/,
                            unprocessedImage[RIGHT],
                            unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError,
                                     "unpackUnprocessedRawOrMono16Image");
    }

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    //unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    //unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);

    FC2::Image * monoImage = imageContainer.mono;

    // check if the unprocessed image is color
    if ( unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE )
    {
        FC2::Image * bgruImage = imageContainer.bgru;
        FC2::Image * bgrImage = imageContainer.bgru;

        for ( int i = 0; i < 2; ++i )
        {
            if ( convertToBGRU(unprocessedImage[i], bgruImage[i]) )
            {
                return 1;
            }
        }

        //DML get left and right image into opencv Mat
        for ( int i = 0; i < 2; ++i )
        {
            if ( convertToBGR(unprocessedImage[i], bgrImage[i]) )
            {
                return 1;
            }
        }
        // convert Left image to OpenCV Mat TODO Find a better way to do this.
        unsigned int rowBytes = (double)bgrImage[LEFT].GetReceivedDataSize()/(double)bgrImage[LEFT].GetRows();
        leftImageBGR = cv::Mat(bgrImage[LEFT].GetRows(), bgrImage[LEFT].GetCols(), CV_8UC3, bgrImage[LEFT].GetData(),rowBytes);

        // convert Right image to OpenCV Mat TODO Find a better way to do this.
        rowBytes = (double)bgrImage[RIGHT].GetReceivedDataSize()/(double)bgrImage[RIGHT].GetRows();
        rightImageBGR = cv::Mat(bgrImage[RIGHT].GetRows(), bgrImage[RIGHT].GetCols(), CV_8UC3, bgrImage[RIGHT].GetData(),rowBytes);

        FC2::PNGOption pngOpt;
        pngOpt.interlaced = false;
        pngOpt.compressionLevel = 9;
        //bgruImage[RIGHT].Save("colorImageRight.png", &pngOpt);
        //bgruImage[LEFT].Save("colorImageLeft.png", &pngOpt);

        FC2::Image & packedColorImage = imageContainer.packed;

        // pack BGRU right and left image into an image
        fc2TriclopsError = FC2T::packTwoSideBySideRgbImage( bgruImage[RIGHT],
                                                            bgruImage[LEFT],
                                                            packedColorImage );

        if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
        {
            return handleFc2TriclopsError(fc2TriclopsError,
                                            "packTwoSideBySideRgbImage");
        }

        // Use the row interleaved images to build up a packed TriclopsInput.
        // A packed triclops input will contain a single image with 32 bpp.
        te = triclopsBuildPackedTriclopsInput( grabbedImage.GetCols(),
                                                grabbedImage.GetRows(),
                                                packedColorImage.GetStride(),
                                                (unsigned long)grabbedImage.GetTimeStamp().seconds,
                                                (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
                                                packedColorImage.GetData(),
                                                &triclopsColorInput );

        _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );


        // the following does not change the size of the image
        // and therefore it PRESERVES the internal buffer!
        packedColorImage.SetDimensions( packedColorImage.GetRows(),
                                        packedColorImage.GetCols(),
                                        packedColorImage.GetStride(),
                                        packedColorImage.GetPixelFormat(),
                                        FC2::NONE);

        //packedColorImage.Save("packedColorImage.png",&pngOpt );

        for ( int i = 0; i < 2; ++i )
        {
            fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);
            if (fc2Error != FlyCapture2::PGRERROR_OK)
            {
                return Fc2Triclops::handleFc2Error(fc2Error);
            }
        }

        //monoImage[RIGHT].Save("monoImageRight.pgm", &pgmOpt);
        //monoImage[LEFT].Save("monoImageLeft.pgm", &pgmOpt);
    }
    else
    {
        monoImage[RIGHT] = unprocessedImage[RIGHT];
        monoImage[LEFT] = unprocessedImage[LEFT];
    }

    // Use the row interleaved images to build up an RGB TriclopsInput.
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput( grabbedImage.GetCols(),
                                        grabbedImage.GetRows(),
                                        grabbedImage.GetCols(),
                                        (unsigned long)grabbedImage.GetTimeStamp().seconds,
                                        (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
                                        monoImage[RIGHT].GetData(),
                                        monoImage[LEFT].GetData(),
                                        monoImage[LEFT].GetData(),
                                        &triclopsMonoInput );

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

    return 0;
}

