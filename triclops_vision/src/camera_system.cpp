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
#include "triclops_vision/image_publisher.h"
#include "triclops_vision/camera_system.h"


//TODO: Move this back to a seperate file
int convertTriclops2Opencv(TriclopsImage16 & bgrImage,
                           cv::Mat & cvImage){
  cvImage = cv::Mat(bgrImage.nrows, bgrImage.ncols, CV_16UC1, bgrImage.data,bgrImage.rowinc);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, bgrImage.rowinc);
  putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

CameraSystem::CameraSystem(int argc, char** argv) {
  namespace FC2 = FlyCapture2;
  namespace FC2T = Fc2Triclops;
  TriclopsContext triclops;


  this->camera.Connect();
  // configure camera - Identifies what camera is being used?


  if ( configureCamera(this->camera) )
  {
      exit(-1);
  }

  // generate the Triclops context
  if ( generateTriclopsContext( this->camera, this->triclops ) )
  {
      exit(-1);
  }

  triclopsSetRectify(this->triclops, true);
  //triclopsSetDisparity(this->triclops, 5, 60);
  triclopsSetResolution(this->triclops, 480,640);
  //triclopsSetStereoMask(this->triclops, 13);
  //triclopsSetDisparityMapping(this->triclops, 1, 1);

  //Use only for visuals?
  //triclopsSetDisparityMapping(this->triclops, 128, 255);

  // Par 1 of 2 for grabImage method
  FC2::Error fc2Error = this->camera.StartCapture();
  if (fc2Error != FC2::PGRERROR_OK)
  {
      exit(FC2T::handleFc2Error(fc2Error));
  }

  // Get the camera info and print it out
  FC2::CameraInfo camInfo;
  fc2Error = this->camera.GetCameraInfo( &camInfo );
  if ( fc2Error != FC2::PGRERROR_OK )
  {
      std::cout << "Failed to get camera info from camera" << std::endl;
      exit(-1);
  }
  /*else
  {
      ROS_INFO(">>>>> CAMERA INFO  Vendor: %s     Model: %s     Serail#: %d \n", camInfo.vendorName, camInfo.modelName, camInfo.serialNumber);
  }*/

  ros::init(argc, argv, "camera");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  // Container of Images used for processing
  image_transport::ImageTransport it(nh);
  //Publishers for the camera
  this->image_pub_left = it.advertise("/camera/left/rgb", 1);
  this->image_pub_right = it.advertise("/camera/right/rgb", 1);
  this->image_pub_disparity = it.advertise("/camera/disparity", 1);
}

//Copied over from older files.
int CameraSystem::configureCamera( FC2::Camera & camera )
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

//Copied over from older files.
int CameraSystem::convertToBGRU( FC2::Image & image, FC2::Image & convertedImage )
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

//Copied over from older files.
int CameraSystem::convertToBGR( FC2::Image & image, FC2::Image & convertedImage )
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

// generate Triclops context from connected camera. Copied over from older files.
int CameraSystem::generateTriclopsContext( FC2::Camera     & camera,
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

//Copied over from older files.
int CameraSystem::generateTriclopsInput( FC2::Image const & grabbedImage, 
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput,
                            TriclopsInput   & triclopsMonoInput ) 
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

    FC2::Image * monoImage = imageContainer.mono;

    // check if the unprocessed image is color
    if ( unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE )
    {
        FC2::Image * bgruImage = imageContainer.bgru;

        for ( int i = 0; i < 2; ++i )
        {
            if ( this->convertToBGRU(unprocessedImage[i], bgruImage[i]) )
            {
                return 1;
            }
        }

        FC2::PNGOption pngOpt;
        pngOpt.interlaced = false;
        pngOpt.compressionLevel = 9;

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

//        packedColorImage.Save("packedColorImage.png",&pngOpt );

        for ( int i = 0; i < 2; ++i )
        {
            fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);
            if (fc2Error != FlyCapture2::PGRERROR_OK)
            {
                return Fc2Triclops::handleFc2Error(fc2Error);
            }
        }
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

int CameraSystem::doStereo( TriclopsContext const & triclops, 
               TriclopsInput  const & stereoData, 
               TriclopsImage16      & depthImage )
{
    TriclopsError te;

    // Set subpixel interpolation on to use 
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>(&stereoData) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Do stereo processing
    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );
   
    // Retrieve the interpolated depth image from the context
    te = triclopsGetImage16( triclops, 
                            TriImg16_DISPARITY, 
                            TriCam_REFERENCE, 
                            &depthImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    return 0;
}

void CameraSystem::run() {
    FC2::Error fc2Error;
    ImageContainer imageContainer;

    // this image contains both right and left images
    fc2Error = this->camera.RetrieveBuffer(&(this->grabbedImage));
    if (fc2Error != FC2::PGRERROR_OK)
    {
      exit(FC2T::handleFc2Error(fc2Error));
    }

    // generate triclops inputs from grabbed image
    if ( this->generateTriclopsInput( this->grabbedImage, imageContainer, this->color, this->mono ) )
    {
      exit(EXIT_FAILURE);
    }

    doStereo(this->triclops, this->mono, this->disparityImageTriclops);
      
    convertTriclops2Opencv(this->disparityImageTriclops, this->disparityImageCV);

    // DEBUG
    cv::imshow("Disparity", this->disparityImageCV);
    cv::waitKey(10);

    // Publish images
    ImagePublisher imagePublisher(this->grabbedImage, imageContainer, &(this->image_pub_left), &(this->image_pub_right));

    sensor_msgs::ImagePtr outmsg = cv_bridge::CvImage(std_msgs::Header(), "mono16", this->disparityImageCV).toImageMsg();
    this->image_pub_disparity.publish(outmsg);

    ros::spinOnce();
}




