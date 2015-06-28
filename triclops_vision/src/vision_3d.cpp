
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

#include <fc2triclops.h>
#include <triclops.h>
#include <stdio.h>
#include <stdlib.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "triclops_vision/typedefs.h"
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/vision_3d.h"

LineFilter lf;

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

int generateTriclopsInput( FC2::Image const & grabbedImage, 
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
            if ( convertToBGRU(unprocessedImage[i], bgruImage[i]) )
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

int doStereo( TriclopsContext const & triclops, 
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

int gets3dPoints( FC2::Image      const & grabbedImage,
                  TriclopsContext const & triclops,
                  TriclopsImage16 const & disparityImage16,
                  TriclopsInput   const & colorData,
                  PointCloud      & returnedPoints)
{
    TriclopsImage monoImage = {0};
    TriclopsColorImage colorImage = {0};
    TriclopsColorImage filterImage = {0};
    TriclopsError te;

    float            x, y, z;
    int	            r, g, b;
    int	             pixelinc ;
    int	             i, j, k;
    unsigned short * row;
    unsigned short   disparity;

    cv::Mat cImage; // An OpenCV version of the rectified stereo image from the camera
    cv::Mat filtered_image; // The image with detected white lines painted cyan
    cv::vector<cv::Vec4i> lines; // The detected white lines

    // Rectify the color image
    te = triclopsRectifyColorImage( triclops,
                                    TriCam_REFERENCE,
                                        const_cast<TriclopsInput *>(&colorData),
                                        &colorImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );

    // Convert the image to OpenCv and detect white lines
    convertTriclops2Opencv(colorImage, cImage);
    lf.findLines(cImage, filtered_image, lines);
    lf.displayCanny();
    lf.displayCyan();

    //Find all pixels in the image that should be marked as obstacles.
    cv::vector<cv::Point2i> obstaclePixels;
    cv::vector<cv::Point3i> obstacleCoords;
    cv::Point3i obstacleCoord;
    lf.findPointsOnLines(cImage, lines, obstaclePixels);

    for (int i = 0; i != obstaclePixels.size();i++)
      {
        cv::Point2i cPix = obstaclePixels[i];
        int pixX = cPix.x;
        int pixY = cPix.y;
        triclopsRCD16ToXYZ( triclops, pixX, pixY, disparity, &x, &y, &z );
        obstacleCoord.x = x;
        obstacleCoord.y = y;
        obstacleCoord.z = z;
        obstacleCoords.push_back(obstacleCoord);
      }
    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...
    // Determine the number of pixels spacing per row
    pixelinc = disparityImage16.rowinc/2;
    for ( i = 0, k = 0; i < disparityImage16.nrows; i++ )
    {
        row = disparityImage16.data + i * pixelinc;
        for ( j = 0; j < disparityImage16.ncols; j++, k++ )
        {
            disparity = row[j];

            // do not save invalid points
            if ( disparity < 0xFF00 )
            {
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                if ( z < 5.0 )
                {
                        b = filtered_image.at<cv::Vec3b>(i,j)[0];
                        g = filtered_image.at<cv::Vec3b>(i,j)[1];
                        r = filtered_image.at<cv::Vec3b>(i,j)[2];
                 }
                    PointT point;
                    point.x = z;
                    point.y = -x;
                    point.z = -y;
                    point.r = r;
                    point.g = g;
                    point.b = b;
                    //TODO currently throwing out row and column. Investigate if they are needed.
                    //point.i = i;
                    //point.j = j;
                    returnedPoints.push_back(point);
                }
            }
        }
    return 0;
}

int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData )
{
    TriclopsImage monoImage = {0};
    TriclopsColorImage colorImage = {0};
    TriclopsError te;

    float            x, y, z; 
    int	            r, g, b;
    FILE             * pPointFile;
    int              nPoints = 0;
    int	             pixelinc ;
    int	             i, j, k;
    unsigned short * row;
    unsigned short   disparity;

    // Rectify the color image if applicable
    bool isColor = false;
    if ( grabbedImage.GetPixelFormat() == FC2::PIXEL_FORMAT_RAW16 )
    {
        isColor = true;
        te = triclopsRectifyColorImage( triclops, 
                                        TriCam_REFERENCE, 
	                                    const_cast<TriclopsInput *>(&colorData), 
	                                    &colorImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
    }
    else
    {
        te = triclopsGetImage( triclops,
	                            TriImg_RECTIFIED,
	                            TriCam_REFERENCE,
	                            &monoImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    }

  
    // Save points to disk
    const char * pFilename = "out.pts";
    pPointFile = fopen( pFilename, "w+" );
    if ( pPointFile != NULL )
    {
        printf("Opening output file %s\n", pFilename);
    }
    else
    {
        printf("Error opening output file %s\n", pFilename);
        return 1;
    }

    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...

    // Determine the number of pixels spacing per row
    pixelinc = disparityImage16.rowinc/2;
    for ( i = 0, k = 0; i < disparityImage16.nrows; i++ )
    {
        row = disparityImage16.data + i * pixelinc;
        for ( j = 0; j < disparityImage16.ncols; j++, k++ )
        {
            disparity = row[j];

            // do not save invalid points
            if ( disparity < 0xFF00 )
            {
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                if ( z < 5.0 )
                {
                    if ( isColor )
                    {
                        r = (int)colorImage.red[k];
                        g = (int)colorImage.green[k];
                        b = (int)colorImage.blue[k];		  
                    }
                    else
                    {
                        // For mono cameras, we just assign the same value to RGB
                        r = (int)monoImage.data[k];
                        g = (int)monoImage.data[k];
                        b = (int)monoImage.data[k];
                    }
                    fprintf( pPointFile, "%f %f %f %d %d %d %d %d\n", x, y, z, r, g, b, i, j );
                    nPoints++;
                }
            }
        }
    }
    fclose( pPointFile );
    printf( "Points in file: %d\n", nPoints );

    return 0;
}

