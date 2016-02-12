#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <pcl_ros/point_cloud.h>

#include "triclops_vision/typedefs.h"
#include "triclops_vision/triclops_camera.h"

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

// generare triclops input
int generateTriclopsInput( FC2::Image const & grabbedImage,
                           ImageContainer   & imageCont,
                           TriclopsInput    & triclopsInput )
{

    FC2::Error      fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError   te;

    FC2::Image * tmpImage = imageCont.bgru;
    FC2::Image * unprocessedImage = imageCont.unprocessed;

    // Convert the pixel interleaved raw data to de-interleaved and color processed data
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                                   grabbedImage,
                                                                   true /*assume little endian*/,
                                   tmpImage[RIGHT],
                                   tmpImage[LEFT] );

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
            return FC2T::handleFc2TriclopsError(fc2TriclopsError,
                                                   "unprocessedRawOrMono16Image()");
    }

    // check if the unprocessed image is color
    if ( tmpImage[0].GetBayerTileFormat() != FC2::NONE )
    {
            for ( int i = 0; i < 2; ++i )
            {
                    if ( convertColorToMonoImage(tmpImage[i], unprocessedImage[i]) )
                    {
                            return 1;
                    }
            }
    }
    else
    {
        unprocessedImage[RIGHT] = tmpImage[RIGHT];
        unprocessedImage[LEFT]  = tmpImage[LEFT];
    }

    // pack image data into a TriclopsInput structure
    te = triclopsBuildRGBTriclopsInput(
               grabbedImage.GetCols(),
               grabbedImage.GetRows(),
               grabbedImage.GetCols(),
               (unsigned long)grabbedImage.GetTimeStamp().seconds,
               (unsigned long)grabbedImage.GetTimeStamp().microSeconds,
               unprocessedImage[RIGHT].GetData(),
               unprocessedImage[LEFT].GetData(),
               unprocessedImage[LEFT].GetData(),
               &triclopsInput);

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

    // save monochrome generated right and left image
    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rightGreyImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("leftGreyImage.pgm", &pgmOpt);

    return 0;
}



int generateTriclopsInput( FC2::Image const & grabbedImage,
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput,
                            TriclopsInput   & triclopsMonoInput)
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

        for ( int i = 0; i < 2; ++i )
        {
            if ( convertToBGR(unprocessedImage[i], bgrImage[i]) )
            {
                return 1;
            }
        }

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

    // Save the interpolated depth image
    /*char const * pDispFilename = "disparity16.pgm";
    te = triclopsSaveImage16( &depthImage, const_cast<char *>(pDispFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );
    */
    return 0;
}

int get3dPoints( PointCloud      & returnedPoints,
                 FC2::Image      const & grabbedImage,
                 TriclopsContext const & triclops,
                 TriclopsImage16 const & disparityImage16,
                 TriclopsInput   const & colorData )
{
   TriclopsImage monoImage = {0};
   TriclopsColorImage colorImage = {0};
   TriclopsError te;

   float            x, y, z;
   int	            r, g, b;
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
                if ( z < 5.0)
                //if ( z < 5.0 and y > 0.3)
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
    }
    return 0;
}


int convertColorToMonoImage( FC2::Image & colorImage, FC2::Image & monoImage )
{
    FC2::Error fc2Error;
    fc2Error = colorImage.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = colorImage.Convert(FC2::PIXEL_FORMAT_MONO8,
                                                 &monoImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}
