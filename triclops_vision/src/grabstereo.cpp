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
// grabstereo
//
// Gets input from the Bumblebee, and performs stereo processing
// to create a disparity image. A rectified image from the reference camera
// and the disparity image are both written out.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "fc2triclops.h"
#include "triclops_vision/grabStereo.h"

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

int generateTriclopsContext( FC2::Camera & camera, 
                         TriclopsContext & triclops )
{
	FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}
   
	FC2T::ErrorType fc2TriclopsError; 
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, 
	                                                &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
    }
	
	return 0;
}

int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage )
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

// generare triclops input
int generateTriclopsInput( FC2::Image const & grabbedImage, 
                           ImageContainer   & imageCont,
                           TriclopsInput    & triclopsInput )
{
   
    FC2::Error      fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError   te;
 
    FC2::Image * tmpImage = imageCont.tmp;
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



int doStereo( TriclopsContext const & triclops, 
                TriclopsInput const & triclopsInput )
{
    TriclopsImage disparityImage, edgeImage, rectifiedImage;
    TriclopsError te;
   
    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>(&triclopsInput) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Do stereo processing
    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );
   
    // Retrieve the disparity image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_DISPARITY, 
                          TriCam_REFERENCE, 
						  &disparityImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    // Retrieve the rectified image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_RECTIFIED, 
                          TriCam_REFERENCE, 
						  &rectifiedImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    // Retrieve the edge image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_EDGE, 
                          TriCam_REFERENCE, 
						  &edgeImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
   
    // Save the disparity, reference and edge images

    const char * pDisparityFilename = "testdisparity.pgm";
    te = triclopsSaveImage( &disparityImage, const_cast<char *>(pDisparityFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    const char * pEdgeFilename = "testedge.pgm";
    te = triclopsSaveImage( &edgeImage, const_cast<char *>(pEdgeFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    const char * pRectifiedFilename = "testrectified.pgm";
    te = triclopsSaveImage( &rectifiedImage, const_cast<char *>(pRectifiedFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );
   
    return 0;
}

