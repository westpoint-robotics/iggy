#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "triclops_vision/typedefs.h"


// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(FC2::Image & bgrImage,
                           cv::Mat & cvImage){
  // convert bgr image to OpenCV Mat
  unsigned int rowBytes = (double)bgrImage.GetReceivedDataSize()/(double)bgrImage.GetRows();
  cvImage = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC3, bgrImage.GetData(),rowBytes);
}

// convert an Opencv into a triclops color image
int convertOpencv2Triclops( cv::Mat & cvImage,
                             FC2::Image & bgrImage)
{\
  unsigned int imgStride = cvImage.step;
  int sizeImg = cvImage.rows*cvImage.cols*int(cvImage.elemSize());
  bgrImage.SetDimensions(cvImage.rows,cvImage.cols, imgStride,FC2::PIXEL_FORMAT_BGR, FC2::NONE);
  bgrImage.SetData(cvImage.data,sizeImg);
/*
  FC2::PNGOption pngOpt;
          pngOpt.interlaced = false;
          pngOpt.compressionLevel = 9;

  FC2::Error fc2Error = bgrImage.Save("rightconverted.png", &pngOpt);
  if (fc2Error != FC2::PGRERROR_OK)
  {
      ROS_INFO("FC2 ERROR HERE error here");
      return FC2T::handleFc2Error(fc2Error);
  }
  else
    {
    ROS_INFO("No fc2 error here");
  }
*/

}
/*
        virtual Error SetData(
            const unsigned char* pData,
            unsigned int         dataSize );

Image(
    unsigned int    rows,
    unsigned int    cols,
    unsigned int    stride,
    unsigned char*  pData,
    unsigned int    dataSize,
    unsigned int    receivedDataSize,
    PixelFormat     format,
    BayerTileFormat bayerFormat = NONE );
    */
