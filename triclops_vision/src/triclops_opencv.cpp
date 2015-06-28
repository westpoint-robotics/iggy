#include <stdio.h>
#include <stdlib.h>

#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "triclops_vision/typedefs.h"

//TODO converting to grabstereo instead of 3d points.h
// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(FC2::Image & bgrImage,
                           cv::Mat & cvImage){
  // convert bgr image to OpenCV Mat
  unsigned int rowBytes = (double)bgrImage.GetReceivedDataSize()/(double)bgrImage.GetRows();
  cvImage = cv::Mat(bgrImage.GetRows(), bgrImage.GetCols(), CV_8UC4, bgrImage.GetData(),rowBytes);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d rowbytes: %d", cvImage.rows,cvImage.cols, rowBytes);
  putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(TriclopsInput & bgrImage,
                           cv::Mat & cvImage){
   //convert bgr image to OpenCV Mat
      cv::Mat R(bgrImage.nrows,bgrImage.ncols,CV_8UC1,bgrImage.u.rgb.red,bgrImage.rowinc);
      cv::Mat B(bgrImage.nrows,bgrImage.ncols,CV_8UC1,bgrImage.u.rgb.blue,bgrImage.rowinc);
      cv::Mat G(bgrImage.nrows,bgrImage.ncols,CV_8UC1,bgrImage.u.rgb.green,bgrImage.rowinc);
      std::vector<cv::Mat> array_to_merge;
      array_to_merge.push_back(B);
      array_to_merge.push_back(G);
      array_to_merge.push_back(R);
      cv::merge(array_to_merge,cvImage);
      char numstr[50];
      sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, bgrImage.rowinc);
      putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
    //cvImage = cv::Mat(bgrImage.nrows, bgrImage.ncols, CV_8UC3, bgrImage.u.rgb,bgrImage.rowinc);
}

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(TriclopsImage & bgrImage,
                           cv::Mat & cvImage){
  cvImage = cv::Mat(bgrImage.nrows, bgrImage.ncols, CV_8UC3, bgrImage.data,bgrImage.rowinc);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, bgrImage.rowinc);
  putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

// convert a triclops color image to opencv mat
int convertTriclops2Opencv(TriclopsImage16 & bgrImage,
                           cv::Mat & cvImage){
  cvImage = cv::Mat(bgrImage.nrows, bgrImage.ncols, CV_16UC1, bgrImage.data,bgrImage.rowinc);
  char numstr[50];
  sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, bgrImage.rowinc);
  putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}


// convert a triclops color image to opencv mat
int convertTriclops2Opencv(TriclopsColorImage & bgrImage,
                           cv::Mat & cvImage){
  //convert bgr image to OpenCV Mat
     cv::Mat R(bgrImage.nrows,bgrImage.ncols,CV_8UC1,bgrImage.red,bgrImage.rowinc);
     cv::Mat B(bgrImage.nrows,bgrImage.ncols,CV_8UC1,bgrImage.blue,bgrImage.rowinc);
     cv::Mat G(bgrImage.nrows,bgrImage.ncols,CV_8UC1,bgrImage.green,bgrImage.rowinc);
     std::vector<cv::Mat> array_to_merge;
     array_to_merge.push_back(B);
     array_to_merge.push_back(G);
     array_to_merge.push_back(R);
     cv::merge(array_to_merge,cvImage);
//     ROS_INFO("c %d r %d rInc: %d",cvImage.cols,cvImage.rows,bgrImage.rowinc);
     char numstr[50];
     sprintf(numstr, "rows: %d cols: %d RowInc: %d", cvImage.rows,cvImage.cols, bgrImage.rowinc);
     putText(cvImage, numstr, cv::Point(10,cvImage.rows-30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,100,250), 1, false);
}

int convertOpencv2Triclops( cv::Mat & cvImage,
                             TriclopsColorImage & bgrImage)
{\
  int from_to[] = {0,0};
  cv::Mat b(cvImage.rows,cvImage.cols,CV_8UC1);
  int bytes = sizeof(b.data);
  cv::mixChannels(&cvImage, 1, &b, 1, from_to, 1);
  from_to[0]= 1;
  cv::Mat g(cvImage.rows,cvImage.cols,CV_8UC1);
  cv::mixChannels(&cvImage, 1, &g, 1, from_to, 1);
  from_to[0]= 2;
  cv::Mat r(cvImage.rows,cvImage.cols,CV_8UC1);
  cv::mixChannels(&cvImage, 1, &r, 1, from_to, 1);
//  int bytes = r.elemSize();
//  unsigned int imgStride = cvImage.step;
  unsigned int imgStride = cvImage.cols;
  bgrImage.ncols=cvImage.cols;
  bgrImage.nrows=cvImage.rows;
  bgrImage.rowinc=imgStride;
  bgrImage.blue = b.data;
  bgrImage.green = g.data;
  bgrImage.red = r.data;// r is 76800 bytes long. sizeof r is 96 size of r.data is 8
  ROS_INFO("c %d r %d stp: %d %zu sffride: %d",cvImage.cols,cvImage.rows,r.size().width,sizeof(b.data),imgStride);
}

// convert an Opencv into a triclops color image
int convertOpencv2Triclops( cv::Mat & cvImage,
                             FC2::Image & bgrImage)
{\
  unsigned int imgStride = cvImage.step;
  int sizeImg = cvImage.rows*cvImage.cols*int(cvImage.elemSize());
  bgrImage.SetDimensions(cvImage.rows,cvImage.cols, imgStride,FC2::PIXEL_FORMAT_BGR, FC2::NONE);
  bgrImage.SetData(cvImage.data,sizeImg);
}

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



  //    cv::Mat R(rectifiedImage.nrows,rectifiedImage.ncols,CV_8UC1,rectifiedImage.red,rectifiedImage.rowinc);
  //    cv::Mat B(rectifiedImage.nrows,rectifiedImage.ncols,CV_8UC1,rectifiedImage.blue,rectifiedImage.rowinc);
  //    cv::Mat G(rectifiedImage.nrows,rectifiedImage.ncols,CV_8UC1,rectifiedImage.green,rectifiedImage.rowinc);
  //    std::vector<cv::Mat> array_to_merge;
  //    array_to_merge.push_back(B);
  //    array_to_merge.push_back(G);
  //    array_to_merge.push_back(R);
  //    cv::Mat rectImage;
  //    cv::merge(array_to_merge,rectImage);


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
