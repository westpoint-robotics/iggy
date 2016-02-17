#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <triclops/triclops.h>
#include <triclops/fc2triclops.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include "triclops_vision/typedefs.h"
#include "triclops_vision/vision_3d.h"
#include "triclops_vision/triclops_opencv.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/image_publisher.h"
#include "triclops_vision/camera_system.h"

int main (int argc, char ** argv) {
	ros::init(argc,argv,"triclops_vision");
	ros::NodeHandle nh;
  	ros::Rate loop_rate(10);

	CameraSystem camera(argc,argv);
	LineFilter linefilter(argc,argv);

	while (ros::ok()) {
		camera.run();
		linefilter.run();
		loop_rate.sleep();
	}	
}