#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "triclops_vision/vision_3d.h"
#include "triclops_vision/line_filter.h"
#include "triclops_vision/camera_system.h"

int main (int argc, char **argv) {
	ros::init(argc,argv,"triclops_vision");
	ros::NodeHandle nh;
	ros::Rate loop_rate(5);

	CameraSystem camera(argc,argv);
	LineFilter linefilter(argc,argv);
    	Vision3D vision3D(argc, argv, &camera);

	while (ros::ok()) {
		camera.run();
		ros::spinOnce();
	  	linefilter.run();
		ros::spinOnce();
         	vision3D.run();
		ros::spinOnce();
		loop_rate.sleep();
	}
}
