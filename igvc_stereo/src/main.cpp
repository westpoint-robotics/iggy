///////////////////////////////////////////////////////////////////////////
// Main file for publishing point clouds with the ZED Camera on the 
// Jetson TK1 board. This file is responsible for initilizing the ZED 
// camera, ROS node, and filter classes. 
// The main purpose of this file is to filter the ZED disparity images for 
// white lines and red/blue flags, and then publish the resulting point 
// cloud to ROS topics for use by the IGVC navigation.
///////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ctime>
#include <chrono>
#include <thread>
#include <mutex>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/gpu/gpu.hpp>

// 2016 Pipeline includes
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "color_filter.h"

#include <zed/Camera.hpp>
#include <zed/utils/GlobalDefine.hpp>

#ifdef _WIN32
#undef max
#undef min
#endif

#include <sensor_msgs/PointCloud2.h>
#include <pcl/common/common_headers.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cuda.h>
#include <cuda_runtime_api.h>

using namespace sl::zed;
using namespace std;


Camera* zed;
SENSING_MODE dm_type = STANDARD;


int main(int argc, char** argv) {
    /*
    Initialize ZED Camera
    */
    printf("Initializing ZED\n");
    zed = new Camera(VGA); //HD720);

    sl::zed::InitParams params;
    params.mode = PERFORMANCE;
    params.unit = METER;
    params.coordinate = RIGHT_HANDED;
    params.verbose = true;
    ERRCODE err = zed->init(params);
    cout << errcode2str(err) << endl;
    if (err != SUCCESS) {
        delete zed;
        return 1;
    }
    int width = zed->getImageSize().width;
    int height = zed->getImageSize().height;
    int size = width*height;

    printf("Initializing ROS\n");
    /*
    Set up ROS variables
    - node handler
    - sensor messages
    - topics
    - publishers
    - spin rate
    */

    ros::init(argc, argv, "zed_ros_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_flags("rb_flag");
    ColorFilter color_filter;
    sensor_msgs::PointCloud2 output_red;
    sensor_msgs::PointCloud2 output_blue;
    sensor_msgs::PointCloud2 output_white;
    string red_cloud_topic = "point_cloud/red_cloud";
    string blue_cloud_topic = "point_cloud/blue_cloud";
    string white_cloud_topic = "point_cloud/white_cloud";
    std::string point_cloud_frame_id = "/zed_current_frame";
    ros::Publisher pub_red_cloud = nh.advertise<sensor_msgs::PointCloud2> (red_cloud_topic, 2);
    ros::Publisher pub_blue_cloud = nh.advertise<sensor_msgs::PointCloud2> (blue_cloud_topic, 2);
    ros::Publisher pub_white_cloud = nh.advertise<sensor_msgs::PointCloud2> (white_cloud_topic, 2);
    ros::Rate loop_rate(30);

    printf("Allocating Data\n");
    /*
    Initialize remaining variables
    - Point clouds
    - cv::Mat image type
    */
    pcl::PointCloud<pcl::PointXYZRGBA> red_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> blue_cloud;
    pcl::PointCloud<pcl::PointXYZRGBA> white_cloud;
    int point_step;
    int row_step;
    Mat gpu_cloud;
    float* cpu_cloud = new float[height*width*4];
    float color;
    int index4 = 0;
    pcl::PointXYZRGBA point;
    red_cloud.clear();
    blue_cloud.clear();
    white_cloud.clear();
    cv::Vec3b wl_point;
    cv::gpu::GpuMat image;//(height, width, CV_8UC4, 1);
    cv::Mat host_image;//(height, width, CV_8UC4, 1);
    cv::Mat cloud_image;

    printf("Entering main loop\n");
    while (nh.ok()) {
	clock_t start = clock();
        if (!zed->grab(dm_type)) { 
		index4 = 0;

		//memcpy(cpu_cloud, zed->retrieveMeasure(MEASURE::XYZBGRA).data, width * height * sizeof (float) * 4);
		//Get image from ZED using the gpu buffer
		gpu_cloud = zed->retrieveMeasure_gpu(MEASURE::XYZBGRA); 	
		//Get size values for retrieved image 	
		point_step = gpu_cloud.channels * gpu_cloud.getDataSize(); 	
		row_step = point_step * width; 	
		//Create a cpu buffer for the image 	
		cpu_cloud = (float*) malloc(row_step * height); 	
		//Copy gpu buffer into cpu buffer 	
		cudaError_t err = cudaMemcpy2D( 	
			cpu_cloud, row_step, gpu_cloud.data, gpu_cloud.getWidthByte(), 	
			row_step, height, cudaMemcpyDeviceToHost 	
		);

		//19% of execution time before here
		
		//Filter the image for white lines and red/blue flags
		cv::cvtColor(slMat2cvMat(zed->retrieveImage(sl::zed::SIDE::LEFT)), host_image, CV_RGBA2RGB);

		cv::gpu::GpuMat cv_filteredImage = color_filter.findLines(host_image);
		cv_filteredImage.download(cloud_image);

		//printf("Cloud height: %d\n", height);
		//printf("filtered height: %d\n", cloud_image.rows);
//		color_filter.displayOriginal();
//		color_filter.displayThreshold();
//		color_filter.displayEroded();
		//color_filter.displayCanny();
		//color_filter.displayRedThreshold();
		//color_filter.displayBluThreshold();

		//86% of execution time before here
		
		clock_t buff_time = clock();
		
        	//Iterate through points in cloud
        	for (index4 = 0; index4 < size*4; index4+=4) {

			//Check if point exists in red image
			if (color_filter.checkRed(cpu_cloud[index4+3])){
				point.y = -cpu_cloud[index4];
				point.z = cpu_cloud[index4+1];
	        		point.x = -cpu_cloud[index4+2];
	        		point.rgb = cpu_cloud[index4+3];
				red_cloud.push_back(point);
			}
			//Check if point exists in blue image
			else if (color_filter.checkBlu(cpu_cloud[index4+3])) {	
				point.y = -cpu_cloud[index4];
				point.z = cpu_cloud[index4+1];
				point.x = -cpu_cloud[index4+2];
				point.rgb = cpu_cloud[index4+3];
				blue_cloud.push_back(point);
			}
			//Check if point exists in white-line image
			else if (color_filter.checkWhite(cpu_cloud[index4+3])){
				wl_point = cloud_image.at<cv::Vec3b>(index4/4/width, index4/4%width);
				if (wl_point[0] != 0){
					point.y = -cpu_cloud[index4];
			        	point.z = cpu_cloud[index4+1];
			        	point.x = -cpu_cloud[index4+2];
			        	point.rgb = cpu_cloud[index4+3];
					white_cloud.push_back(point);
				}
			}


		}
		clock_t gen_cloud = clock();
		free(cpu_cloud);
		//Publish Red Flag Point Cloud
		if (red_cloud.height == 0) {
			red_cloud.push_back(pcl::PointXYZRGBA());
		}

		pcl::toROSMsg(red_cloud, output_red); // Convert the point cloud to a ROS message
		output_red.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
		output_red.header.stamp = ros::Time::now();
		output_red.is_bigendian = false;
		output_red.is_dense = false;
		pub_red_cloud.publish(output_red);
		red_cloud.clear();
        	//Publish Blue Flag Point Cloud
		if (blue_cloud.height == 0) {
			blue_cloud.push_back(pcl::PointXYZRGBA());
		}
		pcl::toROSMsg(blue_cloud, output_blue);
        	output_blue.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
       		output_blue.header.stamp = ros::Time::now();
       		output_blue.is_bigendian = false;
       		output_blue.is_dense = false;
       		pub_blue_cloud.publish(output_blue);
		blue_cloud.clear();
        	//Publish White Line Point Cloud
		if (white_cloud.height == 0){
			white_cloud.push_back(pcl::PointXYZRGBA());
		}
		pcl::toROSMsg(white_cloud, output_white); // Convert the point cloud to a ROS message
       		output_white.header.frame_id = point_cloud_frame_id; // Set the header values of the ROS message
       		output_white.header.stamp = ros::Time::now();
       		output_white.is_bigendian = false;
       		output_white.is_dense = false;
       		pub_white_cloud.publish(output_white);
		white_cloud.clear();
		//Spin once updates dynamic_reconfigure values
		ros::spinOnce();
		loop_rate.sleep();
		clock_t fin = clock();
		//printf("Loop Time:\t %f \n", ((double)fin - start)/CLOCKS_PER_SEC);
		//printf("Buffer Time:\t %f \n", ((double)buff_time-start)/CLOCKS_PER_SEC);
		//printf("Cloud Time:\t %f \n", ((double)gen_cloud-buff_time)/CLOCKS_PER_SEC);
	    }
    }
    delete zed;
    return 0;
}
/*
Before:
Loop Time:	 0.115063 
Buffer Time:	 0.092423 
Cloud Time:	 0.020680 
After:
Loop Time:	 0.095858 
Buffer Time:	 0.076028 
Cloud Time:	 0.017682 

Loop Time:	 0.053562 
Buffer Time:	 0.018948 
Cloud Time:	 0.033982 
*/
