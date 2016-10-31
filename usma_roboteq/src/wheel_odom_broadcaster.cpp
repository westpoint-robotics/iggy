#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/NavSatFix.h>

std::string base_link;



void poseCallback(const sensor_msgs::NavSatFix::ConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(-1.2, -1.2, -msg->altitude) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "utm"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_tf_broadcaster");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("gps/fix", 1, poseCallback);

  ros::spin();
  return 0;
};
