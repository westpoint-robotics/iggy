#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
#include <time.h>

ros::Time current_time_encoder;
double DistancePerCount = 0.021927;  //meters per count

//math for measured --> 70.16cm per rotation /32 = .021927 meters per count
//math from calc --> (3.14159265 * 0.2286) / 32;  //meters per count (pie * diameter) / #black+white

double vx;
double vy;//wertr
long _PreviousLeftEncoderCounts = 0;
long _PreviousRightEncoderCounts = 0;



void WheelCallback(const geometry_msgs::Vector3::ConstPtr& ticks)
{

double deltaLeft;
double deltaRight;
double deltaTime;
  ros::Time last_time_encoder = current_time_encoder;
  current_time_encoder = ros::Time::now();

  deltaLeft = ticks->x - _PreviousLeftEncoderCounts;
  deltaRight = ticks->y - _PreviousRightEncoderCounts;

  deltaTime = (current_time_encoder - last_time_encoder).toSec();

  vx = deltaLeft * DistancePerCount / deltaTime;
  vy = deltaRight * DistancePerCount / deltaTime;

  ROS_INFO("vx %f, vy %f, deltaTime %f", vx, vy, deltaTime);

  _PreviousLeftEncoderCounts = ticks->x;
  _PreviousRightEncoderCounts = ticks->y;
}




int main(int argc, char **argv)
{
  ros::init(argc, argv, "odometry_publisher");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("enc_raw", 100, WheelCallback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheels_odom", 50);   
  tf::TransformBroadcaster odom_broadcaster;


  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  double x;
  double y;
  double th;
  double vth;
  ros::Rate loop_rate(10);
  while(ros::ok()){

    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    double delta_th = vth * dt;

    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
