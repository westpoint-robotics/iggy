#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

double vx = 0.0;
double vy = 0.0;

void publishCallBack(const geometry_msgs::Twist& cmd) {
    printf("I got to the callback!\n");
    vx = cmd.linear.x;
    vy = cmd.linear.y;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odom_generator");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom/virtual", 50);
  ros::Subscriber cmd_read = n.subscribe("roboteq_driver/cmd", 1, publishCallBack);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(50.0);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    double dt = (current_time - last_time).toSec();
    double delta_x = vx * dt;
    double delta_y = vy * dt;

    x += delta_x;
    y += delta_y;


    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;

    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;

    //publish the message
    printf("I published the odom! (%f,%f)\n",vx,vy);
    odom_pub.publish(odom);

    last_time = current_time;
    r.sleep();
  }
}
