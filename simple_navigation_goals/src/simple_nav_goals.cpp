#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <utility>
#include <iostream>
//#include <tuple> // C++11, for std::tie

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


// below comment code not neccisary, use this instead: ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
//subscribe to move_base/result
    //if status.text = "Goal reached."
    //send next command

double xloc, yloc;
double R = 6371000; //rad of earth

//std::pair<double, double> gps2xyz (latin, lonin)
//{
//    //these equations assume spherical  
//    x = R * cos(latin) * cos(lonin);
//    y = R * cos(latin) * sin(lonin);
//    return std::make_pair(x, y);
//}

double gps2x (latin4x, lonin4x)
{
    //these equations assume spherical  
    x = R * cos(latin) * cos(lonin);
    return xloc;
}
double gps2y (latin4y, lonin4y)
{
    //these equations assume spherical  
    y = R * cos(latin) * sin(lonin);
    return yloc;
}


//subscribe to topic that converts gps location to xyz or subcribe to gps location and convert here
void callback(const sensor_msgs::NavSatFix& gpsloc)
{
    double lat = gpsloc.latitude;
    double lon = gpsloc.longitude;
    double xloc = gps2x(lat, lon);
    double yloc = gps2y(lat, lon);
    return xloc, yloc;
}






int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  ros::Subscriber sub = nh.subscribe("gps/fix", 1, callback);
    //returns xloc, yloc

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

    //make this goal creation, sending and status checking a function to call in main

  move_base_msgs::MoveBaseGoal goal1;

  //we'll send a goal to the robot to move 2 meter forward
  goal1.target_pose.header.frame_id = "base_link";
  goal1.target_pose.header.stamp = ros::Time::now();

  goal1.target_pose.pose.position.x = 0.0;   // positive is forward
  //goal1.target_pose.pose.position.y = 10.0;  //postive is to the left
  goal1.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal 1");
  ac.sendGoal(goal1);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Completed goal 1");
      move_base_msgs::MoveBaseGoal goal2;
      //we'll send a goal to the robot to move 2 meter forward
      goal2.target_pose.header.frame_id = "base_link";
      goal2.target_pose.header.stamp = ros::Time::now();

      goal2.target_pose.pose.position.x = 0.0;   // positive is forward
      //goal2.target_pose.pose.position.y = 10.0;  //postive is to the left
      goal2.target_pose.pose.orientation.w = 1.0;

      ROS_INFO("Sending goal 2");
      ac.sendGoal(goal2);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("goal 2 complete");
    

      else
        ROS_INFO("The base failed goal 2");

  else
    ROS_INFO("The base failed goal 1");

  return 0;
}
