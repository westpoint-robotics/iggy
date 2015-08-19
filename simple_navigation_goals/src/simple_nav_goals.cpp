#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <utility>
#include <iostream>
#include <sensor_msgs/NavSatFix.h>

//#include <tuple> // C++11, for std::tie

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// below comment code not neccisary, use this instead: ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
//subscribe to move_base/result
    //if status.text = "Goal reached."
    //send next command

//double R = 6371000; //rad of earth

struct utm {
    double northing;
    double easting;
} currentUTM, goalUTM; 




//std::pair<double, double> gps2xyz (latin, lonin)
//{
//    //these equations assume spherical  
//    x = R * cos(latin) * cos(lonin);
//    y = R * cos(latin) * sin(lonin);
//    return std::make_pair(x, y);
//}

utm gps2UTM(double latin, double lonin)
{
    //these equations assume spherical  
    utm robotUTM;
    robotUTM.northing = 6371000 * cos(latin) * cos(lonin); 
    robotUTM.easting = 6371000 * cos(latin) * sin(lonin);
    return robotUTM;
}

//subscribe to topic that converts gps location to xyz or subcribe to gps location and convert here
void callback(const sensor_msgs::NavSatFix& gpsloc)
{
    double lat = gpsloc.latitude;
    double lon = gpsloc.longitude;
    currentUTM = gps2UTM(lat, lon);
    
}

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  ros::NodeHandle nh;

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
  //TODO get goal from user
  double goallat = 41.390932;
  double goallon = -73.952948;
  currentUTM = gps2UTM(goallat, goallon);
  goal1.target_pose.pose.position.x = goalUTM.northing - currentUTM.northing; // positive is forward
  goal1.target_pose.pose.position.y = goalUTM.easting - currentUTM.easting;  //postive is to the left since we are at neg lon
  goal1.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal 1");
  ac.sendGoal(goal1);

  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Completed goal 1");

  else
    ROS_INFO("The base failed goal 1");

  return 0;
}
