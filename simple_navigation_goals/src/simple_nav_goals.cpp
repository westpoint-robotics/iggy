#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <math.h>
#include <utility>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct utm {
    double northing;
    double easting;
} currentUTM, goalUTM; 

using namespace std;


/*  old method of doing conversion here
utm gps2UTM(double latin, double lonin)
{
    //these equations assume spherical and are not accurate..
    //TODO make equations more accurate --> latLong-UTMConversion.cpp file gives 3 ways to make it more accurate
    utm robotUTM;
    robotUTM.northing = 6371000 * cos(latin) * cos(lonin); 
    robotUTM.easting = 6371000 * cos(latin) * sin(lonin);
    return robotUTM;
}*/

//subscribe to topic that converts gps location to xyz or subcribe to gps location and convert here
/*  old method of doing conversion here
void callback(const sensor_msgs::NavSatFix& gpsloc)
{
    double lat = gpsloc.latitude;
    double lon = gpsloc.longitude;
    currentUTM = gps2UTM(lat, lon);
    ROS_INFO("callback good");

}*/

void curLocCallback(const nav_msgs::Odometry& utmloc)
{
    currentUTM.northing = utmloc.pose.pose.position.x;
    currentUTM.easting = utmloc.pose.pose.position.y;
    ROS_INFO("cur callback good");

}
void goalLocCallback(const nav_msgs::Odometry& utmgoal)
{
    goalUTM.northing = utmgoal.pose.pose.position.x;
    goalUTM.easting = utmgoal.pose.pose.position.y;
    ROS_INFO("goal callback good");

}


int main(int argc, char** argv){
//TODO FIXME this doesnt work.. need two subcribers in one topic
  //ros::MultiThreadedSpinner spinner(2); // Use 2 threads
  //spinner.spin(); // spin() will not return until the node has been shutdown

  //ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle nh;
  //ros::NodeHandle nh2;
  ros::Subscriber sub = nh.subscribe("vo", 1, curLocCallback);
  ros::Subscriber sub2 = nh.subscribe("goal_utm", 1, goalLocCallback);
  ros::Publisher pub = nh.advertise<sensor_msgs::NavSatFix>("goal_gps", 5);



  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);





  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }


  //ask for gps location of goal and publish to be converted
  sensor_msgs::NavSatFix gpsgoal;
  gpsgoal.header.stamp = ros::Time::now();
  cout << "What latitude would you like to go to? (ex: 40.123):";
  cin >> gpsgoal.latitude;
  cout << "What longitude would you like to go to? (ex: -71.123):";
  cin >> gpsgoal.longitude;
  gpsgoal.altitude = -26.7866;
  

  //pub.publish(gpsgoal);

ros::Rate loop_rate(10);

  while(ros::ok()){
  ROS_INFO("top");

  pub.publish(gpsgoal);

  if (currentUTM.northing == 0.0 or goalUTM.northing == 0.0) {
    //try agin.. wait til we get a signal
    ROS_INFO("cur x: %f ", currentUTM.northing);
    //ROS_INFO("cur y: %f ", currentUTM.easting);
    ROS_INFO("goal x: %f ", goalUTM.northing);
    //ROS_INFO("cur y: %f ", goalUTM.easting);
    }//end if
  else {
    //TODO make sending of a goal a function that can be called multiple times
    move_base_msgs::MoveBaseGoal goal1;
    
    goal1.target_pose.header.frame_id = "base_link";
    goal1.target_pose.header.stamp = ros::Time::now();
    ROS_INFO("cur x: %f ", currentUTM.northing);
    ROS_INFO("cur y: %f ", currentUTM.easting);
    ROS_INFO("goal x: %f ", goalUTM.northing);
    ROS_INFO("goal y: %f ", goalUTM.easting);
    double goalx = (goalUTM.northing - currentUTM.northing);
    double goaly = (goalUTM.easting - currentUTM.easting);
    ROS_INFO("dif x: %f ", goalx);
    ROS_INFO("dif y: %f ", goaly);
    goal1.target_pose.pose.position.x = goalx;  // positive is forward
    goal1.target_pose.pose.position.y = goaly;  //postive is to the left since we are at neg lon
    goal1.target_pose.pose.orientation.w = 1.0;

    ROS_INFO("Sending goal 1");
    ac.sendGoal(goal1);

    ac.waitForResult();
            
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Completed goal 1");
        //go back to top? accept new goal?
      }
    else{
      ROS_INFO("The base failed goal 1");
      }

  }  //end else
 // ROS_INFO("0");

  ros::spinOnce(); 
  loop_rate.sleep();
  ROS_INFO("1");
  //TODO add a pause here or way to make is so current utm info isnt pulled as fast as it is currently
 }  //end while
  ROS_INFO("2");

}  //end main
