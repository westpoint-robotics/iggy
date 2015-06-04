#include <ros/ros.h>
#include <igvc_roboteq/oop_igvc_odometry.h>
#include <geometry_msgs/Vector3.h>

class IgvcOdometry
{
public:
  IgvcOdometry()
  {
    _PreviousLeftEncoderCounts = 0;
    _PreviousRightEncoderCounts = 0; 
    //math for measured --> 70.16cm per rotation /32 = .021927 meters per count
    //math from calc --> (3.14159265 * 0.2286) / 32;  //meters per count (pie * diameter) / #black+white
    DistancePerCount = 0.021927;  //meters per count

    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Vector3>("/published_topic", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/subscribed_topic", 1, &IgvcOdometry::callback, this);
  }

  void callback(const geometry_msgs::Vector3::ConstPtr& ticks)
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

    geometry_msgs::Vector3 output;
    //.... do something with the input and generate the output...
    pub_.publish(output);
  }

private:
    ros::Time current_time_encoder;
    double DistancePerCount;

    double vx;
    double vy;
    long _PreviousLeftEncoderCounts;
    long _PreviousRightEncoderCounts; 

    ros::NodeHandle n_;
    ros::Publisher pub_;
    ros::Subscriber sub_;

};//End of class IgvcOdometry

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "subscribe_and_publish");

  //Create an object of class IgvcOdometry that will take care of everything
  IgvcOdometry SAPObject;

  ros::spin();

  return 0;
}
