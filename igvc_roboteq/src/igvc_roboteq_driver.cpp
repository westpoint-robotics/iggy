#include "ros/ros.h"
#include "serial/serial.h"
#include <unistd.h>
#include <iostream>
#include <sstream>
//#include "igvc_roboteq/igvc_roboteq_driver.h"

  class EOMSend {};

  void messageSender(std::string msg) {
    std::stringstream ss;
      if (ss.tellp() == 0) {
        ss << msg << val;
      } 
      else {
        ss << ' ' << val;
      }
    }

  MessageSender command;
  MessageSender query;
  MessageSender param;
  EOMSend send, sendVerify;


int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle nh("~");
  serial::Serial *serial_;
  const char *port_;
  int baud_;
  bool connected_;
  std::stringstream tx_buffer_;


  std::string port = "/dev/ttyACM0";  //add look for ACM1
  int32_t baud = 115200;

//connect with robot
if (!serial_) serial_ = new serial::Serial();
serial::Timeout to(serial::Timeout::simpleTimeout(500));
serial_->setTimeout(to);
serial_->setPort(port_);
serial_->setBaudrate(baud_);

for (int tries = 0; tries < 5; tries++) {
  try {
    serial_->open();
    query << "FID" << send;
    setSerialEcho(false);
    flush();   
    }
  catch (serial::IOException) {
    }
  if (serial_->isOpen()) {
    connected_ = true;
    ROS_INFO("Good Connection with serial port %s",port_);
    break;
    }
  else {
    connected_ = false;
    ROS_INFO("Bad Connection with serial port Error %s",port_);
    }
}

while (ros::ok()) {
  std::string msg = serial_->readline(max_line_length, eol);
  if (!msg.empty()) {
  ROS_INFO("message data: %s");
  }
  else {
  ROS_INFO("no data to read");
  }
}
