#ifndef IGVC_ROBOTEQ_DRIVER
#define IGVC_ROBOTEQ_DRIVER
#include "ros/ros.h"
#include <stdint.h>
#include <string>

namespace serial {
  class Serial;
}

class igvc_roboteq_driver {
private :
  const char *port_;
  int baud_;
  bool connected_;
  std::string version_;
  serial::Serial *serial_;
  std::stringstream tx_buffer_;
  std::vector<Channel*> channels_; 

  ros::NodeHandle nh_;
  ros::Publisher pub_data_;

  void read();
  void write(std::string);
}

class MessageSender {
    public:
    MessageSender(std::string init, Controller* interface)
        : init_(init), interface_(interface) {}

    template<typename T>
    MessageSender& operator<<(const T val) {
      if (ss.tellp() == 0) {
        ss << init_ << val;
      } else {
        ss << ' ' << val;
      }
      return *this;
    }
 
    void operator<<(EOMSend) 
    {
      interface_->write(ss.str());
      ss.str("");
    }
   
    private:
    std::string init_;
    Controller* interface_;
    std::stringstream ss;
  };
















#endif
