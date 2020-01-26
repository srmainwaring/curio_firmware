#ifndef ROS_CUSTOM_H_
#define ROS_CUSTOM_H_

#include "ros_dummy.h"
#include "ros/node_handle.h"
#include "arduino_hardware.h"

namespace ros {
  // Custom buffers:
  //  8 publishers
  //  8 subscribers
  //  1024 byte input buffer
  //  1024 byte output buffer
  typedef NodeHandle_<ArduinoHardware, 8, 8, 1024, 1024> NodeHandle; 
}

#endif // ROS_CUSTOM_H_
