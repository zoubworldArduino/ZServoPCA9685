#if defined(BOARD_ID_Pilo)
#define ROS_SERIAL (PCOM2.Serial2)
#define ROS_BAUDRATE 57600
#include <ZRos.h>
#include "ros/node_handle.h"
 
 namespace ros
 {

   typedef ros::NodeHandle_<ArduinoHardwareSerial> NodeHandle;
 }

#else
//#include <Servo.h> 
#include <ros.h>



#endif
