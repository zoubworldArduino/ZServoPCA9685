/*
	SharpIR

	Arduino library for retrieving distance (in cm) from the analog GP2Y0A21Y and GP2Y0A02YK

	From an original version of Dr. Marcal Casas-Cartagena (marcal.casas@gmail.com)     
	
    Version : 1.0 : Guillaume Rico

	https://github.com/guillaume-rico/SharpIR

*/

#ifndef ZServo_h
#define ZServo_h

#define  ROS_USED 

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ZPCA9685.h>
#ifdef ROS_USED 
#include <ros.h>
#include <ros/time.h>
#include <std_msgs/UInt16.h>
#endif 

//#include <sensor_msgs/Range.h>


class ZServoPCA9685
{

public:
  ZServoPCA9685(ZPCA9685 * myZPCA9685) ;
  uint8_t attach(int pin);           // attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  uint8_t attach(int pin, int min, int max); // as above but also sets min and max values for writes. 
  void detach();
  void write(int value);             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  void writeMicroseconds(int value); // Write pulse width in microseconds 
  int read();                        // returns current pulse width as an angle between 0 and 180 degrees
  int readMicroseconds();            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  bool attached();                   // return true if this servo is attached, otherwise false 

#ifdef ROS_USED 
    void setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,void callbackinstance( const std_msgs::UInt16& cmd_msg),int pin);
	void setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,int pin);
	void loop();
#endif 
  private:
#ifdef ROS_USED 
    ros::NodeHandle  *nh;    
    ros::Subscriber<std_msgs::UInt16> *subscriber;
#endif
	ZPCA9685 * servo;
	char index;
};

#endif
