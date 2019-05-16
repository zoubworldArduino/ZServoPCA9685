/** @file ZServoPCA9685.h
	ZServoPCA9685

	 @par dependency :
   This library use the folowing ones :    ZPCA9685, PinExtender,Rosserial_Arduino_Library
   you can find it on https://github.com/zoubworldArduino/
  
	Library that manage servo with ros on a board with a PCA9685.
	This library offer the same API as the arduino library Servo.
	

*/

#ifndef ZServoPCA9685_h
#define ZServoPCA9685_h

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



class ZServoPCA9685
{

public:
/** constructor of the class*/
  ZServoPCA9685(ZPCA9685 * myZPCA9685 //!< the instance of ZPCA9685 driver associated to the board.
  ) ;
  /** @name API like Servo.h
*/
//@{
  /** attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
  */
  uint8_t attach(int pin);    
  void setPulseRange( int min, int max); // as above but also sets min and max values for writes. 

  /** as above but also sets min and max values for writes. 
  */
  uint8_t attach(int pin, int min, int max); 
  void detach();
  /**if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  */
  void write(int value);      
	/** Write pulse width in microseconds 
	*/  
  void writeMicroseconds(int value); 
  /** returns current pulse width as an angle between 0 and 180 degrees
  */
  int read();                       
/**  returns current pulse width in microseconds for this servo (was read_us() in first release)
*/  
  int readMicroseconds();            
  /** return true if this servo is attached, otherwise false 
  */
  bool attached();                   
//@}
  /** @name API for ROS
*/
//@{
#ifdef ROS_USED 
/** the ros initialisation, it replace attach.

note the ZPCA9685 must be initialised before.
	*/
    void setup( ros::NodeHandle * myNodeHandle //!< the ROS node handler
	,const char   *	topic //!< the topic displayed in ROS
	,void callbackinstance( const std_msgs::UInt16& cmd_msg)//!< the callback executed when a topic is recivied
	,int pin //!< the generic pin number associated to the topic
	);
	/** the ros initialisation, it replace attach .
	
	note the ZPCA9685 must be initialised before.
	*/
	void setup( ros::NodeHandle * myNodeHandle//!< the ROS node handler
	,	const char   *	topic //!< the topic displayed in ROS
	,int pin//!< the generic pin number associated to the topic
	);
	/** function to be called in your main loop.
	*/
	void loop();
#endif 
//@}
  private:
  
#ifdef ROS_USED 
    ros::NodeHandle  *nh;    
    ros::Subscriber<std_msgs::UInt16> *subscriber;
#endif
	/** an instance of ZPCA9685 driver*/
	ZPCA9685 * servo;
	uint8_t index;
};

#endif
