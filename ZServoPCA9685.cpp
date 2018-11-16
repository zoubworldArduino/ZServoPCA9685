/*
	ZServo

	Arduino library for Servo
	Author :
	Pierre Valleau
	history :
		add ros supports


*/
#define DEBUG(a) a
//#define DEBUG(a) {}


#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
//#include "WMath.h"
#include "ZServoPCA9685.h"
#ifdef ROS_USED 

#define ZSERVO_MAX 16
	ZServoPCA9685 * myservo[ZSERVO_MAX]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

#endif

ZServoPCA9685::ZServoPCA9685(ZPCA9685 * myZPCA9685) 
{
	servo=myZPCA9685;
	
	
}


// attach the given pin to the next free channel, sets pinMode, returns channel number or 0 if failure
 uint8_t ZServoPCA9685::attach(int pin)
 {
	 DEBUG(nh->loginfo("ZServoPCA9685::attach1()"));  
	// nh->subscribe(*subscriber);
	index=pin;
	#ifdef ROS_USED 
		myservo[index]=this;	
	#endif
	 return servo->attach(pin);
 }
 
  uint8_t ZServoPCA9685::attach(int pin, int min, int max) // as above but also sets min and max values for writes. 
  {  	
DEBUG(nh->loginfo("ZServoPCA9685::attach()"));  
	//   nh->subscribe(*subscriber);
	index=pin;
	#ifdef ROS_USED 
		myservo[index]=this;	
	#endif
	   return servo->attach(pin,min,max);

  }  
  void ZServoPCA9685::detach()
  {
	  DEBUG(nh->loginfo("ZServoPCA9685::detach()")); 
	  servo->detach(index); 
  //    subscriber->shutdown();
  }  
  void ZServoPCA9685::write(int value)             // if value is < 200 its treated as an angle, otherwise as pulse width in microseconds 
  {
	   DEBUG(nh->loginfo("ZServoPCA9685::write()")); 
	 servo->write(index,value);  
  }  
  
  void ZServoPCA9685::writeMicroseconds(int value) // Write pulse width in microseconds 
  {
	  DEBUG(nh->loginfo("ZServoPCA9685::writeMicroseconds()")); 
	  servo->writeMicroseconds(index,value);  
  }  
  
  int ZServoPCA9685::read()                        // returns current pulse width as an angle between 0 and 180 degrees
  {
	 return servo->read(index);   
  }  
  
  int ZServoPCA9685::readMicroseconds()            // returns current pulse width in microseconds for this servo (was read_us() in first release)
  {
	 return servo->readMicroseconds(index);   
  }  
   // return true if this servo is attached, otherwise false 
  bool ZServoPCA9685::attached()
  {
	  return servo->attached(index);  
  }
  
#ifdef ROS_USED 


static void callbackinstance0( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance0()"));
if (myservo[0]!=0)
myservo[0]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance1( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance1()"));
if (myservo[1]!=0)
myservo[1]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance2( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance2()"));
if (myservo[2]!=0)
myservo[2]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance3( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance3()"));
if (myservo[3]!=0)
myservo[3]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}

static void callbackinstance4( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance4()"));
if (myservo[4]!=0)
	myservo[4]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance5( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance5()"));
if (myservo[5]!=0)
	myservo[5]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance6( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance6()"));
if (myservo[6]!=0)
	myservo[6]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance7( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance7()"));
if (myservo[7]!=0)
	myservo[7]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance8( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance8()"));
if (myservo[8]!=0)
	myservo[8]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance9( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance9()"));
if (myservo[9]!=0)
	myservo[9]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance10( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance10()"));
if (myservo[10]!=0)
	myservo[10]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance11( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance11()"));
if (myservo[11]!=0)
	myservo[11]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance12( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance012()"));
if (myservo[12]!=0)
	myservo[12]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance13( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance13()"));
if (myservo[13]!=0)
	myservo[13]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance14( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance14()"));
if (myservo[14]!=0)
	myservo[14]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}
static void callbackinstance15( const std_msgs::UInt16& cmd_msg)
{	
//DEBUG(nh->loginfo("callbackinstance15()"));
if (myservo[15]!=0)
	myservo[15]->write(cmd_msg.data); //set servo angle, should be from 0-180   
}

void(*callback[ZSERVO_MAX])(const std_msgs::UInt16& cmd_msg)={
	callbackinstance0,callbackinstance1,callbackinstance2,callbackinstance3,
	callbackinstance4,callbackinstance5,callbackinstance6,callbackinstance7,
	callbackinstance8,callbackinstance9,callbackinstance10,callbackinstance11,
	callbackinstance12,callbackinstance13,callbackinstance14,callbackinstance15	
	};

void ZServoPCA9685::setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,int pin)
{
  nh=myNodeHandle;
  attach( pin);
  subscriber=new ros::Subscriber<std_msgs::UInt16> (topic, callback[pin]); 
  nh->subscribe(*subscriber); 
  DEBUG(nh->loginfo("ZServoPCA9685::setup()"));  
}
// ROS SECTION :
//char frameid[] = "/ir_ranger";
/** setup :
  At setup after NodeHandle setup, call this to initialise the topic
*/
void ZServoPCA9685::setup( ros::NodeHandle * myNodeHandle,	const char   *	topic ,void callbackinstance( const std_msgs::UInt16& cmd_msg),int pin)
{
  nh=myNodeHandle;
  subscriber=new ros::Subscriber<std_msgs::UInt16> (topic, callbackinstance); 
  attach( pin);
  nh->subscribe(*subscriber); 
  DEBUG(nh->loginfo("ZServoPCA9685::setup2()"));   
}
/** loop :
  on loop  before NodeHandle refresh(spinOnce), call this to update the topic
*/
void ZServoPCA9685::loop()
{
	//nothing to do just keep for compatibility with others.
}
#endif 

