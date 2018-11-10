//EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : rosrun rosserial_python serial_node.py /dev/ttyS4 & 
// rostopic pub servo/A std_msgs/UInt16 0 --once
//  rostopic pub servo/B std_msgs/UInt16 180 --once


// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server
// sudo chmod 666 /dev/ttyS4 if COM4

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <ZServoPCA9685.h> 
#include <ZPCA9685.h>
#include <Servo.h> 
#include <ros.h>

ros::NodeHandle  nh;
ZPCA9685 card_servos = ZPCA9685();
ZServoPCA9685 servo_cmdA= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdB= ZServoPCA9685(&card_servos);
int pin=9;// digital  9
// example of call back to be define in case of 2 card ZPCA9685.
void callbackinstance00( const std_msgs::UInt16& cmd_msg)
{  
  //nh.loginfo("callbackinstance00()");
servo_cmdA.write(cmd_msg.data); //set servo angle, should be from 0-180   

}


ros::Subscriber<std_msgs::UInt16> sub("servo/A", callbackinstance00);

void setup() {

    card_servos.begin(&Wire,0x43);
  nh.initNode(); 
  //servoA.setup(&nh,"servo/A",callbackinstance00,pin);
  servo_cmdA.setup(&nh,"servo/A",pin);
  servo_cmdB.setup(&nh,"servo/B",8);
  nh.logdebug("Debug Statement");

   //wait until you are actually connected
    while (!nh.connected())
    {
      nh.spinOnce();
    }
    /*
    nh.logdebug("Debug Statement");
    nh.loginfo("Program info");
    nh.logwarn("Warnings.");
    nh.logerror("Errors..");
    nh.logfatal("Fatalities!");
    */
// play with servo to see it moving
    servo_cmdA.write(0);
    servo_cmdB.write(0);
    delay(2000);
     servo_cmdA.write(180);
    servo_cmdB.write(180);
    delay(1000);
     servo_cmdA.write(90);
    servo_cmdB.write(90);
}
void loopnh() {
  
  servo_cmdA.loop();
servo_cmdB.loop();

  nh.spinOnce();
}
void loop()
{// put your main code here, to run repeatedly:
  loopnh();
    nh.loginfo("loop()");
    delay(100);
  
}
