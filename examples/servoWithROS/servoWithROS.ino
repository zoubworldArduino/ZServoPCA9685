/*EXAMPLE OF SERVO WITH ROS.
// do
// roscore &
// for raspberry pi : rosrun rosserial_python serial_node.py /dev/ttyUSB0 &
// or for windows on COM4 : 
        sudo chmod 666 /dev/ttyS4 if COM4
        rosrun rosserial_python serial_node.py /dev/ttyS4 & 
 rostopic pub servo/A std_msgs/UInt16 10 --once
 rostopic pub servo/B std_msgs/UInt16 180 --once
 rostopic pub servo/A std_msgs/UInt16 180 --once
 rostopic pub servo/B std_msgs/UInt16 10 --once
*/

// NOTE : WINDOWS 10 with ubuntu 16.04 as subsystem, after ros install do this :
//      sudo apt-get install ros-kinetic-rosserial-windows
//     sudo apt-get install ros-kinetic-rosserial-server

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <ZServoPCA9685.h> 
#include <ZPCA9685.h>
//#include <Servo.h> 



#define ROS_SERIAL (P_COM3.serial2)
#define ROS_BAUDRATE 57600
#include <ros.h>

#include <WireUtility.h>

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

#define MySerial P_COM3.serial2 //usb
#define WireCard (P_COM0_BIS.wire)

void setup() {

    WireCard.begin();  
if (0)
{
MySerial.begin(57600);  //115200 //9600
	MySerial.println("Setup");

	volatile int ip = scan(MySerial, WireCard);
	while (ip = scanNext(MySerial, WireCard) != 0);
}


    card_servos.begin(&WireCard,0x43);
    
    
 
    
  nh.initNode(); 
  //servoA.setup(&nh,"servo/A",callbackinstance00,pin);
  servo_cmdA.setup(&nh,"servo/A",10);
  servo_cmdB.setup(&nh,"servo/B",11);
  
  /*
  
  servo_cmdA.write(0);//546탎 / 19790탎
    servo_cmdB.write(0);
    delay(2000);
     servo_cmdA.write(180);//2473탎 / 19790탎
    servo_cmdB.write(180);
    delay(1000);
     servo_cmdA.write(90);//1507탎 / 19790탎
    servo_cmdB.write(90);
  */
  
  
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
