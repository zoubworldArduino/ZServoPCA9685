#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <ZServoPCA9685.h> 
#include <ZPCA9685.h>
#define MyWire P_COM0_BIS.wire
ZPCA9685 card_servos = ZPCA9685();
ZServoPCA9685 servo_cmdA= ZServoPCA9685(&card_servos);
ZServoPCA9685 servo_cmdB= ZServoPCA9685(&card_servos);



void setup() {

MyWire.begin();

    card_servos.begin(&MyWire,0x43);
 /*   if (!card_servos.test())
    while(1);
    card_servos.begin(&MyWire,0x43);*/
    servo_cmdA.attach(0xA);
    servo_cmdB.attach(0xB);
     delay(2000);
    servo_cmdA.write(60);
    servo_cmdB.write(60);
    delay(2000);
     servo_cmdA.write(120);
    servo_cmdB.write(120);
    delay(1000);
     servo_cmdA.write(90);
    servo_cmdB.write(90);
}
/*
int i=1;*/
void loop()
{
/*
if (i++%10==0)
card_servos.begin(&MyWire,0x43);*/
delay(500);
   servo_cmdA.write(20);
    servo_cmdB.write(20);
    delay(500);
     servo_cmdA.write(130);
    servo_cmdB.write(130);
    delay(500);
     servo_cmdA.write(90);
    servo_cmdB.write(90);

}
