#include <RCArduinoFastLib.h>

 // MultiChannels
//
// rcarduino.blogspot.com
//
// A simple approach for reading three RC Channels using pin change interrupts
//
// See related posts -
// http://rcarduino.blogspot.co.uk/2012/01/how-to-read-rc-receiver-with.html
// http://rcarduino.blogspot.co.uk/2012/03/need-more-interrupts-to-read-more.html
// http://rcarduino.blogspot.co.uk/2012/01/can-i-control-more-than-x-servos-with.html
//
// rcarduino.blogspot.com
// 

// Assign your channel out pins
#define THROTTLE_OUT_PIN 8
#define STEERING_OUT_PIN 9
#define AUX_OUT_PIN 10
#define OTHER_OUT_PIN 11

// Assign servo indexes
#define SERVO_THROTTLE 0
#define SERVO_STEERING 1
#define SERVO_AUX 2
#define SERVO_OTHER 3
#define SERVO_FRAME_SPACE 4

volatile uint32_t ulCounter = 0;

void setup()
{
  Serial.begin(115200);

  // attach servo objects, these will generate the correct
  // pulses for driving Electronic speed controllers, servos or other devices
  // designed to interface directly with RC Receivers 

  CRCArduinoFastServos::attach(SERVO_THROTTLE,THROTTLE_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_STEERING,STEERING_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_AUX,AUX_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_OTHER,OTHER_OUT_PIN);
  
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,6*2000);

  CRCArduinoFastServos::begin();
  CRCArduinoPPMChannels::begin();
  Serial.begin(9600);
}

void loop()
{
  // Pass the signals straight through - 
 
  uint16_t unThrottleIn =  CRCArduinoPPMChannels::getChannel(SERVO_THROTTLE);
  if(unThrottleIn)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,unThrottleIn);
    Serial.println("SERVO_THROTTLE");
  }

  uint16_t unSteeringIn =  CRCArduinoPPMChannels::getChannel(SERVO_STEERING);
  if(unSteeringIn)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_STEERING,unSteeringIn);
    Serial.println("SERVO_STEERING");
  }

  uint16_t unAuxIn =  CRCArduinoPPMChannels::getChannel(SERVO_AUX);
  if(unAuxIn)
  {
   //CRCArduinoFastServos::writeMicroseconds(SERVO_AUX,unAuxIn);
   Serial.println("SERVO_AUX");
  }
}


/* DB 04/02/2012 REMOVED The interrupt service routine definition here, it clashes with the attachInterrupt in the cpp file */
/* REMOVE BEGIN  
ISR(INT0_vect) {
 CRCArduinoPPMChannels::INT0ISR();
}
REMOVE END */