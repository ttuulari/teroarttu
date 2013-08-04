#include <PinChangeInt.h>
#include <Servo.h>

// Assign channel in pins
#define LEFT_IN_PIN 8
#define RIGHT_IN_PIN 9
#define THROTTLE_IN_PIN 10

// Assign channel out pins
#define THROTTLE_OUT_PIN 5
#define LEFT_OUT_PIN 6
#define RIGHT_OUT_PIN 7

Servo servoThrottle;
Servo servoLeft;
Servo servoRight;

// These bit flags are set in bUpdateFlagsShared to indicate which
// channels have new signals
#define THROTTLE_FLAG 1
#define LEFT_FLAG 2
#define RIGHT_FLAG 4

volatile uint8_t bUpdateFlagsShared;

// shared variables are updated by the ISR and read by loop.
// In loop we immediatley take local copies so that the ISR can keep ownership of the 
// shared ones. To access these in loop
// we first turn interrupts off with noInterrupts
// we take a copy to use in loop and the turn interrupts back on
// as quickly as possible, this ensures that we are always able to receive new signals
volatile uint16_t unThrottleInShared;
volatile uint16_t unLeftInShared;
volatile uint16_t unRightInShared;

// These are used to record the rising edge of a pulse in the calcInput functions
// They do not need to be volatile as they are only used in the ISR. If we wanted
// to refer to these in loop and the ISR then they would need to be declared volatile
uint32_t ulThrottleStart;
uint32_t ulRightStart;
uint32_t ulLeftStart;

void setup()
{
  Serial.begin(9600);
  servoThrottle.attach(THROTTLE_OUT_PIN);
  servoLeft.attach(LEFT_OUT_PIN);
  servoRight.attach(RIGHT_OUT_PIN);

  // using the PinChangeInt library, attach the interrupts
  // used to read the channels
  PCintPort::attachInterrupt(THROTTLE_IN_PIN, calcThrottle, CHANGE); 
  PCintPort::attachInterrupt(RIGHT_IN_PIN, calcRight, CHANGE); 
  PCintPort::attachInterrupt(LEFT_IN_PIN, calcLeft, CHANGE); 
}

void loop()
{
  // create local variables to hold a local copies of the channel inputs
  // these are declared static so that thier values will be retained 
  // between calls to loop.
  static uint16_t unThrottleIn;
  static uint16_t unLeftIn;
  static uint16_t unRightIn;
  // local copy of update flags
  static uint8_t bUpdateFlags;

  // check shared update flags to see if any channels have a new signal
  if(bUpdateFlagsShared)
  {
    noInterrupts(); // turn interrupts off quickly while we take local copies of the shared variables

    // take a local copy of which channels were updated in case we need to use this in the rest of loop
    bUpdateFlags = bUpdateFlagsShared;
        
    if(bUpdateFlags & THROTTLE_FLAG)
    {
      unThrottleIn = unThrottleInShared;
    }
    
    if(bUpdateFlags & LEFT_FLAG)
    {
      unLeftIn = unLeftInShared;
    }
    
    if(bUpdateFlags & RIGHT_FLAG)
    {
      unRightIn = unRightInShared;
    }
     
    // clear shared copy of updated flags as we have already taken the updates
    // we still have a local copy if we need to use it in bUpdateFlags
    bUpdateFlagsShared = 0;
    
    interrupts(); // we have local copies of the inputs, so now we can turn interrupts back on
    // as soon as interrupts are back on, we can no longer use the shared copies, the interrupt
    // service routines own these and could update them at any time. During the update, the 
    // shared copies may contain junk. Luckily we have our local copies to work with :-)
  }
  
  // do any processing from here onwards
  // only use the local values unRightIn, unThrottleIn and unLeftIn, the shared
  // variables unRightInShared, unThrottleInShared, unLeftInShared are always owned by 
  // the interrupt routines and should not be used in loop
  
  // the following code provides simple pass through 
  // this is a good initial test, the Arduino will pass through
  // receiver input as if the Arduino is not there.
  // This should be used to confirm the circuit and power
  // before attempting any custom processing in a project.
  
  // we are checking to see if the channel value has changed, this is indicated  
  // by the flags. For the simple pass through we don't really need this check,
  // but for a more complex project where a new signal requires significant processing
  // this allows us to only calculate new values when we have new inputs, rather than
  // on every cycle.
  if(bUpdateFlags & THROTTLE_FLAG)
  {
    if(servoThrottle.readMicroseconds() != unThrottleIn)
    {
      Serial.print("Throttle ");
      Serial.println(unThrottleIn);
      servoThrottle.writeMicroseconds(unThrottleIn);
    }
  }
  
  if(bUpdateFlags & LEFT_FLAG)
  {
    if(servoLeft.readMicroseconds() != unLeftIn)
    {
      Serial.print("Left ");
      Serial.println(unLeftIn);
      servoLeft.writeMicroseconds(unLeftIn);
    }
  }
  
  if(bUpdateFlags & RIGHT_FLAG)
  {
    if(servoRight.readMicroseconds() != unRightIn)
    {
      Serial.print("Right ");
      Serial.println(unRightIn);
      servoRight.writeMicroseconds(unRightIn);
    }
  }
  
  bUpdateFlags = 0;
}


void calcThrottle()
{
  if(digitalRead(THROTTLE_IN_PIN) == HIGH)
  { 
    ulThrottleStart = micros();
  }
  else
  {
    unThrottleInShared = (uint16_t)(micros() - ulThrottleStart);
    bUpdateFlagsShared |= THROTTLE_FLAG;
  }
}

void calcRight()
{
  if(digitalRead(RIGHT_IN_PIN) == HIGH)
  { 
    ulRightStart = micros();
  }
  else
  {
    unRightInShared = (uint16_t)(micros() - ulRightStart);
    bUpdateFlagsShared |= RIGHT_FLAG;
  }
}

void calcLeft()
{
  if(digitalRead(LEFT_IN_PIN) == HIGH)
  { 
    ulLeftStart = micros();
  }
  else
  {
    unLeftInShared = (uint16_t)(micros() - ulLeftStart);
    bUpdateFlagsShared |= LEFT_FLAG;
  }
}