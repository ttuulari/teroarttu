#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <TinyGPS.h>
#include <RCArduinoFastLib.h>

#define THROTTLE_OUT_PIN 8
#define STEERING_OUT_PIN 9
#define AUX_OUT_PIN 10
#define OTHER_OUT_PIN 11

// Assign servo indexes
#define SERVO_ROLL 0
#define SERVO_PITCH 1
#define SERVO_THROTTLE 2
#define SERVO_OTHER 3
#define SERVO_FRAME_SPACE 4

#define GPS_RX_INDEX 3
#define GPS_TX_INDEX 4

Adafruit_BMP085 bmp;
TinyGPS gps;
SoftwareSerial nss(GPS_RX_INDEX, GPS_TX_INDEX);

static void gpsdump(TinyGPS &gps);
static bool feedgps();
static void print_float(float val, float invalid, int len, int prec);

volatile uint32_t ulCounter = 0;


void setup() {
  nss.begin(57600);

  Serial.begin(115200);
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  }
  CRCArduinoFastServos::attach(SERVO_ROLL,THROTTLE_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_PITCH,STEERING_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_THROTTLE,AUX_OUT_PIN);
  CRCArduinoFastServos::attach(SERVO_OTHER,OTHER_OUT_PIN);
  
  // lets set a standard rate of 50 Hz by setting a frame space of 10 * 2000 = 3 Servos + 7 times 2000
  CRCArduinoFastServos::setFrameSpaceA(SERVO_FRAME_SPACE,6*2000);

  CRCArduinoFastServos::begin();
  CRCArduinoPPMChannels::begin();

}

void loop() {
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 100)
  {
    if (feedgps())
      newdata = true;
  }
  
  gpsdump(gps);
  bmpdump(bmp);
  ppmdump();
  Serial.print("\n");
}

static void ppmdump() {    
  uint16_t rollIn =  CRCArduinoPPMChannels::getChannel(SERVO_ROLL);
  uint16_t pitchIn =  CRCArduinoPPMChannels::getChannel(SERVO_PITCH);
  uint16_t throttleIn =  CRCArduinoPPMChannels::getChannel(SERVO_THROTTLE);
  Serial.print(rollIn);
  Serial.print("\t");
  Serial.print(pitchIn);
  Serial.print("\t");
  Serial.print(throttleIn);
  Serial.print("\t");
}

static bool feedgps()
{
  while (nss.available())
  {
    if (gps.encode(nss.read()))
      return true;
  }
  return false;
}

static void gpsdump(TinyGPS &gps)
{
  float flat, flon;
  unsigned long age, date, time, chars = 0;
  unsigned short sentences = 0, failed = 0;
  static const float LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  gps.f_get_position(&flat, &flon, &age);
  print_float(flat, TinyGPS::GPS_INVALID_F_ANGLE, 9, 5);
  print_float(flon, TinyGPS::GPS_INVALID_F_ANGLE, 10, 5);
  print_float(gps.f_altitude(), TinyGPS::GPS_INVALID_F_ALTITUDE, 8, 2);
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    val = 0.0;
  }
  
  Serial.print(val, prec);
  Serial.print("\t");  
  feedgps();
}

static void bmpdump(Adafruit_BMP085 &bmp) {    
    Serial.print(bmp.readTemperature());
    Serial.print("\t");
    
    Serial.print(bmp.readPressure());
    Serial.print("\t");

    Serial.print(bmp.readAltitude(101500));
    Serial.print("\t");
}

/* DB 04/02/2012 REMOVED The interrupt service routine definition here, it clashes with the attachInterrupt in the cpp file */
/* REMOVE BEGIN  
ISR(INT0_vect) {
 CRCArduinoPPMChannels::INT0ISR();
}
REMOVE END */
