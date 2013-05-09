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
static void print_int(unsigned long val, unsigned long invalid, int len);
static void print_date(TinyGPS &gps);
static void print_str(const char *str, int len);

volatile uint32_t ulCounter = 0;


void setup() {
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

  nss.begin(57600);
}

void loop() {
  bool newdata = false;
  unsigned long start = millis();
  
  // Every second we print an update
  while (millis() - start < 1000)
  {
    if (feedgps())
      newdata = true;
  }
  
  gpsdump(gps);
  bmpdump(bmp);


  uint16_t rollIn =  CRCArduinoPPMChannels::getChannel(SERVO_ROLL);
  if(rollIn)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_ROLL,rollIn);
    Serial.print("SERVO_ROLL ");
    Serial.println(rollIn);
  }

  uint16_t pitchIn =  CRCArduinoPPMChannels::getChannel(SERVO_PITCH);
  if(pitchIn)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_PITCH,pitchIn);
    Serial.print("SERVO_PITCH ");
    Serial.println(pitchIn);
  }

  uint16_t throttleIn =  CRCArduinoPPMChannels::getChannel(SERVO_THROTTLE);
  if(throttleIn)
  {
    //CRCArduinoFastServos::writeMicroseconds(SERVO_THROTTLE,throttleIn);
    Serial.print("SERVO_THROTTLE ");
    Serial.println(throttleIn);
  }
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
  
  Serial.println();
}

static void print_float(float val, float invalid, int len, int prec)
{
  char sz[32];
  if (val == invalid)
  {
    strcpy(sz, "*******");
    sz[len] = 0;
        if (len > 0) 
          sz[len-1] = ' ';
    for (int i=7; i<len; ++i)
        sz[i] = ' ';
    Serial.print(sz);
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1);
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(" ");
  }
  feedgps();
}

static void bmpdump(Adafruit_BMP085 &bmp) {
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");

    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");
    
    Serial.println();    
}

/* DB 04/02/2012 REMOVED The interrupt service routine definition here, it clashes with the attachInterrupt in the cpp file */
/* REMOVE BEGIN  
ISR(INT0_vect) {
 CRCArduinoPPMChannels::INT0ISR();
}
REMOVE END */
