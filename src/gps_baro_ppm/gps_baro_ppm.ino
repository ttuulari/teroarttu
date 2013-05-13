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


// Unions to help float and int to byte conversion
typedef union {
    byte b[4];
    float f;
} u_f;

typedef union {
    byte b[2];
    uint16_t i;
} u_i;

static void gpsdump(TinyGPS &gps, u_f* output);
static void bmpdump(Adafruit_BMP085 &bmp, u_f* output);
static void ppmdump(u_i* output);
static void writeFToSerial(u_f* buff, int bufsize);
static void writeIToSerial(u_i* buff, int bufsize);

Adafruit_BMP085 bmp;
TinyGPS gps;
SoftwareSerial nss(GPS_RX_INDEX, GPS_TX_INDEX);
byte byteRead;


// Buffers to hold sensor data
u_f gpsBuffer[3];
u_f bmpBuffer[1];
u_i ppmBuffer[3];

void setup() {
  nss.begin(57600);

  Serial.begin(115200);
  if (!bmp.begin()) {    
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


unsigned long start = millis();

void loop() {
  unsigned long newmillis = millis();
  if (newmillis - start > 1000) {
    feedgps();
    start = newmillis;
  }
  
  gpsdump(gps, gpsBuffer);
  bmpdump(bmp, bmpBuffer);
  ppmdump(ppmBuffer);
  
  writeFToSerial(gpsBuffer, 3);
  writeFToSerial(bmpBuffer, 1);
  writeIToSerial(ppmBuffer, 3);
}

static void writeFToSerial(u_f* buff, int bufsize) {
  for(int i = 0; i < bufsize; i++) {
    Serial.write(buff[i].b[0]);
    Serial.write(buff[i].b[1]);
    Serial.write(buff[i].b[2]);
    Serial.write(buff[i].b[3]);
  }
}

static void writeIToSerial(u_i* buff, int bufsize) {
  for(int i = 0; i < bufsize; i++) {
    Serial.write(buff[i].b[0]);
    Serial.write(buff[i].b[1]);
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

static void gpsdump(TinyGPS &gps, u_f* output) {
  float flat, flon, falt;
  unsigned long age, date, time, chars = 0;

  gps.f_get_position(&flat, &flon, &age);
  falt = gps.f_altitude();
  output[0].f = flat;
  output[1].f = flon;
  output[2].f = falt;
  feedgps();
}

static void bmpdump(Adafruit_BMP085 &bmp, u_f* output) {    
  float falt = bmp.readAltitude(101500);
  output[0].f = falt;
}

static void ppmdump(u_i* output) {    
  uint16_t rollIn =  CRCArduinoPPMChannels::getChannel(SERVO_ROLL);
  uint16_t pitchIn =  CRCArduinoPPMChannels::getChannel(SERVO_PITCH);
  uint16_t throttleIn =  CRCArduinoPPMChannels::getChannel(SERVO_THROTTLE);
  output[0].i = rollIn;
  output[1].i = pitchIn;
  output[2].i = throttleIn;
}

/* DB 04/02/2012 REMOVED The interrupt service routine definition here, it clashes with the attachInterrupt in the cpp file */
/* REMOVE BEGIN  
ISR(INT0_vect) {
 CRCArduinoPPMChannels::INT0ISR();
}
REMOVE END */
