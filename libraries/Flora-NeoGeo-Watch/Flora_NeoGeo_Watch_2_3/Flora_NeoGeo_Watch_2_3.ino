
// Test code for Adafruit Flora GPS modules
//
// This code shows how to listen to the GPS module in an interrupt
// which allows the program to have more 'freedom' - just parse
// when a new NMEA sentence is available! Then access data when
// desired.
//
// Tested and works great with the Adafruit Flora GPS module
// ------> http://adafruit.com/products/1059
// Pick one up today at the Adafruit electronics shop
// and help support open source hardware & software! -ada

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>
#include <Time.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// -----------------------------------------------------------------------------
// Config
// -----------------------------------------------------------------------------

// --------------------------------------------------|
//                     WAYPOINT                      |
// --------------------------------------------------|
// Please enter the latitude and longitude of your   |
// desired destination:                              |
#define GEO_LAT               52.507100              // 48.009551 - 52.507200 - 52.5072231 - 52.507100
#define GEO_LON               13.309660              //-88.771131 - 13.309600 - 13.3096588 - 13.309660
// --------------------------------------------------|
// Your NeoPixel ring may not line up with ours.     |
// Enter which NeoPixel led is your top LED (0-15).  |
#define TOP_LED               0
// --------------------------------------------------|
// Your compass module may not line up with ours.    |
// Once you run compass mode, compare to a separate  |
// compass (like one found on your smartphone).      |
// Point your TOP_LED north, then count clockwise    |
// how many LEDs away from TOP_LED the lit LED is    |
#define LED_OFFSET             6
#define LED_MODE               -1
// --------------------------------------------------|

#if true

// -----------------------------------------------------------------------------
// Variables, objects
// -----------------------------------------------------------------------------

Adafruit_GPS GPS(&Serial1);

// Assign a unique ID to this sensor at the same time
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(39791);

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// this keeps track of whether we're using the interrupt off by default!
boolean usingInterrupt = true;

// Navigation location
float targetLat = GEO_LAT;
float targetLon = GEO_LON;

// Trip distance
float tripDistance;

// strip object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(16, 6, NEO_GRB + NEO_KHZ800);

// Offset hours from gps time (UTC)
// TBD: try to estimate from GPS location - default UTC
const int offset = 1;   // Central European Time          CET
//const int offset = 2;     // Central European Summer Time   CEST
//const int offset = -4;  // Eastern Daylight Time (USA)    EDT
//const int offset = -5;  // Central Daylight Time (USA)    CDT
//const int offset = -8;  // Pacific Standard Time (USA)    PST
//const int offset = -7;  // Pacific Daylight Time (USA)    PDT

int topLED = TOP_LED;
int compassOffset = LED_OFFSET;

// NeoPixelring
#define numOfLEDs 16    //  16 LEDs

int lastMin = numOfLEDs;
int lastHour = numOfLEDs;
int startLED = 0;
int startLEDlast = numOfLEDs;
int lastCombined = 0;
int start = 0;
int mode = 0;
int lastDir = numOfLEDs;
int dirLED_r = 0;
int dirLED_g = 0;
int dirLED_b = 255;

int compassReading;

// -- Calibration offsets
float magxOffset = 2.55;
float magyOffset = 27.95;

// -- Pushbutton setup
int buttonPin = 10;               // the number of the pushbutton pin
int buttonState;                  // the current reading from the input pin
int lastButtonState = HIGH;       // the previous reading from the input pin
long buttonHoldTime = 0;          // the last time the output pin was toggled
long buttonHoldDelay = 2500;      // how long to hold the button down

// -- the following variables are long's because the time, measured in miliseconds,
//    will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;        // the last time the output pin was toggled
long debounceDelay = 50;          // the debounce time; increase if the output flickers
long menuDelay = 2500;
long menuTime;

float fLat = 0.0;
float fLon = 0.0;

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------

void setup()
{
  // -- connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  //    also spit it out
  Serial.begin(115200);
  Serial.println("Adafruit GPS library basic test!");

  // -- 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);

  //GPS.sendCommand(PMTK_CMD_HOT_START);

  // -- uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  // -- uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  //    For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  //    the parser doesn't care about other sentences at this time

  // -- Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
  //    For the parsing code to work nicely and have time to sort thru the data, and
  //    print it out we don't suggest using anything higher than 1 Hz

  // -- Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  // -- Initialise the sensor
  if (!mag.begin())
  {
    // -- There was a problem detecting the LSM303 ... check your connections
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  // -- Ask for firmware version
  Serial1.println(PMTK_Q_RELEASE);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  // -- Make input & enable pull-up resistors on switch pins for pushbutton
  pinMode(buttonPin, INPUT);
  digitalWrite(buttonPin, HIGH);
}

// -----------------------------------------------------------------------------
// Timers
// -----------------------------------------------------------------------------

uint32_t gpsTimer = millis();
uint32_t startupTimer = gpsTimer;
uint32_t compassTimer = gpsTimer;

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------


void loop() // run over and over again
{
  compassCheck();
  // -- read the state of the switch into a local variable:
  int buttonState = digitalRead(buttonPin);

  if (buttonState == LOW) {
    buttonCheck();
  }

  if (lastButtonState != buttonState) Serial.println(buttonState);

  lastButtonState = buttonState;

  // -- read data from the GPS in the 'main loop'
  char c = GPS.read();
  
  // -- if you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) Serial.print(c);
    
  // -- if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    //    a tricky thing here is if we print the NMEA sentence, or data
    //    we end up not listening and catching other sentences!
    //    so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
    if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
      return; // we can fail to parse a sentence in which case we should just wait for another
  }

  // -- if millis() or timer wraps around, we'll just reset it
  if (gpsTimer > millis()) gpsTimer = millis();

  // -- GPS w/o First Fix (FF)
  if (start == 0) {

    // -- GPS fixed recently - maybe lost again later on
    if (GPS.fix) {
      // -- set the Time to the latest GPS reading
      setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
      delay(50);
      adjustTime(offset * SECS_PER_HOUR);
      delay(500);
      tripDistance = (double)calc_dist(fLat, fLon, targetLat, targetLon);

      // -- set GPS First Fixed (FF)
      start = 1;
    }
  }

  // -- GPS w First Fix (FF)
  //    approximately every 60 seconds or so, update time
  if ((millis() - gpsTimer > 60000) && (start == 1)) {
    gpsTimer = millis(); // reset the timer

    // -- GPS fixed recently - this maybe lost again 
    if (GPS.fix) {
      // -- set the Time to the latest GPS reading
      setTime(GPS.hour, GPS.minute, GPS.seconds, GPS.day, GPS.month, GPS.year);
      delay(50);
      adjustTime(offset * SECS_PER_HOUR);
      delay(500);
    }
  }

  // -- GPS fixed recently - this maybe lost again 
  if (GPS.fix) {
    fLat = decimalDegrees(GPS.latitude, GPS.lat);
    fLon = decimalDegrees(GPS.longitude, GPS.lon);
  }

  // -- state 0
  if (mode == 0) {
    clockMode();
  }

  // -- state 1
  if (mode == 1) {
    navMode();
  }

  // -- state 2
  if (mode == 2) {
    compassMode();
  }
}

// -----------------------------------------------------------------------------
// ColorWipe
// -----------------------------------------------------------------------------

// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for (uint16_t i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}

// -----------------------------------------------------------------------------
// ButtonCheck
// -----------------------------------------------------------------------------

void buttonCheck() {
  menuTime = millis();
  int buttonState = digitalRead(buttonPin);
  if (buttonState == LOW && lastButtonState == HIGH) {
    buttonHoldTime = millis();
  }

  if (buttonState == LOW && lastButtonState == LOW) {
    if ((millis() - buttonHoldTime) > buttonHoldDelay) {

      // -- last state wraps around
      if (mode == 2) {
        mode = 0;
        lastMin = 16;
        lastHour = 16;
        colorWipe(strip.Color(0, 0, 0), 20);
        buttonHoldTime = millis();
      }

      // -- next state
      else {
        mode = mode + 1;
        colorWipe(strip.Color(0, 0, 0), 20);
        buttonHoldTime = millis();
      }
    }
  }
}

// -----------------------------------------------------------------------------
// ClockMode - Mode 0
// -----------------------------------------------------------------------------

void clockMode() {

  // -- GPS w/o First Fix (FF)
  if (start == 1) {
    strip.setPixelColor(startLEDlast, strip.Color(0, 0, 0));      // delete last start led
    strip.show();

    int _hour = hour();
    int _minute = minute();
    int _second = second();

    // -- calculate minutes float formated
    float gpsMin = (_minute + (_second / 60.0));                  // gps minute floating format
    unsigned int ledMin = 0;                                      // reset minute led
    int minTemp = 0;                                              // reset minute temp
    minTemp = topLED - (gpsMin + 1.875) / 3.75;                   // calculate minute temp (formula?) 60/16

    // -- normalize minute led
    if (minTemp < 0) {
      ledMin = minTemp + 16;
    }

    // -- take minute led
    else {
      ledMin = minTemp;
    }

    // -- calculate hours float formated
    float gpsHour = (_hour + (_minute / 60.0));                 // gps hour in floating format

    // -- normalize hour
    if (gpsHour > 12) {
      gpsHour = gpsHour - 12;
    }

    unsigned int ledHour = 0;                                     // reset hour led
    int hourTemp = 0;                                             // reset hour temp
    hourTemp = topLED - (gpsHour + .375) / .75;                   // calculate hour temp (formula?) 12/16

    // -- normalize houf led
    if (hourTemp < 0) {
      ledHour = hourTemp + 16;
    }

    // -- take hour led as is
    else {
      ledHour = hourTemp;
    }

    if (lastMin != ledMin) {
      Serial.print("  time:");
      Serial.print(_hour);
      Serial.print(":");
      Serial.print(_minute);
      Serial.print(":");
      Serial.print(_second);
      Serial.print("  GPS:");
      Serial.print(gpsHour);
      Serial.print(":");
      Serial.print(gpsMin);
      Serial.print("  TMP:");
      Serial.print(hourTemp);
      Serial.print(":");
      Serial.print(minTemp);
      Serial.print("  LED:");
      Serial.print(ledHour);
      Serial.print(":");
      Serial.print(ledMin);
      Serial.println("");
    }

    // -- identical hour and minute led (once only)
    if ((ledHour == ledMin) && (lastCombined == 0)) {
      strip.setPixelColor(lastHour, strip.Color(0, 0, 0));        // delete last hour led
      strip.setPixelColor(lastMin, strip.Color(0, 0, 0));         // delete last minute led
      strip.setPixelColor(ledHour, strip.Color(255, 0, 255));     // set hour (= minute) led to puple
      strip.show();
      lastCombined = 1;                                           // save last combined led
      lastHour = ledHour;                                         // save last hour led
      lastMin = ledMin;                                           // save last minute led
    }

    // -- different hour and minute leds
    else {

      // -- hour changed
      if (lastHour != ledHour) {
        strip.setPixelColor(lastHour, strip.Color(0, 0, 0));      // delete last hour led
        strip.setPixelColor(ledHour, strip.Color(255, 50, 0));    // set hour led to orange
        strip.show();
        lastHour = ledHour;                                       // save last hour led
      }

      // -- minute changed
      if (lastMin != ledMin) {
        strip.setPixelColor(lastMin, strip.Color(0, 0, 0));       // delete last minute led
        strip.setPixelColor(ledMin, strip.Color(200, 200, 0));    // set minute led to yellow

        // -- last combined led
        if (lastCombined == 1) {
          strip.setPixelColor(ledHour, strip.Color(255, 0, 0));   // set hour led to red
          lastCombined = 0;                                       // reset last combined
        }

        strip.show();
        lastMin = ledMin;                                         // save last minute led
      }
    }
  }

  // -- GPS w/o First Fix (FF)
  else {
    // if millis() or timer wraps around, we'll just reset it
    if (startupTimer > millis()) startupTimer = millis();

    // approximately every 200 milliseconds
    if (millis() - startupTimer > 200) {
      startupTimer = millis(); // reset the timer
      if (startLED == 16) {
        startLED = 0;
      }
      strip.setPixelColor(startLEDlast, strip.Color(0, 0, 0));    // reset 
      strip.setPixelColor(startLED, strip.Color(0, 255, 0));      // set to green
      strip.show();
      startLEDlast = startLED;
      startLED++;
      //delay(200);
    }
  }
}

// -----------------------------------------------------------------------------
// NavMode - Mode 1
// -----------------------------------------------------------------------------

void navMode() {

  // -- GPS w First Fix (FF)
  if (start == 1) {

    compassCheck();

    headingDistance((double)calc_dist(fLat, fLon, targetLat, targetLon));

    if ((calc_bearing(fLat, fLon, targetLat, targetLon) - compassReading) > 0) {
      compassDirection(calc_bearing(fLat, fLon, targetLat, targetLon) - compassReading);
    }
    else {
      compassDirection(calc_bearing(fLat, fLon, targetLat, targetLon) - compassReading + 360);
    }

  }

  // -- GPS w/o First Fix (FF)
  else {
    // if millis() or timer wraps around, we'll just reset it
    if (startupTimer > millis()) startupTimer = millis();

    // approximately every 200 milliseconds 
    if (millis() - startupTimer > 200) {
      startupTimer = millis(); // reset the timer
      if (startLED == 16) {
        startLED = 0;
      }
      strip.setPixelColor(startLEDlast, strip.Color(0, 0, 0));    //
      strip.setPixelColor(startLED, strip.Color(0, 0, 255));      // blue 
      strip.show();
      startLEDlast = startLED;
      startLED++;
    }
  }
}

// -----------------------------------------------------------------------------
// CalcBearing
// -----------------------------------------------------------------------------

int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1);
  float y = 69.1 * (flon2 - flon1) * cos(flat1 / 57.3);

  calc = atan2(y, x);

  bear_calc = degrees(calc);

  if (bear_calc <= 1) {
    bear_calc = 360 + bear_calc;
  }
  return bear_calc;
}

// -----------------------------------------------------------------------------
// HeadingDistance
// -----------------------------------------------------------------------------

void headingDistance(int fDist)
{
  //Use this part of the code to determine how far you are away from the destination.
  //The total trip distance (from where you started) is divided into five trip segments.
  float tripSegment = tripDistance / 5;

  if (fDist >= (tripSegment * 4)) {
    dirLED_r = 255;
    dirLED_g = 0;
    dirLED_b = 0;
  }

  if ((fDist >= (tripSegment * 3)) && (fDist < (tripSegment * 4))) {
    dirLED_r = 255;
    dirLED_g = 0;
    dirLED_b = 0;
  }

  if ((fDist >= (tripSegment * 2)) && (fDist < (tripSegment * 3))) {
    dirLED_r = 255;
    dirLED_g = 255;
    dirLED_b = 0;
  }

  if ((fDist >= tripSegment) && (fDist < (tripSegment * 2))) {
    dirLED_r = 255;
    dirLED_g = 255;
    dirLED_b = 0;
  }

  if ((fDist >= 5) && (fDist < tripSegment)) {
    dirLED_r = 255;
    dirLED_g = 255;
    dirLED_b = 0;
  }

  if ((fDist < 5)) { // You are now within 5 meters of your destination.
    Serial.println("Arrived at destination!");
    dirLED_r = 0;
    dirLED_g = 255;
    dirLED_b = 0;
  }
}

// -----------------------------------------------------------------------------
// CalcDist
// -----------------------------------------------------------------------------

// TBD: try to use chip.calc

unsigned long calc_dist(float flat1, float flon1, float flat2, float flon2)
{
  float dist_calc = 0;
  float dist_calc2 = 0;
  float diflat = 0;
  float diflon = 0;

  diflat = radians(flat2 - flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  diflon = radians((flon2) - (flon1));

  dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  dist_calc2 = cos(flat1);
  dist_calc2 *= cos(flat2);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc2 *= sin(diflon / 2.0);
  dist_calc += dist_calc2;

  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));

  dist_calc *= 6371000.0; //Converting to meters
  return dist_calc;
}

// -----------------------------------------------------------------------------
// DecimalDegrees
// -----------------------------------------------------------------------------

// Convert NMEA coordinate to decimal degrees
float decimalDegrees(float nmeaCoord, char dir) {
  uint16_t wholeDegrees = 0.01 * nmeaCoord;
  int modifier = 1;

  if (dir == 'W' || dir == 'S') {
    modifier = -1;
  }

  return (wholeDegrees + (nmeaCoord - 100.0 * wholeDegrees) / 60.0) * modifier;
}

// -----------------------------------------------------------------------------
// CompassMode - Mode 2
// -----------------------------------------------------------------------------

void compassMode() {
  dirLED_r = 0;                         //
  dirLED_g = 0;                         //
  dirLED_b = 255;                       // set led to blue
  compassDirection(compassReading);
}

// -----------------------------------------------------------------------------
// CompassCheck
// -----------------------------------------------------------------------------

void compassCheck() {
  // if millis() or timer wraps around, we'll just reset it
  if (compassTimer > millis()) compassTimer = millis();

  // approximately every 50 milliseconds
  if (millis() - compassTimer > 50) {
    //Get a new sensor event
    sensors_event_t event;
    mag.getEvent(&event);

    float Pi = PI;           //3.14159;

    compassTimer = millis(); // reset the timer

    // Calculate the angle of the vector y,x
    float heading = (atan2(event.magnetic.y + magyOffset, event.magnetic.x + magxOffset) * 180) / Pi;

    // Normalize to 0-360
    if (heading < 0)
    {
      heading = 360 + heading;
    }
    compassReading = heading;
  }
}

// -----------------------------------------------------------------------------
// CompassDirection
// -----------------------------------------------------------------------------

void compassDirection(int compassHeading)
{
  Serial.print("Compass Direction: ");
  Serial.println(compassHeading);

  unsigned int ledDir = 2;
  int tempDir = 0;
  //Use this part of the code to determine which way you need to go.
  //Remember: this is not the direction you are heading, it is the direction to the destination (north = forward).

  if ((compassHeading > 348.75) || (compassHeading < 11.25)) {
    tempDir = topLED;
  }
  for (int i = 1; i < 16; i++) {
    float pieSliceCenter = 45 / 2 * i;
    float pieSliceMin = pieSliceCenter - 11.25;
    float pieSliceMax = pieSliceCenter + 11.25;
    if ((compassHeading >= pieSliceMin) && (compassHeading < pieSliceMax)) {
      if (mode == 2 ) {
        tempDir = topLED - i;
      }
      else {
        tempDir = topLED + i;
      }
    }
  }

  if (tempDir > 15) {
    ledDir = tempDir - 16;
  }

  else if (tempDir < 0) {
    ledDir = tempDir + 16;
  }
  else {
    ledDir = tempDir;
  }

  if (mode == 1) {
    ledDir = ledDir + compassOffset;
    if (ledDir > 15) {
      ledDir = ledDir - 16;
    }
  }
  else {
    ledDir = ledDir + compassOffset;
    if (ledDir > 15) {
      ledDir = ledDir - 16;
    }
  }

  if (lastDir != ledDir) {
    strip.setPixelColor(lastDir, strip.Color(0, 0, 0));
    strip.setPixelColor(ledDir, strip.Color(dirLED_r, dirLED_g, dirLED_b));
    strip.show();
    lastDir = ledDir;
  }
}

#else

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:

}

#endif
// --- END ---
