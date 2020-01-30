#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>


/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

SoftwareSerial mySerial(8, 7);
Adafruit_GPS GPS(&mySerial);
#define GPSECHO  false


void setup() {

  Serial.begin(115200);
  delay(5000);
  Serial.println("Adafruit GPS library basic test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);


  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_2HZ);
  GPS.sendCommand(PGCMD_ANTENNA);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);


  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");

  /* Initialise the sensor */
  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);

  }

  /* Display some basic information on this sensor */
  displaySensorDetails();
  Serial.println("Using Boulder CO magnetic declination");
}

uint32_t timer = millis();
void loop() {
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.13; // Boulder
  //  float declinationAngle = 0.19; // San Diego
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Head (deg) "); Serial.println(headingDegrees);


  char c = GPS.read();

  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences!
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  if (timer > millis())  timer = millis();
  //    if (millis() - timer > 2000) {
  if (millis() - timer > 200) {
    timer = millis(); // reset the timer

    //    Serial.print("\nTime: ");
    //    Serial.print(GPS.hour, DEC); Serial.print(':');
    //    Serial.print(GPS.minute, DEC); Serial.print(':');
    //    Serial.print(GPS.seconds, DEC); Serial.print('.');
    //    Serial.println(GPS.milliseconds);
    //    Serial.print("Date: ");
    //    Serial.print(GPS.day, DEC); Serial.print('/');
    //    Serial.print(GPS.month, DEC); Serial.print("/20");
    //    Serial.println(GPS.year, DEC);
    Serial.println(" ");
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    if (GPS.fix) {
      Serial.print("Loc ");
      Serial.print(GPS.latitudeDegrees, 6); //Serial.print(GPS.lat);
      Serial.print(", ");
      Serial.println(GPS.longitudeDegrees, 6); //Serial.println(GPS.lon);

      Serial.print("Spd (knts) "); Serial.println(GPS.speed);
      Serial.print("Ang "); Serial.println(GPS.angle);
      //      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Sats "); Serial.println((int)GPS.satellites);
      Serial.println(" ");
    }
  }
}
