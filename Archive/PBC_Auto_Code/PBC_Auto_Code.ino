#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>
#include "EEPROMAnything.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  600 // this is the 'maximum' pulse length count (out of 4096)
#define ESCMIN 300
#define ESCMAX 600
#define RUDRSERVOMIN 250 // this equals a hard left turn for the boat
#define RUDRSERVOCENTER 375 // guesstimate
#define RUDRSERVOMAX 500 // this equals a hard right turn for the boat
int RudderServoChannel = 2; // channel number for right now
// channel 0 is the port foil
// channel 1 is the starboard foil
// channel 2 is the rudder
// channel 8 is the throttle

// ---------- IMU/Compass BNO055 setup -----------
/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)
Adafruit_BNO055 bno = Adafruit_BNO055();
// ------END of -- IMU/Compass BNO055 setup -----------

// ---------- GPS setup ----------------
Adafruit_GPS GPS(&Serial2);
HardwareSerial mySerial = Serial2;
#define GPSECHO  false
boolean usingInterrupt = true;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
// ------END of -- GPS setup ----------------

// ------------ Boat state variables --------
float Boat_Lat = 0;
float Boat_Lon = 0;
float Boat_Speed = 0;
float Boat_Trajectory_Angle = 0;
float Boat_Heading = 0;
// ------ END of --- Boat state variables --------

// ---------- nav and control variables ----------
float waypoint_radius = 1; // meters
float dist2wp = 0; // meters
float course2wp = 0; // compass degrees
int target_wp_index = 0;
float heading_error = 0; // abs diff between target and heading
bool turn_right = false;
float Rudder_P_gain = 1.0;
float Rudder_output = 0;
// ---- END of nav and control variables ---------

int waypoint_entry_count = 0;
bool first_waypoint_input = false; // true once the program has been booted and entered a waypoint
const int num_way_pts = 25; // shouldn't need more than 25 waypoints
float waypoint_array [num_way_pts][2];
float EEP_waypoint_array [num_way_pts][2];
// 32.7997, -117.2543


int NowTime = 0;

const int debug_Hz = 1;
const int debug_millis = 1000 / debug_Hz;
int debug_timer = 0;

const int waypoint_pin = 30;
const int auto_manual_pin = 40;

int waypoint_input = 0;
int old_waypoint_input = 0;

int AutoMode = 0;
int last_state_AutoMode = 0;

bool IdleMode = false;

void setup() {
  Serial.begin(115200);

  EEPROM_readAnything(0, EEP_waypoint_array);
  for (int y = 0; y < 25; y++) {
    waypoint_array[y][0] = EEP_waypoint_array[y][0];
    waypoint_array[y][1] = EEP_waypoint_array[y][1];
  }
  Serial.println("Waypoint waypoint_array read from EEPROM");
  for (int y = 0; y < 25; y++) {
    Serial.print(EEP_waypoint_array[y][0], 4);
    Serial.print(" ");
    Serial.println(EEP_waypoint_array[y][1], 4);
  }

  pinMode(waypoint_pin, INPUT);
  pinMode(auto_manual_pin, INPUT);

  // ---------- GPS setup -------------
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz
  useInterrupt(true);
  // -----END of -- GPS setup -------------

  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates


  // -------- BNO055 orientation setup ----------
  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
  // --- END of -- BNO055 orientation setup ----------
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;
  // writing direct to UDR0 is much much faster than Serial.print
  // but only one character can be written at a time.
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}


void loop() {
  NowTime = millis();

  // ------------ reading digitial inputs -------------
  waypoint_input = digitalRead(waypoint_pin);
  AutoMode = digitalRead(auto_manual_pin);
  // ------------ END of reading digitial inputs -------------

  if (GPS.newNMEAreceived()) {
    Boat_Lat = GPS.latitudeDegrees;
    Boat_Lon = GPS.longitudeDegrees;
    Boat_Speed = GPS.speed; // knots
    Boat_Trajectory_Angle = GPS.angle;

    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Boat_Heading = euler.x(); // BNO compass
  int offset = 95;
  if (euler.x() <= offset) {
    Boat_Heading = 360 + euler.x() - offset;
  }
  else {
    Boat_Heading = euler.x() - offset;
  }

  if (AutoMode == false) { ////////////////// MANUAL MODEEEEEEEEEE
    IdleMode = false; // definitely not in auto idle mode

    if (waypoint_input == HIGH && old_waypoint_input == LOW) {
      // rising edge on the waypoint signal
      old_waypoint_input = waypoint_input; // reset the change flag
      Serial.println("WAYPOINT INPUT RISING EDGE");

      // -------- clear the stored waypoint array if it's the first mark after boot ------------
      if (first_waypoint_input == false) {
        waypoint_entry_count = 0;
        first_waypoint_input = true; // reset change flag
        for (int y = 0; y < 25; y++) {
          waypoint_array[y][0] = 0.0;
          waypoint_array[y][1] = 0.0;
        }
        EEPROM_writeAnything(0, waypoint_array);
      }
      // -- END --- clear the stored waypoint array if it's the first mark after boot ------------

      // -------------- adding GPS lat lon to waypoint array ---------------
      //      waypoint_array[waypoint_entry_count][0] = 32.801968;
      //      waypoint_array[waypoint_entry_count][1] = -117.255461;
      waypoint_array[waypoint_entry_count][0] = Boat_Lat;
      waypoint_array[waypoint_entry_count][1] = Boat_Lon;
      waypoint_entry_count++;
      // -------- END --- adding GPS lat lon to waypoint array ---------------


      // print out waypoint array
      for (int y = 0; y < 25; y++) {
        Serial.print(waypoint_array[y][0], 4);
        Serial.print(" ");
        Serial.println(waypoint_array[y][1], 4);
      }
    }
    else if (waypoint_input == LOW && old_waypoint_input == HIGH) {
      // falling edge on the waypoint signal
      old_waypoint_input = waypoint_input; // reset the change flag
      EEPROM_writeAnything(0, waypoint_array);
    }
    //    else {
    //      old_waypoint_input = waypoint_input;
    //    }
  }
  else { // AUTONOMOUS MODEEEEEEEEEEE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (IdleMode == false) {

      first_waypoint_input = false;
      if (last_state_AutoMode == 0) { //////// means first time through so start at beginning of wp array
        // rising edge on autonomous mode
        target_wp_index = 0;
      }

      dist2wp = distance_to(Boat_Lat, waypoint_array[target_wp_index][0], Boat_Lon, waypoint_array[target_wp_index][1]) * 1000;
      if (dist2wp <= waypoint_radius) {
        Serial.println("SUCCESSFULLY HIT WAYPOINT!!!!!");
        target_wp_index++;
      }
      else if (dist2wp > 3000) {
        Serial.println("SKIPPING WAYPOINT, SHOULDNT BE MORE THAN 3KM AWAY");
        target_wp_index++;
      }
      else { // autonomous steering section
        course2wp = course_to(Boat_Lat, waypoint_array[target_wp_index][0], Boat_Lon, waypoint_array[target_wp_index][1]);

        /// go scoot and D!
        int delta = course2wp - Boat_Heading;

        if (delta < 0) {
          delta += 360;
        }
        if (delta > 180) {
          delta -= 360;
        }
        //delta is negative when off to right
        // positive when off to left

        delta /= 2; //divide by 2, scale the delta
        //        Serial.println(90 + delta);
        Rudder_output = map(90 + delta, 0, 180, RUDRSERVOMIN, RUDRSERVOMAX);
        /////////////////
      }
      if (target_wp_index >= 24) IdleMode = true;
    }
  }

  last_state_AutoMode = AutoMode; // update edge flag


  // ----------------- SERVO control ---------------
  pwm.setPWM(RudderServoChannel, 0, Rudder_output);
  // throttle setting
  if (IdleMode == false) {
    pwm.setPWM(8, 0, 420);
  }
  // ----------- END of --- SERVO control ---------------


  if (NowTime - debug_timer >= debug_millis) {
    debug_timer = NowTime; // reset the debug print timer

    Serial.print("State: ");
    Serial.println(" ");
    Serial.println(waypoint_array[0][0], 4);
    Serial.println(" ");
    if (IdleMode) Serial.print("IDLE MODE .... ");
    if (AutoMode) Serial.println("Autonomous");
    else Serial.println("Manual");
    Serial.println(Boat_Heading, 4);
    Serial.println(course2wp, 4);
    Serial.print(" heading error ");
    Serial.println(heading_error, 4);
    Serial.print(" rudder output ");
    Serial.println(Rudder_output, 4);
    Serial.println(turn_right);
    Serial.println(" ");

  }
}
