#include <Servo.h>


//Servo myservo;
//int ServoPos = 0; // 0 - 180
//const int ServoPin = 3;

bool Forward = true;
const int motorPin = 5;
const int motorEnA = 8;
const int motorEnB = 7;
int MotorSpeed = 0; // 0-255

int incomingByte = 0; // for incoming serial data
int Throttle = 0; // 0-100
int SteerAngle = 0; // -20 - 20

bool recievingMsg = false;
bool gettingSteer = false;
bool gettingThrot = false;

bool negativeSteer = false;

String newSteerMsg = "";
String newThrotMsg = "";

const int STEER_MSG = 115;
const int THROT_MSG = 116;
const int END_STR = 101;
const int END_THROT = 110;
const int SPACE = 32;
const int NEGATIVE = 45;

void setup() {
  Serial.begin(115200);

//  pinMode(ServoPin, OUTPUT);
//  myservo.attach(ServoPin);

  pinMode(motorPin, OUTPUT);
  pinMode(motorEnA, OUTPUT);
  pinMode(motorEnB, OUTPUT);
}

// example demo msg: s19e t52n would translate to steering 19 degrees, throttle 52%
void loop() {
  if (Serial.available() > 0) {
    recievingMsg = true;
    incomingByte = Serial.read();
    if (incomingByte == STEER_MSG || gettingSteer) { // getting a steering
      gettingSteer = true;

      if (incomingByte == NEGATIVE) {
        negativeSteer = true;
      }
      else if (incomingByte == END_STR) {
        gettingSteer = false;
        SteerAngle = newSteerMsg.toInt();
        if (negativeSteer) {
          SteerAngle = -1 * SteerAngle;
        }
//        Serial.print("steer");
//        Serial.println(SteerAngle);
        newSteerMsg = "";
      }
      else {
        if (incomingByte != STEER_MSG) {
          newSteerMsg += SerialToInts(incomingByte);
        }
      }
    }
    else if (incomingByte == THROT_MSG || gettingThrot) {
      gettingThrot = true;

      if (incomingByte == END_THROT) {
        gettingThrot = false;
        Throttle = newThrotMsg.toInt();
//        Serial.print("thorot");
//        Serial.println(Throttle);
        newThrotMsg = "";
        recievingMsg = false;
        negativeSteer = false; // reset the negative sign flag
      }
      else {
        if (incomingByte != THROT_MSG) {
          newThrotMsg += SerialToInts(incomingByte);
        }
      }
    }
    else if (incomingByte == SPACE) {
      //      Serial.println(" space ");
    }
    else {
      //      Serial.println("else");
      Throttle = 0;
      SteerAngle = 0;
    }
  }

  if (not recievingMsg) {
//    Serial.print("Steer: ");
//    Serial.print(SteerAngle);
//    Serial.print(" ");
//    Serial.print("Throt: ");
//    Serial.println(Throttle);

    // send steering command
//    ServoPos = map(SteerAngle, -20, 20, 5, 175); // note clipped the servo limits by a couple degrees
//    myservo.write(ServoPos);

    // send motor command
    if (Forward) {
      digitalWrite(motorEnA, HIGH);
      digitalWrite(motorEnB, LOW);
    }
    else {
      digitalWrite(motorEnA, LOW);
      digitalWrite(motorEnB, HIGH);
    }
    MotorSpeed = map(Throttle, 0, 100, 0, 255);
    analogWrite(motorPin, MotorSpeed);
  }

}

String SerialToInts(int incoming) {
  switch (incoming) {
    case 48:
      return "0";
    case 49:
      return "1";
    case 50:
      return "2";
    case 51:
      return "3";
    case 52:
      return "4";
    case 53:
      return "5";
    case 54:
      return "6";
    case 55:
      return "7";
    case 56:
      return "8";
    case 57:
      return "9";
  }
}

