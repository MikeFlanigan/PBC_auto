unsigned long now;                        // timing variables to update data at a regular interval
unsigned long rc_update;
const int channels = 2;                   // specify the number of receiver channels
float RC_in[channels];                    // an array to store the calibrated input from receiver

const int autoManualPin = 5;
const int waypointPin = 6;

const int MotorEnAPin = 9;
const int MotorEnBPin = 10;
bool Forward = true;

void setup() {
  pinMode(autoManualPin, INPUT);
  pinMode(waypointPin, INPUT);
  setup_pwmRead();
  Serial.begin(115200);

  pinMode(MotorEnAPin, OUTPUT);
  pinMode(MotorEnBPin, OUTPUT);

  // send motor command
  if (Forward) {
    digitalWrite(MotorEnAPin, HIGH);
    digitalWrite(MotorEnBPin, LOW);
  }
  else {
    digitalWrite(MotorEnAPin, LOW);
    digitalWrite(MotorEnBPin, HIGH);
  }
}

void loop() {

  now = millis();




  if (RC_avail() || now - rc_update > 25) { // if RC data is available or 25ms has passed since last update (adjust to be equal or greater than the frame rate of receiver)

    rc_update = now;

    if (digitalRead(autoManualPin)) {
      Serial.print("Aut ");
    }
    else {
      Serial.print("Man ");
    }
    if (digitalRead(waypointPin)) {
      Serial.print("WpHi ");
    }
    else {
      Serial.print("WpLo ");
    }

    print_RCpwm();                        // uncommment to print raw data from receiver to serial

    for (int i = 0; i < channels; i++) {    // run through each RC channel
      int CH = i + 1;

      RC_in[i] = RC_decode(CH);             // decode receiver channel and apply failsafe

      print_decimal2percentage(RC_in[i]);   // uncomment to print calibrated receiver input (+-100%) to serial
    }
    //Serial.println();                       // uncomment when printing calibrated receiver input to serial.
  }
}
