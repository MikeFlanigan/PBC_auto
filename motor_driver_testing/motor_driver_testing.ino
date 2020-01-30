// test sketch for the L298N Motor Drive Controller

bool Forward = true;


int motorPin = 5;    

void setup() {
  pinMode(motorPin, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(7, OUTPUT);
  Serial.begin(115200); // console debugging
}

void loop() {

  if (Forward) {
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);
  }
  else {
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
  }

  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(motorPin, fadeValue);
    analogWrite(13, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    Serial.println(fadeValue);
    delay(100);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(motorPin, fadeValue);
    analogWrite(13, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    Serial.println(fadeValue);
    delay(100);
  }
}
