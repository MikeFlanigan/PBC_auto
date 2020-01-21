// test sketch for the L298N Motor Drive Controller

bool Forward = true;


int ledPin = 9;    // LED connected to digital pin 9

void setup() {
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  Serial.begin(115200); // console debugging
}

void loop() {

  if (Forward) {
    digitalWrite(8, HIGH);
    digitalWrite(9, LOW);
  }
  else {
    digitalWrite(8, LOW);
    digitalWrite(9, HIGH);
  }

  // fade in from min to max in increments of 5 points:
  for (int fadeValue = 0 ; fadeValue <= 255; fadeValue += 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    analogWrite(13, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    Serial.println(fadeValue);
    delay(100);
  }

  // fade out from max to min in increments of 5 points:
  for (int fadeValue = 255 ; fadeValue >= 0; fadeValue -= 5) {
    // sets the value (range from 0 to 255):
    analogWrite(ledPin, fadeValue);
    analogWrite(13, fadeValue);
    // wait for 30 milliseconds to see the dimming effect
    Serial.println(fadeValue);
    delay(100);
  }
}
