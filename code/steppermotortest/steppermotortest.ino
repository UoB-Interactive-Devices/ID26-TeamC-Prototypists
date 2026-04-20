// pin connections
const int dirPin = 2; // direction pin
const int stepPin = 3; // step pin

void setup() {
  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  // set direction of rotation to clockwise
  digitalWrite(dirPin, HIGH);
}

void loop() {
  // take one step
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(2000);
  // pause before taking next step
  digitalWrite(stepPin, LOW);
  delayMicroseconds(2000);
}