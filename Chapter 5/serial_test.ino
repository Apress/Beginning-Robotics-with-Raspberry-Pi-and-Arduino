int trig = 7;
int echo = 8;
int duration = 0;
int distance = 0;

void setup() {
  Serial.begin(9600);
  pinMode(trig, OUTPUT);
  pinMode(echo, INPUT);

  digitalWrite(trig,LOW);
}

void loop() {
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  duration = pulseIn(echo, HIGH);
  distance = duration/58.2;

  Serial.write(distance);

  delay(500);
}

