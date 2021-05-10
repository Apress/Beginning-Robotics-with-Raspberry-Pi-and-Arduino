int analogPin = A0;
int digitalPin = A1;

float analogVal = 0.0;
int digitalVal = 0;

void setup() {
  pinMode(analogPin, INPUT);
  pinMode(digitalPin, INPUT);

  Serial.begin(9600);
}

void loop() {
  analogVal = analogRead(analogPin);
  digitalVal = digitalRead(digitalPin);

  Serial.print("analogVal: "); Serial.print(analogVal);
  Serial.print(" - digitalVal: "); Serial.println(digitalVal);

  delay(500);
}

