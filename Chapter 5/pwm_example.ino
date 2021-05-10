int PWMPin = 11;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  for(int i = 0; i < 255; i++){
    analogWrite(PWMPin, i);
  }

  for(int i = 255; i >= 0; i--){
    analogWrite(PWMPin, i);
  }
  delay(100);
}

