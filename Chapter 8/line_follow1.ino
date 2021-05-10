int ir1Pin = A0;
int ir2Pin = A1;
int ir3Pin = A2;
int ir4Pin = A3;
int ir5Pin = A4;

int ir1Val = 0;
int ir2Val = 0;
int ir3Val = 0;
int ir4Val = 0;
int ir5Val = 0;

void setup() {
  pinMode(ir1Pin, INPUT);
  pinMode(ir2Pin, INPUT);
  pinMode(ir3Pin, INPUT);
  pinMode(ir4Pin, INPUT);
  pinMode(ir5Pin, INPUT);

  Serial.begin(9600);
}

void loop() {
  ir1Val = analogRead(ir1Pin);
  ir2Val = analogRead(ir2Pin);
  ir3Val = analogRead(ir3Pin);
  ir4Val = analogRead(ir4Pin);
  ir5Val = analogRead(ir5Pin);

  Serial.print(ir1Val); Serial.print(",");
  Serial.print(ir2Val); Serial.print(",");
  Serial.print(ir3Val); Serial.print(",");
  Serial.print(ir4Val); Serial.print(",");
  Serial.println(ir5Val);

  delay(100);
}

