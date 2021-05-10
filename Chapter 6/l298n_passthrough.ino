int enA = 9;
int in1 = 8;
int in2 = 7;
int in3 = 5;
int in4 = 4;
int enB = 3;

int enAVal, in1Val, in2Val, in3Val, in4Val, enBVal;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(enB, OUTPUT);
}

void loop() {
  // Only work if there is data in the serial buffer
  while(Serial.available() > 0){

    // Read the ints from the serial port
    enAVal = Serial.parseInt();
    in1Val = Serial.parseInt();
    in2Val = Serial.parseInt();
    // Only read the next three if there is data
    if(Serial.available() > 0){    
      in3Val = Serial.parseInt();
      in4Val = Serial.parseInt();
      enBVal = Serial.parseInt();
    }

    // Write the values to the L298N
    analogWrite(enA, enAVal);
    digitalWrite(in1, in1Val);
    digitalWrite(in2, in2Val);
    digitalWrite(in3, in3Val);
    digitalWrite(in4, in4Val);
    analogWrite(enB, enBVal);

    // Purge any remaining data because we don't need it
    while(Serial.available() > 0){
      char x = Serial.read();
    }
  }
}

