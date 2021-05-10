int numbers[5];
int moreNumbers[5] = {1,2,3,4,5};

void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
for(int i = 0; i < 5; i++){
  Serial.println(numbers[i]);
  }

for(int i = 0; i < 5; i++){
  numbers[i] = moreNumbers[i];
}

for(int i = 0; i < 5; i++){
  Serial.println(numbers[i]);
  }

numbers[1] = 12;

for(int i = 0; i < 5; i++){
  Serial.println(numbers[i]);
  }
}

