int trigMid = 2;
int echoMid = 3;
int trigLeft = 4;
int echoLeft = 5;
int trigRight = 6;
int echoRight = 7;
float distMid = 0.0;
float distLeft = 0.0;
float distRight = 0.0;
String serialString;

void setup() {
  // set the pinModes for the sensors
  pinMode(trigMid, OUTPUT);
  pinMode(echoMid, INPUT);
  pinMode(trigLeft, OUTPUT);
  pinMode(echoLeft, INPUT);
  pinMode(trigRight, OUTPUT);
  pinMode(echoRight, INPUT);

  // set trig pins to low;
  digitalWrite(trigMid,LOW);
  digitalWrite(trigLeft,LOW);
  digitalWrite(trigRight,LOW);

  // starting serial
  Serial.begin(115200);
}

// function to operate the sensors
// returns distance in centimeters
float ping(int trigPin, int echoPin){
  // Private variables, not available
  // outside the function
  int duration = 0;
  float distance = 0.0;

  // operate sensor
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // get results and calculate distance
  duration = pulseIn(echoPin, HIGH);
  distance = duration/58.2;

  // return the results
  return distance;
}

void loop() {
  // get the distance for each sensor
  distMid = ping(trigMid, echoMid);
  distLeft = ping(trigLeft, echoLeft);
  distRight = ping(trigRight, echoRight);

  // write the results to the serial port
  Serial.print(distMid); Serial.print(",");
  Serial.print(distLeft); Serial.print(",");
  Serial.println(distRight);

  // wait 500 milliseconds before looping
  delay(500);
}