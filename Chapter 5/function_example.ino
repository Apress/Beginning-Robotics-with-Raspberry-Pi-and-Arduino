int a = 1;
int b = 2;
int val;
int answer;

int add_vars(){
  val = a+b;
  return val;
}
int add_params(int p1, int p2){
  val = p1+p2;
  return val;
}

void printVal(){
  Serial.println(val);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  add_vars();
  printVal();

  add_params(a,b);
  printVal();

  answer = add_vars();
  Serial.println(answer);

  a++;
  b++;
}

