int M1dirpin = 4;
int M1steppin = 5;
int M2dirpin = 7;
int M2steppin = 6;

void setup() {
  Serial.begin(9600);
  
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

}


