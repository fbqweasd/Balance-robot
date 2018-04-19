int M1dirpin = 4;
int M1steppin = 5;
int M2dirpin = 7;
int M2steppin = 6;
void setup()
{
  Serial.begin(9600);
  
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
}
void loop()
{
  Serial.println("1번");
  digitalWrite(M1dirpin,LOW);
  digitalWrite(M2dirpin,LOW);
  speed_up();
  delay(1000);

  Serial.println("2번");
  digitalWrite(M1dirpin,HIGH);
  digitalWrite(M2dirpin,LOW);
  speed_up();
  delay(1000);

  Serial.println("3번");
  digitalWrite(M1dirpin,LOW);
  digitalWrite(M2dirpin,HIGH);
  speed_up();
  delay(1000);

  Serial.println("4번");
  digitalWrite(M1dirpin,HIGH);
  digitalWrite(M2dirpin,HIGH);
  speed_up();
  delay(1000);

}

void speed_up()
{
  int j;
   for(j=0;j<=10000;j++){
    digitalWrite(M1steppin,LOW);
    digitalWrite(M2steppin,LOW);
    delayMicroseconds(2);
    //delay(500);
    digitalWrite(M1steppin,HIGH);
    digitalWrite(M2steppin,HIGH);
    delay(1);
  }
}

