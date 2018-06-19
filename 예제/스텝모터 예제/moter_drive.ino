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
  int i=0;
  for(i=0;i<1000;i++){
    forward();
    speed_up(2);
  }
  for(i=0;i<1000;i++){
    backward();
    speed_up(2);
  }
  for(i=0;i<1000;i++){
    left();
    speed_up(2);
  }
  for(i=0;i<1000;i++){
    right();
    speed_up(2);
  }
}

void speed_up(int a) //모터를 1번 스탭을 주는 함수 즉 1.8도를 회전 시킴
{
    digitalWrite(M1steppin,LOW);
    digitalWrite(M2steppin,LOW);
    delayMicroseconds(a); //이 친구는 몇 초만에 1스탭회전 시킬지를 결정! 따라서 이걸로 속도를 조정할거임
    digitalWrite(M1steppin,HIGH);
    digitalWrite(M2steppin,HIGH);
    delay(1); //절대 건들지 마셈.. 이거 잘못건들면 모터가 상함
 }

void forward(){
  digitalWrite(M1dirpin,LOW);
  digitalWrite(M2dirpin,HIGH);
}

void backward(){
  digitalWrite(M1dirpin,HIGH);
  digitalWrite(M2dirpin,LOW);
}

void left(){
  digitalWrite(M1dirpin,HIGH);
  digitalWrite(M2dirpin,HIGH);
}

void right(){
  digitalWrite(M1dirpin,LOW);
  digitalWrite(M2dirpin,LOW);
}

void stop_moter(){
  digitalWrite(M1steppin,HIGH);
  digitalWrite(M2steppin,HIGH);
}

