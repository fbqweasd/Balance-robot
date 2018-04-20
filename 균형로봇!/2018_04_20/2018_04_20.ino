#include<Wire.h>  //자이로센서 사용 라이브러리
#define N  7900 //안정 상태일때 회전 속도
#define H  2000// 심각하게 기울어졌을 떄 속도

const int MPU=0x68;//MPU6050 I2C주소
int AcX,AcZ,Tmp,GyX,GyY,GyZ;
volatile int AcY;
void get6050();
//여기 까지는 자이로센서 세팅
/*
*/
int M1dirpin = 4;
int M1steppin = 5;
int M2dirpin = 7;
int M2steppin = 6;
//여기는 모터 세팅

void setup() {
  /* 자이로 센서 설정 */
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);//MPU6050 을 동작 대기 모드로 변경
  Wire.endTransmission(true);
  
  Serial.begin(9600); //이건 다 알죠? 시리얼 통신입니다!

  /* 이 친구는 모터의 핀을 세팅 해주어요 */
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);
}

void loop() {
  get_s(); // 센서 값 읽어옴 
  Serial.println(AcY);
  
  if (AcY > 1200)
  {
    forward();
    speed_up((17500 - AcY/100 *AcY/100),8);
  }
  else if(AcY < -700){
    back();
    speed_up((17500 - (AcY/100 *AcY/100)), 9);
  }
}

void get6050(){  //엄청 중요한 함수! 자이로 센서의 값을 읽어와요!
  Wire.beginTransmission(MPU);//MPU6050 호출
  Wire.write(0x3B);//AcX 레지스터 위치 요청
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);//14byte의 데이터를 요청
  AcX=Wire.read()<<8|Wire.read();//두개의 나뉘어진 바이트를 하나로 이어붙입니다.
  AcY+=(Wire.read()<<8|Wire.read()) / 10;
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
}
void get_s(){
  AcY = 0;
  for(int i = 0;i<10;i++){
    get6050();
    delay(4);
  }
  
}

void speed_up(int a, int sec) //모터를 1번 스탭을 주는 함수 즉 1.8도를 회전 시킴
{
  if (a <= 2) a = 2;
  //  if( sec < 7) sec = 7;
  for(int i=0;i<sec;i++){
    digitalWrite(M1steppin,LOW);
    digitalWrite(M2steppin,LOW);
    delayMicroseconds(a); //이 친구는 몇 초만에 1스탭회전 시킬지를 결정! 따라서 이걸로 속도를 조정할거임
    digitalWrite(M1steppin,HIGH);
    digitalWrite(M2steppin,HIGH);
    delay(1); //절대 건들지 마셈.. 이거 잘못건들면 모터가 상함
  }
 }

void back(){
  digitalWrite(M1dirpin,LOW);
  digitalWrite(M2dirpin,HIGH);
}

void forward(){
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

