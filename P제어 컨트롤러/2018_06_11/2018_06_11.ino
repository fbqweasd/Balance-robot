/* ----------------------자이로센서----------------------- */

#include<Wire.h>

const int MPU=0x68;//MPU6050 I2C주소
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //읽어온 값을 저장하는 변수
void get6050(); // 자이로 센서 값을 읽어오는 함수

/* -----------------------모터 드라이버 세팅--------------------------- */

#define EN1 10
#define EN2 5
// 모터의 속도값

#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
// 모터의 전후진 제어 트랜지스터 핀 

/* --------------------- PID 컨트롤러 함수 선언부 ------------------*/

int P();

/* ----------------------------------------------------------------- */

void back(int pwd);
void forward(int pwd);
void left(int pwd);
void right(int pwd);
void stop_moter();

void setup() {
   /* 자이로 센서 설정 */
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);//MPU6050 을 동작 대기 모드로 변경
  Wire.endTransmission(true);
  
  Serial.begin(9600);

  pinMode(EN1,OUTPUT);
  pinMode(EN2,OUTPUT);
  
  pinMode(IN1,OUTPUT);  
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);

  stop_moter();
}

/* ----------------------------------------------------------------- */

void loop() {
    // P();
  back(50);
  Serial.println("back");
  delay(2000);
  
  forward(50);
  Serial.println("forward");
  delay(2000);
  
  left(50);
  Serial.println("left");
  delay(2000);
  
  right(50);
  Serial.println("right");
  delay(2000);
    
}

/* ----------------------------------------------------------------- */

void get6050(){
  Wire.beginTransmission(MPU);//MPU6050 호출
  Wire.write(0x3B);//AcX 레지스터 위치 요청
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);//14byte의 데이터를 요청
  AcX=Wire.read()<<8|Wire.read();//두개의 나뉘어진 바이트를 하나로 이어붙입니다.
  AcY=Wire.read()<<8|Wire.read();
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
}

/* ----------------------------------------------------------------- */

int P(){
  get6050; // 안정범위는 1000 ~ -1000으로 임의로 설정을 하였습니다.
  if( AcX < -1000 || AcX > 1000){ // 안정범위 밖일 경우
    //int bup = AcX / 500; 
    if(AcX > 0){
      back(50);
     }
    else if(AcX < 0){
      forward(50);
    } 
  }
  else{ // 안정범위 안에 있을 경우
    stop_moter();
  }
}

/* -------------------모터 제어 함수--------------------------- */

void back(int pwd){
  analogWrite(EN1,pwd);
  analogWrite(EN2,pwd);

  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void forward(int pwd){
  analogWrite(EN1,pwd);
  analogWrite(EN2,pwd);

  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void left(int pwd){
  analogWrite(EN1,pwd);
  analogWrite(EN2,pwd);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);
}

void right(int pwd){
  analogWrite(EN1,pwd);
  analogWrite(EN2,pwd);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void stop_moter(){
  analogWrite(EN1,0);
  analogWrite(EN2,0);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,LOW);
}

