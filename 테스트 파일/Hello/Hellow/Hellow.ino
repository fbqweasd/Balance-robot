#include <Wire.h>

#define dt 0.004 //센서주기
#define H 5 //바퀴 속도

const int MPU=0x68;//MPU6050 I2C주소
int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; //자이로센서 값 저장 변수

int M1dirpin = 4; 
int M2dirpin = 7; //전진 후진 컨트롤 핀

int M1steppin = 5;
int M2steppin = 6; //스탭 조정 핀
//여기는 모터 세팅

void get6050();
void get_s();

class PID_UKC{
  int Kp,Ki,Kd;
  int data, data_l;

  public: PID_UKC(int Kp,int Ki, int Kd){
     this->Kp = Kp;
     this->Ki = Ki;
     this->Kd = Kd;
     
     get_s(); 
     
     data_l = AcY; //나중에 수정해여
     data = AcY;
  }

  void data_init(){
    data = AcY;
    
    if(data < 0)
      data = -data;
 }

 
  int PID_Math_P(){ //비례식을 구하는 함수
    int P;
    P = Kp * data;
    return P;
  }

  int PID_Math_I(){ //적분항 구하는 함수
    int I;
    I = Ki * data * dt;
    return I;
  }

  int PID_Math_D(){ //미분항 구하는 함수
    int D;
    D = Kd * ( data - data_l / dt );
    return D;
  }

  int PID_Math(){ //PID값 계산 하는 함수
    int P,I,D;

    data_init();

    P = PID_UKC::PID_Math_P();
    I = PID_UKC::PID_Math_I();
    D = PID_UKC::PID_Math_D();

    data_l = data;
    
    return P + I + D;
  }
};

void get6050(){  //엄청 중요한 함수! 자이로 센서의 값을 읽어와요!
  Wire.beginTransmission(MPU);//MPU6050 호출
  Wire.write(0x3B);//AcX 레지스터 위치 요청
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);//14byte의 데이터를 요청
  AcX=Wire.read()<<8|Wire.read();//두개의 나뉘어진 바이트를 하나로 이어붙입니다.
  AcY+=(Wire.read()<<8|Wire.read()) / 5;
  AcZ=Wire.read()<<8|Wire.read();
  Tmp=Wire.read()<<8|Wire.read();
  GyX=Wire.read()<<8|Wire.read();
  GyY=Wire.read()<<8|Wire.read();
  GyZ=Wire.read()<<8|Wire.read();
}

void get_s(){
  AcY = 0;
  for(int i = 0;i<5;i++){
    get6050();
    delay(5);
  }
}

void setup() {
  /* 자이로 센서 설정 */
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);//MPU6050 을 동작 대기 모드로 변경
  Wire.endTransmission(true);
  
  Serial.begin(9600); //이건 다 알죠? 시리얼 통신입니다!
  Serial.println("시리얼은 초코볼");
  /* 이 친구는 모터의 핀을 세팅 해주어요 */
  pinMode(M1dirpin,OUTPUT);
  pinMode(M1steppin,OUTPUT);
  pinMode(M2dirpin,OUTPUT);
  pinMode(M2steppin,OUTPUT);

  get_s();
}
  PID_UKC *test_UKC = new PID_UKC(1,2,1);

void loop(){
  get_s();
  
  Serial.print("PID 값 : " );
  Serial.println(test_UKC->PID_Math());
}

