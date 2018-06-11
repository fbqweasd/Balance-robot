#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define rxPin 12          //rx pin 설정
#define txPin 11          //tx pin 설정
#define EN1  10
#define IN1  9 
#define IN2  8
#define IN3  7
#define IN4  6
#define EN2  5

#define pwmMax  200  // 전후 방향 pwm 의 최댓값
#define pwmMin  -200  // 전후 방향 pwm 의 최솟값

SoftwareSerial swSerial(txPin, rxPin); // 소프트웨어 시리얼 통신 사용 설정

MPU6050 accelgyro;   // mpu6050 객체 생성

float angle = 0.0;        // 지금 읽어 온 전후 방향의 로봇 각도
float oldAngle = 0.0;    // 기존 전후 방향의 로봇 각도
float deltaRotateAngle;   // 안드로이드 폰에서 읽어 온 회전 명령의 각도
float readYaw;                 // 센서에서 읽어 온 yaw 각도 (z 방향 각도)
float lastreadYaw;           // 기존 yaw 각도
float deltareadYaw;         // 읽어 온 yaw 각도의 변화량
float rotateAngle;            // 현재 로봇의 yaw 각

float pwm = 0.0;              // 전후 방향 제어의 계산된 pwm 값
float ITerm =0;                // 전후 방향 제어의 계산된 적분 값
float compRotate = 0.0;   // 회전 방향 제어의 계산된 pwm 값
float yawITerm = 0.0;      // 회전 방향 제어의 계산된 적분 값

float runAdd = 0.0;         // 전 후진을 위하여 안정하기 위한 pwm 값에 더해주는 값
float deltaRunAdd = 0.0;  // runAdd 의 변화량
int MaxAdd = 6;                 // runAdd 의 최대 변화량
float delta_set_angle = 0.0;    // F+1, B+1, 에 의해서 변하는 값. 전후 방향의 기준 각도를 임시로 변경하기 위한 값.

float dt = 0.0;          // 한 loop 에 걸린 시간
long lastTime = 0;    // 이전 loop 의 현재 위치에서의 시간

String receivedCommand = "";    // 새로운 명령어를 저장하기 위한 문자열
int countChar = 0;                        // 하나의 명령어가 다 들어 온 경우 처리해야 하는 문자의 개수

char which;                                   // 안드로이드 폰에서 오는 문자열의 명령어 구분 문자 
String what = "";                            // 명령어와 함께 오는 내용, 주로 값이 온다.

int messageLength = 0;                  // 안드로이드 폰에서 온 처리되지 않은 문자의 개수
char sendMessage[20];                  // 안드로이드 폰에서 온 처리되지 않은 문자열을 저장하기위한 배열

boolean OK = false;                        // 명령어 끝 문자 '#' 이 오면 true
boolean MOTOR_ON = false;         // true 이면 모터 구동을 시작한다.
boolean CALIBRATE = false;           // true 이면 mpu6050 calibration 을 시작한다.
boolean SAVE = false;                      // 안드로이드 폰에서 Save 버튼을 누르면 true, 변수 값들을 EEPROM 에 저장 시작.
boolean CONNECTED = true; 

const int MPU_ID_ADDR = 98;
const int SENSOR_ADDR = 100;    // mpu6050 보정 값을 EEPROM 에 저장하기 시작하는 주소
byte *p;                                               // EEPROMSaving() 에서 사용하는 포인터
const byte EEPROM_ID = 0x98;      // 아래의 DATAS 자료를 저장한 적이 있는지 확인하기 위한 상수
const byte MPU_ID = 0x60;
const int ID_ADDR = 0;                     // EEPROM_ID 가 저장되는 EEPROM 주소
const int START_ADDRESS = 1;     // EEPROM 에 DATAS 를 저장하기 시작하는 주소
int start = START_ADDRESS;          // EEPROMSaving() 에서 저장 시작 주소


struct SENSOR               // mpu6050 의 calibration 값을 저장하는 구조체
{
  boolean calib;                 // calibraton 을 할 것인지 여부.
  float ax_off;                     // x 방향 가속도 보정 값.
  float ay_off;                      // y 방향 가속도 보정 값.
  float az_off;                      // z 방향 가속도 보정 값.
  float gx_off;                       // x 방향 각속도 보정 값
  float gy_off;                      // y 방향 각속도 보정 값
  float gz_off;                       // z 방향 각속도 보정 값
} sensor{ 0,0.0,0.0,0.0,0.0,0.0,0.0 };

struct DATAS                  // 밸런싱 로봇의 밸런싱 및 주행에 관한 데이타를 저장하는 구조체
{
  float setAngle;               // 전후 방향의 중심을 잡을 때 기준이 되는 각도. 
  float kp;                         // 전후 방향의 비례 제어 상수
  float ki;                          // 전후 방향의 적분 제어 상수
  float kd;                         // 전후 방향의 미분 제어 상수
  float YsetAngle;            // 좌우 회전에 대한 제어를 할 때 제어의 목표가 되는 yaw 각
  float RCoeffi;                // 좌우 회전 명령에 대한 민감도. 값이 커지면 강하게 회전한다.
  float Ykp;                     // yaw 제어의 비례 제어 상수
  float Yki;                       // yaw 제어의 적분 제어 상수
  float Ykd;                      // yaw 제어의 미분 제어 상수
  int MaxAdd;                   // 전후 방향 이동시 명령에 대한 민감도
} datas = { -6.0, 20.0, 150.0, 1.45, 0.0, 10.0, 3.0, 0.7, 0.7, 6};


//*******************************************  DMP VARIABLES **********************************************************
/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//*******************************************  DMP VARIABLES END **********************************************************

//*******************************************  CALIBRATION VARIABLES **********************************************************
/* ==========  LICENSE  ==================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2011 Jeff Rowberg
 
 Permission is hereby granted, free of charge, to any person obtaining a copy
 of this software and associated documentation files (the "Software"), to deal
 in the Software without restriction, including without limitation the rights
 to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 copies of the Software, and to permit persons to whom the Software is
 furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 THE SOFTWARE.
 =========================================================
 */
///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
//MPU6050 accelgyro(0x68); // <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset,calib;
//*******************************************  CALIBRATION VARIABLES END **********************************************************


void setup() 
{
    swSerial.begin(9600);       // 소프트웨어 시리얼 통신 설정
    Serial.begin(115200);        // 시리얼 통신 설정

     byte id = EEPROM.read(ID_ADDR);       // EEPROM  의 ID_ADDR 에서 한 바이트를 읽어 옴
     if( id == EEPROM_ID)                              // 참이면 기존에 저장 한 값이 있음
     {
       ReadFromEEprom(&datas,START_ADDRESS);     // EEPROM 에서 값을 읽어 와서 datas 에 저장
     }
     else // 거짓이면
     {
        EEPROM.write(ID_ADDR,EEPROM_ID);                    // EEPROM 의 ID_ADDR 에 EEPROM_ID 를 기록
        WritePIDintoEEPROM(&datas, START_ADDRESS);   // datas 구조체의 값들을 EEPROM의 START_ADDRESS 부터 기록
     }

    byte mpu_id = EEPROM.read(MPU_ID_ADDR);  // EEPROM 의 MPU_ID_ADDR 에서 한 바이트를 읽어서 mpu_id 에 저장
    if(mpu_id == MPU_ID)                                            // 참이면 기존에 저장된 값이 있음.
    {
         ReadFromEEprom(&sensor, SENSOR_ADDR);              // EEPROM 의 SENSOR_ADDR 에서 값을 읽어 와서 sensor 구조체에 저장
         CALIBRATE = sensor.calib;
    }

    pinsInit();                             // 아두이노 핀들의 pinMode 설정
    gyroInit();                             // mpu6050 초기화

    if(!accelgyro.testConnection())     // mpu6050 의 연결 체크, 연결이 안되면
    {
      swSerial.print("H#");                      // 안드로이드 폰으로 "H#" 송신
      CALIBRATE = false;                      // calibrate 하지 않음.
      MOTOR_ON = false;                    // motor 정지
      CONNECTED = false;
    }
      
    if(CALIBRATE)   startCalibrate();                                             // calibrate 하는 것이면 calibrate 시작
    else  
    {
      for(int i = 0; i < 300; i++)    
        {
           getAngles();                                   // calibrate 하는 것이 아니면 센서 안정을 위하여 각도를 300번 읽어 온다.
           Serial.println(angle);
        }
    }
   
    lastTime = millis();
    p = (byte*) &datas;                      // 바이트 형 포인터 p 에 datas 구조체의 시작 주소를 대입.

    swSerial.print("Q#");                     // 안드로이드 폰에 "Q#" 송신
 }


void loop()
{
    if(!CONNECTED) swSerial.print("H#");                      // mpu6050 이 연결되어 있지 않으면 안드로이드 폰으로 "H#" 송신
    
    getCommand();                                     // 안드로이드 폰에서 온 데이타가 있는지 확인 하고 있으면 처리함.  
    
    if(SAVE) EEPROMSaving();      // Save 상태이면 EEPROM 에 datas 구조체의 data를 한 바이트 저장

    if(messageLength > 0) manageMessage();          // 아직 처리되지 않은 수신 문자가 있으면 manageMessage() 실행

     getAngles();                                         // 각도를 읽어 온다.


    if(fabs(angle - datas.setAngle) < 60.0 )      // 로봇의 각도가 60 도 미만인 경우 모터를 작동시킨다. 
     {  
        dt = getdt();                                            // 한 loop 동안 걸린 시간을 계산한다.
//        getCommand();                                     // 안드로이드 폰에서 온 데이타가 있는지 확인 하고 있으면 처리함.
        if(fabs(deltaRotateAngle) > 10.0)  make_turnAdd();  // 일정 크기 이상의 회전 명령이 들어 오면 회전량을 계산
        pwm = cal_PID();                                       // 전후 방향 pwm 값을 계산
        compRotate = cal_yaw_PID();                   // yaw 각에 대한 pwm 값을 계산
        runAdd += deltaRunAdd;                           // 전후 방향으로 추가할 pwm 값을 계산
     }
    else  MOTOR_ON = false;                           // 로봇 각도가 60 도 이상 이면 모터 정지 
    Car_Cont();                                                    // motor 회전을 결정하기 위한 함수 
}

void Car_Cont()
{
     int left, right;
     int left_pwm, right_pwm;
     if(MOTOR_ON) 
     {
          int temp = int(pwm + runAdd);                 // 중심잡기 pwm 과 주행을 위한 추가 pwm 값을 더한 후 정수로 변환

          if(temp > 0) temp = min(int(pwm + runAdd),pwmMax);   // pwmMax 을 넘지 못하도록 제한.
          else temp = max(int(pwm + runAdd),pwmMin);               // pwmMin 아래로 내려가지 못하도록 제한
          
          left = int(temp - compRotate);                                              // pwm 에 compRotate 보정 후 정수로
          right = int(temp + compRotate);                                           // pwm 에 compRotate 보정 후 정수로

          left_pwm = min(abs(left),255);                                             // 255 이하로 제한
          right_pwm = min(abs(right),255);                                        // 255 이하로 제한
     
          if(left < 0 && right < 0) GO_FORWARD(left_pwm,right_pwm);            // 좌우 바퀴 모터의 pwm 값을 각각 주고 전진
          if(left < 0 && right > 0) TURN_RIGHT(left_pwm,right_pwm);                // 좌우 바퀴 모터의 pwm 값을 각각 주고 우회전
          if(left > 0 && right < 0) TURN_LEFT(left_pwm,right_pwm);                  // 좌우 바퀴 모터의 pwm 값을 각각 주고 좌회전
          if(left > 0 && right > 0) GO_BACKWARD(left_pwm,right_pwm);          // 좌우 바퀴 모터의 pwm 값을 각각 주고 후진
     }
     else MOTOR_STOP();
}

void MOTOR_STOP()
{
  analogWrite(EN1, 0);               // 왼쪽 모터 pwm 값 0  
  analogWrite(EN2,0);                // 오른쪽 모터 pwm 값 0
  digitalWrite(IN1,LOW);              // IN1, IN2 두 개로 왼쪽 모터의 회전 방향 결정, 정지 상태 
  digitalWrite(IN2,LOW);
  digitalWrite(IN3,LOW);             // IN3, IN4 두 개로 오른쪽 모터의 회전 방향 결정, 정지 상태 
  digitalWrite(IN4,LOW);  
}

void GO_BACKWARD(int pwm_l, int pwm_r)
{
  analogWrite(EN1, pwm_l);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(EN2,pwm_r);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);
}

void GO_FORWARD(int pwm_l, int pwm_r)
{
  analogWrite(EN1, pwm_l);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(EN2,pwm_r);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);  
}

void TURN_LEFT(int pwm_l, int pwm_r)
{
  analogWrite(EN1, pwm_l);
  digitalWrite(IN1,LOW);
  digitalWrite(IN2,HIGH);
  analogWrite(EN2,pwm_r);
  digitalWrite(IN3,HIGH);
  digitalWrite(IN4,LOW);  
}

void TURN_RIGHT(int pwm_l, int pwm_r)
{
  analogWrite(EN1, pwm_l);
  digitalWrite(IN1,HIGH);
  digitalWrite(IN2,LOW);
  analogWrite(EN2,pwm_r);
  digitalWrite(IN3,LOW);
  digitalWrite(IN4,HIGH);  
}

float cal_yaw_PID()                                      // yaw 에 대한 pwm 계산
{ 
  static float lastRotateAngle = 0.0;              // 전 loop 에서의 yaw 각
  float angularVel,calPwm,error;                   // 각속도, pwm 값, 오차

  error = rotateAngle - (datas.YsetAngle);     // 현재 yaw 각과 목표하는 yaw 각과의 오차
  if(error < -180.0) error += 360.0;                 // 오차가 +,- 180도 보다 크면 180도 이내로 조정
  else if(error > 180.0) error -= 360.0;
      
  yawITerm += datas.Yki * error * dt;              // yaw 적분값 계산 
  if(dt >= 0.0003)                                              // dt 가 너무 작으면 오류로 판단하여 계산하지 않음
  {
     angularVel = (rotateAngle - lastRotateAngle) / dt;  // 각속도 계산
     lastRotateAngle = rotateAngle;                               // 현재 yaw 각 저장
     calPwm = datas.Ykp * error + yawITerm + datas.Ykd * angularVel;     // P + I + D 값 계산
  }
  else calPwm = compRotate;                                   // 오류인 경우 기존의 pwm 값을 그대로 적용
  return calPwm;                                                        // pwm 값 리턴
}

float cal_PID()                     // ballancing pid 계산, cal_yaw_PID() 주석 참조
{ 
  static float lastAngle = 0.0;
  float  angularVel, calPwm, error;

  error = angle - (datas.setAngle - delta_set_angle);   
  ITerm += datas.ki * error * dt; 
  if(dt >= 0.0003)
  {
     angularVel = (angle - lastAngle) / dt;  // Differentiation
     lastAngle = angle;
    
     calPwm = datas.kp * error + ITerm + datas.kd * angularVel;
  }
  else calPwm = pwm;
  return calPwm;
}

void readDMP()
{
  if (!dmpReady) return;
  while (!mpuInterrupt && fifoCount < packetSize) { }
  mpuInterrupt = false;
  mpuIntStatus = accelgyro.getIntStatus();

  // get current FIFO count
  fifoCount = accelgyro.getFIFOCount();
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
        // reset so we can continue cleanly
        accelgyro.resetFIFO();
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 
  else if (mpuIntStatus & 0x02) 
  {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = accelgyro.getFIFOCount();

        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        accelgyro.dmpGetGravity(&gravity, &q);
        accelgyro.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
} 

void pinsInit()
{
    pinMode(EN1, OUTPUT);
    pinMode(EN2,OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    digitalWrite(IN1,LOW);
    digitalWrite(IN2,LOW);
    digitalWrite(IN3,LOW);
    digitalWrite(IN4,LOW);  
}

void gyroInit()
{
     Wire.begin();                                                          // I2C 통신 시작
     accelgyro.initialize();                                              // mpu6050 초기화                                  
     devStatus = accelgyro.dmpInitialize();                   // DMP 초기화

     accelgyro.setXGyroOffset(sensor.gx_off);            // 각각의 보정값 입력
     accelgyro.setYGyroOffset(sensor.gy_off);
     accelgyro.setZGyroOffset(sensor.gz_off);
     accelgyro.setXAccelOffset(sensor.ax_off);
     accelgyro.setYAccelOffset(sensor.ay_off);     
     accelgyro.setZAccelOffset(sensor.az_off);       
   
     if (devStatus == 0)                                                 // dmpInitialize 가 성공하면
     {
       accelgyro.setDMPEnabled(true);                         // DMP 사용 가능
       attachInterrupt(0, dmpDataReady, RISING);       // 2번 핀의 하드웨어 인터럽트를 선언
       mpuIntStatus = accelgyro.getIntStatus();           // mpu6050 의 인터럽트 상태를 가져온다.
       dmpReady = true;                                              // DMP 초기화 성공
       packetSize = accelgyro.dmpGetFIFOPacketSize();     // DMP packetSize 를 읽어온다.
     }  
}

void getAngles()
{
    readDMP();                                                      // DMP 를 사용하여 각도를 읽어 온다.
    angle = ypr[1] * 180 / M_PI;                             // pitch 각을 읽어와 전후 방향의 각도로 사용한다.
    readYaw = ypr[0] * 180 / M_PI;                       // yaw 각을 읽어온다.
    if(readYaw - lastreadYaw < -180.0) deltareadYaw = readYaw - lastreadYaw + 360.0;     // yaw 값의 변화량을 계산한다.
    else if(readYaw - lastreadYaw > 180.0) deltareadYaw = readYaw - lastreadYaw - 360.0;
    else deltareadYaw = readYaw - lastreadYaw;
    if(deltareadYaw > 10.0) 
    {
      deltareadYaw = 10.0;               // yaw 값의 변화량을 10도 이하로 제한한다.
    }
    else if(deltareadYaw < -10.0)
    {
      deltareadYaw = -10.0;
    }
    lastreadYaw = readYaw; 
    rotateAngle += deltareadYaw;           // 로봇의 rotate angle 을 계산한다.
  
    if(fabs(rotateAngle) < 180.0 && fabs(readYaw) < 180.0) rotateAngle = readYaw;
    if(angle - oldAngle >= 5.0) angle = oldAngle + 5.0;         // pitch 각의 변화를 5도 이내로 제한한다.
    else if(angle - oldAngle <= -5.0) angle = oldAngle - 5.0;
    oldAngle = angle;
}


void make_turnAdd()    // 회전 명령이 들어온 경우 
{
     if(deltaRunAdd >= 0.0)      // 전진하면서 회전
     {
         datas.YsetAngle =  rotateAngle + deltaRotateAngle;  // 목표 yaw 값을 변경한다.
         deltaRunAdd = 2.0;                                                      // 원활한 회전을 위하여 전진 방향으로 밀어주는 양을 제한.
     }
     else 
     {
         datas.YsetAngle =  rotateAngle - deltaRotateAngle;  
         deltaRunAdd = -2.0;
     }
}
        
void getCommand()
{
     if(swSerial.available()) {                                                                      // 안드로이드 폰에서 수신된 문자가 있는 경우
        char received = swSerial.read();                                                     // 한 개의 문자를 읽어 온다.
        
        if(received >= 'a' && received <= 'k')
        {
           deltaRunAdd =  float(('f' - received ) * datas.MaxAdd * 0.17);       // 전진 방향의 크기를 읽어와서 더해 줄 값을 계산한다.
        }
        else if(received >= 'm' && received <= 'q')                                      // 회전 방향의 크기를 읽어와서 더해 줄 값을 계산한다.
        {
          deltaRotateAngle = float (( received - 'o' ) * datas.RCoeffi);         
        } 
        else
        {
          sendMessage[messageLength] = received;                  // 전진 또는 회전 이외의 다른 명령어인 경우 처리되지 않은 명령어 문자열에 추가 한다.
          messageLength++;                                                        // 문자의 개수를 1개 증가시킨다.
        }
     }
}

void running_value_init()
{
  ITerm = 0.0;       
  runAdd = 0.0; 
  deltaRunAdd = 0.0; 
  pwm = 0.0;
  datas.YsetAngle =  rotateAngle;                     // 목표 yaw 값을 현재의 각도로 설정
  compRotate = 0.0;
  yawITerm = 0.0; 
  lastTime = millis();
}



 float convertToFloat(String str, int strLength)      // 문자열을 실수 값으로 변환한다.
 {
     int where;
     int count = 0;
     boolean decimalPoint = false;
     float sign = 1.0;
     float res = 0.0;

     if(strLength == 0) return 10000.0;
     where = str.indexOf('.');
     for(int i = 0; i< strLength; i++)
     {
         if(isDigit(str[i])) 
         {
             res = (res *10)+(str[i]-'0') * 1.0;
             if(decimalPoint) count ++;
         }
         else if(str[i] == '-') sign = -1;
         else if(str[i] == '.') decimalPoint = true;
         else return 10000.0;
     }
     if(where >= 0) res = res * sign / pow(10,count);
     else res = res * sign;
     return res;
}

void manageMessage()
    {
         receivedCommand = receivedCommand + sendMessage[0];                           // 처리해야할 명령어 문자열에 수신된 문자 1개를 추가한다.
         countChar ++;                                                                                                     // 문자 개수 1 증가
         if(sendMessage[0] == '#') OK = true;                                                                  // 추가된 문자가 '#' 인 경우, 한 개의 명령어 종료 판단.
         for(int i = 0; i < messageLength ; i++)   sendMessage[i] = sendMessage[i+1];  // 수신된 문자들을 배열에서 1칸씩 앞으로 이동
         messageLength--;                                                                                                // 문자 개수 1 감소
         if(OK)    // 한개의 명령어 문자열이 완성되면
         {
            which = receivedCommand[0];                                                                      // 첫번째 문자를 읽어서 작업 내용 판단.
            for(int i=1; i < countChar -1; i++) what = what + receivedCommand[i];        // 나머지 숫자에 해당하는 문자열을 읽어서 보관
            executeCommand(which, what);                                                                  // 각 명령에 따른 작업을 실시
            receivedCommand = "";                                                                                // 초기화
            what = "";                                                                                                       // 초기화
            countChar = 0;                                                                                              // 초기화
            OK = false;                                                                                                    // 초기화
         }
    }
  
void executeCommand(char command, String value)
{
    float temp_value = 0.0;
    
    switch(command)
    {
             case 'A' :  temp_value = convertToFloat( value, value.length());                  // 들어온 문자열을 실수로 변환한다.
                              if(fabs(temp_value) > 9999.0) temp_value = -datas.setAngle;    // 문자열이 공백으로 들어왔다면 데이터 값을 temp_value 에 저장하고
                              else  datas.setAngle = -temp_value;     // 숫자로 들어왔다면 들어온 숫자를 데이터에 저장. - 부호는 실제 동작시 편하게 느끼도록 부호 조정
                              swSerial.print("A");                                // 들어 온 작업 내용과 
                              swSerial.print(temp_value);                  // 그에 해당하는 데이터 값과
                              swSerial.print("#");                                 // 명령어 종료 문자를 안드로이드 폰에 전송 
                              break;
             case 'w' : temp_value = convertToFloat( value, value.length());
                              if(fabs(temp_value) > 9999.0) temp_value = datas.RCoeffi;
                              else  datas.RCoeffi = temp_value;
                              swSerial.print("A"); 
                              swSerial.print(temp_value);
                              swSerial.print("#");
                             break;                             
             case 'P' : temp_value = convertToFloat( value, value.length());
                             if(fabs(temp_value) > 9999.0) temp_value = datas.kp;
                             else datas.kp = temp_value;
                             swSerial.print("P");
                             swSerial.print(temp_value);
                             swSerial.print("#");
                             break;
             case 'x' : temp_value = convertToFloat( value, value.length());
                            if(fabs(temp_value) > 9999.0) temp_value = datas.Ykp;
                            else datas.Ykp = temp_value;
                            swSerial.print("P");
                            swSerial.print(temp_value);
                            swSerial.print("#");
                            break;
             case 'I' : temp_value = convertToFloat( value, value.length());
                           if(fabs(temp_value) > 9999.0) temp_value = datas.ki;
                            else datas.ki = temp_value;
                            swSerial.print("I");
                            swSerial.print(temp_value);
                            swSerial.print("#");
                            break;
             case 'y' : temp_value = convertToFloat( value, value.length());
                            if(fabs(temp_value) > 9999.0) temp_value =datas.Yki;
                            else datas.Yki = temp_value;
                            swSerial.print("I");
                            swSerial.print(temp_value);
                            swSerial.print("#");
                            break;
             case 'D' : temp_value = convertToFloat( value, value.length());
                             if(fabs(temp_value) > 9999.0) temp_value = datas.kd;
                            else datas.kd = temp_value;
                            swSerial.print("D");
                            swSerial.print(temp_value);
                            swSerial.print("#");
                            break;
             case 'z' : temp_value = convertToFloat( value, value.length());
                             if(fabs(temp_value) > 9999.0) temp_value = datas.Ykd;
                            else datas.Ykd = temp_value;
                            swSerial.print("D");
                            swSerial.print(temp_value);
                            swSerial.print("#");
                            break;                                                                                                                                                                                                    
             case 'F':  delta_set_angle += 1.0;
                           swSerial.print("F");
                            swSerial.print(delta_set_angle);
                            swSerial.print("#");
                            break;            
            case 'B' : delta_set_angle -= 1.0;
                            swSerial.print("B");
                            swSerial.print(delta_set_angle);
                            swSerial.print("#");
                            break;  
            case 'Z' : delta_set_angle = 0.0;
                            swSerial.print("Z");
                            swSerial.print(delta_set_angle);
                            swSerial.print("#");
                            break;
            case 'E' : sensor.calib = true;
                            EEPROM.write(MPU_ID_ADDR,MPU_ID);                    // EEPROM 의 MPU_ID_ADDR 에 MPU_ID 를 기록
                            Serial.println(EEPROM.read(MPU_ID_ADDR));
                            WritePIDintoEEPROM(&sensor, SENSOR_ADDR);     // EEPROM 의 SENSOR_ADDR 에 sensor data 를 기록
                            break;                                                          
            case 'O' : MOTOR_ON = true;
                             running_value_init();    
                             swSerial.print("M");
                             swSerial.print(datas.MaxAdd);
                             swSerial.print("#");
                             break;
            case 'X' :  MOTOR_ON = false; 
                             break;       
            case 'L' :  datas.YsetAngle -= 5.0;
                            break; 
            case 'R' : datas.YsetAngle += 5.0;
                            break;      
            case 'M' : datas.MaxAdd ++;
                            swSerial.print("M");
                            swSerial.print(datas.MaxAdd);
                            swSerial.print("#");
                            break; 
            case 'W' : datas.MaxAdd --;
                             if(datas.MaxAdd < 1) datas.MaxAdd = 1;
                            swSerial.print("M");
                            swSerial.print(datas.MaxAdd);
                            swSerial.print("#");
                            break; 
            case 'S' :  EEPROM.write(ID_ADDR,EEPROM_ID);
                             SAVE = true;
                            break;                                                                                                            
           default :    break;
    }
      
    temp_value = 0.0;    
}

float getdt()
{
     long temp;
     long now = millis();
     temp = lastTime;
     lastTime = now;
     return float((now - temp)/1000.0);    
}

void WritePIDintoEEPROM(struct DATAS *SavingData, int START_ADDRESS)   // EEPROM 에 datas 의 값 저장 
{
  byte cnt = sizeof(struct DATAS);
  byte *pt = (byte*)SavingData;
  int start = START_ADDRESS;
  for(int i = 0; i < cnt; i++)
  {
     EEPROM.write(start++,*pt);                  // 시작 주소에서 1 byte 씩 증가시키며 data size 만큼 자료를 저장한다.
     pt++;
  } 
}

void WritePIDintoEEPROM(struct SENSOR *SavingData, int START_ADDRESS)     // EEPROM 에 sensor 보정값 저장
{
  byte cnt = sizeof(struct SENSOR);
  byte *pt = (byte*)SavingData;
  int start = START_ADDRESS;
  for(int i = 0; i < cnt; i++)
  {
     EEPROM.write(start++,*pt);
     pt++;
  } 
}

void EEPROMSaving()           // EEPROM에 datas 의 값을 1 byte 씩 저장
{
    EEPROM.write(start, *p);
    start ++;
    p++;
    if(start > sizeof(struct DATAS))
    {
      start = START_ADDRESS;
      p = (byte*) &datas;
      SAVE = false;                    // 저장 완료되면 Save 상태 끝
    }
}

void ReadFromEEprom(struct DATAS *ReadingData, int START_ADDRESS)        // EEPROM 에서 datas 의 값을 읽어 온다.
{
    byte cnt = sizeof(struct DATAS);
    byte *pt = (byte*)ReadingData;
    int start = START_ADDRESS;
    for(int i = 0; i < cnt; i++)
    {
       *pt = EEPROM.read(start++);
       pt++;
    } 
}

void ReadFromEEprom(struct SENSOR *ReadingData, int START_ADDRESS)     // EEPROM 에서 sensor 보정 값을 읽어 온다.
{
    byte cnt = sizeof(struct SENSOR);
    byte *pt = (byte*)ReadingData;
    int start = START_ADDRESS;
    for(int i = 0; i < cnt; i++)
    {
       *pt = EEPROM.read(start++);
       pt++;
    } 
}

void startCalibrate()    // sensor 값을 보정한다.
    {
      // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
      TWBR = 12; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.
      // reset offsets
     accelgyro.setXAccelOffset(0);
     accelgyro.setYAccelOffset(0);
     accelgyro.setZAccelOffset(0);
     accelgyro.setXGyroOffset(0);
     accelgyro.setYGyroOffset(0);
     accelgyro.setZGyroOffset(0);

     if (state==0){
       meansensors();
       state++;
       delay(1000);
     }

     if (state==1) {
       calibration();
       state++;
       delay(1000);
     }

     if (state==2) {
        sensor.calib = false;
        sensor.ax_off = ax_offset;
        sensor.ay_off = ay_offset;
        sensor.az_off = az_offset;
        sensor.gx_off = gx_offset;
        sensor.gy_off = gy_offset;
        sensor.gz_off = gz_offset;
        EEPROM.write(MPU_ID_ADDR,MPU_ID);                    // EEPROM 의 MPU_ID_ADDR 에 MPU_ID 를 기록
        WritePIDintoEEPROM(&sensor, SENSOR_ADDR);
       }     
    }
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    swSerial.print("C#");
    
    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

