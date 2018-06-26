/*
 Name:		균형로봇.ino
 Created:	2018-06-20 오후 2:04:16
 Author:	UKC
*/

/* -----------------------모터 드라이버 세팅--------------------------- */

#define EN1 10
#define EN2 5
// 모터의 속도값

#define IN1 9
#define IN2 8
#define IN3 7
#define IN4 6
// 모터의 전후진 제어 트랜지스터 핀 

#define motor_std_speed 37
#define sensor_delay 0.006

#define buzzer 12

/* ---------------------------계산식-----------------------------*/

#define Kp 1.7
#define Ki 1
#define Kd 0

/* ----------------------자이로센서----------------------- */

#include <Wire.h>

const int MPU = 0x68;//MPU6050 I2C주소
int AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ; //읽어온 값을 저장하는 변수
void get6050(); // 자이로 센서 값을 읽어오는 함수

/* --------------------- PID 컨트롤러 함수 선언부 ------------------*/

int P();
int I();
int D();

char flag; //모터 방향 플래그
unsigned long step = millis(); // 센서 주기


void add_motor(int speed); //모터에 값을 적용시키는 함수

/* ----------------------------------------------------------------- */

void back(int pwd);
void forward(int pwd);
void left(int pwd);
void right(int pwd);
void stop_motor();

void setup() {
	/* 자이로 센서 설정 */
	Wire.begin();
	Wire.beginTransmission(MPU);
	Wire.write(0x6B);
	Wire.write(0);//MPU6050 을 동작 대기 모드로 변경
	Wire.endTransmission(true);

	Serial.begin(9600);

	pinMode(EN1, OUTPUT);
	pinMode(EN2, OUTPUT);

	pinMode(IN1, OUTPUT);
	pinMode(IN2, OUTPUT);
	pinMode(IN3, OUTPUT);
	pinMode(IN4, OUTPUT);

	pinMode(buzzer, OUTPUT);
	tone(buzzer, 800, 200);

	stop_motor();
}

/* ----------------------------------------------------------------- */

int sum;

void loop() {

	sum = P();
	//sum = +I();
	add_motor(sum);

	Serial.print("모터값 = ");
	Serial.print(sum);
	Serial.println();
}
/* ----------------------------------------------------------------- */

void get6050() {
	Wire.beginTransmission(MPU);//MPU6050 호출
	Wire.write(0x3B);//AcX 레지스터 위치 요청
	Wire.endTransmission(false);
	Wire.requestFrom(MPU, 14, true);//14byte의 데이터를 요청
	AcX = Wire.read() << 8 | Wire.read();//두개의 나뉘어진 바이트를 하나로 이어붙입니다.
	AcY = Wire.read() << 8 | Wire.read();
	AcZ = Wire.read() << 8 | Wire.read();
	Tmp = Wire.read() << 8 | Wire.read();
	GyX = Wire.read() << 8 | Wire.read();
	GyY = Wire.read() << 8 | Wire.read();
	GyZ = Wire.read() << 8 | Wire.read();
}

/* ---------------------------계산식---------------------------------- */

int P() { // 비례식
	
	// 안정범위는 1200 ~ -1200으로 임의로 설정을 하였습니다.
	get_X(); //센서 값을 읽어옴
	int speed = get_ang_x(); //기울어진 각도를 저장

	if (speed > 93) { // 넘어진 여부를 판단 
		Serial.println("넘어짐");
		stop_motor();

		tone(buzzer, 350, 200);
		delay(400);
		tone(buzzer, 450, 200);

		delay(5000);
	}
	else if (speed > 10) { // 안정범위 밖일 경우
		Serial.println("기울어짐");
		return  Kp * speed + motor_std_speed; //Kp * 오차범위 + (최소 전압값)
	}
	else { // 안정범위 안에 있을 경우
		Serial.println("안정범위");
		return -1; // 안정범위라는 것을 전달
	}

	Serial.println();
}

int I() { // 적분식
	get_X(); //센서값을 읽음
	int a = Ki * get_ang_x() * (sensor_delay); //Ki * 오차범위 * 센서주기
	
	return a;
}

int D() { // 미분식

}

void add_motor(int sum) { // 계산식에서 계산된 값을 적용
	if (sum == -1) {
		stop_motor();
		return;
	}

	if (flag <= 0)
		forward(sum);
	else
		back(sum);
}


/* -------------------모터 제어 함수--------------------------- */

void back(int pwd) {
	analogWrite(EN1, pwd);
	analogWrite(EN2, pwd);

	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

void forward(int pwd) {
	analogWrite(EN1, pwd);
	analogWrite(EN2, pwd);

	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void left(int pwd) {
	analogWrite(EN1, pwd);
	analogWrite(EN2, pwd);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, HIGH);
	digitalWrite(IN3, HIGH);
	digitalWrite(IN4, LOW);
}

void right(int pwd) {
	analogWrite(EN1, pwd);
	analogWrite(EN2, pwd);
	digitalWrite(IN1, HIGH);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, HIGH);
}

void stop_motor() {
	analogWrite(EN1, 0);
	analogWrite(EN2, 0);
	digitalWrite(IN1, LOW);
	digitalWrite(IN2, LOW);
	digitalWrite(IN3, LOW);
	digitalWrite(IN4, LOW);
}


// --------------------- 자이로센서 함수 ------------------ //

void Read_6050() {
	//받아온 센서값을 출력합니다.
	Serial.print("AC : ");
	Serial.print(AcX);
	Serial.print("  ");
	Serial.print(AcY);
	Serial.print("  ");
	Serial.print(AcZ);
	Serial.println();

	Serial.print("Tmp : ");
	Serial.print(Tmp / 340 + 36.53);
	Serial.println();

	Serial.print("Gy : ");
	Serial.print(GyX);
	Serial.print("  ");
	Serial.print(GyY);
	Serial.print("  ");
	Serial.print(GyZ);
	Serial.println();
	Serial.println();

	delay(1000);
}

int get_ang_x() {
	int a = abs(AcX) / 180;

	Serial.print("각도 : ");
	Serial.println(a);

	return a;
}

void get_X() {
	int a=0;
	
	flag = 0;

	for (int i = 0; i < 6; i++) {
		get6050();
		if (AcX > 0) {
			flag++;
		}
		else {
			flag--;
		}
		a += AcX / 6;
		delay(10);
	}

	AcX = a;
	return;
}