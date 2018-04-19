int m1dir = 4;
int m1stp = 5;
int m2dir = 7;
int m2stp = 6;

int d =1;
int a=0;

void setup() {
   Serial.begin(9600);

  //모터 핌모드 설정
  pinMode(m1dir,OUTPUT);
  pinMode(m1stp,OUTPUT);
  pinMode(m2dir,OUTPUT);
  pinMode(m2stp,OUTPUT);
}

void loop() {
  if (a <  1000)  //만약 a가 1000보다 작을때, 
  {
    a++;//a의 값을 계속 증가시키고,
    digitalWrite(m1stp, HIGH);  //각각의 stp를 HIGH로 합니다,
    digitalWrite(m2stp, HIGH); 
    delay(d);               //d밀리초만큼 대기한뒤
    digitalWrite(m1stp, LOW);  // stp를 LOW로 합니다.
    digitalWrite(m2stp, LOW); 
    delay(d); // d밀리초만큼 대기합니다.
  } else     //아닐경우,
  {
    digitalWrite(m1dir, HIGH);//dir을 HIGH로 출력하고,
    digitalWrite(m2dir, HIGH);
    a++;//a값을 증가시킵니다.
    digitalWrite(m1stp, HIGH);//stp를 HIGH로 출력하고,
    digitalWrite(m2stp, HIGH);  
    delay(d);               //d밀리초 만큼 대기한뒤,
    digitalWrite(m1stp, LOW);//stp를 LOW로 출력합니다.
    digitalWrite(m2stp, LOW); 
    delay(d);//d밀리초만큼 대기합니다.

    if (a>2000)  //a가 2000보다 커지면,
    {
      a = 0;//초기화를 합니다.
      digitalWrite(m1dir, LOW);
      digitalWrite(m2dir, LOW);
    }
  }
}
