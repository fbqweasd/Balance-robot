
 
void PID_UKC_init(int Kp,int Ki,int Kd,int * ptr){
     AcY = ptr;
     
     data_l = AcY; //���߿� �����ؿ�
     data = AcY;
  }

  void data_init(){
    data = AcY;
    
    if(data < 0)
      data = -data;
 }

 
  int PID_Math_P(){ //��ʽ��� ���ϴ� �Լ�
    int P;
    P = Kp * data;
    return P;
  }

  int PID_Math_I(){ //������ ���ϴ� �Լ�
    int I;
    I = Ki * data * dt;
    return I;
  }

  int PID_Math_D(){ //�̺��� ���ϴ� �Լ�
    int D;
    D = Kd * ( data - data_l / dt );
    return D;
  }

  int PID_Math(){ //PID�� ��� �ϴ� �Լ�
    int P,I,D;

    data_init();

    P = PID_Math_P();
    I = PID_Math_I();
    D = PID_Math_D();

    data_l = data;
    
    return P + I + D;
  }
