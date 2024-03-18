
#define F_CPU 16000000UL
#define pi 3.141592653589793238462643383279502884197169399375105820974944    // 파이 정의

#include <avr/io.h>                            /////////
#include <util/delay.h>                        ///헤////
#include <stdio.h>                             ///더////
#include <stdlib.h>                            ///파////
#include <avr/interrupt.h>                     ///일////
#include <string.h>                            ///선언//
#include <math.h>                              /////////




////////////////////////////////////
///// 함수 및 전역변수 선언구간 /////
//////////////////////////////////


double IIR_LPF_PM(double data);            //IIR filter 함수 선언_가변저항에 사용
double IIR_LPF_IR(double data);            //IIR filter 함수 선언_IR에 사용
double FIR_LPF(double data);               // FIR filter 함수 선언_CDS 에사용
double IIR_LPF(double data);               // IIR filter 함수 선언_Thermister에 사용
double Kalman_Filter(double data);            // Kalman filter 함수 선언_PSD 에사용
double Low_Pass_Filter(double data);         // LPF 함수 선언(저주파 통과 필터)_사용X
double Moving_Average_Filter(double data);      // MAF 함수 선언(이동 평균 필터)_사용X

void Uart_Init(void);                  // USART 0, 1번 관련 레지스터 설정 함수 선언
void Uart_Trans0(unsigned char data);      // USART 0번 데이터 송신함수 선언
unsigned char Uart_Receive(void);         // USART 1번 데이터 수신함수 선언
void Uart_Trans(unsigned char data);      // USART 1번 데이터 송신함수 선언
void Uart_TransNum(int num);            // USART로 5자리 정수를 송신하는 함수 선언
void Dynamixel_Packet_TX(unsigned char ID,int Pos,int Spd);   // Dynamixel을 구동시키기 위한 패킷통신 함수 선언



volatile int cnt_motion = 0;
volatile int adc[8];                  // 각 센서값들의 raw data를 저장하는 배열번수 선언
volatile int i=0;                     // index 변수 선언
volatile double LM35_ADC=0;               // LM35 raw data를 저장하는 변수 선언 
volatile double CDS_ADC=0;               // CDS raw data를 저장하난 변수 선언
volatile double PM_ADC=0;               // 가변저항 raw data를 저장하난 변수 선언
volatile double PSD_ADC=0;               // PSD raw data를 저장하난 변수 선언
volatile double THERMISTER_ADC=0;         // Thermister raw data를 저장하난 변수 선언
volatile double IR_ADC=0;               // IR raw data를 저장하난 변수 선언

volatile int PLOT_NUM=0;               // ADC 센서값 넘버링 변수
volatile double NF_ADC[6]={0,};            // None Filtering ADC값을 저장하는 배열변수 선언
volatile double F_ADC[6]={0,};            // Filtering을 한 ADC값을 저장하는 배열변수 선언
volatile double Digit_F_ADC[6]={0,};      // 수치화를 진행한 ADC값을 저장하는 배열변수 선언
   
// First Order RC LPF Parameter
volatile double dt=0.01;               // dt: 샘플링 주기
volatile double LPF_fc=5;               // 차단주파수: 5Hz
volatile double LPF_tau=1/(2 * 5 * pi);      // 자단 추파수에 대한 시정수(tau) 값 계산
volatile double LPF=0;                  // LPF 적용값을 저장하는 변수
volatile double LPF_past=0;               // 다음 샘플에 대한 LPF 계산을 위해 현 샘플에 대한 값을 과거값으로 별도 저장

// Moving Average Filter Parameter
volatile double MAF_sample[10]={0,};      // MAF 계산을 위해 센서값을 저장할 샘플 배열변수 선언: Sampling 갯수 10개
volatile int m=0;                     // MAF Sampling 센서값 index 변수 (최대 10번 counting)
volatile double MAF=0;                  // sampling 한 센서값들을 누산하여 저장하는 변수

// Kalman Filter Parameter
volatile double A_kal=1;      // X_k+1 = X_k + W_k
volatile double H_kal=1;      // Z_k+1 = X_k + V_k
volatile double Q_kal=0.05;   // system noise        //실제 시스템을 모르기 때문에 시스템 노이즈를 추가해야한다.
volatile double R_kal=2.8;    // measurement noise   //Q 증가, R 감소->실제 시스템과 같아짐, 시스템을 모를때엔 이 성질 이용하여 적절한 Q, R값찾기
volatile double x_est_kal=200;// 추정값,       초기 예측 거리 20.0cm
volatile double x_pred_kal=0; // 예측값
volatile double p_cal_kal=10; // 오차 공분산,   초기예측 공분산(초기값 정보 없을시에는 오차공분산을 크게 잡음)
volatile double p_pred_kal=0; // 오차 공분산 예측
volatile double k_gain_kal=0; // 칼만 게인

// IIR LPF Parameter filter order=2, Fc=5
volatile double b_iir[3]={0.0002414,0.0004827,0.0002414};             // iir b계수
volatile double a_iir[3]={1.0,-1.9555782,0.9565437};                  // iir a계수
volatile double IIR_input[3]={0,};                                    // 이전adc값 저장
volatile double IIR[3]={0,};                                          // 출력값 저장
   
// IIR LPF_PM Parameter filter order=2, Fc=8
volatile double b_iir_PM[3]={0.0006099,0.001219709437435,0.0006099};  // iir b계수
volatile double a_iir_PM[3]={1.0,-1.9289423,0.9313817};               // iir a계수
volatile double IIR_input_PM[3]={0,};                                 // 이전adc값 저장
volatile double IIR_PM[3]={0,};                                       // 출력값 저장

// IIR LPF_IR Parameter filter order=2, Fc=8
volatile double b_iir_IR[3]={0.0006099,0.001219709437435,0.0006099};  // iir b계수
volatile double a_iir_IR[3]={1.0,-1.9289423,0.9313817};               // iir a계수
volatile double IIR_input_IR[3]={0,};                                 // 이전adc값 저장
volatile double IIR_IR[3]={0,};                                       // 출력값 저장
   
// FIR LPF Parameter filter order=100, Fc=5
volatile double b_fir[101]={0.0010,0.0010,0.0011,0.0012,0.0013,0.0014,0.0016,0.0018,0.0020,0.0022,     //fir 계수
                     0.0025,0.0028,0.0031,0.0035,0.0039,0.0043,0.0048,0.0052,0.0057,0.0062,
                     0.0067,0.0073,0.0078,0.0084,0.0090,0.0096,0.0102,0.0108,0.0114,0.0120,
                     0.0126,0.0132,0.0138,0.0143,0.0149,0.0154,0.0159,0.0164,0.0169,0.0173,
                     0.0177,0.0181,0.0184,0.0187,0.0190,0.0192,0.0194,0.0196,0.0197,0.0197,
                     0.0197,0.0197,0.0197,0.0196,0.0194,0.0192,0.0190,0.0187,0.0184,0.0181,
                     0.0177,0.0173,0.0169,0.0164,0.0159,0.0154,0.0149,0.0143,0.0138,0.0132,
                     0.0126,0.0120,0.0114,0.0108,0.0102,0.0096,0.0090,0.0084,0.0078,0.0073,
                     0.0067,0.0062,0.0057,0.0052,0.0048,0.0043,0.0039,0.0035,0.0031,0.0028,
                     0.0025,0.0022,0.0020,0.0018,0.0016,0.0014,0.0013,0.0012,0.0011,0.0010,0.0010};
volatile double FIR_input[101]={0,};    // 이전adc값 저장
volatile double FIR=0;      // 출력값 저장


////Motion Parameter////
// POTENTIOMETER
// CSD
//THERMISTER
volatile int cnt_THER=0; 
volatile int flag_THER=0;
// PSD
volatile double SERVO_PSD_angle=60;
volatile int cnt_PSD=0; 
volatile int flag_PSD=0;

////수치화 상수////
// 써미스터 수치화를 위한 상수
volatile double AVCC = 5.0;      // 센서의 가동을 위해 입력되는 전압
volatile double R_0 = 1000;      // 25도씨 기준 저항
volatile double beta = 3650;   // 베타값
volatile double t_0 = 25;      // 시상수(초)
volatile double R10 = 4700;      // 10번 저항값
volatile double R_th = 0;      // 써미스터값 저장 변수

// CDS 수치화를 위한 상수
volatile double R_cds = 0;         // cds 저항
volatile double R9 = 4700;         // 9번 저항값
volatile double gamm = 0.8;         // 감마값
volatile double x = 0;            // cds 수치화를 위해 지수로서 곱해지는 값





/// 외부 인터럽트 정의 
ISR(INT0_vect)
{   // 외부 버튼을 눌러 PLOT_NUM을 카운트한다
   // 5가 되면 다시 0으로 초기화
   if (PLOT_NUM==5) PLOT_NUM=0;
   else PLOT_NUM++;
}
ISR(INT1_vect)
{

}

/// 타이머/카운터 0번 인터럽트 ///
ISR(TIMER0_OVF_vect)
{
   TCNT0=100; //10ms 주기
   PORTA = ~((~PORTA & 0b10111111)|(0b00000000));   // 스텝모터 여자X
   
      
   // 센서값 읽기
   for(i=0;i<8;i++)               // ADC는 총 8개이므로 8번을 돌린다
   {
      ADMUX=(0x40)|i;            // 각 i번째의 ADC 채널 접근
      ADCSRA |=(1<<ADSC);         // 해당 채널에서 ADC변환 시작
      while(!(ADCSRA&(1<<ADIF)));   // ADC변환이 완료되면 다음 줄로
      adc[i]=ADC;               // i 번째 채널에서 변환된 센서값을 adc 배열의 i 번째에 저장
   }
   
   // adc 배열변수에 받은 값들을 해당 센서값을 저장하는 변수에 저장
   PM_ADC = adc[0];            // adc 0번: 가변저항
   CDS_ADC = adc[1];         // adc 1번: CDS
   LM35_ADC = adc[2];         // adc 2번: LM35
   THERMISTER_ADC = adc[3];      // adc 3번: 써미스터`
   PSD_ADC = adc[4];         // adc 4번: PSD
   IR_ADC = adc[5];            // adc 5번: IR
   
   // USART로 원활한 주파수 비교 및 확인을 위해 필터링 이전 값들을 따로 배열에 저장
   NF_ADC[0] = PM_ADC;         // adc 0번: 가변저항
   NF_ADC[1] = CDS_ADC;         // adc 1번: CDS
   NF_ADC[2] = LM35_ADC;      // adc 2번: LM35
   NF_ADC[3] = THERMISTER_ADC;   // adc 3번: 써미스터`
   NF_ADC[4] = PSD_ADC;         // adc 4번: PSD
   NF_ADC[5] = IR_ADC;         // adc 5번: IR
   
   // 필터링을 마친 센서값을 저장
   F_ADC[0] = IIR_LPF_PM(PM_ADC);          // 가변저항 - IIR 필터 적용
   F_ADC[1] = FIR_LPF(CDS_ADC);            // CDS - FIR 필터 적용
   F_ADC[2] = LM35_ADC;                    // LM35 - IIR 필터 적용
   F_ADC[3] = IIR_LPF(THERMISTER_ADC);     // 서미스터 - IIR 필터 적용
   F_ADC[4] = Kalman_Filter(PSD_ADC);      // PSD - 칼만필터 적용
   F_ADC[5] = IIR_LPF_IR(IR_ADC);          // IR - IIR 필터 적용
   
   
   ///////// 
   //수치화//
   /////////
   // CDS 수치화를 위한 파라미터 계산
   R_cds = (((R9*AVCC*1023)/(F_ADC[1] * 5)) - R9);
   x = (1 - ((log10(R_cds) - log10(40000)) / gamm));
   // Thermister 수치화를 위한 파라미터 계산
   R_th = (double)((AVCC * 1023 * R10 / (F_ADC[3] * 5 ))  - R10);
   
   // 최종 계산된 수치화 값 저장
   Digit_F_ADC[0] = (double)((F_ADC[0]*100)/1023);    // 가변저항 수치화  0~1023 -> 0~100 
   Digit_F_ADC[1] = pow(10, x);
   Digit_F_ADC[2] = (double)(F_ADC[2]*5.0*100)/(1023*2);
   Digit_F_ADC[3] = (double)(1 / ((1 / (t_0 + 273.15) + ((1 / beta)*(log(R_th)-log(R_0))))) - 273.15);
   Digit_F_ADC[4] = (double)(47104/F_ADC[4]);   // PSD 수치화 (mm단위 까지 보기위해 *10)
   Digit_F_ADC[5] = (F_ADC[5]-0.0)/(1023.0-0.0) * 307;  
						 // 다이나믹셀을 0~90도까지 작동시키기 위해 90도 x 3.41 = 307로 곱해준다.
   
   PORTA = ~((~PORTA & 0b11000000)|(1<<PLOT_NUM)); // 6번 7번 LED유지 0~5번 변경
   // 현재 명 번 ADC를 USART로 출력하고 있는지 그 번호의 LED만 켠다
   
   /*
   Uart_TransNum(NF_ADC[PLOT_NUM]);
   Uart_Trans(0x2C);   // 시리얼 차트 상에서 구분을 위해 콤마를 중간에 출력
   Uart_TransNum(F_ADC[PLOT_NUM]);
   Uart_Trans(0x0d);   // 시리얼 차트 상에서 마지막을 알리기 위한 출력
   Uart_Trans(13);      // 줄 바꿈
   */
   Uart_TransNum(NF_ADC[PLOT_NUM]);
   Uart_Trans(0x20);
   Uart_TransNum(F_ADC[PLOT_NUM]);
   Uart_Trans(0x0d);   // 시리얼 차트 상에서 마지막을 알리기 위한 출력
   
   ////////////////////
   //센서 값에따른 작동//
   ///////////////////
   if (cnt_motion>300) //초기에 센서값 튈수있으므로 초기 3초간 작동 정지
   {
   // adc0-POTENTIOMETER(가변저항)에 따른 모션_dc PWM(팬)
   if (Digit_F_ADC[0] < Digit_F_ADC[2]) OCR3A = 50*(Digit_F_ADC[2]-Digit_F_ADC[0]);  
									// 설정온도와 LM35와 값비교하여 차이만큼 팬속도 조절
   else OCR3A = 0;      //  가변저항 값이 크면 팬을 돌리지 않는다
   
   // adc1-CDS에따른 모션_LED밝기
   if (Digit_F_ADC[1] < 90) OCR1B = (abs(100-Digit_F_ADC[1]))*40; // 어두워질수록 밝아짐
   else OCR1B = 0;	// 실외가 밝으면 불을 끈다
      
   // adc2-LM35에 따른 모션_3색LED
   if (Digit_F_ADC[2] > 65) PORTC = ~((~PORTC & 0b00011111)|(0b10000000));         //65도 이상일때 빨간색
   else if (Digit_F_ADC[2] > 60) PORTC = ~((~PORTC & 0b00011111)|(0b11000000));    //60도 이상일때 노란색
   else if (Digit_F_ADC[2] > 55) PORTC = ~((~PORTC & 0b00011111)|(0b01000000));    //55도 이상일때 초록색
   else if (Digit_F_ADC[2] > 50) PORTC = ~((~PORTC & 0b00011111)|(0b01100000));    //50도 이상일때 하늘색
   else if (Digit_F_ADC[2] > 45) PORTC = ~((~PORTC & 0b00011111)|(0b00100000));    //45도 이상일때 파란색
   else PORTC = ~((~PORTC & 0b00011111)|(0b11100000));                             //평소에 흰색 
   
   // adc3-THERMISTER에따른 모션_stepmotor
   if ((Digit_F_ADC[3] > (Digit_F_ADC[2]+30))) // LM35보다 30도 높으면 스텝모터 이용하여 원운동 선운동으로 바꾸어 차단벽 닫음
   {
      cnt_THER++;
      
      if (flag_THER==0 && cnt_THER<300)                  // 문 안닫혀있으면 3초동안 스텝모터 구동하여 개폐
      {
         PORTC = ~((~PORTC & 0b11110011)|(0b00000100));  // 스텝모터 구동방향설정(문여는방향)
         PORTA = ~((~PORTA & 0b01111111)|(0b10000000));  // LM35보다 30도 높으면 LED ON
         PORTA = ~((~PORTA & 0b10111111)|(0b01000000));  // 10ms마다 스텝모터를 여자시켜 스텝모터 구동
      }
      else
      {
         PORTC = ~((~PORTC & 0b11110011)|(0b00001100));  // 스텝모터 구동방향설정(구동X)
         PORTA = ~((~PORTA & 0b10111111)|(0b00000000));  // 스텝모터 여자X
         flag_THER = 1; 
         cnt_THER = 0;
      }
      
   }
   else
   {
      cnt_THER++;
      if (flag_THER==1 && cnt_THER<300)
      {
         PORTC = ~((~PORTC & 0b11110011)|(0b00001000)); // 스텝모터 구동방향설정(문닫는방향)
         PORTA = ~((~PORTA & 0b10111111)|(0b01000000)); // LM35보다 30도 낮으면 LED off
         PORTA = ~((~PORTA & 0b01111111)|(0b00000000)); // 스텝모터 여자X
      }
      else
      {
         PORTC = ~((~PORTC & 0b11110011)|(0b00001100)); // 스텝모터 구동방향설정(구동X)
         PORTA = ~((~PORTA & 0b10111111)|(0b00000000)); // 스텝모터 여자X
    
         flag_THER = 0;
         cnt_THER = 0;
         
      }
   }

   
   // adc4-PSD에 따른 모션_서보모터
   if (Digit_F_ADC[4] < 100 || flag_PSD == 1)  // 물체 인식하면 서보모터 이용하여 출입문 염 
   {  
      flag_PSD = 1;
      SERVO_PSD_angle = 60;
      cnt_PSD++;
      if (cnt_PSD>=150)                        // 열고 약 1.5초동안 열린상태 유지, 1.5초뒤에 물체있으면 열린상태유지 아니면 닫음
      {
         flag_PSD = 0;
         cnt_PSD = 0;
      }
   }
   else if(flag_PSD == 0) SERVO_PSD_angle = 120;
   OCR1A = ((45 + SERVO_PSD_angle) / 1800 ) * ICR1;
   
   }
   
   
   cnt_motion++;
   
   
    
   // adc5-IR에 따른 모션_다이나믹셀
   Dynamixel_Packet_TX(1, (int)Digit_F_ADC[5], 0);   // (ID, 이동할 각도 위치, 속도 모드)
      // IR값을 307까지의 값으로 변환하여 그 값을 다이나믹셀 이동 위치값 파라미터로 바로 넘겨준다
   

   ///////////////////////////////여기까지 센서값에따른 작동

}




/////////////////////////////////
///// 메        인        문 /////
/////////////////////////////////

int main(void)
{
   DDRA=0xff;      // LED핀 설정 
   DDRB=0xff;      // Timer/Counter 설정
   DDRC=0b11101111;      // 모터방향,3색LED출력,부져(부저 초기값은 OFF)
   DDRD=0b11101000;  // UART 1번 TX는 출력 포트, RX는 입력 포트(센서값 전용), 0,1번 외부 인터럽트
   DDRE=0xFE;       // UART 0번 TX는 출력 포트, RX는 입력 포트(Dynamixel 전용)
   DDRF=0x00;        // ADC핀 입력 설정
   
   PORTA = 0xff;   // PORT A 입출력 설정
   PORTC = 0xEF;   // PORT C 입출력 설정
   

   TCCR0=(0<<WGM01)|(0<<WGM00)|(0<<COM01)|(0<<COM00)|(1<<CS02)|(1<<CS01)|(1<<CS00);   // normal mode, 1024 prescaler
   TIMSK=(1<<TOIE0);   // Timer/Counter overflow interrupt 활성화
   TCNT0=100;
   
   EICRA = (1<<ISC01)|(1<<ISC11);   // falling edge interrupt
   EIMSK = (1<<INT1)|(1<<INT0);      // 0, 1번 스위치 interrupt 활성화
   
   // 16bit Timer1 Counter 레지스터 설정
   TCCR1A = (1<<COM1A1)|(0<<COM1A0)|(1<<COM1B1)|(0<<COM1B0)|(1<<WGM11)|(0<<WGM10); // Fast PWM Mode
   TCCR1B = (1<<WGM13)|(1<<WGM12)|(0<<CS12)|(1<<CS11)|(1<<CS10); // 64 prescaler
   ICR1 = 4999;   // Timer/Counter 1의 OCR값 설정: // (16MHz / 분주비 64) * 20ms = 5000
   // 16bit Timer3 Counter 레지스터 설정
   TCCR3A = (1<<COM3A1)|(0<<COM3A0)|(1<<COM3C1)|(0<<COM3C0)|(1<<WGM31)|(0<<WGM30); // Fast PWM Mode
   TCCR3B = (1<<WGM33)|(1<<WGM32)|(0<<CS32)|(1<<CS31)|(1<<CS30); // 64 prescaler
   ICR3 = 4999;   // Timer/Counter 3의 OCR값 설정: // (16MHz / 분주비 64) * 20ms = 5000

   ADMUX=0x40;      // 외부 AVCC 단자로 입력전압 사용, 오른쪽 정렬
   ADCSRA=0x87;      // ADC ON, ADC prescaler 128
   
   sei();         // 전역 인터럽트 활성화
   
   Uart_Init();      //uart 초기 설정
         
   while(1);
   return 0;
}



/////////////////////////////
///// 필터 함수 정의구간 /////
////////////////////////////

double FIR_LPF(double data)
{
   double fir_save[100]={0,}; // 이전adc값 저장에 사용위한 배열
   int j=0;
   int k=0;
   int l=0;
   
   FIR_input[0] = data;  // 현재 adc 값 저장 
   FIR = 0;
   
   for(j=0;j<101;j++) FIR += b_fir[j]*FIR_input[j];   // FIR계수곱하여 더해주기

   for(k=0;k<100;k++) fir_save[k] = FIR_input[k];     //이전adc값 저장//
   for(l=0;l<100;l++) FIR_input[l+1] = fir_save[l];   /////////////////

   return FIR;
}

double IIR_LPF(double data)
{
   double iir_save=0;  // 이전adc값 저장
   int j=0;
   int k=0;
   IIR_input[0] = data;
   IIR[0] = 0;
   
   for(j=0;j<3;j++) IIR[0] += b_iir[j]*IIR_input[j];  // IIR 이전 ADC값에 b계수 곱하여 더해주기
   for(k=1;k<3;k++) IIR[0] -= a_iir[k]*IIR[k];        // IIR 이전 출력값에 a계수 곱하여 빼주기
    
   iir_save=IIR[1];                /////////////////////////
   IIR[1]=IIR[0];                  /////////////////////////
   IIR[2]=iir_save;                //이전adc값과 출력값 저장//
   iir_save=IIR_input[1];          /////////////////////////
   IIR_input[1]=IIR_input[0];      /////////////////////////
   IIR_input[2]=iir_save;          /////////////////////////
   
   return IIR[0];
}

double IIR_LPF_PM(double data)
{
   double iir_save_PM=0; // 이전adc값 저장
   int j=0;
   int k=0;
   IIR_input_PM[0] = data;
   IIR_PM[0] = 0;
   
   for(j=0;j<3;j++) IIR_PM[0] += b_iir_PM[j]*IIR_input_PM[j]; // IIR 이전 ADC값에 b계수 곱하여 더해주기
   for(k=1;k<3;k++) IIR_PM[0] -= a_iir_PM[k]*IIR_PM[k];       // IIR 이전 출력값에 a계수 곱하여 빼주기
     
   iir_save_PM=IIR_PM[1];                    /////////////////////////
   IIR_PM[1]=IIR_PM[0];                      /////////////////////////
   IIR_PM[2]=iir_save_PM;                    //이전adc값과 출력값 저장//
   iir_save_PM=IIR_input_PM[1];              /////////////////////////
   IIR_input_PM[1]=IIR_input_PM[0];          /////////////////////////
   IIR_input_PM[2]=iir_save_PM;              /////////////////////////
   
   return IIR_PM[0];
}

double IIR_LPF_IR(double data)
{
   double iir_save_IR=0;  // 이전adc값 저장
   int j=0;
   int k=0;
   IIR_input_IR[0] = data;
   IIR_IR[0] = 0;
   
   for(j=0;j<3;j++) IIR_IR[0] += b_iir_IR[j]*IIR_input_IR[j]; // IIR 이전 ADC값에 b계수 곱하여 더해주기
   for(k=1;k<3;k++) IIR_IR[0] -= a_iir_IR[k]*IIR_IR[k];       // IIR 이전 출력값에 a계수 곱하여 빼주기
   
   iir_save_IR=IIR_IR[1];                 /////////////////////////
   IIR_IR[1]=IIR_IR[0];                   /////////////////////////
   IIR_IR[2]=iir_save_IR;                 /////////////////////////
   iir_save_IR=IIR_input_IR[1];           //이전adc값과 출력값 저장//
   IIR_input_IR[1]=IIR_input_IR[0];       /////////////////////////
   IIR_input_IR[2]=iir_save_IR;           /////////////////////////
   
   return IIR_IR[0];
}

double Kalman_Filter(double data)
{
   x_pred_kal = A_kal*x_est_kal;                                            // 예측값
   p_pred_kal = A_kal*p_cal_kal*A_kal + Q_kal;                              // 오차공분산 예측
   
   k_gain_kal = (double)(p_pred_kal*H_kal/(H_kal*p_pred_kal*H_kal+R_kal));  // 칼만게인 계산
   
   x_est_kal = x_pred_kal + (k_gain_kal*(data-H_kal*x_pred_kal));           // 추정값
   p_cal_kal = p_pred_kal - (k_gain_kal*H_kal*p_pred_kal);                  // 오차 공분산
   
   return x_est_kal;                                                        // 추정값 반환
}

// LPF 함수 
double Low_Pass_Filter(double data)
{
   LPF = (dt*data+LPF_tau*LPF_past)/(LPF_tau+dt);   // LPF 적용
   LPF_past = LPF;      // 이전 샘플링의 LPF 적용값을 저장
   
   return LPF;         // 최종 LPF 적용값 반환
}

// MAF 함수
double Moving_Average_Filter(double data)
{
   MAF = 0;                  // 표본들을 누산하는 변수 초기화
   MAF_sample[m] = data;      // Sampling 값을 변수에 순서대로 저장
   int j=0;
   for(j=0;j<10;j++)
   {
      MAF += MAF_sample[j];      // MAF_sample 배열변수에 저장된 Sample을 누산
   }
   if (m==9) m=0;            // 이 작업을 10번 반복하면 다시 m=0으로 초기화
   else m++;               //  왜?: sampling 갯수를 10개로 잡았기 때문에
   
   return (MAF/10);            // 최종 MAF 적용값 반환
}




/////////////////////////////
///// 기타 함수 정의구간 /////
////////////////////////////

// USART 관련 레지스터 설정 함수
void Uart_Init(void){
   // USART 0번 레지스터 설정: Dynamixel 구동 전용 USART
   UCSR0A = 0x00;   // 비동기 모드, 1배속 통신모드
   UCSR0B = (1<<RXEN0) | (1<<TXEN0);   //uart 송수신 기능 허용
   UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);   //비동기, non-parity mode, stop pit: 1bit, 8bit data
   
   UBRR0H = 0;
   UBRR0L = 103;      //속도 9600bps
   
   
   // USART 1번 레지스터 설정: 센서값 확인 전용 USART
   UCSR1A = 0x00;   // 비동기 모드, 1배속 통신모드
   UCSR1B = (1<<RXEN1) | (1<<TXEN1);   //uart 송수신 기능 허용
   UCSR1C = (1<<UCSZ11) | (1<<UCSZ10);   //비동기, non-parity mode, stop pit: 1bit, 8bit data
   
   UBRR1H = 0;
   UBRR1L = 103;      //속도 9600bps
}

// USART0 데이터 송신 함수
void Uart_Trans0(unsigned char data){
   while(!(UCSR0A & (1<<UDRE0)));
   UDR0 = data;//들어온 데이터를 UDR에 넣어줌
}

// USART1 데이터 송신 함수
void Uart_Trans(unsigned char data){
   while(!(UCSR1A & (1<<UDRE1)));
   UDR1 = data;//들어온 데이터를 UDR에 넣어줌
}

// USART1 데이터 수신 함수
unsigned char Uart_Receive(void){
   while(!(UCSR1A & (1<<RXC1)));
   return UDR1;//받은 데이터를 리턴
}

// 5자리 정수를 UART 통신으로 송신하는 함수
void Uart_TransNum(int num)
{
   if(num < 0)   // 입력한 숫자가 음수면
   {
      Uart_Trans('-');   // (-)를 먼저 출력
      num = -num;   // 양수로 바꿔준다
   }
   Uart_Trans((num/10000) + 48);         // 만의 자리 출력
   Uart_Trans(((num%10000)/1000) + 48);      // 쳔의 자리 출력
   Uart_Trans(((num%1000)/100) + 48);      // 백의 자리 출력
   Uart_Trans(((num%100)/10) + 48);         // 십의 자리 출력
   Uart_Trans(((num%10)/1) + 48);         // 일의 자리 출력
   //Uart_Trans(13);
}


// Dynamixel을 구동시키기 위한 패킷통신 함수
void Dynamixel_Packet_TX(unsigned char ID,int Pos,int Spd)
{

   unsigned char Pos_H,Pos_L,Spd_H,Spd_L;

   //if Pos = 300 = 0b00000001 00101100
   //Pos_L = 0b00101100
   //Pos >> 8 = 0b00000000 00000001 = Pos_H
   Pos_L = Pos;
   Pos_H = (Pos>>8);
   Spd_L = Spd;
   Spd_H = (Spd>>8);

   Uart_Trans0(0xFF);  //ini
   Uart_Trans0(0xFF);  //ini, Packet의 시작을 알림
   Uart_Trans0(ID);  //ID, Broadcase = 0xFE
   Uart_Trans0(0x07);  // Packet의 길이: 7
   Uart_Trans0(0x03);  //Dynamixel에 내릴 명령어(숫자) 입력: Instruction WRITE = 3
   Uart_Trans0(0x1E);  //모터 회전
   Uart_Trans0(Pos_L);  //각도L bit
   Uart_Trans0(Pos_H);  //각도H bit
   Uart_Trans0(Spd_L);  //속도L bit
   Uart_Trans0(Spd_H);  //속도L bit
   Uart_Trans0((unsigned char)(~(ID + 0x07 + 0x03 + 0x1E + Pos_L + Pos_H + Spd_L + Spd_H))); 
            // CheckSum 데이터 검사
}




