
/*
ESP32 S2 mini 
Board Mngr : esp32 s2 로 검색할 것 (esp32 설치하면 0번 USER INT PIN not working)
*/

#include <Arduino.h>
#include <stddef.h>
#include "rtwtypes.h"
#include <driver/adc.h>
#include "simulink_param.h"

#define VOLUMECOUNT 10  // 튜닝시에 적용되는 볼륨단계 0,1
#define DATACOUNT 28    // 레지스터 개수
#define BAUDRATE 115200


#define USER_INT_PIN 0  // ESP32 S2 mini 보드의 버튼을 REUSE 함.

//------------------------------------------------------
#define LED_PIN 15
#define TIMER_DURATION_US 120  // DAC의 출력주기임.  100us --> 10kHz로 DAC 보냄 x
hw_timer_t* timer = NULL;
// portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool ledState = false, bTimerState = true; 
void IRAM_ATTR ReadAndProcessIRQ();
//------------------------------------------------------

static int volume = 0;
// static float data[VOLUMECOUNT][DATACOUNT] = {
static int data[VOLUMECOUNT][DATACOUNT] = {

  { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8 },
  { 2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 1, 2, 3, 4, 5, 6, 7, 8 }
};


// const float MEMdata[] PROGMEM = {
const int MEMdata[] PROGMEM = {
  1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  2, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  3, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  4, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  5, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  6, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  7, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  9, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28,
  10, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28
};

const int AACPin2 = 18;  // ESP32-S2 mini DAC2
const int ANCPin = 17;   // ESP32-S2 mini DAC1

int myselect;
boolean bDataOut = true;


/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;


void ParseInputTo2DArray();
void UImenu();
void SetVolumeData();
void GetVolumeData();
void printAllVolumeData();
void SetAppParameters(int volume);
void SetAppParametersFromMEM(int volume);
void printAppParameters(int volume);
void printAppParametersMEM(int volume);
void UserInput();
void ReadAndProcessIRQ();

unsigned long AXC_step();
unsigned long delay_10us();

// 처리시간 측정용
unsigned long processing_times[100];  // 처리 시간을 저장할 배열
int myindex = 0;                      // 배열 인덱스


// ********************************************************************** //
void setup() {

  Serial.begin(BAUDRATE);  // 시리얼 통신 속도 설정

  pinMode(USER_INT_PIN, INPUT_PULLUP);  // 사용자 인터럽트 버튼
  pinMode(LED_PIN, OUTPUT);

  // ADC 초기화
  adc1_config_width(ADC_WIDTH_BIT_13);
  adc1_config_channel_atten(ADC1_CHANNEL_2, ADC_ATTEN_DB_11);  // ADC1 채널 2 설정 (3번핀)
  adc1_config_channel_atten(ADC1_CHANNEL_4, ADC_ATTEN_DB_11);  // ADC1 채널 4 설정 (5번핀)

  Serial.println("Serial Port Connected ...");
  Serial.println("Ver: 2.0.0  230726");
  Serial.println("--note------------------");
  Serial.println("MCU : S2 mini ");
  Serial.println(" ");
  Serial.println("--Pin map------------------------");
  Serial.println("MIC1 : 3");
  Serial.println("MIC2 : 5");
  Serial.println("ANC  : 17");
  Serial.println("AAC2 : 18");
  Serial.println("STOP : 0");

  UImenu();

  // ESP32 인터럽트 타이머 설정 ----------------------------------------------------------
  // Use 0th timer of 4 (counted from zero).
  // Set 80 divider for prescaler (see ESP32-S2 Technical Reference Manual for more info).
  timer = timerBegin(0, 80, true);

  // Attach IRQ function to the timer interrupt
  timerAttachInterrupt(timer, &ReadAndProcessIRQ, true);

  // Set alarm to call IRQ function every millisecond (value in microseconds)
  timerAlarmWrite(timer, TIMER_DURATION_US, true);

  // Start the timer alarm
  timerAlarmEnable(timer);
  //----------------------------------------------------------------------------------------
}
// ********************************************************************** //






// ********************************************************************** //
void loop() {

  // 사용자 인터럽트 버튼 체크
  if (digitalRead(USER_INT_PIN) == LOW) {  // 사용자 인터럽트
    bTimerState = false;
    timerAlarmDisable(timer);

    delay(1000);
    UImenu();
  }

  // Built-in LED
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);

  // dacWrite(AACPin2, rtY.AAC2);
  // dacWrite(ANCPin, rtY.ANC);

  delay(1000);
}






//--------------------------------------------------------
void ReadAndProcessIRQ() {

  // 마이크 신호 읽기
  rtU.Mic2_1 = adc1_get_raw(ADC1_CHANNEL_2);  // ADC 값을 읽음
  rtU.Mic2_2 = adc1_get_raw(ADC1_CHANNEL_4);  // ADC 값을 읽음
  // SignalProcessing
  AXC_step();  // loop 안으로 변경

  // DAC 출력
  // Generate random value between 0 and 255
  // uint8_t randomValue = random(10);
  // dacWrite(AACPin2, randomValue);
  // dacWrite(ANCPin, randomValue);

  dacWrite(AACPin2, rtY.AAC2);
  dacWrite(ANCPin, rtY.ANC);

  // // LED
  // ledState = !ledState;
  // digitalWrite(LED_PIN, ledState);
}

//--------------------------------------------------------



//--------------------------------------------------------
void UImenu() {
  Serial.println("---------------UI V0.7 (23.07.26) -----------------");
  Serial.println("[1] Set Params(10 x 28)");
  Serial.println("[2] Calibration(TBD)");
  Serial.println("[3] Monitoring (defaut: No printing)(V2,A2,Mic2,AAC2,ANC");
  Serial.println("[4] Start");
  Serial.println("-----------------------------------------");
  Serial.println("");
  UserInput();
}
//--------------------------------------------------------


//--------------------------------------------------------
void ParseInputTo2DArray() {

  int MaxDataCount = DATACOUNT;

  if (Serial.available() > 0) {
    for (byte i = 0; i < MaxDataCount * 100; i++) {
      if (i == 0) {
        volume = Serial.parseInt();
        Serial.print("volume: ");
        Serial.println(volume);
        Serial.print("Params: ");

      } else if (Serial.peek() == -1) {
        break;
      } else {
        // data[volume][i - 1] = Serial.parseInt(); //0428

        Serial.print(data[volume][i - 1]);
        Serial.print(" ");
      }
    }
    Serial.println("");
  }
}
//----------------------------------------------------------











//--------------------------------------------------------
void UserInput() {
  // timerStop(timer);

  Serial.println("select Menu");
  // 사용자 입력
  while (Serial.available() == 0) {
  }
  myselect = Serial.parseInt();
  Serial.println(myselect);
  Serial.println("");


  switch (myselect) {


    //-------------------------
    case 1: /* 파라미터 설정 */
      myselect = 0;
      Serial.println("myselect.");
      Serial.println("1)Input Params.     2)Print Params.     3) Print all Params.");

      while (Serial.available() == 0) {
      }
      myselect = Serial.parseInt();
      Serial.println(myselect);

      if (myselect == 1) {
        SetVolumeData();
      } else if (myselect == 2) {
        GetVolumeData();
      } else if (myselect == 3) {
        printAllVolumeData();
      } else {
        Serial.println("wrong input....");
      }


      UImenu();
      break;
    //---------------------------

    //---------------------------
    case 2: /* Calibration */

      Serial.println("Calibration TBD.");

      UImenu();
      break;
    //---------------------------

    //---------------------------
    case 3: /* 시리얼모니터 출력 */

      bDataOut = !bDataOut;

      if (bDataOut) {
        Serial.println("print Serial data.");
      } else {
        Serial.println("not print Serial data.");
      }

      UImenu();
      break;
    //---------------------------

    //---------------------------
    case 4: /* 시작 종료 */

      Serial.print("Input VOLUME for application: ");
      while (Serial.available() == 0) {
      }
      myselect = Serial.parseInt();
      Serial.println(myselect);
      SetAppParameters(myselect);
      // SetAppParametersFromMEM(myselect);   // ROM 메모리로부터 파라미터를 입력받을 경우

      Serial.println("28 params applied.");

      Serial.println("check params ------------");
      printAppParameters(myselect);
      // printAppParametersMEM(myselect);    // ROM 메모리로부터 파라미터를 입력받을 경우

      Serial.println(" ");
      Serial.println("Input 1 to start !!! ");
      // Serial.print("timer상태는"); Serial.println(bTimerState);
      while (Serial.available() == 0) {}
      myselect = Serial.parseInt();

      if (myselect == 1) {
        if (bTimerState == false) {
          timerAlarmEnable(timer);
          bTimerState == true;
        }
      Serial.println("");Serial.println("");Serial.println("");  
      Serial.println(".............................. ");  
      Serial.println("now Working !................... ");   
      Serial.println("Check Blue LED blinking on Main Board.. ");
      Serial.println("Push [Reset Button] for tuning.. ");
      Serial.println("");Serial.println("");Serial.println("");               
      break;
      }

    default:  // CASE 해당사항 없으면 수행
      UImenu();
      break;
      //---------------------------

  }  // switch
}  // textUI
//--------------------------------------------------------





// -----------------------------------------------------
void SetVolumeData() {

  int MaxDataCount = DATACOUNT;
  Serial.println("first one is VOLUME, the others (28 ea) are INTEGER params.(* Insert SPACE between data )");
  Serial.println("ex) 1 2 33 44 400 ...");

  // wait for data input
  while (Serial.available() == 0) {
  }



  if (Serial.available() > 0) {

    for (byte i = 0; i < MaxDataCount * 100; i++) {
      if (i == 0) {
        volume = Serial.parseInt();
        Serial.print("volume: ");
        Serial.println(volume);
        Serial.print("Params: ");

      } else if (Serial.peek() == -1) {
        break;
      } else {

        data[volume][i - 1] = Serial.parseInt();  //0619
        // data[volume][i - 1] = Serial.parseFloat();  //0508

        Serial.print(data[volume][i - 1]);
        Serial.print(" ");
      }
    }
    Serial.println("");
  }
}
// -----------------------------------------------------


// -----------------------------------------------------
void GetVolumeData() {
  Serial.println("Volume: ");
  // wait for data input
  while (Serial.available() == 0) {
  }
  volume = Serial.parseInt();

  for (int i = 0; i < VOLUMECOUNT; i++) {
    Serial.print(data[volume][i]);
    Serial.print(" ");
  }
  Serial.println(" ");
}
// -----------------------------------------------------

// -----------------------------------------------------
void SetAppParameters(int volume) {
  rtU.Gain_Velocity2 = data[volume][0];
  rtU.Gain_Pressure2 = data[volume][1];
  rtU.Gain_BPF1 = data[volume][2];
  rtU.Gain_BPF2 = data[volume][3];
  rtU.a0_Velocity2 = data[volume][4];
  rtU.ma1_Velocity2 = data[volume][5];
  rtU.ma2_Velocity2 = data[volume][6];
  rtU.b0_Velocity2 = data[volume][7];
  rtU.b1_Velocity2 = data[volume][8];
  rtU.b2_Velocity2 = data[volume][9];
  rtU.a0_Voltage2 = data[volume][10];
  rtU.ma1_Voltage2 = data[volume][11];
  rtU.ma2_Voltage2 = data[volume][12];
  rtU.b0_Voltage2 = data[volume][13];
  rtU.b1_Voltage2 = data[volume][14];
  rtU.b2_Voltage2 = data[volume][15];
  rtU.a0_MHHC1 = data[volume][16];
  rtU.ma1_MHHC1 = data[volume][17];
  rtU.ma2_MHHC1 = data[volume][18];
  rtU.b0_MHHC1 = data[volume][19];
  rtU.b1_MHHC1 = data[volume][20];
  rtU.b2_MHHC1 = data[volume][21];
  rtU.a0_MHHC2 = data[volume][22];
  rtU.ma1_MHHC2 = data[volume][23];
  rtU.ma2_MHHC2 = data[volume][24];
  rtU.b0_MHHC2 = data[volume][25];
  rtU.b1_MHHC2 = data[volume][26];
  rtU.b2_MHHC2 = data[volume][27];
}
// -----------------------------------------------------

// -----------------------------------------------------
void SetAppParametersFromMEM(int volume) {

  // https://onlinedocs.microchip.com/oxy/GUID-BD1C16C8-7FA3-4D73-A4BE-241EE05EF592-en-US-5/GUID-90FCF448-97F2-404D-ABF3-8B5A4AFDACBD.html
  // pgm_read_word_near 정수
  // pgm_read_word_near float경우
  rtU.Gain_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 0);
  rtU.Gain_Pressure2 = pgm_read_word_near(volume * 28 + MEMdata + 1);
  rtU.Gain_BPF1 = pgm_read_word_near(volume * 28 + MEMdata + 2);
  rtU.Gain_BPF2 = pgm_read_word_near(volume * 28 + MEMdata + 3);
  rtU.a0_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 4);
  rtU.ma1_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 5);
  rtU.ma2_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 6);
  rtU.b0_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 7);
  rtU.b1_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 8);
  rtU.b2_Velocity2 = pgm_read_word_near(volume * 28 + MEMdata + 9);
  rtU.a0_Voltage2 = pgm_read_word_near(volume * 28 + MEMdata + 10);
  rtU.ma1_Voltage2 = pgm_read_word_near(volume * 28 + MEMdata + 11);
  rtU.ma2_Voltage2 = pgm_read_word_near(volume * 28 + MEMdata + 12);
  rtU.b0_Voltage2 = pgm_read_word_near(volume * 28 + MEMdata + 13);
  rtU.b1_Voltage2 = pgm_read_word_near(volume * 28 + MEMdata + 14);
  rtU.b2_Voltage2 = pgm_read_word_near(volume * 28 + MEMdata + 15);
  rtU.a0_MHHC1 = pgm_read_word_near(volume * 28 + MEMdata + 16);
  rtU.ma1_MHHC1 = pgm_read_word_near(volume * 28 + MEMdata + 17);
  rtU.ma2_MHHC1 = pgm_read_word_near(volume * 28 + MEMdata + 18);
  rtU.b0_MHHC1 = pgm_read_word_near(volume * 28 + MEMdata + 19);
  rtU.b1_MHHC1 = pgm_read_word_near(volume * 28 + MEMdata + 20);
  rtU.b2_MHHC1 = pgm_read_word_near(volume * 28 + MEMdata + 21);
  rtU.a0_MHHC2 = pgm_read_word_near(volume * 28 + MEMdata + 22);
  rtU.ma1_MHHC2 = pgm_read_word_near(volume * 28 + MEMdata + 23);
  rtU.ma2_MHHC2 = pgm_read_word_near(volume * 28 + MEMdata + 24);
  rtU.b0_MHHC2 = pgm_read_word_near(volume * 28 + MEMdata + 25);
  rtU.b1_MHHC2 = pgm_read_word_near(volume * 28 + MEMdata + 26);
  rtU.b2_MHHC2 = pgm_read_word_near(volume * 28 + MEMdata + 27);
}
// -----------------------------------------------------

void printAppParameters(int volume) {
  Serial.print(rtU.Gain_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][0]);
  Serial.print(rtU.Gain_Pressure2);
  Serial.print(" : ");
  Serial.println(data[volume][1]);
  Serial.print(rtU.Gain_BPF1);
  Serial.print(" : ");
  Serial.println(data[volume][2]);
  Serial.print(rtU.Gain_BPF2);
  Serial.print(" : ");
  Serial.println(data[volume][3]);
  Serial.print(rtU.a0_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][4]);
  Serial.print(rtU.ma1_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][5]);
  Serial.print(rtU.ma2_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][6]);
  Serial.print(rtU.b0_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][7]);
  Serial.print(rtU.b1_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][8]);
  Serial.print(rtU.b2_Velocity2);
  Serial.print(" : ");
  Serial.println(data[volume][9]);
  Serial.print(rtU.a0_Voltage2);
  Serial.print(" : ");
  Serial.println(data[volume][10]);
  Serial.print(rtU.ma1_Voltage2);
  Serial.print(" : ");
  Serial.println(data[volume][11]);
  Serial.print(rtU.ma2_Voltage2);
  Serial.print(" : ");
  Serial.println(data[volume][12]);
  Serial.print(rtU.b0_Voltage2);
  Serial.print(" : ");
  Serial.println(data[volume][13]);
  Serial.print(rtU.b1_Voltage2);
  Serial.print(" : ");
  Serial.println(data[volume][14]);
  Serial.print(rtU.b2_Voltage2);
  Serial.print(" : ");
  Serial.println(data[volume][15]);
  Serial.print(rtU.a0_MHHC1);
  Serial.print(" : ");
  Serial.println(data[volume][16]);
  Serial.print(rtU.ma1_MHHC1);
  Serial.print(" : ");
  Serial.println(data[volume][17]);
  Serial.print(rtU.ma2_MHHC1);
  Serial.print(" : ");
  Serial.println(data[volume][18]);
  Serial.print(rtU.b0_MHHC1);
  Serial.print(" : ");
  Serial.println(data[volume][19]);
  Serial.print(rtU.b1_MHHC1);
  Serial.print(" : ");
  Serial.println(data[volume][20]);
  Serial.print(rtU.b2_MHHC1);
  Serial.print(" : ");
  Serial.println(data[volume][21]);
  Serial.print(rtU.a0_MHHC2);
  Serial.print(" : ");
  Serial.println(data[volume][22]);
  Serial.print(rtU.ma1_MHHC2);
  Serial.print(" : ");
  Serial.println(data[volume][23]);
  Serial.print(rtU.ma2_MHHC2);
  Serial.print(" : ");
  Serial.println(data[volume][24]);
  Serial.print(rtU.b0_MHHC2);
  Serial.print(" : ");
  Serial.println(data[volume][25]);
  Serial.print(rtU.b1_MHHC2);
  Serial.print(" : ");
  Serial.println(data[volume][26]);
  Serial.print(rtU.b2_MHHC2);
  Serial.print(" : ");
  Serial.println(data[volume][27]);
}


void printAppParametersMEM(int volume) {
  Serial.print(rtU.Gain_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 0));
  Serial.print(rtU.Gain_Pressure2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 1));
  Serial.print(rtU.Gain_BPF1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 2));
  Serial.print(rtU.Gain_BPF2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 3));
  Serial.print(rtU.a0_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 4));
  Serial.print(rtU.ma1_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 5));
  Serial.print(rtU.ma2_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 6));
  Serial.print(rtU.b0_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 7));
  Serial.print(rtU.b1_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 8));
  Serial.print(rtU.b2_Velocity2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 9));
  Serial.print(rtU.a0_Voltage2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 10));
  Serial.print(rtU.ma1_Voltage2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 11));
  Serial.print(rtU.ma2_Voltage2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 12));
  Serial.print(rtU.b0_Voltage2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 13));
  Serial.print(rtU.b1_Voltage2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 14));
  Serial.print(rtU.b2_Voltage2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 15));
  Serial.print(rtU.a0_MHHC1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 16));
  Serial.print(rtU.ma1_MHHC1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 17));
  Serial.print(rtU.ma2_MHHC1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 18));
  Serial.print(rtU.b0_MHHC1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 19));
  Serial.print(rtU.b1_MHHC1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 20));
  Serial.print(rtU.b2_MHHC1);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 21));
  Serial.print(rtU.a0_MHHC2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 22));
  Serial.print(rtU.ma1_MHHC2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 23));
  Serial.print(rtU.ma2_MHHC2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 24));
  Serial.print(rtU.b0_MHHC2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 25));
  Serial.print(rtU.b1_MHHC2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 26));
  Serial.print(rtU.b2_MHHC2);
  Serial.print(" : ");
  Serial.println(pgm_read_word_near(volume * 28 + MEMdata + 27));
}


// -----------------------------------------------------
void printAllVolumeData() {
  Serial.println("------Params @ VOLUME-------------------- ");

  for (int i = 0; i < VOLUMECOUNT; i++) {
    Serial.print("VOLUME ");
    Serial.print(i);
    Serial.print(": ");
    for (int j = 0; j < DATACOUNT; j++) {
      Serial.print(data[i][j]);
      Serial.print(" ");
    }
    Serial.println(" ");
    Serial.println(" ");
  }
}
// -----------------------------------------------------




// -----------------------------------------------------
/* Model step function */
unsigned long AXC_step(void) {  // 측정시간용

  unsigned long start_time = micros();  // 처리 시작 시간 측정
  real_T rtb_Delay;
  real_T rtb_Delay_h;
  real_T rtb_Delay_k;
  real_T rtb_Delay_o;
  real_T rtb_Divide;
  real_T rtb_Divide_a;
  real_T rtb_Divide_b;
  real_T rtb_Sum2_d;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Inport: '<Root>/Mic2_1'
   *  Inport: '<Root>/Mic2_2'
   */
  rtb_Sum2_d = rtU.Mic2_1 + rtU.Mic2_2;

  /* Delay: '<S3>/Delay' */
  rtb_Delay = rtDW.Delay_DSTATE;

  /* Product: '<S3>/Divide' incorporates:
   *  Delay: '<S3>/Delay'
   *  Delay: '<S3>/Delay1'
   *  Inport: '<Root>/Mic2_1'
   *  Inport: '<Root>/Mic2_2'
   *  Inport: '<Root>/a0_Velocity2'
   *  Inport: '<Root>/ma1_Velocity2'
   *  Inport: '<Root>/ma2_Velocity2'
   *  Product: '<S3>/Product'
   *  Product: '<S3>/Product2'
   *  Sum: '<Root>/Sum1'
   *  Sum: '<S3>/Sum2'
   *  Sum: '<S3>/Sum3'
   */
  rtb_Divide = ((rtDW.Delay_DSTATE * rtU.ma1_Velocity2 + rtDW.Delay1_DSTATE * rtU.ma2_Velocity2) + (rtU.Mic2_1 - rtU.Mic2_2)) / rtU.a0_Velocity2;

  /* Delay: '<S4>/Delay' */
  rtb_Delay_o = rtDW.Delay_DSTATE_o;

  /* Product: '<S4>/Divide' incorporates:
   *  Delay: '<S3>/Delay'
   *  Delay: '<S3>/Delay1'
   *  Delay: '<S4>/Delay'
   *  Delay: '<S4>/Delay1'
   *  Inport: '<Root>/Gain_Pressure2'
   *  Inport: '<Root>/Gain_Velocity2'
   *  Inport: '<Root>/a0_Voltage2'
   *  Inport: '<Root>/b0_Velocity2'
   *  Inport: '<Root>/b1_Velocity2'
   *  Inport: '<Root>/b2_Velocity2'
   *  Inport: '<Root>/ma1_Voltage2'
   *  Inport: '<Root>/ma2_Voltage2'
   *  Product: '<Root>/Product2'
   *  Product: '<Root>/Product3'
   *  Product: '<S3>/Product3'
   *  Product: '<S3>/Product4'
   *  Product: '<S3>/Product5'
   *  Product: '<S4>/Product'
   *  Product: '<S4>/Product2'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S3>/Sum'
   *  Sum: '<S3>/Sum1'
   *  Sum: '<S4>/Sum2'
   *  Sum: '<S4>/Sum3'
   */
  rtb_Divide_b = ((((rtDW.Delay_DSTATE * rtU.b1_Velocity2 + rtDW.Delay1_DSTATE * rtU.b2_Velocity2) + rtb_Divide * rtU.b0_Velocity2) * rtU.Gain_Velocity2 + rtb_Sum2_d * rtU.Gain_Pressure2) + (rtDW.Delay_DSTATE_o * rtU.ma1_Voltage2 + rtDW.Delay1_DSTATE_d * rtU.ma2_Voltage2)) / rtU.a0_Voltage2;

  /* Outport: '<Root>/AAC2' incorporates:
   *  Delay: '<S4>/Delay'
   *  Delay: '<S4>/Delay1'
   *  Inport: '<Root>/b0_Voltage2'
   *  Inport: '<Root>/b1_Voltage2'
   *  Inport: '<Root>/b2_Voltage2'
   *  Product: '<S4>/Product3'
   *  Product: '<S4>/Product4'
   *  Product: '<S4>/Product5'
   *  Sum: '<S4>/Sum'
   *  Sum: '<S4>/Sum1'
   */
  rtY.AAC2 = (rtDW.Delay_DSTATE_o * rtU.b1_Voltage2 + rtDW.Delay1_DSTATE_d * rtU.b2_Voltage2) + rtb_Divide_b * rtU.b0_Voltage2;

  /* Delay: '<S1>/Delay' */
  rtb_Delay_h = rtDW.Delay_DSTATE_d;

  /* Product: '<S1>/Divide' incorporates:
   *  Delay: '<S1>/Delay'
   *  Delay: '<S1>/Delay1'
   *  Inport: '<Root>/a0_MHHC1'
   *  Inport: '<Root>/ma1_MHHC1'
   *  Inport: '<Root>/ma2_MHHC1'
   *  Product: '<S1>/Product'
   *  Product: '<S1>/Product2'
   *  Sum: '<S1>/Sum2'
   *  Sum: '<S1>/Sum3'
   */
  rtb_Divide_a = ((rtDW.Delay_DSTATE_d * rtU.ma1_MHHC1 + rtDW.Delay1_DSTATE_p * rtU.ma2_MHHC1) + rtb_Sum2_d) / rtU.a0_MHHC1;

  /* Delay: '<S2>/Delay' */
  rtb_Delay_k = rtDW.Delay_DSTATE_m;

  /* Product: '<S2>/Divide' incorporates:
   *  Delay: '<S2>/Delay'
   *  Delay: '<S2>/Delay1'
   *  Inport: '<Root>/a0_MHHC2'
   *  Inport: '<Root>/ma1_MHHC2'
   *  Inport: '<Root>/ma2_MHHC2'
   *  Product: '<S2>/Product'
   *  Product: '<S2>/Product2'
   *  Sum: '<S2>/Sum2'
   *  Sum: '<S2>/Sum3'
   */
  rtb_Sum2_d = ((rtDW.Delay_DSTATE_m * rtU.ma1_MHHC2 + rtDW.Delay1_DSTATE_h * rtU.ma2_MHHC2) + rtb_Sum2_d) / rtU.a0_MHHC2;

  /* Outport: '<Root>/ANC' incorporates:
   *  Delay: '<S1>/Delay'
   *  Delay: '<S1>/Delay1'
   *  Delay: '<S2>/Delay'
   *  Delay: '<S2>/Delay1'
   *  Inport: '<Root>/Gain_BPF1'
   *  Inport: '<Root>/Gain_BPF2'
   *  Inport: '<Root>/b0_MHHC1'
   *  Inport: '<Root>/b0_MHHC2'
   *  Inport: '<Root>/b1_MHHC1'
   *  Inport: '<Root>/b1_MHHC2'
   *  Inport: '<Root>/b2_MHHC1'
   *  Inport: '<Root>/b2_MHHC2'
   *  Product: '<Root>/Product4'
   *  Product: '<Root>/Product5'
   *  Product: '<S1>/Product3'
   *  Product: '<S1>/Product4'
   *  Product: '<S1>/Product5'
   *  Product: '<S2>/Product3'
   *  Product: '<S2>/Product4'
   *  Product: '<S2>/Product5'
   *  Sum: '<Root>/Sum4'
   *  Sum: '<S1>/Sum'
   *  Sum: '<S1>/Sum1'
   *  Sum: '<S2>/Sum'
   *  Sum: '<S2>/Sum1'
   */
  rtY.ANC = ((rtDW.Delay_DSTATE_d * rtU.b1_MHHC1 + rtDW.Delay1_DSTATE_p * rtU.b2_MHHC1) + rtb_Divide_a * rtU.b0_MHHC1) * rtU.Gain_BPF1 + ((rtDW.Delay_DSTATE_m * rtU.b1_MHHC2 + rtDW.Delay1_DSTATE_h * rtU.b2_MHHC2) + rtb_Sum2_d * rtU.b0_MHHC2) * rtU.Gain_BPF2;

  /* Update for Delay: '<S3>/Delay' */
  rtDW.Delay_DSTATE = rtb_Divide;

  /* Update for Delay: '<S3>/Delay1' */
  rtDW.Delay1_DSTATE = rtb_Delay;

  /* Update for Delay: '<S4>/Delay' */
  rtDW.Delay_DSTATE_o = rtb_Divide_b;

  /* Update for Delay: '<S4>/Delay1' */
  rtDW.Delay1_DSTATE_d = rtb_Delay_o;

  /* Update for Delay: '<S1>/Delay' */
  rtDW.Delay_DSTATE_d = rtb_Divide_a;

  /* Update for Delay: '<S1>/Delay1' */
  rtDW.Delay1_DSTATE_p = rtb_Delay_h;

  /* Update for Delay: '<S2>/Delay' */
  rtDW.Delay_DSTATE_m = rtb_Sum2_d;

  /* Update for Delay: '<S2>/Delay1' */
  rtDW.Delay1_DSTATE_h = rtb_Delay_k;

  unsigned long end_time = micros();  // 처리 종료 시간 측정
  return end_time - start_time;
}


unsigned long delay_10us() {
  unsigned long start_time = micros();  // 처리 시작 시간 측정
  delayMicroseconds(10);
  unsigned long end_time = micros();  // 처리 종료 시간 측정
  return end_time - start_time;
}