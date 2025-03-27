#define TX1_PIN 5  // STM32-door 보드1 RX (PA10) 과 연결
#define RX1_PIN 4  // STM32-door 보드1 TX (PA9) 과 연결

#define TX2_PIN 17  // STM32-battery 보드2 RX (예: PC11, 2번(안쪽)) 과 연결
#define RX2_PIN 16  // STM32-battery 보드2 TX (예: PC10, 1번(바깥)) 과 연결

#define BLYNK_TEMPLATE_ID "INPUT YOUR ID"
#define BLYNK_TEMPLATE_NAME "INPUT YOUR NAME"
#define BLYNK_AUTH_TOKEN "INPUT YOUR TOKEN"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "INPUT YOUR WIFI ssid";
char pass[] = "INPUT YOUR WIFI pass";

HardwareSerial mySerial1(1);  // UART1 사용
HardwareSerial mySerial2(2);  // UART2 (보드2와 통신)
BlynkTimer timer;  // 🔹 Blynk 타이머 추가

// 초기 상태
int switchState = 0; 
int engineState = 0;
int soc = 100;
int soh = 100;

// 🔹 문콕방지 시스템 잠금/해제
BLYNK_WRITE(V1) {
  switchState = param.asInt();  
  Serial.print("Switch State: ");
  Serial.println(switchState);
  mySerial1.println(switchState); //stm_door 로 switchState 전송
}

// 🔹 차 시동 (시동 ON 시 문 자동 OFF)
BLYNK_WRITE(V0) {
  engineState = param.asInt();
  Serial.print("Engine State: ");
  Serial.println(engineState);
  mySerial2.println(engineState); //stm_battery로 engineState 전송

  // 🔸 시동이 켜지면 문 시스템 자동 OFF
  if (engineState == 1) {
    switchState = 0;  
    Blynk.virtualWrite(V1, LOW); // Blynk에서 문 시스템 OFF
    Serial.println("Engine ON -> Locking Doors (V1=0)");
    mySerial1.println(switchState);
  }
  else if(engineState == 0){
    switchState = 1;
    Blynk.virtualWrite(V1, switchState); //문열림 시스템 ON
    mySerial1.println(switchState);

  
    
    Blynk.virtualWrite(V2, soc);  // 최신 SOC 값 전송
    Blynk.virtualWrite(V3, soh);  // 최신 SOH 값 전송
    Serial.print("final SOC: ");
    Serial.println(soc);
    Serial.print("final SOH: ");
    Serial.println(soh);

    mySerial1.println(6); //InPeople=1;
    // switchState = param.asInt();  
    // Serial.print("Switch State: ");
    // Serial.println(switchState);
    // mySerial1.println(switchState);
  }
}

// 🔹 배터리 잔량 확인 시스템
BLYNK_WRITE(V2) {
  soc = param.asInt();
  Serial.print("SOC: ");
  Serial.println (soc);
}

BLYNK_WRITE(V3){
  soh = param.asInt();
  Serial.print(" SOH: ");
  Serial.println (soh);
}

// 🔹 랜덤 배터리 잔량 전송 (1초마다 실행)
void myTimerEvent() {
  if(engineState==1){
    Blynk.virtualWrite(V2, soc);  // Blynk에 배터리 상태 업데이트
    Serial.print("updated SOC: ");
    Serial.print(soc);

    Blynk.virtualWrite(V3, soh);  // Blynk에 배터리 상태 업데이트
    Serial.print(" updated SOH: ");
    Serial.println(soh);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("ESP32 Ready!");
  mySerial1.begin(115200, SERIAL_8N1, RX1_PIN, TX1_PIN);
  mySerial2.begin(115200, SERIAL_8N1, RX2_PIN, TX2_PIN);
  mySerial2.setTimeout(1000);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  Serial.println("UART1 & UART2 Initialized.");

  timer.setInterval(1000L, myTimerEvent);  // 🔹 5초마다 myTimerEvent() 실행
}

void loop() {
  if(engineState==1){
    if (mySerial2.available()) {
      String data = mySerial2.readStringUntil('\n');  // 개행 문자까지 읽기
      Serial.println("from stm32-battery: " + data);

      // 'SOC : 3%, SOH : 123%' 형식에서 숫자만 추출
      if (sscanf(data.c_str(), "SOC : %d%%, SOH : %d%%", &soc, &soh) == 2) {
          
          Serial.print(soc);
          Serial.print(" ");
          Serial.println(soh);
      } 
      else {
          Serial.println("데이터 수신 오류");
      }
      
    }
  }

  delay(1000);

  Blynk.run();
  timer.run();  // 🔹 타이머 실행 (myTimerEvent()가 주기적으로 실행됨)
}
