// ---------------------------------------------------------------------------
// Created by Coding Array - we@CodingArray.cc
// Copyright 2024 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
// IR Remote Control Example for 4WD Rubber Wheel
// ---------------------------------------------------------------------------

#include <CA_MotorShield.h>
//#include <IRremote.h>  // IRremote 라이브러리 버전 4.2.1, Ken Shirriff 작성. 
#include <IRremote.hpp>  // IRremote 라이브러리 버전 4.2.1, Ken Shirriff 작성 / C++ 지원. 
//https://github.com/Arduino-IRremote/Arduino-IRremote - 이 라이브러리의 소스 코드와 문서는 GitHub에서 확인할 수 있습니다.


// 모터 쉴드 객체를 생성하고 I2C 주소를 초기화합니다.
CA_MotorShield dcMotor = CA_MotorShield(); // 기본 주소 0x60으로 생성 

// 각 모터에 대한 포인터를 초기화합니다.
CA_DCMotor *motor1 = dcMotor.getMotor(1);         // 첫 번째 모터 연결
CA_DCMotor *motor2 = dcMotor.getMotor(2);         // 두 번째 모터 연결
CA_DCMotor *motor3 = dcMotor.getMotor(3);         // 세 번째 모터 연결
CA_DCMotor *motor4 = dcMotor.getMotor(4);         // 네 번째 모터 연결

///////////////////////////////////////////////////////////////

// 최대 속도를 정의하는 상수입니다.
#define MAX_SPEED 100 // 최대 속도는 0에서 100까지 정의할 수 있습니다. 
                      // 이 값은 RC카의 모터 속도 제어에 사용되며, 모터가 달성할 수 있는 최고 속도를 설정합니다. 

uint16_t controlSpeed = 70; // 모터의 초기 제어 속도를 70으로 설정합니다. 이 변수는 RC카의 현재 속도를 나타내며, 
                            // IR 리모콘의 버튼을 사용하여 증가시키거나 감소시킬 수 있습니다.

// 코딩어레이 스마트 RC카 방향 전환 제어 상수
// 이 상수들은 메카넘 휠을 장착한 코딩어레이 스마트 RC카의 다양한 이동 및 회전 동작을 정의합니다.
// 각 상수는 RC카의 특정한 동작 모드를 나타내며, 이를 통해 RC카를 다양한 방향으로 조작할 수 있습니다.
#define CMD_RELEASE 0             // 정지: 코딩어레이 스마트 RC카를 정지시킵니다.
#define CMD_FORWARD 1             // ↑: 코딩어레이 스마트 RC카를 전진시킵니다.
#define CMD_BACKWARD 2            // ↓: 코딩어레이 스마트 RC카를 후진시킵니다.
#define CMD_LEFT 3                // ←: 코딩어레이 스마트 RC카를 좌측으로 회전시킵니다.
#define CMD_RIGHT 4               // →: 코딩어레이 스마트 RC카를 우측으로 회전시킵니다.

#define IR_RECEIVE_PIN 3    // IR 수신기를 연결할 아두이노의 핀 번호를 3번으로 설정합니다. 인터럽트 예제와 호환되도록 3번 핀을 사용합니다.
//#define IR_SEND_PIN 3     // IR 송신기를 연결할 아두이노의 핀 번호를 3번으로 설정할 수 있습니다. 
#define ALTERNATIVE_IR_FEEDBACK_LED_PIN 13  // 수신되는 IR 신호 피드백을 아두이노의 핀 번호 13번으로 설정합니다. LED 출력.

// NEC 프로토콜 기반 코딩어레이 IR 리모컨 명령 코드 목록
// NEC 프로토콜 코드 예시: 0x00FF45BA
//-----------------------------------------------------
// 이 구조는 4부분으로 나뉘어져 있습니다:
// 1. 어드레스 (Address): 0x00
//    - 특정 장치를 식별하는 데 사용되는 부분입니다. 여기서는 0x00으로 표현됩니다.
// 2. 어드레스 반전 (Address Inverse): 0xFF
//    - 데이터 전송의 무결성 검증을 위해 어드레스의 반전 값입니다. 여기서는 0xFF로 표현됩니다.
// 3. 명령값 (Command): 0x45
//    - 리모컨 버튼의 실제 기능을 나타내는 부분으로, 여기서 '숫자 1'에 해당하는 기능이 0x45입니다.
// 4. 명령값 반전 (Command Inverse): 0xBA
//    - 명령의 정확성을 확인하기 위한 명령값의 반전 데이터입니다. 여기서는 0xBA로 표현됩니다.
//-----------------------------------------------------
/*
0x00FF45BA; // 숫자 1
0x00FF46B9; // 숫자 2
0x00FF47B8; // 숫자 3
0x00FF44BB; // 숫자 4
0x00FF40BF; // 숫자 5
0x00FF43BC; // 숫자 6
0x00FF07F8; // 숫자 7
0x00FF15EA; // 숫자 8
0x00FF09F6; // 숫자 9
0x00FF19E6; // 숫자 0
0x00FF16E9; // 전원 or +
0x00FF0DF2; // 메뉴 or -
0x00FF0CF3; // 좌측 상단 ↖
0x00FF18E7; // 상단 ↑
0x00FF5EA1; // 우측 상단 ↗
0x00FF08F7; // 좌측 ←
0x00FF1CE3; // 엔터 ⏎
0x00FF5AA5; // 우측 →
0x00FF42BD; // 좌측 하단 ↙
0x00FF52AD; // 하단 ↓
0x00FF4AB5; // 우측 하단 ↘
*/

/**
 * @brief 시스템 시작 시 초기 설정을 수행하는 함수입니다.
 *
 * 이 함수는 시스템이 시작될 때 한 번만 실행되며, 시리얼 통신을 시작하고, 모터 쉴드를 초기화하며,
 * 적외선(IR) 리시버를 설정합니다. 또한, 모터의 동작 주파수를 설정하고, 시스템이 안정화될 수 있도록 초기 대기 시간을 제공합니다.
 *
 * 시리얼 통신은 디버깅 메시지를 출력하기 위해 사용되며, 모터 쉴드의 정상 작동 여부를 확인하여
 * 문제가 있을 경우 사용자에게 경고 메시지를 제공합니다. 이는 문제 해결을 위한 첫 단계로, 배선 연결 상태를 점검하게 합니다.
 * 
 * IR 리시버의 설정은 리모콘 신호를 받아들일 준비를 합니다. 이는 리모콘 컨트롤러로부터의 명령을 수신하고 처리하는 데 필수적입니다.
 */
void setup() {
  Serial.begin(115200);  // 시리얼 통신을 115200 baud rate로 시작합니다.
  Serial.println("........Start........");  // 시작 메시지를 시리얼 모니터에 출력합니다.

  // 모터 쉴드 초기화와 동작 주파수 설정
  if (!dcMotor.begin()) {  // 모터 쉴드를 기본 주파수인 1.6KHz로 시작합니다. 실패할 경우 오류 메시지를 출력합니다.
    Serial.println("모터 쉴드를 찾을 수 없습니다. 배선을 확인하세요.");  // 모터 쉴드가 없다는 메시지를 출력합니다.
    while (1);  // 오류 상태에서 무한 루프에 진입하여 프로그램을 중지합니다.
  }
  // 모터의 동작 주파수 설정 부분은 현재 비활성화되어 있습니다.
  //dcMotor.setFrequency(1600); // 이 코드를 활성화하면 모터의 동작 주파수를 1.6Khz로 설정할 수 있습니다.

  // IR 리시버 설정
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);  // IR 리시버를 시작하고, LED 피드백을 활성화합니다.

  delay(2000);  // 시스템이 안정화될 수 있도록 2초간 대기합니다.
}

/**
 * @brief 메인 루프에서 IR 리모콘 신호를 지속적으로 수신하여 RC카의 동작을 제어하는 함수입니다.
 *
 * 이 함수는 무한 루프 내에서 실행되며, IR 리모콘으로부터의 입력을 받아 RC카의 모터 방향과 속도를 조절합니다.
 * NEC 프로토콜을 사용하는 IR 신호를 해석하고, 해당 신호에 따라 다양한 동작(전진, 후진, 좌회전, 우회전 등)을 실행합니다.
 * 또한, 속도 조절 기능을 통해 사용자는 RC카의 속도를 증가시키거나 감소시킬 수 있습니다.
 *
 * IR 리모콘의 버튼에 따라 다양한 명령을 실행하며, '리피트 코드'는 무시하여 중복 명령을 방지합니다.
 * 수신된 데이터는 시리얼 모니터에 출력되어, 디버깅 과정에서 유용하게 사용됩니다.
 */
void loop() {
  // IR 리시버가 데이터를 수신했는지 확인합니다.
  if (IrReceiver.decode()) {
    // 수신된 데이터가 NEC 프로토콜을 사용하는지 확인합니다.
    if (IrReceiver.decodedIRData.protocol == NEC) {
      
      // 수신된 신호가 리피트 코드인 경우 무시하고 리시버를 재개합니다. 
      // 리피트 코드란 IR 리모콘 버튼을 누르고 있으면 동일한 신호가 반복적으로 발신되는 코드입니다.
      // 이미 실행한 명령에 대한 동일 코드 재실행 방지시 사용합니다.
      if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) {
        // 다음 명령을 받기 위해 IR 수신을 재개합니다.
        IrReceiver.resume();
        return;
      }

      // 수신된 데이터의 프로토콜, 주소, 명령을 시리얼 모니터에 출력합니다.
      Serial.print(F("수신된 데이터: 프로토콜=NEC, 주소=0x"));
      Serial.print(IrReceiver.decodedIRData.address, HEX);
      Serial.print(F(", 명령=0x"));
      Serial.println(IrReceiver.decodedIRData.command, HEX);

      // 코딩어레이 IR 리모콘 주소(주소 : 0x00)에서 오는 명령만을 처리합니다.
      if (IrReceiver.decodedIRData.address == 0x00) {
        // 수신된 명령에 따라 적절한 동작을 실행합니다.
        switch (IrReceiver.decodedIRData.command) {
          case 0x1C: setMotorsDirection(CMD_RELEASE, 0); break;                      // 엔터 ⏎: 정지
          case 0x18: setMotorsDirection(CMD_FORWARD, controlSpeed); break;           // 상단 ↑: 전진
          case 0x52: setMotorsDirection(CMD_BACKWARD, controlSpeed); break;          // 하단 ↓: 후진
          case 0x08: setMotorsDirection(CMD_LEFT, controlSpeed); break;              // 촤측 ←: 좌회전
          case 0x5A: setMotorsDirection(CMD_RIGHT, controlSpeed); break;             // 우측 →: 우회전
          case 0x16:
            // 전원 or +: 속도 증가
            if (controlSpeed < MAX_SPEED) controlSpeed += 5;
            setMotorsSpeed(controlSpeed);
            break;
          case 0x0D:
            // 메뉴 or -: 속도 감소
            if (controlSpeed > 0) controlSpeed -= 5;
            setMotorsSpeed(controlSpeed);
            break;
          default:
            break;
        }
      }
    }
    // 다음 명령을 받기 위해 IR 수신을 재개합니다.
    IrReceiver.resume();
  }
}

/**
 * @brief RC카의 모터들의 방향을 설정하고, 속도를 조절하는 함수입니다.
 *
 * 이 함수는 모터의 방향 전환 전에 속도를 0으로 설정하여 급격한 전류 증가를 방지하고,
 * 따라서 RC카의 안정성을 향상시킵니다. 그 후 지정된 방향으로 모터의 회전 방향을 설정합니다.
 * CMD_RELEASE가 아닌 경우에만 지정된 속도로 모터의 속도를 조절하여, RC카가 원하는 방향으로 움직이도록 합니다.
 *
 * @param direction 설정할 모터의 방향입니다. CMD_LEFT, CMD_RIGHT, CMD_FORWARD, CMD_BACKWARD, CMD_RELEASE 중 하나를 사용합니다.
 *                  CMD_RELEASE를 선택할 경우 모든 모터가 정지합니다.
 * @param speed 모터가 도달할 속도입니다. 이 속도는 모터의 방향이 CMD_RELEASE가 아닐 때 적용됩니다.
 *              속도는 pwm 신호로 조절됩니다.
 */
void setMotorsDirection(uint8_t direction, uint16_t speed) {
  // 방향을 변경하기 전에 모터의 속도를 0으로 설정하여 급격한 전류 증가를 방지합니다.
  setMotorsSpeed(0);  // 안정성 확보를 위해 모터의 속도를 0으로 설정합니다.
  delay(50);  // 속도 변경 후 안정화를 위한 짧은 지연입니다.

  if (direction == CMD_RELEASE) {
    // RELEASE 명령을 받으면 모든 모터를 정지시킵니다.
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
    motor4->run(RELEASE);
  } else if (direction == CMD_LEFT) {
    // 좌회전 명령을 받으면 1번과 2번 모터는 뒤로, 3번과 4번 모터는 앞으로 설정하여 제자리에서 좌회전을 합니다.
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
  } else if (direction == CMD_RIGHT) {
    // 우회전 명령을 받으면 1번과 2번 모터는 앞으로, 3번과 4번 모터는 뒤로 설정하여 제자리에서 우회전을 합니다.
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(BACKWARD);
    motor4->run(BACKWARD);
  } else if (direction == CMD_FORWARD) {
    // 전진을 위해 모든 모터를 앞으로 설정합니다.
    motor1->run(FORWARD);
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
  } else if (direction == CMD_BACKWARD) {
    // 후진을 위해 모든 모터를 뒤로 설정합니다.
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);
    motor4->run(BACKWARD);
  } 

  if(direction != CMD_RELEASE) setMotorsSpeed(speed); // 모터가 도달할 속도입니다. 이 속도는 모터의 방향이 CMD_RELEASE가 아닐 때 적용됩니다.

}

/**
 * @brief 모터의 속도를 설정하는 함수입니다.
 *
 * 이 함수는 모든 모터에 대해 동일한 속도를 설정합니다. PWM 신호를 사용하여 속도를 조절하며,
 * 이를 통해 RC카의 움직임을 더 섬세하게 제어할 수 있습니다.
 *
 * @param speed 설정할 속도입니다. 이 값은 모든 모터에 동일하게 적용됩니다.
 */
void setMotorsSpeed(uint16_t speed) {
  // 모든 모터에 대해 동일한 속도를 설정합니다.
  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  motor3->setSpeed(speed);
  motor4->setSpeed(speed);
}
