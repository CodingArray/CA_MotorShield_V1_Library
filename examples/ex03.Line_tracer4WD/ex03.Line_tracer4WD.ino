// ---------------------------------------------------------------------------
// Created by Coding Array - we@CodingArray.cc
// Copyright 2024 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
// Line Tracing Example for 4WD Rubber Wheel + 4WD Mecanum Wheel
// ---------------------------------------------------------------------------

#include <CA_MotorShield.h>

// 모터 쉴드 객체를 생성하고 I2C 주소를 초기화합니다.
CA_MotorShield dcMotor(0x60);

// 각 모터에 대한 포인터를 초기화합니다.
CA_DCMotor *motor1 = dcMotor.getMotor(1);         // 첫 번째 모터 연결
CA_DCMotor *motor2 = dcMotor.getMotor(2);         // 두 번째 모터 연결
CA_DCMotor *motor3 = dcMotor.getMotor(3);         // 세 번째 모터 연결
CA_DCMotor *motor4 = dcMotor.getMotor(4);         // 네 번째 모터 연결

// 최대 속도를 정의하는 상수입니다.
#define MAX_SPEED 80 // 최대 속도는 0에서 100까지 정의할 수 있습니다.

// 코딩어레이 스마트 RC카 방향 전환 제어 상수
// 이 상수들은 메카넘 휠을 장착한 코딩어레이 스마트 RC카의 다양한 이동 및 회전 동작을 정의합니다.
// 각 상수는 RC카의 특정한 동작 모드를 나타내며, 이를 통해 RC카를 다양한 방향으로 조작할 수 있습니다.
#define CMD_RELEASE 0           // 정지: 코딩어레이 스마트 RC카를 정지시킵니다.
#define CMD_FORWARD 1           // ↑: 코딩어레이 스마트 RC카를 전진시킵니다.
#define CMD_BACKWARD 2          // ↓: 코딩어레이 스마트 RC카를 후진시킵니다.
#define CMD_LEFT 3              // ←: 코딩어레이 스마트 RC카를 좌측으로 회전시킵니다.
#define CMD_RIGHT 4             // →: 코딩어레이 스마트 RC카를 우측으로 회전시킵니다.

// 적외선(IR) 센서 핀을 정의하는 상수입니다. 이 센서들은 무광 흑색과 밝은색 사이를 구분하는 데 사용됩니다.
const int leftSensor = A2;  // 왼쪽 적외선 센서로, 라인트레이서의 경로에 따라 무광 흑색과 밝은색을 감지합니다.
const int rightSensor = A3; // 오른쪽 적외선 센서로, 마찬가지로 라인트레이서의 경로에 따라 무광 흑색과 밝은색을 감지합니다.

/**
 * @brief 라인트레이서 프로그램의 초기 설정을 수행하는 함수입니다.
 *
 * 이 함수는 라인트레이서가 시작될 때 한 번 호출되어, 필요한 하드웨어 설정을 초기화합니다.
 * - 시리얼 통신을 시작하여 디버깅 메시지를 출력할 수 있도록 설정합니다.
 * - 적외선 센서를 사용하기 위해 해당 핀을 입력 모드로 설정합니다.
 * - 모터 쉴드를 초기화하고, 모터의 동작을 위한 주파수를 설정합니다.
 *
 * @details
 * Serial.begin(9600): 시리얼 통신을 9600bps의 전송 속도로 시작합니다. 이를 통해
 * 프로그램의 실행 상태를 모니터링하고, 디버깅 정보를 출력할 수 있습니다.
 *
 * pinMode(): leftSensor와 rightSensor 핀을 INPUT 모드로 설정하여, 적외선 센서로부터
 * 신호를 읽을 수 있게 합니다.
 *
 * dcMotor.begin(1600): 모터 쉴드를 초기화 및 모터의 동작 주파수를 1.6Khz로 설정합니다.
 * dcMotor.setFrequency(1600): 별도로 모터의 동작 주파수를 1.6Khz로 설정할 수 있습니다. 
 */
void setup() {
  Serial.begin(9600);                       
  Serial.println("........Start........");  

  // 센서 핀을 입력으로 설정합니다.
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  // 모터 쉴드를 초기화 및 모터의 동작 주파수를 1.6Khz로 설정합니다.
  if (!dcMotor.begin(1600)) {                                           // 모터 쉴드를 1.6KHz로 주파수로 시작합니다. 만약 모터 쉴드를 찾지 못하면 오류 메시지를 출력합니다.
    Serial.println("모터 쉴드를 찾을 수 없습니다. 배선을 확인하세요.");  // 모터 쉴드를 찾지 못했다는 메시지를 시리얼 모니터에 출력합니다.
    while (1)
      ;  // 무한 루프에 빠져 프로그램을 멈춥니다. 이는 모터 쉴드가 제대로 연결되지 않았을 경우를 나타냅니다.
  }
  //dcMotor.setFrequency(1600); //모터의 동작 주파수를 1.6Khz로 설정합니다.

  delay(2000);  // 2초간 대기하여 RC카가 회전 후 안정화될 수 있도록 합니다. 
}

/**
 * @brief 주요 실행 루프에서 라인트레이서의 동작을 제어합니다.
 *
 * 이 함수는 라인트레이서의 적외선 센서 상태를 지속적으로 검사하여, 센서가 감지하는
 * 바닥의 색에 따라 모터의 방향과 속도를 조절합니다. 센서가 검은색 라인을 감지하면
 * 낮은 전기 신호(LOW)를, 밝은 바닥을 감지하면 높은 전기 신호(HIGH)를 출력합니다.
 * 이 정보를 바탕으로 라인트레이서는 전진, 좌회전, 우회전, 정지 중 하나의 동작을 수행합니다.
 *
 * - 전진: 두 센서 모두 검은색 라인 위에 있을 때
 * - 좌회전: 왼쪽 센서만 검은색 라인을 감지할 때
 * - 우회전: 오른쪽 센서만 검은색 라인을 감지할 때
 * - 정지: 두 센서 모두 밝은 바닥을 감지할 때
 *
 * 이 로직은 라인트레이서가 주어진 경로를 따라 정확하게 이동하도록 보장합니다.
 */
void loop() {
  // 전진: 왼쪽과 오른쪽 적외선 센서가 모두 검은색 라인 위에 있을 때
  // 센서가 검은색을 감지하면 디지털 신호를 LOW(0)로 출력합니다.
  if (digitalRead(leftSensor) == LOW && digitalRead(rightSensor) == LOW) {
    setMotorsDirection(CMD_FORWARD, MAX_SPEED); // 모터를 전진 방향으로 설정하고 최대 속도로 이동합니다.
  }
  // 좌회전: 왼쪽 센서만 검은색을 감지하고, 오른쪽 센서는 흰색 바닥을 감지할 때
  // 왼쪽 센서가 LOW, 오른쪽 센서가 HIGH 신호를 출력합니다.
  else if (digitalRead(leftSensor) == LOW && digitalRead(rightSensor) == HIGH) {
    setMotorsDirection(CMD_LEFT, MAX_SPEED); // 모터를 좌회전 방향으로 설정하고 최대 속도로 회전합니다.
  }
  // 우회전: 오른쪽 센서만 검은색을 감지하고, 왼쪽 센서는 흰색 바닥을 감지할 때
  // 오른쪽 센서가 LOW, 왼쪽 센서가 HIGH 신호를 출력합니다.
  else if (digitalRead(leftSensor) == HIGH && digitalRead(rightSensor) == LOW) {
    setMotorsDirection(CMD_RIGHT, MAX_SPEED); // 모터를 우회전 방향으로 설정하고 최대 속도로 회전합니다.
  }
  // 정지: 두 센서 모두 흰색 바닥을 감지할 때
  // 두 센서 모두 HIGH 신호를 출력합니다.
  else {
    setMotorsDirection(CMD_RELEASE, 0); // 모든 모터를 정지시킵니다.
  }
}

/**
 * @brief RC카의 모터들을 특정 방향으로 설정하고, 해당 방향으로 움직이도록 속도를 조절하는 함수입니다.
 *
 * 이 함수는 지정된 방향으로 모터의 회전 방향을 설정하고, 
 * CMD_RELEASE가 아닌 경우에만 지정된 속도로 모터의 속도를 조절합니다. 이를 통해 RC카가 원하는 방향으로 움직이고,
 * 속도를 조절할 수 있습니다.
 *
 * @param direction 설정할 모터의 방향. CMD_LEFT, CMD_RIGHT, CMD_FORWARD, CMD_BACKWARD, CMD_RELEASE 중 하나를 사용합니다.
 *                  CMD_RELEASE를 선택할 경우 모든 모터가 정지합니다.
 * @param speed 모터가 도달할 속도입니다. 이 속도는 모터의 방향이 CMD_RELEASE가 아닐 때 적용됩니다. 속도는 pwm 신호로 조절됩니다.
 */
void setMotorsDirection(uint8_t direction, uint8_t speed) {
  // 모터의 방향을 설정하는 조건문입니다.
  if (direction == CMD_RELEASE) {
    // RELEASE 명령을 받으면 모든 모터를 정지시킵니다.
    motor1->run(RELEASE);
    motor2->run(RELEASE);
    motor3->run(RELEASE);
    motor4->run(RELEASE);
  } else if (direction == CMD_LEFT) {
    // 왼쪽 회전을 위해 1번과 2번 모터는 뒤로, 3번과 4번 모터는 앞으로 설정합니다.
    motor1->run(BACKWARD);
    motor2->run(BACKWARD);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
  } else if (direction == CMD_RIGHT) {
    // 오른쪽 회전을 위해 1번과 2번 모터는 앞으로, 3번과 4번 모터는 뒤로 설정합니다.
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
  // CMD_RELEASE 상태가 아닌 경우, 지정된 속도로 모터의 속도를 조절합니다.
  if(direction != CMD_RELEASE) setMotorsSpeed(speed);
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