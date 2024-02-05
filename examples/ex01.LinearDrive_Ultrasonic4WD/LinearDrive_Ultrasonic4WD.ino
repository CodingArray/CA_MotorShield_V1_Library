// ---------------------------------------------------------------------------
// Created by Coding Array - we@CodingArray.cc
// Copyright 2012 License: GNU GPL v3 http://www.gnu.org/licenses/gpl-3.0.html
// Ultrasonic Obstacle Avoidance 4WD Rubber Wheel + 4WD Mecanum  Wheel
// ---------------------------------------------------------------------------

#include <CA_MotorShield.h>        // 코딩어레이 모터 쉴드 라이브러리 포함
#include "NewPing.h"               // 초음파 센서 라이브러리 포함

// 초음파 센서 핀 설정
#define TRIGGER_PIN 2     // 초음파 센서 트리거 핀 (신호를 보내는 핀)
#define ECHO_PIN 4        // 초음파 센서 에코 핀 (신호를 받는 핀)
#define MAX_DISTANCE 300  // 초음파 센서가 감지할 수 있는 최대 거리 설정 (300cm)

// 모터 속도 설정
#define MIN_SPEED 80  // 모터가 돌 수 있는 최소 속도( TT모터 스타트 업 토크 = PWM 값 80 )

#define FORWARD_SPEED 150   // 전진 최대 속도(PWM 값), 초음파 센서 감지를 위해 보다 천천히 전진
#define BACKWARD_SPEED 150  // 후진 최대 속도(PWM 값)
#define TURN_SPEED 180      // 좌회전, 우회전 최대 속도(PWM 값) : 속도 값을 변경하면 회전 각도 변경

#define OBSTACLE_THRESHOLD 80  // 장애물을 감지하는 거리 임계값 (80cm)

// 모터 쉴드와 모터 초기화
CA_MotorShield DC_Motor(0x60);  // 모터 쉴드 객체 생성
CA_DCMotor *motor1 = DC_Motor.getMotor(1);         // 첫 번째 모터 연결
CA_DCMotor *motor2 = DC_Motor.getMotor(2);         // 두 번째 모터 연결
CA_DCMotor *motor3 = DC_Motor.getMotor(3);         // 세 번째 모터 연결
CA_DCMotor *motor4 = DC_Motor.getMotor(4);         // 네 번째 모터 연결

// 초음파 센서 객체 생성
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);  // 초음파 센서 설정

// RC카의 이동 방향을 나타내는 상수 정의
#define C_LEFT 1      // 좌회전
#define C_RIGHT 2     // 우회전
#define C_FORWARD 3   // 전진
#define C_BACKWARD 4  // 후진

/**
 * @brief RC카의 초기 설정을 수행하는 함수입니다.
 *
 * 이 함수는 RC카 프로그램의 시작 시 실행되며, 여러 초기화 작업을 수행합니다.
 * 시리얼 통신을 시작하여 컴퓨터와 데이터를 주고받을 수 있는 환경을 구성합니다.
 * 모터 쉴드의 초기화를 시도하고, 이 과정에서 모터 쉴드가 제대로 연결되어 있는지 확인합니다.
 * 만약 모터 쉴드가 제대로 연결되지 않았다면, 오류 메시지를 출력하고 프로그램을 멈춥니다.
 * 모터 쉴드가 제대로 연결되었다면, 관련 메시지를 출력합니다.
 *
 * setup 함수는 프로그램 실행 시 단 한 번만 호출됩니다.
 */
void setup() {
  Serial.begin(9600);                       // 시리얼 통신을 9600 보드레이트로 시작합니다. 이를 통해 컴퓨터와 데이터를 주고받을 수 있습니다.
  Serial.println("........Start........");  // 시리얼 모니터에 "........Start........"을 출력합니다. 이는 프로그램이 시작되었음을 나타냅니다.

  if (!DC_Motor.begin()) {                                           // 모터 쉴드를 기본 주파수인 1.6KHz로 시작합니다. 만약 모터 쉴드를 찾지 못하면 오류 메시지를 출력합니다.
    Serial.println("모터 쉴드를 찾을 수 없습니다. 배선을 확인하세요.");  // 모터 쉴드를 찾지 못했다는 메시지를 시리얼 모니터에 출력합니다.
    while (1)
      ;  // 무한 루프에 빠져 프로그램을 멈춥니다. 이는 모터 쉴드가 제대로 연결되지 않았을 경우를 나타냅니다.
  }
  Serial.println("모터 쉴드를 찾았습니다.");  // 모터 쉴드를 찾았다는 메시지를 시리얼 모니터에 출력합니다.

  delay(2000);  // 2초간 대기하여 RC카가 회전 후 안정화될 수 있도록 합니다. 
}

/**
 * @brief RC카의 메인 루프 함수로, 지속적으로 RC카의 상태를 갱신하고 적절한 동작을 수행합니다.
 *
 * 이 함수는 RC카의 기본 동작 루틴을 구현합니다. 주요 작업은 정면 장애물까지의 거리를 
 * 지속적으로 측정하고, 이 거리에 따라 RC카의 동작을 결정하는 것입니다.
 * 장애물이 가까이 있을 경우, RC카는 뒤로 이동한 후 왼쪽으로 회전하여 장애물을 피합니다.
 * 장애물이 멀리 있을 경우, RC카는 계속해서 전진합니다. 전진 중에도 정면 장애물까지의 거리를
 * 지속적으로 측정하여 장애물에 대응합니다.
 *
 * loop 함수는 아두이노 프로그램에서 지속적으로 반복 실행되며, RC카의 주요 행동을 결정합니다.
 */
void loop() {

  // 정면에 있는 장애물까지의 거리를 측정합니다.
  uint16_t frontDistance = measureDistance();  // 초음파 센서를 사용하여 정면 거리를 측정합니다.
  Serial.print("정면 거리: ");
  Serial.println(frontDistance);  // 시리얼 모니터에 정면 장애물까지의 거리를 표시합니다.

  // 장애물과의 거리를 임계값과 비교하여 조건에 따라 다르게 행동합니다.
  if (frontDistance < OBSTACLE_THRESHOLD) {
    // 장애물이 가까이 있을 경우, RC카가 뒤로 이동한 후 왼쪽으로 회전합니다.
    backwardLeftTurn(500);  //0.5초간 대기하여 RC카가 회전 후 안정화될 수 있도록 합니다.
  } else {
    // 장애물이 멀리 있을 경우, RC카는 계속 전진합니다.
    setMotorsDirection(C_FORWARD, FORWARD_SPEED);  // RC카를 전진 방향으로 설정하고 전진합니다.
    delay(500);                  // 대기시간 동안 대기하여 RC카가 회전 후 안정화될 수 있도록 합니다. 
}

/**
 * @brief RC카를 뒤로 이동시킨 후 왼쪽으로 회전시키는 함수입니다.
 * 이 함수는 먼저 RC카를 부드럽게 정지시키고, 후진 방향으로 설정하여 후진 속도로 이동을 시작합니다.
 * 지정된 대기 시간(`delayTime`)만큼 대기한 후, RC카의 방향을 왼쪽으로 변경하고 회전 속도로 회전을 시작합니다.
 * 이 과정을 통해 RC카는 뒤로 이동한 후 왼쪽으로 회전하게 됩니다. 
 * 회전 전후에는 RC카가 멈출 수 있도록 정지시키는 `StopAllMotors` 함수를 호출합니다.
 * 또한, `delay` 함수를 사용하여 지정된 시간만큼 대기합니다.
 * 이는 RC카가 회전 후 안정되는 시간을 제공하고, 시스템의 정상 작동을 확인합니다.
 *
 * @param delayTime 방향 전환 간 대기할 시간 (밀리초 단위).
 */
void backwardLeftTurn(unsigned long delayTime) {
  StopAllMotors();                                  // RC카가 정지합니다.
  setMotorsDirection(C_BACKWARD, BACKWARD_SPEED);   // RC카를 후진 방향으로 설정하고 후진 속도로 이동을 시작합니다.
  StopAllMotors();                                  // RC카가 정지합니다.
  delay(delayTime);                                 // 대기시간 동안 대기하여 RC카가 회전 후 안정화될 수 있도록 합니다.
  setMotorsDirection(C_LEFT, TURN_SPEED);           // RC카의 방향을 왼쪽으로 변경하고 회전 속도로 회전을 시작합니다.
  StopAllMotors();                                  // RC카가 정지합니다.
  delay(delayTime);                                 // 대기시간 동안 대기하여 RC카가 회전 후 안정화될 수 있도록 합니다.
}

/**
 * @brief NewPing 라이브러리를 사용하여 초음파 센서로부터 거리 측정값을 얻고 평균을 계산하는 함수입니다.
 *
 * 이 함수는 초음파 센서를 사용하여 여러 번 거리를 측정하고, 측정된 값들의 평균을 계산합니다.
 * 측정은 ping_median 메소드를 사용하여 이루어지며, 이는 여러 번 측정하여 얻은 값들 중 중간값을 반환합니다.
 * 이렇게 얻은 중간값을 cm 단위로 변환한 후, 총 합계에 더합니다.
 * 모든 측정이 완료되면, 유효한 측정값들의 평균을 계산하여 반환합니다.
 * 
 * 이는 장시간의 측정 과정에서 시스템이 멈추지 않도록 보장하기 위함입니다.
 *
 * @return 평균 거리 (unsigned int 형식). 측정값이 없거나 오류가 발생한 경우 최대 거리(MAX_DISTANCE)를 반환할 수 있습니다.
 */
unsigned int measureDistance() {
  const int maxReadings = 5;   // 유효한 측정값을 5회 수집하기 위한 변수입니다.
  uint16_t totalDistance = 0;  // 측정된 거리의 합을 저장하기 위한 변수입니다.
  uint8_t validReadings = 0;   // 유효한 측정 횟수를 세기 위한 변수입니다.

  while (validReadings < maxReadings) {
    uint16_t uS = sonar.ping_median(5);        // 초음파 센서로 5번 측정하여 중간값을 마이크로초 단위로 측정합니다.
    uint16_t distance = sonar.convert_cm(uS);  // 측정된 마이크로초 값을 센티미터로 변환합니다.

    if (distance == 0) distance = MAX_DISTANCE;  // 측정된 거리가 0이면 최대 거리로 간주합니다.

    totalDistance += distance;  // 측정된 거리를 총 합계에 더합니다.
    validReadings++;            // 유효한 측정 횟수를 증가시킵니다.
  }

  return totalDistance / validReadings;  // 평균 거리를 계산하여 반환합니다.
}

/**
 * @brief RC카의 모터들을 특정 방향으로 설정하는 함수입니다.
 *
 * 이 함수는 입력된 방향에 따라 RC카의 모터들을 적절히 제어합니다. 왼쪽, 오른쪽,
 * 전진, 후진 등의 방향에 따라 모터들의 회전 방향을 설정합니다. 이를 통해 RC카가
 * 원하는 방향으로 움직일 수 있도록 합니다.
 *
 * @param direction 설정할 모터의 방향. C_LEFT, C_RIGHT, C_FORWARD, C_BACKWARD 중 하나를 사용합니다.
 * @param speed 모터가 도달할 속도. 이 속도는 감속 또는 증가됩니다.
 */
void setMotorsDirection(uint8_t direction, uint8_t speed) {
  // 모터의 방향을 설정합니다.
  if (direction == C_LEFT) {
    motor1->run(BACKWARD);  // 1번 모터를 뒤로 돌려 왼쪽으로 회전시킵니다.
    motor2->run(BACKWARD);  // 2번 모터도 뒤로 돌려 왼쪽으로 회전시킵니다.
    motor3->run(FORWARD);   // 3번 모터를 앞으로 돌려 왼쪽으로 회전시킵니다.
    motor4->run(FORWARD);   // 4번 모터도 앞으로 돌려 왼쪽으로 회전시킵니다.
  } else if (direction == C_RIGHT) {
    motor1->run(FORWARD);   // 1번 모터를 앞으로 돌려 오른쪽으로 회전시킵니다.
    motor2->run(FORWARD);   // 2번 모터도 앞으로 돌려 오른쪽으로 회전시킵니다.
    motor3->run(BACKWARD);  // 3번 모터를 뒤로 돌려 오른쪽으로 회전시킵니다.
    motor4->run(BACKWARD);  // 4번 모터도 뒤로 돌려 오른쪽으로 회전시킵니다.
  } else if (direction == C_FORWARD) {
    motor1->run(FORWARD);  // 모든 모터를 앞으로 돌려 RC카가 전진하도록 합니다.
    motor2->run(FORWARD);
    motor3->run(FORWARD);
    motor4->run(FORWARD);
  } else if (direction == C_BACKWARD) {
    motor1->run(BACKWARD);  // 모든 모터를 뒤로 돌려 RC카가 후진하도록 합니다.
    motor2->run(BACKWARD);
    motor3->run(BACKWARD);
    motor4->run(BACKWARD);
  }

  motor1->setSpeed(speed);
  motor2->setSpeed(speed);
  motor3->setSpeed(speed);
  motor4->setSpeed(speed);
}

/**
 * @brief RC카의 모든 모터들을 점차적으로 감속시켜 정지시키는 함수입니다.
 *
 * 함수 실행이 끝나면 모든 모터는 정지 상태(RELEASE)가 됩니다, 
 */
void StopAllMotors() {
  // 모든 모터를 정지시킵니다.
  motor1->run(RELEASE);
  motor2->run(RELEASE);
  motor3->run(RELEASE);
  motor4->run(RELEASE);
}

