#include <CA_MotorShield.h>

// 모터 쉴드 객체를 생성하고 I2C 주소를 초기화합니다.
CA_MotorShield dcMotor(0x60);

// 각 모터에 대한 포인터를 초기화합니다.
CA_DCMotor *motor1 = dcMotor.getMotor(1);
CA_DCMotor *motor2 = dcMotor.getMotor(2);
CA_DCMotor *motor3 = dcMotor.getMotor(3);
CA_DCMotor *motor4 = dcMotor.getMotor(4);

// 최대 속도를 정의하는 상수입니다.
#define MAX_SPEED 120

// 라인 트레이서의 이동 방향을 나타내는 상수입니다.
#define C_LEFT 1      // 좌회전
#define C_RIGHT 2     // 우회전
#define C_FORWARD 3   // 전진
#define C_BACKWARD 4  // 후진

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
 * dcMotor.begin()과 dcMotor.setFrequency(1600): 모터 쉴드를 사용하기 위한 초기화 과정을
 * 수행하고, 모터의 동작 주파수를 1600Hz로 설정합니다. 이 설정은 모터의 성능과 효율에 영향을 줍니다.
 */
void setup() {
  Serial.begin(9600);                       
  Serial.println("........Start........");  

  // 센서 핀을 입력으로 설정합니다.
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);

  // 모터 쉴드를 초기화합니다.
  dcMotor.begin();
  dcMotor.setFrequency(1600);
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
    setMotorsDirection(C_FORWARD, MAX_SPEED); // 모터를 전진 방향으로 설정하고 최대 속도로 이동합니다.
  }
  // 좌회전: 왼쪽 센서만 검은색을 감지하고, 오른쪽 센서는 흰색 바닥을 감지할 때
  // 왼쪽 센서가 LOW, 오른쪽 센서가 HIGH 신호를 출력합니다.
  else if (digitalRead(leftSensor) == LOW && digitalRead(rightSensor) == HIGH) {
    setMotorsDirection(C_LEFT, MAX_SPEED); // 모터를 좌회전 방향으로 설정하고 최대 속도로 회전합니다.
  }
  // 우회전: 오른쪽 센서만 검은색을 감지하고, 왼쪽 센서는 흰색 바닥을 감지할 때
  // 오른쪽 센서가 LOW, 왼쪽 센서가 HIGH 신호를 출력합니다.
  else if (digitalRead(leftSensor) == HIGH && digitalRead(rightSensor) == LOW) {
    setMotorsDirection(C_RIGHT, MAX_SPEED); // 모터를 우회전 방향으로 설정하고 최대 속도로 회전합니다.
  }
  // 정지: 두 센서 모두 흰색 바닥을 감지할 때
  // 두 센서 모두 HIGH 신호를 출력합니다.
  else {
    stopAllMotors(); // 모든 모터를 정지시킵니다.
  }
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