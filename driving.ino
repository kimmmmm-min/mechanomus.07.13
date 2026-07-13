/*
  driving_terminal_test.ino

  터미널에서 다음 형식으로 명령 수신:
    s{steering}l{left_speed}r{right_speed}\n

  예:
    s-10l0r0  -> 최대 왼쪽 조향
    s0l0r0    -> 조향 중앙
    s10l0r0   -> 최대 오른쪽 조향
    s0l1r1    -> 직진
    s0l0r0    -> 정지

  핵심 수정:
  - 50ms마다 출력하던 DEBUG 제거
  - 시리얼 송신 버퍼가 가득 차는 문제 방지
  - 포텐셔미터 목표값에 허용 오차 적용
  - map 결과 범위 제한
*/

const unsigned int MAX_INPUT = 32;

// 핀 번호
const int STEERING_1 = 9;
const int STEERING_2 = 10;

const int FORWARD_RIGHT_1 = 5;
const int FORWARD_RIGHT_2 = 6;

const int FORWARD_LEFT_1 = 3;
const int FORWARD_LEFT_2 = 11;

const int POT = A5;

// 조향 PWM
const int STEERING_SPEED = 150;

// 주행 PWM
const int DRIVE_SPEED = 205;

// 포텐셔미터 실제 끝값
const int RESISTANCE_MOST_LEFT = 630;
const int RESISTANCE_MOST_RIGHT = 465;

// 조향 명령 범위
const int MAX_STEERING_STEP = 10;

// 현재값과 목표값 차이가 이 값 이하면 정지
const int STEERING_DEADBAND = 1;

// 제어 주기
const unsigned long CONTROL_INTERVAL = 10;

// 제어 상태
int targetAngle = 0;
int leftSpeed = 0;
int rightSpeed = 0;

unsigned long lastControlTime = 0;


// 함수 선언
void stopSteering();
void steerLeft();
void steerRight();

void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);

void processIncomingByte(byte inByte);
void processData(const char* data);


void setup() {
  Serial.begin(115200);

  pinMode(POT, INPUT);

  pinMode(STEERING_1, OUTPUT);
  pinMode(STEERING_2, OUTPUT);

  pinMode(FORWARD_RIGHT_1, OUTPUT);
  pinMode(FORWARD_RIGHT_2, OUTPUT);

  pinMode(FORWARD_LEFT_1, OUTPUT);
  pinMode(FORWARD_LEFT_2, OUTPUT);

  stopSteering();
  setLeftMotorSpeed(0);
  setRightMotorSpeed(0);

  delay(500);

  // 시작할 때 한 번만 출력
  Serial.println("ARDUINO_READY");
}


void loop() {
  // 시리얼 명령은 항상 먼저 처리
  while (Serial.available() > 0) {
    processIncomingByte((byte)Serial.read());
  }

  unsigned long currentTime = millis();

  if (currentTime - lastControlTime < CONTROL_INTERVAL) {
    return;
  }

  lastControlTime = currentTime;

  // 현재 포텐셔미터 값
  int resistance = analogRead(POT);

  // 왼쪽 끝 = -10, 오른쪽 끝 = +10
  int currentAngle = map(
    resistance,
    RESISTANCE_MOST_LEFT,
    RESISTANCE_MOST_RIGHT,
    -MAX_STEERING_STEP,
    MAX_STEERING_STEP
  );

  currentAngle = constrain(
    currentAngle,
    -MAX_STEERING_STEP,
    MAX_STEERING_STEP
  );

  int error = targetAngle - currentAngle;

  // 목표 부근에서는 조향 모터 정지
  if (abs(error) <= STEERING_DEADBAND) {
    stopSteering();
  }
  else if (error < 0) {
    // 목표가 현재보다 왼쪽
    steerLeft();
  }
  else {
    // 목표가 현재보다 오른쪽
    steerRight();
  }

  setLeftMotorSpeed(leftSpeed);
  setRightMotorSpeed(rightSpeed);
}


// 조향 오른쪽
void steerRight() {
  analogWrite(STEERING_1, STEERING_SPEED);
  analogWrite(STEERING_2, 0);
}


// 조향 왼쪽
void steerLeft() {
  analogWrite(STEERING_1, 0);
  analogWrite(STEERING_2, STEERING_SPEED);
}


// 조향 정지
void stopSteering() {
  analogWrite(STEERING_1, 0);
  analogWrite(STEERING_2, 0);
}


// 왼쪽 주행 모터
void setLeftMotorSpeed(int speed) {
  if (speed > 0) {
    analogWrite(FORWARD_LEFT_1, DRIVE_SPEED);
    analogWrite(FORWARD_LEFT_2, 0);
  }
  else if (speed < 0) {
    analogWrite(FORWARD_LEFT_1, 0);
    analogWrite(FORWARD_LEFT_2, DRIVE_SPEED);
  }
  else {
    analogWrite(FORWARD_LEFT_1, 0);
    analogWrite(FORWARD_LEFT_2, 0);
  }
}


// 오른쪽 주행 모터
void setRightMotorSpeed(int speed) {
  if (speed > 0) {
    analogWrite(FORWARD_RIGHT_1, DRIVE_SPEED);
    analogWrite(FORWARD_RIGHT_2, 0);
  }
  else if (speed < 0) {
    analogWrite(FORWARD_RIGHT_1, 0);
    analogWrite(FORWARD_RIGHT_2, DRIVE_SPEED);
  }
  else {
    analogWrite(FORWARD_RIGHT_1, 0);
    analogWrite(FORWARD_RIGHT_2, 0);
  }
}


// 한 글자씩 명령 수신
void processIncomingByte(byte inByte) {
  static char inputLine[MAX_INPUT];
  static unsigned int inputPos = 0;

  if (inByte == '\r') {
    return;
  }

  if (inByte == '\n') {
    inputLine[inputPos] = '\0';

    if (inputPos > 0) {
      processData(inputLine);
    }

    inputPos = 0;
    return;
  }

  if (inputPos < MAX_INPUT - 1) {
    inputLine[inputPos++] = (char)inByte;
  }
  else {
    inputPos = 0;
  }
}


// s-5l0r0 형식 파싱
void processData(const char* data) {
  const char* sPtr = strchr(data, 's');
  const char* lPtr = strchr(data, 'l');
  const char* rPtr = strchr(data, 'r');

  if (sPtr == NULL || lPtr == NULL || rPtr == NULL) {
    return;
  }

  int newAngle = atoi(sPtr + 1);
  int newLeftSpeed = atoi(lPtr + 1);
  int newRightSpeed = atoi(rPtr + 1);

  targetAngle = constrain(
    newAngle,
    -MAX_STEERING_STEP,
    MAX_STEERING_STEP
  );

  leftSpeed = constrain(newLeftSpeed, -255, 255);
  rightSpeed = constrain(newRightSpeed, -255, 255);

  // 명령을 받을 때만 한 줄 출력
  Serial.print("ACK s");
  Serial.print(targetAngle);
  Serial.print(" l");
  Serial.print(leftSpeed);
  Serial.print(" r");
  Serial.println(rightSpeed);
}

