#include <ESP32Servo.h>

// TB6612FNG Motor Driver pins
#define STBY_PIN 33
#define AIN1_PIN 25
#define AIN2_PIN 26
#define PWMA_PIN 27
#define BIN1_PIN 14
#define BIN2_PIN 13
#define PWMB_PIN 12

// IR sensor pins
#define LEFT_IR_PIN 36
#define MIDDLE_IR_PIN 39
#define RIGHT_IR_PIN 34

// Ultrasonic sensor pins
#define TRIG_PIN 23
#define ECHO_PIN 19

// Servo pin
#define SERVO_PIN 18
Servo scanServo;

// Constants
#define OBSTACLE_DISTANCE 25  // cm
#define LINE_THRESHOLD 500   // Adjust after testing
#define BASE_SPEED 10
#define MAX_SPEED 20
#define SCAN_DELAY 600
#define REVERSE_DURATION 400

// PID constants
float Kp = 30.0;
float Kd = 20.0;
float Ki = 0.0;

float lastError = 0;
float integral = 0;

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(STBY_PIN, OUTPUT);
  pinMode(AIN1_PIN, OUTPUT);
  pinMode(AIN2_PIN, OUTPUT);
  pinMode(PWMA_PIN, OUTPUT);
  pinMode(BIN1_PIN, OUTPUT);
  pinMode(BIN2_PIN, OUTPUT);
  pinMode(PWMB_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  // Sensor pins
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(MIDDLE_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo
  scanServo.attach(SERVO_PIN);
  scanServo.write(90);  // Center
  delay(300);

  Serial.println("PID Line Following Robot Initialized.");
}

void loop() {
  if (isObstacleAhead()) {
    avoidObstacle();
  } else {
    followLinePID();
  }
}

// ------------------- PID Line Following ---------------------
void followLinePID() {
  int left = analogRead(LEFT_IR_PIN);
  int middle = analogRead(MIDDLE_IR_PIN);
  int right = analogRead(RIGHT_IR_PIN);

  // Convert sensor readings to binary line detection
  bool l = (left < LINE_THRESHOLD);
  bool m = (middle < LINE_THRESHOLD);
  bool r = (right < LINE_THRESHOLD);

  Serial.print("IR => L: "); Serial.print(left);
  Serial.print(" M: "); Serial.print(middle);
  Serial.print(" R: "); Serial.println(right);

  // Line lost? Try to reacquire
  if (!l && !m && !r) {
    stopMotors();
    searchForLine();
    return;
  }

  // Position error (-1 to 1)
  float error = 0;
  if (l) error -= 1;
  if (r) error += 1;

  // PID terms
  float derivative = error - lastError;
  integral += error;

  float correction = Kp * error + Kd * derivative + Ki * integral;
  lastError = error;

  // Motor speed adjustments
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed = constrain(leftSpeed, 0, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, 0, MAX_SPEED);

  moveMotors(leftSpeed, rightSpeed);
}

// ------------------- Reacquire Line ---------------------
void searchForLine() {
  Serial.println("✘ Line lost. Searching...");

  // Rotate left slowly
  for (int i = 0; i < 40; i++) {
    turnLeft();
    delay(50);
    if (lineDetected()) return;
  }

  // Rotate right slowly
  for (int i = 0; i < 80; i++) {
    turnRight();
    delay(50);
    if (lineDetected()) return;
  }

  // Rotate left again if still not found
  for (int i = 0; i < 40; i++) {
    turnLeft();
    delay(50);
    if (lineDetected()) return;
  }

  stopMotors();
  Serial.println("⚠ Line not found after search.");
}

bool lineDetected() {
  int l = analogRead(LEFT_IR_PIN);
  int m = analogRead(MIDDLE_IR_PIN);
  int r = analogRead(RIGHT_IR_PIN);

  return (l < LINE_THRESHOLD || m < LINE_THRESHOLD || r < LINE_THRESHOLD);
}

// ------------------- Motor Control ---------------------
void moveMotors(int leftSpeed, int rightSpeed) {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, LOW);
  analogWrite(PWMA_PIN, leftSpeed);

  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, LOW);
  analogWrite(PWMB_PIN, rightSpeed);
}

void turnLeft() {
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, 120);

  digitalWrite(BIN1_PIN, HIGH);
  digitalWrite(BIN2_PIN, LOW);
  analogWrite(PWMB_PIN, 120);
}

void turnRight() {
  digitalWrite(AIN1_PIN, HIGH);
  digitalWrite(AIN2_PIN, LOW);
  analogWrite(PWMA_PIN, 120);

  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, HIGH);
  analogWrite(PWMB_PIN, 120);
}

void stopMotors() {
  analogWrite(PWMA_PIN, 0);
  analogWrite(PWMB_PIN, 0);
}

// ------------------- Obstacle Avoidance ---------------------
bool isObstacleAhead() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  float distance = (duration / 2.0) * 0.0343;

  Serial.print("Ultrasonic Distance: ");
  Serial.println(distance);

  return (distance > 0 && distance < OBSTACLE_DISTANCE);
}

void avoidObstacle() {
  Serial.println("⚠ Obstacle Detected!");

  stopMotors();
  delay(300);

  // Reverse briefly
  digitalWrite(AIN1_PIN, LOW);
  digitalWrite(AIN2_PIN, HIGH);
  digitalWrite(BIN1_PIN, LOW);
  digitalWrite(BIN2_PIN, HIGH);
  analogWrite(PWMA_PIN, BASE_SPEED);
  analogWrite(PWMB_PIN, BASE_SPEED);
  delay(REVERSE_DURATION);
  stopMotors();
  delay(200);

  // Scan with servo
  scanServo.write(45);
  delay(SCAN_DELAY);
  float leftDist = readDistance();

  scanServo.write(135);
  delay(SCAN_DELAY);
  float rightDist = readDistance();

  scanServo.write(90);
  delay(300);

  // Decide direction
  if (leftDist > rightDist) {
    Serial.println("↶ Turning Left to Avoid");
    turnLeft();
    delay(500);
  } else {
    Serial.println("↷ Turning Right to Avoid");
    turnRight();
    delay(500);
  }

  moveMotors(BASE_SPEED, BASE_SPEED);
  delay(600);

  // Resume line reacquisition
  searchForLine();
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  return (duration / 2.0) * 0.0343;
}