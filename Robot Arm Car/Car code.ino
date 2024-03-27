#include <PS4Controller.h>
#include <ESP32Servo.h>

// Define motor pins and PWM settings for four motors
int enableMotor1 = 22; //up-right motor
int motor1Pin1 = 16;
int motor1Pin2 = 17;

int enableMotor2 = 23; //down-right motor
int motor2Pin1 = 18;
int motor2Pin2 = 19;

int enableMotor3 = 14; //up-left motor
int motor3Pin1 = 26;
int motor3Pin2 = 27;

int enableMotor4 = 32; //down-left motor
int motor4Pin1 = 33;
int motor4Pin2 = 25;

const int PWMFreq = 1000; /* 1 KHz */
const int PWMResolution = 8;
const int motor1PWMSpeedChannel = 4;
const int motor2PWMSpeedChannel = 5;
const int motor3PWMSpeedChannel = 6;
const int motor4PWMSpeedChannel = 7;

void notify() {
  int motor1Speed = 0;
  int motor2Speed = 0;
  int motor3Speed = 0;
  int motor4Speed = 0;

  if (PS4.Cross()) { // Forward movement
    motor1Speed = 90;
    motor2Speed = 90;
    motor3Speed = 90;
    motor4Speed = 90;
  } else if (PS4.Square()) { // Backward movement
    motor1Speed = -90;
    motor2Speed = -90;
    motor3Speed = -90;
    motor4Speed = -90;
  }

  if (PS4.Share()) { // Turning right
    motor1Speed = -90;
    motor2Speed = -90;
    motor3Speed = 90;
    motor4Speed = 90;
  } else if (PS4.Options()) { // Turning left
    motor1Speed = 90;
    motor2Speed = 90;
    motor3Speed = -90;
    motor4Speed = -90;
  }

  rotateMotor(motor1Speed, motor2Speed, motor3Speed, motor4Speed);
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisConnect() {
  rotateMotor(0, 0, 0, 0);
  Serial.println("Disconnected!");
}

void rotateMotor(int speed1, int speed2, int speed3, int speed4) {
  // Motor 1
  if (speed1 < 0) {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
  } else if (speed1 > 0) {
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
  } else {
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
  }

  // Motor 2
  if (speed2 < 0) {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  } else if (speed2 > 0) {
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else {
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  }

  // Motor 3
  if (speed3 < 0) {
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, HIGH);
  } else if (speed3 > 0) {
    digitalWrite(motor3Pin1, HIGH);
    digitalWrite(motor3Pin2, LOW);
  } else {
    digitalWrite(motor3Pin1, LOW);
    digitalWrite(motor3Pin2, LOW);
  }

  // Motor 4
  if (speed4 < 0) {
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, HIGH);
  } else if (speed4 > 0) {
    digitalWrite(motor4Pin1, HIGH);
    digitalWrite(motor4Pin2, LOW);
  } else {
    digitalWrite(motor4Pin1, LOW);
    digitalWrite(motor4Pin2, LOW);
  }

  ledcWrite(motor1PWMSpeedChannel, abs(speed1));
  ledcWrite(motor2PWMSpeedChannel, abs(speed2));
  ledcWrite(motor3PWMSpeedChannel, abs(speed3));
  ledcWrite(motor4PWMSpeedChannel, abs(speed4));
}

void setUpPinModes() {
  pinMode(enableMotor1, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);

  pinMode(enableMotor2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  pinMode(enableMotor3, OUTPUT);
  pinMode(motor3Pin1, OUTPUT);
  pinMode(motor3Pin2, OUTPUT);

  pinMode(enableMotor4, OUTPUT);
  pinMode(motor4Pin1, OUTPUT);
  pinMode(motor4Pin2, OUTPUT);

  // Set up PWM for motor speed
  ledcSetup(motor1PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(motor2PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(motor3PWMSpeedChannel, PWMFreq, PWMResolution);
  ledcSetup(motor4PWMSpeedChannel, PWMFreq, PWMResolution);

  ledcAttachPin(enableMotor1, motor1PWMSpeedChannel);
  ledcAttachPin(enableMotor2, motor2PWMSpeedChannel);
  ledcAttachPin(enableMotor3, motor3PWMSpeedChannel);
  ledcAttachPin(enableMotor4, motor4PWMSpeedChannel);

  rotateMotor(0, 0, 0, 0);
}

void setup() {
  setUpPinModes();
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");
}

void loop() {
}