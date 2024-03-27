#include <PS4Controller.h>
#include <ESP32Servo.h>

#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

// Define servo movement delays for each servo
#define WRIST_ROLL_DELAY 5
#define WRIST_PITCH_DELAY 5
#define SHOULDER_DELAY 10
#define ELBOW_DELAY 15

// Waist servo
Servo myServo1;
int servoPin1 = 2;

// Structure to hold servo information
struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;
  unsigned long lastMoveTime;  // Time of the last servo movement
};

// Create instances for each servo
ServoPins wristRoll = { Servo(), 15 , "Wrist roll", 90, 0 }; // Wrist roll on pin 15
ServoPins wristPitch = { Servo(), 4 , "Wrist pitch", 90, 0 }; // Wrist pitch on pin 4 
ServoPins shoulder = { Servo(), 13 , "Shoulder", 90, 0 }; // Shoulder servo on pin 13
ServoPins elbow = { Servo(), 12 , "Elbow", 90, 0 };       // Elbow servo on pin 12

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

// Function to write new servo position based on step size and current position
void writeServoValues(ServoPins& servo, int servoMoveStepSize, int servoMovementDelay, bool servoStepSizeIsActualServoPosition = false)
{
  unsigned long currentTime = millis();

  // Check if enough time has passed since the last movement
  if (currentTime - servo.lastMoveTime >= servoMovementDelay)
  {
    int servoPosition;
    if (servoStepSizeIsActualServoPosition)
    {
      servoPosition = servoMoveStepSize; 
    }
    else
    {
      servoPosition = servo.servo.read();
      servoPosition = servoPosition + servoMoveStepSize;
    }

    // Check the bounds for servo position
    if (servoPosition > 180 || servoPosition < 0)
    {
      return;
    }

    // Update the servo position and record the time of movement
    servo.servo.write(servoPosition);
    servo.lastMoveTime = currentTime;
  }
}

void notify() {
  // Initial value of motors is 0
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

  // Control for wrist roll
  if (PS4.L2())
  {
    writeServoValues(wristRoll, SERVO_BACKWARD_STEP_ANGLE, WRIST_ROLL_DELAY);
  }
  else if (PS4.R2())
  {
    writeServoValues(wristRoll, SERVO_FORWARD_STEP_ANGLE, WRIST_ROLL_DELAY);
  }

  // Control for wrist pitch
  if (PS4.Triangle())
  {
    writeServoValues(wristPitch, SERVO_BACKWARD_STEP_ANGLE, WRIST_PITCH_DELAY);
  }
  else if (PS4.Circle())
  {
    writeServoValues(wristPitch, SERVO_FORWARD_STEP_ANGLE, WRIST_PITCH_DELAY);
  }
  
  // Control for waist
  int servoSpeed1 = PS4.Right() ? 120 : (PS4.Left() ? 70 : 90);
  myServo1.write(servoSpeed1);

  // Control for shoulder
  if (PS4.Up())
  {
    writeServoValues(shoulder, SERVO_FORWARD_STEP_ANGLE, SHOULDER_DELAY);
  }
  else if (PS4.Down())
  {
    writeServoValues(shoulder, SERVO_BACKWARD_STEP_ANGLE, SHOULDER_DELAY);
  }

  // Control for elbow
  if (PS4.L1())
  {
    writeServoValues(elbow, SERVO_FORWARD_STEP_ANGLE, ELBOW_DELAY);
  }
  else if (PS4.R1())
  {
    writeServoValues(elbow, SERVO_BACKWARD_STEP_ANGLE, ELBOW_DELAY);
  }
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

  // Set up wrist roll
  wristRoll.servo.attach(wristRoll.servoPin);
  wristRoll.servo.write(wristRoll.initialPosition);

  // Set up wrist pitch
  wristPitch.servo.attach(wristPitch.servoPin);
  wristPitch.servo.write(wristPitch.initialPosition);   

  // Set up shoulder
  shoulder.servo.attach(shoulder.servoPin);
  shoulder.servo.write(shoulder.initialPosition);

  // Set up elbow
  elbow.servo.attach(elbow.servoPin);
  elbow.servo.write(elbow.initialPosition);
}

void setup() {
  setUpPinModes();
  myServo1.attach(servoPin1); // Waist servo
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");
}

void loop() {
}