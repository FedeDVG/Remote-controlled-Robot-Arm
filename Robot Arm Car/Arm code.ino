#include <PS4Controller.h>
#include <ESP32Servo.h>

// Define step angles for servo movement
#define SERVO_FORWARD_STEP_ANGLE 1
#define SERVO_BACKWARD_STEP_ANGLE -1

// Define servo movement delays for each servo
#define WRIST_ROLL_DELAY 5
#define WRIST_PITCH_DELAY 5
#define WAIST_DELAY 10
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
// ServoPins waist = { Servo(), 14 , "Waist", 90, 0 };     // Waist servo on pin 14  
ServoPins shoulder = { Servo(), 13 , "Shoulder", 90, 0 }; // Shoulder servo on pin 13
ServoPins elbow = { Servo(), 12 , "Elbow", 90, 0 };       // Elbow servo on pin 12

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

// Function to handle PS4 controller input and control servos
void notify()
{
  // Control for wrist roll
  if (PS4.R2())
  {
    writeServoValues(wristRoll, SERVO_BACKWARD_STEP_ANGLE, WRIST_ROLL_DELAY);
  }
  else if (PS4.L2())
  {
    writeServoValues(wristRoll, SERVO_FORWARD_STEP_ANGLE, WRIST_ROLL_DELAY);
  }

  // Control for wrist pitch
  if (PS4.R1())
  {
    writeServoValues(wristPitch, SERVO_BACKWARD_STEP_ANGLE, WRIST_PITCH_DELAY);
  }
  else if (PS4.L1())
  {
    writeServoValues(wristPitch, SERVO_FORWARD_STEP_ANGLE, WRIST_PITCH_DELAY);
  }

  // Control for waist
  int servoSpeed1 = PS4.Up() ? 105 : (PS4.Down() ? 80 : 90);
  myServo1.write(servoSpeed1);

  // Control for shoulder
  if (PS4.Left())
  {
    writeServoValues(shoulder, SERVO_FORWARD_STEP_ANGLE, SHOULDER_DELAY);
  }
  else if (PS4.Right())
  {
    writeServoValues(shoulder, SERVO_BACKWARD_STEP_ANGLE, SHOULDER_DELAY);
  }

  // Control for elbow
  if (PS4.Triangle())
  {
    writeServoValues(elbow, SERVO_FORWARD_STEP_ANGLE, ELBOW_DELAY);
  }
  else if (PS4.Cross())
  {
    writeServoValues(elbow, SERVO_BACKWARD_STEP_ANGLE, ELBOW_DELAY);
  }
}

// Function called when the PS4 controller is connected
void onConnect()
{
  Serial.println("Connected!");
}

// Function called when the PS4 controller is disconnected
void onDisConnect()
{
  Serial.println("Disconnected!");
}

// Function to attach servos to their respective pins and set initial positions
void setUpPinModes()
{
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

// Arduino setup function
void setup()
{
  // Set up servo pins and PS4 controller
  setUpPinModes();
  myServo1.attach(servoPin1); // Waist servo
  Serial.begin(115200);
  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin();
  Serial.println("Ready.");
}

// Arduino loop function (empty as the main functionality is handled by callback functions and setup)
void loop()
{
  // Empty loop
}
