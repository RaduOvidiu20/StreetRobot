#include <Arduino.h>

// Define pins for left motor
#define CCPWM_LEFT 12
#define CCWWM_LEFT 15

// Define encoder pins for left motor
#define ENCODER_A_LEFT 5  // GPIO34 for ENCODER_A (input only)
#define ENCODER_B_LEFT 8  // GPIO35 for ENCODER_B (input only)

// Define pins for right motor
#define CCPWM_RIGHT 32
#define CCWPWM_RIGHT 33

// Define encoder pins for right motor
#define ENCODER_A_RIGHT 19
#define ENCODER_B_RIGHT 20

// PWM settings
#define PWM_FREQ 10000
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)

// PWM channels
#define PWM_CHANNEL_CCPWM_LEFT 0
#define PWM_CHANNEL_CCWWM_LEFT  1
#define PWM_CHANNEL_CCPWM_RIGHT 2
#define PWM_CHANNEL_CCWPWM_RIGHT 3

// Variables to store encoder positions
volatile long encoderPositionLeft = 0;
volatile long encoderPositionRight = 0;

// Variables to store the last state of encoder pins
volatile int lastEncodedLeft = 0;
volatile int lastEncodedRight = 0;

// Variables to store the current speed commands
int currentSpeedLeft = 0;
int currentSpeedRight = 0;

void IRAM_ATTR updateEncoderLeft() {
  int MSB = digitalRead(ENCODER_A_LEFT);  // Most significant bit
  int LSB = digitalRead(ENCODER_B_LEFT);  // Least significant bit

  int encoded = (MSB << 1) | LSB;           // Convert the 2 bits to a single value
  int sum = (lastEncodedLeft << 2) | encoded;  // Combine previous and current

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPositionLeft++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPositionLeft--;

  lastEncodedLeft = encoded;  // Store this value for next time
}

void IRAM_ATTR updateEncoderRight() {
  int MSB = digitalRead(ENCODER_A_RIGHT);  // Most significant bit
  int LSB = digitalRead(ENCODER_B_RIGHT);  // Least significant bit

  int encoded = (MSB << 1) | LSB;           // Convert the 2 bits to a single value
  int sum = (lastEncodedRight << 2) | encoded;  // Combine previous and current

  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPositionRight++;
  if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPositionRight--;

  lastEncodedRight = encoded;  // Store this value for next time
}

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  Serial.println("Program started. Enter commands to control motors in format 'speedLeft,speedRight'.");

  // Setup PWM channels for left motor
  ledcSetup(PWM_CHANNEL_CCPWM_LEFT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_CCWWM_LEFT, PWM_FREQ, PWM_RESOLUTION);

  // Setup PWM channels for right motor
  ledcSetup(PWM_CHANNEL_CCPWM_RIGHT, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_CCWPWM_RIGHT, PWM_FREQ, PWM_RESOLUTION);

  // Attach the channels to the GPIO pins
  ledcAttachPin(CCPWM_LEFT, PWM_CHANNEL_CCPWM_LEFT);
  ledcAttachPin(CCWWM_LEFT, PWM_CHANNEL_CCWWM_LEFT);
  ledcAttachPin(CCPWM_RIGHT, PWM_CHANNEL_CCPWM_RIGHT);
  ledcAttachPin(CCWPWM_RIGHT, PWM_CHANNEL_CCWPWM_RIGHT);

  // Initialize encoder pins as inputs
  pinMode(ENCODER_A_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_B_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);

  // Read the initial state of encoder pins
  int encoder_A_Left = digitalRead(ENCODER_A_LEFT);
  int encoder_B_Left = digitalRead(ENCODER_B_LEFT);
  lastEncodedLeft = (encoder_A_Left << 1) | encoder_B_Left;

  int encoder_A_Right = digitalRead(ENCODER_A_RIGHT);
  int encoder_B_Right = digitalRead(ENCODER_B_RIGHT);
  lastEncodedRight = (encoder_A_Right << 1) | encoder_B_Right;

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_LEFT), updateEncoderLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), updateEncoderRight, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B_RIGHT), updateEncoderRight, CHANGE);
}

// Function to set motor speed and direction
void setMotorSpeed(const char* motor, int speed) {
  int speed_value = abs(speed);
  if (speed_value > 255)
    speed_value = 255;

  int rpwm_channel, lpwm_channel;

  if (strcmp(motor, "left") == 0) {
    rpwm_channel = PWM_CHANNEL_CCPWM_LEFT;
    lpwm_channel = PWM_CHANNEL_CCWWM_LEFT;
    currentSpeedLeft = speed;
  } else if (strcmp(motor, "right") == 0) {
    rpwm_channel = PWM_CHANNEL_CCPWM_RIGHT;
    lpwm_channel = PWM_CHANNEL_CCWPWM_RIGHT;
    currentSpeedRight = speed;
  } else {
    // Invalid motor name
    Serial.println("Invalid motor name.");
    return;
  }

  if (speed > 0) {
    // Forward
    ledcWrite(rpwm_channel, speed_value);
    ledcWrite(lpwm_channel, 0);
  } else if (speed < 0) {
    // Backward
    ledcWrite(rpwm_channel, 0);
    ledcWrite(lpwm_channel, speed_value);
  } else {
    // Stop
    ledcWrite(rpwm_channel, 0);
    ledcWrite(lpwm_channel, 0);
  }
}

// Function to parse and execute commands from Serial
void parseCommand(String command) {
  // Stop motors if command is '0,0'
  if (command == "0,0") {
    setMotorSpeed("left", 0);
    setMotorSpeed("right", 0);
    currentSpeedLeft = 0;
    currentSpeedRight = 0;
    Serial.println("Motors stopped.");
    return;
  }

  // Expected command format: "speedLeft,speedRight"
  command.trim();  // Remove any leading/trailing whitespace

  int commaIndex = command.indexOf(',');
  if (commaIndex == -1) {
    Serial.println("Invalid command format. Use 'speedLeft,speedRight'.");
    return;
  }

  String speedLeftStr = command.substring(0, commaIndex);
  String speedRightStr = command.substring(commaIndex + 1);

  speedLeftStr.trim();
  speedRightStr.trim();

  int speedLeft = speedLeftStr.toInt();
  int speedRight = speedRightStr.toInt();

  if (speedLeft < -255 || speedLeft > 255 || speedRight < -255 || speedRight > 255) {
    Serial.println("Speeds must be between -255 and 255.");
    return;
  }

  setMotorSpeed("left", speedLeft);
  setMotorSpeed("right", speedRight);

  Serial.print("Left motor set to speed ");
  Serial.print(speedLeft);
  Serial.print(", Right motor set to speed ");
  Serial.println(speedRight);
}

// The loop routine runs over and over again forever.
void loop() {
  // Check if there is serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();  // Remove any leading/trailing whitespace
    if (input.length() > 0) {
      parseCommand(input);
    }
  }

  // Display encoder positions every 200 ms
  static unsigned long lastDisplayTime = 0;
  if (millis() - lastDisplayTime >= 200) {
    lastDisplayTime = millis();
    Serial.print("Left encoder position: ");
    Serial.print(encoderPositionLeft);
    Serial.print(", Right encoder position: ");
    Serial.println(encoderPositionRight);
  }
}
