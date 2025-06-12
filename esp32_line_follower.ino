/**
 * @file esp32_line_follower.ino
 * @author Your Name
 * @brief Firmware for an ESP32-based line-following robot with Bluetooth manual control and PID tuning.
 * @version 2.6 - Corrected Sensor Pinout Order
 * @date 2024-06-11
 *
 * @details
 * This code controls a two-wheeled robot designed to follow a black line on a white surface.
 * It features:
 * - QTR-5-RC reflectance sensor array for line detection.
 * - L298N-style motor driver for differential drive.
 * - Bluetooth serial communication for mode switching, manual control, and live PID tuning.
 * - A PID (Proportional-Integral-Derivative) controller for smooth line following.
 * - An automatic sensor calibration routine on startup.
 *
 * --- Bluetooth Commands (Simplified) ---
 * - 'L': Toggles line-following mode ON/OFF.
 * - 'F', 'B', 'R', 'L', 'G', 'I', 'H', 'J': Movement commands. Automatically switches to Manual Mode.
 * - 'S': Stops the robot (in Manual Mode).
 * - '0'-'9', 'q': Adjust speed (in Manual Mode).
 * - PID Tuning (when in line-following mode):
 * The robot expects two bytes. The first identifies the parameter, the second is the value.
 * 1: Set Kp, 2: Set Kp multiplier, 3: Set Ki, 4: Set Ki multiplier, 5: Set Kd, 6: Set Kd multiplier
 */

#include <QTRSensors.h>
#include "BluetoothSerial.h"

// Bluetooth check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Define the built-in LED pin for ESP32 (usually GPIO 2)
#define LED_BUILTIN 2

// =================== PIN DEFINITIONS ===================
// Motor A (Left)
const int MOTOR_A_IN1 = 27;
const int MOTOR_A_IN2 = 26;
const int MOTOR_A_EN = 14;

// Motor B (Right)
const int MOTOR_B_IN1 = 33;
const int MOTOR_B_IN2 = 25;
const int MOTOR_B_EN = 32;

// QTR Reflectance Sensors
const uint8_t SENSOR_COUNT = 5;
// CRITICAL FIX: Updated with the correct pin order provided by the user.
// Sensor 1 -> PIN 34 (Array index 0)
// Sensor 2 -> PIN 35 (Array index 1)
// Sensor 3 -> PIN 18 (Array index 2)
// Sensor 4 -> PIN 19 (Array index 3)
// Sensor 5 -> PIN 21 (Array index 4)
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {34, 35, 18, 19, 21}; 

// =================== GLOBAL OBJECTS & VARIABLES ===================
QTRSensors qtr;
BluetoothSerial SerialBT;

// Robot State Management
enum RobotState { IDLE, MANUAL_CONTROL, LINE_FOLLOWING_ACTIVE };
RobotState currentState = IDLE;

// Line Following & PID Control
float Kp = 1.0;
float Ki = 0.0;
float Kd = 0.1;

// Multipliers allow adjusting PID constants by powers of 10 via Bluetooth
uint8_t Kp_multiplier = 1;
uint8_t Ki_multiplier = 1;
uint8_t Kd_multiplier = 1;

int lastError = 0;
float integral = 0;
const float INTEGRAL_LIMIT = 4000; // Anti-windup limit

int baseSpeed = 200;

// Manual Control
char manualCommand = 'S'; // Default to Stop
int manualSpeed = 200;
const int MAX_SPEED = 255;

// PID Tuning via Bluetooth
int bt_value, bt_cmd_counter = 0, bt_cmd_buffer[3];

// =================== FUNCTION PROTOTYPES ===================
void setup();
void loop();
void handleBluetooth();
void processPidTuningCommand();
void lineFollowController();
void manualController();
void adjustSpeedFromBT(char cmd);
void motorDrive(int leftSpeed, int rightSpeed);
void moveForward(int speed);
void moveBackward(int speed);
void pivotRight(int speed);
void pivotLeft(int speed);
void moveForwardRight(int speed);
void moveForwardLeft(int speed);
void moveBackwardRight(int speed);
void moveBackwardLeft(int speed);
void stopMotors();

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  Serial.println("Initializing Line Follower Robot...");

  // Configure motor driver pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);

  // Configure QTR sensors
  qtr.setTypeAnalog();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);

  // Start Bluetooth Serial
  SerialBT.begin("ESP32_LineFollower");
  Serial.println("Bluetooth started. Waiting for connection...");

  // Calibrate sensors
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Calibrating sensors... Move robot over the line for 10 seconds.");

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    delay(20);
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibration complete!");

  // Print calibration data for debugging
  Serial.println("--- Sensor Calibration Results ---");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("Sensor ");
    Serial.print(i);
    Serial.print(" (GPIO ");
    Serial.print(SENSOR_PINS[i]);
    Serial.print(") min: ");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" max: ");
    Serial.println(qtr.calibrationOn.maximum[i]);
  }
  Serial.println("--------------------------------");

  // Start in IDLE state
  stopMotors();
  currentState = IDLE;
  Serial.println("\nRobot is ready!");
  Serial.println("Bluetooth Commands: L=Toggle Line Follow, F/B/R/L..=Move, S=Stop");
}

// =================== MAIN LOOP ===================
void loop() {
  // Always listen for new commands
  handleBluetooth();

  // State machine for robot control
  switch (currentState) {
    case LINE_FOLLOWING_ACTIVE:
      lineFollowController();
      break;

    case MANUAL_CONTROL:
      manualController();
      break;

    case IDLE:
    default:
      stopMotors();
      break;
  }

  delay(10); // Small delay for stability
}

// =================== BLUETOOTH HANDLING (SIMPLIFIED) ===================
void handleBluetooth() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("Received command: ");
    Serial.println(command);

    // --- High Priority Commands (Mode Switching & Manual Control) ---

    // 'L' toggles Line Following mode
    if (command == 'L') { 
      if (currentState == LINE_FOLLOWING_ACTIVE) {
        currentState = IDLE; // Go to idle when stopping line follower
        Serial.println("Line Following STOPPED");
        SerialBT.println("Line Following OFF");
      } else {
        currentState = LINE_FOLLOWING_ACTIVE;
        lastError = 0; // Reset PID state
        integral = 0;
        Serial.println("Line Following STARTED");
        SerialBT.println("Line Following ON");
      }
      return; // Command processed
    }

    // Any movement command (including Stop) automatically engages Manual Control mode
    if (strchr("FBLRGIJHS", command)) { 
      // If we weren't in manual mode, announce the change.
      if (currentState != MANUAL_CONTROL) {
        Serial.println("Switched to MANUAL mode");
        SerialBT.println("Manual Control ON");
      }
      currentState = MANUAL_CONTROL;
      manualCommand = command;
      return; // Command processed
    }
    
    // --- Context-Sensitive Commands ---

    // Speed commands are only valid in manual mode
    if (currentState == MANUAL_CONTROL && strchr("0123456789q", command)) {
      adjustSpeedFromBT(command);
      return; // Command processed
    }

    // PID tuning commands are only valid in line-following mode
    if (currentState == LINE_FOLLOWING_ACTIVE) {
      // Process PID tuning commands
      bt_value = command;
      bt_cmd_counter++;
      bt_cmd_buffer[bt_cmd_counter] = bt_value;
      if (bt_cmd_counter == 2) {
        bt_cmd_counter = 0;
        processPidTuningCommand();
      }
      return; // Command processed
    }
  }
}


// =================== PID TUNING PROCESSING ===================
void processPidTuningCommand() {
  int param_id = bt_cmd_buffer[1];
  int param_val = bt_cmd_buffer[2];

  switch (param_id) {
    case 1: Kp = param_val; Serial.print("Set Kp = "); Serial.println(Kp); break;
    case 2: Kp_multiplier = param_val; Serial.print("Set Kp_multiplier = "); Serial.println(Kp_multiplier); break;
    case 3: Ki = param_val; Serial.print("Set Ki = "); Serial.println(Ki); break;
    case 4: Ki_multiplier = param_val; Serial.print("Set Ki_multiplier = "); Serial.println(Ki_multiplier); break;
    case 5: Kd = param_val; Serial.print("Set Kd = "); Serial.println(Kd); break;
    case 6: Kd_multiplier = param_val; Serial.print("Set Kd_multiplier = "); Serial.println(Kd_multiplier); break;
    default: Serial.println("Unknown PID parameter ID"); break;
  }
}

// =================== LINE FOLLOWING CONTROLLER ===================
void lineFollowController() {
  uint16_t sensorValues[SENSOR_COUNT];
  // The line position is returned as a value from 0 to 4000 for 5 sensors.
  uint16_t position = qtr.readLineBlack(sensorValues);

  // More robust line-lost check: The library returns 0 or 4000 when the line is not seen.
  bool lineLost = (position == 0 || position == 4000);

  if (lineLost) {
    // If we lose the line, pivot in the direction of the last known error.
    if (lastError > 0) { // Was last seen to the right (position > 2000)
      motorDrive(baseSpeed, -baseSpeed); // Pivot right to find it
    } else { // Was last seen to the left (position < 2000) or was centered
      motorDrive(-baseSpeed, baseSpeed); // Pivot left to find it
    }
    return;
  }
  
  // PID calculation
  int error = position - 2000;

  float proportional = error;
  integral = integral + error;
  float derivative = error - lastError;
  lastError = error;

  // Anti-windup: constrain the integral term
  integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float p_term = (Kp / pow(10, Kp_multiplier)) * proportional;
  float i_term = (Ki / pow(10, Ki_multiplier)) * integral;
  float d_term = (Kd / pow(10, Kd_multiplier)) * derivative;

  float pidValue = p_term + i_term + d_term;

  int leftSpeed = baseSpeed + pidValue;
  int rightSpeed = baseSpeed - pidValue;

  // Constrain motor speeds to valid PWM range
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  motorDrive(leftSpeed, rightSpeed);
}

// =================== MANUAL CONTROLLER ===================
void manualController() {
  switch (manualCommand) {
    case 'F': moveForward(manualSpeed); break;
    case 'B': moveBackward(manualSpeed); break;
    case 'R': pivotRight(manualSpeed); break;
    case 'L': pivotLeft(manualSpeed); break;
    case 'S': stopMotors(); break;
    case 'I': moveForwardRight(manualSpeed); break;
    case 'G': moveForwardLeft(manualSpeed); break;
    case 'J': moveBackwardRight(manualSpeed); break;
    case 'H': moveBackwardLeft(manualSpeed); break;
  }
}

void adjustSpeedFromBT(char cmd) {
  int newSpeed = manualSpeed;
  switch (cmd) {
    case '0': newSpeed = 0; break;
    case '1': newSpeed = MAX_SPEED * 0.1; break;
    case '2': newSpeed = MAX_SPEED * 0.2; break;
    case '3': newSpeed = MAX_SPEED * 0.3; break;
    case '4': newSpeed = MAX_SPEED * 0.4; break;
    case '5': newSpeed = MAX_SPEED * 0.5; break;
    case '6': newSpeed = MAX_SPEED * 0.6; break;
    case '7': newSpeed = MAX_SPEED * 0.7; break;
    case '8': newSpeed = MAX_SPEED * 0.8; break;
    case '9': newSpeed = MAX_SPEED * 0.9; break;
    case 'q': newSpeed = MAX_SPEED; break;
    default: return;
  }
  manualSpeed = constrain(newSpeed, 0, MAX_SPEED);
  Serial.print("Manual speed set to: ");
  Serial.println(manualSpeed);
  SerialBT.print("Speed: ");
  SerialBT.println(manualSpeed);
}

// =================== MOTOR MOVEMENT FUNCTIONS ===================

/**
 * @brief Controls both motors directly with explicit stop handling.
 * @param leftSpeed Speed for the left motor (-255 to 255).
 * @param rightSpeed Speed for the right motor (-255 to 255).
 */
void motorDrive(int leftSpeed, int rightSpeed) {
  // Constrain inputs to be safe
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // --- Left Motor ---
  if (leftSpeed > 0) { // Forward
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_EN, leftSpeed);
  } else if (leftSpeed < 0) { // Backward
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_EN, -leftSpeed); // Use positive value for speed
  } else { // Stop (coast)
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_EN, 0);
  }

  // --- Right Motor ---
  if (rightSpeed > 0) { // Forward
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_EN, rightSpeed);
  } else if (rightSpeed < 0) { // Backward
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_EN, -rightSpeed); // Use positive value for speed
  } else { // Stop (coast)
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_EN, 0);
  }
}

void moveForward(int speed) { motorDrive(speed, speed); }
void moveBackward(int speed) { motorDrive(-speed, -speed); }
void pivotRight(int speed) { motorDrive(speed, -speed); }
void pivotLeft(int speed) { motorDrive(-speed, speed); }
void stopMotors() { motorDrive(0, 0); }
void moveForwardRight(int speed) { motorDrive(speed, speed / 2); }
void moveForwardLeft(int speed) { motorDrive(speed / 2, speed); }
void moveBackwardRight(int speed) { motorDrive(-speed, -speed / 2); }
void moveBackwardLeft(int speed) { motorDrive(-speed / 2, -speed); }
