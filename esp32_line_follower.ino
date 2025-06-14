/**
 * @file esp32_line_follower_auto_bluetooth.ino
 * @author Modificado para controle automático via Bluetooth
 * @brief Firmware para robô seguidor de linha ESP32 com controle automático Bluetooth
 * @version 3.1 - Auto modo baseado em conexão Bluetooth
 * @date 2024-06-13
 *
 * @details
 * NOVA FUNCIONALIDADE:
 * - Ativa seguidor de linha automaticamente quando Bluetooth desconectado
 * - Desativa seguidor quando Bluetooth conecta
 * - Mantém todas as funcionalidades anteriores
 * - Indicação visual via LED do status da conexão
 */

#include <QTRSensors.h>
#include "BluetoothSerial.h"

// Bluetooth check
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

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

// QTR Reflectance Sensors - CORRIGIDO PARA 4 SENSORES
const uint8_t SENSOR_COUNT = 4;
// Removido GPIO 32 (sensor 2 defeituoso)
// Sensor 1 -> GPIO 34 (índice 0)
// Sensor 3 -> GPIO 18 (índice 1) 
// Sensor 4 -> GPIO 19 (índice 2)
// Sensor 5 -> GPIO 21 (índice 3)
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {34, 18, 19, 21}; 

// =================== GLOBAL OBJECTS & VARIABLES ===================
QTRSensors qtr;
BluetoothSerial SerialBT;

// Robot State Management
enum RobotState { IDLE, MANUAL_CONTROL, LINE_FOLLOWING_ACTIVE };
RobotState currentState = IDLE;

// NOVA VARIÁVEL: Controle de conexão Bluetooth
bool bluetoothConnected = false;
bool lastBluetoothState = false;
unsigned long lastConnectionCheck = 0;
const unsigned long CONNECTION_CHECK_INTERVAL = 1000; // Verificar a cada 1 segundo

// Line Following & PID Control - VALORES CORRIGIDOS
float Kp = 0.8;        // Reduzido para evitar oscilação
float Ki = 0.0;        // Começar sem integral
float Kd = 0.15;       // Aumentado para melhor estabilidade

uint8_t Kp_multiplier = 1;
uint8_t Ki_multiplier = 1;
uint8_t Kd_multiplier = 1;

int lastError = 0;
float integral = 0;
const float INTEGRAL_LIMIT = 2000; // Reduzido para 4 sensores

int baseSpeed = 178;   // 70% da potência total 
const int SEARCH_SPEED = 143; // 70% da potência para procurar linha

// Manual Control
char manualCommand = 'S';
int manualSpeed = 200;
const int MAX_SPEED = 255;

// PID Tuning via Bluetooth
int bt_value, bt_cmd_counter = 0, bt_cmd_buffer[3];

// Controle de linha perdida
unsigned long lineLastSeen = 0;
const unsigned long LINE_LOST_TIMEOUT = 1000; // 1 segundo
int lastValidPosition = 1500; // Centro para 4 sensores

// =================== FUNCTION PROTOTYPES ===================
void setup();
void loop();
void checkBluetoothConnection();
void handleBluetoothConnectionChange();
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
void printSensorDebug(uint16_t* sensors, uint16_t position);

// =================== SETUP ===================
void setup() {
  Serial.begin(115200);
  Serial.println("=== ESP32 Line Follower - Auto Bluetooth Mode ===");

  // Configure motor driver pins
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);
  pinMode(MOTOR_A_EN, OUTPUT);
  pinMode(MOTOR_B_EN, OUTPUT);

  // CORREÇÃO CRÍTICA: Configure QTR sensors como RC (não Analog)
  qtr.setTypeRC();
  qtr.setSensorPins(SENSOR_PINS, SENSOR_COUNT);
  qtr.setEmitterPin(2); // LED IR no GPIO 2

  // Start Bluetooth Serial
  SerialBT.begin("ESP32_LineFollower_4S");
  Serial.println("Bluetooth iniciado: ESP32_LineFollower_4S");

  // Configure LED for Bluetooth status indication
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  
  Serial.println("=== CALIBRAÇÃO DOS SENSORES ===");
  Serial.println("Mova o robô sobre a linha por 10 segundos...");
  Serial.println("3... 2... 1... COMEÇOU!");

  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
    
    // Mostrar progresso
    if (i % 50 == 0) {
      Serial.print("Progresso: ");
      Serial.print((i * 100) / 400);
      Serial.println("%");
    }
    delay(25);
  }

  digitalWrite(LED_BUILTIN, LOW);
  Serial.println("Calibração completa!");

  // Print calibration data for debugging
  Serial.println("=== RESULTADOS DA CALIBRAÇÃO ===");
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" (GPIO ");
    Serial.print(SENSOR_PINS[i]);
    Serial.print(") min: ");
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(" max: ");
    Serial.print(qtr.calibrationOn.maximum[i]);
    
    // Verificar se calibração foi bem-sucedida
    int range = qtr.calibrationOn.maximum[i] - qtr.calibrationOn.minimum[i];
    if (range < 200) {
      Serial.print(" ⚠️ AVISO: Range baixo!");
    } else {
      Serial.print(" ✅ OK");
    }
    Serial.println();
  }
  Serial.println("=====================================");

  // Start in IDLE state
  stopMotors();
  currentState = IDLE;
  lineLastSeen = millis();
  
  // Inicializar estado do Bluetooth
  bluetoothConnected = SerialBT.hasClient();
  lastBluetoothState = bluetoothConnected;
  
  Serial.println("🤖 Robô pronto!");
  Serial.println("🔄 MODO AUTOMÁTICO ATIVADO:");
  Serial.println("   • Sem Bluetooth = Seguidor de linha ON");
  Serial.println("   • Com Bluetooth = Controle manual disponível");
  Serial.println("Comandos Bluetooth: L=Toggle Linha, F/B/R/L=Mover, S=Parar");
  Serial.println("Valores PID atuais:");
  Serial.println("Kp=" + String(Kp) + " Ki=" + String(Ki) + " Kd=" + String(Kd));
  
  // Verificar estado inicial e ativar modo apropriado
  handleBluetoothConnectionChange();
}

// =================== MAIN LOOP ===================
void loop() {
  // Verificar conexão Bluetooth periodicamente
  checkBluetoothConnection();
  
  // Processar comandos Bluetooth se conectado
  if (bluetoothConnected) {
    handleBluetooth();
  }

  // Executar ação baseada no estado atual
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

  delay(10);
}

// =================== BLUETOOTH CONNECTION MANAGEMENT ===================
void checkBluetoothConnection() {
  if (millis() - lastConnectionCheck > CONNECTION_CHECK_INTERVAL) {
    bluetoothConnected = SerialBT.hasClient();
    
    // Verificar se houve mudança no status de conexão
    if (bluetoothConnected != lastBluetoothState) {
      handleBluetoothConnectionChange();
      lastBluetoothState = bluetoothConnected;
    }
    
    lastConnectionCheck = millis();
  }
}

void handleBluetoothConnectionChange() {
  if (bluetoothConnected) {
    // Bluetooth conectado - parar seguidor automático
    Serial.println("📱 BLUETOOTH CONECTADO!");
    Serial.println("🛑 Seguidor automático DESATIVADO");
    Serial.println("🎮 Controle manual disponível");
    
    currentState = IDLE;
    stopMotors();
    digitalWrite(LED_BUILTIN, HIGH); // LED aceso = Bluetooth conectado
    
    SerialBT.println("=== CONECTADO ===");
    SerialBT.println("Seguidor AUTO OFF");
    SerialBT.println("Comandos: L=Toggle, F/B/R/L=Mover, S=Parar");
    
  } else {
    // Bluetooth desconectado - ativar seguidor automático
    Serial.println("📱 BLUETOOTH DESCONECTADO!");
    Serial.println("🚀 Seguidor automático ATIVADO");
    
    currentState = LINE_FOLLOWING_ACTIVE;
    lastError = 0;
    integral = 0;
    lineLastSeen = millis();
    lastValidPosition = 1500;
    digitalWrite(LED_BUILTIN, LOW); // LED apagado = Bluetooth desconectado
  }
}

// =================== BLUETOOTH HANDLING ===================
void handleBluetooth() {
  if (SerialBT.available()) {
    char command = SerialBT.read();
    Serial.print("Comando recebido: ");
    Serial.println(command);

    if (command == 'L') { 
      if (currentState == LINE_FOLLOWING_ACTIVE) {
        currentState = IDLE;
        Serial.println("🛑 Seguidor de linha PARADO (manual)");
        SerialBT.println("Seguidor OFF");
      } else {
        currentState = LINE_FOLLOWING_ACTIVE;
        lastError = 0;
        integral = 0;
        lineLastSeen = millis();
        lastValidPosition = 1500; // Centro para 4 sensores
        Serial.println("🚀 Seguidor de linha ATIVADO (manual)");
        SerialBT.println("Seguidor ON");
      }
      return;
    }

    if (strchr("FBLRGIJHS", command)) { 
      if (currentState != MANUAL_CONTROL) {
        Serial.println("🎮 Modo MANUAL ativado");
        SerialBT.println("Controle Manual ON");
      }
      currentState = MANUAL_CONTROL;
      manualCommand = command;
      return;
    }
    
    if (currentState == MANUAL_CONTROL && strchr("0123456789q", command)) {
      adjustSpeedFromBT(command);
      return;
    }

    if (currentState == LINE_FOLLOWING_ACTIVE) {
      bt_value = command;
      bt_cmd_counter++;
      bt_cmd_buffer[bt_cmd_counter] = bt_value;
      if (bt_cmd_counter == 2) {
        bt_cmd_counter = 0;
        processPidTuningCommand();
      }
      return;
    }
  }
}

// =================== PID TUNING ===================
void processPidTuningCommand() {
  int param_id = bt_cmd_buffer[1];
  int param_val = bt_cmd_buffer[2];

  switch (param_id) {
    case 1: 
      Kp = param_val; 
      Serial.println("Kp = " + String(Kp));
      SerialBT.println("Kp=" + String(Kp));
      break;
    case 2: 
      Kp_multiplier = param_val; 
      Serial.println("Kp_mult = " + String(Kp_multiplier));
      break;
    case 3: 
      Ki = param_val; 
      Serial.println("Ki = " + String(Ki));
      SerialBT.println("Ki=" + String(Ki));
      break;
    case 4: 
      Ki_multiplier = param_val; 
      Serial.println("Ki_mult = " + String(Ki_multiplier));
      break;
    case 5: 
      Kd = param_val; 
      Serial.println("Kd = " + String(Kd));
      SerialBT.println("Kd=" + String(Kd));
      break;
    case 6: 
      Kd_multiplier = param_val; 
      Serial.println("Kd_mult = " + String(Kd_multiplier));
      break;
    default: 
      Serial.println("❌ ID de parâmetro PID inválido");
      break;
  }
}

// =================== LINE FOLLOWING CONTROLLER - CORRIGIDO ===================
void lineFollowController() {
  uint16_t sensorValues[SENSOR_COUNT];
  
  // CORREÇÃO: Para 4 sensores, posição vai de 0 a 3000
  uint16_t position = qtr.readLineBlack(sensorValues);
  
  // Debug dos sensores (descomente para diagnóstico)
  // printSensorDebug(sensorValues, position);
  
  // Verificar se a linha está sendo vista
  bool lineDetected = false;
  int maxSensorValue = 0;
  
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (sensorValues[i] > maxSensorValue) {
      maxSensorValue = sensorValues[i];
    }
    if (sensorValues[i] > 300) { // Limiar para detectar linha
      lineDetected = true;
    }
  }
  
  if (lineDetected) {
    lineLastSeen = millis();
    lastValidPosition = position;
    
    // PID calculation - CORRIGIDO para 4 sensores
    int error = position - 1500; // Centro é 1500 para 4 sensores (3000/2)

    float proportional = error;
    integral = integral + error;
    float derivative = error - lastError;
    lastError = error;

    // Anti-windup
    integral = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    // Aplicar multiplicadores PID
    float p_term = (Kp / pow(10, Kp_multiplier)) * proportional;
    float i_term = (Ki / pow(10, Ki_multiplier)) * integral;
    float d_term = (Kd / pow(10, Kd_multiplier)) * derivative;

    float pidValue = p_term + i_term + d_term;

    int leftSpeed = baseSpeed + pidValue;
    int rightSpeed = baseSpeed - pidValue;

    // Constrain motor speeds
    leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
    rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

    motorDrive(leftSpeed, rightSpeed);
    
    // Debug PID (descomente se necessário)
    /*
    Serial.print("Pos: "); Serial.print(position);
    Serial.print(" Err: "); Serial.print(error);
    Serial.print(" PID: "); Serial.print(pidValue);
    Serial.print(" L: "); Serial.print(leftSpeed);
    Serial.print(" R: "); Serial.println(rightSpeed);
    */
    
  } else {
    // LINHA PERDIDA - Algoritmo melhorado
    unsigned long timeLost = millis() - lineLastSeen;
    
    if (timeLost < LINE_LOST_TIMEOUT) {
      // Procurar linha baseado na última posição conhecida
      if (lastValidPosition > 1500) {
        // Linha estava à direita, virar à direita
        motorDrive(SEARCH_SPEED, -SEARCH_SPEED);
        Serial.println("🔍 Procurando linha à DIREITA");
      } else {
        // Linha estava à esquerda, virar à esquerda  
        motorDrive(-SEARCH_SPEED, SEARCH_SPEED);
        Serial.println("🔍 Procurando linha à ESQUERDA");
      }
    } else {
      // Linha perdida há muito tempo, parar
      stopMotors();
      Serial.println("❌ LINHA PERDIDA! Parando...");
      
      // Se Bluetooth conectado, notificar
      if (bluetoothConnected) {
        SerialBT.println("Linha perdida!");
      }
      
      // Opcional: voltar para modo IDLE
      // currentState = IDLE;
    }
  }
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
  Serial.println("Velocidade manual: " + String(manualSpeed));
  SerialBT.println("Speed: " + String(manualSpeed));
}

// =================== MOTOR FUNCTIONS ===================
void motorDrive(int leftSpeed, int rightSpeed) {
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Left Motor
  if (leftSpeed > 0) {
    digitalWrite(MOTOR_A_IN1, HIGH);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_EN, leftSpeed);
  } else if (leftSpeed < 0) {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, HIGH);
    analogWrite(MOTOR_A_EN, -leftSpeed);
  } else {
    digitalWrite(MOTOR_A_IN1, LOW);
    digitalWrite(MOTOR_A_IN2, LOW);
    analogWrite(MOTOR_A_EN, 0);
  }

  // Right Motor
  if (rightSpeed > 0) {
    digitalWrite(MOTOR_B_IN1, HIGH);
    digitalWrite(MOTOR_B_IN2, LOW);
    analogWrite(MOTOR_B_EN, rightSpeed);
  } else if (rightSpeed < 0) {
    digitalWrite(MOTOR_B_IN1, LOW);
    digitalWrite(MOTOR_B_IN2, HIGH);
    analogWrite(MOTOR_B_EN, -rightSpeed);
  } else {
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

// =================== DEBUG FUNCTION ===================
void printSensorDebug(uint16_t* sensors, uint16_t position) {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 500) { // Print a cada 500ms
    Serial.print("Sensores: ");
    for (int i = 0; i < SENSOR_COUNT; i++) {
      Serial.print(sensors[i]);
      Serial.print("\t");
    }
    Serial.print("Posição: ");
    Serial.println(position);
    lastPrint = millis();
  }
}
