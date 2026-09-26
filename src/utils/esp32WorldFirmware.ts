// ESP32 Arduino C++ Firmware Code for Bot Trajectory & Smart Sonar Mapping Rover

export const ESP32_WORLD_DISCOVERER_CODE = `/*
 * ==============================================================================
 * PROYECTO: ESP32 ROVER DE MAPEO Y REGISTRO DE TRAYECTORIA CON SONAR INTELIGENTE
 * ==============================================================================
 * Funcionalidad solicitada:
 *   1. Punto de inicio del servomotor: 90 GRADOS (Frente / 0° relativo).
 *   2. Modo Vigilancia Normal: Sondeo estrecho de 0 a +15° y de 0 a -15° (75° a 105°).
 *      El sensor NO gira los 180° todo el tiempo innecesariamente.
 *   3. Detección de Obstáculo a <= 40 cm:
 *      En el momento en que detecta algo a 40 cm o menos, activa inmediatamente
 *      un barrido panorámico completo de -90° a +90° partiendo del centro (0° a 180° absolutos)
 *      para mapear el obstáculo y encontrar una ruta de escape.
 *   4. Retorno continuo de distancia: Devuelve los cm exactos medidos por el HC-SR04:
 *      - PING:angulo,distancia_cm
 *      - DIST:distancia_cm
 *      - ALERT:OBSTACLE,distancia_cm
 *   5. Control de motores para tracción del bot y telemetría de odometría (POS:x,y,rumbo).
 *
 * Conexión de Pines ESP32:
 *   - Servomotor (PWM):      GPIO 18
 *   - HC-SR04 TRIG:          GPIO 5
 *   - HC-SR04 ECHO:          GPIO 19 (divisor de voltaje 5V a 3.3V recomendado)
 *   - Motor Izquierdo (IN1): GPIO 25 (driver L298N o TB6612)
 *   - Motor Izquierdo (IN2): GPIO 26
 *   - Motor Derecho   (IN3): GPIO 32
 *   - Motor Derecho   (IN4): GPIO 33
 *   - Alimentación Servo:    5V (VIN / fuente externa con GND común)
 *   - Baudrate Serial:       115200 bps
 * ==============================================================================
 */

#include <ESP32Servo.h>

// --- Definición de Pines ---
const int SERVO_PIN = 18;     // Pin PWM de control del servomotor
const int TRIG_PIN  = 5;      // Pin de disparo ultrasónico TRIG
const int ECHO_PIN  = 19;     // Pin de recepción ultrasónico ECHO

// Pines de tracción de motores
const int MOTOR_L_IN1 = 25;
const int MOTOR_L_IN2 = 26;
const int MOTOR_R_IN1 = 32;
const int MOTOR_R_IN2 = 33;

// --- Instancias y Variables de Sonar ---
Servo radarServo;

// Parámetros de barrido inteligente
const int CENTER_ANGLE       = 90;   // Punto de inicio: 90° (0° relativo / Frente)
const int NARROW_SPAN        = 15;   // Rango estrecho: ±15° (de 75° a 105°)
const int PANORAMIC_MIN      = 0;    // -90° relativo
const int PANORAMIC_MAX      = 180;  // +90° relativo
const float OBSTACLE_TRIGGER_CM = 40.0; // Umbral de detección (40 cm)

enum ScanMode {
  NARROW_PATROL,        // Sondeo estrecho 75° a 105°
  OBSTACLE_PANORAMIC    // Barrido 0° a 180° (-90° a +90°) al detectar objeto <= 40cm
};

ScanMode currentScanMode = NARROW_PATROL;
int currentAngle         = CENTER_ANGLE; // Inicia en 90°
int sweepDirection       = 1;            // +1 hacia la izquierda, -1 hacia la derecha
bool isScanningActive    = true;
unsigned long lastStepTime = 0;
unsigned long totalSweepsCompleted = 0;

// Odometría estimada
float posX_cm = 0.0;
float posY_cm = 30.0;
float botHeadingDeg = 90.0; // 90° es Norte (+Y)

// --- Control de Motores ---
void stopMotors() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
}

void driveForward(int speedPwm = 200) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  
  float rad = botHeadingDeg * 0.0174533;
  posX_cm += 4.0 * cos(rad);
  posY_cm += 4.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void driveBackward(int speedPwm = 200) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  
  float rad = botHeadingDeg * 0.0174533;
  posX_cm -= 4.0 * cos(rad);
  posY_cm -= 4.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void turnLeft(int deg = 15) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  delay(110);
  stopMotors();
  
  botHeadingDeg += deg;
  if (botHeadingDeg >= 360.0) botHeadingDeg -= 360.0;
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void turnRight(int deg = 15) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  delay(110);
  stopMotors();
  
  botHeadingDeg -= deg;
  if (botHeadingDeg < 0.0) botHeadingDeg += 360.0;
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

// Lectura de ultrasonido HC-SR04
float readUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout (~5m)
  if (duration == 0) return 400.0;

  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0) distance = 2.0;
  if (distance > 400.0) distance = 400.0;
  return distance;
}

void setup() {
  Serial.begin(115200);
  delay(400);

  // Pines de ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Pines de motores
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  stopMotors();

  // Configurar servo PWM
  ESP32PWM::allocateTimer(0);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);

  // Punto de inicio: 90 GRADOS
  currentAngle = CENTER_ANGLE;
  radarServo.write(currentAngle);
  delay(400);

  Serial.println("SYS:READY,SMART_SONAR_ROVER_ESP32_V4");
  Serial.printf("SYS:START_ANGLE,%d\\n", CENTER_ANGLE);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void loop() {
  // 1. Procesar comandos seriales desde la App Web
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\\n');
    cmd.trim();
    cmd.toUpperCase();

    if (cmd == "START") {
      isScanningActive = true;
      Serial.println("SYS:SCAN_RESUMED");
    } else if (cmd == "STOP" || cmd == "PAUSE") {
      isScanningActive = false;
      stopMotors();
      Serial.println("SYS:STOPPED");
    } else if (cmd.startsWith("MOVE:F")) {
      driveForward();
    } else if (cmd.startsWith("MOVE:B")) {
      driveBackward();
    } else if (cmd.startsWith("TURN:L")) {
      turnLeft(15);
    } else if (cmd.startsWith("TURN:R")) {
      turnRight(15);
    } else if (cmd.startsWith("GOTO:")) {
      int target = cmd.substring(5).toInt();
      currentAngle = constrain(target, PANORAMIC_MIN, PANORAMIC_MAX);
      radarServo.write(currentAngle);
      float dist = readUltrasonicDistanceCm();
      Serial.printf("PING:%d,%.1f\\n", currentAngle, dist);
      Serial.printf("DIST:%.1f\\n", dist);
    } else if (cmd == "PING") {
      float dist = readUltrasonicDistanceCm();
      Serial.printf("PING:%d,%.1f\\n", currentAngle, dist);
      Serial.printf("DIST:%.1f\\n", dist);
    }
  }

  // 2. Lógica del Sondeo Inteligente
  if (isScanningActive) {
    unsigned long now = millis();
    unsigned long stepDelay = (currentScanMode == NARROW_PATROL) ? 75 : 100;

    if (now - lastStepTime >= stepDelay) {
      lastStepTime = now;

      // Mover el servomotor al ángulo actual
      radarServo.write(currentAngle);

      // Medir la distancia ultrasónica en centímetros
      float distance = readUltrasonicDistanceCm();

      // Emitir telemetría con los centímetros exactos del objeto
      Serial.printf("PING:%d,%.1f\\n", currentAngle, distance);
      Serial.printf("DIST:%.1f\\n", distance);

      // 3. Condición de detección a <= 40 cm
      if (distance <= OBSTACLE_TRIGGER_CM) {
        if (currentScanMode == NARROW_PATROL) {
          // Activar inmediatamente el barrido panorámico de 180° (-90° a +90°)
          currentScanMode = OBSTACLE_PANORAMIC;
          Serial.printf("ALERT:OBSTACLE,%.1f\\n", distance);
          Serial.println("SYS:MODE,OBSTACLE_PANORAMIC_180");
        }
      }

      // 4. Avance del ángulo según el modo activo
      if (currentScanMode == NARROW_PATROL) {
        // Sondeo estrecho de 0 a +15° y de 0 a -15° (75° a 105°)
        const int minNarrow = CENTER_ANGLE - NARROW_SPAN; // 75°
        const int maxNarrow = CENTER_ANGLE + NARROW_SPAN; // 105°

        currentAngle += (sweepDirection * 2);

        if (currentAngle >= maxNarrow) {
          currentAngle = maxNarrow;
          sweepDirection = -1;
        } else if (currentAngle <= minNarrow) {
          currentAngle = minNarrow;
          sweepDirection = 1;
        }
      } else {
        // Barrido panorámico 0° a 180° (-90° a +90° desde el centro 90°)
        currentAngle += (sweepDirection * 1);

        if (currentAngle >= PANORAMIC_MAX) {
          currentAngle = PANORAMIC_MAX;
          sweepDirection = -1;
          totalSweepsCompleted++;
          Serial.printf("SYS:SWEEP_CYCLE_COMPLETE,%lu\\n", totalSweepsCompleted);

          // Si el camino se despejó (> 40 cm), retornar al punto de inicio 90°
          if (distance > OBSTACLE_TRIGGER_CM) {
            currentScanMode = NARROW_PATROL;
            currentAngle = CENTER_ANGLE;
            radarServo.write(currentAngle);
            Serial.println("SYS:PATH_CLEAR_RETURNING_TO_NARROW_PATROL");
          }
        } else if (currentAngle <= PANORAMIC_MIN) {
          currentAngle = PANORAMIC_MIN;
          sweepDirection = 1;
          totalSweepsCompleted++;
          Serial.printf("SYS:SWEEP_CYCLE_COMPLETE,%lu\\n", totalSweepsCompleted);

          if (distance > OBSTACLE_TRIGGER_CM) {
            currentScanMode = NARROW_PATROL;
            currentAngle = CENTER_ANGLE;
            radarServo.write(currentAngle);
            Serial.println("SYS:PATH_CLEAR_RETURNING_TO_NARROW_PATROL");
          }
        }
      }
    }
  }
}
`;
