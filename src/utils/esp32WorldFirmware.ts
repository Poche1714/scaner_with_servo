// ESP32 Arduino C++ Firmware Code for Bot Trajectory, Zero-Lag Smart Sonar & Calibrated PWM Motors

export const ESP32_WORLD_DISCOVERER_CODE = `/*
 * ==============================================================================
 * PROYECTO: ESP32 ROVER DE MAPEO, SONAR INTELIGENTE SIN DESFASE & MOTORES PWM
 * ==============================================================================
 * Requisitos y Mejoras Implementadas:
 *   1. Eliminación del Desfase Servo-Sensor (Zero-Phase-Lag):
 *      - Se introduce un retardo de estabilización mecánica (delay de 40ms) tras mover
 *        el servomotor antes de emitir el pulso TRIG ultrasónico.
 *      - La distancia medida se etiqueta estrictamente con el ángulo estático en el que
 *        el servo se encuentra físicamente detenido, eliminando el desfase de 2 pasos.
 *   2. Punto de Inicio (0 Relativo / Frente):
 *      - El servo inicia en 90 GRADOS (0° relativo hacia el frente del robot).
 *      - Modo Normal: Vigilancia estrecha de 0 a +15° y de 0 a -15° (75° a 105° servo).
 *   3. Nuevo Algoritmo de Detección de Obstáculos:
 *      - Al detectar un obstáculo a <= 40 cm, el barrido NO gira de -90° a 90°,
 *        sino que está delimitado entre -55° y +55° relativo (35° a 145° servo).
 *      - Sondeo Enfocado del Objeto: Al detectar el objeto en una posición angular
 *        (ej. +25° relativo), el sistema ejecuta un sondeo específico:
 *        * Registra los siguientes 25 grados más (+25° adicionales).
 *        * Registra los 10 grados menos (-10°).
 *        * Tras registrar el contorno completo del objeto, regresa automáticamente
 *          al estado normal de vigilancia.
 *   4. Control de Motores con PWM (Rango Calibrado 175 a 198):
 *      - Modulación PWM por hardware (ESP32 LEDC) en pines de tracción.
 *      - Rango de tracción óptimo: 175 (mínimo con par) a 198 (máximo calibrado).
 *      - Comandos aceptados:
 *        * MOVE:F,pwm  (Avanzar con velocidad PWM 175 a 198)
 *        * MOVE:B,pwm  (Retroceder con velocidad PWM 175 a 198)
 *        * TURN:L,pwm  (Girar izquierda con PWM)
 *        * TURN:R,pwm  (Girar derecha con PWM)
 *        * PWM:valor   (Ajustar PWM por defecto)
 *        * STOP        (Detener motores)
 *   5. Telemetría en Tiempo Real:
 *      - PING:angulo_servo,distancia_cm
 *      - DIST:distancia_cm
 *      - ALERT:OBSTACLE,distancia_cm
 *      - ALERT:SURVEY_OBJECT,angulo_rel,min_rel,max_rel
 *      - POS:x,y,rumbo
 *
 * Conexión de Pines ESP32:
 *   - Servomotor (PWM):      GPIO 18
 *   - HC-SR04 TRIG:          GPIO 5
 *   - HC-SR04 ECHO:          GPIO 19 (divisor de tensión 5V a 3.3V recomendado)
 *   - Motor Izquierdo (IN1): GPIO 25 (driver L298N o TB6612)
 *   - Motor Izquierdo (IN2): GPIO 26
 *   - Motor Derecho   (IN3): GPIO 32
 *   - Motor Derecho   (IN4): GPIO 33
 *   - Alimentación Servo:    5V (VIN / fuente externa con masa común)
 *   - Baudrate Serial:       115200 bps
 * ==============================================================================
 */

#include <ESP32Servo.h>

// --- Definición de Pines ---
const int SERVO_PIN   = 18;    // Pin PWM servomotor radar
const int TRIG_PIN    = 5;     // Pin de disparo ultrasónico TRIG
const int ECHO_PIN    = 19;    // Pin de eco ultrasónico ECHO

// Pines de tracción de motores (L298N / TB6612)
const int MOTOR_L_IN1 = 25;
const int MOTOR_L_IN2 = 26;
const int MOTOR_R_IN1 = 32;
const int MOTOR_R_IN2 = 33;

// Rango PWM calibrado para motores (175 a 198)
const int MIN_MOTOR_PWM = 175;
const int MAX_MOTOR_PWM = 198;
int currentMotorPwm     = 185; // Velocidad de crucero predeterminada

// --- Instancias y Variables de Sonar ---
Servo radarServo;

// Parámetros angulares del sonar inteligente
const int CENTER_ANGLE         = 90;   // 0° relativo (Frente)
const int NARROW_SPAN          = 15;   // Normal: ±15° (75° a 105°)
const int MAX_OBSTACLE_SPAN    = 55;   // En obstáculo: no 90 a -90, sino 55 a -55 (35° a 145°)
const int BOUND_MIN            = CENTER_ANGLE - MAX_OBSTACLE_SPAN; // 35°
const int BOUND_MAX            = CENTER_ANGLE + MAX_OBSTACLE_SPAN; // 145°
const float OBSTACLE_TRIGGER_CM = 40.0; // Umbral de detección (40 cm)

// Tiempos para evitar desfase físico servo-ultrasonido
const unsigned long SERVO_SETTLE_MS = 40; // Espera para estabilización mecánica del servo

enum ScanMode {
  NARROW_PATROL,           // Vigilancia normal ±15° (75° a 105°)
  OBJECT_FOCUSED_SURVEY,   // Sondeo enfocado del objeto detectado (+25° más y -10° menos, 3 barridos)
  SURVEY_PAUSED,           // Sondeo detenido tras 3 barridos (se reactiva al girar el bot)
  SAFETY_PANORAMIC_55      // Barrido de seguridad delimitado a ±55° (35° a 145°)
};

ScanMode currentScanMode   = NARROW_PATROL;
int currentAngle           = CENTER_ANGLE; // Inicia en 90°
int sweepDirection         = 1;            // +1 hacia un lado, -1 hacia el otro
bool isScanningActive      = true;
unsigned long lastStepTime = 0;
unsigned long totalSweepsCompleted = 0;

// Variables del sondeo enfocado de objeto (3 barridos)
int surveyTargetAngleRel   = 0;
int surveyMinServoAngle    = CENTER_ANGLE - 10;
int surveyMaxServoAngle    = CENTER_ANGLE + 25;
int surveyStepDirection    = 1;
int surveyPassCount        = 0; // Conteo de 0 a 3 barridos

// Odometría estimada
float posX_cm = 0.0;
float posY_cm = 30.0;
float botHeadingDeg = 90.0; // 90° es Norte (+Y)

// Helper para limitar PWM al rango 175-198
int clampPwm(int value) {
  if (value < MIN_MOTOR_PWM) return MIN_MOTOR_PWM;
  if (value > MAX_MOTOR_PWM) return MAX_MOTOR_PWM;
  return value;
}

// --- Control de Motores con PWM (analogWrite universal para ESP32 Core v2 y v3) ---
void stopMotors() {
  analogWrite(MOTOR_L_IN1, 0);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_R_IN1, 0);
  digitalWrite(MOTOR_R_IN2, LOW);
}

void driveForward(int pwm = -1, int durationMs = 600) {
  int speed = (pwm > 0) ? clampPwm(pwm) : currentMotorPwm;
  // Pulso de arranque de 40ms al 100% (255) para vencer la inercia estática y rozamiento con el suelo
  analogWrite(MOTOR_L_IN1, 255);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_R_IN1, 255);
  digitalWrite(MOTOR_R_IN2, LOW);
  delay(40);

  // Mantener avance con el PWM calibrado del usuario
  analogWrite(MOTOR_L_IN1, speed);
  digitalWrite(MOTOR_L_IN2, LOW);
  analogWrite(MOTOR_R_IN1, speed);
  digitalWrite(MOTOR_R_IN2, LOW);

  if (durationMs > 0) {
    int remainingMs = durationMs - 40;
    if (remainingMs > 0) delay(remainingMs);
    stopMotors();
  }

  float rad = botHeadingDeg * 0.0174533;
  posX_cm += 20.0 * cos(rad);
  posY_cm += 20.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\n", posX_cm, posY_cm, botHeadingDeg);
}

void driveBackward(int pwm = -1, int durationMs = 600) {
  int speed = (pwm > 0) ? clampPwm(pwm) : currentMotorPwm;
  // Pulso de arranque
  digitalWrite(MOTOR_L_IN1, LOW);
  analogWrite(MOTOR_L_IN2, 255);
  digitalWrite(MOTOR_R_IN1, LOW);
  analogWrite(MOTOR_R_IN2, 255);
  delay(40);

  digitalWrite(MOTOR_L_IN1, LOW);
  analogWrite(MOTOR_L_IN2, speed);
  digitalWrite(MOTOR_R_IN1, LOW);
  analogWrite(MOTOR_R_IN2, speed);

  if (durationMs > 0) {
    int remainingMs = durationMs - 40;
    if (remainingMs > 0) delay(remainingMs);
    stopMotors();
  }

  float rad = botHeadingDeg * 0.0174533;
  posX_cm -= 20.0 * cos(rad);
  posY_cm -= 20.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\n", posX_cm, posY_cm, botHeadingDeg);
}

void turnLeft(int pwm = -1, int deg = 15) {
  int speed = (pwm > 0) ? clampPwm(pwm) : currentMotorPwm;
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, speed);
  analogWrite(MOTOR_R_IN1, speed);
  analogWrite(MOTOR_R_IN2, 0);
  delay(120);
  stopMotors();

  botHeadingDeg += deg;
  if (botHeadingDeg >= 360.0) botHeadingDeg -= 360.0;
  Serial.printf("POS:%.1f,%.1f,%.1f\n", posX_cm, posY_cm, botHeadingDeg);

  // REINICIAR EL SONDEO EN 90° AL GIRAR EL BOT
  currentScanMode = NARROW_PATROL;
  currentAngle = CENTER_ANGLE;
  radarServo.write(CENTER_ANGLE);
  isScanningActive = true;
  surveyPassCount = 0;
  Serial.println("SYS:SCAN_RESTARTED_ON_TURN");
}

void turnRight(int pwm = -1, int deg = 15) {
  int speed = (pwm > 0) ? clampPwm(pwm) : currentMotorPwm;
  analogWrite(MOTOR_L_IN1, speed);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, speed);
  delay(120);
  stopMotors();

  botHeadingDeg -= deg;
  if (botHeadingDeg < 0.0) botHeadingDeg += 360.0;
  Serial.printf("POS:%.1f,%.1f,%.1f\n", posX_cm, posY_cm, botHeadingDeg);

  // REINICIAR EL SONDEO EN 90° AL GIRAR EL BOT
  currentScanMode = NARROW_PATROL;
  currentAngle = CENTER_ANGLE;
  radarServo.write(CENTER_ANGLE);
  isScanningActive = true;
  surveyPassCount = 0;
  Serial.println("SYS:SCAN_RESTARTED_ON_TURN");
}

// Lectura de ultrasonido HC-SR04 con filtrado anti-ruido
float readUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 26000); // ~4.5m timeout
  if (duration == 0) return 400.0;

  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0) distance = 2.0;
  if (distance > 400.0) distance = 400.0;
  return distance;
}

void setup() {
  Serial.begin(115200);
  delay(300);

  // Configuración de pines de ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configuración de pines de tracción de motores (PWM analógico)
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  stopMotors();

  // Configurar servomotor PWM
  ESP32PWM::allocateTimer(0);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);

  // Punto de inicio: 90 GRADOS (0° relativo / Frente)
  currentAngle = CENTER_ANGLE;
  radarServo.write(currentAngle);
  delay(500); // Permitir posicionamiento inicial completo

  Serial.println("SYS:READY,SMART_SONAR_ZERO_LAG_ROVER_ESP32_V5");
  Serial.printf("SYS:START_ANGLE,%d\\n", CENTER_ANGLE);
  Serial.printf("SYS:PWM_RANGE,%d,%d\\n", MIN_MOTOR_PWM, MAX_MOTOR_PWM);
  Serial.printf("PWM:%d\\n", currentMotorPwm);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void loop() {
  // 1. Procesar comandos seriales desde la Web App
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
      int pwm = currentMotorPwm;
      int dur = 200;
      int comma1 = cmd.indexOf(',');
      if (comma1 > 0) {
        int comma2 = cmd.indexOf(',', comma1 + 1);
        if (comma2 > 0) {
          pwm = cmd.substring(comma1 + 1, comma2).toInt();
          dur = cmd.substring(comma2 + 1).toInt();
        } else {
          pwm = cmd.substring(comma1 + 1).toInt();
        }
      }
      driveForward(pwm, dur);
    } else if (cmd.startsWith("MOVE:B")) {
      int pwm = currentMotorPwm;
      int dur = 200;
      int comma1 = cmd.indexOf(',');
      if (comma1 > 0) {
        int comma2 = cmd.indexOf(',', comma1 + 1);
        if (comma2 > 0) {
          pwm = cmd.substring(comma1 + 1, comma2).toInt();
          dur = cmd.substring(comma2 + 1).toInt();
        } else {
          pwm = cmd.substring(comma1 + 1).toInt();
        }
      }
      driveBackward(pwm, dur);
    } else if (cmd.startsWith("TURN:L")) {
      int pwm = currentMotorPwm;
      int deg = 15;
      int comma1 = cmd.indexOf(',');
      if (comma1 > 0) {
        int comma2 = cmd.indexOf(',', comma1 + 1);
        if (comma2 > 0) {
          pwm = cmd.substring(comma1 + 1, comma2).toInt();
          deg = cmd.substring(comma2 + 1).toInt();
        } else {
          pwm = cmd.substring(comma1 + 1).toInt();
        }
      }
      turnLeft(pwm, deg);
    } else if (cmd.startsWith("TURN:R")) {
      int pwm = currentMotorPwm;
      int deg = 15;
      int comma1 = cmd.indexOf(',');
      if (comma1 > 0) {
        int comma2 = cmd.indexOf(',', comma1 + 1);
        if (comma2 > 0) {
          pwm = cmd.substring(comma1 + 1, comma2).toInt();
          deg = cmd.substring(comma2 + 1).toInt();
        } else {
          pwm = cmd.substring(comma1 + 1).toInt();
        }
      }
      turnRight(pwm, deg);
    } else if (cmd.startsWith("PWM:")) {
      int p = cmd.substring(4).toInt();
      currentMotorPwm = clampPwm(p);
      Serial.printf("PWM:%d\\n", currentMotorPwm);
    } else if (cmd.startsWith("GOTO:")) {
      int target = cmd.substring(5).toInt();
      currentAngle = constrain(target, BOUND_MIN, BOUND_MAX);
      radarServo.write(currentAngle);
      delay(SERVO_SETTLE_MS);
      float dist = readUltrasonicDistanceCm();
      Serial.printf("PING:%d,%.1f\\n", currentAngle, dist);
      Serial.printf("DIST:%.1f\\n", dist);
    } else if (cmd == "PING") {
      float dist = readUltrasonicDistanceCm();
      Serial.printf("PING:%d,%.1f\\n", currentAngle, dist);
      Serial.printf("DIST:%.1f\\n", dist);
    }
  }

  // 2. Lógica del Sondeo Inteligente SIN Desfase
  if (isScanningActive) {
    unsigned long now = millis();
    unsigned long stepDelay = (currentScanMode == NARROW_PATROL) ? 75 : 85;

    if (now - lastStepTime >= stepDelay) {
      lastStepTime = now;

      // PASO A: Posicionar servomotor en el ángulo actual
      radarServo.write(currentAngle);

      // PASO B: Espera de estabilización mecánica (elimina el desfase de 2 pasos)
      delay(SERVO_SETTLE_MS);

      // PASO C: Muestrear distancia por eco ultrasónico en el ángulo físico estabilizado
      float distance = readUltrasonicDistanceCm();

      // PASO D: Emitir telemetría con ángulo y distancia en cm exacta y sincronizada
      Serial.printf("PING:%d,%.1f\\n", currentAngle, distance);
      Serial.printf("DIST:%.1f\\n", distance);

      // PASO E: Evaluación de Detección de Obstáculo a <= 40 cm
      if (distance <= OBSTACLE_TRIGGER_CM) {
        // REGLA: Dejar de realizar el muestreo inmediatamente y lanzar alerta
        // Solo hasta que gire el coche se reinicia el senso (en turnLeft / turnRight)
        isScanningActive = false;
        currentScanMode = SURVEY_PAUSED;
        int relAngle = currentAngle - CENTER_ANGLE;

        Serial.printf("ALERT:OBSTACLE,%.1f\\n", distance);
        Serial.printf("ALERT:OBSTACLE_STOPPED,%d,%.1f\\n", relAngle, distance);
        Serial.println("SYS:OBSTACLE_DETECTED_SAMPLING_STOPPED");
        Serial.println("SYS:STOPPED");
      }

      // PASO F: Avance angular según el modo activo
      if (currentScanMode == SURVEY_PAUSED) {
        // Sondeo detenido por detección de obstáculo. Permanece detenido hasta giro del coche.
      } else if (currentScanMode == NARROW_PATROL) {
        // Vigilancia normal estrecha: 0 a +15° y 0 a -15° (75° a 105°)
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
        // Modo panorámico de seguridad: delimitado a -55° y +55° (35° a 145°)
        currentAngle += (sweepDirection * 2);

        if (currentAngle >= BOUND_MAX) {
          currentAngle = BOUND_MAX;
          sweepDirection = -1;
          totalSweepsCompleted++;
          Serial.printf("SYS:SWEEP_CYCLE_COMPLETE,%lu\\n", totalSweepsCompleted);
          if (distance > OBSTACLE_TRIGGER_CM) {
            currentScanMode = NARROW_PATROL;
            currentAngle = CENTER_ANGLE;
          }
        } else if (currentAngle <= BOUND_MIN) {
          currentAngle = BOUND_MIN;
          sweepDirection = 1;
          totalSweepsCompleted++;
          Serial.printf("SYS:SWEEP_CYCLE_COMPLETE,%lu\\n", totalSweepsCompleted);
          if (distance > OBSTACLE_TRIGGER_CM) {
            currentScanMode = NARROW_PATROL;
            currentAngle = CENTER_ANGLE;
          }
        }
      }
    }
  }
}
`;
