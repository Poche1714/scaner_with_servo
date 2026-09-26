// ESP32 Arduino C++ Firmware Code for Bot Trajectory, Zero-Lag Smart Sonar & Calibrated PWM Motors

export const ESP32_WORLD_DISCOVERER_CODE = `/*
 * ==============================================================================
 * PROYECTO: ESP32 ROVER DE MAPEO, SONAR INTELIGENTE SIN DESFASE & MOTORES PWM
 * ==============================================================================
 * Especificaciones de Funcionamiento:
 *   1. Eliminación del Desfase Servo-Sensor (Zero-Phase-Lag):
 *      - Retardo de estabilización mecánica (delay de 40ms) tras mover el servomotor
 *        antes de emitir el pulso TRIG ultrasónico.
 *      - Medición etiquetada con el ángulo físico real estabilizado.
 *
 *   2. Rango de Senso Normal y con Obstáculo:
 *      - Punto de inicio: 90 GRADOS (0° relativo / Frente).
 *      - Estado Normal: Senso de 0 a 30° y de 0 a -30° (60° a 120° servo).
 *      - Detección de Obstáculo (<= 40 cm):
 *        * NO se detiene el muestreo (el senso continúa activo).
 *        * Se disminuye el rango de senso a 0 a 10° y de 0 a -10° (80° a 100° servo).
 *      - Al girar el bot: Vuelve al estado normal de 0 a 30° y de 0 a -30°.
 *
 *   3. Rutina de Motores Calibrada con PWM (175 a 198):
 *      - Tracción mediante analogWrite en ambos canales del puente H.
 *      - Duración calibrada por paso para garantizar movimiento físico real.
 *      - Giros como en la versión anterior (120ms) que restauran el senso normal.
 *
 * Conexión de Pines ESP32:
 *   - Servomotor (PWM):      GPIO 18
 *   - HC-SR04 TRIG:          GPIO 5
 *   - HC-SR04 ECHO:          GPIO 19 (divisor resistivo recomendado)
 *   - Motor Izquierdo (IN1): GPIO 25 (driver L298N o TB6612)
 *   - Motor Izquierdo (IN2): GPIO 26
 *   - Motor Derecho   (IN3): GPIO 32
 *   - Motor Derecho   (IN4): GPIO 33
 *   - Alimentación Servo:    5V (fuente con masa común)
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
int currentMotorPwm     = 185; // Velocidad predeterminada

// --- Instancias y Variables de Sonar ---
Servo radarServo;

// Parámetros angulares del sonar inteligente
const int CENTER_ANGLE          = 90;   // 0° relativo (Frente)
const int NORMAL_SPAN           = 30;   // Estado normal: 0 a 30° y 0 a -30° (60° a 120° servo)
const int OBSTACLE_SPAN         = 10;   // Con obstáculo: 0 a 10° y de 0 a -10° (80° a 100° servo)
const float OBSTACLE_TRIGGER_CM = 40.0; // Umbral de detección (40 cm)

// Tiempos para evitar desfase físico servo-ultrasonido
const unsigned long SERVO_SETTLE_MS = 40; // Espera para estabilización mecánica del servo

enum ScanMode {
  NORMAL_SWEEP,          // 0 a 30° y 0 a -30° (60° a 120°)
  OBSTACLE_REDUCED_SWEEP // 0 a 10° y 0 a -10° (80° a 100°)
};

ScanMode currentScanMode   = NORMAL_SWEEP;
int currentAngle           = CENTER_ANGLE; // Inicia en 90°
int sweepDirection         = 1;            // +1 hacia un lado, -1 hacia el otro
bool isScanningActive      = true;
unsigned long lastStepTime = 0;
unsigned long totalSweepsCompleted = 0;

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

// --- Control de Motores (Tracción por hardware analogWrite) ---
void stopMotors() {
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, 0);
}

// Rutina de avance modificada: tracción firme sin estados flotantes y con duración real
void driveForward(int pwm = -1, int durationMs = 450) {
  int speed = (pwm > 0) ? clampPwm(pwm) : currentMotorPwm;
  analogWrite(MOTOR_L_IN1, speed);
  analogWrite(MOTOR_L_IN2, 0);
  analogWrite(MOTOR_R_IN1, speed);
  analogWrite(MOTOR_R_IN2, 0);

  int dur = (durationMs > 0) ? durationMs : 450;
  delay(dur);
  stopMotors();

  float rad = botHeadingDeg * 0.0174533;
  posX_cm += 20.0 * cos(rad);
  posY_cm += 20.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

// Rutina de retroceso modificada
void driveBackward(int pwm = -1, int durationMs = 450) {
  int speed = (pwm > 0) ? clampPwm(pwm) : currentMotorPwm;
  analogWrite(MOTOR_L_IN1, 0);
  analogWrite(MOTOR_L_IN2, speed);
  analogWrite(MOTOR_R_IN1, 0);
  analogWrite(MOTOR_R_IN2, speed);

  int dur = (durationMs > 0) ? durationMs : 450;
  delay(dur);
  stopMotors();

  float rad = botHeadingDeg * 0.0174533;
  posX_cm -= 20.0 * cos(rad);
  posY_cm -= 20.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

// Giros: conservados como en la versión anterior + restablecimiento del senso al estado normal (0 a 30 y 0 a -30)
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
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);

  // AL GIRAR EL BOT: vuelve al estado normal de 0 a 30 y de 0 a -30
  currentScanMode = NORMAL_SWEEP;
  isScanningActive = true;
  Serial.println("SYS:SCAN_NORMAL_RESET_30");
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
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);

  // AL GIRAR EL BOT: vuelve al estado normal de 0 a 30 y de 0 a -30
  currentScanMode = NORMAL_SWEEP;
  isScanningActive = true;
  Serial.println("SYS:SCAN_NORMAL_RESET_30");
}

// Lectura de ultrasonido HC-SR04
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

  Serial.println("SYS:READY,SMART_SONAR_ZERO_LAG_ROVER_ESP32_V6");
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
      int dur = 450;
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
      int dur = 450;
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
    } else if (cmd == "RESET_SCAN") {
      currentScanMode = NORMAL_SWEEP;
      isScanningActive = true;
      Serial.println("SYS:SCAN_NORMAL_RESET_30");
    } else if (cmd.startsWith("GOTO:")) {
      int target = cmd.substring(5).toInt();
      currentAngle = constrain(target, 0, 180);
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

  // 2. Lógica del Sondeo Inteligente SIN Desfase (Muestreo continuo sin detener)
  if (isScanningActive) {
    unsigned long now = millis();
    unsigned long stepDelay = 65; // Muestreo ágil continuo

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
      // "no quiero que pares si encuentra el obstaculo, solo quiero que disminuyas el rango de senso,
      //  pasa de 0 a 30 y 0 a -30 a 0 a 10 y de 0 a -10"
      if (distance <= OBSTACLE_TRIGGER_CM) {
        if (currentScanMode != OBSTACLE_REDUCED_SWEEP) {
          currentScanMode = OBSTACLE_REDUCED_SWEEP;
          Serial.printf("ALERT:OBSTACLE_REDUCED_SPAN,%.1f\\n", distance);
        }
      }

      // PASO F: Avance angular según el modo activo
      // Normal: 0 a 30° y 0 a -30° (60° a 120° servo)
      // Obstáculo: 0 a 10° y 0 a -10° (80° a 100° servo)
      int activeSpan = (currentScanMode == OBSTACLE_REDUCED_SWEEP) ? OBSTACLE_SPAN : NORMAL_SPAN;
      int minAngle = CENTER_ANGLE - activeSpan; // 80° con obstáculo, 60° normal
      int maxAngle = CENTER_ANGLE + activeSpan; // 100° con obstáculo, 120° normal

      // Si el ángulo actual está fuera del rango (ej. al reducirse el span repentinamente)
      if (currentAngle > maxAngle) {
        currentAngle = maxAngle;
        sweepDirection = -1;
      } else if (currentAngle < minAngle) {
        currentAngle = minAngle;
        sweepDirection = 1;
      } else {
        currentAngle += (sweepDirection * 2);
        if (currentAngle >= maxAngle) {
          currentAngle = maxAngle;
          sweepDirection = -1;
          totalSweepsCompleted++;
        } else if (currentAngle <= minAngle) {
          currentAngle = minAngle;
          sweepDirection = 1;
          totalSweepsCompleted++;
        }
      }
    }
  }
}
`;
