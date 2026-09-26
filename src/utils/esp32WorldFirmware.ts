// ESP32 Arduino C++ Firmware Code for Bot Trajectory & 2D World Mapping Rover

export const ESP32_WORLD_DISCOVERER_CODE = `/*
 * ==============================================================================
 * PROYECTO: ESP32 ROVER DE MAPEO 2D Y REGISTRO DE TRAYECTORIA
 * ==============================================================================
 * Funcionalidad:
 *   1. Sensor ultrasónico HC-SR04 montado sobre servomotor que rota continuamente
 *      180 grados cada 20 segundos (9°/segundo = ~111ms por grado).
 *   2. Envío de telemetría de distancias en tiempo real: PING:angulo,distancia_cm
 *   3. Recepción de comandos de movimiento para las ruedas del bot:
 *      - MOVE:F,velocidad (Avanzar)
 *      - MOVE:B,velocidad (Retroceder)
 *      - TURN:L,grados    (Girar Izquierda)
 *      - TURN:R,grados    (Girar Derecha)
 *      - STOP             (Detener bot)
 *   4. Estimación de odometría/trayecto por dead-reckoning reportado a la app:
 *      POS:x_cm,y_cm,rumbo_grados
 *
 * Conexión de Pines ESP32:
 *   - Servomotor (PWM):      GPIO 18
 *   - HC-SR04 TRIG:          GPIO 5
 *   - HC-SR04 ECHO:          GPIO 19 (divisor 5V->3.3V recomendado)
 *   - Motor Izquierdo (IN1): GPIO 25 (opcional driver L298N/TB6612)
 *   - Motor Izquierdo (IN2): GPIO 26
 *   - Motor Derecho   (IN3): GPIO 32
 *   - Motor Derecho   (IN4): GPIO 33
 *   - Alimentación Servo:    5V (fuente externa o VIN) + GND común
 *   - Baudrate Serial:       115200 bps
 * ==============================================================================
 */

#include <ESP32Servo.h>

// --- Definición de Pines ---
const int SERVO_PIN = 18;     // Pin PWM de control del servomotor
const int TRIG_PIN  = 5;      // Pin de disparo ultrasónico TRIG
const int ECHO_PIN  = 19;     // Pin de recepción ultrasónico ECHO

// Pines de tracción de ruedas (opcionales para chasis motorizado)
const int MOTOR_L_IN1 = 25;
const int MOTOR_L_IN2 = 26;
const int MOTOR_R_IN1 = 32;
const int MOTOR_R_IN2 = 33;

// --- Instancias y Variables de Sonar ---
Servo radarServo;

// Parámetros de barrido temporal (180 grados en 20 segundos)
const float SWEEP_PERIOD_SEC = 20.0;          // 20.0 segundos por cada barrido de 180°
const int MIN_ANGLE          = 0;
const int MAX_ANGLE          = 180;
const int ANGLE_SPAN         = MAX_ANGLE - MIN_ANGLE; // 180°
const int STEP_SIZE_DEG      = 1;
const unsigned long STEP_DELAY_MS = (unsigned long)((SWEEP_PERIOD_SEC * 1000.0) / (ANGLE_SPAN / STEP_SIZE_DEG)); // ~111 ms

int currentAngle        = 0;
int sweepDirection      = 1;  // +1: avanzando de 0° a 180°, -1: volviendo de 180° a 0°
bool isScanningActive   = true;
unsigned long lastStepTime = 0;
unsigned long totalSweepsCompleted = 0;

// --- Odometría del Bot (Estimación de trayectoria) ---
float posX_cm = 0.0;
float posY_cm = 30.0;
float botHeadingDeg = 90.0; // 90° es Norte (+Y)
unsigned long lastOdomTime = 0;

// Funciones de control de motores
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
  
  // Actualizar estimación de posición
  float rad = botHeadingDeg * 0.0174533;
  posX_cm += 5.0 * cos(rad);
  posY_cm += 5.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void driveBackward(int speedPwm = 200) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  
  float rad = botHeadingDeg * 0.0174533;
  posX_cm -= 5.0 * cos(rad);
  posY_cm -= 5.0 * sin(rad);
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

void turnLeft(int deg = 15) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  delay(120);
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
  delay(120);
  stopMotors();
  
  botHeadingDeg -= deg;
  if (botHeadingDeg < 0.0) botHeadingDeg += 360.0;
  Serial.printf("POS:%.1f,%.1f,%.1f\\n", posX_cm, posY_cm, botHeadingDeg);
}

// Lectura de sensor ultrasónico HC-SR04
float readUltrasonicDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000); // 30ms timeout
  if (duration == 0) return 400.0;

  float distance = (duration * 0.0343) / 2.0;
  if (distance < 2.0) distance = 2.0;
  if (distance > 400.0) distance = 400.0;
  return distance;
}

void setup() {
  Serial.begin(115200);
  delay(400);

  // Configurar ultrasonido
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // Configurar pines de motores
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT);
  pinMode(MOTOR_R_IN2, OUTPUT);
  stopMotors();

  // Configurar servo PWM en ESP32
  ESP32PWM::allocateTimer(0);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);

  currentAngle = 0;
  radarServo.write(currentAngle);
  delay(500);

  Serial.println("SYS:READY,ESP32_ROVER_ROUTE_MAPPER_V3");
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
      currentAngle = constrain(target, MIN_ANGLE, MAX_ANGLE);
      radarServo.write(currentAngle);
      float dist = readUltrasonicDistanceCm();
      Serial.printf("PING:%d,%.1f\\n", currentAngle, dist);
    } else if (cmd == "PING") {
      float dist = readUltrasonicDistanceCm();
      Serial.printf("PING:%d,%.1f\\n", currentAngle, dist);
    }
  }

  // 2. Barrido continuo de 180° cada 20 segundos
  if (isScanningActive) {
    unsigned long now = millis();

    if (now - lastStepTime >= STEP_DELAY_MS) {
      lastStepTime = now;

      // Posicionar servo
      radarServo.write(currentAngle);

      // Medir distancia ultrasónica
      float distance = readUltrasonicDistanceCm();

      // Emitir telemetría al puerto serial: PING:angulo,distancia
      Serial.printf("PING:%d,%.1f\\n", currentAngle, distance);

      // Avanzar al siguiente grado
      currentAngle += (sweepDirection * STEP_SIZE_DEG);

      // Invertir sentido al completar los 180 grados
      if (currentAngle >= MAX_ANGLE) {
        currentAngle = MAX_ANGLE;
        sweepDirection = -1;
        totalSweepsCompleted++;
        Serial.printf("SYS:SWEEP_CYCLE_COMPLETE,%lu\\n", totalSweepsCompleted);
      } else if (currentAngle <= MIN_ANGLE) {
        currentAngle = MIN_ANGLE;
        sweepDirection = 1;
        totalSweepsCompleted++;
        Serial.printf("SYS:SWEEP_CYCLE_COMPLETE,%lu\\n", totalSweepsCompleted);
      }
    }
  }
}
`;
