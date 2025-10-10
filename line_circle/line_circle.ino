#include <Arduino.h>
#include <math.h>

#ifndef ARDUINO_ARCH_ESP32
#error "Compile this code for an ESP32 board."
#endif


// === Motor configuration  ============


// --- Motor 1 (Basis / q1) ---
const int M1_IN1 = 12;   // Direction A
const int M1_IN2 = 13;   // Direction B
const int M1_EN  = 14;   // PWM (Enable)

// --- Motor 2 (Elbow / q2) ---
const int M2_IN1 = 25;   // Direction A
const int M2_IN2 = 26;   // Direction B
const int M2_EN  = 27;   // PWM (Enable)


// === Parameters of the robot ========================

const float L1 = 0.30;   // [m]
const float L2 = 0.18;   // [m]


// === Configuration PWM ========================

const uint32_t PWM_FREQ_HZ  = 1000;   // 1 kHz
const uint8_t  PWM_RES_BITS = 10;     // 10 bits → 0–1023
const uint32_t PWM_MAX       = (1u << PWM_RES_BITS) - 1u;
const uint8_t  PWM_CH_M1 = 0;
const uint8_t  PWM_CH_M2 = 1;


// === Trajectories (Line + Circle) ====================

const int N_LINE = 100;
const int N_CIRC = 200;


// === General tools =================================

float rad2deg(float rad) { return rad * 180.0 / M_PI; }
float deg2rad(float deg) { return deg * M_PI / 180.0; }

// Conversion for PWM
static inline uint32_t dutyFromPercent(float pct) {
  if (pct >= 100.0) return PWM_MAX;
  if (pct <= 0.0) return 0;
  return (uint32_t)(pct * (PWM_MAX + 1.0) / 100.0);
}

// Motor control function
void setMotorSpeed(int pwmChannel, int IN1, int IN2, float speed) {
  // speed en [-1.0, +1.0]
  bool dir = (speed >= 0);
  float val = fabs(speed);
  val = constrain(val, 0.0, 1.0);

  digitalWrite(IN1, dir ? HIGH : LOW);
  digitalWrite(IN2, dir ? LOW : HIGH);
  ledcWrite(pwmChannel, (int)(val * PWM_MAX));
}

// Stop motor
void stopMotor(int IN1, int IN2, int pwmChannel) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  ledcWrite(pwmChannel, 0);
}


// === Inverse kinematics==============

void inverseKinematics(float x, float y, float &q1_deg, float &q2_deg) {
  float c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  c2 = constrain(c2, -1, 1);
  float s2 = -sqrt(1 - c2 * c2);
  float q2 = atan2(s2, c2);
  float k1 = L1 + L2 * c2;
  float k2 = L2 * s2;
  float q1 = atan2(y, x) - atan2(k2, k1);
  q1_deg = rad2deg(q1);
  q2_deg = rad2deg(q2);
}


// === SETUP ============================================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== SCARA Robot - Line and Circle ===");

  // Configuration Motor 1
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  ledcAttach(M1_EN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(M1_EN, 0);

  // Configuration Motor 2
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  ledcAttach(M2_EN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(M2_EN, 0);

  Serial.println("Motors initialized");
}


// === Movement towards a target angle ====================

void moveToAngles(float q1_target, float q2_target, float duration_s = 0.2) {
 
  float q1_speed = 0.5; // Fix values (50%)
  float q2_speed = 0.5;

  setMotorSpeed(PWM_CH_M1, M1_IN1, M1_IN2, q1_speed);
  setMotorSpeed(PWM_CH_M2, M2_IN1, M2_IN2, q2_speed);

  delay((int)(duration_s * 1000));

  stopMotor(M1_IN1, M1_IN2, PWM_CH_M1);
  stopMotor(M2_IN1, M2_IN2, PWM_CH_M2);
}

// === Main loop ================================

void loop() {
  // =================== LINE TRAJECTORIE ====================
  Serial.println("\n--- Tracking the line ---");
  float x1 = 0.30, y1 = 0.30;
  float x2 = 0.15, y2 = 0.15;
  for (int i = 0; i < N_LINE; i++) {
    float t = (float)i / (N_LINE - 1);
    float x = x1 + t * (x2 - x1);
    float y = y1 + t * (y2 - y1);

    float q1t, q2t;
    inverseKinematics(x, y, q1t, q2t);

    Serial.printf("Line Point %d: x=%.3f y=%.3f -> q1=%.1f°, q2=%.1f°\n", i, x, y, q1t, q2t);

    moveToAngles(q1t, q2t, 0.15);
  }

  stopMotor(M1_IN1, M1_IN2, PWM_CH_M1);
  stopMotor(M2_IN1, M2_IN2, PWM_CH_M2);
  delay(1000);

  // =================== CIRCLE TRAJECTORIE ====================
  Serial.println("\n--- Tracking the circle ---");
  float cx = 0.25, cy = 0.25, r = 0.10;
  for (int i = 0; i < N_CIRC; i++) {
    float theta = 2 * M_PI * i / N_CIRC;
    float x = cx + r * cos(theta);
    float y = cy + r * sin(theta);

    float q1t, q2t;
    inverseKinematics(x, y, q1t, q2t);

    Serial.printf("Circle θ=%.2f rad -> x=%.3f y=%.3f | q1=%.1f°, q2=%.1f°\n", theta, x, y, q1t, q2t);

    moveToAngles(q1t, q2t, 0.05);
  }

  stopMotor(M1_IN1, M1_IN2, PWM_CH_M1);
  stopMotor(M2_IN1, M2_IN2, PWM_CH_M2);

  Serial.println("\nTrajectories finished !");
  delay(3000);
}
