#include <Arduino.h>

#ifndef ARDUINO_ARCH_ESP32
#error "Compile this code for an ESP32 board."
#endif

// ========== H-BRIDGE PINS ==========
const int IN1 = 12;     // Direction A
const int IN2 = 13;     // Direction B
const int EN  = 14;     // PWM (Enable)

// ========== PWM CONFIGURATION ==========
const uint32_t PWM_FREQ_HZ   = 1000;   // 1 kHz PWM frequency
const uint8_t  PWM_RES_BITS  = 10;     // 10 bits → 0–1023
const uint32_t PWM_MAX       = (1u << PWM_RES_BITS) - 1u;  // 1023

// Duty cycles to test (in %)
const uint8_t DUTIES_PCT[] = {25, 50, 75, 100};
const size_t  N_DUTIES     = sizeof(DUTIES_PCT) / sizeof(DUTIES_PCT[0]);

// Convert percentage to duty cycle value
static inline uint32_t dutyFromPercent(uint8_t pct) {
  if (pct >= 100) return PWM_MAX;
  return (uint32_t)((uint32_t)pct * (PWM_MAX + 1u) / 100u);
}

void setup() {
  // Configure H-Bridge pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  // PWM configuration (ESP32 core v3.x)
  ledcAttach(EN, PWM_FREQ_HZ, PWM_RES_BITS);
  ledcWrite(EN, 0);

  Serial.begin(115200);
  Serial.println("System started - Motor PWM test");
}

void loop() {
  // ---- Direction A ----
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  for (size_t i = 0; i < N_DUTIES; ++i) {
    ledcWrite(EN, dutyFromPercent(DUTIES_PCT[i]));
    Serial.print("Direction A - Duty: ");
    Serial.print(DUTIES_PCT[i]);
    Serial.println("%");
    delay(2000);
  }

  ledcWrite(EN, 0);
  delay(400);

  // ---- Direction B ----
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  for (size_t i = 0; i < N_DUTIES; ++i) {
    ledcWrite(EN, dutyFromPercent(DUTIES_PCT[i]));
    Serial.print("Direction B - Duty: ");
    Serial.print(DUTIES_PCT[i]);
    Serial.println("%");
    delay(2000);
  }

  ledcWrite(EN, 0);
  delay(400);
}
