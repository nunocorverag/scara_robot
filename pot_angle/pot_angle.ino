#include <Arduino.h>

const int POT_PIN = 34;        // Analog input pin (GPIO34)
const float ADC_MAX = 4095.0;  // 12-bit ADC resolution
const float VREF = 3.3;        // Reference voltage (3.3V)

// Convert ADC reading (0–4095) to degrees (0–360)
float adcToDegrees(int adcValue) {
  return (adcValue * 360.0) / ADC_MAX;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("=== Angle measurement with potentiometer ===");

  analogReadResolution(12);
}

void loop() {
  int raw = analogRead(POT_PIN);
  float voltage = (raw / ADC_MAX) * VREF;
  float degrees = adcToDegrees(raw);

  Serial.print("ADC: ");
  Serial.print(raw);
  Serial.print(" | Voltage: ");
  Serial.print(voltage, 2);
  Serial.print(" V | Angle: ");
  Serial.print(degrees, 1);
  Serial.println("°");

  delay(300);
}
