#include <Arduino.h>

#include "I2Cdev.h"
#include <L3G4200D.h>
#include "Wire.h"
#include <LiquidCrystal_I2C.h>
#include <math.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);
L3G4200D gyro;

const int ACCEL_X_PIN = A0;
const int ACCEL_Y_PIN = A1;
const int ACCEL_Z_PIN = A2;

const float ACCEL_SENSITIVITY = 67.5;

int ACCEL_X_ZERO = 512;
int ACCEL_Y_ZERO = 512;
int ACCEL_Z_ZERO = 512;

float gyro_x_offset = 0;
float gyro_y_offset = 0;
float gyro_z_offset = 0;

const float IMPACT_THRESHOLD_G = 2.0;
const float ROTATION_THRESHOLD_DPS = 200.0;

const float PI_F = 3.14159265358979323846;

const int MORSE_UNIT = 200;

void morsePulse(int duration) {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(duration);
  digitalWrite(LED_BUILTIN, LOW);
  delay(MORSE_UNIT);
}

void morseDot() {
  morsePulse(MORSE_UNIT);
}

void morseDash() {
  morsePulse(MORSE_UNIT * 3);
}

void morseLetterSpace() {
  delay(MORSE_UNIT * 2);
}

void morseWordSpace() {
  delay(MORSE_UNIT * 6);
}

void playMorseMessage() {
  morseDot(); morseDot(); morseLetterSpace();
  morseDot(); morseDash(); morseDot(); morseDot(); morseLetterSpace();
  morseDash(); morseDot(); morseDash(); morseDash();
  morseWordSpace();
  morseDot(); morseDash(); morseDot(); morseDot(); morseLetterSpace();
  morseDot(); morseDash(); morseLetterSpace();
  morseDash(); morseDot(); morseDash(); morseLetterSpace();
  morseDot(); morseDot(); morseDot(); morseLetterSpace();
  morseDot(); morseDot(); morseDot(); morseDot(); morseLetterSpace();
  morseDot(); morseDash(); morseLetterSpace();
  morseDash(); morseDot(); morseLetterSpace();
  morseDot(); morseDash(); morseLetterSpace();
  morseDot(); morseDash(); morseLetterSpace();
}

void displaySensorData(float ax, float ay, float az, float gx, float gy, float gz) {
  char buffer[12];
  lcd.setCursor(0, 0);
  lcd.print("A:");
  float totalA = sqrt(ax * ax + ay * ay + az * az);
  dtostrf(totalA, 4, 1, buffer);
  lcd.print(buffer);
  lcd.print("G ");
  lcd.setCursor(9, 0);
  lcd.print("G:");
  float totalG = sqrt(gx * gx + gy * gy + gz * gz);
  dtostrf(totalG, 4, 0, buffer);
  lcd.print(buffer);
  lcd.print("dps");
  float tiltAngle = atan2(ay, az) * 180.0 / PI_F;
  lcd.setCursor(0, 1);
  lcd.print("Tilt:");
  dtostrf(tiltAngle, 4, 0, buffer);
  lcd.print(buffer);
  lcd.print((char)223);
  lcd.print("   ");
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" <= BlackBox => ");
  delay(1500);
  pinMode(LED_BUILTIN, OUTPUT);
  gyro.initialize();
  if (!gyro.testConnection()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("L3G4200D Not Found");
    while (1);
  }
  gyro.setFullScale(L3G4200D_FS_500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating...");
  lcd.setCursor(0, 1);
  lcd.print("Keep bike level!");
  const int num_samples = 1000;
  long accel_x_sum = 0;
  long accel_y_sum = 0;
  long accel_z_sum = 0;
  long gyro_x_sum = 0;
  long gyro_y_sum = 0;
  long gyro_z_sum = 0;
  for (int i = 0; i < num_samples; i++) {
    accel_x_sum += analogRead(ACCEL_X_PIN);
    accel_y_sum += analogRead(ACCEL_Y_PIN);
    accel_z_sum += analogRead(ACCEL_Z_PIN);
    int16_t raw_gx, raw_gy, raw_gz;
    gyro.getAngularVelocity(&raw_gx, &raw_gy, &raw_gz);
    gyro_x_sum += raw_gx;
    gyro_y_sum += raw_gy;
    gyro_z_sum += raw_gz;
    delay(2);
  }
  ACCEL_X_ZERO = accel_x_sum / num_samples;
  ACCEL_Y_ZERO = accel_y_sum / num_samples;
  ACCEL_Z_ZERO = (accel_z_sum / num_samples) - ACCEL_SENSITIVITY;
  gyro_x_offset = (float)gyro_x_sum / num_samples;
  gyro_y_offset = (float)gyro_y_sum / num_samples;
  gyro_z_offset = (float)gyro_z_sum / num_samples;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibration OK!");
  delay(1500);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Starting");
  //playMorseMessage();
  delay(100);
  lcd.clear();
  lcd.print("System Ready");
  delay(500);
  lcd.clear();
}

void loop() {
  float accelX_g = (float)(analogRead(ACCEL_X_PIN) - ACCEL_X_ZERO) / ACCEL_SENSITIVITY;
  float accelY_g = (float)(analogRead(ACCEL_Y_PIN) - ACCEL_Y_ZERO) / ACCEL_SENSITIVITY;
  float accelZ_g = (float)(analogRead(ACCEL_Z_PIN) - ACCEL_Z_ZERO) / ACCEL_SENSITIVITY;
  int16_t raw_gx, raw_gy, raw_gz;
  gyro.getAngularVelocity(&raw_gx, &raw_gy, &raw_gz);
  float gyroX_dps = ((float)raw_gx - gyro_x_offset) * 0.0175;
  float gyroY_dps = ((float)raw_gy - gyro_y_offset) * 0.0175;
  float gyroZ_dps = ((float)raw_gz - gyro_z_offset) * 0.0175;
  float totalAccelerationG = sqrt(accelX_g * accelX_g + accelY_g * accelY_g + accelZ_g * accelZ_g);
  float totalRotationDPS = sqrt(gyroX_dps * gyroX_dps + gyroY_dps * gyroY_dps + gyroZ_dps * gyroZ_dps);
  
  Serial.print(accelX_g); 
  Serial.print(","); 
  Serial.print(accelY_g); 
  Serial.print(","); 
  Serial.print(accelZ_g); 
  Serial.print(","); 
  Serial.print(gyroX_dps); 
  Serial.print(","); 
  Serial.print(gyroY_dps); 
  Serial.print(","); 
  Serial.print(gyroZ_dps); 
  Serial.print(","); 
  Serial.println((totalAccelerationG > IMPACT_THRESHOLD_G || totalRotationDPS > ROTATION_THRESHOLD_DPS) ? "CRASH" : "SAFE");
  
  if (totalAccelerationG > IMPACT_THRESHOLD_G || totalRotationDPS > ROTATION_THRESHOLD_DPS) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("CRASH DETECTED");
    lcd.setCursor(0, 1);
    lcd.print("CALLING HELP!");
    delay(2000);
    lcd.clear();
  } else {
    displaySensorData(accelX_g, accelY_g, accelZ_g, gyroX_dps, gyroY_dps, gyroZ_dps);
  }
  delay(50);
}