#include <Wire.h>

int16_t AccXLSB, AccYLSB, AccZLSB;
int16_t GyroX, GyroY, GyroZ;

float AccX, AccY, AccZ;
volatile float RatePitch, RateRoll, RateYaw;
volatile float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
volatile float AccXCalibration, AccYCalibration, AccZCalibration;

int RateCalibrationNumber;

// ------------------------------------------------------
// Utility: Clear Serial Buffer
// ------------------------------------------------------
void clearSerial() {
  while (Serial.available()) Serial.read();
}

// ------------------------------------------------------
// SETUP
// ------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(100);

  Serial.println("Place FC on a flat surface and press Enter to begin calibration...");
  clearSerial();
  while (!Serial.available()) {}
  clearSerial();

  // Perform simple calibration
  calibrateGyroSimple();

  Serial.println("\nCalibration complete:");
  Serial.print("RateCalibrationRoll=");  Serial.println(RateCalibrationRoll);
  Serial.print("RateCalibrationPitch="); Serial.println(RateCalibrationPitch);
  Serial.print("RateCalibrationYaw=");   Serial.println(RateCalibrationYaw);
  Serial.print("AccXCalibration=");      Serial.println(AccXCalibration);
  Serial.print("AccYCalibration=");      Serial.println(AccYCalibration);
  Serial.print("AccZCalibration=");      Serial.println(AccZCalibration);

  Serial.println("\nPress Enter to stream calibrated values...");
  clearSerial();
  while (!Serial.available()) {}
  clearSerial();
}

// ------------------------------------------------------
// MAIN LOOP
// ------------------------------------------------------
void loop() {

  gyro_signals();

  // Apply calibration
  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // Stream data
  Serial.print("AccX: "); Serial.print(AccX);
  Serial.print("  AccY: "); Serial.print(AccY);
  Serial.print("  AccZ: "); Serial.print(AccZ);

  Serial.print("  |  GyroX: "); Serial.print(RateRoll);
  Serial.print("  GyroY: "); Serial.print(RatePitch);
  Serial.print("  GyroZ: "); Serial.println(RateYaw);

  delay(50);
}

// ------------------------------------------------------
// Read MPU6050 Accelerometer + Gyroscope
// ------------------------------------------------------
void gyro_signals() {
  // ACCEL setup
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();

  // Read ACCEL
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  AccXLSB = Wire.read() << 8 | Wire.read();
  AccYLSB = Wire.read() << 8 | Wire.read();
  AccZLSB = Wire.read() << 8 | Wire.read();

  // GYRO setup
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08);
  Wire.endTransmission();

  // Read GYRO
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);

  GyroX = Wire.read() << 8 | Wire.read();
  GyroY = Wire.read() << 8 | Wire.read();
  GyroZ = Wire.read() << 8 | Wire.read();

  // Convert raw to real values
  RateRoll  = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw   = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;
}

// ------------------------------------------------------
// Calibrate Gyro + Acc (simple method)
// ------------------------------------------------------
void calibrateGyroSimple() {
  RateCalibrationRoll  = 0;
  RateCalibrationPitch = 0;
  RateCalibrationYaw   = 0;
  AccXCalibration      = 0;
  AccYCalibration      = 0;
  AccZCalibration      = 0;

  for (RateCalibrationNumber = 0; RateCalibrationNumber < 2000; RateCalibrationNumber++) {
    gyro_signals();

    RateCalibrationRoll  += RateRoll;
    RateCalibrationPitch += RatePitch;
    RateCalibrationYaw   += RateYaw;

    AccXCalibration += AccX;
    AccYCalibration += AccY;
    AccZCalibration += AccZ;

    delay(1);
  }

  RateCalibrationRoll  /= 2000;
  RateCalibrationPitch /= 2000;
  RateCalibrationYaw   /= 2000;

  AccXCalibration /= 2000;
  AccYCalibration /= 2000;
  AccZCalibration /= 2000;

  // Z-axis should read +1g when flat
  AccZCalibration -= 1;
}
