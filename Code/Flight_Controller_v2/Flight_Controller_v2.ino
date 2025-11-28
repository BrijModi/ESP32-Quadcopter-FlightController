
#include <Wire.h>
#include <ESP32Servo.h> 

// ---------- Global Variables ---------- //
volatile float RatePitch, RateRoll, RateYaw;
float RateCalibrationPitch, RateCalibrationRoll, RateCalibrationYaw;
float AccXCalibration, AccYCalibration, AccZCalibration;

int ESCfreq = 500; // ESC PWM frequency (Hz)

// ----- PID constants (Angle) ----- //
float PAngleRoll=3.6;
float IAngleRoll=0.01;
float DAngleRoll=0.03;

float PAnglePitch=PAngleRoll;
float IAnglePitch=IAngleRoll;
float DAnglePitch=DAngleRoll;

// ----- PID constants (Rate) ----- //
float PRateRoll = 1.25;
float IRateRoll = 0.02;
float DRateRoll = 0.0086;

float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 3.8;
float IRateYaw = 3;
float DRateYaw = 0.02;

uint32_t LoopTimer;
float t = 0.012; // Loop time (≈83Hz)

// ----- Motor setup (X configuration) ----- //
Servo mot1, mot2, mot3, mot4;
// mot1 = FR (CCW), mot2 = RR (CW), mot3 = RL (CCW), mot4 = FL (CW)
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

// ----- Receiver setup ----- //
volatile uint32_t current_time;
volatile uint32_t last_channel_1 = 0, last_channel_2 = 0, last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0, last_channel_5 = 0, last_channel_6 = 0;
volatile uint32_t timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
volatile int ReceiverValue[6]; // Channels 1–6

// Receiver input pins
const int channel_1_pin = 34; // Roll
const int channel_2_pin = 35; // Pitch
const int channel_3_pin = 32; // Throttle
const int channel_4_pin = 33; // Yaw
const int channel_5_pin = 25; // Aux1 (Arm)
const int channel_6_pin = 26; // Aux2

// ----- PID variables ----- //
volatile float PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw, ItermYaw, DtermYaw, PIDOutputYaw;  

int ThrottleIdle = 1170;
int ThrottleCutOff = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = {0, 0, 0};

volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

// ---------- Receiver Interrupt Handler ---------- //
void channelInterruptHandler() {
  current_time = micros();

  // Channel 1 (Roll)
  if (digitalRead(channel_1_pin)) {
    if (!last_channel_1) { last_channel_1 = 1; timer_1 = current_time; }
  } else if (last_channel_1) {
    last_channel_1 = 0; ReceiverValue[0] = current_time - timer_1;
  }

  // Channel 2 (Pitch)
  if (digitalRead(channel_2_pin)) {
    if (!last_channel_2) { last_channel_2 = 1; timer_2 = current_time; }
  } else if (last_channel_2) {
    last_channel_2 = 0; ReceiverValue[1] = current_time - timer_2;
  }

  // Channel 3 (Throttle)
  if (digitalRead(channel_3_pin)) {
    if (!last_channel_3) { last_channel_3 = 1; timer_3 = current_time; }
  } else if (last_channel_3) {
    last_channel_3 = 0; ReceiverValue[2] = current_time - timer_3;
  }

  // Channel 4 (Yaw)
  if (digitalRead(channel_4_pin)) {
    if (!last_channel_4) { last_channel_4 = 1; timer_4 = current_time; }
  } else if (last_channel_4) {
    last_channel_4 = 0; ReceiverValue[3] = current_time - timer_4;
  }

  // Channel 5 (Aux1)
  if (digitalRead(channel_5_pin)) {
    if (!last_channel_5) { last_channel_5 = 1; timer_5 = current_time; }
  } else if (last_channel_5) {
    last_channel_5 = 0; ReceiverValue[4] = current_time - timer_5;
  }

  // Channel 6 (Aux2)
  if (digitalRead(channel_6_pin)) {
    if (!last_channel_6) { last_channel_6 = 1; timer_6 = current_time; }
  } else if (last_channel_6) {
    last_channel_6 = 0; ReceiverValue[5] = current_time - timer_6;
  }
}

// ---------- IMU Data Acquisition ---------- //
void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

  // --- Read Accelerometer --- //
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  // --- Read Gyroscope --- //
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  // Convert to engineering units
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096;
  AccY = (float)AccYLSB / 4096;
  AccZ = (float)AccZLSB / 4096;

  // Compute angles from accelerometer
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
}

// ---------- PID Equation ---------- //
void pid_equation(float Error, float P, float I, float D, float PrevError, float PrevIterm) {
  float Pterm = P * Error;
  float Iterm = PrevIterm + (I * (Error + PrevError) * (t / 2));
  Iterm = constrain(Iterm, -400, 400);
  float Dterm = D * ((Error - PrevError) / t);
  float PIDOutput = constrain(Pterm + Iterm + Dterm, -400, 400);

  PIDReturn[0] = PIDOutput;
  PIDReturn[1] = Error;
  PIDReturn[2] = Iterm;
}

// ---------- Setup ---------- //
void setup(void) {
  Serial.begin(115200);

  // LED Blink (Boot Indicator)
  int led_time = 100;
  pinMode(15, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
  }
  
  // Receiver setup
  pinMode(channel_1_pin, INPUT_PULLUP);
  pinMode(channel_2_pin, INPUT_PULLUP);
  pinMode(channel_3_pin, INPUT_PULLUP);
  pinMode(channel_4_pin, INPUT_PULLUP);
  pinMode(channel_5_pin, INPUT_PULLUP);
  pinMode(channel_6_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(channel_1_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_2_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_3_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_4_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_5_pin), channelInterruptHandler, CHANGE);
  attachInterrupt(digitalPinToInterrupt(channel_6_pin), channelInterruptHandler, CHANGE);
  delay(100);

  // IMU initialization
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); Wire.write(0x00);
  Wire.endTransmission();

  // ESC setup
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(1000);  // Wait before attaching first motor

  // Attach and configure each motor with proper ESC frequency
  mot1.attach(mot1_pin, 1000, 2000);
  delay(1000);
  mot1.setPeriodHertz(ESCfreq);
  delay(100);

  mot2.attach(mot2_pin, 1000, 2000);
  delay(1000);
  mot2.setPeriodHertz(ESCfreq);
  delay(100);

  mot3.attach(mot3_pin, 1000, 2000);
  delay(1000);
  mot3.setPeriodHertz(ESCfreq);
  delay(100);

  mot4.attach(mot4_pin, 1000, 2000);
  delay(1000);
  mot4.setPeriodHertz(ESCfreq);
  delay(100);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  delay(500);

  // LED Blink (Motor Attached Indicator)
  pinMode(15, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(15, HIGH);
    delay(led_time);
    digitalWrite(15, LOW);
    delay(led_time);
  }
  digitalWrite(15, LOW);
  delay(1000);
  digitalWrite(15, HIGH);

  // IMU calibration offsets (from Gyro_accelerometer_calibration.ino)
  RateCalibrationRoll=-1.58;
  RateCalibrationPitch=1.68;
  RateCalibrationYaw=-0.05;
  AccXCalibration=0.10;
  AccYCalibration=0.01;
  AccZCalibration=0.03;

  LoopTimer = micros();
}

// ---------- Main Loop ---------- //
void loop(void) {
  // --- Read IMU data --- //
  Wire.beginTransmission(0x68);
  Wire.write(0x1A); Wire.write(0x05);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x1C); Wire.write(0x10);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); 
  Wire.write(0x8);
  Wire.endTransmission();  

  Wire.beginTransmission(0x68);
  Wire.write(0x1B); Wire.write(0x8);
  Wire.endTransmission();

  Wire.beginTransmission(0x68);
  Wire.write(0x43); 
  Wire.endTransmission();
  
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX / 65.5 - RateCalibrationRoll;
  RatePitch = (float)GyroY / 65.5 - RateCalibrationPitch;
  RateYaw = (float)GyroZ / 65.5 - RateCalibrationYaw;

  AccX = (float)AccXLSB / 4096 - AccXCalibration;
  AccY = (float)AccYLSB / 4096 - AccYCalibration;
  AccZ = (float)AccZLSB / 4096 - AccZCalibration;

  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;

  // --- Complementary filter --- //
  complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll;
  complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch;

  // Clamp roll/pitch angles to ±20°
  complementaryAngleRoll = constrain(complementaryAngleRoll, -20, 20);
  complementaryAnglePitch = constrain(complementaryAnglePitch, -20, 20);

  // --- Desired values from receiver --- //
  DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle = ReceiverValue[2];
  DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500);

  // ----- Nested PID: Angle → Rate → Motor control ----- //
  // Angle → Rate (Roll)
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Angle → Rate (Pitch)
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // --- Rate PID for Roll, Pitch, Yaw --- //
  ErrorRateRoll = DesiredRateRoll - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw = DesiredRateYaw - RateYaw;

  // Roll Rate PID
  PtermRoll = PRateRoll * ErrorRateRoll;
  ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  InputRoll = PIDOutputRoll;
  PrevErrorRateRoll = ErrorRateRoll;
  PrevItermRateRoll = ItermRoll;

  // Pitch Rate PID
  PtermPitch = PRatePitch * ErrorRatePitch;
  ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  InputPitch = PIDOutputPitch;
  PrevErrorRatePitch = ErrorRatePitch;
  PrevItermRatePitch = ItermPitch;

  // Yaw Rate PID
  PtermYaw = PRateYaw * ErrorRateYaw;
  ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
  ItermYaw = constrain(ItermYaw, -400, 400);
  DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
  PIDOutputYaw = constrain(PtermYaw + ItermYaw + DtermYaw, -400, 400);
  InputYaw = PIDOutputYaw;
  PrevErrorRateYaw = ErrorRateYaw;
  PrevItermRateYaw = ItermYaw;

  // ----- Motor Mixing (X configuration) ----- //
  InputThrottle = constrain(InputThrottle, 1000, 1800);

  MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw); //Front Right - (CCW)
  MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw); //Rear Right - (CW)
  MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw); //Rear Left  - (CCW)
  MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw); //Front Left - (CW)

  // Clamp motor outputs to ESC range
  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  // ----- Safety: Disarm with AUX1 switch or throttle ----- //
  if (ReceiverValue[2] < 1030 || ReceiverValue[4] < 1500) {  // Throttle low OR AUX1 low
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;

    // Reset PID errors and integrals
    PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0;
    PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0;
    PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = 0; PrevItermAnglePitch = 0;
  }

  // ----- Send outputs to motors ----- //
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  // ----- Maintain fixed loop timing ----- //
  while (micros() - LoopTimer < (t * 1000000)) {
    // wait
  }
  LoopTimer = micros();
}

// PID Values =>
// PAngleRoll / PAnglePitch -> Strength of self-leveling when drone leans sideways (roll) or when nose dips or rises (pitch)
//      ↑ Increase: Levels faster but may oscillate
//      ↓ Decrease: Slower leveling, feels loose
// IAngleRoll / IAnglePitch -> Fixes slow roll / pitch drift over time
//      ↑ Increase: Better drift correction, may cause slow oscillation
//      ↓ Decrease: May drift or not hold level
// DAngleRoll / DAnglePitch -> Reduces wobble when correcting roll / pitch
//      ↑ Increase: Smoother motion, less overshoot, can cause motor jitter
//      ↓ Decrease: More wobble or bounce when leveling

// PRateRoll / PRatePitch -> How sharply it rolls/pitches on stick input
//      ↑ Increase: Faster, more responsive, may oscillate
//      ↓ Decrease: Slower, less responsive
// IRateRoll / IRatePitch -> Keeps constant roll/pitch rate, resists bias
//      ↑ Increase: Better rate hold, may cause slow oscillation
//      ↓ Decrease: Uneven or drifting rate
// DRateRoll / DRatePitch -> Smooths rapid roll/pitch movements
//      ↑ Increase: Smoother stops, reduces overshoot, may add noise
//      ↓ Decrease: Bouncy or overshooting stops

// PRateYaw -> How fast it rotates on yaw stick input
//      ↑ Increase: Faster yaw response, may oscillate
//      ↓ Decrease: Sluggish yaw
// IRateYaw -> Keeps heading steady, resists drift
//      ↑ Increase: Better heading hold, may cause slow oscillation
//      ↓ Decrease: Yaw drift over time
// DRateYaw -> Prevents overshoot in turns
//      ↑ Increase: Smoother yaw stop, can add delay or noise
//      ↓ Decrease: Overshoots or jerky yaw stops

