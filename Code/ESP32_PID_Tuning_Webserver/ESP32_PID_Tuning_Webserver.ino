// ============================================================================
//                                LIBRARIES
// ============================================================================
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <Wire.h>
#include <ESP32Servo.h>

// ============================================================================
//                              WIFI CREDENTIALS
// ============================================================================
const char* ssid     = "Modi";
const char* password = "Brij12345";

// ============================================================================
//                         GLOBAL PID CONSTANTS (INITIAL)
// ============================================================================
float PRateRoll = 1.25;
float IRateRoll = 0.02;
float DRateRoll = 0.0086;

float PAngleRoll = 3.6;
float IAngleRoll = 0.01;
float DAngleRoll = 0.03;

float PRateYaw = 3.8;
float IRateYaw = 3;
float DRateYaw = 0.02;

float ESCfreq = 500;

float PRatePitch  = PRateRoll;
float IRatePitch  = IRateRoll;
float DRatePitch  = DRateRoll;
float PAnglePitch = PAngleRoll;
float IAnglePitch = IAngleRoll;
float DAnglePitch = DAngleRoll;

float t = 0.012; // Loop cycle time
uint32_t LoopTimer;

// ============================================================================
//                          IMU + SENSOR GLOBAL VARIABLES
// ============================================================================
volatile float RateRoll, RatePitch, RateYaw;
volatile float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
volatile float AccXCalibration, AccYCalibration, AccZCalibration;

volatile float AccX, AccY, AccZ;
volatile float AngleRoll, AnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

// ============================================================================
//                           MOTOR CONFIGURATION
// ============================================================================
Servo mot1, mot2, mot3, mot4;
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;

int ThrottleIdle   = 1170;
int ThrottleCutOff = 1000;

// ============================================================================
//                           RECEIVER CONFIGURATION
// ============================================================================
volatile uint32_t current_time;

volatile uint32_t last_channel_1 = 0;
volatile uint32_t last_channel_2 = 0;
volatile uint32_t last_channel_3 = 0;
volatile uint32_t last_channel_4 = 0;
volatile uint32_t last_channel_5 = 0;
volatile uint32_t last_channel_6 = 0;

volatile uint32_t timer_1, timer_2, timer_3, timer_4, timer_5, timer_6;
volatile int ReceiverValue[6];

const int channel_1_pin = 34;
const int channel_2_pin = 35;
const int channel_3_pin = 32;
const int channel_4_pin = 33;
const int channel_5_pin = 25;
const int channel_6_pin = 26;

// ============================================================================
//                            PID WORKING VARIABLES
// ============================================================================
volatile float PtermRoll,  ItermRoll,  DtermRoll,  PIDOutputRoll;
volatile float PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch;
volatile float PtermYaw,   ItermYaw,   DtermYaw,   PIDOutputYaw;

volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;

volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float ErrorRateRoll,  ErrorRatePitch,  ErrorRateYaw;

volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;

volatile float PrevItermAngleRoll, PrevItermAnglePitch;
volatile float PrevItermRateRoll,  PrevItermRatePitch,  PrevItermRateYaw;

volatile float InputRoll, InputPitch, InputYaw, InputThrottle;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;

volatile float PIDReturn[] = {0, 0, 0};

// ============================================================================
//                         WIFI + PID TUNING SERVER
// ============================================================================
AsyncWebServer server(80);

const char* PARAM_P_GAIN = "pGain";
const char* PARAM_I_GAIN = "iGain";
const char* PARAM_D_GAIN = "dGain";

const char* PARAM_P_A_GAIN = "pAGain";
const char* PARAM_I_A_GAIN = "iAGain";
const char* PARAM_D_A_GAIN = "dAGain";

const char* PARAM_P_YAW = "pYaw";
const char* PARAM_I_YAW = "iYaw";
const char* PARAM_D_YAW = "dYaw";

const char* PARAM_TIME_CYCLE = "tc";

// ============================================================================
//                                HTML PAGE
// ============================================================================
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>ESP32 PID Tuning</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script>
    function submitMessage() {
      alert("Value saved!");
      setTimeout(() => { location.reload(); }, 400);
    }
  </script>
</head>
<body>
  <h3>ESP32 WiFi PID Tuning Panel</h3>

  <form action="/get" target="hidden-form">
    P Rate (Pitch/Roll) - %pGain%: <input type="number" name="pGain" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    I Rate (Pitch/Roll) - %iGain%: <input type="number" name="iGain" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    D Rate (Pitch/Roll) - %dGain%: <input type="number" name="dGain" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    P Angle - %pAGain%: <input type="number" name="pAGain" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    I Angle - %iAGain%: <input type="number" name="iAGain" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    D Angle - %dAGain%: <input type="number" name="dAGain" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    Yaw P - %pYaw%: <input type="number" name="pYaw" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    Yaw I - %iYaw%: <input type="number" name="iYaw" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    Yaw D - %dYaw%: <input type="number" name="dYaw" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <form action="/get" target="hidden-form">
    Loop Time (t) - %tc%: <input type="number" name="tc" step="any">
    <input type="submit" onclick="submitMessage()">
  </form>

  <iframe style="display:none" name="hidden-form"></iframe>
</body></html>
)rawliteral";

// ============================================================================
//                             FILE READ/WRITE
// ============================================================================
String readFile(fs::FS &fs, const char *path) {
  File file = fs.open(path, "r");
  if (!file) return "";
  String s = file.readString();
  file.close();
  return s;
}

void writeFile(fs::FS &fs, const char *path, const char *msg) {
  File file = fs.open(path, "w");
  if (!file) return;
  file.print(msg);
  file.close();
}

String processor(const String &var) {
  if (var == "pGain")  return readFile(SPIFFS, "/pGain.txt");
  if (var == "iGain")  return readFile(SPIFFS, "/iGain.txt");
  if (var == "dGain")  return readFile(SPIFFS, "/dGain.txt");
  if (var == "pAGain") return readFile(SPIFFS, "/pAGain.txt");
  if (var == "iAGain") return readFile(SPIFFS, "/iAGain.txt");
  if (var == "dAGain") return readFile(SPIFFS, "/dAGain.txt");
  if (var == "pYaw")   return readFile(SPIFFS, "/pYaw.txt");
  if (var == "iYaw")   return readFile(SPIFFS, "/iYaw.txt");
  if (var == "dYaw")   return readFile(SPIFFS, "/dYaw.txt");
  if (var == "tc")     return readFile(SPIFFS, "/tc.txt");
  return "";
}

// ============================================================================
//                         INTERRUPT: RECEIVER HANDLER
// ============================================================================
void IRAM_ATTR channelInterruptHandler() {
  current_time = micros();

  #define HANDLE_CH(N, PIN)                       \
    if (digitalRead(PIN)) {                        \
      if (!last_channel_##N) {                     \
        last_channel_##N = 1; timer_##N = current_time; \
      }                                            \
    } else if (last_channel_##N) {                 \
      last_channel_##N = 0;                        \
      ReceiverValue[N-1] = current_time - timer_##N; \
    }

  HANDLE_CH(1, channel_1_pin)
  HANDLE_CH(2, channel_2_pin)
  HANDLE_CH(3, channel_3_pin)
  HANDLE_CH(4, channel_4_pin)
  HANDLE_CH(5, channel_5_pin)
  HANDLE_CH(6, channel_6_pin)
}

// ============================================================================
//                               IMU READ FUNCTION
// ============================================================================
void gyro_signals() {
  Wire.beginTransmission(0x68); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission();

  Wire.beginTransmission(0x68); Wire.write(0x3B); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission();
  Wire.beginTransmission(0x68); Wire.write(0x43); Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll  = GyroX / 65.5;
  RatePitch = GyroY / 65.5;
  RateYaw   = GyroZ / 65.5;

  AccX = AccXLSB / 4096.0;
  AccY = AccYLSB / 4096.0;
  AccZ = AccZLSB / 4096.0;

  AngleRoll  = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 57.29;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 57.29;
}

// ============================================================================
//                               PID EQUATION
// ============================================================================
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

// ============================================================================
//                                  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);

  // --------------------- SPIFFS + WIFI ---------------------
  SPIFFS.begin(true);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  WiFi.waitForConnectResult();

  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  delay(1000);

  // --------------------- WEB ROUTES ------------------------
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send_P(200, "text/html", index_html, processor);
  });

  server.on("/get", HTTP_GET, [](AsyncWebServerRequest *req) {
    String input;
    if      (req->hasParam(PARAM_P_GAIN))  input = req->getParam(PARAM_P_GAIN)->value(),  writeFile(SPIFFS,"/pGain.txt",input.c_str());
    else if (req->hasParam(PARAM_I_GAIN))  input = req->getParam(PARAM_I_GAIN)->value(),  writeFile(SPIFFS,"/iGain.txt",input.c_str());
    else if (req->hasParam(PARAM_D_GAIN))  input = req->getParam(PARAM_D_GAIN)->value(),  writeFile(SPIFFS,"/dGain.txt",input.c_str());
    else if (req->hasParam(PARAM_P_A_GAIN))input = req->getParam(PARAM_P_A_GAIN)->value(),writeFile(SPIFFS,"/pAGain.txt",input.c_str());
    else if (req->hasParam(PARAM_I_A_GAIN))input = req->getParam(PARAM_I_A_GAIN)->value(),writeFile(SPIFFS,"/iAGain.txt",input.c_str());
    else if (req->hasParam(PARAM_D_A_GAIN))input = req->getParam(PARAM_D_A_GAIN)->value(),writeFile(SPIFFS,"/dAGain.txt",input.c_str());
    else if (req->hasParam(PARAM_P_YAW))   input = req->getParam(PARAM_P_YAW)->value(),   writeFile(SPIFFS,"/pYaw.txt",input.c_str());
    else if (req->hasParam(PARAM_I_YAW))   input = req->getParam(PARAM_I_YAW)->value(),   writeFile(SPIFFS,"/iYaw.txt",input.c_str());
    else if (req->hasParam(PARAM_D_YAW))   input = req->getParam(PARAM_D_YAW)->value(),   writeFile(SPIFFS,"/dYaw.txt",input.c_str());
    else if (req->hasParam(PARAM_TIME_CYCLE)) input = req->getParam(PARAM_TIME_CYCLE)->value(), writeFile(SPIFFS,"/tc.txt",input.c_str());
    else input = "No message";

    req->send(200, "text/text", input);
  });

  server.begin();

  // ---------------------- LED BLINK -------------------------
  pinMode(15, OUTPUT);
  for (int i = 0; i < 8; i++) {
    digitalWrite(15, HIGH); delay(100);
    digitalWrite(15, LOW);  delay(100);
  }

  // ---------------------- RX PINS ---------------------------
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

  // ---------------------- IMU SETUP -------------------------
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // ---------------------- MOTOR INIT ------------------------
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  mot1.attach(mot1_pin, 1000, 2000); delay(1000); mot1.setPeriodHertz(ESCfreq);
  mot2.attach(mot2_pin, 1000, 2000); delay(1000); mot2.setPeriodHertz(ESCfreq);
  mot3.attach(mot3_pin, 1000, 2000); delay(1000); mot3.setPeriodHertz(ESCfreq);
  mot4.attach(mot4_pin, 1000, 2000); delay(1000); mot4.setPeriodHertz(ESCfreq);

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);

  RateCalibrationRoll  = -1.56;
  RateCalibrationPitch = 1.70;
  RateCalibrationYaw   = -0.04;
  AccXCalibration = 0.10;
  AccYCalibration = 0.00;
  AccZCalibration = 0.03;

  LoopTimer = micros();
}

// ============================================================================
//                                    LOOP
// ============================================================================
void loop() {

  // ---------------- WIFI TUNING UPLOAD ----------------
  if (ReceiverValue[4] > 1500) {
    PRateRoll = readFile(SPIFFS, "/pGain.txt").toFloat();
    IRateRoll = readFile(SPIFFS, "/iGain.txt").toFloat();
    DRateRoll = readFile(SPIFFS, "/dGain.txt").toFloat();

    PRatePitch = PRateRoll;
    IRatePitch = IRateRoll;
    DRatePitch = DRateRoll;

    PAngleRoll = readFile(SPIFFS, "/pAGain.txt").toFloat();
    IAngleRoll = readFile(SPIFFS, "/iAGain.txt").toFloat();
    DAngleRoll = readFile(SPIFFS, "/dAGain.txt").toFloat();

    PAnglePitch = PAngleRoll;
    IAnglePitch = IAngleRoll;
    DAnglePitch = DAngleRoll;

    PRateYaw = readFile(SPIFFS, "/pYaw.txt").toFloat();
    IRateYaw = readFile(SPIFFS, "/iYaw.txt").toFloat();
    DRateYaw = readFile(SPIFFS, "/dYaw.txt").toFloat();

    t = readFile(SPIFFS, "/tc.txt").toFloat();
  }

  // ---------------------- IMU READ -----------------------
  gyro_signals();

  RateRoll  -= RateCalibrationRoll;
  RatePitch -= RateCalibrationPitch;
  RateYaw   -= RateCalibrationYaw;

  AccX -= AccXCalibration;
  AccY -= AccYCalibration;
  AccZ -= AccZCalibration;

  // ---------------- COMPLEMENTARY FILTER ----------------
  complementaryAngleRoll =
      0.991 * (complementaryAngleRoll + RateRoll * t) +
      0.009 * AngleRoll;

  complementaryAnglePitch =
      0.991 * (complementaryAnglePitch + RatePitch * t) +
      0.009 * AnglePitch;

  complementaryAngleRoll  = constrain(complementaryAngleRoll,  -20, 20);
  complementaryAnglePitch = constrain(complementaryAnglePitch, -20, 20);

  // ---------------- RX INPUT PROCESSING -----------------
  DesiredAngleRoll  = 0.1 * (ReceiverValue[0] - 1500);
  DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500);
  InputThrottle     = ReceiverValue[2];
  DesiredRateYaw    = 0.15 * (ReceiverValue[3] - 1500);

  // ---------------- ANGLE → RATE PID --------------------
  // Roll
  ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
  PtermRoll = PAngleRoll * ErrorAngleRoll;
  ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
  ItermRoll = constrain(ItermRoll, -400, 400);
  DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
  PIDOutputRoll = constrain(PtermRoll + ItermRoll + DtermRoll, -400, 400);
  DesiredRateRoll = PIDOutputRoll;
  PrevErrorAngleRoll = ErrorAngleRoll;
  PrevItermAngleRoll = ItermRoll;

  // Pitch
  ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
  PtermPitch = PAnglePitch * ErrorAnglePitch;
  ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
  ItermPitch = constrain(ItermPitch, -400, 400);
  DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
  PIDOutputPitch = constrain(PtermPitch + ItermPitch + DtermPitch, -400, 400);
  DesiredRatePitch = PIDOutputPitch;
  PrevErrorAnglePitch = ErrorAnglePitch;
  PrevItermAnglePitch = ItermPitch;

  // ---------------- RATE PID ----------------------------
  ErrorRateRoll  = DesiredRateRoll  - RateRoll;
  ErrorRatePitch = DesiredRatePitch - RatePitch;
  ErrorRateYaw   = DesiredRateYaw   - RateYaw;

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

  // ---------------- MOTOR MIXER -------------------------
  InputThrottle = constrain(InputThrottle, 1000, 1800);

  MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw; // FR CCW
  MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw; // RR CW
  MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw; // RL CCW
  MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw; // FL CW

  MotorInput1 = constrain(MotorInput1, ThrottleIdle, 1999);
  MotorInput2 = constrain(MotorInput2, ThrottleIdle, 1999);
  MotorInput3 = constrain(MotorInput3, ThrottleIdle, 1999);
  MotorInput4 = constrain(MotorInput4, ThrottleIdle, 1999);

  // ---------------- SAFETY: DISARM ----------------------
  if (ReceiverValue[2] < 1030) {
    MotorInput1 = ThrottleCutOff;
    MotorInput2 = ThrottleCutOff;
    MotorInput3 = ThrottleCutOff;
    MotorInput4 = ThrottleCutOff;

    PrevErrorRateRoll = PrevErrorRatePitch = PrevErrorRateYaw = 0;
    PrevItermRateRoll = PrevItermRatePitch = PrevItermRateYaw = 0;
    PrevErrorAngleRoll = PrevErrorAnglePitch = 0;
    PrevItermAngleRoll = PrevItermAnglePitch = 0;
  }

  // ---------------- WRITE MOTOR OUTPUT ------------------
  mot1.writeMicroseconds(MotorInput1);
  mot2.writeMicroseconds(MotorInput2);
  mot3.writeMicroseconds(MotorInput3);
  mot4.writeMicroseconds(MotorInput4);

  // ---------------- LOOP TIMING -------------------------
  while (micros() - LoopTimer < (t * 1000000));
  LoopTimer = micros();
}
