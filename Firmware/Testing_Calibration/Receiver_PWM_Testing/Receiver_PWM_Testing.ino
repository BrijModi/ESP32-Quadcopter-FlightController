// ------------------------------------------------------
// Variables
// ------------------------------------------------------
volatile uint32_t current_time;

volatile uint32_t last_state[6] = {0};
volatile uint32_t start_time[6] = {0};
volatile int ReceiverValue[6] = {0};

// Pin mapping for 6 channels
const int channel_pins[6] = {34, 35, 32, 33, 25, 26};


// ------------------------------------------------------
// Interrupt Handler
// ------------------------------------------------------
void IRAM_ATTR channelInterruptHandler() {
  current_time = micros();

  for (int i = 0; i < 6; i++) {
    int pinState = digitalRead(channel_pins[i]);

    if (pinState) {
      if (last_state[i] == 0) {
        last_state[i] = 1;
        start_time[i] = current_time;
      }
    } 
    else {
      if (last_state[i] == 1) {
        last_state[i] = 0;
        ReceiverValue[i] = current_time - start_time[i];
      }
    }
  }
}


// ------------------------------------------------------
// Neutral Deadband Adjustment (Optional)
// ------------------------------------------------------
void neutralPositionAdjustment() {
  int min = 1490;
  int max = 1510;

  for (int i = 0; i < 4; i++) {
    if (ReceiverValue[i] > min && ReceiverValue[i] < max) {
      ReceiverValue[i] = 1500;
    }
  }

  if (ReceiverValue[0] == ReceiverValue[1] &&
      ReceiverValue[1] == ReceiverValue[3]) 
  {
    ReceiverValue[0] = ReceiverValue[1] = ReceiverValue[3] = 1500;
  }
}


// ------------------------------------------------------
// Setup
// ------------------------------------------------------
void setup() {
  Serial.begin(115200);

  // Configure pins + interrupts
  for (int i = 0; i < 6; i++) {
    pinMode(channel_pins[i], INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(channel_pins[i]), channelInterruptHandler, CHANGE);
  }
}


// ------------------------------------------------------
// Loop: Print Receiver Values
// ------------------------------------------------------
void loop() {

  Serial.print("CH1: "); Serial.print(ReceiverValue[0]);
  Serial.print("  CH2: "); Serial.print(ReceiverValue[1]);
  Serial.print("  CH3: "); Serial.print(ReceiverValue[2]);
  Serial.print("  CH4: "); Serial.print(ReceiverValue[3]);
  Serial.print("  CH5: "); Serial.print(ReceiverValue[4]);
  Serial.print("  CH6: "); Serial.println(ReceiverValue[5]);

  delay(300);
}
