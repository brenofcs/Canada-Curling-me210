// ============================================================
// ME210 Curling DROID — Post-Exit Test Sketch
// ============================================================

// ---- PIN DEFINITIONS ---------------------------------------

// left motor
const int L_IN1 = 8;
const int L_IN2 = 5;
const int L_ENA = 9;

// right motor — physically wired in reverse on H-bridge,
// so IN logic is flipped relative to left motor
const int R_IN1 = 6;
const int R_IN2 = 7;
const int R_ENA = 10;

// rack & pinion motor
const int RP_IN1 = 12;
const int RP_IN2 = 13;
const int RP_EN  = 11;

// tape sensors
const int TAPE_CENTER_MID  = 2;
const int TAPE_CENTER_REAR = A5;
const int TAPE_HOG_LEFT    = 3;
const int TAPE_HOG_RIGHT   = 4;

//ir sonsors
const int IR_SENSOR_RIGHT = A0;
const int IR_SENSOR_LEFT = A1;

// ---- SPEED CONSTANTS (TUNE) --------------------------------

const int DRIVE_SPEED = 160;
const int FAST_SPEED = 190;
const int TURN_SPEED  = 140;
const int RACK_SPEED  = 235;

// ---- MOTOR TRIM --------------------------------------------
// Right motor runs faster than left. Scale it down until bot
// drives straight. Start at 1.0 and decrease in steps of 0.05.
// Example: 0.90 means right motor runs at 90% of commanded speed.
const float R_TRIM = 0.8; // TUNE THIS

// ---- TIMING CONSTANTS (TUNE) -------------------------------

const unsigned long TURN_90_TIME     = 1000; // ms — tune on actual arena surface
const unsigned long RACK_EXTEND_TIME = 2000;  // ms to extend rack & pinion
const unsigned long EXIT_START_TIME = 4000; // ms 

// ---- TAPE SENSOR DEFINES -----------------------------------

#define ON_TAPE  HIGH
#define OFF_TAPE LOW

int prevState;

// IR SENSING STUFF 

const float TARGET_FREQ      = 909.0;
const int   SAMPLE_RATE      = 9090;
const int   N_SAMPLES        = 50;
const float DETECT_THRESHOLD = 1000.0;

float goertzelCoeff;

// ── Internal state ────────────────────────────────────────────────────────────
static bool a0_triggered = false;

// ---- STATE MACHINE -----------------------------------------

enum State {
  STATE_FIND_BEARING,
  STATE_EXIT_START,
  STATE_TURN_RIGHT_90,
  STATE_FIND_CENTERLINE,
  STATE_ALIGN_REAR,
  STATE_ADVANCE_TO_HOG,
  STATE_LAUNCH,
  STATE_DONE
};

State currentState;
unsigned long stateStartTime = 0;

// ---- TAPE SENSOR -------------------------------------------

bool onTape(int pin) {
  return digitalRead(pin) == ON_TAPE;
}

// ---- MOTOR HELPERS -----------------------------------------
// Left motor:  IN1=HIGH, IN2=LOW  → forward
//              IN1=LOW,  IN2=HIGH → backward
// Right motor: IN1=LOW,  IN2=HIGH → forward  (wired in reverse)
//              IN1=HIGH, IN2=LOW  → backward (wired in reverse)

void setLeftMotor(int speed) {
  if (speed > 0) {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, LOW);
    analogWrite(L_ENA, speed);
  } else if (speed < 0) {
    digitalWrite(L_IN1, LOW);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_ENA, -speed);
  } else {
    digitalWrite(L_IN1, HIGH);
    digitalWrite(L_IN2, HIGH);
    analogWrite(L_ENA, 0);
  }
}

void setRightMotor(int speed) {
  int trimmedSpeed = (int)(abs(speed) * R_TRIM); // apply trim to magnitude only
  if (speed > 0) {
    digitalWrite(R_IN1, LOW);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_ENA, trimmedSpeed);
  } else if (speed < 0) {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, LOW);
    analogWrite(R_ENA, trimmedSpeed);
  } else {
    digitalWrite(R_IN1, HIGH);
    digitalWrite(R_IN2, HIGH);
    analogWrite(R_ENA, 0);
  }
}

// ---- MOTOR CONTROL -----------------------------------------

void driveForward(int speedLeft, int speedRight) {
  setLeftMotor(speedLeft);
  setRightMotor(speedRight);
}

void driveBackward(int speedLeft, int speedRight) {
  setLeftMotor(-speedLeft);
  setRightMotor(-speedRight);
}

void rotateRight(int speedLeft, int speedRight) {
  setLeftMotor(speedLeft);
  setRightMotor(-speedRight);
}

void rotateLeft(int speedLeft, int speedRight) {
  setLeftMotor(-speedLeft);
  setRightMotor(speedRight);
}

void stopDrive() {
  setLeftMotor(0);
  setRightMotor(0);
}

void extendArm(int speed) {
  analogWrite(RP_EN, speed);
  digitalWrite(RP_IN1, LOW);
  digitalWrite(RP_IN2, HIGH);
}

void stopArm() {
  analogWrite(RP_EN, 0);
  digitalWrite(RP_IN1, LOW);
  digitalWrite(RP_IN2, LOW);
}

// ── Sample one pin and return Goertzel power ──────────────────────────────────
float samplePin(int pin) {
  int samples[N_SAMPLES];

  for (int i = 0; i < N_SAMPLES; i++) {
    unsigned long t = micros();
    samples[i] = analogRead(pin);
    while (micros() - t < 110);
  }

  long sum = 0;
  for (int i = 0; i < N_SAMPLES; i++) sum += samples[i];
  int dc = (int)(sum / N_SAMPLES);
  for (int i = 0; i < N_SAMPLES; i++) samples[i] -= dc;

  float s_prev  = 0.0;
  float s_prev2 = 0.0;
  for (int i = 0; i < N_SAMPLES; i++) {
    float s = (float)samples[i] + goertzelCoeff * s_prev - s_prev2;
    s_prev2 = s_prev;
    s_prev  = s;
  }
  return s_prev * s_prev + s_prev2 * s_prev2 - goertzelCoeff * s_prev * s_prev2;
}

/*
 * checkIRGate()
 * 
 * Call this repeatedly from your main loop.
 * Returns true once when the sequence A0 → A1 is completed.
 * Automatically resets after returning true.
 * 
 * State machine:
 *   IDLE         → A0 detects → WAITING_A1
 *   WAITING_A1   → A1 detects → return true, reset to IDLE
 *   WAITING_A1   → A0 detects → stay in WAITING_A1 (A0 still blocked)
 */
bool bearingFound() {
  bool a0 = (samplePin(A0) > DETECT_THRESHOLD);
  bool a1 = (samplePin(A1) > DETECT_THRESHOLD);

  if (!a0_triggered) {
    // Waiting for A0 to fire first
    if (a0) {
      a0_triggered = true;
      Serial.println(F("A0 triggered — waiting for A1..."));
    }
  } else {
    // A0 has fired, now watching for A1
    if (a1) {
      a0_triggered = false;   // reset for next pass
      Serial.println(F("A1 triggered — gate complete!"));
      return true;
    }
    // Optional: reset if A0 goes low then high again (re-entry)
    // Remove this block if you want to latch A0 indefinitely
    if (!a0) {
      // A0 cleared — still waiting for A1, no reset needed
    }
  }

  return false;
}

// ---- SETUP -------------------------------------------------

void setup() {
  pinMode(L_IN1, OUTPUT);
  pinMode(L_IN2, OUTPUT);
  pinMode(L_ENA, OUTPUT);
  pinMode(R_IN1, OUTPUT);
  pinMode(R_IN2, OUTPUT);
  pinMode(R_ENA, OUTPUT);

  pinMode(RP_IN1, OUTPUT);
  pinMode(RP_IN2, OUTPUT);
  pinMode(RP_EN,  OUTPUT);

  pinMode(TAPE_CENTER_MID,  INPUT);
  pinMode(TAPE_CENTER_REAR, INPUT);
  pinMode(TAPE_HOG_LEFT,    INPUT);
  pinMode(TAPE_HOG_RIGHT,   INPUT);

  Serial.begin(115200);
  Serial.println("Post-exit test starting.");

  ADCSRA = (ADCSRA & 0xF8) | 0x04;

  int k = (int)(0.5 + ((float)N_SAMPLES * TARGET_FREQ / SAMPLE_RATE));
  float omega = (2.0 * PI * (float)k) / (float)N_SAMPLES;
  goertzelCoeff = 2.0 * cos(omega);

  currentState = STATE_FIND_BEARING;
  stateStartTime = millis();
  prevState = -2;
}

// ---- MAIN LOOP ---------------------------------------------

void loop() {
  switch (currentState) {

    case STATE_FIND_BEARING:
      if (prevState == -2) {
        rotateRight(TURN_SPEED, (int)(TURN_SPEED*0.8));
        prevState = -1;
      } 
      if (bearingFound()) {
        Serial.println("Bearing Found. Moving forward...");
        stateStartTime = millis();
        currentState = STATE_EXIT_START;
      }
    break;

    case STATE_EXIT_START:
      if (prevState = -1) {
        driveForward(255, 255);
        prevState = 0;
      }
      if (millis() - stateStartTime >= EXIT_START_TIME) {
        stopDrive();
        Serial.println("Done exiting start. Turning right...");
        stateStartTime = millis();
        currentState = STATE_TURN_RIGHT_90;
      }
    break;

    case STATE_TURN_RIGHT_90:
      if (prevState == 0) {
        rotateRight(TURN_SPEED, (int)(TURN_SPEED*0.8));
        prevState = 1;
      }
      if (millis() - stateStartTime >= TURN_90_TIME) {
        stopDrive();
        Serial.println("Turned 90. Seeking center line...");
        Serial.print("  Tape snapshot — MID: ");
        Serial.print(onTape(TAPE_CENTER_MID)  ? "ON" : "off");
        Serial.print("  REAR: ");
        Serial.println(onTape(TAPE_CENTER_REAR) ? "ON" : "off");
        stateStartTime = millis();
        currentState = STATE_FIND_CENTERLINE;
      }
      break;

    case STATE_FIND_CENTERLINE:
      if (onTape(TAPE_CENTER_MID)) {
        stopDrive();
        Serial.println("Center line found. Aligning rear sensor...");
        Serial.print("  Tape snapshot — MID: ");
        Serial.print(onTape(TAPE_CENTER_MID)  ? "ON" : "off");
        Serial.print("  REAR: ");
        Serial.println(onTape(TAPE_CENTER_REAR) ? "ON" : "off");
        stateStartTime = millis();
        currentState = STATE_ALIGN_REAR;
      } else {
        if (prevState == 1) {
          driveForward(FAST_SPEED, (int)(FAST_SPEED*0.85));
          prevState = 2;
        }
      }
      break;

    case STATE_ALIGN_REAR:
      if (onTape(TAPE_CENTER_REAR)) {
        stopDrive();
        Serial.println("Aligned on center line. Advancing to hog line...");
        Serial.print("  Tape snapshot — MID: ");
        Serial.print(onTape(TAPE_CENTER_MID)  ? "ON" : "off");
        Serial.print("  REAR: ");
        Serial.println(onTape(TAPE_CENTER_REAR) ? "ON" : "off");
        stateStartTime = millis();
        currentState = STATE_ADVANCE_TO_HOG;
      } else {
        if (prevState == 2) {
          rotateLeft(TURN_SPEED, (int)(TURN_SPEED*0.7));
          prevState = 3;
        }
      }
      break;

    case STATE_ADVANCE_TO_HOG:
      if (onTape(TAPE_HOG_LEFT) || onTape(TAPE_HOG_RIGHT)) {
        stopDrive();
        Serial.println("Hog line reached. Launching puck...");
        Serial.println("  Tape snapshot — HOG_L: ON  HOG_R: ON");
        stateStartTime = millis();
        currentState = STATE_LAUNCH;
      } else {
        if (prevState = 4) {
          driveForward(FAST_SPEED, (int)(FAST_SPEED*1.05));
          prevState = 5;
        }
      }
      break;

    case STATE_LAUNCH:
      if (prevState == 5) {  
        extendArm(RACK_SPEED);
        prevState = 6;
      }
      if (millis() - stateStartTime >= RACK_EXTEND_TIME) {
        stopArm();
        Serial.println("Puck launched. Test complete.");
        currentState = STATE_DONE;
      }
      break;

    case STATE_DONE:
      stopDrive();
      stopArm();
      break;
  }
}