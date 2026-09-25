// AIBOMECH AgroBot motor-controller firmware.
//
// Target: Arduino Mega 2560 (also builds for ESP32 / Teensy 4 with the Servo
// library of that core). Talks to aibomech_agrobot_hardware over USB serial,
// see include/aibomech_agrobot_hardware/protocol.hpp for the frame format.
//
// Channels 0-4: hobby/industrial PWM servos for joint_1..joint_4 and the
//               gripper jaw. Open loop, so the reported position is the
//               slew-limited setpoint. For true feedback use smart bus servos
//               (Feetech STS / Dynamixel) and report their measured position
//               in sendState() instead of the estimate.
// Channel 5:    stepper driver (STEP/DIR) for the rail trolley, homed on a
//               limit switch at the rail start.
//
// Safety:
//   * E-stop input (normally closed contact to GND). Open circuit = e-stop:
//     servo PWM detached, stepper disabled, latched until the host re-enables.
//   * Command watchdog: no setpoint for WATCHDOG_MS -> hold position.
//   * Setpoints are clamped to the joint limits and slew-rate limited here,
//     independent of what the host sends.

#include <Servo.h>

// ---------------------------------------------------------------- config ----
const uint32_t BAUD = 115200;
const uint16_t STATE_PERIOD_MS = 20;     // 50 Hz telemetry
const uint16_t WATCHDOG_MS = 250;
const uint8_t NUM_SERVOS = 5;
const uint8_t NUM_CHANNELS = 6;

const uint8_t PIN_ESTOP = 2;             // NC contact to GND, INPUT_PULLUP
const uint8_t PIN_RAIL_STEP = 22;
const uint8_t PIN_RAIL_DIR = 23;
const uint8_t PIN_RAIL_ENABLE = 24;      // active low on A4988/DRV8825/TMC2209
const uint8_t PIN_RAIL_HOME = 25;        // limit switch to GND at rail start
const uint8_t SERVO_PINS[NUM_SERVOS] = {3, 5, 6, 9, 10};

// Per-channel calibration: pulse width at 0 rad (or 0 m) and microseconds per
// unit. Tune these on the bench (docs/real_robot.md, "Step 5: Calibrate the joints").
const float SERVO_CENTER_US[NUM_SERVOS] = {1500, 1500, 1500, 1500, 1500};
const float SERVO_US_PER_UNIT[NUM_SERVOS] = {636.6, 636.6, 636.6, 636.6, 40000.0};  // 2000us/pi rad, 40us/mm
const float LIMIT_LO[NUM_CHANNELS] = {-1.09, -2.10, -2.00, -1.00, -0.017, 0.0};
const float LIMIT_HI[NUM_CHANNELS] = {2.10, 1.11, 1.10, 2.30, 0.010, 3.0};
const float MAX_SPEED[NUM_CHANNELS] = {2.0, 2.0, 2.0, 3.0, 0.05, 0.30};  // unit/s
const float RAIL_STEPS_PER_M = 5000.0;  // GT2 belt, 20T pulley, 1/8 microstepping
const float HOME_SPEED = 0.05;          // m/s

// ----------------------------------------------------------------- state ----
enum Status : uint32_t {
  ST_ENABLED = 1, ST_ESTOP = 2, ST_WATCHDOG = 4, ST_FAULT = 8, ST_RAIL_NOT_HOMED = 16
};

Servo servos[NUM_SERVOS];
float target[NUM_CHANNELS];
float position[NUM_CHANNELS];
float velocity[NUM_CHANNELS];
bool enabled = false;
bool estopLatched = false;
bool railHomed = false;
long railSteps = 0;
uint32_t sequence = 0;
uint32_t lastCommandMs = 0;
uint32_t lastStateMs = 0;
uint32_t lastUpdateUs = 0;
uint32_t lastStepUs = 0;
char line[160];
uint8_t lineLen = 0;

// --------------------------------------------------------------- helpers ----
uint8_t xorChecksum(const char *s, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; ++i) c ^= (uint8_t)s[i];
  return c;
}

void sendFrame(const char *payload) {
  char tail[6];
  snprintf(tail, sizeof(tail), "*%02X", xorChecksum(payload, strlen(payload)));
  Serial.print('$');
  Serial.print(payload);
  Serial.println(tail);
}

bool estopActive() { return digitalRead(PIN_ESTOP) == HIGH; }

void setDrives(bool on) {
  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    if (on && !servos[i].attached()) servos[i].attach(SERVO_PINS[i]);
    if (!on && servos[i].attached()) servos[i].detach();
  }
  digitalWrite(PIN_RAIL_ENABLE, on ? LOW : HIGH);
  enabled = on;
}

void sendState() {
  // Floats are formatted with dtostrf because AVR printf has no %f.
  static char payload[200];
  char num[16];
  uint32_t status = 0;
  if (enabled) status |= ST_ENABLED;
  if (estopLatched) status |= ST_ESTOP;
  if (enabled && millis() - lastCommandMs > WATCHDOG_MS) status |= ST_WATCHDOG;
  if (!railHomed) status |= ST_RAIL_NOT_HOMED;
  snprintf(payload, sizeof(payload), "S,%lu,%lu", (unsigned long)sequence++, (unsigned long)status);
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    strcat(payload, ",");
    strcat(payload, dtostrf(position[i], 1, 4, num));
  }
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    strcat(payload, ",");
    strcat(payload, dtostrf(velocity[i], 1, 4, num));
  }
  sendFrame(payload);
}

// ------------------------------------------------------------- commands ----
void handleLine(char *s) {
  // s = "$...*HH"
  char *star = strrchr(s, '*');
  if (s[0] != '$' || !star) return;
  *star = '\0';
  char *payload = s + 1;
  if (strtol(star + 1, NULL, 16) != xorChecksum(payload, strlen(payload))) return;

  char *tok = strtok(payload, ",");
  if (!tok) return;
  if (strcmp(tok, "C") == 0) {
    for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
      tok = strtok(NULL, ",");
      if (!tok) return;  // incomplete frame, ignore entirely
      target[i] = constrain(atof(tok), LIMIT_LO[i], LIMIT_HI[i]);
    }
    lastCommandMs = millis();
    return;  // setpoints are not acknowledged, the 50 Hz stream is enough
  } else if (strcmp(tok, "E") == 0) {
    tok = strtok(NULL, ",");
    bool on = tok && tok[0] == '1';
    if (on && estopActive()) return;   // cannot enable while e-stop is pressed
    if (on) {
      estopLatched = false;
      for (uint8_t i = 0; i < NUM_CHANNELS; ++i) target[i] = position[i];
      lastCommandMs = millis();
    }
    setDrives(on);
  }
  sendState();  // acknowledges E and answers P (ping)
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen) {
        line[lineLen] = '\0';
        handleLine(line);
        lineLen = 0;
      }
    } else if (lineLen < sizeof(line) - 1) {
      line[lineLen++] = c;
    } else {
      lineLen = 0;  // overflow, resynchronise on next newline
    }
  }
}

// --------------------------------------------------------------- motion ----
void updateMotion() {
  uint32_t now = micros();
  float dt = (now - lastUpdateUs) * 1e-6f;
  if (dt < 0.002f) return;  // 500 Hz
  lastUpdateUs = now;

  if (estopActive() && !estopLatched) {
    estopLatched = true;
    setDrives(false);
  }
  bool hold = !enabled || (millis() - lastCommandMs > WATCHDOG_MS);

  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    float goal = hold ? position[i] : target[i];
    if (i == 5 && !railHomed) goal = position[i];
    float step = constrain(goal - position[i], -MAX_SPEED[i] * dt, MAX_SPEED[i] * dt);
    position[i] += step;
    velocity[i] = step / dt;
  }
  for (uint8_t i = 0; i < NUM_SERVOS; ++i) {
    if (servos[i].attached()) {
      servos[i].writeMicroseconds((int)(SERVO_CENTER_US[i] + position[i] * SERVO_US_PER_UNIT[i]));
    }
  }
}

// Rail: homing, then step generation towards position[5].
void updateRail() {
  if (!enabled) return;
  uint32_t now = micros();
  if (!railHomed) {
    if (digitalRead(PIN_RAIL_HOME) == LOW) {
      railHomed = true;
      railSteps = 0;
      position[5] = target[5] = 0.0;
      return;
    }
    if (now - lastStepUs >= (uint32_t)(1e6 / (HOME_SPEED * RAIL_STEPS_PER_M))) {
      digitalWrite(PIN_RAIL_DIR, LOW);  // towards the home switch
      digitalWrite(PIN_RAIL_STEP, HIGH);
      delayMicroseconds(2);
      digitalWrite(PIN_RAIL_STEP, LOW);
      lastStepUs = now;
    }
    return;
  }
  long wanted = lround(position[5] * RAIL_STEPS_PER_M);
  if (wanted != railSteps && now - lastStepUs >= 40) {
    bool forward = wanted > railSteps;
    digitalWrite(PIN_RAIL_DIR, forward ? HIGH : LOW);
    digitalWrite(PIN_RAIL_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(PIN_RAIL_STEP, LOW);
    railSteps += forward ? 1 : -1;
    lastStepUs = now;
  }
}

// ---------------------------------------------------------------- setup ----
void setup() {
  Serial.begin(BAUD);
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  pinMode(PIN_RAIL_HOME, INPUT_PULLUP);
  pinMode(PIN_RAIL_STEP, OUTPUT);
  pinMode(PIN_RAIL_DIR, OUTPUT);
  pinMode(PIN_RAIL_ENABLE, OUTPUT);
  digitalWrite(PIN_RAIL_ENABLE, HIGH);
  // PWM servos have no position sensor: assume they start at the calibrated
  // zero ("transport pose"). Park the arm there before switching off.
  for (uint8_t i = 0; i < NUM_CHANNELS; ++i) {
    position[i] = target[i] = 0.0;
    velocity[i] = 0.0;
  }
  position[4] = target[4] = LIMIT_HI[4];  // gripper open
  lastUpdateUs = micros();
}

void loop() {
  readSerial();
  updateMotion();
  updateRail();
  if (millis() - lastStateMs >= STATE_PERIOD_MS) {
    lastStateMs = millis();
    sendState();
  }
}
