/*
 * Arduino MEGA 2560 - low-level controller for pingtung_bot.
 * Drives every actuator on commands from the Jetson (mega_bridge node):
 *
 *   2 DC wheels    JZ2407DB dual driver (IN1/IN2 + EN)  <- /mega/cmd_vel via wheel_control
 *   2 steppers     gantry, step/dir drivers             <- /mega/step    [step1, step2]
 *   1 servo        gripper, MG996R                      <- /mega/gripper  angle in degrees
 *
 * Needs the AccelStepper library (Arduino Library Manager).
 *
 * WIRING
 *   JZ2407DB   M1: IN1 -> D22, IN2 -> D23, EN -> D11
 *              M2: IN1 -> D24, IN2 -> D25, EN -> D12
 *              Driver GND -> MEGA GND (COMMON GROUND IS MANDATORY)
 *   Stepper 1  PUL+ -> D2, DIR+ -> D3, PUL-/DIR- -> GND
 *   Stepper 2  PUL+ -> D5, DIR+ -> D6, PUL-/DIR- -> GND
 *   Servo      signal -> D8, power from an external 5-6 V supply, its GND to MEGA GND
 *   D9 / D10   reserved for the water pump PWM
 *
 * The JZ2407DB was previously driven at 3.3 V logic from an ESP32; check the
 * datasheet that its IN/EN inputs accept the Mega's 5 V.
 *
 * EN is on Timer1 (D11/D12). setup() raises Timer1 to ~3.9 kHz so the wheels
 * behave close to the ESP32's 5 kHz and the tuned min_pwm values carry over.
 * Timer0 (millis) and Timer5 (Servo) are left alone.
 *
 * ============================ SERIAL PROTOCOL ============================
 * USB serial, 115200. Must match mega_bridge/protocol.py.
 *
 *   0xAA 0xA5 | type u8 | len u8 | payload[len] | crc8(type, len, payload)
 *
 *   crc8: poly 0x07, init 0x00. All values little-endian.
 *
 *   0x01 DRIVE   int16 m1, int16 m2     wheel PWM -255..255, sign = direction
 *   0x02 STEP    int32 s1, int32 s2     RELATIVE steps, added to the current target
 *   0x03 SERVO   uint8 deg              0..180, eased at 1 deg / 15 ms
 *   0x81 STATUS  int32 rem1, int32 rem2, uint8 servo, uint8 flags    (Mega -> Jetson, 10 Hz)
 *                rem = steps still to go, flags bit0 = wheel failsafe tripped
 *
 * FAILSAFE: no DRIVE frame for CMD_TIMEOUT_MS -> the wheels ramp to a stop.
 * The steppers and the servo are not affected: they finish the move they
 * were given.
 *
 * Nothing but binary frames goes out on Serial - no debug prints.
 * ========================================================================
 */

#include <AccelStepper.h>
#include <Servo.h>

// ---------------- wheels: pins ----------------
const uint8_t M1_IN1 = 22, M1_IN2 = 23, M1_EN = 11;
const uint8_t M2_IN1 = 24, M2_IN2 = 25, M2_EN = 12;

// If a motor turns the wrong way, flip these instead of rewiring.
// (wheel_control also has invert_motor1/2 - use one place, not both.)
const bool M1_INVERT = false;
const bool M2_INVERT = false;

// ---------------- steppers ----------------
const uint8_t S1_PUL = 2, S1_DIR = 3;
const uint8_t S2_PUL = 5, S2_DIR = 6;
const bool S1_INVERT = false;            // set true if the axis runs backwards
const bool S2_INVERT = false;

const float STEP_MAX_SPEED  = 400;       // steps/s (1600 steps/rev at 1/8 microstep)
const float STEP_ACCEL      = 800;       // steps/s^2
const unsigned STEP_PULSE_US = 20;       // long enough for optocoupled drivers

// ---------------- servo ----------------
const uint8_t SERVO_PIN = 8;
const int SERVO_MIN = 0;                 // change to 10/170 if it buzzes at the limits
const int SERVO_MAX = 180;
const int SERVO_CENTER = 90;
const unsigned long SERVO_MOVE_MS = 15;  // 1 degree every 15 ms

// ---------------- serial ----------------
const uint32_t SERIAL_BAUD     = 115200;
const uint16_t CMD_TIMEOUT_MS  = 300;    // wheel failsafe window
const uint16_t STATUS_MS       = 100;
const bool     SOFT_STOP_ON_ZERO = true; // commanded 0,0 -> ramped stop

// =============================== WHEELS ===============================
// Ported from pingtung_contest.ino (ESP32); LEDC replaced by analogWrite.

struct MotorState {
  bool dir;        // last commanded direction
  int  pwm;        // last commanded drive duty, 0..255
};
static MotorState st1 = {true, 0};
static MotorState st2 = {true, 0};

void brakeProfileCancel();

// directions: true = forward, false = reverse.  pwm: 0..255 (0 = hard brake).
void motorOut(uint8_t in1, uint8_t in2, uint8_t en, bool invert,
              MotorState &st, bool directions, int pwm) {
  pwm = constrain(pwm, 0, 255);
  brakeProfileCancel();                 // an explicit command wins
  st.dir = directions;
  st.pwm = pwm;

  bool dir = invert ? !directions : directions;

  if (pwm == 0) {                       // hard brake
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(en, 255);
    return;
  }
  digitalWrite(in1, dir ? HIGH : LOW);
  digitalWrite(in2, dir ? LOW  : HIGH);
  analogWrite(en, pwm);
}

void motor1(bool d, int pwm) { motorOut(M1_IN1, M1_IN2, M1_EN, M1_INVERT, st1, d, pwm); }
void motor2(bool d, int pwm) { motorOut(M2_IN1, M2_IN2, M2_EN, M2_INVERT, st2, d, pwm); }

// strength 0..255: 0 = free coast, 255 = full short-circuit brake.
void motor1Brake(int strength) {
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  analogWrite(M1_EN, constrain(strength, 0, 255));
  st1.pwm = 0;
}

void motor2Brake(int strength) {
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  analogWrite(M2_EN, constrain(strength, 0, 255));
  st2.pwm = 0;
}

void motorsCoast() {
  brakeProfileCancel();
  motor1Brake(0);
  motor2Brake(0);
}

// ---------------- slow brake profile (non-blocking) ----------------
enum BrakeStage : uint8_t { BRAKE_OFF, BRAKE_RELEASE, BRAKE_APPLY, BRAKE_HOLD };

static BrakeStage brakeStage   = BRAKE_OFF;
static uint32_t   brakeT0      = 0;
static uint16_t   brakeRelease = 0;
static uint16_t   brakeApply   = 0;
static uint8_t    brakeHold    = 255;
static int        brakeFrom1   = 0;
static int        brakeFrom2   = 0;
static bool       brakeDir1    = true;
static bool       brakeDir2    = true;

void brakeProfileCancel() { brakeStage = BRAKE_OFF; }

void motorsBrakeSlowStart(uint16_t releaseMs, uint16_t applyMs, uint8_t holdStrength) {
  brakeFrom1   = st1.pwm;
  brakeFrom2   = st2.pwm;
  brakeDir1    = st1.dir;
  brakeDir2    = st2.dir;
  brakeRelease = releaseMs;
  brakeApply   = applyMs;
  brakeHold    = holdStrength;
  brakeT0      = millis();
  brakeStage   = (releaseMs > 0) ? BRAKE_RELEASE : BRAKE_APPLY;
}

void motorsBrakeSlowUpdate() {
  if (brakeStage == BRAKE_OFF || brakeStage == BRAKE_HOLD) return;

  uint32_t elapsed = millis() - brakeT0;

  if (brakeStage == BRAKE_RELEASE) {
    float t = (brakeRelease == 0) ? 1.0f : (float)elapsed / brakeRelease;
    if (t >= 1.0f) {
      brakeStage = BRAKE_APPLY;
      brakeT0    = millis();
      motor1Brake(0);
      motor2Brake(0);
      return;
    }
    float k = 1.0f - t;                 // linear drive ramp-down
    bool d1 = M1_INVERT ? !brakeDir1 : brakeDir1;
    bool d2 = M2_INVERT ? !brakeDir2 : brakeDir2;
    digitalWrite(M1_IN1, d1 ? HIGH : LOW);
    digitalWrite(M1_IN2, d1 ? LOW  : HIGH);
    analogWrite(M1_EN, (int)(brakeFrom1 * k));
    digitalWrite(M2_IN1, d2 ? HIGH : LOW);
    digitalWrite(M2_IN2, d2 ? LOW  : HIGH);
    analogWrite(M2_EN, (int)(brakeFrom2 * k));
    return;
  }

  // BRAKE_APPLY
  float t = (brakeApply == 0) ? 1.0f : (float)elapsed / brakeApply;
  if (t >= 1.0f) {
    motor1Brake(brakeHold);
    motor2Brake(brakeHold);
    brakeStage = BRAKE_HOLD;
    return;
  }
  int s = (int)(brakeHold * t);
  motor1Brake(s);
  motor2Brake(s);
}

static uint32_t lastDriveMs = 0;
static bool     failsafeTripped = true;  // start tripped until the Jetson talks

void applyDrive(int m1, int m2) {
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  lastDriveMs = millis();
  failsafeTripped = false;

  if (SOFT_STOP_ON_ZERO && m1 == 0 && m2 == 0) {
    // Only start a fresh ramp if one is not already running, otherwise a
    // stream of zeros would restart the profile every packet and never finish.
    if (brakeStage == BRAKE_OFF) motorsBrakeSlowStart(400, 300, 255);
    return;
  }
  motor1(m1 >= 0, abs(m1));
  motor2(m2 >= 0, abs(m2));
}

void failsafeCheck() {
  if (failsafeTripped) return;
  if (millis() - lastDriveMs > CMD_TIMEOUT_MS) {
    failsafeTripped = true;
    motorsBrakeSlowStart(300, 200, 255);   // short ramp - this is a fault stop
  }
}

// ============================== STEPPERS ==============================
AccelStepper stepper1(AccelStepper::DRIVER, S1_PUL, S1_DIR);
AccelStepper stepper2(AccelStepper::DRIVER, S2_PUL, S2_DIR);

void setupStepper(AccelStepper &s, bool invert) {
  s.setPinsInverted(invert, false, false);   // direction only
  s.setMinPulseWidth(STEP_PULSE_US);
  s.setMaxSpeed(STEP_MAX_SPEED);
  s.setAcceleration(STEP_ACCEL);
}

// Relative to the current TARGET, so two [1600, 0] commands sent back to
// back move 3200 steps in total instead of the second one cutting the first short.
void stepRelative(AccelStepper &s, int32_t steps) {
  s.moveTo(s.targetPosition() + steps);
}

// =============================== SERVO ================================
Servo servo;
int servoPos = SERVO_CENTER;
int servoTarget = SERVO_CENTER;
unsigned long servoLast = 0;

void runServo() {
  if (servoPos == servoTarget) return;
  if (millis() - servoLast >= SERVO_MOVE_MS) {
    servoLast = millis();
    servoPos += (servoTarget > servoPos) ? 1 : -1;
    servo.write(servoPos);
  }
}

// ============================ SERIAL LINK =============================
const uint8_t SYNC1 = 0xAA, SYNC2 = 0xA5;
const uint8_t T_DRIVE = 0x01, T_STEP = 0x02, T_SERVO = 0x03, T_STATUS = 0x81;
const uint8_t MAX_LEN = 32;
const uint8_t STATUS_LEN = 10;

static uint8_t crc8(uint8_t crc, const uint8_t *data, uint8_t len) {
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// AVR is little-endian, same as the wire format, so memcpy decodes directly.
void handleFrame(uint8_t type, const uint8_t *p, uint8_t len) {
  if (type == T_DRIVE && len == 4) {
    int16_t m1, m2;
    memcpy(&m1, p, 2);
    memcpy(&m2, p + 2, 2);
    applyDrive(m1, m2);
  } else if (type == T_STEP && len == 8) {
    int32_t s1, s2;
    memcpy(&s1, p, 4);
    memcpy(&s2, p + 4, 4);
    stepRelative(stepper1, s1);
    stepRelative(stepper2, s2);
  } else if (type == T_SERVO && len == 1) {
    servoTarget = constrain((int)p[0], SERVO_MIN, SERVO_MAX);
  }
}

enum RxState : uint8_t { RX_SYNC1, RX_SYNC2, RX_TYPE, RX_LEN, RX_PAYLOAD, RX_CRC };
static RxState rxState = RX_SYNC1;
static uint8_t rxHdr[2];                 // type, len - both covered by the crc
static uint8_t rxBuf[MAX_LEN];
static uint8_t rxIdx = 0;

void serialPoll() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    switch (rxState) {
      case RX_SYNC1:
        if (b == SYNC1) rxState = RX_SYNC2;
        break;
      case RX_SYNC2:
        rxState = (b == SYNC2) ? RX_TYPE : (b == SYNC1) ? RX_SYNC2 : RX_SYNC1;
        break;
      case RX_TYPE:
        rxHdr[0] = b;
        rxState = RX_LEN;
        break;
      case RX_LEN:
        rxHdr[1] = b;
        rxIdx = 0;
        rxState = (b > MAX_LEN) ? RX_SYNC1 : (b == 0) ? RX_CRC : RX_PAYLOAD;
        break;
      case RX_PAYLOAD:
        rxBuf[rxIdx++] = b;
        if (rxIdx >= rxHdr[1]) rxState = RX_CRC;
        break;
      case RX_CRC:
        rxState = RX_SYNC1;
        if (crc8(crc8(0, rxHdr, 2), rxBuf, rxHdr[1]) == b)
          handleFrame(rxHdr[0], rxBuf, rxHdr[1]);   // bad crc: drop, do NOT act
        break;
    }
  }
}

void sendStatus() {
  uint8_t f[4 + STATUS_LEN + 1];
  int32_t rem1 = stepper1.distanceToGo();
  int32_t rem2 = stepper2.distanceToGo();
  f[0] = SYNC1;
  f[1] = SYNC2;
  f[2] = T_STATUS;
  f[3] = STATUS_LEN;
  memcpy(f + 4, &rem1, 4);
  memcpy(f + 8, &rem2, 4);
  f[12] = (uint8_t)servoPos;
  f[13] = failsafeTripped ? 0x01 : 0x00;
  f[14] = crc8(0, f + 2, 2 + STATUS_LEN);
  Serial.write(f, sizeof f);
}

// ============================ SETUP / LOOP ============================
void setup() {
  pinMode(M1_IN1, OUTPUT); pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT); pinMode(M2_IN2, OUTPUT);
  pinMode(M1_EN, OUTPUT);  pinMode(M2_EN, OUTPUT);
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);

  // Timer1 prescaler 64 -> 8: EN PWM on D11/D12 goes from 490 Hz to ~3.9 kHz.
  TCCR1B = (TCCR1B & 0b11111000) | 0x02;
  motorsCoast();                         // safe state before the supply settles

  setupStepper(stepper1, S1_INVERT);
  setupStepper(stepper2, S2_INVERT);

  servo.write(SERVO_CENTER);             // start centered at 90 degrees
  servo.attach(SERVO_PIN);

  Serial.begin(SERIAL_BAUD);
}

void loop() {
  serialPoll();                          // decode whatever arrived
  failsafeCheck();                       // stop the wheels if the Jetson went quiet
  motorsBrakeSlowUpdate();               // advance any running brake ramp
  stepper1.run();
  stepper2.run();
  runServo();

  static uint32_t lastStatusMs = 0;
  if (millis() - lastStatusMs >= STATUS_MS) {
    lastStatusMs = millis();
    sendStatus();
  }
  // Keep everything here non-blocking: stepper pulses come from run() above,
  // so any delay() stalls the gantry and loosens the failsafe timing.
}
