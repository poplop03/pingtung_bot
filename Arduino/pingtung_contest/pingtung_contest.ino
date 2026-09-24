/*
 * JZ2407DB / MB-ELC-DUAL-MOTOR-DRV-7A  -  dual DC motor driver
 * ESP32 (LEDC hardware PWM), ramped braking, serial command interface.
 *
 * 24V 7A per channel, IN1/IN2 + EN interface.
 *
 * Control logic per channel:
 *   INx  INy  EN(PWM)   result
 *    0    0     0       coast (outputs disabled)
 *    0    0     1       brake (both low-side FETs on, motor shorted)
 *    0    0    PWM      PROPORTIONAL BRAKE - duty on EN sets holding force
 *    1    0    PWM      forward at duty
 *    0    1    PWM      reverse at duty
 *
 * The driver accepts 3.3V logic directly - no level shifter needed.
 *
 * Minimum valid pulse width on EN is 5 us. At the 5 kHz set below the period
 * is 200 us, so duty values under about 7/255 produce nothing.
 *
 * WIRING
 *   Driver VM / GND   -> 7..24 V supply
 *   Driver GND        -> ESP32 GND     (COMMON GROUND IS MANDATORY)
 *   Motor 1           -> OUT1 / OUT2
 *   Motor 2           -> OUT3 / OUT4
 *   Never power the motors from the ESP32 3V3 or VIN rail.
 *
 * WARNING: braking a spinning motor pushes current back toward the supply.
 * A battery absorbs it; a bench PSU will trip OVP. On a PSU keep the brake
 * ramp gentle and fit >= 1000 uF across the driver's supply pins.
 *
 * ============================ SERIAL PROTOCOL ============================
 * Two framings are accepted on the same port; the parser auto-detects which.
 *
 * A) BINARY (use this for a host program) - 7 bytes, little-endian:
 *
 *      0xAA  0xA5  |  int16 m1  |  int16 m2  |  uint8 crc8
 *      <-- sync -->   <------ 4 byte payload ------>  over payload only
 *
 *    crc8: poly 0x07, init 0x00, computed over the 4 payload bytes.
 *    Values are clamped to -255..255. Sign = direction, magnitude = duty.
 *    Python:  struct.pack('<BBhhB', 0xAA, 0xA5, m1, m2, crc)
 *
 * B) ASCII (use this from a serial monitor) - one line, newline terminated:
 *
 *      -120,255      or      -120 255
 *      s             stop now (slow brake profile)
 *      x             coast
 *      ?             print status
 *
 * FAILSAFE: if no valid command arrives within CMD_TIMEOUT_MS the slow brake
 * profile runs automatically. Keep the host sending, even when sending zeros.
 * ========================================================================
 */

// ---------------- pin map (safe GPIOs on a classic ESP32 DevKit) -------------
// Avoided: 0/2/5/12/15 (strapping), 6-11 (flash), 34-39 (input only).
const uint8_t M1_IN1 = 26;   // motor 1 direction A
const uint8_t M1_IN2 = 27;   // motor 1 direction B
const uint8_t M1_EN  = 25;   // motor 1 PWM

const uint8_t M2_IN1 = 32;   // motor 2 direction A
const uint8_t M2_IN2 = 33;   // motor 2 direction B
const uint8_t M2_EN  = 14;   // motor 2 PWM

// If a motor turns the wrong way, flip these instead of rewiring.
const bool M1_INVERT = false;
const bool M2_INVERT = false;

// ---------------- PWM config ----------------
const uint32_t PWM_FREQ = 5000;   // Hz
const uint8_t  PWM_RES  = 8;      // bits -> duty range 0..255

const uint8_t M1_CH = 0;          // LEDC channels (core 2.x only)
const uint8_t M2_CH = 1;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_SETUP(pin, ch)   ledcAttach((pin), PWM_FREQ, PWM_RES)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((pin), (duty))
#else
  #define PWM_SETUP(pin, ch)   do { ledcSetup((ch), PWM_FREQ, PWM_RES); \
                                    ledcAttachPin((pin), (ch)); } while (0)
  #define PWM_WRITE(pin, ch, duty) ledcWrite((ch), (duty))
#endif

// ---------------- serial config ----------------
const uint32_t SERIAL_BAUD    = 115200;
const uint16_t CMD_TIMEOUT_MS = 300;    // failsafe window
const bool     SOFT_STOP_ON_ZERO = true;// commanded 0,0 -> ramped stop
const bool     ASCII_ACK      = true;   // echo "ok m1 m2" for ASCII commands

// ---------------- motor state ----------------
struct MotorState {
  bool dir;        // last commanded direction
  int  pwm;        // last commanded drive duty, 0..255
};
static MotorState st1 = {true, 0};
static MotorState st2 = {true, 0};

void brakeProfileCancel();               // fwd decl

// ---------------- core API ----------------
// directions: true = forward, false = reverse.  pwm: 0..255 (0 = hard brake).

void motor1(bool directions, int pwm) {
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;

  brakeProfileCancel();                 // an explicit command wins
  st1.dir = directions;
  st1.pwm = pwm;

  bool dir = M1_INVERT ? !directions : directions;

  if (pwm == 0) {                       // hard brake
    digitalWrite(M1_IN1, LOW);
    digitalWrite(M1_IN2, LOW);
    PWM_WRITE(M1_EN, M1_CH, 255);
    return;
  }

  digitalWrite(M1_IN1, dir ? HIGH : LOW);
  digitalWrite(M1_IN2, dir ? LOW  : HIGH);
  PWM_WRITE(M1_EN, M1_CH, pwm);
}

void motor2(bool directions, int pwm) {
  if (pwm < 0)   pwm = 0;
  if (pwm > 255) pwm = 255;

  brakeProfileCancel();
  st2.dir = directions;
  st2.pwm = pwm;

  bool dir = M2_INVERT ? !directions : directions;

  if (pwm == 0) {
    digitalWrite(M2_IN1, LOW);
    digitalWrite(M2_IN2, LOW);
    PWM_WRITE(M2_EN, M2_CH, 255);
    return;
  }

  digitalWrite(M2_IN1, dir ? HIGH : LOW);
  digitalWrite(M2_IN2, dir ? LOW  : HIGH);
  PWM_WRITE(M2_EN, M2_CH, pwm);
}

// ---------------- braking primitives ----------------
// strength 0..255: 0 = free coast, 255 = full short-circuit brake.
void motor1Brake(int strength) {
  strength = constrain(strength, 0, 255);
  digitalWrite(M1_IN1, LOW);
  digitalWrite(M1_IN2, LOW);
  PWM_WRITE(M1_EN, M1_CH, strength);
  st1.pwm = 0;
}

void motor2Brake(int strength) {
  strength = constrain(strength, 0, 255);
  digitalWrite(M2_IN1, LOW);
  digitalWrite(M2_IN2, LOW);
  PWM_WRITE(M2_EN, M2_CH, strength);
  st2.pwm = 0;
}

void motorsBrake() {                    // hard stop, immediate
  brakeProfileCancel();
  motor1Brake(255);
  motor2Brake(255);
}

void motorsCoast() {                    // outputs off, motors free-spin
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

void motorsBrakeSlowStart(uint16_t releaseMs = 600,
                          uint16_t applyMs   = 400,
                          uint8_t  holdStrength = 255) {
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

// Swap for a different curve. Linear by default; smoothstep is commented.
static inline float brakeCurve(float t) {
  return t;
  // return t * t * (3.0f - 2.0f * t);
}

bool motorsBrakeSlowUpdate() {
  if (brakeStage == BRAKE_OFF || brakeStage == BRAKE_HOLD) return false;

  uint32_t elapsed = millis() - brakeT0;

  if (brakeStage == BRAKE_RELEASE) {
    float t = (brakeRelease == 0) ? 1.0f : (float)elapsed / brakeRelease;
    if (t >= 1.0f) {
      brakeStage = BRAKE_APPLY;
      brakeT0    = millis();
      motor1Brake(0);
      motor2Brake(0);
      return true;
    }
    float k = 1.0f - brakeCurve(t);
    bool d1 = M1_INVERT ? !brakeDir1 : brakeDir1;
    bool d2 = M2_INVERT ? !brakeDir2 : brakeDir2;
    digitalWrite(M1_IN1, d1 ? HIGH : LOW);
    digitalWrite(M1_IN2, d1 ? LOW  : HIGH);
    PWM_WRITE(M1_EN, M1_CH, (int)(brakeFrom1 * k));
    digitalWrite(M2_IN1, d2 ? HIGH : LOW);
    digitalWrite(M2_IN2, d2 ? LOW  : HIGH);
    PWM_WRITE(M2_EN, M2_CH, (int)(brakeFrom2 * k));
    return true;
  }

  // BRAKE_APPLY
  float t = (brakeApply == 0) ? 1.0f : (float)elapsed / brakeApply;
  if (t >= 1.0f) {
    motor1Brake(brakeHold);
    motor2Brake(brakeHold);
    brakeStage = BRAKE_HOLD;
    return false;
  }
  int s = (int)(brakeHold * brakeCurve(t));
  motor1Brake(s);
  motor2Brake(s);
  return true;
}

void motorsBrakeSlow(uint16_t releaseMs = 600,
                     uint16_t applyMs   = 400,
                     uint8_t  holdStrength = 255) {
  motorsBrakeSlowStart(releaseMs, applyMs, holdStrength);
  while (motorsBrakeSlowUpdate()) delay(2);
}

// ---------------- helpers ----------------
void motor1Signed(int speed) { motor1(speed >= 0, abs(speed)); }
void motor2Signed(int speed) { motor2(speed >= 0, abs(speed)); }

void drive(int throttle, int turn) {
  int left  = constrain(throttle + turn, -255, 255);
  int right = constrain(throttle - turn, -255, 255);
  motor1Signed(left);
  motor2Signed(right);
}

// ============================ SERIAL INTERFACE ============================

static int32_t  cmd1 = 0, cmd2 = 0;     // last accepted command
static uint32_t lastCmdMs = 0;          // for the failsafe
static bool     failsafeTripped = true; // start in the tripped state
static uint32_t pktGood = 0, pktBad = 0;

static uint8_t crc8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *data++;
    for (uint8_t i = 0; i < 8; i++)
      crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
  }
  return crc;
}

// One place where a decoded command becomes motor output.
void applyCommand(int32_t m1, int32_t m2) {
  m1 = constrain(m1, -255, 255);
  m2 = constrain(m2, -255, 255);
  cmd1 = m1;
  cmd2 = m2;
  lastCmdMs = millis();
  failsafeTripped = false;

  if (SOFT_STOP_ON_ZERO && m1 == 0 && m2 == 0) {
    // Only start a fresh ramp if one is not already running, otherwise a
    // stream of zeros would restart the profile every packet and never finish.
    if (brakeStage == BRAKE_OFF) motorsBrakeSlowStart(400, 300, 255);
    return;
  }

  motor1Signed(m1);
  motor2Signed(m2);
}

void printStatus() {
  Serial.printf("m1=%ld m2=%ld brake=%u age=%lums good=%lu bad=%lu\n",
                (long)cmd1, (long)cmd2, (unsigned)brakeStage,
                (unsigned long)(millis() - lastCmdMs),
                (unsigned long)pktGood, (unsigned long)pktBad);
}

// ---- ASCII line parser: "-120,255", "-120 255", "s", "x", "?" ----
static void handleAsciiLine(char *line) {
  // trim leading space
  while (*line == ' ' || *line == '\t') line++;
  if (*line == '\0' || *line == '\r') return;

  if (line[0] == 's' || line[0] == 'S') { motorsBrakeSlowStart(); lastCmdMs = millis(); failsafeTripped = false; if (ASCII_ACK) Serial.println("ok stop"); return; }
  if (line[0] == 'x' || line[0] == 'X') { motorsCoast();          lastCmdMs = millis(); failsafeTripped = false; if (ASCII_ACK) Serial.println("ok coast"); return; }
  if (line[0] == '?')                   { printStatus(); return; }

  char *end1 = nullptr;
  long v1 = strtol(line, &end1, 10);
  if (end1 == line) { pktBad++; if (ASCII_ACK) Serial.println("err parse"); return; }

  while (*end1 == ',' || *end1 == ' ' || *end1 == '\t' || *end1 == ';') end1++;

  char *end2 = nullptr;
  long v2 = strtol(end1, &end2, 10);
  if (end2 == end1) { pktBad++; if (ASCII_ACK) Serial.println("err parse"); return; }

  pktGood++;
  applyCommand(v1, v2);
  if (ASCII_ACK) Serial.printf("ok %ld %ld\n", (long)cmd1, (long)cmd2);
}

// ---- combined reader: binary state machine + ASCII fallback ----
const uint8_t PKT_SYNC1 = 0xAA;
const uint8_t PKT_SYNC2 = 0xA5;

enum RxState : uint8_t { RX_IDLE, RX_SYNC2, RX_PAYLOAD, RX_CRC };
static RxState rxState = RX_IDLE;
static uint8_t rxBuf[4];
static uint8_t rxIdx = 0;

static char    lineBuf[48];
static uint8_t lineIdx = 0;

void serialPoll() {
  while (Serial.available()) {
    uint8_t b = Serial.read();

    // ---- binary path ----
    switch (rxState) {
      case RX_SYNC2:
        if (b == PKT_SYNC2) { rxState = RX_PAYLOAD; rxIdx = 0; continue; }
        if (b == PKT_SYNC1) { continue; }   // 0xAA 0xAA... keep waiting
        rxState = RX_IDLE;
        // not our packet - fall through so the byte is still parsed as ASCII
        break;

      case RX_PAYLOAD:
        rxBuf[rxIdx++] = b;
        if (rxIdx >= 4) rxState = RX_CRC;
        continue;

      case RX_CRC: {
        rxState = RX_IDLE;
        if (crc8(rxBuf, 4) == b) {
          int16_t m1 = (int16_t)((uint16_t)rxBuf[0] | ((uint16_t)rxBuf[1] << 8));
          int16_t m2 = (int16_t)((uint16_t)rxBuf[2] | ((uint16_t)rxBuf[3] << 8));
          pktGood++;
          applyCommand(m1, m2);
        } else {
          pktBad++;                     // bad CRC - drop it, do NOT act on it
        }
        continue;
      }

      case RX_IDLE:
      default:
        if (b == PKT_SYNC1) { rxState = RX_SYNC2; continue; }
        break;
    }

    // ---- ASCII path ----
    if (b == '\n' || b == '\r') {
      if (lineIdx > 0) {
        lineBuf[lineIdx] = '\0';
        handleAsciiLine(lineBuf);
        lineIdx = 0;
      }
    } else if (lineIdx < sizeof(lineBuf) - 1) {
      lineBuf[lineIdx++] = (char)b;
    } else {
      lineIdx = 0;                      // overlong garbage - resync
      pktBad++;
    }
  }
}

// Failsafe: no valid command in CMD_TIMEOUT_MS -> ramp to a stop.
void failsafeCheck() {
  if (failsafeTripped) return;
  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    failsafeTripped = true;
    cmd1 = cmd2 = 0;
    motorsBrakeSlowStart(300, 200, 255);   // short ramp - this is a fault stop
    Serial.println("failsafe: command timeout");
  }
}

// ---------------- setup / loop ----------------
void setup() {
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // Drive the direction pins low before the PWM channels come up.
  digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
  digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);

  PWM_SETUP(M1_EN, M1_CH);
  PWM_SETUP(M2_EN, M2_CH);
  motorsCoast();                        // safe state before the supply settles

  Serial.begin(SERIAL_BAUD);
  delay(500);                           // let the driver's logic supply come up
  Serial.println("jz2407db ready - ascii: \"m1,m2\" | s | x | ?");
}

void loop() {
  serialPoll();                         // decode whatever arrived
  failsafeCheck();                      // stop if the host went quiet
  motorsBrakeSlowUpdate();              // advance any running brake ramp

  // ... your own periodic work goes here. Keep it non-blocking; no delay()
  // longer than a few ms, or the failsafe timing gets sloppy.
}