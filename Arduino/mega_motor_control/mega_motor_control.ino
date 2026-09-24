/*
 * Arduino MEGA 2560: Two Stepper Motors + One MG996R Servo Motor
 *
 * Wiring
 *   Left Stepper Driver:
 *     PUL+ -> D2
 *     DIR+ -> D3
 *     PUL−, DIR− -> Breadboard GND rail
 *
 *   Right Stepper Driver:
 *     PUL+ -> D5
 *     DIR+ -> D6
 *     PUL−, DIR− -> Breadboard GND rail
 *
 *   MG996R Servo:
 *     Orange (Signal) -> D8
 *     Red (+)         -> External 5~6V power supply positive
 *     Brown (−)       -> External 5~6V power supply negative
 *                        and connected to MEGA GND
 *
 *   MEGA GND -> Breadboard GND rail
 *
 * Operation:
 *   Open Serial Monitor at 115200 baud.
 *   Type a command and press Enter.
 *
 *   A = Left motor forward
 *   D = Left motor reverse
 *   S = Left motor stop
 *
 *   Z = Right motor forward
 *   C = Right motor reverse
 *   X = Right motor stop
 *
 *   5 = Servo +30°
 *   6 = Servo −30°
 *   7 = Servo return to 90°
 *
 *   0 = Stop everything
 *   ? = Show help
 *
 *   Stepper motors keep rotating until S, X, or 0 is pressed.
 *
 *   Multiple commands can be entered at once.
 *   Example: "AZ" makes both stepper motors rotate forward.
 */

#include <Servo.h>

// ---------------- Adjustable Parameters ----------------
const int L_PUL = 2, L_DIR = 3;          // Left motor
const int R_PUL = 5, R_DIR = 6;          // Right motor
const int SERVO_PIN = 8;                 // Servo signal
                                          // (D9 and D10 reserved for water pump PWM)

const bool L_INVERT = false;             // Set to true if rotation direction is reversed
const bool R_INVERT = false;

const long STEPS_PER_REV = 1600;         // 1/8 microstepping = 1600 steps/rev
                                         // (used only for RPM display)
const long SPEED = 400;                  // Stepper speed (steps/sec)

const int SERVO_MIN = 0;                 // Change to 10/170 if servo buzzes at limits
const int SERVO_MAX = 180;
const int SERVO_CENTER = 90;
const int SERVO_STEP = 30;               // Degrees moved per key press
const unsigned long SERVO_MOVE_MS = 15;  // Move 1 degree every 15 ms

// ---------------- Stepper Motor ----------------
struct Wheel {
  const char* name;
  int pul, dir;
  bool invert;
  int state;               // 1 = forward, -1 = reverse, 0 = stop
  bool level;              // Current PUL pin level
  unsigned long last;      // Last pulse toggle time
};

Wheel wheelL = { "Left Motor",  L_PUL, L_DIR, L_INVERT, 0, false, 0 };
Wheel wheelR = { "Right Motor", R_PUL, R_DIR, R_INVERT, 0, false, 0 };

void setWheel(Wheel &w, int state) {
  w.state = state;

  if (state == 0) {
    digitalWrite(w.pul, LOW);
    w.level = false;
    Serial.print(w.name);
    Serial.println(F(" Stopped"));
    return;
  }

  bool forward = (state > 0);
  digitalWrite(w.dir, (forward != w.invert) ? HIGH : LOW);

  delayMicroseconds(10);   // Allow direction signal to stabilize before pulsing

  Serial.print(w.name);
  Serial.println(forward ? F(" Forward") : F(" Reverse"));
}

// Square-wave pulses.
// High and low states each occupy half the period.
// Pulse width is long enough for slower optocouplers.
void runWheel(Wheel &w) {
  if (w.state == 0) return;

  unsigned long half = 500000UL / SPEED;
  unsigned long now = micros();

  if (now - w.last >= half) {
    w.last = now;
    w.level = !w.level;
    digitalWrite(w.pul, w.level ? HIGH : LOW);
  }
}

// ---------------- Servo Motor ----------------
Servo servo;

int servoPos = SERVO_CENTER;
int servoTarget = SERVO_CENTER;
unsigned long servoLast = 0;

void setServoTarget(int deg) {
  servoTarget = constrain(deg, SERVO_MIN, SERVO_MAX);

  Serial.print(F("Servo -> "));
  Serial.print(servoTarget);
  Serial.println(F(" degrees"));
}

void runServo() {
  if (servoPos == servoTarget) return;

  if (millis() - servoLast >= SERVO_MOVE_MS) {
    servoLast = millis();

    servoPos += (servoTarget > servoPos) ? 1 : -1;
    servo.write(servoPos);

    if (servoPos == servoTarget)
      Serial.println(F("Servo movement complete"));
  }
}

// ---------------- Main Program ----------------
void printHelp() {
  Serial.println(F("--------------- Commands ---------------"));
  Serial.println(F("A Left Forward   D Left Reverse   S Left Stop"));
  Serial.println(F("Z Right Forward  C Right Reverse X Right Stop"));
  Serial.println(F("5 Servo +30deg   6 Servo -30deg  7 Servo to 90deg"));
  Serial.println(F("0 Stop All       ? Show Help"));

  Serial.print(F("Current speed: "));
  Serial.print(SPEED);
  Serial.print(F(" steps/sec (~"));

  Serial.print(SPEED * 60.0 / STEPS_PER_REV, 1);
  Serial.println(F(" RPM)"));

  Serial.println(F("----------------------------------------"));
}

void setup() {
  pinMode(L_PUL, OUTPUT);
  pinMode(L_DIR, OUTPUT);
  pinMode(R_PUL, OUTPUT);
  pinMode(R_DIR, OUTPUT);

  digitalWrite(L_PUL, LOW);
  digitalWrite(R_PUL, LOW);

  servo.write(SERVO_CENTER);     // Start centered at 90°
  servo.attach(SERVO_PIN);

  Serial.begin(115200);

  Serial.println(F("Two Steppers + MG996R Test Started (Servo at 90°)"));
  printHelp();
}