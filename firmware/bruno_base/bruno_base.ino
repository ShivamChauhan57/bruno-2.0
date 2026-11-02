// Bruno Robot - base drive firmware (L298N differential drive + 3x HC-SR04)

// Motor driver pins
const int IN1 = 16, IN2 = 18, ENA = 25; // left motor
const int IN3 = 19, IN4 = 23, ENB = 26; // right motor

// Status LEDs
const int LED_BLUE = 15;   // active/driving
const int LED_GREEN = 2;   // heartbeat
const int LED_YELLOW = 4;  // warning (approaching obstacle)
const int LED_RED = 5;     // halt/blocked

// Ultrasonic sensors
const int TRIG_L = 13, ECHO_L = 34;
const int TRIG_M = 14, ECHO_M = 35;
const int TRIG_R = 27, ECHO_R = 39;

const long SAFE_DISTANCE_CM = 15;
const long WARN_DISTANCE_CM = 30;
const unsigned long ECHO_TIMEOUT_US = 25000UL; // 25ms non-blocking cap, ~4 m max range

const unsigned long ULTRASONIC_INTERVAL_MS = 60;
const unsigned long STREAM_INTERVAL_MS = 150;
const unsigned long HEARTBEAT_INTERVAL_MS = 500;

const float LEFT_TRIM = 0.65; // mechanical trim test: 35% reduction gives straightest open-loop tracking

int commandedLeft = 0;
int commandedRight = 0;
bool haltActive = false;

long distLeft = -1, distMid = -1, distRight = -1;

String serialLineBuffer;

void driveMotor(int pinA, int pinB, int pinPwm, int speed) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) {
    digitalWrite(pinA, HIGH);
    digitalWrite(pinB, LOW);
  } else if (speed < 0) {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, HIGH);
  } else {
    digitalWrite(pinA, LOW);
    digitalWrite(pinB, LOW);
  }
  analogWrite(pinPwm, abs(speed));
}

long readDistanceCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1; // no echo -> out of range
  return (long)(duration * 0.0343 / 2.0);
}

void updateUltrasonics() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < ULTRASONIC_INTERVAL_MS) return;
  last = now;

  distLeft = readDistanceCm(TRIG_L, ECHO_L);
  distMid = readDistanceCm(TRIG_M, ECHO_M);
  distRight = readDistanceCm(TRIG_R, ECHO_R);
}

void streamUltrasonicData() {
  static unsigned long last = 0;
  unsigned long now = millis();
  if (now - last < STREAM_INTERVAL_MS) return;
  last = now;

  Serial.print("U ");
  Serial.print(distLeft);
  Serial.print(' ');
  Serial.print(distMid);
  Serial.print(' ');
  Serial.println(distRight);
}

bool withinRange(long d, long threshold) {
  // -1 (and any other non-positive value) means no echo / out of range,
  // so it must never be treated as an obstacle by the safety cutoff.
  return d > 0 && d < threshold;
}

void evaluateSafety() {
  bool obstacle = withinRange(distLeft, SAFE_DISTANCE_CM) ||
                  withinRange(distMid, SAFE_DISTANCE_CM) ||
                  withinRange(distRight, SAFE_DISTANCE_CM);
  bool warning = withinRange(distLeft, WARN_DISTANCE_CM) ||
                 withinRange(distMid, WARN_DISTANCE_CM) ||
                 withinRange(distRight, WARN_DISTANCE_CM);

  if (obstacle && !haltActive) {
    haltActive = true;
    Serial.println("S HALT");
  } else if (!obstacle && haltActive) {
    haltActive = false;
    Serial.println("S CLEAR");
  }

  digitalWrite(LED_RED, haltActive ? HIGH : LOW);
  digitalWrite(LED_YELLOW, (!haltActive && warning) ? HIGH : LOW);
}

void applyMotorOutputs() {
  int outLeft = haltActive ? 0 : (int)(commandedLeft * LEFT_TRIM);
  int outRight = haltActive ? 0 : commandedRight;

  driveMotor(IN1, IN2, ENA, outLeft);
  driveMotor(IN4, IN3, ENB, outRight); // right motor is mounted/wired mirrored; swap IN3/IN4 to invert direction

  digitalWrite(LED_BLUE, (!haltActive && (outLeft != 0 || outRight != 0)) ? HIGH : LOW);
}

void updateHeartbeat() {
  static unsigned long last = 0;
  static bool state = false;
  unsigned long now = millis();
  if (now - last >= HEARTBEAT_INTERVAL_MS) {
    last = now;
    state = !state;
    digitalWrite(LED_GREEN, state ? HIGH : LOW);
  }
}

void processLine(const String &line) {
  char cmd;
  int l, r;
  if (sscanf(line.c_str(), " %c %d %d", &cmd, &l, &r) == 3 && (cmd == 'M' || cmd == 'm')) {
    commandedLeft = constrain(l, -255, 255);
    commandedRight = constrain(r, -255, 255);
  }
}

void pollSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n') {
      processLine(serialLineBuffer);
      serialLineBuffer = "";
    } else if (c != '\r') {
      serialLineBuffer += c;
      if (serialLineBuffer.length() > 32) {
        serialLineBuffer = ""; // guard against a runaway/garbled line
      }
    }
  }
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(TRIG_L, OUTPUT);
  pinMode(ECHO_L, INPUT);
  pinMode(TRIG_M, OUTPUT);
  pinMode(ECHO_M, INPUT);
  pinMode(TRIG_R, OUTPUT);
  pinMode(ECHO_R, INPUT);

  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  driveMotor(IN1, IN2, ENA, 0);
  driveMotor(IN3, IN4, ENB, 0);

  Serial.begin(115200);
  serialLineBuffer.reserve(32);
}

void loop() {
  pollSerial();
  updateUltrasonics();
  evaluateSafety();
  applyMotorOutputs();
  updateHeartbeat();
  streamUltrasonicData();
}
