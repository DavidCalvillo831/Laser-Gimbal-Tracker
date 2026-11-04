#include <Servo.h>

Servo panServo;   //controls in charge of LR movement
Servo tiltServo;  //controls in charge of UD movement

const int PAN_PIN = 10;
const int TILT_PIN = 9; // for testing purposes a arduino UNO was used and pin 10 was PAN and pin 9 controlled tilt.

// Pulse width range the Servo library uses for 0 and 180 degrees
const int PULSE_MIN = 544;
const int PULSE_MAX = 2400;

const float ANGLE_MIN = 15.0;
const float ANGLE_MAX = 165.0;
const float CENTER = 90.0;

//Smoothing between camera updates
// SMOOTH: 1.0 = jump instantly/twitchy movement (can test if want but I didn't like it, 0.2 = very smooth, 0.4 = good balance
const float SMOOTH = 0.4;
const unsigned long UPDATE_MS = 4;

float targetPan = CENTER, targetTilt = CENTER;
float currentPan = CENTER, currentTilt = CENTER;
int lastPanUs = -1, lastTiltUs = -1;
unsigned long lastUpdate = 0;

char buf[24];
byte bufLen = 0;

int angleToMicros(float angle) {
  return (int)(PULSE_MIN + (angle / 180.0) * (PULSE_MAX - PULSE_MIN) + 0.5);
}

bool isNumber(const char *s) {
  if (*s == '\0') return false;
  while (*s) {
    if (!isdigit(*s) && *s != '.') return false;
    s++;
  }
  return true;
}

void handleLine() {
  buf[bufLen] = '\0';
  char *comma = strchr(buf, ',');
  if (comma != NULL) {
    *comma = '\0';
    if (isNumber(buf) && isNumber(comma + 1)) {
      targetPan = constrain(atof(buf), ANGLE_MIN, ANGLE_MAX);
      targetTilt = constrain(atof(comma + 1), ANGLE_MIN, ANGLE_MAX);
    }
  }
  bufLen = 0;
}

void setup() {
  Serial.begin(115200);
  panServo.attach(PAN_PIN, PULSE_MIN, PULSE_MAX);
  tiltServo.attach(TILT_PIN, PULSE_MIN, PULSE_MAX);
  panServo.writeMicroseconds(angleToMicros(CENTER));
  tiltServo.writeMicroseconds(angleToMicros(CENTER));
}

void loop() {
 
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleLine();
    } else if (c != '\r') {
      if (bufLen < sizeof(buf) - 1) {
        buf[bufLen++] = c;
      } else {
        bufLen = 0;
      }
    }
  }

  // Glide toward the target at a smooth rate
  unsigned long now = millis();
  if (now - lastUpdate >= UPDATE_MS) {
    lastUpdate = now;
    currentPan += (targetPan - currentPan) * SMOOTH;
    currentTilt += (targetTilt - currentTilt) * SMOOTH;

    int panUs = angleToMicros(currentPan);
    int tiltUs = angleToMicros(currentTilt);

    // Only send a new pulse width when it actually changes
    if (panUs != lastPanUs) {
      panServo.writeMicroseconds(panUs);
      lastPanUs = panUs;
    }
    if (tiltUs != lastTiltUs) {
      tiltServo.writeMicroseconds(tiltUs);
      lastTiltUs = tiltUs;
    }
  }
}

