#include <Servo.h>
#include <math.h>

// ── Pins ──────────────────────────────────────────
#define L_HIP_PIN    10
#define L_KNEE_PIN    9
#define L_ANKLE_PIN   6
#define R_HIP_PIN     3
#define R_KNEE_PIN   11
#define R_ANKLE_PIN   5

Servo lHip, lKnee, lAnkle;
Servo rHip, rKnee, rAnkle;

// ── Leg Dimensions ────────────────────────────────
const float L1 = 6.0;    // hip   → knee  (cm)
const float L2 = 5.5;    // knee  → ankle (cm)
// max reach = 11.5 cm
// min reach = 0.5 cm

// ── State ─────────────────────────────────────────
bool          walking       = false;
bool kicking = false;
int           walkPhase     = 0;
String        command       = "";
unsigned long lastPhaseTime = 0;
const unsigned long PHASE_DELAY = 400;

// ═══════════════════════════════════════════════════
//  FORWARD KINEMATICS
//  Given hip and knee angles → where is the ankle?
//
//  t1_deg = hip angle (degrees from horizontal)
//  t2_deg = knee angle (degrees, relative to thigh)
//  x_out  = ankle forward distance from hip (cm)
//  y_out  = ankle downward distance from hip (cm)
// ═══════════════════════════════════════════════════
void forwardKinematics(float t1_deg, float t2_deg,
                        float &x_out, float &y_out) {
  float t1  = radians(t1_deg);
  float t12 = radians(t1_deg + t2_deg);  // absolute shin angle

  x_out = L1 * cos(t1) + L2 * cos(t12);
  y_out = L1 * sin(t1) + L2 * sin(t12);

  Serial.print("[FK] hip=");     Serial.print(t1_deg, 1);
  Serial.print("deg  knee=");    Serial.print(t2_deg, 1);
  Serial.print("deg  =>  x=");   Serial.print(x_out, 2);
  Serial.print("cm  y=");        Serial.println(y_out, 2);
}

// ═══════════════════════════════════════════════════
//  INVERSE KINEMATICS — law of cosines
//  Given ankle target position → what angles are needed?
//
//  x      = how far forward the ankle target is (cm)
//  y      = how far downward the ankle target is (cm)
//  t1_out = required hip angle (degrees)
//  t2_out = required knee angle (degrees)
//  returns false if target is unreachable
// ═══════════════════════════════════════════════════
bool inverseKinematics(float x, float y,
                        float &t1_out, float &t2_out) {
  float D = (x*x + y*y - L1*L1 - L2*L2) / (2.0 * L1 * L2);

  if (abs(D) > 1.0) {
    Serial.print("[IK] UNREACHABLE — D="); Serial.println(D, 3);
    Serial.print("      max reach=");      Serial.println(L1 + L2);
    return false;
  }

  // Knee bends backward — natural kicking posture
  float t2 = atan2(-sqrt(1.0 - D*D), D);
  float t1 = atan2(y, x) - atan2(L2 * sin(t2), L1 + L2 * cos(t2));

  t1_out = degrees(t1);
  t2_out = degrees(t2);

  Serial.print("[IK] target x="); Serial.print(x, 2);
  Serial.print("cm  y=");         Serial.print(y, 2);
  Serial.print("cm  =>  hip=");   Serial.print(t1_out, 1);
  Serial.print("deg  knee=");     Serial.println(t2_out, 1);

  return true;
}

// ═══════════════════════════════════════════════════
//  FK VALIDATION
//  After every IK solve, plug angles back into FK.
//  Result must match the original target within 0.5cm.
// ═══════════════════════════════════════════════════
bool validateIK(float t1, float t2, float tgtX, float tgtY) {
  float vx, vy;
  forwardKinematics(t1, t2, vx, vy);

  float error = sqrt((vx - tgtX)*(vx - tgtX) + (vy - tgtY)*(vy - tgtY));
  Serial.print("[FK Validate] error="); Serial.print(error, 3); Serial.println("cm");

  if (error < 0.5) {
    Serial.println("[FK Validate] PASS");
    return true;
  }
  Serial.println("[FK Validate] FAIL — aborting");
  return false;
}

// ── Angle to servo value ──────────────────────────
// 90 = neutral (leg straight down)
int angleToServo(float angleDeg) {
  return constrain(90 + (int)round(angleDeg), 0, 180);
}

// ═══════════════════════════════════════════════════
//  SERVO HELPERS
// ═══════════════════════════════════════════════════
float ease(float t) { return (1.0 - cos(PI * t)) / 2.0; }

void sweepOne(Servo &s, int from, int to) {
  for (int i = 0; i <= 60; i++) {
    s.write(from + (int)((to - from) * ease((float)i / 60)));
    delay(12);
  }
}

void sweepTwo(Servo &s1, int f1, int t1,
              Servo &s2, int f2, int t2) {
  for (int i = 0; i <= 60; i++) {
    float e = ease((float)i / 60);
    s1.write(f1 + (int)((t1 - f1) * e));
    s2.write(f2 + (int)((t2 - f2) * e));
    delay(12);
  }
}

void allNeutral() {
  lHip.write(90);
  lKnee.write(90);
  lAnkle.write(90);

  rHip.write(90);
  rKnee.write(90);
  rAnkle.write(90);
}

// ═══════════════════════════════════════════════════
//  KICK — fully IK driven
//
//  dist    = ball distance from camera (cm)
//  lateral = ball left/right offset (cm)
//            positive = ball to the right
// ═══════════════════════════════════════════════════
void kickWithIK(float dist) {
  
  Serial.println("KICK");
  allNeutral();

    // balance
  sweepOne(lAnkle ,90, 60);
      // bend knee

  delay(300);

   sweepTwo(rHip, 90,60, lKnee, 90,120);

  delay(2000);

  sweepTwo(rHip, 60,90, lKnee, 120,90);
}
// ═══════════════════════════════════════════════════
//  WALKING — non-blocking state machine
//  Serial is checked between every phase so
//  IK commands are never missed during walking
// ═══════════════════════════════════════════════════
void runWalkPhase(int phase) {
  switch (phase) {
    case 0: rAnkle.write(115); break;                          // shift weight right
    case 1: sweepTwo(lHip, 90,130, lKnee, 90,110); break;     // left leg forward
    case 2: rAnkle.write(90);  break;                          // weight back
    case 3: sweepTwo(lHip, 130,90, lKnee, 110,90); break;     // left leg home
    case 4: lAnkle.write(75);  break;                          // shift weight left
    case 5: sweepTwo(rHip, 90,65,  rKnee, 90,65);  break;     // right leg forward
    case 6: lAnkle.write(90);  break;                          // weight back
    case 7: sweepTwo(rHip, 65,90,  rKnee, 65,90);  break;     // right leg home
  }
}

// ═══════════════════════════════════════════════════
//  SETUP & LOOP
// ═══════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);

  lHip.attach(L_HIP_PIN);
  lKnee.attach(L_KNEE_PIN);
  lAnkle.attach(L_ANKLE_PIN);
  rHip.attach(R_HIP_PIN);
  rKnee.attach(R_KNEE_PIN);
  rAnkle.attach(R_ANKLE_PIN);

  allNeutral();
  delay(1000);
  Serial.println("Arduino ready — L1=6cm L2=5.5cm maxReach=11.5cm");
}

void loop() {
  // ── Read serial FIRST every iteration ─────────
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "WALK") {
      allNeutral();
      delay(500);
      walking = true;
      Serial.println("WALKING");

    } else if (command.startsWith("DIST:")) {
      float d = command.substring(5).toFloat();
      Serial.print("Approaching — dist="); Serial.println(d);

    } else if (command.startsWith("IK:") && !kicking) {
    kicking = true;
    walking = false;
    delay(300);

    int comma = command.indexOf(',');
    float dist = command.substring(3, comma).toFloat();

    Serial.print("IK command — dist=");
    Serial.println(dist);

    kickWithIK(dist);

    kicking = false;   // unlock after kick is finished
  // allow next kick later
    }
    else if (command.startsWith("TURN:")) {
  float lateral = command.substring(6).toFloat();

  // simple steering logic
  if (lateral > 1) {
    // turn right
    rHip.write(80);
    lHip.write(100);
  }
  else if (lateral < -1) {
    // turn left
    rHip.write(100);
    lHip.write(80);
  }
  else {
    // straight
    rHip.write(90);
    lHip.write(90);
  }

  walking = true;
}

  }

  // ── Non-blocking walk ─────────────────────────
  if (walking && millis() - lastPhaseTime > PHASE_DELAY) {
    runWalkPhase(walkPhase);
    walkPhase     = (walkPhase + 1) % 8;
    lastPhaseTime = millis();
  }
}