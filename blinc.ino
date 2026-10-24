#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

#define SDA_PIN 21
#define SCL_PIN 22

#define XS_L 25
#define XS_F 26
#define XS_R 27
#define XS_D 14

#define MOT_L 13
#define MOT_R 12

#define ADDR_L 0x30
#define ADDR_F 0x31
#define ADDR_R 0x32
#define ADDR_D 0x29

#define SIDE_TH 110
#define FRONT_TH 150
#define ELEV_TH 15
#define ELEV_HYS 5
#define ELEV_CONFIRM 2
#define CAL_N 100
#define TILT_MAX 35
#define PULSE_MS 250
#define TIMEOUT_MS 300

#define TOF_MIN 2
#define TOF_MAX 200
#define TOFD_MAX 400

#define MPU 0x68
#define DLPF 3
#define SRDIV 4

#define Q_ANG 0.001f
#define Q_BIAS 0.003f
#define R_MEAS 0.050f
#define R_GAIN 60.0f
#define GATE_G 0.35f

enum tmode_t { M_PLOT, M_CSV, M_HUMAN, M_OFF };
tmode_t mode = M_PLOT;

enum st_t { S_CLEAR, S_SIDE, S_RAISE, S_DROP, S_FRONT };
st_t state = S_CLEAR;
st_t prevState = S_CLEAR;

Adafruit_VL53L0X tofL;
Adafruit_VL53L0X tofF;
Adafruit_VL53L0X tofR;
Adafruit_VL53L1X tofD = Adafruit_VL53L1X();

bool okL = false, okF = false, okR = false, okD = false, okImu = false;

Preferences prefs;

float gBias[3] = {0, 0, 0};
bool gCalOk = false;
float aBias[3] = {0, 0, 0};
float aScale[3] = {1, 1, 1};
bool aCalOk = false;

float accX = 0, accY = 0, accZ = 1;
float gyX = 0, gyY = 0, gyZ = 0;
float tempC = 0;
float aMag = 1;
bool accTrust = true;
float lastR = R_MEAS;

struct Kal {
  float angle = 0;
  float bias = 0;
  float P[2][2] = {{0, 0}, {0, 0}};

  void reset(float a) {
    angle = a;
    bias = 0;
    P[0][0] = 0; P[0][1] = 0;
    P[1][0] = 0; P[1][1] = 0;
  }

  void resetAngle(float a) {
    angle = a;
    P[0][0] = 0; P[0][1] = 0; P[1][0] = 0;
  }

  void predict(float rate, float dt) {
    float r = rate - bias;
    angle += dt * r;
    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_ANG);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_BIAS * dt;
  }

  float update(float newAngle, float rate, float dt, float R) {
    predict(rate, dt);
    float S = P[0][0] + R;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;
    float y = newAngle - angle;
    angle += K0 * y;
    bias += K1 * y;
    if (bias > 5.0) bias = 5.0;
    if (bias < -5.0) bias = -5.0;
    float p00 = P[0][0];
    float p01 = P[0][1];
    P[0][0] -= K0 * p00;
    P[0][1] -= K0 * p01;
    P[1][0] -= K1 * p00;
    P[1][1] -= K1 * p01;
    float s = 0.5f * (P[0][1] + P[1][0]);
    P[0][1] = s;
    P[1][0] = s;
    return angle;
  }
};

Kal kRoll, kPitch;

float pitch = 0, roll = 0;
unsigned long lastImuUs = 0;
unsigned long lastSampleUs = 0;
bool imuStale = false;
float imuHz = 0;
unsigned long imuCnt = 0;

float dL = -1, dF = -1, dR = -1, dD = -1;
unsigned long tL = 0, tF = 0, tR = 0, tD = 0;

float h = -1;
float hCur = -1;
unsigned long tH = 0;
unsigned long lastHSample = 0;

double Hg = 0;
bool calDone = false;
double calSum = 0;
int calCnt = 0;

int elevRaw = 0;
int elevState = 0;
int elevCnt = 0;

int pwmL = 0, pwmR = 0;

unsigned long lastHaptic = 0;
unsigned long lastSer = 0;

unsigned long loops = 0;
unsigned long loopWin = 0;
float loopHz = 0;

bool pulseOn = false;
unsigned long lastPulse = 0;

unsigned long patClock = 0;
int patStep = 0;

void motors(int l, int r);
void resetPat();
void csvHeader();
void seedAngles();

const char* stName(st_t s) {
  if (s == S_CLEAR) return "CLEAR";
  if (s == S_SIDE) return "SIDE";
  if (s == S_RAISE) return "RAISE";
  if (s == S_DROP) return "DROP";
  if (s == S_FRONT) return "FRONT";
  return "?";
}

bool mpuWr(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRd(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU, (int)n) != n) return false;
  for (int i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool mpuReady() {
  uint8_t s;
  if (!mpuRd(0x3A, &s, 1)) return false;
  return (s & 0x01) != 0;
}

bool readImu() {
  uint8_t b[14];
  if (!mpuRd(0x3B, b, 14)) return false;

  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rt  = (int16_t)((b[6] << 8) | b[7]);
  int16_t rgx = (int16_t)((b[8] << 8) | b[9]);
  int16_t rgy = (int16_t)((b[10] << 8) | b[11]);
  int16_t rgz = (int16_t)((b[12] << 8) | b[13]);

  accX = ((rax / 8192.0f) - aBias[0]) * aScale[0];
  accY = ((ray / 8192.0f) - aBias[1]) * aScale[1];
  accZ = ((raz / 8192.0f) - aBias[2]) * aScale[2];

  gyX = (rgx / 65.5f) - gBias[0];
  gyY = (rgy / 65.5f) - gBias[1];
  gyZ = (rgz / 65.5f) - gBias[2];

  tempC = (rt / 340.0f) + 36.53f;
  aMag = sqrtf(accX * accX + accY * accY + accZ * accZ);
  return true;
}

void loadCal() {
  prefs.begin("blincimu", true);
  aCalOk = prefs.getBool("acalok", false);
  if (aCalOk) {
    aBias[0] = prefs.getFloat("abx", 0);
    aBias[1] = prefs.getFloat("aby", 0);
    aBias[2] = prefs.getFloat("abz", 0);
    aScale[0] = prefs.getFloat("asx", 1);
    aScale[1] = prefs.getFloat("asy", 1);
    aScale[2] = prefs.getFloat("asz", 1);
  }
  prefs.end();
}

void saveCal() {
  prefs.begin("blincimu", false);
  prefs.putFloat("abx", aBias[0]);
  prefs.putFloat("aby", aBias[1]);
  prefs.putFloat("abz", aBias[2]);
  prefs.putFloat("asx", aScale[0]);
  prefs.putFloat("asy", aScale[1]);
  prefs.putFloat("asz", aScale[2]);
  prefs.putBool("acalok", true);
  prefs.end();
  aCalOk = true;
}

void printCal() {
  Serial.println();
  Serial.println("---- imu cal ----");
  Serial.printf("gyro bias  %+7.3f %+7.3f %+7.3f\n", gBias[0], gBias[1], gBias[2]);
  if (aCalOk) {
    Serial.printf("accel bias %+7.4f %+7.4f %+7.4f\n", aBias[0], aBias[1], aBias[2]);
    Serial.printf("accel scl  %7.5f %7.5f %7.5f\n", aScale[0], aScale[1], aScale[2]);
  } else {
    Serial.println("accel cal not done, press a");
  }
  Serial.printf("temp %.1f\n", tempC);
  Serial.println("-----------------");
}

bool calGyro() {
  double sum[3] = {0, 0, 0};
  float mn[3] = {1e9, 1e9, 1e9};
  float mx[3] = {-1e9, -1e9, -1e9};
  float old0 = gBias[0], old1 = gBias[1], old2 = gBias[2];
  gBias[0] = 0; gBias[1] = 0; gBias[2] = 0;

  Serial.println("gyro cal, dont move it...");

  int got = 0;
  unsigned long st = millis();
  while (got < 2000) {
    if (millis() - st > 20000) {
      gBias[0] = old0; gBias[1] = old1; gBias[2] = old2;
      Serial.println("gyro cal timeout, kept old");
      return false;
    }
    yield();
    if (!mpuReady()) continue;
    if (!readImu()) continue;
    float g[3] = {gyX, gyY, gyZ};
    for (int i = 0; i < 3; i++) {
      sum[i] += g[i];
      if (g[i] < mn[i]) mn[i] = g[i];
      if (g[i] > mx[i]) mx[i] = g[i];
    }
    got++;
  }

  float spread = 0;
  for (int i = 0; i < 3; i++) {
    if (mx[i] - mn[i] > spread) spread = mx[i] - mn[i];
  }

  if (spread > 2.0) {
    if (gCalOk) {
      gBias[0] = old0; gBias[1] = old1; gBias[2] = old2;
      Serial.printf("gyro cal bad (spread %.2f) - it moved, kept old, try z again\n", spread);
      return false;
    }
    for (int i = 0; i < 3; i++) gBias[i] = sum[i] / got;
    Serial.printf("gyro cal bad (spread %.2f) but using it anyway, redo with z\n", spread);
    Serial.printf("bias %+.3f %+.3f %+.3f\n", gBias[0], gBias[1], gBias[2]);
    return false;
  }

  for (int i = 0; i < 3; i++) gBias[i] = sum[i] / got;
  gCalOk = true;
  Serial.printf("gyro bias %+.3f %+.3f %+.3f  spread %.2f  %.1fC\n", gBias[0], gBias[1], gBias[2], spread, tempC);
  return true;
}

void calAccel() {
  const char* faces[6] = {
    "Z up (flat, parts facing up)",
    "Z down (flat, parts facing down)",
    "X up (on edge, +X up)",
    "X down (on edge, -X up)",
    "Y up (other edge, +Y up)",
    "Y down (other edge, -Y up)"
  };

  float mn[3] = {1e9, 1e9, 1e9};
  float mx[3] = {-1e9, -1e9, -1e9};
  float sb[3] = {aBias[0], aBias[1], aBias[2]};
  float ss[3] = {aScale[0], aScale[1], aScale[2]};
  aBias[0] = 0; aBias[1] = 0; aBias[2] = 0;
  aScale[0] = 1; aScale[1] = 1; aScale[2] = 1;

  Serial.println();
  Serial.println("==== accel cal (6 faces) ====");
  Serial.println("put it down on each side and hit enter, q = quit");

  while (Serial.available()) Serial.read();

  for (int f = 0; f < 6; f++) {
    Serial.printf("\n[%d/6] %s  > ", f + 1, faces[f]);
    while (true) {
      yield();
      if (!Serial.available()) continue;
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        for (int i = 0; i < 3; i++) { aBias[i] = sb[i]; aScale[i] = ss[i]; }
        Serial.println("\nquit, old cal kept");
        return;
      }
      if (c == '\n' || c == '\r') break;
    }

    double sum[3] = {0, 0, 0};
    int got = 0;
    unsigned long st = millis();
    while (got < 400 && millis() - st < 8000) {
      yield();
      if (!mpuReady()) continue;
      if (!readImu()) continue;
      sum[0] += accX;
      sum[1] += accY;
      sum[2] += accZ;
      got++;
    }

    if (got < 200) {
      for (int i = 0; i < 3; i++) { aBias[i] = sb[i]; aScale[i] = ss[i]; }
      Serial.println("\nlost the mpu, cal stopped");
      return;
    }

    float avg[3];
    for (int i = 0; i < 3; i++) {
      avg[i] = sum[i] / got;
      if (avg[i] < mn[i]) mn[i] = avg[i];
      if (avg[i] > mx[i]) mx[i] = avg[i];
    }
    Serial.printf("  got %+.4f %+.4f %+.4f\n", avg[0], avg[1], avg[2]);
  }

  for (int i = 0; i < 3; i++) {
    float span = mx[i] - mn[i];
    if (span < 1.5 || span > 2.5) {
      for (int j = 0; j < 3; j++) { aBias[j] = sb[j]; aScale[j] = ss[j]; }
      Serial.printf("\naxis %d span %.3f is wrong (should be ~2), not saved\n", i, span);
      return;
    }
  }

  for (int i = 0; i < 3; i++) {
    aBias[i] = 0.5f * (mx[i] + mn[i]);
    aScale[i] = 2.0f / (mx[i] - mn[i]);
  }
  saveCal();
  Serial.println("\naccel cal saved");
  printCal();
}

bool imuBegin() {
  uint8_t who;
  if (!mpuRd(0x75, &who, 1)) {
    Serial.println("mpu6050 not found!!");
    return false;
  }
  if (who != 0x68) Serial.printf("whoami = 0x%02X (clone?)\n", who);

  if (!mpuWr(0x6B, 0x80)) return false;
  delay(100);
  if (!mpuWr(0x6B, 0x01)) return false;
  delay(10);
  if (!mpuWr(0x1A, DLPF)) return false;
  if (!mpuWr(0x19, SRDIV)) return false;
  if (!mpuWr(0x1B, 0x08)) return false;
  if (!mpuWr(0x1C, 0x08)) return false;
  if (!mpuWr(0x38, 0x01)) return false;
  if (!mpuWr(0x37, 0x00)) return false;
  delay(50);

  Serial.println("mpu ok");
  return true;
}

bool initTof(Adafruit_VL53L0X &s, int xs, uint8_t addr, const char *nm) {
  digitalWrite(xs, HIGH);
  delay(20);
  if (!s.begin(addr, false, &Wire)) {
    digitalWrite(xs, LOW);
    Serial.printf("tof %s FAIL\n", nm);
    return false;
  }
  s.setMeasurementTimingBudgetMicroSeconds(20000);
  s.startRangeContinuous(25);
  Serial.printf("tof %s ok 0x%02X\n", nm, addr);
  return true;
}

void setup() {
  Serial.begin(500000);
  delay(300);
  Serial.println();
  Serial.println("BLINC v3 - keyboard warriors");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  ledcAttach(MOT_L, 5000, 8);
  ledcAttach(MOT_R, 5000, 8);
  ledcWrite(MOT_L, 0);
  ledcWrite(MOT_R, 0);

  okImu = imuBegin();
  if (okImu) {
    loadCal();
    calGyro();
    if (!aCalOk) Serial.println("no accel cal saved, tilt might be off a bit (press a)");
  } else {
    Serial.println("no mpu, tilt comp off");
  }

  pinMode(XS_L, OUTPUT);
  pinMode(XS_F, OUTPUT);
  pinMode(XS_R, OUTPUT);
  pinMode(XS_D, OUTPUT);
  digitalWrite(XS_L, LOW);
  digitalWrite(XS_F, LOW);
  digitalWrite(XS_R, LOW);
  digitalWrite(XS_D, LOW);
  delay(50);

  okL = initTof(tofL, XS_L, ADDR_L, "LEFT");
  okF = initTof(tofF, XS_F, ADDR_F, "FRONT");
  okR = initTof(tofR, XS_R, ADDR_R, "RIGHT");

  digitalWrite(XS_D, HIGH);
  delay(20);
  if (tofD.begin(ADDR_D, &Wire)) {
    tofD.setTimingBudget(33);
    if (tofD.VL53L1X_SetInterMeasurementInMs(40) != 0) Serial.println("couldnt set down period");
    tofD.startRanging();
    okD = true;
    Serial.println("tof DOWN ok");
  } else {
    digitalWrite(XS_D, LOW);
    Serial.println("tof DOWN FAIL - no raise/drop");
  }

  Wire.setClock(400000);

  seedAngles();

  unsigned long now = millis();
  lastHaptic = now;
  lastSer = now;
  loopWin = now;

  Serial.println("p c h o s r z a k x");
  if (mode == M_CSV) csvHeader();
}

void seedAngles() {
  if (!okImu) {
    roll = 0;
    pitch = 0;
    return;
  }
  double sx = 0, sy = 0, sz = 0;
  int got = 0;
  unsigned long st = millis();
  while (got < 64 && millis() - st < 500) {
    yield();
    if (!mpuReady()) continue;
    if (!readImu()) continue;
    sx += accX; sy += accY; sz += accZ;
    got++;
  }

  lastImuUs = micros();
  lastSampleUs = lastImuUs;
  imuStale = false;

  if (got == 0) {
    roll = 0;
    pitch = 0;
    return;
  }

  float x = sx / got, y = sy / got, z = sz / got;
  kRoll.reset(atan2f(y, z) * 180.0f / PI);
  kPitch.reset(atan2f(-x, sqrtf(y * y + z * z)) * 180.0f / PI);
  roll = kRoll.angle;
  pitch = kPitch.angle;
}

void updateAngles() {
  if (!okImu) return;

  unsigned long nowUs = micros();
  if (nowUs - lastSampleUs > 200000) {
    if (!imuStale) {
      imuStale = true;
      Serial.println("imu stopped responding!");
    }
  }

  if (nowUs - lastImuUs < 4000) return;
  if (!mpuReady()) return;
  if (!readImu()) return;

  nowUs = micros();
  float dt = (nowUs - lastImuUs) * 1e-6f;
  lastImuUs = nowUs;
  lastSampleUs = nowUs;

  if (imuStale) {
    imuStale = false;
    Serial.println("imu back");
  }

  if (dt <= 0 || dt > 0.5f) dt = 1.0f / (1000.0f / (1 + SRDIV));

  imuCnt++;

  float accRoll = atan2f(accY, accZ) * 180.0f / PI;
  float accPitch = atan2f(-accX, sqrtf(accY * accY + accZ * accZ)) * 180.0f / PI;

  float rr = roll * PI / 180.0f;
  float sr = sinf(rr);
  float cr = cosf(rr);
  float pc = pitch;
  if (pc > 60) pc = 60;
  if (pc < -60) pc = -60;
  float tp = tanf(pc * PI / 180.0f);

  float rollRate = gyX + gyY * sr * tp + gyZ * cr * tp;
  float pitchRate = gyY * cr - gyZ * sr;

  float err = fabsf(aMag - 1.0f);
  accTrust = (err <= GATE_G);
  lastR = R_MEAS * (1.0f + R_GAIN * err * err);

  if (!accTrust) {
    kRoll.predict(rollRate, dt);
    kPitch.predict(pitchRate, dt);
    roll = kRoll.angle;
    pitch = kPitch.angle;
    return;
  }

  if (fabsf(accRoll - kRoll.angle) > 90.0f) {
    kRoll.resetAngle(accRoll);
    roll = accRoll;
  } else {
    roll = kRoll.update(accRoll, rollRate, dt, lastR);
  }
  pitch = kPitch.update(accPitch, pitchRate, dt, lastR);
}

void pollTof(Adafruit_VL53L0X &s, bool ok, float &out, unsigned long &stamp) {
  if (!ok) return;
  if (!s.isRangeComplete()) return;
  uint16_t mm = s.readRangeResult();
  uint8_t stat = s.readRangeStatus();
  if (stat == 4 || mm == 0 || mm > 8000) {
    out = -1;
  } else {
    float cm = mm / 10.0f;
    if (cm < TOF_MIN || cm > TOF_MAX) out = -1;
    else out = cm;
  }
  stamp = millis();
}

void expire(float &v, unsigned long stamp) {
  if (v > 0 && millis() - stamp >= TIMEOUT_MS) v = -1;
}

float ldir[3] = {0.0f, 0.0f, 1.0f};

void pollDown() {
  if (!okD) return;
  if (!tofD.dataReady()) return;

  int16_t mm = tofD.distance();
  tofD.clearInterrupt();

  if (mm <= 0) {
    dD = -1;
    tD = millis();
    return;
  }

  float cm = mm / 10.0f;
  if (cm < TOF_MIN || cm > TOFD_MAX) dD = -1;
  else dD = cm;
  tD = millis();

  if (dD > 0 && fabs(pitch) < TILT_MAX && fabs(roll) < TILT_MAX) {
    float rr = roll * PI / 180.0f;
    float pr = pitch * PI / 180.0f;
    float gx_ = -sinf(pr);
    float gy_ = sinf(rr) * cosf(pr);
    float gz_ = cosf(rr) * cosf(pr);
    float n = sqrtf(ldir[0] * ldir[0] + ldir[1] * ldir[1] + ldir[2] * ldir[2]);
    if (n > 0.001f) {
      float vdot = (ldir[0] * gx_ + ldir[1] * gy_ + ldir[2] * gz_) / n;
      if (vdot >= 0.20f) {
        h = dD * vdot;
        hCur = h;
        tH = millis();
      }
    }
  }
}

bool hFresh() {
  return (hCur > 0) && (millis() - tH < TIMEOUT_MS);
}

void startCal() {
  calDone = false;
  calSum = 0;
  calCnt = 0;
  Hg = 0;
  Serial.println("recal height - stand still on flat floor");
}

void calHeight(bool newS) {
  if (calDone || !newS || !hFresh()) return;
  calSum += hCur;
  calCnt++;
  if (calCnt >= CAL_N) {
    Hg = calSum / calCnt;
    calDone = true;
    Serial.printf("H_global = %.1f cm (%d)\n", Hg, calCnt);
  }
}

void updateElev(bool newS) {
  if (!calDone || !hFresh()) {
    elevRaw = 0;
    elevCnt = 0;
    elevState = 0;
    return;
  }
  if (!newS) return;

  float delta = Hg - hCur;
  float bound;
  if (elevState != 0) bound = ELEV_TH - ELEV_HYS;
  else bound = ELEV_TH;

  int raw = 0;
  if (delta > bound) raw = 1;
  else if (delta < -bound) raw = -1;

  if (raw == elevRaw) {
    if (elevCnt < ELEV_CONFIRM) elevCnt++;
  } else {
    elevRaw = raw;
    elevCnt = 1;
  }
  if (elevCnt >= ELEV_CONFIRM) elevState = elevRaw;
}

void printPlot() {
  float sh = hFresh() ? hCur : -1.0f;
  float sg = calDone ? (float)Hg : -1.0f;
  float sd = (calDone && hFresh()) ? (float)Hg - hCur : 0.0f;
  Serial.printf("Left_cm:%.2f\tFront_cm:%.2f\tRight_cm:%.2f\tDownRaw_cm:%.2f\tVerticalHeight_cm:%.2f\t"
                "GlobalHeight_cm:%.2f\tHeightChange_cm:%.2f\tPitch_deg:%.3f\tRoll_deg:%.3f\t"
                "AccX_g:%.4f\tAccY_g:%.4f\tAccZ_g:%.4f\tAccMag_g:%.4f\t"
                "GyroX_dps:%.3f\tGyroY_dps:%.3f\tGyroZ_dps:%.3f\t"
                "KalmanPitchBias:%.4f\tKalmanRollBias:%.4f\tAccelTrusted:%d\tIMUTemp_C:%.1f\tState:%d\t"
                "LeftPWM:%d\tRightPWM:%d\tLoopHz:%.1f\tIMUHz:%.1f\t"
                "LeftOK:%d\tFrontOK:%d\tRightOK:%d\tDownOK:%d\tIMUOK:%d\n",
                dL, dF, dR, dD, sh, sg, sd, pitch, roll,
                accX, accY, accZ, aMag, gyX, gyY, gyZ,
                kPitch.bias, kRoll.bias, accTrust ? 1 : 0, tempC, (int)state,
                pwmL, pwmR, loopHz, imuHz,
                okL ? 1 : 0, okF ? 1 : 0, okR ? 1 : 0, okD ? 1 : 0, (okImu && !imuStale) ? 1 : 0);
}

void csvHeader() {
  Serial.println("ms,left_cm,front_cm,right_cm,down_raw_cm,vertical_height_cm,global_height_cm,height_delta_cm,pitch_deg,roll_deg,acc_x_g,acc_y_g,acc_z_g,acc_mag_g,acc_trusted,gyro_x_dps,gyro_y_dps,gyro_z_dps,kalman_pitch_bias_dps,kalman_roll_bias_dps,imu_temp_c,left_pwm,right_pwm,state,loop_hz,imu_hz");
}

void printCsv() {
  float sh = hFresh() ? hCur : -1.0f;
  float sg = calDone ? (float)Hg : -1.0f;
  float sd = (calDone && hFresh()) ? (float)Hg - hCur : 0.0f;
  Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,%.1f,%d,%d,%d,%.1f,%.1f\n",
                millis(), dL, dF, dR, dD, sh, sg, sd, pitch, roll,
                accX, accY, accZ, aMag, accTrust ? 1 : 0,
                gyX, gyY, gyZ, kPitch.bias, kRoll.bias, tempC,
                pwmL, pwmR, (int)state, loopHz, imuHz);
}

void printHuman() {
  bool vh = hFresh();
  Serial.println();
  Serial.println("========== BLINC ==========");
  Serial.printf("state: %s\n", stName(state));
  Serial.printf("L %s  F %s  R %s  D %s  MPU %s\n",
                okL ? "ok" : "FAIL", okF ? "ok" : "FAIL", okR ? "ok" : "FAIL", okD ? "ok" : "FAIL",
                okImu ? (imuStale ? "STALE" : "ok") : "FAIL");
  Serial.printf("left %.1f  front %.1f  right %.1f  (cm, -1 = none)\n", dL, dF, dR);
  Serial.printf("down raw %.1f cm\n", dD);
  if (vh) Serial.printf("height %.1f cm\n", hCur);
  else Serial.println("height --");
  if (calDone) {
    Serial.printf("H_global %.1f cm\n", (float)Hg);
    if (vh) Serial.printf("change %+.1f cm\n", (float)Hg - hCur);
  } else {
    Serial.printf("calibrating %d/%d\n", calCnt, CAL_N);
  }
  Serial.printf("pitch %+.2f  roll %+.2f\n", pitch, roll);
  Serial.printf("acc %+.3f %+.3f %+.3f  |a| %.3f %s\n", accX, accY, accZ, aMag, accTrust ? "" : "(ignored)");
  Serial.printf("gyro %+.2f %+.2f %+.2f\n", gyX, gyY, gyZ);
  Serial.printf("kal bias p %+.3f r %+.3f\n", kPitch.bias, kRoll.bias);
  Serial.printf("temp %.1f  imu %.1f Hz  accel cal %s\n", tempC, imuHz, aCalOk ? "yes" : "no");
  Serial.printf("motors L %d R %d   loop %.1f Hz\n", pwmL, pwmR, loopHz);
  Serial.println("===========================");
}

void printStatus() {
  Serial.println();
  Serial.println("---- status ----");
  Serial.printf("L%s F%s R%s D%s IMU%s\n", okL ? "+" : "-", okF ? "+" : "-", okR ? "+" : "-", okD ? "+" : "-", okImu ? "+" : "-");
  Serial.printf("loop %.1f Hz  imu %.1f Hz %s\n", loopHz, imuHz, imuStale ? "STALE" : "");
  Serial.printf("pitch %.2f roll %.2f\n", pitch, roll);
  Serial.printf("bias p %.4f r %.4f\n", kPitch.bias, kRoll.bias);
  Serial.printf("|a| %.4f %s R %.4f\n", aMag, accTrust ? "trusted" : "gated", lastR);
  Serial.printf("accel cal %s\n", aCalOk ? "loaded" : "NOT SET");
  Serial.printf("temp %.1f\n", tempC);
  Serial.printf("height %s  Hg %.1f\n", calDone ? "calibrated" : "calibrating", (float)Hg);
  Serial.printf("state %s\n", stName(state));
  Serial.println("----------------");
}

void serialCmds() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'p') { mode = M_PLOT; Serial.println("plot"); }
    else if (c == 'c') { mode = M_CSV; csvHeader(); }
    else if (c == 'h') { mode = M_HUMAN; Serial.println("dashboard"); }
    else if (c == 'o') { mode = M_OFF; Serial.println("off"); }
    else if (c == 'r') startCal();
    else if (c == 's') printStatus();
    else if (c == 'z') {
      if (!okImu) { Serial.println("no imu"); continue; }
      motors(0, 0);
      calGyro();
      seedAngles();
      resetPat();
    }
    else if (c == 'a') {
      if (!okImu) { Serial.println("no imu"); continue; }
      motors(0, 0);
      calAccel();
      seedAngles();
      resetPat();
    }
    else if (c == 'k') printCal();
    else if (c == 'x') {
      prefs.begin("blincimu", false);
      prefs.clear();
      prefs.end();
      for (int i = 0; i < 3; i++) { aBias[i] = 0; aScale[i] = 1; }
      aCalOk = false;
      Serial.println("accel cal erased");
    }
  }
}

void doSerial() {
  serialCmds();
  if (mode == M_OFF) return;
  unsigned long now = millis();
  unsigned long iv = (mode == M_HUMAN) ? 500 : 50;
  if (now - lastSer < iv) return;
  lastSer = now;
  if (mode == M_PLOT) printPlot();
  else if (mode == M_CSV) printCsv();
  else printHuman();
}

void motors(int l, int r) {
  l = constrain(l, 0, 255);
  r = constrain(r, 0, 255);
  pwmL = l;
  pwmR = r;
  ledcWrite(MOT_L, l);
  ledcWrite(MOT_R, r);
}

void resetPat() {
  patClock = millis();
  patStep = 0;
  pulseOn = true;
  lastPulse = millis();
}

void patOff() {
  motors(0, 0);
}

void patSide(bool la, float l, bool ra, float r) {
  int lp = 0, rp = 0;
  if (la) lp = map(constrain((int)l, 0, SIDE_TH), 0, SIDE_TH, 255, 100);
  if (ra) rp = map(constrain((int)r, 0, SIDE_TH), 0, SIDE_TH, 255, 100);
  motors(lp, rp);
}

void patFront(float d) {
  if (d <= 0) {
    patOff();
    return;
  }
  int str = map(constrain((int)d, 0, FRONT_TH), 0, FRONT_TH, 255, 100);
  if (millis() - lastPulse > PULSE_MS) {
    lastPulse = millis();
    pulseOn = !pulseOn;
  }
  if (pulseOn) motors(str, str);
  else motors(0, 0);
}

void patDrop() {
  if (millis() - patClock > 120) {
    patClock = millis();
    patStep++;
    if (patStep > 4) patStep = 0;
  }
  int wave[5] = {255, 180, 100, 180, 255};
  int v = wave[constrain(patStep, 0, 4)];
  motors(v, v);
}

void patRaise() {
  if (millis() - patClock > 120) {
    patClock = millis();
    patStep++;
    if (patStep > 5) patStep = 0;
  }
  if (patStep == 0 || patStep == 2) motors(240, 240);
  else motors(0, 0);
}

void loop() {
  updateAngles();

  pollTof(tofL, okL, dL, tL);
  pollTof(tofF, okF, dF, tF);
  pollTof(tofR, okR, dR, tR);
  pollDown();

  expire(dL, tL);
  expire(dF, tF);
  expire(dR, tR);

  bool newH = (tH != lastHSample);
  if (newH) lastHSample = tH;

  calHeight(newH);
  updateElev(newH);

  bool lOn = (dL > 0 && dL < SIDE_TH);
  bool rOn = (dR > 0 && dR < SIDE_TH);
  bool fOn = (dF > 0 && dF < FRONT_TH);
  bool upOn = (elevState == 1);
  bool dropOn = (elevState == -1);

  if (dropOn) state = S_DROP;
  else if (fOn) state = S_FRONT;
  else if (upOn) state = S_RAISE;
  else if (lOn || rOn) state = S_SIDE;
  else state = S_CLEAR;

  if (state != prevState) {
    resetPat();
    prevState = state;
  }

  unsigned long now = millis();
  if (now - lastHaptic >= 10) {
    lastHaptic = now;
    if (state == S_FRONT) patFront(dF);
    else if (state == S_DROP) patDrop();
    else if (state == S_RAISE) patRaise();
    else if (state == S_SIDE) patSide(lOn, dL, rOn, dR);
    else patOff();
  }

  doSerial();

  loops++;
  if (now - loopWin >= 1000) {
    loopHz = loops * 1000.0f / (now - loopWin);
    imuHz = imuCnt * 1000.0f / (now - loopWin);
    loops = 0;
    imuCnt = 0;
    loopWin = now;
  }
}
