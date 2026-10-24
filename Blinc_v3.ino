// =============================================================================
//  keyBoardWarriors — nil, joy, avi
//  Wearable Navigation System for the Visually Impaired
//  ESP32 + VL53L0X (x3) + VL53L1X + MPU6050 + Dual Haptic Motors
//
//  v3.1 — bug fixes, OLED removed, non-blocking low-latency loop,
//         2-state Kalman filter (angle + gyro bias), rewritten serial monitor
//
//  v3.2 — FRONT_THRESHOLD no longer equals the sensor ceiling (FRONT was
//         permanently latched, masking DROP/RAISE/SIDE); calibration and the
//         elevation debounce are now driven by sensor samples instead of loop
//         iterations; forward readings expire; a failed ToF init releases its
//         XSHUT; VL53L1X inter-measurement period is actually applied; IMU
//         presence is probed; assorted smaller fixes marked "v3.2 FIX" below.
//
//  REQUIRES ESP32 Arduino core 3.x — ledcAttach(pin, freq, res) does not exist
//  on core 2.x (there it was ledcSetup + ledcAttachPin).
//
//  DO NOT add #include <Kalman.h>. The Kalman filter is implemented inline in
//  the IMUKalman struct below — there is no library to install. If a folder
//  named "Kalman" exists in Documents/Arduino/libraries, DELETE IT: that is
//  rfetick/Kalman, a generic matrix filter needing BasicLinearAlgebra, and it
//  will break this sketch with "'Array' does not name a type".
//
//  Libraries actually required:
//    Adafruit_VL53L0X   Adafruit_VL53L1X
//    (plus Adafruit BusIO + Adafruit Unified Sensor, pulled in automatically)
//
//  MPU6050_tockn is NO LONGER REQUIRED as of v3.3 — the IMU is driven directly.
//  You can leave the library installed; nothing includes it. Preferences.h is
//  part of the ESP32 core, not a download.
// =============================================================================

#include <Wire.h>
#include <math.h>
#include <Preferences.h>
#include "Adafruit_VL53L0X.h"
#include "Adafruit_VL53L1X.h"

//  v3.3: MPU6050_tockn is GONE — we talk to the MPU6050 directly. Not for
//  purity: the library hardcodes a configuration that is actively wrong for a
//  body-worn unit, and exposes no way to change it. Verified in its begin():
//
//    CONFIG (0x1A) = 0x00  → DLPF DISABLED. Accel bandwidth 260 Hz, gyro 256 Hz,
//                            and the gyro output rate becomes 8 kHz. We sample at
//                            200 Hz, so every footstep transient above 100 Hz
//                            folded straight back into the band as fake low-
//                            frequency tilt. No filter tuning can undo aliasing —
//                            it has to be stopped in the sensor. Biggest single
//                            accuracy win in this file.
//    ACCEL_CONFIG (0x1C) = 0x00 → +/-2 g. Heel strike on a body-mounted board
//                            routinely exceeds 2 g, and a clipped accel axis
//                            corrupts the computed angle asymmetrically (it
//                            cannot be averaged out). We use +/-4 g.
//    SMPLRT_DIV (0x19) = 0x00 → no decimation, consistent with the above.
//
//  It also offers gyro offset calibration but NO accelerometer calibration, and
//  accel bias is the dominant error in any gravity-referenced tilt estimate:
//  ~0.017 g of bias is ~1 degree of permanent tilt error that the Kalman filter
//  will faithfully track forever, because it treats the accelerometer as truth
//  for the DC angle. See imuCalibrateAccel().
//
//  What it got right and we keep: PWR_MGMT_1 = 0x01 (PLL with X-gyro reference,
//  more stable than the internal oscillator) and GYRO_CONFIG = 0x08 (+/-500 deg/s).


// =============================================================================
//  PIN DEFINITIONS
// =============================================================================

#define SDA_PIN      21
#define SCL_PIN      22

#define XSHUT_LEFT   25
#define XSHUT_FRONT  26
#define XSHUT_RIGHT  27
#define XSHUT_DOWN   14

#define MOTOR_LEFT   13

//  CAUTION: GPIO12 is MTDI, an ESP32 strapping pin — it selects flash voltage
//  while the chip is in reset. ledcAttach() runs long after strapping is
//  sampled, so no code here can protect against it: if the motor driver holds
//  this pin HIGH at power-up, the ESP32 sets VDD_SDIO to 1.8 V and fails to
//  boot. If you ever see intermittent boot failures, move this to GPIO 4, 16,
//  17, 18 or 19 (all unencumbered) rather than debugging the firmware.
#define MOTOR_RIGHT  12


// =============================================================================
//  I2C ADDRESSES  (reassigned at boot via XSHUT sequencing)
// =============================================================================

#define ADDR_LEFT    0x30
#define ADDR_FRONT   0x31
#define ADDR_RIGHT   0x32
#define ADDR_DOWN    0x29   // VL53L1X keeps default — different chip, no conflict


// =============================================================================
//  TIMING / RATE CONFIGURATION
//
//  The old build ran three BLOCKING rangingTest() calls back-to-back plus a
//  delay(30). Each blocking single-shot costs ~35-50 ms, so the real loop rate
//  was ~7-10 Hz, not the 33 Hz the comment claimed. Everything below now runs
//  in continuous mode and is polled non-blocking — no delay() anywhere.
// =============================================================================

#define I2C_CLOCK            400000  // Hz — 4x faster than the 100 kHz default
#define TOF_BUDGET_US         20000  // VL53L0X timing budget (us) — speed/accuracy
#define TOF_PERIOD_MS            25  // VL53L0X continuous inter-measurement period
#define TOF_DOWN_BUDGET_MS       33  // VL53L1X timing budget (15/20/33/50/100/200/500)
#define TOF_DOWN_PERIOD_MS       40  // VL53L1X inter-measurement period

//  v3.3: was IMU_INTERVAL_MS, which became dead when the IMU switched to
//  DATA_RDY-driven sampling. Repurposed: don't bother asking the MPU whether it
//  has a sample until a sample could plausibly exist. Without this gate the
//  DATA_RDY poll is one extra I2C transaction on every one of the ~1-2 kHz loop
//  iterations, all of it contending with the four ToF sensors on the same bus.
//  4 ms against a 5 ms sample period means one or two polls per sample.
#define IMU_POLL_AFTER_US      4000
#define HAPTIC_INTERVAL_MS       10  // 100 Hz motor refresh
#define SERIAL_INTERVAL_MS       50  // 20 Hz telemetry (keeps UART off the hot path)
#define HUMAN_INTERVAL_MS       500  // 2 Hz readable dashboard; detection stays full-speed
#define DATA_TIMEOUT_MS         300  // a sensor value older than this is stale


// =============================================================================
//  DETECTION THRESHOLDS
// =============================================================================

#define SIDE_THRESHOLD          110  // cm — lateral obstacle triggers STATE_SIDE

//  v3.2 FIX: this was 200 — identical to TOF_MAX_CM. Since pollForwardSensor
//  discards anything above TOF_MAX_CM, distFront was always in [2,200] and
//  `distFront < FRONT_THRESHOLD` was true for EVERY valid reading. FRONT is the
//  top priority, so indoors the unit sat in STATE_FRONT permanently and DROP /
//  RAISE / SIDE were unreachable. Keep this comfortably below TOF_MAX_CM.
#define FRONT_THRESHOLD         150  // cm — frontal obstacle triggers STATE_FRONT

#define ELEVATION_THRESHOLD      15  // cm — terrain deviation triggers RAISE/DROP
#define ELEVATION_HYSTERESIS      5  // cm — must fall this far back inside to clear
#define ELEVATION_CONFIRM         2  // consecutive DOWN-SENSOR FRAMES before RAISE/DROP

//  v3.2 FIX: renamed from CALIBRATION_LOOPS. These are down-sensor samples now,
//  not loop iterations — see calibrateHeight(). At the VL53L1X's ~25 Hz that is
//  about 4 s of standing still on flat ground.
#define CALIBRATION_SAMPLES     100  // samples averaged to establish H_global
#define TILT_REJECT_DEG          35  // ignore height readings past this tilt
#define PULSE_INTERVAL          250  // ms - front pattern toggle period

// Fixed laser-beam direction in MPU6050 body coordinates. The default +Z
// matches the old assumption that the MPU Z-axis and VL53L1X optical axis are
// parallel. The vector is normalized in code, so measured values may be used.
#define LASER_DIR_X             0.0f
#define LASER_DIR_Y             0.0f
#define LASER_DIR_Z             1.0f
#define MIN_VERTICAL_DOT        0.20f // reject a nearly-horizontal/upward beam

#define TOF_MIN_CM                2  // below this the reading is noise
#define TOF_MAX_CM              200  // VL53L0X practical ceiling
#define TOF_DOWN_MAX_CM         400  // VL53L1X practical ceiling


// =============================================================================
//  SERIAL MONITOR CONFIGURATION
//
//  MODE_PLOT   — default: one labelled line per sample for Arduino Serial Plotter.
//  MODE_CSV    — machine-readable data for Excel / pandas logging.
//  MODE_HUMAN  — readable multi-line dashboard for Arduino Serial Monitor.
//  MODE_OFF    — silence. Use this when timing the loop or during a demo.
//
//  Runtime commands (type into Serial Monitor, newline-terminated):
//     p  → switch to PLOT         c → switch to CSV
//     h  → switch to DASHBOARD
//     o  → switch to OFF          r → re-run height calibration
//     s  → print a one-shot status block
//     z  → re-measure gyro bias + re-seed attitude (hold still, ~10 s)
//     a  → 6-point accelerometer calibration wizard (bench, ~2 min, saved to NVS)
//     k  → print current IMU calibration constants
//     x  → erase the stored accelerometer calibration
// =============================================================================

#define SERIAL_BAUD    500000   // 115200 was ~5 ms/line — enough to stall the loop

//  NOTE: this enum is called TelemetryMode, not SerialMode. The ESP32 core's
//  HardwareSerial.h already declares `typedef uart_mode_t SerialMode;`, so an
//  enum by that name collides with it at global scope.
enum TelemetryMode { MODE_PLOT, MODE_CSV, MODE_HUMAN, MODE_OFF };
TelemetryMode serialMode = MODE_PLOT;


// =============================================================================
//  KALMAN FILTER TUNING
//
//  Q_ANGLE   — process noise on the angle state. Up = trust the gyro less.
//  Q_BIAS    — process noise on the gyro-bias state. Up = bias adapts faster.
//  R_MEASURE — accelerometer measurement noise. Up = trust the accelerometer
//              less; important here because footstep shock is injected straight
//              into the accel signal on a body-worn unit.
// =============================================================================

#define KALMAN_Q_ANGLE     0.001f
#define KALMAN_Q_BIAS      0.003f
#define KALMAN_R_MEASURE   0.050f

//  v3.3 — ADAPTIVE MEASUREMENT TRUST
//
//  A fixed R is the wrong model for a walking human. The accelerometer measures
//  gravity PLUS whatever linear acceleration the body is undergoing, so during a
//  heel strike atan2() of it is not the tilt angle at all — it is a lie with the
//  same magnitude as a real 20-degree lean. With R fixed, the filter took every
//  one of those lies as a valid observation and the angle bounced at stride rate.
//
//  Gravity has a signature: |a| == 1 g. Any deviation is linear acceleration, and
//  the size of the deviation bounds how wrong the derived angle can be. So:
//    err = | |a| - 1g |
//    R   = R_MEASURE * (1 + R_GAIN * err^2)     → smoothly distrust
//    err > GATE_G                               → skip the correction entirely,
//                                                 coast on the gyro for that frame
//
//  This is why the gyro-bias state matters so much: during the gated frames the
//  bias estimate is the only thing keeping the coasted angle from drifting.
#define ACCEL_R_GAIN        60.0f  // how fast R inflates inside the gate
#define ACCEL_GATE_G         0.35f // |a|-1g beyond this → gyro-only for this frame
#define KALMAN_BIAS_LIMIT    5.0f  // deg/s — clamp; post-calibration residual is
                                   // well under 1, so this only stops windup if a
                                   // sustained accel disturbance fools the filter

//  Gyro sampling. The MPU is configured for a 200 Hz internal rate and we consume
//  every sample via its DATA_RDY flag, so dt is measured, not assumed.
#define IMU_DLPF_CFG            3  // 0=off(256Hz) 1=188 2=98 3=42 4=20 5=10 6=5 Hz
                                   // 3 keeps head-turn bandwidth (~42 Hz) while
                                   // killing shock; drop to 4 if still too lively
#define IMU_SMPLRT_DIV          4  // 1 kHz / (1+4) = 200 Hz internal sample rate
#define IMU_STALE_US       200000  // no fresh sample this long → flag the IMU dead

//  Calibration
#define GYRO_CAL_SAMPLES     2000  // ~10 s at 200 Hz
#define GYRO_CAL_MAX_DEV      2.0f // deg/s — per-sample spread above which we
                                   // declare "you moved" and refuse the result
#define ACCEL_CAL_SAMPLES     400  // ~2 s averaged per face of the 6-point routine


// =============================================================================
//  STATE MACHINE
//
//  RAISE → ground rose  (brick / bump / step up)     delta POSITIVE
//  DROP  → ground fell  (pothole / stair down)       delta NEGATIVE
//
//  Priority (highest → lowest): DROP > RAISE > FRONT > SIDE > CLEAR
//  Terrain hazards override frontal/side obstacles; DROP remains highest because
//  a pothole, ledge, or stair-down presents the greatest immediate fall risk.
// =============================================================================

enum SystemState {
  STATE_CLEAR,
  STATE_SIDE,
  STATE_RAISE,
  STATE_DROP,
  STATE_FRONT
};

SystemState currentState     = STATE_CLEAR;
SystemState previousState    = STATE_CLEAR;

const char* stateName(SystemState s) {
  switch (s) {
    case STATE_CLEAR: return "CLEAR";
    case STATE_SIDE:  return "SIDE";
    case STATE_RAISE: return "RAISE";
    case STATE_DROP:  return "DROP";
    case STATE_FRONT: return "FRONT";
  }
  return "?";
}

const char* stateMeaning(SystemState s) {
  switch (s) {
    case STATE_CLEAR: return "Path clear";
    case STATE_SIDE:  return "Obstacle on left and/or right";
    case STATE_RAISE: return "Ground rose: bump, curb, or step up";
    case STATE_DROP:  return "Ground fell: pothole, ledge, or step down";
    case STATE_FRONT: return "Obstacle directly ahead";
  }
  return "Unknown";
}


// =============================================================================
//  SENSOR OBJECTS
// =============================================================================

Adafruit_VL53L0X leftSensor;
Adafruit_VL53L0X frontSensor;
Adafruit_VL53L0X rightSensor;
Adafruit_VL53L1X downSensor = Adafruit_VL53L1X();

bool okLeft = false, okFront = false, okRight = false, okDown = false, okIMU = false;


// =============================================================================
//  MPU6050 — direct register driver
//
//  Registers we touch, and why each value was chosen, are documented at the
//  include block above. Scale factors follow from the two range selections:
//    GYRO_CONFIG  = 0x08 → FS_SEL  = 1 → +/-500 deg/s → 65.5 LSB/(deg/s)
//    ACCEL_CONFIG = 0x08 → AFS_SEL = 1 → +/-4 g       →  8192 LSB/g
// =============================================================================

#define MPU_ADDR            0x68
#define MPU_SMPLRT_DIV      0x19
#define MPU_CONFIG          0x1A
#define MPU_GYRO_CONFIG     0x1B
#define MPU_ACCEL_CONFIG    0x1C
#define MPU_INT_PIN_CFG     0x37
#define MPU_INT_ENABLE      0x38
#define MPU_INT_STATUS      0x3A
#define MPU_ACCEL_XOUT_H    0x3B
#define MPU_PWR_MGMT_1      0x6B
#define MPU_WHO_AM_I        0x75

#define GYRO_LSB_PER_DPS    65.5f
#define ACCEL_LSB_PER_G     8192.0f

// — Calibration, persisted in NVS so a power cycle does not throw it away
Preferences imuPrefs;

float gyroBias[3]  = {0, 0, 0};          // deg/s, body axes, from imuCalibrateGyro
bool  gyroCalValid = false;              // have we ever accepted a gyro cal?
float accelBias[3] = {0, 0, 0};          // g,     body axes, from imuCalibrateAccel
float accelScale[3] = {1.0f, 1.0f, 1.0f};
bool  accelCalValid = false;

// — Latest calibrated sample
float imuAx = 0, imuAy = 0, imuAz = 1;   // g
float imuGx = 0, imuGy = 0, imuGz = 0;   // deg/s, bias-corrected
float imuTempC = 0;
float accelMag = 1.0f;                   // |a| in g — drives the adaptive trust
bool  accelTrusted = true;               // false while gated out
float lastR = KALMAN_R_MEASURE;          // for telemetry

static bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}

static bool mpuRead(uint8_t reg, uint8_t *buf, uint8_t len) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;      // repeated start
  if (Wire.requestFrom((int)MPU_ADDR, (int)len) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

//  True when the MPU has a new sample waiting. Reading INT_STATUS clears the
//  flag, so consuming one sample per assertion gives us exactly the sensor's
//  200 Hz with no duplicated gyro samples — a duplicate integrated against a
//  real dt is a small but systematic angle error, which is exactly the kind of
//  thing that ruins a "precise" build.
static bool mpuDataReady() {
  uint8_t s;
  if (!mpuRead(MPU_INT_STATUS, &s, 1)) return false;
  return (s & 0x01) != 0;
}

//  Burst-read all 14 bytes: accel XYZ, temperature, gyro XYZ. One transaction
//  matters — accel and gyro must come from the same instant or the Euler-rate
//  transform below mixes an attitude with a rate that no longer matches it.
static bool mpuReadRaw(int16_t *a, int16_t *g, int16_t *t) {
  uint8_t b[14];
  if (!mpuRead(MPU_ACCEL_XOUT_H, b, 14)) return false;

  a[0] = (int16_t)((b[0]  << 8) | b[1]);
  a[1] = (int16_t)((b[2]  << 8) | b[3]);
  a[2] = (int16_t)((b[4]  << 8) | b[5]);
  *t   = (int16_t)((b[6]  << 8) | b[7]);
  g[0] = (int16_t)((b[8]  << 8) | b[9]);
  g[1] = (int16_t)((b[10] << 8) | b[11]);
  g[2] = (int16_t)((b[12] << 8) | b[13]);
  return true;
}

//  Raw → calibrated engineering units.
static bool mpuSample() {
  int16_t a[3], g[3], t;
  if (!mpuReadRaw(a, g, &t)) return false;

  imuAx = ((a[0] / ACCEL_LSB_PER_G) - accelBias[0]) * accelScale[0];
  imuAy = ((a[1] / ACCEL_LSB_PER_G) - accelBias[1]) * accelScale[1];
  imuAz = ((a[2] / ACCEL_LSB_PER_G) - accelBias[2]) * accelScale[2];

  imuGx = (g[0] / GYRO_LSB_PER_DPS) - gyroBias[0];
  imuGy = (g[1] / GYRO_LSB_PER_DPS) - gyroBias[1];
  imuGz = (g[2] / GYRO_LSB_PER_DPS) - gyroBias[2];

  imuTempC = (t / 340.0f) + 36.53f;      // datasheet formula

  accelMag = sqrtf(imuAx * imuAx + imuAy * imuAy + imuAz * imuAz);
  return true;
}


// =============================================================================
//  KALMAN FILTER — 2-state (angle + gyro bias), one instance per axis
//
//  The v2 filter carried a bias term but updated it with an ad-hoc rule applied
//  AFTER the angle correction, using a residual that had already been partly
//  absorbed. Sign handling was inconsistent and the term could wind up instead
//  of converging. This is the standard formulation: bias is a real state, and
//  the covariance matrix decides how much of each innovation belongs to angle
//  versus bias.
// =============================================================================

struct IMUKalman {
  float angle = 0.0f;
  float bias  = 0.0f;
  float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};

  // Full reset — angle re-seeded, bias estimate discarded. Use at boot and after
  // imuCalibrateGyro(), i.e. only when the gyro offsets themselves just changed.
  void reset(float startAngle) {
    angle = startAngle;
    bias  = 0.0f;
    P[0][0] = 0.0f; P[0][1] = 0.0f;
    P[1][0] = 0.0f; P[1][1] = 0.0f;
  }

  // v3.2 FIX: the ±180 wrap guard used to call reset(), which zeroed `bias` and
  // threw away a slowly-converged estimate every time the angle wrapped. A wrap
  // says nothing about gyro bias, so keep bias and P[1][1] (its uncertainty) and
  // only clear the angle variance and the cross terms — the angle is now known
  // exactly from the accelerometer.
  void resetAngle(float startAngle) {
    angle = startAngle;
    P[0][0] = 0.0f; P[0][1] = 0.0f; P[1][0] = 0.0f;
  }

  // Prediction step only — integrate the bias-corrected gyro rate and grow the
  // covariance. Used on its own when the accelerometer is untrustworthy (see
  // ACCEL_GATE_G): the estimate coasts on the gyro and P[0][0] grows, so the
  // first believable accel sample afterwards is given proportionally more weight.
  void predict(float newRate, float dt) {
    float rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + KALMAN_Q_ANGLE);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += KALMAN_Q_BIAS * dt;
  }

  // Full predict + correct. R is passed in per-call so the caller can inflate it
  // when linear acceleration is contaminating the gravity measurement.
  float update(float newAngle, float newRate, float dt, float R) {
    predict(newRate, dt);

    // ── Update ── correct with the accelerometer angle
    float S = P[0][0] + R;
    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    float y = newAngle - angle;          // innovation
    angle += K0 * y;
    bias  += K1 * y;

    // Clamp the bias state. A real MPU6050 gyro bias after calibration is well
    // under 1 deg/s; anything approaching this limit means the filter is being
    // driven by a sustained accel disturbance rather than by real bias, and
    // letting it wind up would corrupt the angle long after the disturbance ends.
    if (bias >  KALMAN_BIAS_LIMIT) bias =  KALMAN_BIAS_LIMIT;
    if (bias < -KALMAN_BIAS_LIMIT) bias = -KALMAN_BIAS_LIMIT;

    float P00 = P[0][0];
    float P01 = P[0][1];
    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;

    // Force symmetry. The two cross terms are mathematically identical but drift
    // apart through float rounding over millions of updates, and an asymmetric
    // covariance can eventually make the filter behave oddly. Cheap insurance.
    float sym = 0.5f * (P[0][1] + P[1][0]);
    P[0][1] = sym;
    P[1][0] = sym;

    return angle;
  }
};

IMUKalman kalRoll, kalPitch;


// =============================================================================
//  RUNTIME STATE
// =============================================================================

// — Orientation
float pitch = 0, roll = 0;

//  v3.3 FIX: dt used to come from millis(). At a 5 ms update interval that is 1 ms
//  of quantization on a 5 ms number — up to 20% error on every gyro integration
//  step, which shows up directly as angle error and as a bias the filter then
//  tries to explain. micros() makes the quantization irrelevant. Unsigned
//  subtraction handles the ~71 min rollover correctly; do not make these signed.
unsigned long lastIMUTimeUs = 0;
unsigned long lastIMUSampleUs = 0;   // when we last got a real sample, for staleness
bool          imuStale = false;
float         imuRateHz = 0;         // measured IMU sample rate, for telemetry
unsigned long imuSampleCount = 0;

// — Distance readings (cm). -1 means "no valid reading".
float distLeft = -1, distFront = -1, distRight = -1, distDown = -1;
unsigned long stampLeft = 0, stampFront = 0, stampRight = 0, stampDown = 0;

// — Tilt-compensated ground height
float height          = -1;   // latest raw vertical ground distance
unsigned long stampHeight = 0;
float heightCurrent   = -1;   // unfiltered: updated on every valid down frame

// v3.2 FIX: stampHeight value the detectors last consumed. calibrateHeight() and
// the elevation confirm counter must advance once per DOWN-SENSOR FRAME, not once
// per loop iteration — the loop runs ~1-2 kHz while the VL53L1X delivers ~25 Hz.
unsigned long lastHeightSample = 0;

// — Ground height calibration
double Height_global    = 0;
bool   calibrated       = false;
double calibrationSum   = 0;
int    calibrationCount = 0;

// — Elevation debounce
int  elevationRaw   = 0;   //  1 rose, -1 fell, 0 flat
int  elevationState = 0;   // confirmed value after debounce
int  elevationCount = 0;

// — Motor output
int leftPWM = 0, rightPWM = 0;

// — Schedulers
unsigned long lastHapticTime = 0;
unsigned long lastSerialTime = 0;

// — Loop instrumentation
unsigned long loopCount = 0;
unsigned long loopWindowStart = 0;
float measuredLoopHz = 0;

// — Front pulse timing
bool          pulseState = false;
unsigned long lastPulse  = 0;


// =============================================================================
//  FORWARD DECLARATIONS
// =============================================================================

void noPattern();
void patternSide(bool la, float l, bool ra, float r);
void patternFront(float front);
void patternDrop();
void patternRaise();
void resetPatterns();
static void driveMotors(int l, int r);
void printCsvHeader();
void printStatusBlock();
void startHeightCalibration();
void seedOrientationFromAccel();
bool imuBegin();
bool imuCalibrateGyro();
void imuCalibrateAccel();
void imuLoadCalibration();
void imuSaveCalibration();
void imuPrintCalibration();


// =============================================================================
//  IMU CALIBRATION
//
//  Two separate things, commonly confused:
//
//  GYRO BIAS — the rate each axis reports while perfectly still. Drifts with
//    temperature, so it is measured at every boot. Cheap: hold still, average.
//    The Kalman bias state then tracks the slow residual from there.
//
//  ACCEL BIAS + SCALE — the dominant error source for gravity-referenced tilt,
//    and the thing MPU6050_tockn never addressed. Each axis has an offset and a
//    slightly wrong sensitivity from the factory. Because the filter treats the
//    accelerometer as truth for the DC angle, an uncorrected 0.017 g bias is
//    ~1 degree of permanent tilt error — it does not average out and no amount of
//    Q/R tuning touches it. Measured once with the 6-point method and stored in
//    NVS, because it is a property of the board, not of the session.
//
//    6-point method: rest the board on each of its six faces in turn. For each
//    axis the resting extremes are +1 g and -1 g, so
//        bias  = (max + min) / 2          (the midpoint that should read zero)
//        scale = 2 / (max - min)          (the span that should be exactly 2 g)
//    This needs no jig and no known angles — only that a face is roughly level,
//    because near an extremum the reading is insensitive to small tilt (cos error).
// =============================================================================

void imuLoadCalibration() {
  imuPrefs.begin("blincimu", true);                 // read-only
  accelCalValid = imuPrefs.getBool("acalok", false);
  if (accelCalValid) {
    accelBias[0]  = imuPrefs.getFloat("abx", 0.0f);
    accelBias[1]  = imuPrefs.getFloat("aby", 0.0f);
    accelBias[2]  = imuPrefs.getFloat("abz", 0.0f);
    accelScale[0] = imuPrefs.getFloat("asx", 1.0f);
    accelScale[1] = imuPrefs.getFloat("asy", 1.0f);
    accelScale[2] = imuPrefs.getFloat("asz", 1.0f);
  }
  imuPrefs.end();
}

void imuSaveCalibration() {
  imuPrefs.begin("blincimu", false);                // read-write
  imuPrefs.putFloat("abx", accelBias[0]);
  imuPrefs.putFloat("aby", accelBias[1]);
  imuPrefs.putFloat("abz", accelBias[2]);
  imuPrefs.putFloat("asx", accelScale[0]);
  imuPrefs.putFloat("asy", accelScale[1]);
  imuPrefs.putFloat("asz", accelScale[2]);
  imuPrefs.putBool("acalok", true);
  imuPrefs.end();
  accelCalValid = true;
}

void imuPrintCalibration() {
  Serial.println();
  Serial.println("------------- IMU CALIBRATION -------------");
  Serial.printf(" Gyro bias  : %+7.3f %+7.3f %+7.3f deg/s\n",
                gyroBias[0], gyroBias[1], gyroBias[2]);
  if (accelCalValid) {
    Serial.printf(" Accel bias : %+7.4f %+7.4f %+7.4f g\n",
                  accelBias[0], accelBias[1], accelBias[2]);
    Serial.printf(" Accel scale: %7.5f %7.5f %7.5f\n",
                  accelScale[0], accelScale[1], accelScale[2]);
    Serial.println(" Accel cal  : loaded from NVS");
  } else {
    Serial.println(" Accel cal  : NONE — run 'a' once on a level table.");
    Serial.println("              Expect up to a few degrees of static tilt");
    Serial.println("              error until you do.");
  }
  Serial.printf(" Temp       : %.1f C\n", imuTempC);
  Serial.println("------------------------------------------");
  Serial.println();
}

//  Average N still samples per axis. Also measures the spread, so "I calibrated
//  while the bus was moving" becomes a visible error rather than a silent 3 deg/s
//  bias baked in for the whole session.
bool imuCalibrateGyro() {
  double sum[3] = {0, 0, 0};
  float  mn[3]  = {  1e9f,  1e9f,  1e9f };
  float  mx[3]  = { -1e9f, -1e9f, -1e9f };

  float saveBias[3] = { gyroBias[0], gyroBias[1], gyroBias[2] };
  gyroBias[0] = gyroBias[1] = gyroBias[2] = 0;      // measure raw

  Serial.printf("[IMU] Gyro calibration — hold still (%d samples, ~%d s)\n",
                GYRO_CAL_SAMPLES, GYRO_CAL_SAMPLES / 200);

  int got = 0;
  unsigned long started = millis();
  while (got < GYRO_CAL_SAMPLES) {
    if (millis() - started > 20000) {               // sensor stopped talking
      gyroBias[0] = saveBias[0];
      gyroBias[1] = saveBias[1];
      gyroBias[2] = saveBias[2];
      Serial.println("[IMU] gyro calibration TIMED OUT — previous bias kept");
      return false;
    }
    //  yield() matters here. This loop runs for ~10 s at loopTask priority; the
    //  old code got away with a tight wait only because MPU6050_tockn used
    //  delay(2) between samples. Starving the idle task that long trips the ESP32
    //  task watchdog and reboots mid-calibration.
    yield();
    if (!mpuDataReady()) continue;
    if (!mpuSample())    continue;

    float g[3] = { imuGx, imuGy, imuGz };
    for (int i = 0; i < 3; i++) {
      sum[i] += g[i];
      if (g[i] < mn[i]) mn[i] = g[i];
      if (g[i] > mx[i]) mx[i] = g[i];
    }
    got++;
  }

  float worstSpread = 0;
  for (int i = 0; i < 3; i++) {
    float spread = mx[i] - mn[i];
    if (spread > worstSpread) worstSpread = spread;
  }

  if (worstSpread > GYRO_CAL_MAX_DEV) {
    if (gyroCalValid) {
      // We have something better already — keep it.
      gyroBias[0] = saveBias[0];
      gyroBias[1] = saveBias[1];
      gyroBias[2] = saveBias[2];
      Serial.printf("[IMU] REJECTED — spread %.2f deg/s exceeds %.2f. The device "
                    "moved; previous bias kept. Retry with 'z'.\n",
                    worstSpread, (float)GYRO_CAL_MAX_DEV);
      return false;
    }
    // Nothing better exists. Rejecting outright would leave bias at ZERO, and an
    // uncorrected MPU6050 can sit several deg/s off — far worse than a mean taken
    // while the unit wobbled. Take it, flag it, and tell them to redo it.
    for (int i = 0; i < 3; i++) gyroBias[i] = (float)(sum[i] / got);
    Serial.printf("[IMU] POOR CALIBRATION — spread %.2f deg/s exceeds %.2f, but "
                  "no previous bias exists so this one is being used.\n",
                  worstSpread, (float)GYRO_CAL_MAX_DEV);
    Serial.printf("      Bias: %+.3f %+.3f %+.3f deg/s. Put the unit down and "
                  "send 'z' to redo it properly.\n",
                  gyroBias[0], gyroBias[1], gyroBias[2]);
    return false;
  }

  for (int i = 0; i < 3; i++) gyroBias[i] = (float)(sum[i] / got);
  gyroCalValid = true;

  Serial.printf("[IMU] Gyro bias: %+.3f %+.3f %+.3f deg/s (spread %.2f, %.1f C)\n",
                gyroBias[0], gyroBias[1], gyroBias[2], worstSpread, imuTempC);
  return true;
}

//  Blocking, interactive, bench-only. Six prompts, one per face. Motors are held
//  off throughout and the caller re-seeds the filters afterwards.
void imuCalibrateAccel() {
  static const char* prompts[6] = {
    "Z UP    — board flat, components facing the ceiling",
    "Z DOWN  — board flat, components facing the floor",
    "X UP    — on its edge, +X axis pointing at the ceiling",
    "X DOWN  — on its edge, -X axis pointing at the ceiling",
    "Y UP    — on its other edge, +Y axis pointing at the ceiling",
    "Y DOWN  — on its other edge, -Y axis pointing at the ceiling"
  };

  float mn[3] = {  1e9f,  1e9f,  1e9f };
  float mx[3] = { -1e9f, -1e9f, -1e9f };

  // Measure raw — existing correction must not feed back into the new numbers.
  float sb[3] = { accelBias[0],  accelBias[1],  accelBias[2]  };
  float ss[3] = { accelScale[0], accelScale[1], accelScale[2] };
  accelBias[0] = accelBias[1] = accelBias[2] = 0;
  accelScale[0] = accelScale[1] = accelScale[2] = 1.0f;

  Serial.println();
  Serial.println("=========== 6-POINT ACCEL CALIBRATION ===========");
  Serial.println(" Bench procedure. The unit is NOT navigating while this runs.");
  Serial.println(" Rest the board on a flat table in each orientation asked for,");
  Serial.println(" let go of it, then press ENTER. Roughly level is good enough —");
  Serial.println(" near 1 g a few degrees of tilt costs almost nothing.");
  Serial.println(" Send 'q' at any prompt to abort and keep the old values.");
  Serial.println("=================================================");

  while (Serial.available()) Serial.read();         // drain

  for (int face = 0; face < 6; face++) {
    Serial.printf("\n [%d/6] %s\n       ENTER when steady > ", face + 1, prompts[face]);

    for (;;) {
      yield();                        // this wait is unbounded — user-paced
      if (!Serial.available()) continue;
      char c = Serial.read();
      if (c == 'q' || c == 'Q') {
        accelBias[0]  = sb[0]; accelBias[1]  = sb[1]; accelBias[2]  = sb[2];
        accelScale[0] = ss[0]; accelScale[1] = ss[1]; accelScale[2] = ss[2];
        Serial.println("\n[IMU] aborted — previous accel calibration kept");
        return;
      }
      if (c == '\n' || c == '\r') break;
    }

    double sum[3] = {0, 0, 0};
    int got = 0;
    unsigned long started = millis();
    while (got < ACCEL_CAL_SAMPLES && millis() - started < 8000) {
      yield();
      if (!mpuDataReady()) continue;
      if (!mpuSample())    continue;
      sum[0] += imuAx; sum[1] += imuAy; sum[2] += imuAz;
      got++;
    }

    if (got < ACCEL_CAL_SAMPLES / 2) {
      accelBias[0]  = sb[0]; accelBias[1]  = sb[1]; accelBias[2]  = sb[2];
      accelScale[0] = ss[0]; accelScale[1] = ss[1]; accelScale[2] = ss[2];
      Serial.println("\n[IMU] lost the sensor mid-calibration — aborted");
      return;
    }

    float avg[3] = { (float)(sum[0] / got), (float)(sum[1] / got),
                     (float)(sum[2] / got) };
    for (int i = 0; i < 3; i++) {
      if (avg[i] < mn[i]) mn[i] = avg[i];
      if (avg[i] > mx[i]) mx[i] = avg[i];
    }
    Serial.printf("       captured  %+.4f %+.4f %+.4f g\n",
                  avg[0], avg[1], avg[2]);
  }

  // Every axis must have seen both extremes, or the faces were not what we asked
  // for. A correct run gives a span near 2 g on all three.
  for (int i = 0; i < 3; i++) {
    float span = mx[i] - mn[i];
    if (span < 1.5f || span > 2.5f) {
      accelBias[0]  = sb[0]; accelBias[1]  = sb[1]; accelBias[2]  = sb[2];
      accelScale[0] = ss[0]; accelScale[1] = ss[1]; accelScale[2] = ss[2];
      Serial.printf("\n[IMU] REJECTED — axis %d span %.3f g, expected ~2.0. The "
                    "orientations did not cover all six faces. Nothing saved.\n",
                    i, span);
      return;
    }
  }

  for (int i = 0; i < 3; i++) {
    accelBias[i]  = 0.5f * (mx[i] + mn[i]);
    accelScale[i] = 2.0f / (mx[i] - mn[i]);
  }

  imuSaveCalibration();
  Serial.println("\n[IMU] accel calibration saved to NVS — survives power cycles");
  imuPrintCalibration();
}


// =============================================================================
//  IMU INITIALISATION
// =============================================================================

bool imuBegin() {
  uint8_t who;
  if (!mpuRead(MPU_WHO_AM_I, &who, 1)) {
    Serial.println("[FAIL] MPU6050 not answering at 0x68");
    return false;
  }
  //  WHO_AM_I reports 0x68 for a genuine MPU6050. Clones report 0x70 (MPU6500),
  //  0x72, 0x73, 0x75... all register-compatible for what we use, so warn rather
  //  than refuse — but the gyro/accel noise specs differ, so it is worth knowing.
  if (who != 0x68) {
    Serial.printf("[WARN] WHO_AM_I = 0x%02X, not 0x68 — clone or MPU6500 variant\n",
                  who);
  }

  if (!mpuWrite(MPU_PWR_MGMT_1, 0x80)) return false;   // device reset
  delay(100);
  if (!mpuWrite(MPU_PWR_MGMT_1, 0x01)) return false;   // wake, PLL / X-gyro clock
  delay(10);

  if (!mpuWrite(MPU_CONFIG,       IMU_DLPF_CFG)) return false;  // the aliasing fix
  if (!mpuWrite(MPU_SMPLRT_DIV,   IMU_SMPLRT_DIV)) return false;
  if (!mpuWrite(MPU_GYRO_CONFIG,  0x08)) return false;  // +/-500 deg/s
  if (!mpuWrite(MPU_ACCEL_CONFIG, 0x08)) return false;  // +/-4 g

  //  REQUIRED for mpuDataReady() to work, and easy to miss: the DATA_RDY bit in
  //  INT_STATUS only ever gets set if the data-ready interrupt is ENABLED here.
  //  Leave this out and INT_STATUS bit 0 reads 0 forever — mpuDataReady() never
  //  returns true, updateOrientation() silently does nothing, and pitch/roll sit
  //  at their seed values with no error anywhere. We never wire the physical INT
  //  pin; we only want the status bit.
  if (!mpuWrite(MPU_INT_ENABLE, 0x01)) return false;    // DATA_RDY_EN

  //  INT_PIN_CFG: LATCH_INT_EN=0, INT_RD_CLEAR=0 → the status bit latches until
  //  INT_STATUS is read, which is exactly the handshake mpuDataReady() relies on
  //  to hand out each sample exactly once. This is the reset default; written
  //  explicitly so a future edit cannot break the sampling by changing it.
  if (!mpuWrite(MPU_INT_PIN_CFG, 0x00)) return false;
  delay(50);

  static const float dlpfHz[7] = { 256, 188, 98, 42, 20, 10, 5 };
  Serial.printf("[ OK ] MPU6050 @ 0x%02X — DLPF %.0f Hz, %d Hz rate, "
                "+/-500 deg/s, +/-4 g\n",
                MPU_ADDR, dlpfHz[IMU_DLPF_CFG], 1000 / (1 + IMU_SMPLRT_DIV));
  return true;
}


// =============================================================================
//  SENSOR INITIALISATION
//
//  BUG FIX: the old code called leftSensor.begin(0x29, &Wire).
//  Adafruit_VL53L0X::begin() is begin(addr, debug, TwoWire*) — the second
//  parameter is a bool. Passing &Wire there silently coerced the pointer to
//  `true`, switching every VL53L0X into debug mode (which floods the UART) and
//  leaving all three on the default Wire bus by luck rather than intent.
//  Adafruit_VL53L1X::begin() genuinely is begin(addr, TwoWire*), so only the
//  VL53L0X calls were wrong — easy to miss because they looked identical.
// =============================================================================

bool initVL53L0X(Adafruit_VL53L0X &sensor, uint8_t xshutPin, uint8_t newAddr,
                 const char* label) {

  digitalWrite(xshutPin, HIGH);
  delay(20);

  if (!sensor.begin(newAddr, false, &Wire)) {
    //  v3.2 FIX: the old code returned here with XSHUT still HIGH, leaving a
    //  half-alive sensor squatting on the default 0x29. The next sensor was then
    //  brought up and told to move off 0x29 while a second device was answering
    //  there, so it usually failed too — and finally the VL53L1X came up at
    //  ADDR_DOWN (0x29) into a live collision. One bad solder joint took out all
    //  four sensors instead of one. Power the failed part down and move on.
    digitalWrite(xshutPin, LOW);
    Serial.printf("[FAIL] VL53L0X %s did not respond — powered down\n", label);
    return false;
  }

  sensor.setMeasurementTimingBudgetMicroSeconds(TOF_BUDGET_US);
  sensor.startRangeContinuous(TOF_PERIOD_MS);

  Serial.printf("[ OK ] VL53L0X %s @ 0x%02X\n", label, newAddr);
  return true;
}


// =============================================================================
//  SETUP
// =============================================================================

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);

  Serial.println();
  Serial.println("=============================================");
  Serial.println(" Wearable Navigation System  —  v3");
  Serial.println(" keyBoardWarriors : nil / joy / avi");
  Serial.println("=============================================");

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(I2C_CLOCK);            // was missing — bus ran at 100 kHz

  // ── Motors first, so nothing buzzes during sensor boot ────────────────────
  ledcAttach(MOTOR_LEFT,  5000, 8);
  ledcAttach(MOTOR_RIGHT, 5000, 8);
  ledcWrite(MOTOR_LEFT,  0);
  ledcWrite(MOTOR_RIGHT, 0);

  // ── MPU6050 ───────────────────────────────────────────────────────────────
  //  v3.2 FIX: okIMU used to be set to true unconditionally, because
  //  MPU6050_tockn::begin() returns void and cannot report failure. imuBegin()
  //  checks WHO_AM_I and every config write. With no IMU the unit still runs:
  //  pitch/roll stay 0, so height is uncompensated raw distance and RAISE/DROP
  //  works as long as the sensor is held level.
  okIMU = imuBegin();

  if (okIMU) {
    imuLoadCalibration();          // accel bias/scale from NVS, if ever measured
    imuCalibrateGyro();            // always re-measured: bias is temperature-bound
    if (!accelCalValid) {
      Serial.println("[WARN] No accel calibration stored. Static tilt may be off");
      Serial.println("       by a few degrees. Run 'a' once on a level table —");
      Serial.println("       it persists across power cycles.");
    }
  } else {
    Serial.println("[FAIL] MPU6050 unavailable — tilt compensation off");
  }

  // ── ToF sensors — sequential XSHUT addressing ─────────────────────────────
  pinMode(XSHUT_LEFT,  OUTPUT);
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_DOWN,  OUTPUT);

  digitalWrite(XSHUT_LEFT,  LOW);
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_DOWN,  LOW);
  delay(50);

  okLeft  = initVL53L0X(leftSensor,  XSHUT_LEFT,  ADDR_LEFT,  "LEFT");
  okFront = initVL53L0X(frontSensor, XSHUT_FRONT, ADDR_FRONT, "FRONT");
  okRight = initVL53L0X(rightSensor, XSHUT_RIGHT, ADDR_RIGHT, "RIGHT");

  digitalWrite(XSHUT_DOWN, HIGH);
  delay(20);
  if (downSensor.begin(ADDR_DOWN, &Wire)) {
    downSensor.setTimingBudget(TOF_DOWN_BUDGET_MS);

    //  v3.2 FIX: TOF_DOWN_PERIOD_MS was defined and never used, so the
    //  inter-measurement period sat at whatever VL53L1X_SensorInit()'s default
    //  register block gives (~100 ms, i.e. ~10 Hz) — setTimingBudget() does not
    //  touch it, and the Adafruit wrapper that would have set it is commented out
    //  in Adafruit_VL53L1X.cpp. Reach past the wrapper to the inherited ULD call.
    //  Must run AFTER setTimingBudget, and the ULD requires period >= budget
    //  without checking it, so keep TOF_DOWN_PERIOD_MS >= TOF_DOWN_BUDGET_MS.
    if (downSensor.VL53L1X_SetInterMeasurementInMs(TOF_DOWN_PERIOD_MS) != 0) {
      Serial.println("[WARN] could not set VL53L1X inter-measurement period");
    }

    downSensor.startRanging();
    okDown = true;
    Serial.printf("[ OK ] VL53L1X DOWN @ 0x%02X (%d ms budget / %d ms period)\n",
                  ADDR_DOWN, TOF_DOWN_BUDGET_MS, TOF_DOWN_PERIOD_MS);
  } else {
    digitalWrite(XSHUT_DOWN, LOW);
    Serial.println("[FAIL] VL53L1X DOWN did not respond");
  }

  if (!okDown) {
    Serial.println("[WARN] No down sensor — RAISE/DROP detection disabled");
  }

  //  v3.2: re-assert the bus clock. Both Adafruit begin() implementations call
  //  theWire->begin() internally; current ESP32 cores skip re-init on an already
  //  started bus and preserve the clock, but this is cheap insurance against a
  //  core update quietly dropping us back to 100 kHz.
  Wire.setClock(I2C_CLOCK);

  // ── Seed the Kalman filters from the accelerometer ────────────────────────
  //  BUG FIX: v2 stamped lastMPUTime BEFORE ~250 ms of ToF init delays, so the
  //  very first dt was ~0.3 s and the gyro integration jumped on frame one.
  seedOrientationFromAccel();

  unsigned long now = millis();
  lastHapticTime  = now;
  lastSerialTime  = now;
  loopWindowStart = now;

  Serial.println("---------------------------------------------");
  Serial.println(" Commands: p=plot  c=csv  h=dashboard  o=off  s=status");
  Serial.println("           r=height recal   z=gyro recal");
  Serial.println("           a=accel cal      k=show cal  x=erase cal");
  Serial.println("---------------------------------------------");

  if (serialMode == MODE_CSV) printCsvHeader();
}


// =============================================================================
//  ORIENTATION — Kalman fusion, fixed-rate
// =============================================================================

//  Seed both filters straight from gravity. Used at boot and by the 'z' command,
//  so the two paths can no longer drift apart (see handleSerialCommands).
void seedOrientationFromAccel() {
  if (!okIMU) { roll = 0; pitch = 0; return; }

  // Average a short burst rather than trusting one sample — the seed sets the
  // filter's starting point and a single noisy frame there takes seconds to decay.
  double sx = 0, sy = 0, sz = 0;
  int got = 0;
  unsigned long started = millis();
  while (got < 64 && millis() - started < 500) {
    yield();
    if (!mpuDataReady()) continue;
    if (!mpuSample())    continue;
    sx += imuAx; sy += imuAy; sz += imuAz;
    got++;
  }

  // Stamp the clocks either way — leaving lastIMUSampleUs at 0 would make the
  // staleness watchdog fire immediately on the first loop.
  lastIMUTimeUs   = micros();
  lastIMUSampleUs = lastIMUTimeUs;
  imuStale        = false;

  if (got == 0) { roll = 0; pitch = 0; return; }

  float ax = (float)(sx / got), ay = (float)(sy / got), az = (float)(sz / got);

  kalRoll.reset(atan2f(ay, az) * 180.0f / PI);
  kalPitch.reset(atan2f(-ax, sqrtf(ay * ay + az * az)) * 180.0f / PI);
  roll  = kalRoll.angle;
  pitch = kalPitch.angle;

}

// =============================================================================
//  ORIENTATION UPDATE
//
//  v3.3 — two substantive changes beyond the plumbing.
//
//  1. BODY RATES ARE NOT EULER RATES. The old code fed gx straight in as "roll
//     rate" and gy as "pitch rate". That is only true at zero tilt. The actual
//     relationship is
//         roll_dot  = gx + gy*sin(roll)*tan(pitch) + gz*cos(roll)*tan(pitch)
//         pitch_dot = gy*cos(roll) - gz*sin(roll)
//     Note gz appears in both — and gz is the LARGEST rate on a walking human,
//     because turning your body is a yaw motion. At 30 degrees of pitch,
//     tan(30) = 0.58, so more than half the yaw rate was being silently injected
//     into roll as error every time the wearer turned a corner. This was the
//     single largest dynamic error in the old estimator and it grew with tilt,
//     which is exactly when the height reading needs the angle most.
//
//  2. ADAPTIVE ACCEL TRUST. See the ACCEL_R_GAIN block near the top.
//
//  Note on where bias is removed: the per-axis gyro biases are subtracted in
//  mpuSample(), i.e. in the BODY frame, before the transform — which is the frame
//  they were measured in and where they physically live. Each Kalman filter's own
//  `bias` state then absorbs the slow thermal residual on the resulting Euler
//  rate. Doing it the other way round (letting the filters remove raw body bias
//  through the transform) would smear one axis' bias across two outputs.
// =============================================================================

void updateOrientation() {
  if (!okIMU) return;

  // Staleness watchdog — a wearable's IMU can drop off the bus mid-walk.
  unsigned long nowUs = micros();
  if (nowUs - lastIMUSampleUs > IMU_STALE_US) {
    if (!imuStale) {
      imuStale = true;
      Serial.println("[WARN] IMU went quiet — angles frozen, tilt comp suspect");
    }
  }

  // Cheap gate before spending an I2C transaction on the DATA_RDY poll.
  if (nowUs - lastIMUTimeUs < IMU_POLL_AFTER_US) return;

  if (!mpuDataReady()) return;
  if (!mpuSample())    return;

  nowUs = micros();
  float dt = (nowUs - lastIMUTimeUs) * 1e-6f;
  lastIMUTimeUs   = nowUs;
  lastIMUSampleUs = nowUs;

  if (imuStale) {
    imuStale = false;
    Serial.println("[ OK ] IMU back");
  }

  // Guard against a stall or a first frame after a blocking calibration.
  if (dt <= 0 || dt > 0.5f) dt = 1.0f / (1000.0f / (1 + IMU_SMPLRT_DIV));

  imuSampleCount++;

  // ── Gravity direction from the accelerometer ──────────────────────────────
  float accRoll  = atan2f(imuAy, imuAz) * 180.0f / PI;
  float accPitch = atan2f(-imuAx, sqrtf(imuAy * imuAy + imuAz * imuAz)) * 180.0f / PI;

  // ── Body rates → Euler rates, using the current attitude estimate ─────────
  float rollRad = roll * PI / 180.0f;
  float sr = sinf(rollRad), cr = cosf(rollRad);

  // tan() blows up at +/-90 degrees of pitch. The transform is genuinely singular
  // there (gimbal lock), so clamp rather than pretend: past 60 degrees this unit
  // has already stopped trusting the height reading anyway (TILT_REJECT_DEG=35).
  float pitchClamped = pitch;
  if (pitchClamped >  60.0f) pitchClamped =  60.0f;
  if (pitchClamped < -60.0f) pitchClamped = -60.0f;
  float tp = tanf(pitchClamped * PI / 180.0f);

  float rollRate  = imuGx + imuGy * sr * tp + imuGz * cr * tp;
  float pitchRate = imuGy * cr - imuGz * sr;

  // ── How much do we believe gravity right now? ─────────────────────────────
  float err = fabsf(accelMag - 1.0f);
  accelTrusted = (err <= ACCEL_GATE_G);
  lastR = KALMAN_R_MEASURE * (1.0f + ACCEL_R_GAIN * err * err);

  if (!accelTrusted) {
    // Linear acceleration is swamping gravity — this frame's "angle" is fiction.
    // Coast on the gyro. P grows, so the next believable sample pulls harder.
    kalRoll.predict(rollRate,   dt);
    kalPitch.predict(pitchRate, dt);
    roll  = kalRoll.angle;
    pitch = kalPitch.angle;
    return;
  }

  // atan2(ay, az) wraps at +/-180. Without this the filter chases the long way
  // round and the angle takes seconds to settle after a sharp lean.
  // v3.2 FIX: resetAngle() instead of reset() — keeps the converged gyro bias.
  if (fabsf(accRoll - kalRoll.angle) > 90.0f) {
    kalRoll.resetAngle(accRoll);
    roll = accRoll;
  } else {
    roll = kalRoll.update(accRoll, rollRate, dt, lastR);
  }

  pitch = kalPitch.update(accPitch, pitchRate, dt, lastR);
}


// =============================================================================
//  DISTANCE SENSORS — non-blocking polling
//
//  BUG FIX: v2 never checked RangeStatus, so an out-of-range VL53L0X reading
//  (8190 mm) entered the pipeline as a legitimate 819 cm value.
// =============================================================================

void pollForwardSensor(Adafruit_VL53L0X &sensor, bool ok,
                       float &out, unsigned long &stamp) {
  if (!ok) return;
  if (!sensor.isRangeComplete()) return;

  uint16_t mm = sensor.readRangeResult();
  uint8_t  st = sensor.readRangeStatus();

  if (st == 4 || mm == 0 || mm > 8000) {   // 4 = phase fail / out of range
    out = -1;
  } else {
    float cm = mm / 10.0f;
    out = (cm < TOF_MIN_CM || cm > TOF_MAX_CM) ? -1 : cm;
  }
  stamp = millis();
}

//  v3.2 FIX: stampLeft/Front/Right were written on every poll and read NOWHERE —
//  DATA_TIMEOUT_MS was only ever used by heightFresh(). If a forward sensor stops
//  producing while the bus still works (a reset drops it out of continuous mode,
//  a connector works loose under vibration), isRangeComplete() simply never comes
//  back true, the last good value sits in distX forever, and the state machine
//  keeps alerting on a reading from minutes ago. Expiring the value fixes the
//  state machine and the telemetry log together.
static void expireForwardSensor(float &out, unsigned long stamp) {
  if (out > 0 && millis() - stamp >= DATA_TIMEOUT_MS) out = -1;
}

void pollDownSensor() {
  if (!okDown) return;
  if (!downSensor.dataReady()) return;

  int16_t mm = downSensor.distance();
  downSensor.clearInterrupt();

  if (mm <= 0) { distDown = -1; stampDown = millis(); return; }

  float cm = mm / 10.0f;
  distDown  = (cm < TOF_MIN_CM || cm > TOF_DOWN_MAX_CM) ? -1 : cm;
  stampDown = millis();

  // ── Dual-axis tilt compensation ─────────────────────────────────────────
  //  H = D_raw * dot(laser direction, gravity direction). This supports a fixed
  //  laser-to-MPU mounting offset; +Z reduces to the former cosine expression.
  //  Past ~35 degrees, reject frames rather than feed weak geometry to detection.
  if (distDown > 0 && fabs(pitch) < TILT_REJECT_DEG && fabs(roll) < TILT_REJECT_DEG) {
    float rollRad  = roll  * PI / 180.0f;
    float pitchRad = pitch * PI / 180.0f;

    // Unit gravity/up direction expressed in MPU body coordinates. These signs
    // match the accelerometer angle equations used by updateOrientation().
    float gravityX = -sinf(pitchRad);
    float gravityY =  sinf(rollRad) * cosf(pitchRad);
    float gravityZ =  cosf(rollRad) * cosf(pitchRad);

    float laserNorm = sqrtf(LASER_DIR_X * LASER_DIR_X +
                            LASER_DIR_Y * LASER_DIR_Y +
                            LASER_DIR_Z * LASER_DIR_Z);

    if (laserNorm > 0.001f) {
      // vertical height = range * dot(laser direction, gravity direction)
      float verticalDot = (LASER_DIR_X * gravityX +
                           LASER_DIR_Y * gravityY +
                           LASER_DIR_Z * gravityZ) / laserNorm;

      if (verticalDot >= MIN_VERTICAL_DOT) {
        height = distDown * verticalDot;
        heightCurrent = height; // intentionally raw: no EMA or moving-average lag
        stampHeight = millis();
      }
    }
  }
}

bool heightFresh() {
  return (heightCurrent > 0) && (millis() - stampHeight < DATA_TIMEOUT_MS);
}


// =============================================================================
//  GROUND HEIGHT CALIBRATION
// =============================================================================

void startHeightCalibration() {
  calibrated       = false;
  calibrationSum   = 0;
  calibrationCount = 0;
  Height_global    = 0;
  Serial.println("[CAL] Re-calibrating — stand still on flat ground");
}

//  v3.2 FIX: this used to be called unconditionally every loop iteration, while
//  heightCurrent only changes when the VL53L1X delivers a frame. heightFresh()
//  checks that the sample is RECENT, not that it is NEW, so the same reading was
//  accumulated once per loop — CALIBRATION_LOOPS (100) was satisfied in ~100
//  iterations, i.e. tens of milliseconds, from one or two distinct samples.
//  H_global was effectively a single unaveraged frame. Now gated on a new sample.
void calibrateHeight(bool newSample) {
  if (calibrated || !newSample || !heightFresh()) return;

  calibrationSum += heightCurrent;
  calibrationCount++;

  if (calibrationCount >= CALIBRATION_SAMPLES) {
    Height_global = calibrationSum / calibrationCount;
    calibrated    = true;
    Serial.printf("[CAL] H_global locked: %.1f cm (%d samples)\n",
                  Height_global, calibrationCount);
  }
}


// =============================================================================
//  ELEVATION CHANGE DETECTION
//
//  delta = H_global - H_current
//    ground rose (bump/step up) → H falls → delta POSITIVE →  1 → STATE_RAISE
//    ground fell (pothole)      → H rises → delta NEGATIVE → -1 → STATE_DROP
//
//  BUG FIX: v2 recomputed elevation from `down` every loop, and `down` was -1
//  on every iteration where the VL53L1X had no fresh sample. Elevation
//  therefore collapsed to 0 between samples and the DROP/RAISE haptic pattern
//  stuttered in and out. Now the last valid height is held for DATA_TIMEOUT_MS
//  and a short confirm counter suppresses single-frame spikes. (That last claim
//  was aspirational until v3.2 — see the note on the confirm counter below.)
// =============================================================================

//  v3.2 FIX: the confirm counter advanced once per LOOP ITERATION, so
//  ELEVATION_CONFIRM (2) was satisfied ~1 ms later and a single spiky frame
//  latched STATE_DROP — the debounce this function's comment advertised did not
//  exist. It now advances only on a new down-sensor frame, so ELEVATION_CONFIRM
//  really means 2 consecutive frames (~80 ms at a 40 ms period).
//
//  The staleness branch below still runs every loop on purpose: if the down
//  sensor dies while a state is latched, that state must clear within
//  DATA_TIMEOUT_MS rather than waiting for a frame that will never arrive.
void updateElevation(bool newSample) {
  if (!calibrated || !heightFresh()) {
    elevationRaw = 0;
    elevationCount = 0;
    elevationState = 0;
    return;
  }

  if (!newSample) return;

  float delta = Height_global - heightCurrent;

  // Hysteresis: once latched, a smaller deviation keeps the state alive.
  float enter = ELEVATION_THRESHOLD;
  float exit  = ELEVATION_THRESHOLD - ELEVATION_HYSTERESIS;
  float bound = (elevationState != 0) ? exit : enter;

  int raw = 0;
  if      (delta >  bound) raw =  1;
  else if (delta < -bound) raw = -1;

  if (raw == elevationRaw) {
    if (elevationCount < ELEVATION_CONFIRM) elevationCount++;
  } else {
    elevationRaw   = raw;
    elevationCount = 1;
  }

  if (elevationCount >= ELEVATION_CONFIRM) elevationState = elevationRaw;
}


// =============================================================================
//  SERIAL MONITOR
// =============================================================================

// Arduino Serial Plotter mode: one sample per line, every value explicitly
// labelled. Open Tools > Serial Plotter at 500000 baud and enable/disable traces
// in its legend as needed. Units are included in each label where practical.
void printPlot() {
  float shownHeight = heightFresh() ? heightCurrent : -1.0f;
  float shownGlobal = calibrated ? (float)Height_global : -1.0f;
  float shownDelta  = (calibrated && heightFresh())
                    ? (float)Height_global - heightCurrent : 0.0f;

  Serial.printf("Left_cm:%.2f\tFront_cm:%.2f\tRight_cm:%.2f\t"
                "DownRaw_cm:%.2f\tVerticalHeight_cm:%.2f\t"
                "GlobalHeight_cm:%.2f\tHeightChange_cm:%.2f\t"
                "Pitch_deg:%.3f\tRoll_deg:%.3f\t"
                "AccX_g:%.4f\tAccY_g:%.4f\tAccZ_g:%.4f\tAccMag_g:%.4f\t"
                "GyroX_dps:%.3f\tGyroY_dps:%.3f\tGyroZ_dps:%.3f\t"
                "KalmanPitchBias:%.4f\tKalmanRollBias:%.4f\t"
                "AccelTrusted:%d\tIMUTemp_C:%.1f\tState:%d\t"
                "LeftPWM:%d\tRightPWM:%d\tLoopHz:%.1f\tIMUHz:%.1f\t"
                "LeftOK:%d\tFrontOK:%d\tRightOK:%d\tDownOK:%d\tIMUOK:%d\n",
                distLeft, distFront, distRight,
                distDown, shownHeight, shownGlobal, shownDelta,
                pitch, roll,
                imuAx, imuAy, imuAz, accelMag,
                imuGx, imuGy, imuGz,
                kalPitch.bias, kalRoll.bias,
                accelTrusted ? 1 : 0, imuTempC, (int)currentState,
                leftPWM, rightPWM, measuredLoopHz, imuRateHz,
                okLeft ? 1 : 0, okFront ? 1 : 0, okRight ? 1 : 0,
                okDown ? 1 : 0, (okIMU && !imuStale) ? 1 : 0);
}

void printCsvHeader() {
  Serial.println("ms,left_cm,front_cm,right_cm,down_raw_cm,vertical_height_cm,"
                 "global_height_cm,height_delta_cm,pitch_deg,roll_deg,"
                 "acc_x_g,acc_y_g,acc_z_g,acc_mag_g,acc_trusted,"
                 "gyro_x_dps,gyro_y_dps,gyro_z_dps,kalman_pitch_bias_dps,"
                 "kalman_roll_bias_dps,imu_temp_c,left_pwm,right_pwm,state,"
                 "loop_hz,imu_hz");
}

void printCsv() {
  float shownHeight = heightFresh() ? heightCurrent : -1.0f;
  float shownGlobal = calibrated ? (float)Height_global : -1.0f;
  float shownDelta  = (calibrated && heightFresh())
                    ? (float)Height_global - heightCurrent : 0.0f;

  Serial.printf("%lu,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.3f,%.3f,"
                "%.4f,%.4f,%.4f,%.4f,%d,%.4f,%.4f,%.4f,%.4f,%.4f,"
                "%.1f,%d,%d,%d,%.1f,%.1f\n",
                millis(),
                distLeft, distFront, distRight, distDown,
                shownHeight, shownGlobal, shownDelta,
                pitch, roll,
                imuAx, imuAy, imuAz, accelMag, accelTrusted ? 1 : 0,
                imuGx, imuGy, imuGz, kalPitch.bias, kalRoll.bias, imuTempC,
                leftPWM, rightPWM, (int)currentState, measuredLoopHz, imuRateHz);
}

static void printHumanDistance(const char *name, float value) {
  Serial.printf("%-6s: ", name);
  if (value > 0) Serial.printf("%7.1f cm", value);
  else           Serial.print("INVALID   ");
}

void printHuman() {
  bool validHeight = heightFresh();
  float delta = (calibrated && validHeight)
              ? (float)Height_global - heightCurrent : 0.0f;

  Serial.println();
  Serial.println("===================== BLINC LIVE DATA =====================");
  Serial.printf("CURRENT STATE : %-6s | %s\n",
                stateName(currentState), stateMeaning(currentState));
  Serial.printf("SENSOR HEALTH : Left %s | Front %s | Right %s | Down %s | MPU %s\n",
                okLeft ? "OK" : "FAIL", okFront ? "OK" : "FAIL",
                okRight ? "OK" : "FAIL", okDown ? "OK" : "FAIL",
                okIMU ? (imuStale ? "STALE" : "OK") : "FAIL");

  Serial.println("------------------- OBSTACLE DISTANCE ---------------------");
  printHumanDistance("Left", distLeft);   Serial.print(" | ");
  printHumanDistance("Front", distFront); Serial.print(" | ");
  printHumanDistance("Right", distRight); Serial.println();

  Serial.println("-------------------- DOWN / GROUND ------------------------");
  Serial.print("Raw down distance       : ");
  if (distDown > 0) Serial.printf("%7.1f cm  (VL53L1X beam distance)\n", distDown);
  else              Serial.println("INVALID    (no valid VL53L1X range)");
  if (validHeight) {
    Serial.printf("Vertical ground height  : %7.1f cm  (beam projected onto gravity)\n",
                  heightCurrent);
  } else {
    Serial.println("Vertical ground height  : INVALID    (no recent usable reading)");
  }

  if (calibrated) {
    Serial.printf("Global/base height      : %7.1f cm  (flat-ground calibration)\n",
                  (float)Height_global);
    if (validHeight) {
      Serial.printf("Height change           : %+7.1f cm  (+ ground rose, - ground fell)\n",
                    delta);
    } else {
      Serial.println("Height change           : INVALID");
    }
  } else {
    Serial.printf("Global/base height      : CALIBRATING %d/%d samples\n",
                  calibrationCount, CALIBRATION_SAMPLES);
    Serial.println("Height change           : waiting for calibration");
  }

  Serial.println("------------------------- MPU6050 -------------------------");
  Serial.printf("Orientation             : Pitch %+7.2f deg | Roll %+7.2f deg\n",
                pitch, roll);
  Serial.printf("Accelerometer           : X %+7.3f g | Y %+7.3f g | Z %+7.3f g\n",
                imuAx, imuAy, imuAz);
  Serial.printf("Acceleration magnitude  : %7.3f g | Kalman accel: %s\n",
                accelMag, accelTrusted ? "TRUSTED" : "IGNORED - gyro coasting");
  Serial.printf("Gyroscope corrected     : X %+7.2f | Y %+7.2f | Z %+7.2f deg/s\n",
                imuGx, imuGy, imuGz);
  Serial.printf("Kalman residual bias    : Pitch %+7.3f | Roll %+7.3f deg/s\n",
                kalPitch.bias, kalRoll.bias);
  Serial.printf("IMU temperature / rate  : %5.1f C | %5.1f Hz | Accel cal: %s\n",
                imuTempC, imuRateHz, accelCalValid ? "LOADED" : "NOT SET");

  Serial.println("------------------------- OUTPUT --------------------------");
  Serial.printf("Vibration motors        : Left %3d/255 | Right %3d/255\n",
                leftPWM, rightPWM);
  Serial.printf("Main loop rate          : %.1f Hz\n", measuredLoopHz);
  Serial.println("Commands: p=plot h=dashboard c=CSV o=off s=status r=height recal");
  Serial.println("          z=gyro recal  a=accel cal  k=show cal  x=erase cal");
  Serial.println("===========================================================");
}

void printStatusBlock() {
  Serial.println();
  Serial.println("---------------- STATUS ----------------");
  Serial.printf(" Sensors : L%s  F%s  R%s  D%s  IMU%s\n",
                okLeft ? "+" : "-", okFront ? "+" : "-", okRight ? "+" : "-",
                okDown ? "+" : "-", okIMU ? "+" : "-");
  Serial.printf(" Loop    : %.1f Hz   IMU %.1f Hz%s\n",
                measuredLoopHz, imuRateHz, imuStale ? "  [STALE]" : "");
  Serial.printf(" Angles  : pitch %.2f  roll %.2f\n", pitch, roll);
  Serial.printf(" Residual: pitch %.4f  roll %.4f deg/s (Kalman bias state)\n",
                kalPitch.bias, kalRoll.bias);
  Serial.printf(" Accel   : |a| %.4f g  %s  R %.4f\n",
                accelMag, accelTrusted ? "trusted" : "GATED (coasting)", lastR);
  Serial.printf(" AccelCal: %s\n", accelCalValid ? "loaded" : "NOT SET — run 'a'");
  Serial.printf(" Temp    : %.1f C\n", imuTempC);
  Serial.printf(" Height  : %s  H_global %.1f cm\n",
                calibrated ? "calibrated" : "CALIBRATING",
                (float)Height_global);
  Serial.printf(" State   : %s\n", stateName(currentState));
  Serial.println("----------------------------------------");
  Serial.println();
}

void handleSerialCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'p': serialMode = MODE_PLOT; Serial.println("[UI] serial plot"); break;
      case 'c': serialMode = MODE_CSV;   printCsvHeader();               break;
      case 'h': serialMode = MODE_HUMAN; Serial.println("[UI] live dashboard"); break;
      case 'o': serialMode = MODE_OFF;   Serial.println("[UI] off");     break;
      case 'r': startHeightCalibration();                                break;
      case 's': printStatusBlock();                                      break;
      //  v3.2 FIX (three things): roll was re-seeded from gravity but pitch was
      //  re-seeded from the current FILTERED pitch, locking in whatever drift was
      //  already there — the asymmetry made 'z' a partial no-op for pitch. The
      //  gyro calibration also blocks (~10 s now) inside loop(), during which the
      //  motors used to hold their last PWM, so a re-zero while an alert was
      //  firing meant seconds of uncommanded buzzing. And the IMU clock was left
      //  stale. seedOrientationFromAccel() handles both axes identically and
      //  restamps; driveMotors(0,0) makes the blocking wait silent.
      case 'z':
        if (!okIMU) { Serial.println("[IMU] no IMU — nothing to zero"); break; }
        driveMotors(0, 0);
        imuCalibrateGyro();
        seedOrientationFromAccel();
        resetPatterns();
        break;

      //  v3.3 — accelerometer calibration. Bench-only and blocking; this is the
      //  one that removes static tilt error, and it only needs doing once per
      //  board because the result is stored in NVS.
      case 'a':
        if (!okIMU) { Serial.println("[IMU] no IMU"); break; }
        driveMotors(0, 0);
        imuCalibrateAccel();
        seedOrientationFromAccel();
        resetPatterns();
        break;

      case 'k': imuPrintCalibration(); break;

      case 'x':
        imuPrefs.begin("blincimu", false);
        imuPrefs.clear();
        imuPrefs.end();
        accelBias[0]  = accelBias[1]  = accelBias[2]  = 0.0f;
        accelScale[0] = accelScale[1] = accelScale[2] = 1.0f;
        accelCalValid = false;
        Serial.println("[IMU] stored accel calibration erased");
        break;
      default: break;
    }
  }
}

void serviceSerial() {
  handleSerialCommands();

  if (serialMode == MODE_OFF) return;
  unsigned long now = millis();
  unsigned long interval = (serialMode == MODE_HUMAN)
                         ? HUMAN_INTERVAL_MS : SERIAL_INTERVAL_MS;
  if (now - lastSerialTime < interval) return;
  lastSerialTime = now;

  if      (serialMode == MODE_PLOT) printPlot();
  else if (serialMode == MODE_CSV)  printCsv();
  else                              printHuman();
}


// =============================================================================
//  MAIN LOOP  — nothing blocks, nothing delays
// =============================================================================

void loop() {

  // ── 1. Orientation (200 Hz) ───────────────────────────────────────────────
  updateOrientation();

  // ── 2. Distance sensors (polled, non-blocking) ────────────────────────────
  pollForwardSensor(leftSensor,  okLeft,  distLeft,  stampLeft);
  pollForwardSensor(frontSensor, okFront, distFront, stampFront);
  pollForwardSensor(rightSensor, okRight, distRight, stampRight);
  pollDownSensor();

  // Drop readings that have gone stale, so a silently stalled sensor can no
  // longer drive the state machine off a value from minutes ago.
  expireForwardSensor(distLeft,  stampLeft);
  expireForwardSensor(distFront, stampFront);
  expireForwardSensor(distRight, stampRight);

  // ── 3. Height calibration + elevation analysis ────────────────────────────
  //  Both are sample-driven now: pollDownSensor() only moves stampHeight when the
  //  VL53L1X actually produced a tilt-valid frame, so this fires ~25 times a
  //  second instead of once per loop iteration.
  bool newHeightSample = (stampHeight != lastHeightSample);
  if (newHeightSample) lastHeightSample = stampHeight;

  calibrateHeight(newHeightSample);
  updateElevation(newHeightSample);

  // ── 4. Active flags ───────────────────────────────────────────────────────
  bool leftActive  = (distLeft  > 0 && distLeft  < SIDE_THRESHOLD);
  bool rightActive = (distRight > 0 && distRight < SIDE_THRESHOLD);
  bool frontActive = (distFront > 0 && distFront < FRONT_THRESHOLD);
  bool raiseActive = (elevationState ==  1);
  bool dropActive  = (elevationState == -1);

  // ── 5. Priority state selection ───────────────────────────────────────────
  if      (dropActive)                currentState = STATE_DROP;
  else if (frontActive)               currentState = STATE_FRONT;
  else if (raiseActive)               currentState = STATE_RAISE;
  else if (leftActive || rightActive) currentState = STATE_SIDE;
  else                                currentState = STATE_CLEAR;

  // BUG FIX: patternDrop/patternRaise held their step counters in statics that
  // survived a state exit, so re-entering DROP resumed mid-waveform instead of
  // restarting at full intensity — the first alert of a new hazard could be the
  // quietest one.
  if (currentState != previousState) {
    resetPatterns();
    previousState = currentState;
  }

  // ── 6. Haptic output (100 Hz) ─────────────────────────────────────────────
  unsigned long now = millis();
  if (now - lastHapticTime >= HAPTIC_INTERVAL_MS) {
    lastHapticTime = now;
    switch (currentState) {
      case STATE_FRONT: patternFront(distFront);                              break;
      case STATE_DROP:  patternDrop();                                        break;
      case STATE_RAISE: patternRaise();                                       break;
      case STATE_SIDE:  patternSide(leftActive, distLeft,
                                    rightActive, distRight);                  break;
      case STATE_CLEAR:
      default:          noPattern();                                          break;
    }
  }

  // ── 7. Telemetry (20 Hz) ──────────────────────────────────────────────────
  serviceSerial();

  // ── 8. Loop-rate measurement ──────────────────────────────────────────────
  loopCount++;
  if (now - loopWindowStart >= 1000) {
    measuredLoopHz  = loopCount * 1000.0f / (now - loopWindowStart);
    // Should read ~200 Hz. If it does not, the DLPF/SMPLRT_DIV config did not
    // take, or the I2C bus is saturated and IMU samples are being dropped —
    // either way the gyro integration is missing time and the angle will drift.
    imuRateHz       = imuSampleCount * 1000.0f / (now - loopWindowStart);
    loopCount       = 0;
    imuSampleCount  = 0;
    loopWindowStart = now;
  }
}


// =============================================================================
//  HAPTIC PATTERNS
//
//  BUG FIX: v2 wrote the mapped PWM into the leftPWM/rightPWM globals even when
//  the corresponding motor was being driven at 0, so the telemetry log showed
//  drive that never reached the hardware. The globals now mirror reality.
// =============================================================================

static unsigned long patternClock = 0;
static int           patternStep  = 0;

void resetPatterns() {
  patternClock = millis();
  patternStep  = 0;
  pulseState   = true;
  lastPulse    = millis();
}

static void driveMotors(int l, int r) {
  l = constrain(l, 0, 255);
  r = constrain(r, 0, 255);
  leftPWM  = l;
  rightPWM = r;
  ledcWrite(MOTOR_LEFT,  l);
  ledcWrite(MOTOR_RIGHT, r);
}

// CLEAR — silence confirms an unobstructed path
void noPattern() {
  driveMotors(0, 0);
}

// SIDE — independent motors, intensity rises as the obstacle closes in
void patternSide(bool la, float l, bool ra, float r) {
  int lp = la ? map(constrain((int)l, 0, SIDE_THRESHOLD), 0, SIDE_THRESHOLD, 255, 100) : 0;
  int rp = ra ? map(constrain((int)r, 0, SIDE_THRESHOLD), 0, SIDE_THRESHOLD, 255, 100) : 0;
  driveMotors(lp, rp);
}

// FRONT — bilateral pulse, intensity rises as the obstacle closes in
void patternFront(float front) {
  if (front <= 0) { noPattern(); return; }

  int strength = map(constrain((int)front, 0, FRONT_THRESHOLD),
                     0, FRONT_THRESHOLD, 255, 100);

  if (millis() - lastPulse > PULSE_INTERVAL) {
    lastPulse  = millis();
    pulseState = !pulseState;
  }

  if (pulseState) driveMotors(strength, strength);
  else            driveMotors(0, 0);
}

// DROP — ground fell (pothole / stair down) — HIGHER PRIORITY
// Escalating wave 255-180-100-180-255, 120 ms per step, no silence gaps
void patternDrop() {
  if (millis() - patternClock > 120) {
    patternClock = millis();
    if (++patternStep > 4) patternStep = 0;
  }

  static const int wave[5] = {255, 180, 100, 180, 255};
  int intensity = wave[constrain(patternStep, 0, 4)];
  driveMotors(intensity, intensity);
}

// RAISE — ground rose (brick / bump / step up) — LOWER PRIORITY
// Double-burst then silence. Less alarming than DROP: a bump is survivable.
void patternRaise() {
  if (millis() - patternClock > 120) {
    patternClock = millis();
    if (++patternStep > 5) patternStep = 0;
  }

  if (patternStep == 0 || patternStep == 2) driveMotors(240, 240);
  else                                      driveMotors(0, 0);
}
