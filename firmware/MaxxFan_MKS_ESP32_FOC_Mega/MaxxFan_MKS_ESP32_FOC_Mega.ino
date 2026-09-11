/*
  ============================================================================
  MaxxFan BLDC Controller - v0.3.6.3 DECOUPLED-GUI NOHALL
  Board : MKS ESP32 FOC Mega (single motor)
  Motor : StepperOnline 57BYA54-12-01
  Library: SimpleFOC 2.4.0

  Control modes:
    0) OPEN_CURRENT_FOC
       MotionControlType::velocity_openloop + TorqueControlType::foc_current
       - physical phase-current feedback
       - NO Hall initialization or rotor-position feedback

    1) HALL_CURRENT_FOC
       MotionControlType::velocity + TorqueControlType::foc_current
       - physical phase-current feedback
       - Hall rotor feedback

  Important hardware mapping used by the supplied MKS Mega examples:
    PWM U/V/W  : GPIO 32 / 33 / 25
    ENABLE     : GPIO 12  <-- confirmed by the user's working Mega setup
    Hall A/B/C : GPIO 18 / 19 / 15
    Current A/B: GPIO 39 / 36
    Shunt      : 0.01 ohm
    Gain       : 50 V/V

  IMPORTANT:
  - This sketch deliberately DOES NOT read VIN on GPIO13 while Wi-Fi is active.
    GPIO13 is ADC2 on classic ESP32 and can conflict with Wi-Fi/ADC use.
  - No RUN command is restored at boot. Electrical alignment can move the rotor.
  - Hall calibration is saved after the first successful Hall initFOC().
  - Current phase alignment is enabled; initialization may move the rotor.
  - GUI telemetry transmission is intentionally suspended while PWM is active.
  - GUI control commands are coalesced and applied after an 80 ms network-quiet window.
  - In OPEN mode, direction reversal is one continuous signed S-curve; EN stays ON.
    This restores the timing behavior of the known-smooth v0.3.7 reference: the
    browser can still command the fan, but periodic network traffic is not generated
    from the motor loop while the motor is running/ramping/reversing.
  - Final source candidate: hardware behavior must still be verified on your exact board/motor.
  ============================================================================
*/

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// Forward declarations for Arduino's .ino auto-prototype generator.
// Without these, Arduino can emit prototypes using these custom types
// before their full struct definitions, causing e.g.:
//   'Telemetry' does not name a type
struct Config;
struct Telemetry;

// ============================================================================
// Local retry policy (kept in this .ino: no extra header/library required)
// ============================================================================
struct RetryPolicy {
  static constexpr uint32_t cooldownMs = 10000;
  static constexpr uint8_t maxAttempts = 3;
  static constexpr uint32_t healthyResetMs = 60000;

  bool pending = false;
  uint8_t attempts = 0;
  uint32_t scheduledAt = 0;
  uint32_t healthyAt = 0;
  bool trackingHealthy = false;
  float percent = 0.0f;
  int8_t direction = 1;

  void cancel() {
    pending = false;
    percent = 0.0f;
    trackingHealthy = false;
  }

  void reset() {
    cancel();
    attempts = 0;
  }

  bool schedule(const char* code, float request, int8_t dir,
                bool hallAvailable, uint32_t now) {
    cancel();
    if (!code || (strcmp(code, "STALL") && strcmp(code, "OVERLOAD")) ||
        !hallAvailable || request <= 0.0f || attempts >= maxAttempts) {
      return false;
    }
    percent = request;
    direction = dir;
    scheduledAt = now;
    pending = true;
    return true;
  }

  uint32_t remainingMs(uint32_t now) const {
    if (!pending) return 0;
    const uint32_t elapsed = now - scheduledAt;
    return elapsed >= cooldownMs ? 0 : cooldownMs - elapsed;
  }

  bool due(uint32_t now) const {
    return pending && remainingMs(now) == 0;
  }

  void launched() {
    pending = false;
    ++attempts;
    trackingHealthy = false;
  }

  void observeHealthy(bool healthy, uint32_t now) {
    if (!healthy) {
      trackingHealthy = false;
      return;
    }
    if (!trackingHealthy) {
      trackingHealthy = true;
      healthyAt = now;
    }
    if (now - healthyAt >= healthyResetMs) {
      attempts = 0;
    }
  }
};

// ============================================================================
// 1. HARDWARE - CHANGE ONLY IF YOUR BOARD REVISION REQUIRES IT
// ============================================================================

static constexpr int PIN_PWM_U = 32;
static constexpr int PIN_PWM_V = 33;
static constexpr int PIN_PWM_W = 25;
static constexpr int PIN_ENABLE = 12;

static constexpr int PIN_HALL_A = 18;
static constexpr int PIN_HALL_B = 19;
static constexpr int PIN_HALL_C = 15;

static constexpr int PIN_CURRENT_A = 39;
static constexpr int PIN_CURRENT_B = 36;

static constexpr int MOTOR_POLE_PAIRS = 2;

// 57BYA54-12-01 datasheet values
static constexpr float MOTOR_PHASE_RESISTANCE_OHM = 0.27f;
static constexpr float MOTOR_PHASE_INDUCTANCE_H = 0.00057f;

static constexpr float SUPPLY_VOLTAGE_V = 12.0f;
static constexpr float CURRENT_SHUNT_OHM = 0.01f;
static constexpr float CURRENT_AMP_GAIN = 50.0f;

// Validate phase correspondence at initFOC rather than assuming board wiring.
// Alignment applies phase voltage and can move the rotor at boot.
// Never skip based only on an example for a different board revision.
static constexpr bool SKIP_CURRENT_SENSE_ALIGNMENT = false;

// ============================================================================
// 2. ABSOLUTE SAFETY CEILINGS
//    GUI / NVS values are clamped below these values.
//    Driver uses full bus for PWM centering. Each d/q voltage is <=3 V;
//    vector magnitude <=sqrt(2)*3 V, below the 6 V half-bus at nominal 12 V.
// ============================================================================

static constexpr float HARD_MAX_CURRENT_A = 2.0f;
static constexpr float HARD_MAX_MOTOR_VOLTAGE_V = 3.0f;
static constexpr float HARD_DRIVER_VOLTAGE_LIMIT_V = SUPPLY_VOLTAGE_V;
static constexpr float HARD_MAX_RPM = 3000.0f;
static constexpr float HARD_MAX_ALIGN_VOLTAGE_V = 1.2f;

// STOP releases the driver; physical coast-down is observed before restarting.

// Normal speed changes, STOP and reversal use a monotonic quintic S-curve.
// cfg.accelRpmPerSec is kept internally for NVS compatibility; the GUI exposes
// the equivalent full 0->100% ramp time because it is easier to tune.
static constexpr uint32_t SCURVE_MIN_DURATION_US = 250000;  // 250 ms for tiny changes
static constexpr uint32_t SCURVE_MAX_DURATION_US = 8000000; // sanity ceiling
static constexpr float SCURVE_ZERO_EPS_RAD_S = 0.05f;
static constexpr uint32_t OPEN_REVERSE_PAUSE_MS = 350;

// OPEN + foc_current has a subtle property in SimpleFOC: velocityOpenloop()
// returns current_limit even at 0 rad/s. Therefore cutting EN exactly at zero
// can release a still-energised stator field and make an audible click. Fade the
// current setpoint smoothly before disabling, and fade it in after enabling.
static constexpr uint32_t OPEN_CURRENT_FADE_IN_US = 180000;
static constexpr uint32_t OPEN_CURRENT_FADE_OUT_US = 140000;

// GUI commands are received by AsyncTCP/WebServer on an asynchronous network
// task.  Do not retarget the motor in the same burst that parses and ACKs the
// HTTP request.  Keep only the newest GUI command and apply it after this short
// network-quiet window.  Serial commands remain immediate.
static constexpr uint32_t GUI_COMMAND_QUIET_MS = 80;


// Independent stall / overload watchdog.
// HALL mode uses Hall motion for stall/overload supervision. OPEN mode is
// deliberately Hall-independent: no Hall init, no Hall telemetry, no Hall watchdog.
static constexpr bool USE_HALL_WATCHDOG_IN_OPEN_LOOP = false;
static constexpr float STALL_MIN_COMMAND_RPM = 35.0f;
static constexpr uint32_t STALL_STARTUP_GRACE_MS = 1800;
static constexpr uint32_t STALL_NO_HALL_TIMEOUT_MS = 800;
static constexpr float OVERLOAD_MIN_SPEED_RATIO = 0.35f;
static constexpr float OVERLOAD_CURRENT_RATIO = 0.75f;
static constexpr uint32_t OVERLOAD_HOLD_MS = 1500;
static constexpr uint32_t SAFETY_CHECK_PERIOD_MS = 20;

// Commissioning compatibility with the known-good v0.3.7 behavior.
// Keep the diagnostic code compiled in, but do not let experimental
// WINDMILL / STALL / OVERLOAD / LOOP_GAP / software over-current trips
// block OPEN_CURRENT_FOC while Hall is not connected.
static constexpr bool ENABLE_EXPERIMENTAL_MOTION_SAFETY = false;

// ============================================================================
// 3. WI-FI
// ============================================================================

static const char* AP_SSID = "MaxxFan-Setup";
static const char* AP_PASSWORD = "MaxxFan123";   // >= 8 characters

static constexpr const char* FIRMWARE_VERSION = "0.3.6.3-DECOUPLED-GUI-NOHALL";

// AP is always enabled, so 192.168.4.1 remains a recovery path.
// Optional home Wi-Fi credentials are entered in the GUI and stored in NVS.

// ============================================================================
// 4. CONTROL CONFIGURATION
// ============================================================================

enum ControlMode : uint8_t {
  OPEN_CURRENT_FOC = 0,
  HALL_CURRENT_FOC = 1
};

static constexpr uint32_t CONFIG_VERSION = 6;

struct Config {
  uint32_t version;
  uint8_t mode;

  float maxRpm;
  float minRpm;
  float accelRpmPerSec;

  float currentLimitA;
  float motorVoltageLimitV;
  float alignVoltageV;

  float currentP;
  float currentI;
  float currentTf;

  float velocityP;
  float velocityI;
  float velocityTf;

  bool defaultReverse;

  bool hallCalValid;
  float hallZeroElectric;
  int8_t hallDirection; // +1=CW, -1=CCW
};

Config makeDefaultConfig() {
  Config c{};
  c.version = CONFIG_VERSION;
  c.mode = OPEN_CURRENT_FOC;

  // Conservative first-test limits. OPEN is the safe/default commissioning mode.
  // Hall is enabled only after explicitly selecting HALL_CURRENT_FOC. 
  c.maxRpm = 600.0f;
  c.minRpm = 120.0f;
  c.accelRpmPerSec = 350.0f;

  c.currentLimitA = 0.60f;
  c.motorVoltageLimitV = 3.0f;
  c.alignVoltageV = 0.60f;

  // Current PI: inherited commissioning seeds, not validated tuning.
  // These are intentionally below the aggressive MKS P=1/I=500 example.
  c.currentP = 0.40f;
  c.currentI = 180.0f;
  c.currentTf = 0.002f;

  // Hall velocity PI in current-output units.
  c.velocityP = 0.008f;
  c.velocityI = 0.080f;
  c.velocityTf = 0.030f;

  c.defaultReverse = false;

  c.hallCalValid = false;
  c.hallZeroElectric = 0.0f;
  c.hallDirection = 1;
  return c;
}

static Config cfg;
static Config publishedConfig;

// ============================================================================
// 5. SIMPLEFOC OBJECTS
// ============================================================================

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_ENABLE);
InlineCurrentSense currentSense = InlineCurrentSense(
  CURRENT_SHUNT_OHM,
  CURRENT_AMP_GAIN,
  PIN_CURRENT_A,
  PIN_CURRENT_B
);

HallSensor hall = HallSensor(PIN_HALL_A, PIN_HALL_B, PIN_HALL_C, MOTOR_POLE_PAIRS);

volatile uint32_t hallTransitions = 0;
portMUX_TYPE hallMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint8_t lastHallState = 0;
volatile uint32_t lastHallEdgeUs = 0;

bool validHallStep(uint8_t previous, uint8_t next) {
  const uint8_t changed = previous ^ next;
  return previous > 0 && previous < 7 && next > 0 && next < 7 &&
         (changed == 1 || changed == 2 || changed == 4);
}
uint8_t readHallState() {
  return (digitalRead(PIN_HALL_A) << 2) |
         (digitalRead(PIN_HALL_B) << 1) | digitalRead(PIN_HALL_C);
}
void recordHallEdge() {
  const uint8_t next = readHallState();
  portENTER_CRITICAL_ISR(&hallMux);
  if (validHallStep(lastHallState, next)) ++hallTransitions;
  lastHallState = next;
  lastHallEdgeUs = micros(); // any activity delays restart, including noise
  portEXIT_CRITICAL_ISR(&hallMux);
}
void doHallA() { hall.handleA(); recordHallEdge(); }
void doHallB() { hall.handleB(); recordHallEdge(); }
void doHallC() { hall.handleC(); recordHallEdge(); }
uint32_t hallQuietUs() {
  portENTER_CRITICAL(&hallMux);
  const uint32_t edge = lastHallEdgeUs;
  portEXIT_CRITICAL(&hallMux);
  return micros() - edge;
}

// ============================================================================
// 6. NVS / WEB / THREAD-SAFE STATE
// ============================================================================

Preferences prefs;
AsyncWebServer server(80);

portMUX_TYPE stateMux = portMUX_INITIALIZER_UNLOCKED;

struct Telemetry {
  bool motorReady;
  bool enabled;
  bool reversing;
  uint8_t mode;
  float requestedPercent;
  int8_t requestedDirection;
  float commandedRpm;
  float measuredRpm;
  bool measuredRpmValid;
  float iqA;
  float idA;
  float uqV;
  float udV;
  float currentLimitA;
  float voltageLimitV;
  uint32_t hallTransitions;
  float hallRpm;
  bool hallWatchdogEnabled;
  uint32_t hallQuietMs;
  uint32_t loopGapUs;
  bool sCurveActive;
  bool faultLatched;
  bool retryPending;
  uint8_t retryAttempts;
  uint32_t retryRemainingMs;
  int32_t wifiRssi;
  bool staConnected;
  char staIp[20];
  char apIp[20];
  char staSsid[33];
  char fault[96];
};

Telemetry telemetry{};

// Explicit prototypes are intentional.
// Arduino's .ino preprocessor can otherwise generate a prototype for a
// custom return type before the struct declaration and fail with:
//   'Telemetry' does not name a type
Telemetry telemetrySnapshot();
Config configSnapshot();

struct PendingControl {
  bool valid;
  float percent;
  int8_t direction;
  uint32_t receivedAtMs;
};
PendingControl pendingControl{};

struct PendingSettings {
  bool valid;
  Config value;
};
PendingSettings pendingSettings{};

struct PendingWifi {
  bool valid;
  char ssid[33];
  char password[65];
};
PendingWifi pendingWifi{};

volatile bool pendingStop = false;
volatile bool pendingClearFault = false;
volatile bool pendingRecalibrate = false;
volatile bool pendingDefaults = false;
volatile bool pendingRestart = false;

// ============================================================================
// 7. MOTOR RUNTIME STATE
// ============================================================================

bool motorReady = false;
bool motorEnabled = false;
bool reversalPending = false;
bool reversePauseActive = false;

float requestedPercent = 0.0f;
int8_t requestedDirection = 1;
int8_t driveDirection = 1;
float rampedRadPerSec = 0.0f;

struct SCurveState {
  bool active = false;
  float target = 0.0f;
  uint32_t startedUs = 0;
  uint32_t durationUs = 0;
  // omega(tau) = c0 + c1*tau + ... + c5*tau^5, tau in [0,1].
  // Every retarget starts from the CURRENT commanded speed and uses the classic
  // 6x^5-15x^4+10x^3 smootherstep shape. This guarantees a monotonic command
  // and avoids the old mid-ramp reversal overshoot caused by preserving an
  // acceleration that was pointing away from the new target.
  float c0 = 0.0f, c1 = 0.0f, c2 = 0.0f, c3 = 0.0f, c4 = 0.0f, c5 = 0.0f;
};
SCurveState speedCurve;

float openCurrentScale = 0.0f;
bool openCurrentFadeIn = false;
bool openCurrentFadeOut = false;
uint32_t openCurrentFadeStartedUs = 0;
float openCurrentFadeStartScale = 0.0f;

uint32_t loopGapWindowMaxUs = 0;

uint32_t controlTimestampUs = 0;
uint32_t startWaitingAtMs = 0;
uint32_t overCurrentAtMs = 0;
uint32_t invalidHallAtMs = 0;
static constexpr uint32_t START_QUIET_US = 500000;
static constexpr uint32_t START_WAIT_LIMIT_MS = 10000;
static constexpr uint32_t LOOP_GAP_LIMIT_US = 10000;
static constexpr uint32_t OVERCURRENT_HOLD_MS = 20;

// Safety watchdog runtime state. Faults are latched: once a stall/overload is
// detected the driver is disabled immediately. Bounded recovery retries
// STALL/OVERLOAD; all other faults require explicit clear.
bool safetyFaultLatched = false;
RetryPolicy retryPolicy;
bool hallWatchdogEnabled = false;
uint32_t motorEnabledAtMs = 0;
uint32_t lastHallCount = 0;
uint32_t lastHallActivityMs = 0;
uint32_t overloadStartedMs = 0;
uint32_t lastSafetyCheckMs = 0;
float safetyHallRpm = 0.0f;

char faultText[96] = "";

uint32_t scheduledRestartAtMs = 0;

// ============================================================================
// 8. HELPERS
// ============================================================================

static inline float rpmToRad(float rpm) {
  return rpm * (_2PI / 60.0f);
}

static inline float radToRpm(float rad_s) {
  return rad_s * (60.0f / _2PI);
}

static inline float clampf(float x, float lo, float hi) {
  if (!isfinite(x)) return lo;
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setFault(const char* msg) {
  strncpy(faultText, msg ? msg : "", sizeof(faultText) - 1);
  faultText[sizeof(faultText) - 1] = '\0';
  Serial.print("FAULT: ");
  Serial.println(faultText);
}

void clearFault() {
  faultText[0] = '\0';
}

void validateConfig(Config& c) {
  c.version = CONFIG_VERSION;
  c.mode = (c.mode == HALL_CURRENT_FOC) ? HALL_CURRENT_FOC : OPEN_CURRENT_FOC;

  c.maxRpm = clampf(c.maxRpm, 100.0f, HARD_MAX_RPM);
  c.minRpm = clampf(c.minRpm, 50.0f, c.maxRpm);
  c.accelRpmPerSec = clampf(c.accelRpmPerSec, 50.0f, 5000.0f);

  c.currentLimitA = clampf(c.currentLimitA, 0.10f, HARD_MAX_CURRENT_A);
  c.motorVoltageLimitV = clampf(c.motorVoltageLimitV, 0.50f, HARD_MAX_MOTOR_VOLTAGE_V);
  c.alignVoltageV = clampf(c.alignVoltageV, 0.20f, HARD_MAX_ALIGN_VOLTAGE_V);
  if (c.alignVoltageV > c.motorVoltageLimitV) c.alignVoltageV = c.motorVoltageLimitV;

  c.currentP = clampf(c.currentP, 0.01f, 2.0f);
  c.currentI = clampf(c.currentI, 1.0f, 1000.0f);
  c.currentTf = clampf(c.currentTf, 0.0005f, 0.020f);

  c.velocityP = clampf(c.velocityP, 0.0001f, 0.100f);
  c.velocityI = clampf(c.velocityI, 0.0f, 2.0f);
  c.velocityTf = clampf(c.velocityTf, 0.001f, 0.100f);

  if (!isfinite(c.hallZeroElectric) || c.hallZeroElectric < 0.0f ||
      c.hallZeroElectric >= _2PI ||
      (c.hallDirection != -1 && c.hallDirection != 1)) {
    c.hallCalValid = false;
    c.hallZeroElectric = 0.0f;
    c.hallDirection = 1;
  }
}

String jsonEscape(const char* s) {
  String out;
  if (!s) return out;
  while (*s) {
    char ch = *s++;
    switch (ch) {
      case '\\': out += "\\\\"; break;
      case '"': out += "\\\""; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)ch >= 0x20) out += ch;
        break;
    }
  }
  return out;
}

static inline float smootherstep01(float x) {
  x = clampf(x, 0.0f, 1.0f);
  return x*x*x * (x * (x * 6.0f - 15.0f) + 10.0f);
}

uint32_t sCurveDurationUs(float fromRad, float toRad) {
  const float deltaRpm = fabsf(radToRpm(toRad - fromRad));
  if (deltaRpm < 0.01f) return 0;
  const float seconds = deltaRpm / max(cfg.accelRpmPerSec, 1.0f);
  return (uint32_t)clampf(seconds * 1000000.0f,
                          (float)SCURVE_MIN_DURATION_US,
                          (float)SCURVE_MAX_DURATION_US);
}

void resetSCurve(float value = 0.0f) {
  speedCurve.active = false;
  speedCurve.target = value;
  speedCurve.startedUs = micros();
  speedCurve.durationUs = 0;
  speedCurve.c0 = value;
  speedCurve.c1 = speedCurve.c2 = speedCurve.c3 = 0.0f;
  speedCurve.c4 = speedCurve.c5 = 0.0f;
  rampedRadPerSec = value;
}

void evaluateSCurve(uint32_t nowUs, float& omega, float& acceleration, float& jerk) {
  if (!speedCurve.active || speedCurve.durationUs == 0) {
    omega = speedCurve.target;
    acceleration = 0.0f;
    jerk = 0.0f;
    return;
  }

  const uint32_t elapsed = nowUs - speedCurve.startedUs;
  if (elapsed >= speedCurve.durationUs) {
    omega = speedCurve.target;
    acceleration = 0.0f;
    jerk = 0.0f;
    return;
  }

  const float x = (float)elapsed / (float)speedCurve.durationUs;
  const float x2 = x * x;
  const float x3 = x2 * x;
  const float x4 = x3 * x;
  const float x5 = x4 * x;

  omega = speedCurve.c0 + speedCurve.c1*x + speedCurve.c2*x2 +
          speedCurve.c3*x3 + speedCurve.c4*x4 + speedCurve.c5*x5;

  const float dTau = speedCurve.c1 + 2.0f*speedCurve.c2*x +
                     3.0f*speedCurve.c3*x2 + 4.0f*speedCurve.c4*x3 +
                     5.0f*speedCurve.c5*x4;
  const float d2Tau = 2.0f*speedCurve.c2 + 6.0f*speedCurve.c3*x +
                      12.0f*speedCurve.c4*x2 + 20.0f*speedCurve.c5*x3;
  const float T = speedCurve.durationUs * 1e-6f;
  acceleration = dTau / T;
  jerk = d2Tau / (T * T);
}

void retargetSCurve(float target, uint32_t nowUs) {
  if (!isfinite(target)) target = 0.0f;
  if (fabsf(target - speedCurve.target) < 0.001f) return;

  float omega = rampedRadPerSec;
  if (speedCurve.active) {
    float acceleration = 0.0f, jerk = 0.0f;
    evaluateSCurve(nowUs, omega, acceleration, jerk);
    rampedRadPerSec = omega;
  }

  const uint32_t durationUs = sCurveDurationUs(omega, target);
  if (!durationUs) {
    resetSCurve(target);
    return;
  }

  // Monotonic quintic smootherstep from the CURRENT speed to the new target.
  // We intentionally do not preserve acceleration across an asynchronous
  // retarget. Preserving a positive acceleration while asking for zero/reverse
  // mathematically forces the trajectory to keep accelerating the wrong way
  // before braking. Speed itself remains perfectly continuous here.
  const float d = target - omega;
  speedCurve.c0 = omega;
  speedCurve.c1 = 0.0f;
  speedCurve.c2 = 0.0f;
  speedCurve.c3 = 10.0f * d;
  speedCurve.c4 = -15.0f * d;
  speedCurve.c5 = 6.0f * d;
  speedCurve.target = target;
  speedCurve.startedUs = nowUs;
  speedCurve.durationUs = durationUs;
  speedCurve.active = true;
}

void updateSCurve(uint32_t nowUs) {
  if (!speedCurve.active) {
    rampedRadPerSec = speedCurve.target;
    return;
  }

  const uint32_t elapsed = nowUs - speedCurve.startedUs;
  if (elapsed >= speedCurve.durationUs) {
    rampedRadPerSec = speedCurve.target;
    speedCurve.active = false;
    return;
  }

  float acceleration, jerk;
  evaluateSCurve(nowUs, rampedRadPerSec, acceleration, jerk);
}

void startOpenCurrentFadeIn(uint32_t nowUs, float fromScale = 0.0f) {
  if (cfg.mode != OPEN_CURRENT_FOC) return;
  openCurrentScale = clampf(fromScale, 0.0f, 1.0f);
  openCurrentFadeStartScale = openCurrentScale;
  openCurrentFadeStartedUs = nowUs;
  openCurrentFadeIn = true;
  openCurrentFadeOut = false;
}

void startOpenCurrentFadeOut(uint32_t nowUs) {
  if (cfg.mode != OPEN_CURRENT_FOC || openCurrentFadeOut) return;
  openCurrentFadeStartScale = clampf(openCurrentScale, 0.0f, 1.0f);
  openCurrentFadeStartedUs = nowUs;
  openCurrentFadeIn = false;
  openCurrentFadeOut = true;
}

// Returns true once a requested fade-out has reached zero current.
bool updateOpenCurrentEnvelope(uint32_t nowUs) {
  if (cfg.mode != OPEN_CURRENT_FOC) {
    openCurrentScale = 1.0f;
    openCurrentFadeIn = openCurrentFadeOut = false;
    motor.current_limit = cfg.currentLimitA;
    return true;
  }

  if (openCurrentFadeOut) {
    const uint32_t elapsed = nowUs - openCurrentFadeStartedUs;
    const float x = OPEN_CURRENT_FADE_OUT_US ?
      clampf((float)elapsed / (float)OPEN_CURRENT_FADE_OUT_US, 0.0f, 1.0f) : 1.0f;
    openCurrentScale = openCurrentFadeStartScale * (1.0f - smootherstep01(x));
    if (x >= 1.0f) {
      openCurrentScale = 0.0f;
      openCurrentFadeOut = false;
    }
  } else if (openCurrentFadeIn) {
    const uint32_t elapsed = nowUs - openCurrentFadeStartedUs;
    const float x = OPEN_CURRENT_FADE_IN_US ?
      clampf((float)elapsed / (float)OPEN_CURRENT_FADE_IN_US, 0.0f, 1.0f) : 1.0f;
    openCurrentScale = openCurrentFadeStartScale +
      (1.0f - openCurrentFadeStartScale) * smootherstep01(x);
    if (x >= 1.0f) {
      openCurrentScale = 1.0f;
      openCurrentFadeIn = false;
    }
  }

  motor.current_limit = cfg.currentLimitA * clampf(openCurrentScale, 0.0f, 1.0f);
  return !openCurrentFadeOut && openCurrentScale <= 0.0005f;
}

void resetControlLoops() {
  motor.PID_velocity.reset();
  motor.PID_current_q.reset();
  motor.PID_current_d.reset();
}

void disableMotorNow() {
  if (motorEnabled) motor.disable();
  digitalWrite(PIN_ENABLE, LOW);
  motorEnabled = false;
  overloadStartedMs = 0;
  openCurrentScale = 0.0f;
  openCurrentFadeIn = false;
  openCurrentFadeOut = false;
  // Keep the configured limit as the resting configuration. enableMotorNow()
  // will start OPEN mode from zero and fade it in before torque is applied.
  motor.current_limit = cfg.currentLimitA;
  resetControlLoops();
}

void resetSafetyWatchdogForStart() {
  const uint32_t now = millis();
  motorEnabledAtMs = now;
  lastHallCount = hallTransitions;
  lastHallActivityMs = now;
  overloadStartedMs = 0;
  lastSafetyCheckMs = 0;
  safetyHallRpm = 0.0f;

  // OPEN mode is deliberately Hall-independent. Hall is initialized and read
  // only in HALL_CURRENT_FOC.
  hallWatchdogEnabled = ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
                        (cfg.mode == HALL_CURRENT_FOC);
}

void enableMotorNow() {
  if (!motorReady || motorEnabled || safetyFaultLatched) return;
  resetControlLoops();
  resetSafetyWatchdogForStart();
  motor.current_sp = 0.0f;
  motor.target = 0.0f;
  if (cfg.mode == OPEN_CURRENT_FOC) {
    openCurrentScale = 0.0f;
    motor.current_limit = 0.0f;
    startOpenCurrentFadeIn(micros(), 0.0f);
  } else {
    openCurrentScale = 1.0f;
    motor.current_limit = cfg.currentLimitA;
    openCurrentFadeIn = openCurrentFadeOut = false;
  }
  motor.enable();
  motorEnabled = true;
}

void tripSafetyFault(const char* code, const char* reason) {
  if (!ENABLE_EXPERIMENTAL_MOTION_SAFETY) return;
  if (safetyFaultLatched) return;

  const float cmdRpm = fabsf(radToRpm(rampedRadPerSec));
  const float hallRpm = fabsf(safetyHallRpm);
  const float iq = motorEnabled ? fabsf(motor.current.q) : 0.0f;
  const uint32_t quietMs = millis() - lastHallActivityMs;

  const bool retryScheduled = retryPolicy.schedule(code, requestedPercent,
    requestedDirection, hallWatchdogEnabled, millis());
  safetyFaultLatched = true;
  requestedPercent = 0.0f;
  resetSCurve(0.0f);
  motor.target = 0.0f;
  disableMotorNow();

  reversalPending = false;
  reversePauseActive = false;
  startWaitingAtMs = 0;
  prefs.putBool("safetytrip", true); // power cycling does not clear a fault
  snprintf(faultText, sizeof(faultText), "%s: %s", code, reason);

  // One clear event, no Serial flooding.
  Serial.println();
  Serial.println("================ SAFETY FAULT ================");
  Serial.print("Code: "); Serial.println(code);
  Serial.print("Reason: "); Serial.println(reason);
  Serial.printf("Commanded: %.0f rpm | Hall: %.0f rpm | Iq: %.3f A\n",
                cmdRpm, hallRpm, iq);
  Serial.printf("No Hall transition: %lu ms | Hall transitions: %lu\n",
                (unsigned long)quietMs, (unsigned long)hallTransitions);
  if (retryScheduled) {
    Serial.printf("Driver OFF. Automatic retry %u/%u in 10 seconds; STOP cancels.\n",
      retryPolicy.attempts + 1, RetryPolicy::maxAttempts);
  } else {
    Serial.println("Driver OFF. Manual clear required (hard fault or retries exhausted).");
  }
  Serial.println("Manual clear stops retries: GUI Clear fault / Serial clear");
  Serial.println("==============================================");
}

void clearSafetyFault() {
  retryPolicy.reset();
  if (!safetyFaultLatched) return;
  requestedPercent = 0.0f;
  resetSCurve(0.0f);
  disableMotorNow();
  if (prefs.putBool("safetytrip", false) == 0) {
    setFault("NVS: cannot clear persisted fault");
    return;
  }
  safetyFaultLatched = false;
  reversalPending = reversePauseActive = false;
  startWaitingAtMs = 0;
  hallWatchdogEnabled = false;
  overloadStartedMs = 0;
  clearFault();
  Serial.println("[SAFETY] Fault cleared. Motor remains STOPPED.");
  if (!motorReady) {
    // Explicit clear after a persisted fault permits initialization on reboot.
    scheduledRestartAtMs = millis() + 600;
  }
}

void serviceAutoRecovery() {
  if (!ENABLE_EXPERIMENTAL_MOTION_SAFETY) {
    retryPolicy.cancel();
    return;
  }
  if (cfg.mode != HALL_CURRENT_FOC) {
    retryPolicy.cancel();
    return;
  }

  const uint32_t now = millis();
  const float cmd = fabsf(radToRpm(rampedRadPerSec));
  retryPolicy.observeHealthy(motorEnabled && !safetyFaultLatched &&
    !reversalPending && cmd >= STALL_MIN_COMMAND_RPM &&
    safetyHallRpm >= 0.70f * cmd, now);
  if (!retryPolicy.due(now)) return;
  if (!motorReady || scheduledRestartAtMs || !safetyFaultLatched) {
    retryPolicy.cancel();
    return;
  }
  // Driver is already off. Never clear persistent trip while PWM is active.
  disableMotorNow();
  if (prefs.putBool("safetytrip", false) == 0) {
    retryPolicy.cancel();
    setFault("NVS: retry cancelled; cannot clear stored trip");
    return;
  }
  const float resumePercent = retryPolicy.percent;
  const int8_t resumeDirection = retryPolicy.direction;
  retryPolicy.launched();
  safetyFaultLatched = false;
  clearFault();
  resetSCurve(0.0f);
  motor.target = motor.current_sp = 0.0f;
  startWaitingAtMs = 0;
  reversalPending = reversePauseActive = false;
  requestedPercent = resumePercent;
  requestedDirection = resumeDirection;
  Serial.printf("[RECOVERY] Retry %u/%u requested; wait for Hall quiet, then ramp.\n",
    retryPolicy.attempts, RetryPolicy::maxAttempts);
}

// ============================================================================
// 9. PREFERENCES
// ============================================================================

void saveConfig() {
  // One NVS blob replaces the multi-key partially-written configuration.
  if (prefs.putBytes("config6", &cfg, sizeof(cfg)) != sizeof(cfg)) {
    safetyFaultLatched = true;
    setFault("NVS: config save failed; restart inhibited until checked");
  }
}

void loadConfig() {
  // NVS migration strategy:
  // - NEVER discard existing settings just because CONFIG_VERSION changed.
  // - Existing keys are loaded as-is.
  // - Any newly introduced/missing key automatically receives its current
  //   default value.
  // - After validation, the config is written back with the current version.
  //
  // This means a normal firmware update preserves tuning, Hall calibration,
  // mode and limits. Wi-Fi credentials use separate NVS keys and are also kept.
  if (prefs.getBytesLength("config6") == sizeof(Config)) {
    Config saved{};
    if (prefs.getBytes("config6", &saved, sizeof(saved)) == sizeof(saved) &&
        saved.version == CONFIG_VERSION) {
      cfg = saved;
      validateConfig(cfg);
      return;
    }
  }
  const Config d = makeDefaultConfig();
  const uint32_t storedVersion = prefs.getUInt("cfgver", 0);

  cfg.version = CONFIG_VERSION;
  cfg.mode = prefs.getUChar("mode", d.mode);
  cfg.maxRpm = prefs.getFloat("maxrpm", d.maxRpm);
  cfg.minRpm = prefs.getFloat("minrpm", d.minRpm);
  cfg.accelRpmPerSec = prefs.getFloat("accel", d.accelRpmPerSec);
  cfg.currentLimitA = prefs.getFloat("curlim", d.currentLimitA);
  cfg.motorVoltageLimitV = prefs.getFloat("vlim", d.motorVoltageLimitV);
  cfg.alignVoltageV = prefs.getFloat("alignv", d.alignVoltageV);
  cfg.currentP = prefs.getFloat("curp", d.currentP);
  cfg.currentI = prefs.getFloat("curi", d.currentI);
  cfg.currentTf = prefs.getFloat("curtf", d.currentTf);
  cfg.velocityP = prefs.getFloat("velp", d.velocityP);
  cfg.velocityI = prefs.getFloat("veli", d.velocityI);
  cfg.velocityTf = prefs.getFloat("veltf", d.velocityTf);
  cfg.defaultReverse = prefs.getBool("defrev", d.defaultReverse);
  cfg.hallCalValid = prefs.getBool("hvalid", d.hallCalValid);
  cfg.hallZeroElectric = prefs.getFloat("hzero", d.hallZeroElectric);
  cfg.hallDirection = prefs.getChar("hdir", d.hallDirection);

  validateConfig(cfg);

  if (storedVersion != CONFIG_VERSION) {
    Serial.print("NVS config migration: ");
    Serial.print(storedVersion);
    Serial.print(" -> ");
    Serial.println(CONFIG_VERSION);
    saveConfig();
  }
}

// ============================================================================
// 10. MOTOR SETUP
// ============================================================================

void applyRuntimeConfig() {
  validateConfig(cfg);

  motor.updateCurrentLimit(cfg.currentLimitA);
  motor.updateVoltageLimit(cfg.motorVoltageLimitV);
  motor.updateVelocityLimit(rpmToRad(cfg.maxRpm));

  motor.PID_current_q.P = cfg.currentP;
  motor.PID_current_q.I = cfg.currentI;
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = cfg.currentP;
  motor.PID_current_d.I = cfg.currentI;
  motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = cfg.currentTf;
  motor.LPF_current_d.Tf = cfg.currentTf;

  motor.PID_velocity.P = cfg.velocityP;
  motor.PID_velocity.I = cfg.velocityI;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = cfg.velocityTf;

  // Do not touch Hall runtime state in OPEN commissioning mode.
  if (cfg.mode == HALL_CURRENT_FOC) {
    hall.velocity_max = max(rpmToRad(cfg.maxRpm) * 1.7f, 150.0f);
  }

  resetControlLoops();
}

bool setupMotor() {
  clearFault();
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY && prefs.getBool("safetytrip", false)) {
    setFault("PREVIOUS_FAULT: inspect hardware, clear to reinitialize");
    return false;
  }

  // Exact pre-driver state used by the known-good no-Hall v0.3.7 path.
  // The bridge remains disabled through PIN_ENABLE, while all PWM inputs are
  // held LOW until SimpleFOC configures the PWM hardware.
  pinMode(PIN_PWM_U, OUTPUT);
  pinMode(PIN_PWM_V, OUTPUT);
  pinMode(PIN_PWM_W, OUTPUT);
  digitalWrite(PIN_PWM_U, LOW);
  digitalWrite(PIN_PWM_V, LOW);
  digitalWrite(PIN_PWM_W, LOW);

  // Motor parameters used by SimpleFOC 2.4 current-control compensation.
  // Do not enable model compensation with unverified phase/line parameters.
  // Current PI runs directly on measured phase currents.


  // Explicitly initialise v2.4 feed-forward state. Global objects are normally
  // zero-initialised already, but doing it here removes any ambiguity.
  motor.feed_forward_current.q = 0.0f;
  motor.feed_forward_current.d = 0.0f;
  motor.feed_forward_voltage.q = 0.0f;
  motor.feed_forward_voltage.d = 0.0f;
  motor.feed_forward_velocity = 0.0f;

  // Hall is intentionally absent from OPEN_CURRENT_FOC: no init, interrupts,
  // startup gate, telemetry or watchdog dependency. This makes OPEN mode a real
  // Hall-free test mode. HALL mode keeps the full sensor path.
  if (cfg.mode == HALL_CURRENT_FOC) {
    hall.pullup = Pullup::USE_EXTERN;
    hall.velocity_max = max(rpmToRad(cfg.maxRpm) * 1.7f, 150.0f);
    hall.init();
    lastHallState = readHallState();
    lastHallEdgeUs = micros();
    hall.enableInterrupts(doHallA, doHallB, doHallC);
    motor.linkSensor(&hall);

    // Keep the Hall safety gate available for later qualification, but do not
    // let it participate during current no-Hall commissioning.
    if (ENABLE_EXPERIMENTAL_MOTION_SAFETY) {
      const uint32_t waitStart = millis();
      while (hallQuietUs() < START_QUIET_US) {
        if (millis() - waitStart >= START_WAIT_LIMIT_MS) {
          setFault("WINDMILL: cannot initialize Hall FOC while rotor is moving");
          return false;
        }
        delay(1);
      }
      const uint8_t initialHall = readHallState();
      if (initialHall == 0 || initialHall == 7) {
        setFault("HALL_INVALID: check Hall wiring before Hall FOC initialization");
        return false;
      }
    }
  } else {
    hallWatchdogEnabled = false;
    hallTransitions = 0;
    safetyHallRpm = 0.0f;
    Serial.println("OPEN_CURRENT_FOC: Hall hardware is NOT used.");
  }
  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = SUPPLY_VOLTAGE_V;
  driver.voltage_limit = HARD_DRIVER_VOLTAGE_LIMIT_V;

  if (!driver.init()) {
    setFault("Driver init failed");
    return false;
  }
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.controller = (cfg.mode == HALL_CURRENT_FOC)
    ? MotionControlType::velocity
    : MotionControlType::velocity_openloop;

  motor.current_limit = cfg.currentLimitA;
  motor.voltage_limit = cfg.motorVoltageLimitV;
  motor.velocity_limit = rpmToRad(cfg.maxRpm);
  motor.voltage_sensor_align = cfg.alignVoltageV;

  motor.PID_current_q.P = cfg.currentP;
  motor.PID_current_q.I = cfg.currentI;
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = cfg.currentP;
  motor.PID_current_d.I = cfg.currentI;
  motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = cfg.currentTf;
  motor.LPF_current_d.Tf = cfg.currentTf;

  motor.PID_velocity.P = cfg.velocityP;
  motor.PID_velocity.I = cfg.velocityI;
  motor.PID_velocity.D = 0.0f;
  motor.LPF_velocity.Tf = cfg.velocityTf;

  if (!motor.init()) {
    motor.disable();
    setFault("Motor init failed");
    return false;
  }

  // Must be linked to the initialized driver before current-sense init.
  currentSense.linkDriver(&driver);

  if (!currentSense.init()) {
    setFault("Current sense init failed");
    motor.disable();
    return false;
  }

  // Check current phase order/sign at initialization; offsets calibrate every boot.
  currentSense.skip_align = SKIP_CURRENT_SENSE_ALIGNMENT;
  motor.linkCurrentSense(&currentSense);

  bool usedSavedHallCalibration = false;
  if (cfg.mode == HALL_CURRENT_FOC && cfg.hallCalValid) {
    motor.zero_electric_angle = cfg.hallZeroElectric;
    motor.sensor_direction = (cfg.hallDirection > 0) ? Direction::CW : Direction::CCW;
    usedSavedHallCalibration = true;
    Serial.println("Using saved Hall FOC calibration.");
  }

  Serial.printf("initFOC: mode=%s, Hall=%s, currentSense=GPIO%d/GPIO%d, align=%.2f V, skipCurrentAlign=%s\n",
                cfg.mode == OPEN_CURRENT_FOC ? "OPEN_CURRENT_FOC" : "HALL_CURRENT_FOC",
                cfg.mode == OPEN_CURRENT_FOC ? "UNUSED" : "ACTIVE",
                PIN_CURRENT_A, PIN_CURRENT_B, cfg.alignVoltageV,
                SKIP_CURRENT_SENSE_ALIGNMENT ? "true" : "false");

  if (!motor.initFOC()) {
    if (cfg.mode == OPEN_CURRENT_FOC)
      setFault("OPEN initFOC failed - current-sense alignment failed (Hall unused)");
    else
      setFault("HALL initFOC failed - check Hall/current-sense alignment");
    motor.disable();
    return false;
  }

  motor.disable();

  if (cfg.mode == HALL_CURRENT_FOC && !usedSavedHallCalibration) {
    cfg.hallCalValid = true;
    cfg.hallZeroElectric = motor.zero_electric_angle;
    cfg.hallDirection = (motor.sensor_direction == Direction::CW) ? 1 : -1;
    saveConfig();

    Serial.print("Saved Hall zero_electric_angle = ");
    Serial.println(cfg.hallZeroElectric, 6);
    Serial.print("Saved Hall direction = ");
    Serial.println(cfg.hallDirection > 0 ? "CW" : "CCW");
  }

  applyRuntimeConfig();

  // NEVER auto-start after boot.
  requestedPercent = 0.0f;
  requestedDirection = cfg.defaultReverse ? -1 : 1;
  driveDirection = requestedDirection;
  resetSCurve(0.0f);
  motor.target = 0.0f;
  motor.disable();
  motorEnabled = false;
  controlTimestampUs = micros();

  Serial.println("Motor initialized and disabled (safe stop).");
  return true;
}

// ============================================================================
// 11. MOTOR CONTROL LOOP + STALL / OVERLOAD SAFETY
// ============================================================================

void runSafetyMonitor() {
  if (!ENABLE_EXPERIMENTAL_MOTION_SAFETY) return;
  if (!motorReady || !motorEnabled || safetyFaultLatched) return;
  if (!hallWatchdogEnabled) return; // OPEN mode has no Hall dependency.

  const uint32_t now = millis();
  if (now - lastSafetyCheckMs < SAFETY_CHECK_PERIOD_MS) return;
  lastSafetyCheckMs = now;

  const uint32_t hc = hallTransitions;
  if (hc != lastHallCount) {
    lastHallCount = hc;
    lastHallActivityMs = now;
  }
  safetyHallRpm = fabsf(radToRpm(hall.getVelocity()));

  // Do not diagnose commanded deceleration/reversal as a stall/overload.
  if (requestedPercent <= 0.01f || reversalPending || reversePauseActive) {
    overloadStartedMs = 0;
    return;
  }

  const uint32_t runMs = now - motorEnabledAtMs;
  if (runMs < STALL_STARTUP_GRACE_MS) {
    overloadStartedMs = 0;
    return;
  }

  const float commandRpm = fabsf(radToRpm(rampedRadPerSec));
  if (commandRpm < STALL_MIN_COMMAND_RPM) {
    overloadStartedMs = 0;
    return;
  }

  const uint32_t quietMs = now - lastHallActivityMs;
  if (quietMs >= STALL_NO_HALL_TIMEOUT_MS) {
    tripSafetyFault("STALL", "no Hall motion while torque is commanded");
    return;
  }

  const float speedRatio = (commandRpm > 1.0f) ? safetyHallRpm / commandRpm : 1.0f;
  const float iq = fabsf(motor.current.q);
  const bool overloaded =
    speedRatio < OVERLOAD_MIN_SPEED_RATIO &&
    iq >= cfg.currentLimitA * OVERLOAD_CURRENT_RATIO;

  if (overloaded) {
    if (overloadStartedMs == 0) overloadStartedMs = now;
    if (now - overloadStartedMs >= OVERLOAD_HOLD_MS) {
      tripSafetyFault("OVERLOAD", "low RPM + high current persisted");
      return;
    }
  } else {
    overloadStartedMs = 0;
  }
}

// Coast before STOP/reverse. Never infer physical standstill from ramped target.
void runMotorControl() {
  if (!motorReady) return;

  if (safetyFaultLatched || scheduledRestartAtMs) {
    requestedPercent = 0.0f;
    resetSCurve(0.0f);
    disableMotorNow();
    return;
  }

  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - controlTimestampUs;
  controlTimestampUs = nowUs;
  if (elapsedUs > loopGapWindowMaxUs) loopGapWindowMaxUs = elapsedUs;
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
      motorEnabled && elapsedUs > LOOP_GAP_LIMIT_US) {
    tripSafetyFault("LOOP_GAP", "motor loop exceeded 10 ms");
    return;
  }

  if (cfg.mode == HALL_CURRENT_FOC) {
    hall.update();
    safetyHallRpm = fabsf(radToRpm(hall.getVelocity()));
  } else {
    safetyHallRpm = 0.0f;
  }

  const bool stopRequested = requestedPercent <= 0.01f;

  // OPEN mode: direction is simply the sign of the velocity target.  SimpleFOC
  // velocity_openloop natively accepts negative velocity.  Keeping the bridge
  // enabled lets the same S-curve that makes ordinary speed changes clean also
  // perform +RPM -> -RPM continuously, without an EN OFF/ON click at zero.
  if (cfg.mode == OPEN_CURRENT_FOC && motorEnabled && !stopRequested &&
      requestedDirection != driveDirection) {
    driveDirection = requestedDirection;
    reversalPending = true;
    reversePauseActive = false;
  }

  // Hall mode keeps the conservative stop/coast/restart reversal sequence.
  bool directionChange = !stopRequested && (requestedDirection != driveDirection);

  // If already disabled, stay stopped or perform the controlled startup gate.
  if (!motorEnabled) {
    resetSCurve(0.0f);
    if (stopRequested) {
      reversalPending = false;
      reversePauseActive = false;
      startWaitingAtMs = 0;
      return;
    }

    if (!startWaitingAtMs) startWaitingAtMs = millis();

    if (cfg.mode == HALL_CURRENT_FOC && ENABLE_EXPERIMENTAL_MOTION_SAFETY) {
      const uint8_t h = readHallState();
      if (h == 0 || h == 7) {
        tripSafetyFault("HALL_INVALID", "Hall state 000/111 before start");
        return;
      }
      if (hallQuietUs() < START_QUIET_US ||
          millis() - startWaitingAtMs < START_QUIET_US / 1000) {
        reversalPending = reversePauseActive;
        if (millis() - startWaitingAtMs >= START_WAIT_LIMIT_MS) {
          tripSafetyFault("WINDMILL", "rotor did not settle within 10 seconds");
        }
        return;
      }
    } else if (reversePauseActive && millis() - startWaitingAtMs < OPEN_REVERSE_PAUSE_MS) {
      reversalPending = true;
      return;
    }

    driveDirection = requestedDirection;
    reversalPending = false;
    reversePauseActive = false;
    startWaitingAtMs = 0;
    overCurrentAtMs = invalidHallAtMs = 0;
    enableMotorNow();
    resetSCurve(0.0f);
    directionChange = false;
  }

  float desiredRadPerSec = 0.0f;
  if (!stopRequested) {
    const float targetRpm = clampf(cfg.maxRpm * requestedPercent / 100.0f,
                                   cfg.minRpm, cfg.maxRpm);
    if (cfg.mode == OPEN_CURRENT_FOC) {
      // Direct signed target: reversal is just another monotonic S-curve.
      desiredRadPerSec = rpmToRad(targetRpm) * requestedDirection;
    } else if (!directionChange) {
      desiredRadPerSec = rpmToRad(targetRpm) * driveDirection;
    } else {
      reversalPending = true;
    }
  }

  retargetSCurve(desiredRadPerSec, nowUs);
  updateSCurve(nowUs);

  // Only STOP and Hall-mode reversal need to actually reach zero and release
  // the bridge.  OPEN-mode reversal crosses zero continuously with EN asserted.
  const bool needsZeroRelease = stopRequested ||
    (cfg.mode == HALL_CURRENT_FOC && directionChange);
  const bool zeroTransitionComplete = needsZeroRelease && !speedCurve.active &&
    fabsf(rampedRadPerSec) <= SCURVE_ZERO_EPS_RAD_S;

  if (!zeroTransitionComplete && openCurrentFadeOut && !stopRequested &&
      !(cfg.mode == HALL_CURRENT_FOC && directionChange)) {
    startOpenCurrentFadeIn(nowUs, openCurrentScale);
  }

  // For a real STOP, fade q-current to zero before disabling the bridge.  OPEN
  // reversal deliberately does NOT enter this path, avoiding the EN transition.
  if (zeroTransitionComplete && cfg.mode == OPEN_CURRENT_FOC && !openCurrentFadeOut &&
      openCurrentScale > 0.0005f) {
    startOpenCurrentFadeOut(nowUs);
  }
  const bool openFadeOutDone = updateOpenCurrentEnvelope(nowUs);

  motor.loopFOC();
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
      (!isfinite(motor.current.q) || !isfinite(motor.current.d) ||
       !isfinite(motor.shaft_velocity))) {
    tripSafetyFault("NUMERIC", "non-finite control feedback");
    return;
  }

  const float magnitude = hypotf(motor.current.q, motor.current.d);
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
      magnitude > cfg.currentLimitA * 1.5f) {
    if (!overCurrentAtMs) overCurrentAtMs = millis();
    if (millis() - overCurrentAtMs >= OVERCURRENT_HOLD_MS) {
      tripSafetyFault("OVERCURRENT", "filtered dq magnitude above 150 percent");
      return;
    }
  } else {
    overCurrentAtMs = 0;
  }

  if (cfg.mode == HALL_CURRENT_FOC && ENABLE_EXPERIMENTAL_MOTION_SAFETY) {
    const uint8_t h = readHallState();
    if (h == 0 || h == 7) {
      if (!invalidHallAtMs) invalidHallAtMs = millis();
      if (millis() - invalidHallAtMs >= 20) {
        tripSafetyFault("HALL_INVALID", "Hall state 000/111 persisted");
        return;
      }
    } else {
      invalidHallAtMs = 0;
    }
  }

  motor.move(rampedRadPerSec);

  // OPEN reversal is finished when the signed target has been reached.  There
  // is no disable, pause or re-enable event at the zero crossing.
  if (cfg.mode == OPEN_CURRENT_FOC && reversalPending && !speedCurve.active &&
      !stopRequested) {
    reversalPending = false;
  }

  if (zeroTransitionComplete) {
    motor.move(0.0f);

    if (cfg.mode == OPEN_CURRENT_FOC && !openFadeOutDone) {
      runSafetyMonitor();
      return;
    }

    disableMotorNow();
    resetSCurve(0.0f);
    if (cfg.mode == HALL_CURRENT_FOC && directionChange) {
      reversePauseActive = true;
      reversalPending = true;
      startWaitingAtMs = millis();
    } else {
      reversalPending = false;
      reversePauseActive = false;
      startWaitingAtMs = 0;
    }
    return;
  }

  runSafetyMonitor();
}

// ============================================================================
// 12. PENDING COMMANDS FROM WEB CALLBACKS
//    All Preferences + motor mutations happen here in the Arduino loop.
// ============================================================================

void scheduleRestart(uint32_t delayMs = 500) {
  scheduledRestartAtMs = millis() + delayMs;
}

void processPendingCommands() {
  bool maintenancePending = false;
  bool stopPending = false;
  bool clearPending = false;

  portENTER_CRITICAL(&stateMux);
  maintenancePending = pendingSettings.valid || pendingWifi.valid ||
                       pendingRecalibrate || pendingDefaults || pendingRestart;
  stopPending = pendingStop;
  clearPending = pendingClearFault;
  portEXIT_CRITICAL(&stateMux);

  // STOP is a normal smooth command. It cancels auto-retry but does NOT hard-cut
  // PWM. Safety faults still call disableMotorNow() immediately.
  if (stopPending) {
    retryPolicy.cancel();
    requestedPercent = 0.0f;
    portENTER_CRITICAL(&stateMux);
    pendingStop = false;
    pendingControl.valid = false;
    portEXIT_CRITICAL(&stateMux);
  }

  if (clearPending) {
    portENTER_CRITICAL(&stateMux);
    pendingClearFault = false;
    portEXIT_CRITICAL(&stateMux);
    clearSafetyFault();
  }

  // Settings, Wi-Fi writes, recalibration, defaults and restart first request a
  // normal S-curve stop. The NVS/radio operation executes only once disabled.
  if (maintenancePending && motorEnabled) {
    retryPolicy.cancel();
    requestedPercent = 0.0f;
    portENTER_CRITICAL(&stateMux);
    pendingControl.valid = false;
    portEXIT_CRITICAL(&stateMux);
    return;
  }

  PendingControl pc{};
  PendingSettings ps{};
  PendingWifi pw{};
  bool doRecal = false;
  bool doDefaults = false;
  bool doRestart = false;

  const uint32_t controlNowMs = millis();
  portENTER_CRITICAL(&stateMux);
  if (!maintenancePending && pendingControl.valid &&
      (uint32_t)(controlNowMs - pendingControl.receivedAtMs) >= GUI_COMMAND_QUIET_MS) {
    pc = pendingControl;
    pendingControl.valid = false;
  }
  if (pendingSettings.valid) { ps = pendingSettings; pendingSettings.valid = false; }
  if (pendingWifi.valid) { pw = pendingWifi; pendingWifi.valid = false; }
  doRecal = pendingRecalibrate; pendingRecalibrate = false;
  doDefaults = pendingDefaults; pendingDefaults = false;
  doRestart = pendingRestart; pendingRestart = false;
  if (maintenancePending) pendingControl.valid = false;
  portEXIT_CRITICAL(&stateMux);

  if (pc.valid && !safetyFaultLatched && !scheduledRestartAtMs) {
    requestedPercent = clampf(pc.percent, 0.0f, 100.0f);
    requestedDirection = (pc.direction < 0) ? -1 : 1;
  }

  if (ps.valid) {
    Config next = ps.value;
    validateConfig(next);
    const bool modeChanged = next.mode != cfg.mode;
    next.hallCalValid = cfg.hallCalValid;
    next.hallZeroElectric = cfg.hallZeroElectric;
    next.hallDirection = cfg.hallDirection;
    portENTER_CRITICAL(&stateMux);
    cfg = next;
    portEXIT_CRITICAL(&stateMux);
    saveConfig();
    if (modeChanged) scheduleRestart(600);
    else if (motorReady) applyRuntimeConfig();
  }

  if (pw.valid) {
    prefs.putString("wssid", pw.ssid);
    prefs.putString("wpass", pw.password);
    scheduleRestart(600);
  }

  if (doRecal) {
    cfg.hallCalValid = false;
    cfg.hallZeroElectric = 0.0f;
    cfg.hallDirection = 1;
    saveConfig();
    scheduleRestart(600);
  }

  if (doDefaults) {
    Config d = makeDefaultConfig();
    portENTER_CRITICAL(&stateMux);
    cfg = d;
    portEXIT_CRITICAL(&stateMux);
    saveConfig();
    scheduleRestart(600);
  }

  if (doRestart) scheduleRestart(400);
}

// ============================================================================
// 13. TELEMETRY
// ============================================================================

void updateTelemetry() {
  static uint32_t lastMs = 0;
  static uint32_t lastWifiMs = 0;
  static bool cachedStaConnected = false;
  static int32_t cachedRssi = 0;
  static char cachedStaIp[20] = "-";
  static char cachedApIp[20] = "192.168.4.1";
  static char cachedSsid[33] = "";

  const uint32_t now = millis();
  if (now - lastMs < 200) return;
  lastMs = now;

  // Wi-Fi metadata is slow-changing. More importantly, do not call WiFi/String
  // helpers while PWM is active: those calls can take locks / allocate heap and
  // create periodic timing jitter. Cache network metadata only while fully stopped.
  const bool motorTrafficSensitive = motorEnabled || speedCurve.active ||
                                     reversalPending || reversePauseActive ||
                                     requestedPercent > 0.01f;
  // While PWM is active there is no periodic GUI polling, so avoid even the
  // telemetry struct copy/critical section.  The accumulated loop-gap maximum
  // remains available once the motor stops.
  if (motorTrafficSensitive) return;
  if (!motorTrafficSensitive && (now - lastWifiMs >= 2000 || lastWifiMs == 0)) {
    lastWifiMs = now;
    cachedStaConnected = WiFi.status() == WL_CONNECTED;
    cachedRssi = cachedStaConnected ? WiFi.RSSI() : 0;
    String staIp = cachedStaConnected ? WiFi.localIP().toString() : "-";
    String apIp = WiFi.softAPIP().toString();
    String ssid = cachedStaConnected ? WiFi.SSID() : "";
    strncpy(cachedStaIp, staIp.c_str(), sizeof(cachedStaIp) - 1);
    cachedStaIp[sizeof(cachedStaIp) - 1] = 0;
    strncpy(cachedApIp, apIp.c_str(), sizeof(cachedApIp) - 1);
    cachedApIp[sizeof(cachedApIp) - 1] = 0;
    strncpy(cachedSsid, ssid.c_str(), sizeof(cachedSsid) - 1);
    cachedSsid[sizeof(cachedSsid) - 1] = 0;
  }

  Telemetry t{};
  t.motorReady = motorReady;
  t.enabled = motorEnabled;
  t.reversing = reversalPending || reversePauseActive;
  t.mode = cfg.mode;
  t.requestedPercent = requestedPercent;
  t.requestedDirection = requestedDirection;
  t.commandedRpm = radToRpm(rampedRadPerSec);
  t.currentLimitA = cfg.currentLimitA;
  t.voltageLimitV = cfg.motorVoltageLimitV;
  t.loopGapUs = loopGapWindowMaxUs;
  loopGapWindowMaxUs = 0;
  t.sCurveActive = speedCurve.active;

  if (cfg.mode == HALL_CURRENT_FOC) {
    t.hallTransitions = hallTransitions;
    t.hallRpm = safetyHallRpm;
    t.hallWatchdogEnabled = hallWatchdogEnabled;
    t.hallQuietMs = motorEnabled ? (millis() - lastHallActivityMs) : 0;
    t.measuredRpm = radToRpm(hall.getVelocity());
    t.measuredRpmValid = motorReady;
  } else {
    t.hallTransitions = 0;
    t.hallRpm = 0.0f;
    t.hallWatchdogEnabled = false;
    t.hallQuietMs = 0;
    t.measuredRpm = 0.0f;
    t.measuredRpmValid = false;
  }

  t.faultLatched = safetyFaultLatched;
  t.retryPending = retryPolicy.pending;
  t.retryAttempts = retryPolicy.attempts;
  t.retryRemainingMs = retryPolicy.remainingMs(now);

  if (motorReady && motorEnabled) {
    t.iqA = motor.current.q;
    t.idA = motor.current.d;
    t.uqV = motor.voltage.q;
    t.udV = motor.voltage.d;
  } else {
    t.iqA = 0.0f;
    t.idA = 0.0f;
    t.uqV = 0.0f;
    t.udV = 0.0f;
  }

  t.staConnected = cachedStaConnected;
  t.wifiRssi = cachedRssi;
  strncpy(t.staIp, cachedStaIp, sizeof(t.staIp) - 1);
  strncpy(t.apIp, cachedApIp, sizeof(t.apIp) - 1);
  strncpy(t.staSsid, cachedSsid, sizeof(t.staSsid) - 1);
  strncpy(t.fault, faultText, sizeof(t.fault) - 1);

  portENTER_CRITICAL(&stateMux);
  telemetry = t;
  publishedConfig = cfg;
  portEXIT_CRITICAL(&stateMux);
}

Telemetry telemetrySnapshot() {
  Telemetry t{};
  portENTER_CRITICAL(&stateMux);
  t = telemetry;
  portEXIT_CRITICAL(&stateMux);
  return t;
}

Config configSnapshot() {
  Config c{};
  portENTER_CRITICAL(&stateMux);
  c = publishedConfig;
  portEXIT_CRITICAL(&stateMux);
  return c;
}

// ============================================================================
// 14. WI-FI
// ============================================================================

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.setSleep(false);
  WiFi.mode(WIFI_AP_STA);

  if (WiFi.softAP(AP_SSID, AP_PASSWORD)) {
    Serial.print("Recovery AP: ");
    Serial.print(AP_SSID);
    Serial.print("  IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("WARNING: failed to start recovery AP");
  }

  String ssid = prefs.getString("wssid", "");
  String pass = prefs.getString("wpass", "");
  if (ssid.length() > 0) {
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.print("Connecting STA to: ");
    Serial.println(ssid);

    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 7000) {
      delay(50);
    }

    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("STA connected. IP: ");
      Serial.println(WiFi.localIP());
    } else {
      Serial.println("STA not connected; recovery AP remains available at 192.168.4.1");
    }
  }

  // Friendly local hostname. 192.168.4.1 remains the guaranteed recovery URL.
  if (MDNS.begin("maxxfan")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS: http://maxxfan.local");
  } else {
    Serial.println("WARNING: mDNS start failed; use the IP address instead.");
  }
}

// ============================================================================
// 15. WEB GUI
// ============================================================================

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>MaxxFan FOC</title>
<style>
:root{color-scheme:dark;--bg:#101318;--card:#1a2028;--line:#303946;--txt:#eef3f7;--muted:#99a8b8;--good:#65d49c;--warn:#ffbf69;--bad:#ff6b6b;--accent:#65a8ff}
*{box-sizing:border-box}body{font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--txt);margin:0;padding:18px}.wrap{max-width:900px;margin:auto}h1{font-size:1.6rem;margin:0 0 4px}.sub{color:var(--muted);margin-bottom:18px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px}.label{font-size:.78rem;color:var(--muted);text-transform:uppercase;letter-spacing:.06em}.value{font-size:1.25rem;margin-top:5px}.section{margin-top:14px}.section h2{font-size:1.05rem;margin:0 0 12px}.row{display:flex;gap:10px;flex-wrap:wrap;align-items:center}.grow{flex:1;min-width:190px}button{border:1px solid var(--line);background:#252d38;color:var(--txt);border-radius:10px;padding:10px 14px;font-weight:650;cursor:pointer}button.primary{background:#245fa8}button.stop{background:#8d3030}button.active{outline:2px solid var(--accent)}input,select{width:100%;background:#10151b;border:1px solid var(--line);color:var(--txt);border-radius:9px;padding:9px}input[type=range]{padding:0}.formgrid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:10px}.field label{display:block;color:var(--muted);font-size:.82rem;margin:0 0 5px}.note{font-size:.82rem;color:var(--muted);line-height:1.45}.msg{min-height:1.3em;color:var(--good);margin-top:8px}.fault{color:var(--bad);font-weight:700}.pill{display:inline-block;padding:3px 8px;border-radius:999px;font-size:.78rem;background:#252d38}.big{font-size:1.35rem;font-weight:700}.split{display:grid;grid-template-columns:1.3fr .7fr;gap:10px}@media(max-width:650px){.split{grid-template-columns:1fr}}
</style>
</head>
<body><div class="wrap">
<h1>MaxxFan BLDC Controller</h1>
<div class="sub">MKS ESP32 FOC Mega · SimpleFOC 2.4 · physical current feedback</div>

<div class="grid">
 <div class="card"><div class="label">Mode</div><div id="modeCard" class="value">-</div></div>
 <div class="card"><div class="label">Motor</div><div id="motorCard" class="value">-</div></div>
 <div class="card"><div class="label">Commanded</div><div id="cmdRpm" class="value">0 rpm</div></div>
 <div class="card"><div class="label">Measured</div><div id="measRpm" class="value">-</div></div>
 <div class="card"><div class="label">Iq / Id</div><div id="curr" class="value">0 / 0 A</div></div>
 <div class="card"><div class="label">Uq / Ud</div><div id="volt" class="value">0 / 0 V</div></div>
 <div class="card"><div class="label">Wi-Fi</div><div id="wifi" class="value">AP</div></div>
</div>

<div class="card section">
 <h2>Fan control</h2>
 <div class="split">
  <div>
   <div class="row"><div class="big"><span id="speedText">0</span>%</div><div class="pill" id="rpmScale">max 600 rpm</div></div>
   <input id="speed" type="range" min="0" max="100" step="1" value="0">
  </div>
  <div class="row">
   <button id="fwd" class="active" onclick="setDir(1)">Forward</button>
   <button id="rev" onclick="setDir(-1)">Reverse</button>
   <button class="stop" onclick="stopNow()">STOP</button>
  </div>
 </div>
 <div id="fault" class="fault"></div>
 <div class="row" style="margin-top:8px"><button id="clearFaultBtn" style="display:none" onclick="clearFaultNow()">Clear fault</button></div>
 <div id="ctrlMsg" class="msg"></div>
</div>

<div class="card section">
 <h2>Safety watchdog</h2>
 <div class="grid">
  <div><div class="label">Hall watchdog</div><div id="hallWatch" class="value">-</div></div>
  <div><div class="label">Hall RPM</div><div id="hallRpm" class="value">0 rpm</div></div>
  <div><div class="label">Hall quiet</div><div id="hallQuiet" class="value">0 ms</div></div>
  <div><div class="label">Loop gap (200 ms max)</div><div id="loopGap" class="value">0 us</div></div>
  <div><div class="label">Fault latch</div><div id="faultLatch" class="value">CLEAR</div></div>
 </div>
 <div class="note" style="margin-top:10px">In OPEN mode Hall is completely unused. In HALL mode, a hard stall or persistent low-RPM/high-current overload disables the driver immediately and latches the fault.</div>
</div>

<div class="card section">
 <h2>Motor settings</h2>
 <div class="formgrid">
  <div class="field"><label>Mode</label><select id="mode"><option value="0">Open-loop + real current FOC</option><option value="1">Hall closed-loop + current FOC</option></select></div>
  <div class="field"><label>Max RPM</label><input id="maxRpm" type="number" step="10"></div>
  <div class="field"><label>Minimum running RPM</label><input id="minRpm" type="number" step="10"></div>
  <div class="field"><label>Full 0→100% S-curve time (s)</label><input id="rampTime" type="number" min="0.25" max="8" step="0.1"></div>
  <div class="field"><label>Current limit A (hard max 2 A)</label><input id="currentLimit" type="number" step="0.05"></div>
  <div class="field"><label>Motor voltage limit V (hard max 3 V)</label><input id="voltageLimit" type="number" step="0.1"></div>
  <div class="field"><label>FOC alignment voltage V (current sense / Hall)</label><input id="alignVoltage" type="number" step="0.05"></div>
  <div class="field"><label>Current PI - P</label><input id="currentP" type="number" step="0.01"></div>
  <div class="field"><label>Current PI - I</label><input id="currentI" type="number" step="1"></div>
  <div class="field"><label>Current LPF Tf s</label><input id="currentTf" type="number" step="0.0005"></div>
  <div class="field"><label>Hall velocity PI - P</label><input id="velocityP" type="number" step="0.001"></div>
  <div class="field"><label>Hall velocity PI - I</label><input id="velocityI" type="number" step="0.01"></div>
  <div class="field"><label>Hall velocity LPF Tf s</label><input id="velocityTf" type="number" step="0.005"></div>
 </div>
 <div class="row" style="margin-top:12px">
  <button class="primary" onclick="saveSettings()">Apply & save</button>
  <button onclick="recalHall()">Recalibrate Hall</button>
  <button onclick="resetDefaults()">Safe defaults</button>
  <button onclick="restartEsp()">Restart</button>
 </div>
 <div class="note" style="margin-top:10px">Changing control mode restarts the ESP32. Hall recalibration also restarts and may move the motor briefly during FOC alignment. In OPEN mode the same alignment-voltage setting is used only for current-sense phase/sign alignment; Hall remains unused. The fan boots without a run command, but electrical alignment can move it briefly.</div>
 <div id="settingsMsg" class="msg"></div>
</div>

<div class="card section">
 <h2>Home Wi-Fi (optional)</h2>
 <div class="formgrid">
  <div class="field"><label>SSID</label><input id="ssid" list="wifiList" maxlength="32" autocomplete="off"><datalist id="wifiList"></datalist></div>
  <div class="field"><label>Password</label><input id="pass" type="password" maxlength="64" autocomplete="new-password"></div>
 </div>
 <div class="row" style="margin-top:12px">
  
  <button class="primary" onclick="saveWifi()">Save Wi-Fi & restart</button>
 </div>
 <div id="scanMsg" class="msg"></div>
 <div class="note" style="margin-top:10px">Enter the network SSID manually. Radio scans are disabled. Recovery AP is always available: <b>MaxxFan-Setup</b> / <b>MaxxFan123</b>, normally at <b>192.168.4.1</b>. You can also try <b>maxxfan.local</b>. The saved home password is never returned by the status API.</div>
 <div id="wifiMsg" class="msg"></div>
</div>

<div class="card section note">
 <b>Current mode note:</b> OPEN mode has real phase-current feedback but does not initialize or read the Hall sensor. HALL mode closes both rotor velocity/angle information and current control. All normal speed changes use a monotonic quintic S-curve. OPEN-mode current is softly faded at enable/disable to avoid the zero-speed click. The GUI intentionally makes no periodic network requests while the motor is active.<br><br>
 <span id="extra"></span>
</div>
</div>
<script>
let dir=1, first=true, timer=null, refreshTimer=null, fullRampMs=1800;
let controlBusy=false, controlDirty=false;
const $=id=>document.getElementById(id);
function enc(o){return Object.entries(o).map(([k,v])=>encodeURIComponent(k)+'='+encodeURIComponent(v)).join('&')}
async function post(url,obj){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-MaxxFan-Control':'1'},body:enc(obj)});return await r.text()}
function stopRefresh(){if(refreshTimer!==null){clearTimeout(refreshTimer);refreshTimer=null}}
function scheduleRefresh(ms){stopRefresh();if(ms>0)refreshTimer=setTimeout(refresh,ms)}
function setDirUI(){$('fwd').classList.toggle('active',dir===1);$('rev').classList.toggle('active',dir===-1)}
function setDir(d){clearTimeout(timer);dir=d;setDirUI();queueControl()}
function queueControl(){stopRefresh();controlDirty=true;flushControl()}
async function flushControl(){
 if(controlBusy)return;
 controlBusy=true;
 try{
  while(controlDirty){
   controlDirty=false;
   const sp=Number($('speed').value), d=dir;
   try{await post('/api/control',{speed:sp,dir:d})}catch(e){$('fault').textContent='GUI command connection lost'}
  }
 }finally{controlBusy=false}
}
$('speed').addEventListener('input',()=>{$('speedText').textContent=$('speed').value});
$('speed').addEventListener('change',()=>{clearTimeout(timer);queueControl()});
async function stopNow(){
 clearTimeout(timer);stopRefresh();controlDirty=false;
 $('speed').value=0;$('speedText').textContent=0;
 try{await post('/api/stop',{});$('ctrlMsg').textContent='Smooth stop requested'}catch(e){$('fault').textContent='STOP connection lost'}
 // No network traffic during the S-curve. Refresh only after the worst-case
 // full ramp time plus margin, when the driver should already be disabled.
 scheduleRefresh(fullRampMs+700);
}
async function clearFaultNow(){clearTimeout(timer);stopRefresh();$('speed').value=0;$('speedText').textContent=0;$('ctrlMsg').textContent=await post('/api/clear-fault',{});scheduleRefresh(800)}
function f(n,d=3){return Number(n).toFixed(d)}
function applyStatus(s){
 $('modeCard').textContent=s.mode===1?'HALL CURRENT FOC':'OPEN CURRENT FOC';
 $('motorCard').textContent=!s.ready?'FAULT':s.retry_pending?'RETRY IN '+Math.ceil(s.retry_remaining_ms/1000)+' s':s.fault_latched?'FAULT LATCHED':s.reversing?'REVERSING / SETTLING':s.enabled?(s.scurve?'RAMPING':'RUNNING'):'STOPPED';
 $('cmdRpm').textContent=f(s.commanded_rpm,0)+' rpm';
 $('measRpm').textContent=s.measured_valid?f(s.measured_rpm,0)+' rpm':'open-loop';
 $('curr').textContent=f(s.iq,2)+' / '+f(s.id,2)+' A';
 $('volt').textContent=f(s.uq,2)+' / '+f(s.ud,2)+' V';
 $('wifi').textContent=s.sta_connected?(s.sta_ip+' · '+s.rssi+' dBm'):'AP '+s.ap_ip;
 $('fault').textContent=(s.fault||'')+(s.retry_pending?' — automatic retry '+(s.retry_attempts+1)+'/3; STOP cancels':'');
 $('clearFaultBtn').style.display=s.fault_latched?'inline-block':'none';
 $('hallWatch').textContent=s.hall_watchdog?'ACTIVE':'OFF';
 $('hallRpm').textContent=s.hall_watchdog?f(s.hall_rpm,0)+' rpm':'-';
 $('hallQuiet').textContent=s.hall_watchdog?s.hall_quiet_ms+' ms':'-';
 $('loopGap').textContent=s.loop_gap_us+' us';
 $('faultLatch').textContent=s.fault_latched?'LATCHED':'CLEAR';
 $('extra').textContent='Current limit '+f(s.current_limit,2)+' A · voltage limit '+f(s.voltage_limit,2)+' V · telemetry + Wi-Fi polling paused while motor runs';
 if(first){dir=s.requested_dir<0?-1:1;setDirUI();$('speed').value=Math.round(s.requested_percent);$('speedText').textContent=Math.round(s.requested_percent);first=false}
}
async function refresh(){
 try{
  const s=await (await fetch('/api/status',{cache:'no-store'})).json();
  applyStatus(s);
  // Exactly like the known-clean 3.7 principle: do NOT generate periodic Wi-Fi
  // traffic while PWM/control is active.
  if(!s.enabled && !s.reversing && !s.scurve && Number(s.requested_percent)<=0.1 && !document.hidden) scheduleRefresh(2000);
 }catch(e){$('fault').textContent='GUI connection lost';if(!document.hidden)scheduleRefresh(2000)}
}
async function loadConfig(){
 try{const c=await (await fetch('/api/config',{cache:'no-store'})).json();
  $('mode').value=c.mode;$('maxRpm').value=c.max_rpm;$('minRpm').value=c.min_rpm;
  const rt=Number(c.max_rpm)/Math.max(Number(c.accel_rpm_s),1);fullRampMs=Math.max(250,Math.min(8000,rt*1000));$('rampTime').value=(fullRampMs/1000).toFixed(2);
  $('currentLimit').value=c.current_limit;$('voltageLimit').value=c.voltage_limit;$('alignVoltage').value=c.align_voltage;
  $('currentP').value=c.current_p;$('currentI').value=c.current_i;$('currentTf').value=c.current_tf;
  $('velocityP').value=c.velocity_p;$('velocityI').value=c.velocity_i;$('velocityTf').value=c.velocity_tf;
  $('rpmScale').textContent='max '+f(c.max_rpm,0)+' rpm';
 }catch(e){$('fault').textContent='Config connection lost'}
}
async function saveSettings(){
 const maxRpm=Math.max(Number($('maxRpm').value),1), rampTime=Math.max(Number($('rampTime').value),0.25);
 const accel=maxRpm/rampTime;
 const obj={mode:$('mode').value,maxRpm:$('maxRpm').value,minRpm:$('minRpm').value,accel,currentLimit:$('currentLimit').value,voltageLimit:$('voltageLimit').value,alignVoltage:$('alignVoltage').value,currentP:$('currentP').value,currentI:$('currentI').value,currentTf:$('currentTf').value,velocityP:$('velocityP').value,velocityI:$('velocityI').value,velocityTf:$('velocityTf').value};
 stopRefresh();$('settingsMsg').textContent=await post('/api/settings',obj)
}
async function recalHall(){if(confirm('Clear saved Hall calibration and restart? The motor may move during calibration.')){stopRefresh();$('settingsMsg').textContent=await post('/api/recal',{})}}
async function resetDefaults(){if(confirm('Restore safe motor defaults? Wi-Fi credentials are kept.')){stopRefresh();$('settingsMsg').textContent=await post('/api/defaults',{})}}
async function restartEsp(){stopRefresh();$('settingsMsg').textContent=await post('/api/restart',{})}
async function saveWifi(){stopRefresh();$('wifiMsg').textContent=await post('/api/wifi',{ssid:$('ssid').value,pass:$('pass').value})}
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopRefresh();else refresh()});
loadConfig();refresh();
</script></body></html>
)rawliteral";

bool requirePostParam(AsyncWebServerRequest* request, const char* name) {
  return request->hasParam(name, true);
}

bool parseFiniteNumber(const char* text, float& value) {
  if (!text || !*text) return false;
  char* end = nullptr;
  errno = 0;
  value = strtof(text, &end);
  return end != text && *end == '\0' && errno != ERANGE && isfinite(value);
}
bool validateNumericPost(AsyncWebServerRequest* request) {
  const char* names[] = {"speed", "dir", "mode", "maxRpm", "minRpm", "accel",
    "currentLimit", "voltageLimit", "alignVoltage", "currentP", "currentI",
    "currentTf", "velocityP", "velocityI", "velocityTf"};
  for (const char* name : names) {
    if (!request->hasParam(name, true)) continue;
    float value;
    if (!parseFiniteNumber(request->getParam(name, true)->value().c_str(), value) ||
        (!strcmp(name, "dir") && value != -1 && value != 1) ||
        (!strcmp(name, "mode") && value != 0 && value != 1)) {
      request->send(400, "text/plain", "Invalid numeric parameter");
      return false;
    }
  }
  return true;
}
float postFloat(AsyncWebServerRequest* request, const char* name, float fallback) {
  if (!request->hasParam(name, true)) return fallback;
  float value;
  return parseFiniteNumber(request->getParam(name, true)->value().c_str(), value)
    ? value : fallback;
}

int postInt(AsyncWebServerRequest* request, const char* name, int fallback) {
  if (!request->hasParam(name, true)) return fallback;
  return request->getParam(name, true)->value().toInt();
}

bool authorizeRequest(AsyncWebServerRequest* request) {
  if (!request->authenticate("admin", AP_PASSWORD)) {
    request->requestAuthentication();
    return false;
  }
  // Cross-origin forms cannot provide this header; no CORS permission is sent.
  if (request->method() == HTTP_POST &&
      (!request->hasHeader("X-MaxxFan-Control") ||
       request->getHeader("X-MaxxFan-Control")->value() != "1")) {
    request->send(403, "text/plain", "Missing control header");
    return false;
  }
  return true;
}
void jsonSafeCopy(const char* src, char* dst, size_t dstSize) {
  if (!dstSize) return;
  size_t n = 0;
  if (src) {
    while (*src && n + 1 < dstSize) {
      const char c = *src++;
      // Fault strings are firmware-generated. Replace JSON-breaking/control chars
      // instead of allocating a temporary escaped String in the live path.
      dst[n++] = (c == '"' || c == '\\' || (uint8_t)c < 0x20) ? '_' : c;
    }
  }
  dst[n] = 0;
}

size_t buildLiveJson(const Telemetry& t, char* out, size_t outSize) {
  char safeFault[96];
  jsonSafeCopy(t.fault, safeFault, sizeof(safeFault));
  const int written = snprintf(out, outSize,
    "{\"ready\":%s,\"enabled\":%s,\"reversing\":%s,\"scurve\":%s,"
    "\"mode\":%u,\"requested_percent\":%.1f,\"requested_dir\":%d,"
    "\"commanded_rpm\":%.1f,\"measured_valid\":%s,\"measured_rpm\":%.1f,"
    "\"iq\":%.3f,\"id\":%.3f,\"uq\":%.3f,\"ud\":%.3f,\"current_limit\":%.3f,\"voltage_limit\":%.3f,"
    "\"hall_rpm\":%.1f,\"hall_watchdog\":%s,\"hall_quiet_ms\":%lu,"
    "\"loop_gap_us\":%lu,\"fault_latched\":%s,\"retry_pending\":%s,"
    "\"retry_attempts\":%u,\"retry_remaining_ms\":%lu,\"sta_connected\":%s,"
    "\"rssi\":%ld,\"sta_ip\":\"%s\",\"ap_ip\":\"%s\",\"fault\":\"%s\"}",
    t.motorReady ? "true" : "false",
    t.enabled ? "true" : "false",
    t.reversing ? "true" : "false",
    t.sCurveActive ? "true" : "false",
    (unsigned)t.mode,
    t.requestedPercent,
    (int)t.requestedDirection,
    t.commandedRpm,
    t.measuredRpmValid ? "true" : "false",
    t.measuredRpm,
    t.iqA,
    t.idA,
    t.uqV,
    t.udV,
    t.currentLimitA,
    t.voltageLimitV,
    t.hallRpm,
    t.hallWatchdogEnabled ? "true" : "false",
    (unsigned long)t.hallQuietMs,
    (unsigned long)t.loopGapUs,
    t.faultLatched ? "true" : "false",
    t.retryPending ? "true" : "false",
    (unsigned)t.retryAttempts,
    (unsigned long)t.retryRemainingMs,
    t.staConnected ? "true" : "false",
    (long)t.wifiRssi,
    t.staIp,
    t.apIp,
    safeFault);
  if (written < 0 || (size_t)written >= outSize) {
    if (outSize) out[0] = 0;
    return 0;
  }
  return (size_t)written;
}

String buildConfigJson(const Config& c) {
  String j;
  j.reserve(500);
  j += "{";
  j += "\"mode\":" + String((int)c.mode);
  j += ",\"max_rpm\":" + String(c.maxRpm, 1);
  j += ",\"min_rpm\":" + String(c.minRpm, 1);
  j += ",\"accel_rpm_s\":" + String(c.accelRpmPerSec, 1);
  j += ",\"current_limit\":" + String(c.currentLimitA, 3);
  j += ",\"voltage_limit\":" + String(c.motorVoltageLimitV, 3);
  j += ",\"align_voltage\":" + String(c.alignVoltageV, 3);
  j += ",\"current_p\":" + String(c.currentP, 4);
  j += ",\"current_i\":" + String(c.currentI, 3);
  j += ",\"current_tf\":" + String(c.currentTf, 5);
  j += ",\"velocity_p\":" + String(c.velocityP, 5);
  j += ",\"velocity_i\":" + String(c.velocityI, 4);
  j += ",\"velocity_tf\":" + String(c.velocityTf, 5);
  j += ",\"hall_cal_valid\":" + String(c.hallCalValid ? "true" : "false");
  j += "}";
  return j;
}

void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    AsyncWebServerResponse* response = request->beginResponse_P(200, "text/html", INDEX_HTML);
    response->addHeader("Cache-Control", "no-store, no-cache, must-revalidate");
    request->send(response);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    char json[768];
    if (!buildLiveJson(telemetrySnapshot(), json, sizeof(json))) {
      request->send(500, "text/plain", "Status serialization failed");
      return;
    }
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", json);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  server.on("/api/config", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    const String j = buildConfigJson(configSnapshot());
    AsyncWebServerResponse* response = request->beginResponse(200, "application/json", j);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
  });

  server.on("/api/control", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    if (!validateNumericPost(request)) return;
    if (request->hasParam("speed", true) && postFloat(request, "speed", -1) == 0) {
      portENTER_CRITICAL(&stateMux);
      pendingStop = true;
      portEXIT_CRITICAL(&stateMux);
      request->send(200, "text/plain", "Smooth stop requested; automatic retry cancelled.");
      return;
    }
    if (telemetrySnapshot().faultLatched) {
      request->send(409, "text/plain", "Recovery pending or fault latched. STOP cancels; clear leaves stopped.");
      return;
    }
    if (!requirePostParam(request, "speed") || !requirePostParam(request, "dir")) {
      request->send(400, "text/plain", "Missing speed/dir");
      return;
    }
    PendingControl pc{};
    pc.valid = true;
    pc.percent = clampf(postFloat(request, "speed", 0.0f), 0.0f, 100.0f);
    pc.direction = postInt(request, "dir", 1) < 0 ? -1 : 1;
    pc.receivedAtMs = millis();

    portENTER_CRITICAL(&stateMux);
    pendingControl = pc;
    portEXIT_CRITICAL(&stateMux);
    request->send(204, "text/plain", "");
  });

  server.on("/api/stop", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    portENTER_CRITICAL(&stateMux);
    pendingStop = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "text/plain", "Stop requested");
  });

  server.on("/api/clear-fault", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    portENTER_CRITICAL(&stateMux);
    pendingClearFault = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "text/plain", "Fault clear requested. Motor remains stopped.");
  });

  server.on("/api/settings", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    if (!validateNumericPost(request)) return;
    Config c = configSnapshot();
    c.mode = (uint8_t)postInt(request, "mode", c.mode);
    c.maxRpm = postFloat(request, "maxRpm", c.maxRpm);
    c.minRpm = postFloat(request, "minRpm", c.minRpm);
    c.accelRpmPerSec = postFloat(request, "accel", c.accelRpmPerSec);
    c.currentLimitA = postFloat(request, "currentLimit", c.currentLimitA);
    c.motorVoltageLimitV = postFloat(request, "voltageLimit", c.motorVoltageLimitV);
    c.alignVoltageV = postFloat(request, "alignVoltage", c.alignVoltageV);
    c.currentP = postFloat(request, "currentP", c.currentP);
    c.currentI = postFloat(request, "currentI", c.currentI);
    c.currentTf = postFloat(request, "currentTf", c.currentTf);
    c.velocityP = postFloat(request, "velocityP", c.velocityP);
    c.velocityI = postFloat(request, "velocityI", c.velocityI);
    c.velocityTf = postFloat(request, "velocityTf", c.velocityTf);
    validateConfig(c);

    PendingSettings ps{};
    ps.valid = true;
    ps.value = c;

    portENTER_CRITICAL(&stateMux);
    pendingSettings = ps;
    portEXIT_CRITICAL(&stateMux);

    request->send(200, "text/plain", "Settings accepted. Fan will S-curve stop before applying them.");
  });

  server.on("/api/wifi-scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    const Telemetry t = telemetrySnapshot();

    // A synchronous scan can briefly occupy the Wi-Fi stack. Do it only
    // while the fan is fully stopped so it cannot disturb motor control.
    if (t.enabled || t.reversing || t.requestedPercent > 0.1f) {
      request->send(409, "text/plain", "Stop the motor before scanning Wi-Fi.");
      return;
    }

    request->send(409, "text/plain", "Enter SSID manually; radio scanning disabled to protect motor timing.");
  });

  server.on("/api/wifi", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    if (!request->hasParam("ssid", true)) {
      request->send(400, "text/plain", "Missing SSID");
      return;
    }
    String ssid = request->getParam("ssid", true)->value();
    String pass = request->hasParam("pass", true) ? request->getParam("pass", true)->value() : "";
    ssid.trim();
    if (ssid.length() > 32 || pass.length() > 64) {
      request->send(400, "text/plain", "SSID/password too long");
      return;
    }

    PendingWifi p{};
    p.valid = true;
    strncpy(p.ssid, ssid.c_str(), sizeof(p.ssid) - 1);
    strncpy(p.password, pass.c_str(), sizeof(p.password) - 1);

    portENTER_CRITICAL(&stateMux);
    pendingWifi = p;
    portEXIT_CRITICAL(&stateMux);

    request->send(200, "text/plain", "Wi-Fi saved. Restarting...");
  });

  server.on("/api/recal", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    portENTER_CRITICAL(&stateMux);
    pendingRecalibrate = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "text/plain", "Hall calibration cleared. Restarting...");
  });

  server.on("/api/defaults", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    portENTER_CRITICAL(&stateMux);
    pendingDefaults = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "text/plain", "Safe defaults restored. Restarting...");
  });

  server.on("/api/restart", HTTP_POST, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    portENTER_CRITICAL(&stateMux);
    pendingRestart = true;
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "text/plain", "Restarting...");
  });

  server.onNotFound([](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    request->send(404, "text/plain", "Not found");
  });

  server.begin();
  Serial.println("Web server started.");
}

// ============================================================================
// 16. SERIAL - intentionally quiet unless requested
// ============================================================================

void printSerialStatus() {
  if (Serial.availableForWrite() < 400) return;
  Telemetry t = telemetrySnapshot();
  Serial.printf("retry_pending=%d attempts=%u/3 cooldown=%lums ",
    t.retryPending, t.retryAttempts, (unsigned long)t.retryRemainingMs);
  Serial.printf(
    "mode=%s ready=%d enabled=%d req=%.0f%% dir=%d cmd=%.0frpm ",
    t.mode == HALL_CURRENT_FOC ? "HALL" : "OPEN",
    t.motorReady,
    t.enabled,
    t.requestedPercent,
    t.requestedDirection,
    t.commandedRpm
  );
  if (t.measuredRpmValid) Serial.printf("meas=%.0frpm ", t.measuredRpm);
  Serial.printf("Hall=%.0frpm quiet=%lums watchdog=%d gap=%luus S=%d Iq=%.3fA Id=%.3fA latched=%d fault=%s\n",
                t.hallRpm, (unsigned long)t.hallQuietMs, t.hallWatchdogEnabled,
                (unsigned long)t.loopGapUs, t.sCurveActive, t.iqA, t.idA,
                t.faultLatched, t.fault);
  Serial.printf("Uq=%.3fV Ud=%.3fV\n", t.uqV, t.udV);
}

void handleSerial() {
  static char buf[48];
  static size_t n = 0;
  static bool overflow = false;
  unsigned budget = 32;
  while (budget-- && Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\r') continue;
    if (ch == '\n') {
      if (overflow) { n = 0; overflow = false; continue; }
      buf[n] = 0;
      n = 0;
      String s(buf);
      s.trim();
      s.toLowerCase();
      if (s == "?" || s == "status") {
        printSerialStatus();
      } else if (s == "stop" || s == "x") {
        retryPolicy.cancel();
        requestedPercent = 0.0f;
        Serial.println("Smooth S-curve stop requested");
      } else if (s == "kill") {
        retryPolicy.cancel();
        requestedPercent = 0.0f;
        resetSCurve(0.0f);
        disableMotorNow();
        Serial.println("KILL: driver disabled immediately");
      } else if (s == "clear") {
        clearSafetyFault();
      } else if (s == "f") {
        retryPolicy.cancel();
        requestedDirection = 1;
        Serial.println("Direction: forward");
      } else if (s == "r") {
        retryPolicy.cancel();
        requestedDirection = -1;
        Serial.println("Direction: reverse");
      } else if (s.startsWith("s ")) {
        float stopValue;
        if (parseFiniteNumber(s.substring(2).c_str(), stopValue) && stopValue == 0) {
          retryPolicy.cancel();
          requestedPercent = 0;
          continue;
        }
        if (safetyFaultLatched) {
          Serial.println("Safety fault is latched. Use: clear");
        } else {
          float value;
          if (!parseFiniteNumber(s.substring(2).c_str(), value)) continue;
          requestedPercent = clampf(value, 0.0f, 100.0f);
          Serial.printf("Speed request: %.0f%%\n", requestedPercent);
        }
      } else if (s.length()) {
        Serial.println("Commands: status | s 0..100 | f | r | stop | kill | clear");
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = ch;
    } else {
      overflow = true;
    }
  }
}

// ============================================================================
// 17. ARDUINO SETUP / LOOP
// ============================================================================

void setup() {
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);
  Serial.setTxBufferSize(1024);
  Serial.begin(115200);
  delay(150);
  Serial.println();
  Serial.print("=== MaxxFan MKS ESP32 FOC Mega v");
  Serial.print(FIRMWARE_VERSION);
  Serial.println(" ===");
  Serial.println("SimpleFOC 2.4 expected");

  if (!prefs.begin("maxxfan", false)) {
    Serial.println("FATAL: NVS unavailable; driver disabled");
    while (true) delay(1000);
  }
  loadConfig();

  // One-time repair for v0.3.4/v0.3.5 NOHALL builds that could persist
  // HALL_CURRENT_FOC as the default mode in NVS. This commissioning build
  // forces OPEN exactly once, then leaves future GUI mode selections alone.
  // This is intentionally a safe migration because no Hall sensor is connected
  // for the present commissioning setup.
  if (!prefs.getBool("nhfix036", false)) {
    if (cfg.mode == HALL_CURRENT_FOC) {
      Serial.println("NVS migration v0.3.6: forcing OPEN_CURRENT_FOC once (old builds could store HALL by default).");
      cfg.mode = OPEN_CURRENT_FOC;
      saveConfig();
    }
    if (prefs.putBool("nhfix036", true) == 0) {
      Serial.println("WARNING: could not store v0.3.6 mode-migration marker");
    }
  }

  Serial.print("Control mode: ");
  Serial.println(cfg.mode == HALL_CURRENT_FOC ? "HALL_CURRENT_FOC" : "OPEN_CURRENT_FOC");

  if (!ENABLE_EXPERIMENTAL_MOTION_SAFETY) prefs.putBool("safetytrip", false);
  const bool persistedFault = ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
                             prefs.getBool("safetytrip", false);
  motorReady = setupMotor();
  if (!motorReady) disableMotorNow();
  if (persistedFault) {
    safetyFaultLatched = true;
    setFault("PREVIOUS_FAULT: inspect hardware, then clear manually");
  }
  publishedConfig = cfg;

  setupWiFi();
  setupWebServer();
  updateTelemetry();

  Serial.println("GUI recovery AP: MaxxFan-Setup / MaxxFan123");
  Serial.println("GUI URL: http://192.168.4.1  or  http://maxxfan.local");
  Serial.println("Open: http://192.168.4.1");
  Serial.printf("Safety: experimental trips %s; Hall watchdog %s; OPEN mode is Hall-independent\n",
                ENABLE_EXPERIMENTAL_MOTION_SAFETY ? "ON" : "OFF",
                (ENABLE_EXPERIMENTAL_MOTION_SAFETY && cfg.mode == HALL_CURRENT_FOC) ? "ON" : "OFF");
  Serial.println("Serial commands: status | s 0..100 | f | r | stop | kill | clear");
}

void loop() {
  // Motor loop first and as frequently as possible.
  runMotorControl();

  processPendingCommands();
  updateTelemetry();
  handleSerial();
  serviceAutoRecovery();

  if (scheduledRestartAtMs && (int32_t)(millis() - scheduledRestartAtMs) >= 0) {
    requestedPercent = 0.0f;
    resetSCurve(0.0f);
    disableMotorNow();
    delay(20);
    ESP.restart();
  }
}
