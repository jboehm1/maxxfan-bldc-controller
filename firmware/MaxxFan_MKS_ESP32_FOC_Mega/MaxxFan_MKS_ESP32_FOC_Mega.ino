/*
  ============================================================================
  MaxxFan BLDC Controller
  Firmware : v0.3.7-TEST
  Board    : MKS ESP32 FOC Mega (single motor)
  Motor    : StepperOnline 57BYA54-12-01
  Library  : SimpleFOC 2.4.0
  Target   : classic ESP32 / Arduino-ESP32

  CONTROL MODES
  ---------------------------------------------------------------------------
  OPEN_CURRENT_FOC
    MotionControlType::velocity_openloop
    TorqueControlType::foc_current

    - Real physical phase-current feedback on GPIO39/GPIO36.
    - SimpleFOC generates the electrical angle from commanded velocity.
    - Hall sensors are COMPLETELY UNUSED:
        no Hall init
        no Hall GPIO reads
        no Hall interrupts
        no Hall RPM
        no Hall startup check
        no Hall stall watchdog
    - d/q current regulation and current limiting remain active.
    - Rotor position is not measured, so synchronism cannot be guaranteed.

  HALL_CURRENT_FOC
    MotionControlType::velocity
    TorqueControlType::foc_current

    - Real physical phase-current feedback.
    - Hall rotor feedback on GPIO18/GPIO19/GPIO15.
    - Closed-loop velocity + closed-loop d/q current control.
    - Hall stall/overload safety is deliberately DISABLED in this test build.
    - Recommended mode for long-duration operation after commissioning.

  HARDWARE MAPPING
  ---------------------------------------------------------------------------
    PWM U/V/W    : GPIO 32 / 33 / 25
    Driver EN    : GPIO 12
    Hall A/B/C   : GPIO 18 / 19 / 15
    Current A/B  : GPIO 39 / 36
    Current shunt: 0.01 ohm
    Amplifier gain: 50 V/V
    Motor pole pairs: 2
    Nominal supply: 12 V

  SAFETY BEHAVIOUR
  ---------------------------------------------------------------------------
  - Motor always boots STOPPED.
  - Driver is disabled at zero command.
  - Direction reversal disables the bridge and coasts for at least 500 ms.
  - Hall mode additionally waits until Hall activity is quiet.
  - The experimental Hall/current/timing safety layer is disabled temporarily.
    Its code remains below behind ENABLE_EXPERIMENTAL_MOTION_SAFETY.
  - OPEN mode intentionally has no rotor-stall detector because it has no
    rotor feedback. Current limiting and over-current supervision remain active.

  WI-FI / GUI
  ---------------------------------------------------------------------------
    Recovery AP : MaxxFan-Setup
    AP password : MaxxFan123
    URL         : http://192.168.4.1
    mDNS        : http://maxxfan.local
    HTTP user   : admin
    HTTP password = AP password

    Optional home Wi-Fi credentials are stored in ESP32 NVS.
    Wi-Fi scanning is permitted only while the motor is stopped.

  ADC NOTE
  ---------------------------------------------------------------------------
  This firmware deliberately does NOT read VIN through GPIO13 while Wi-Fi is
  active. GPIO13 is ADC2 on classic ESP32. Physical current feedback uses ADC1
  pins GPIO39/GPIO36, avoiding the previous Wi-Fi/ADC contention path.

  QUALIFICATION NOTE
  ---------------------------------------------------------------------------
  This is the final software architecture for the present hardware. Before
  unattended real-world deployment it still requires successful target build,
  current-scale validation, Hall validation, thermal testing and endurance
  testing on the actual board/motor/fan.

  The .ino is self-contained apart from normal Arduino/ESP32/SimpleFOC/
  AsyncTCP/ESPAsyncWebServer libraries. No project-local header is required.
  ============================================================================
*/

#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <SimpleFOC.h>
#include <WiFi.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

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
//    Default motor voltage is 3 V for commissioning. After hardware
//    validation the GUI may be raised up to the 6 V SinePWM half-bus ceiling.
// ============================================================================

static constexpr float HARD_MAX_CURRENT_A = 2.0f;
static constexpr float HARD_MAX_MOTOR_VOLTAGE_V = 6.0f;
static constexpr float HARD_DRIVER_VOLTAGE_LIMIT_V = SUPPLY_VOLTAGE_V;
static constexpr float HARD_MAX_RPM = 3000.0f;
static constexpr float HARD_MAX_ALIGN_VOLTAGE_V = 1.2f;

// STOP releases the driver; physical coast-down is observed before restarting.


static constexpr float STALL_MIN_COMMAND_RPM = 35.0f;
static constexpr uint32_t STALL_STARTUP_GRACE_MS = 1800;
static constexpr uint32_t STALL_NO_HALL_TIMEOUT_MS = 800;
static constexpr float OVERLOAD_MIN_SPEED_RATIO = 0.35f;
static constexpr float OVERLOAD_CURRENT_RATIO = 0.75f;
static constexpr uint32_t OVERLOAD_HOLD_MS = 1500;
static constexpr uint32_t SAFETY_CHECK_PERIOD_MS = 20;

// Temporary commissioning switch requested by the user.
// false: no WINDMILL / STALL / OVERLOAD / Hall-invalid / timing / software-
// current safety trip, no automatic retry and no stored safety fault.
// true: re-enables the experimental safety layer after it has been validated
// with the actual Hall wiring and fan mechanics.
static constexpr bool ENABLE_EXPERIMENTAL_MOTION_SAFETY = false;


// ============================================================================
// BOUNDED AUTOMATIC RECOVERY - HALL MODE ONLY
// ============================================================================
//
// Only Hall-mode STALL and OVERLOAD faults are eligible for automatic retry.
// OPEN_CURRENT_FOC cannot diagnose rotor stall reliably because it intentionally
// has no rotor feedback.
//
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

    const bool recoverable =
      code &&
      (strcmp(code, "STALL") == 0 || strcmp(code, "OVERLOAD") == 0);

    if (!recoverable || !hallAvailable || request <= 0.0f ||
        attempts >= maxAttempts) {
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
// 3. WI-FI
// ============================================================================

static const char* AP_SSID = "MaxxFan-Setup";
static const char* AP_PASSWORD = "MaxxFan123";   // >= 8 characters

static constexpr const char* FIRMWARE_VERSION = "0.3.8-GUI-SAFE-TEST";

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

  // Conservative first-test limits.
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
  float currentLimitA;
  float voltageLimitV;
  uint32_t hallTransitions;
  float hallRpm;
  bool hallWatchdogEnabled;
  uint32_t hallQuietMs;
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
uint32_t controlTimestampUs = 0;
uint32_t startWaitingAtMs = 0;
uint32_t overCurrentAtMs = 0;
uint32_t invalidHallAtMs = 0;
static constexpr uint32_t START_QUIET_US = 500000;
static constexpr uint32_t START_WAIT_LIMIT_MS = 10000;
static constexpr uint32_t REVERSE_COAST_MS = 500;
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

float rampToward(float value, float target, float maxStep) {
  if (value < target) return min(value + maxStep, target);
  if (value > target) return max(value - maxStep, target);
  return value;
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

  // Hall watchdog exists only in HALL_CURRENT_FOC.
  // OPEN_CURRENT_FOC is intentionally completely Hall-independent.
  hallWatchdogEnabled = ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
                        (cfg.mode == HALL_CURRENT_FOC);
}

void enableMotorNow() {
  if (!motorReady || motorEnabled || safetyFaultLatched) return;
  resetControlLoops();
  resetSafetyWatchdogForStart();
  motor.current_sp = 0.0f;
  motor.target = 0.0f;
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
    requestedDirection, cfg.mode == HALL_CURRENT_FOC, millis());
  safetyFaultLatched = true;
  requestedPercent = 0.0f;
  rampedRadPerSec = 0.0f;
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
  rampedRadPerSec = 0.0f;
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
  rampedRadPerSec = 0.0f;
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
  // Preferred format: one configuration blob.
  //
  // If the stored blob has the same struct size, load it even if its version
  // differs. validateConfig() applies current safe bounds and the blob is then
  // rewritten with CONFIG_VERSION. This prevents ordinary firmware upgrades
  // from losing valid tuning solely because the schema version changed.
  //
  // If no compatible blob exists, import the legacy individual NVS keys once.
  const size_t blobLength = prefs.getBytesLength("config6");

  if (blobLength == sizeof(Config)) {
    Config saved{};

    if (prefs.getBytes("config6", &saved, sizeof(saved)) == sizeof(saved)) {
      const uint32_t oldVersion = saved.version;
      cfg = saved;
      validateConfig(cfg);

      if (oldVersion != CONFIG_VERSION) {
        Serial.printf("NVS blob migration: %lu -> %lu\n",
                      (unsigned long)oldVersion,
                      (unsigned long)CONFIG_VERSION);
        saveConfig();
      }
      return;
    }
  }

  const Config d = makeDefaultConfig();

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
  Serial.println("NVS: imported legacy/default configuration.");
  saveConfig();
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

  // Hall runtime configuration is touched only in Hall mode.
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

  // Hold all driver inputs inactive until SimpleFOC configures PWM.
  pinMode(PIN_PWM_U, OUTPUT);
  pinMode(PIN_PWM_V, OUTPUT);
  pinMode(PIN_PWM_W, OUTPUT);
  digitalWrite(PIN_PWM_U, LOW);
  digitalWrite(PIN_PWM_V, LOW);
  digitalWrite(PIN_PWM_W, LOW);
  // Torque control uses physical inline phase-current feedback.
  // No estimated-current/model-based torque controller is used.


  // Explicitly initialise v2.4 feed-forward state. Global objects are normally
  // zero-initialised already, but doing it here removes any ambiguity.
  motor.feed_forward_current.q = 0.0f;
  motor.feed_forward_current.d = 0.0f;
  motor.feed_forward_voltage.q = 0.0f;
  motor.feed_forward_voltage.d = 0.0f;
  motor.feed_forward_velocity = 0.0f;

  // TRUE HALL-INDEPENDENT OPEN LOOP:
  // Do not even initialise or read Hall GPIOs unless HALL_CURRENT_FOC is active.
  if (cfg.mode == HALL_CURRENT_FOC) {
    hall.pullup = Pullup::USE_EXTERN;
    hall.velocity_max = max(rpmToRad(cfg.maxRpm) * 1.7f, 150.0f);
    hall.init();
    lastHallState = readHallState();
    lastHallEdgeUs = micros();
    hall.enableInterrupts(doHallA, doHallB, doHallC);
    motor.linkSensor(&hall);

    // Reserved for a future safety pass. Do not block current commissioning
    // on a noisy/moving Hall input.
    if (ENABLE_EXPERIMENTAL_MOTION_SAFETY) {
      const uint32_t waitStart = millis();
      while (hallQuietUs() < START_QUIET_US) {
        if (millis() - waitStart >= START_WAIT_LIMIT_MS) { {
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

  if (!motor.initFOC()) {
    setFault("initFOC failed - check phases/Hall/current sense");
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
  rampedRadPerSec = 0.0f;
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

  const uint32_t now = millis();
  if (now - lastSafetyCheckMs < SAFETY_CHECK_PERIOD_MS) return;
  lastSafetyCheckMs = now;

  // Hall-based stall/overload monitoring exists only in Hall mode.
  // OPEN_CURRENT_FOC must never read or update the Hall sensor.
  if (cfg.mode != HALL_CURRENT_FOC) {
    hallWatchdogEnabled = false;
    overloadStartedMs = 0;
    return;
  }

  const uint32_t hc = hallTransitions;
  if (hc != lastHallCount) {
    lastHallCount = hc;
    lastHallActivityMs = now;
  }

  safetyHallRpm = fabsf(radToRpm(hall.getVelocity()));

  // STOP/reversal release the driver; do not diagnose coast-down as overload.
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

  if (!hallWatchdogEnabled) {
    overloadStartedMs = 0;
    return;
  }

  // Level 1: hard stall / missing Hall motion. The timeout is deliberately
  // far longer than normal Hall-sector timing, rejecting brief gusts/RPM ripple.
  const uint32_t quietMs = now - lastHallActivityMs;
  if (quietMs >= STALL_NO_HALL_TIMEOUT_MS) {
    tripSafetyFault("STALL", "no Hall motion while torque is commanded");
    return;
  }

  // Level 2: mechanical overload. Require BOTH severe speed loss AND current
  // near the configured current limit, continuously for OVERLOAD_HOLD_MS.
  // This rejects short wind gusts and acceleration transients.
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
    rampedRadPerSec = 0.0f;
    disableMotorNow();
    return;
  }
  const uint32_t nowUs = micros();
  const uint32_t elapsedUs = nowUs - controlTimestampUs;
  controlTimestampUs = nowUs;
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY && motorEnabled && elapsedUs > LOOP_GAP_LIMIT_US) {
    tripSafetyFault("LOOP_GAP", "motor loop exceeded 10 ms");
    return;
  }
  const float dt = min(elapsedUs * 1e-6f, 0.01f);
  if (cfg.mode == HALL_CURRENT_FOC) {
    hall.update();
    safetyHallRpm = fabsf(radToRpm(hall.getVelocity()));
  } else {
    safetyHallRpm = 0.0f;
  }

  if (requestedPercent <= 0.01f) {
    disableMotorNow();
    rampedRadPerSec = 0.0f;
    reversalPending = reversePauseActive = false;
    startWaitingAtMs = 0;
    return;
  }
  if (motorEnabled && requestedDirection != driveDirection) {
    // Never reverse an actively driven fan. Release the bridge first.
    // OPEN mode has no rotor sensor, so it uses a fixed minimum coast period.
    // HALL mode uses the same minimum period plus a Hall-quiet check.
    disableMotorNow();
    rampedRadPerSec = 0.0f;
    reversalPending = true;
    reversePauseActive = true;
    startWaitingAtMs = millis();
  }

  if (!motorEnabled) {
    if (!startWaitingAtMs) {
      startWaitingAtMs = millis();
    }

    const uint32_t stoppedForMs = millis() - startWaitingAtMs;

    if (reversePauseActive && stoppedForMs < REVERSE_COAST_MS) {
      reversalPending = true;
      return;
    }

    if (cfg.mode == HALL_CURRENT_FOC && ENABLE_EXPERIMENTAL_MOTION_SAFETY) {
      const uint8_t h = readHallState();

      if (h == 0 || h == 7) {
        tripSafetyFault("HALL_INVALID", "Hall state 000/111 before start");
        return;
      }

      if (hallQuietUs() < START_QUIET_US ||
          stoppedForMs < START_QUIET_US / 1000) {
        reversalPending = true;

        if (stoppedForMs >= START_WAIT_LIMIT_MS) {
          tripSafetyFault("WINDMILL", "rotor did not settle within 10 seconds");
        }
        return;
      }
    }

    reversalPending = false;
    reversePauseActive = false;
    startWaitingAtMs = 0;
    driveDirection = requestedDirection;
    overCurrentAtMs = 0;
    invalidHallAtMs = 0;
    enableMotorNow();
  }
  const float targetRpm = clampf(cfg.maxRpm * requestedPercent / 100.0f,
                                  cfg.minRpm, cfg.maxRpm);
  rampedRadPerSec = rampToward(rampedRadPerSec,
    rpmToRad(targetRpm) * driveDirection, rpmToRad(cfg.accelRpmPerSec) * dt);
  motor.loopFOC();
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
      (!isfinite(motor.current.q) || !isfinite(motor.current.d) ||
      !isfinite(motor.shaft_velocity))) {
    tripSafetyFault("NUMERIC", "non-finite control feedback");
    return;
  }
  // Filtered vector current is an additional software supervisor, NOT a
  // hardware short-circuit protector or guaranteed instantaneous current cap.
  const float magnitude = hypotf(motor.current.q, motor.current.d);
  if (ENABLE_EXPERIMENTAL_MOTION_SAFETY && magnitude > cfg.currentLimitA * 1.5f) {
    if (!overCurrentAtMs) overCurrentAtMs = millis();
    if (millis() - overCurrentAtMs >= OVERCURRENT_HOLD_MS) {
      tripSafetyFault("OVERCURRENT", "filtered dq magnitude above 150 percent");
      return;
    }
  } else overCurrentAtMs = 0;
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
  } else {
    invalidHallAtMs = 0;
  }
  motor.move(rampedRadPerSec);
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
  PendingControl pc{};
  PendingSettings ps{};
  PendingWifi pw{};
  bool doStop = false;
  bool doClearFault = false;
  bool doRecal = false;
  bool doDefaults = false;
  bool doRestart = false;

  portENTER_CRITICAL(&stateMux);
  if (pendingControl.valid) {
    pc = pendingControl;
    pendingControl.valid = false;
  }
  if (pendingSettings.valid) {
    ps = pendingSettings;
    pendingSettings.valid = false;
  }
  if (pendingWifi.valid) {
    pw = pendingWifi;
    pendingWifi.valid = false;
  }
  doStop = pendingStop; pendingStop = false;
  doClearFault = pendingClearFault; pendingClearFault = false;
  doRecal = pendingRecalibrate; pendingRecalibrate = false;
  doDefaults = pendingDefaults; pendingDefaults = false;
  doRestart = pendingRestart; pendingRestart = false;
  portEXIT_CRITICAL(&stateMux);

  const bool maintenance = ps.valid || pw.valid || doRecal || doDefaults || doRestart;
  if (maintenance || doStop || doClearFault) {
    retryPolicy.cancel();
    requestedPercent = 0.0f;
    rampedRadPerSec = 0.0f;
    reversalPending = reversePauseActive = false;
    startWaitingAtMs = 0;
    disableMotorNow();
    pc.valid = false;
  }
  if (pc.valid && !safetyFaultLatched && !scheduledRestartAtMs) {
    requestedPercent = clampf(pc.percent, 0.0f, 100.0f);
    requestedDirection = (pc.direction < 0) ? -1 : 1;
  }

  if (doStop) {
    requestedPercent = 0.0f;
  }

  if (doClearFault) {
    clearSafetyFault();
  }

  if (ps.valid) {
    Config next = ps.value;
    validateConfig(next);
    const bool modeChanged = next.mode != cfg.mode;

    // Keep existing Hall calibration unless explicitly recalibrated.
    next.hallCalValid = cfg.hallCalValid;
    next.hallZeroElectric = cfg.hallZeroElectric;
    next.hallDirection = cfg.hallDirection;

    portENTER_CRITICAL(&stateMux);
    cfg = next;
    portEXIT_CRITICAL(&stateMux);
    saveConfig();

    if (modeChanged) {
      requestedPercent = 0.0f;
      disableMotorNow();
      scheduleRestart(600);
    } else if (motorReady) {
      applyRuntimeConfig();
    }
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
    requestedPercent = 0.0f;
    disableMotorNow();
    scheduleRestart(600);
  }

  if (doDefaults) {
    Config d = makeDefaultConfig();
    // Wi-Fi credentials live under separate keys and are intentionally kept.
    portENTER_CRITICAL(&stateMux);
    cfg = d;
    portEXIT_CRITICAL(&stateMux);
    saveConfig();
    requestedPercent = 0.0f;
    disableMotorNow();
    scheduleRestart(600);
  }

  if (doRestart) {
    requestedPercent = 0.0f;
    disableMotorNow();
    scheduleRestart(400);
  }
}

// ============================================================================
// 13. TELEMETRY
// ============================================================================

void updateTelemetry() {
  static uint32_t lastMs = 0;
  const uint32_t now = millis();
  if (now - lastMs < 200) return;
  lastMs = now;

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
  t.hallTransitions = hallTransitions;
  t.hallRpm = safetyHallRpm;
  t.hallWatchdogEnabled = hallWatchdogEnabled;
  t.hallQuietMs = motorEnabled ? (millis() - lastHallActivityMs) : 0;
  t.faultLatched = safetyFaultLatched;
  t.retryPending = retryPolicy.pending;
  t.retryAttempts = retryPolicy.attempts;
  t.retryRemainingMs = retryPolicy.remainingMs(now);

  // Only Hall mode has a real measured rotor RPM.
  // OPEN_CURRENT_FOC deliberately has no rotor feedback.
  if (cfg.mode == HALL_CURRENT_FOC && motorReady) {
    t.measuredRpm = radToRpm(hall.getVelocity());
    t.measuredRpmValid = true;
  } else {
    t.measuredRpm = 0.0f;
    t.measuredRpmValid = false;
    t.hallTransitions = 0;
    t.hallRpm = 0.0f;
    t.hallWatchdogEnabled = false;
    t.hallQuietMs = 0;
  }

  if (motorReady && motorEnabled) {
    // SimpleFOC already maintains filtered D/Q currents during foc_current.
    t.iqA = motor.current.q;
    t.idA = motor.current.d;
  } else {
    t.iqA = 0.0f;
    t.idA = 0.0f;
  }

  t.staConnected = WiFi.status() == WL_CONNECTED;
  t.wifiRssi = t.staConnected ? WiFi.RSSI() : 0;

  String staIp = t.staConnected ? WiFi.localIP().toString() : "-";
  String apIp = WiFi.softAPIP().toString();
  String ssid = t.staConnected ? WiFi.SSID() : "";

  strncpy(t.staIp, staIp.c_str(), sizeof(t.staIp) - 1);
  strncpy(t.apIp, apIp.c_str(), sizeof(t.apIp) - 1);
  strncpy(t.staSsid, ssid.c_str(), sizeof(t.staSsid) - 1);
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
    // Do not wait for a saved home network. The recovery AP/server must be
    // reachable immediately even when that network is absent.
    Serial.println("STA connection continues in background; AP remains available.");
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
  <div><div class="label">Fault latch</div><div id="faultLatch" class="value">CLEAR</div></div>
 </div>
 <div class="note" style="margin-top:10px">In OPEN mode Hall is completely unused: no Hall init, reads, interrupts, RPM feedback or watchdog. OPEN mode keeps real current feedback only. Hall stall/overload protection is available only in HALL CURRENT FOC.</div>
</div>

<div class="card section">
 <h2>Motor settings</h2>
 <div class="formgrid">
  <div class="field"><label>Mode</label><select id="mode"><option value="0">Open-loop + real current FOC</option><option value="1">Hall closed-loop + current FOC</option></select></div>
  <div class="field"><label>Max RPM</label><input id="maxRpm" type="number" step="10"></div>
  <div class="field"><label>Minimum running RPM</label><input id="minRpm" type="number" step="10"></div>
  <div class="field"><label>Acceleration RPM/s</label><input id="accel" type="number" step="10"></div>
  <div class="field"><label>Current limit A (hard max 2 A)</label><input id="currentLimit" type="number" step="0.05"></div>
  <div class="field"><label>Motor voltage limit V (hard max 6 V)</label><input id="voltageLimit" type="number" step="0.1"></div>
  <div class="field"><label>Hall alignment voltage V</label><input id="alignVoltage" type="number" step="0.05"></div>
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
 <div class="note" style="margin-top:10px">Changing control mode restarts the ESP32. Hall recalibration also restarts and may move the motor briefly during FOC alignment. The fan boots without a run command, but electrical alignment can move it briefly.</div>
 <div id="settingsMsg" class="msg"></div>
</div>

<div class="card section">
 <h2>Home Wi-Fi (optional)</h2>
 <div class="formgrid">
  <div class="field"><label>SSID</label><input id="ssid" list="wifiList" maxlength="32" autocomplete="off"><datalist id="wifiList"></datalist></div>
  <div class="field"><label>Password</label><input id="pass" type="password" maxlength="64" autocomplete="new-password"></div>
 </div>
 <div class="row" style="margin-top:12px">
  <button onclick="scanWifi()">Scan 2.4 GHz networks</button>
  <button class="primary" onclick="saveWifi()">Save Wi-Fi & restart</button>
 
 </div>
 <div id="scanMsg" class="msg"></div>
 <div class="note" style="margin-top:10px">Wi-Fi scan is allowed only while the motor is fully stopped. Recovery AP is always available: <b>MaxxFan-Setup</b> / <b>MaxxFan123</b>, normally at <b>192.168.4.1</b>. You can also try <b>maxxfan.local</b>. The saved home password is never returned by the status API.</div>
 <div id="wifiMsg" class="msg"></div>
</div>

<div class="card section note">
 <b>Current mode note:</b> OPEN mode has real phase-current feedback, but no rotor-angle feedback. It is therefore current-closed-loop / angle-open-loop. HALL mode closes both rotor velocity/angle information and current control.<br><br>
 <span id="extra"></span>
</div>
</div>
<script>
let dir=1, first=true, timer=null, refreshTimer=null;
const $=id=>document.getElementById(id);
function enc(o){return Object.entries(o).map(([k,v])=>encodeURIComponent(k)+'='+encodeURIComponent(v)).join('&')}
async function post(url,obj){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-MaxxFan-Control':'1'},body:enc(obj)});return await r.text()}
function stopRefresh(){if(refreshTimer!==null){clearTimeout(refreshTimer);refreshTimer=null}}
function scheduleRefresh(ms){stopRefresh();if(ms>0)refreshTimer=setTimeout(refresh,ms)}
function setDir(d){dir=d;$('fwd').classList.toggle('active',d===1);$('rev').classList.toggle('active',d===-1);sendControl()}
async function sendControl(){stopRefresh();await post('/api/control',{speed:$('speed').value,dir});}
$('speed').addEventListener('input',()=>{$('speedText').textContent=$('speed').value;clearTimeout(timer);timer=setTimeout(sendControl,120)});
async function stopNow(){clearTimeout(timer);stopRefresh();$('speed').value=0;$('speedText').textContent=0;await post('/api/stop',{});$('ctrlMsg').textContent='Driver disabled; fan coasting';refresh()}
async function clearFaultNow(){clearTimeout(timer);stopRefresh();$('speed').value=0;$('speedText').textContent=0;$('ctrlMsg').textContent=await post('/api/clear-fault',{});refresh()}
function f(n,d=3){return Number(n).toFixed(d)}
async function refresh(){
 try{
  const s=await (await fetch('/api/status',{cache:'no-store'})).json();
  $('modeCard').textContent=s.mode===1?'HALL CURRENT FOC':'OPEN CURRENT FOC';
  $('motorCard').textContent=!s.ready?'FAULT':s.retry_pending?'RETRY IN '+Math.ceil(s.retry_remaining_ms/1000)+' s':s.fault_latched?'FAULT LATCHED':s.reversing?'WAITING FOR ROTOR':s.enabled?'RUNNING':'STOPPED';
  $('cmdRpm').textContent=f(s.commanded_rpm,0)+' rpm';
  $('measRpm').textContent=s.measured_valid?f(s.measured_rpm,0)+' rpm':'open-loop';
  $('curr').textContent=f(s.iq,2)+' / '+f(s.id,2)+' A';
  $('wifi').textContent=s.sta_connected?(s.sta_ip+' · '+s.rssi+' dBm'):'AP '+s.ap_ip;
  $('fault').textContent=(s.fault||'')+(s.retry_pending?' — automatic retry '+(s.retry_attempts+1)+'/3; STOP cancels':'');
  $('clearFaultBtn').style.display=s.fault_latched?'inline-block':'none';
  $('hallWatch').textContent=s.hall_watchdog?'ACTIVE':'OFF';
  $('hallRpm').textContent=f(s.hall_rpm,0)+' rpm';
  $('hallQuiet').textContent=s.hall_quiet_ms+' ms';
  $('faultLatch').textContent=s.fault_latched?'LATCHED':'CLEAR';
  $('extra').textContent='Hall transitions: '+s.hall_transitions+' · current limit '+f(s.current_limit,2)+' A · voltage limit '+f(s.voltage_limit,2)+' V';
  if(first){
   dir=s.requested_dir<0?-1:1;setDirUI();
   $('speed').value=Math.round(s.requested_percent);$('speedText').textContent=Math.round(s.requested_percent);
   const c=s.config;
   $('mode').value=c.mode;$('maxRpm').value=c.max_rpm;$('minRpm').value=c.min_rpm;$('accel').value=c.accel_rpm_s;
   $('currentLimit').value=c.current_limit;$('voltageLimit').value=c.voltage_limit;$('alignVoltage').value=c.align_voltage;
   $('currentP').value=c.current_p;$('currentI').value=c.current_i;$('currentTf').value=c.current_tf;
   $('velocityP').value=c.velocity_p;$('velocityI').value=c.velocity_i;$('velocityTf').value=c.velocity_tf;
   $('rpmScale').textContent='max '+f(c.max_rpm,0)+' rpm';
   first=false;
  }
  // Do not continuously query the ESP32 while PWM is active: on a small supply
  // or a busy Wi-Fi task, periodic HTTP/heap activity can modulate the motor.
  // The screen remains live while stopped and refreshes after an explicit STOP.
  if(!s.enabled && s.requested_percent<=0.1 && !document.hidden) scheduleRefresh(2000);
 }catch(e){$('fault').textContent='GUI connection lost — motor state unknown; reconnect to MaxxFan-Setup and open 192.168.4.1';if(!document.hidden)scheduleRefresh(2000)}
}
function setDirUI(){$('fwd').classList.toggle('active',dir===1);$('rev').classList.toggle('active',dir===-1)}
async function saveSettings(){
 const obj={mode:$('mode').value,maxRpm:$('maxRpm').value,minRpm:$('minRpm').value,accel:$('accel').value,currentLimit:$('currentLimit').value,voltageLimit:$('voltageLimit').value,alignVoltage:$('alignVoltage').value,currentP:$('currentP').value,currentI:$('currentI').value,currentTf:$('currentTf').value,velocityP:$('velocityP').value,velocityI:$('velocityI').value,velocityTf:$('velocityTf').value};
 $('settingsMsg').textContent=await post('/api/settings',obj);setTimeout(()=>location.reload(),900)
}
async function recalHall(){if(confirm('Clear saved Hall calibration and restart? The motor may move during calibration.')){$('settingsMsg').textContent=await post('/api/recal',{});setTimeout(()=>location.reload(),1500)}}
async function resetDefaults(){if(confirm('Restore safe motor defaults? Wi-Fi credentials are kept.')){$('settingsMsg').textContent=await post('/api/defaults',{});setTimeout(()=>location.reload(),1200)}}
async function restartEsp(){$('settingsMsg').textContent=await post('/api/restart',{});setTimeout(()=>location.reload(),1200)}
async function scanWifi(){
 $('scanMsg').textContent='Scanning 2.4 GHz networks...';
 try{
  const r=await fetch('/api/wifi-scan',{cache:'no-store'});
  if(!r.ok){$('scanMsg').textContent=await r.text();return}
  const nets=await r.json();
  const dl=$('wifiList');dl.innerHTML='';
  const seen=new Set();
  nets.sort((a,b)=>b.rssi-a.rssi);
  let count=0;
  for(const n of nets){
   if(!n.ssid || seen.has(n.ssid)) continue;
   seen.add(n.ssid);
   const o=document.createElement('option');
   o.value=n.ssid;
   o.label=n.rssi+' dBm'+(n.secure?' - secured':' - open');
   dl.appendChild(o);count++;
  }
  $('scanMsg').textContent=count?count+' network(s) found. Click the SSID field to choose.':'No visible network found.';
 }catch(e){$('scanMsg').textContent='Wi-Fi scan failed'}
}
async function saveWifi(){$('wifiMsg').textContent=await post('/api/wifi',{ssid:$('ssid').value,pass:$('pass').value});setTimeout(()=>location.reload(),1800)}
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopRefresh();else refresh()});
refresh();
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
void setupWebServer() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    request->send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/api/status", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    const Telemetry t = telemetrySnapshot();
    const Config c = configSnapshot();

    String j;
    j.reserve(1050);
    j += "{";
    j += "\"ready\":" + String(t.motorReady ? "true" : "false");
    j += ",\"enabled\":" + String(t.enabled ? "true" : "false");
    j += ",\"reversing\":" + String(t.reversing ? "true" : "false");
    j += ",\"mode\":" + String((int)t.mode);
    j += ",\"requested_percent\":" + String(t.requestedPercent, 1);
    j += ",\"requested_dir\":" + String((int)t.requestedDirection);
    j += ",\"commanded_rpm\":" + String(t.commandedRpm, 1);
    j += ",\"measured_valid\":" + String(t.measuredRpmValid ? "true" : "false");
    j += ",\"measured_rpm\":" + String(t.measuredRpm, 1);
    j += ",\"iq\":" + String(t.iqA, 3);
    j += ",\"id\":" + String(t.idA, 3);
    j += ",\"current_limit\":" + String(t.currentLimitA, 3);
    j += ",\"voltage_limit\":" + String(t.voltageLimitV, 3);
    j += ",\"hall_transitions\":" + String(t.hallTransitions);
    j += ",\"hall_rpm\":" + String(t.hallRpm, 1);
    j += ",\"hall_watchdog\":" + String(t.hallWatchdogEnabled ? "true" : "false");
    j += ",\"hall_quiet_ms\":" + String(t.hallQuietMs);
    j += ",\"fault_latched\":" + String(t.faultLatched ? "true" : "false");
    j += ",\"retry_pending\":" + String(t.retryPending ? "true" : "false");
    j += ",\"retry_attempts\":" + String(t.retryAttempts);
    j += ",\"retry_remaining_ms\":" + String(t.retryRemainingMs);
    j += ",\"sta_connected\":" + String(t.staConnected ? "true" : "false");
    j += ",\"rssi\":" + String(t.wifiRssi);
    j += ",\"sta_ip\":\"" + jsonEscape(t.staIp) + "\"";
    j += ",\"ap_ip\":\"" + jsonEscape(t.apIp) + "\"";
    j += ",\"ssid\":\"" + jsonEscape(t.staSsid) + "\"";
    j += ",\"fault\":\"" + jsonEscape(t.fault) + "\"";
    j += ",\"config\":{";
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
    j += "}}";

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
      request->send(200, "text/plain", "Stop requested; automatic retry cancelled.");
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

    portENTER_CRITICAL(&stateMux);
    pendingControl = pc;
    portEXIT_CRITICAL(&stateMux);
    request->send(200, "text/plain", "OK");
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

    request->send(200, "text/plain", "Settings accepted. Mode changes reboot automatically.");
  });

  server.on("/api/wifi-scan", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (!authorizeRequest(request)) return;
    const Telemetry t = telemetrySnapshot();

    // Synchronous radio scanning is permitted only with the motor fully off.
    if (t.enabled || t.reversing || t.requestedPercent > 0.1f ||
        scheduledRestartAtMs != 0) {
      request->send(409, "text/plain",
                    "Stop the motor completely before scanning Wi-Fi.");
      return;
    }

    const int count = WiFi.scanNetworks(false, true);
    if (count < 0) {
      request->send(500, "text/plain", "Wi-Fi scan failed.");
      return;
    }

    String json;
    json.reserve(32 + count * 72);
    json += "[";

    for (int i = 0; i < count; ++i) {
      if (i) json += ",";
      json += "{\"ssid\":\"";
      json += jsonEscape(WiFi.SSID(i).c_str());
      json += "\",\"rssi\":";
      json += String(WiFi.RSSI(i));
      json += ",\"secure\":";
      json += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "false" : "true";
      json += "}";
    }

    json += "]";
    WiFi.scanDelete();

    AsyncWebServerResponse* response =
      request->beginResponse(200, "application/json", json);
    response->addHeader("Cache-Control", "no-store");
    request->send(response);
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
  Serial.printf("Hall=%.0frpm quiet=%lums watchdog=%d Iq=%.3fA Id=%.3fA latched=%d fault=%s\n",
                t.hallRpm, (unsigned long)t.hallQuietMs, t.hallWatchdogEnabled,
                t.iqA, t.idA, t.faultLatched, t.fault);
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
        Serial.println("Stop requested");
      } else if (s == "clear") {
        clearSafetyFault();
      } else if (s == "mode open") {
        retryPolicy.cancel();
        requestedPercent = 0.0f;
        disableMotorNow();
        cfg.mode = OPEN_CURRENT_FOC;
        saveConfig();
        Serial.println("Mode saved: OPEN_CURRENT_FOC. Restarting...");
        scheduleRestart(500);
      } else if (s == "mode hall") {
        retryPolicy.cancel();
        requestedPercent = 0.0f;
        disableMotorNow();
        cfg.mode = HALL_CURRENT_FOC;
        saveConfig();
        Serial.println("Mode saved: HALL_CURRENT_FOC. Restarting...");
        scheduleRestart(500);
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
        Serial.println("Commands: status | mode open | mode hall | s 0..100 | f | r | stop | clear");
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
  Serial.println("Required library: SimpleFOC 2.4.0");

  if (!prefs.begin("maxxfan", false)) {
    Serial.println("FATAL: NVS unavailable; driver disabled");
    while (true) delay(1000);
  }
  loadConfig();

  Serial.print("Control mode: ");
  Serial.println(cfg.mode == HALL_CURRENT_FOC ? "HALL_CURRENT_FOC" : "OPEN_CURRENT_FOC");

  // Clear an old experimental safety latch so a previous WINDMILL test cannot
  // prevent this commissioning build from initializing.
  if (!ENABLE_EXPERIMENTAL_MOTION_SAFETY) prefs.putBool("safetytrip", false);
  const bool persistedFault = ENABLE_EXPERIMENTAL_MOTION_SAFETY &&
                             prefs.getBool("safetytrip", false);

  // Start connectivity before any driver/current-sense/FOC initialization.
  // If motor setup fails or takes time, MaxxFan-Setup remains reachable for
  // diagnosis instead of leaving the browser with a connection timeout.
  publishedConfig = cfg;
  setupWiFi();
  setupWebServer();
  updateTelemetry();

  motorReady = setupMotor();
  if (!motorReady) disableMotorNow();
  if (persistedFault) {
    safetyFaultLatched = true;
    setFault("PREVIOUS_FAULT: inspect hardware, then clear manually");
  }
  publishedConfig = cfg;
  updateTelemetry();

  Serial.println("GUI recovery AP: MaxxFan-Setup / MaxxFan123");
  Serial.println("GUI URL: http://192.168.4.1  or  http://maxxfan.local");
  Serial.println("Open: http://192.168.4.1");
  Serial.println("OPEN mode: Hall completely OFF; physical current feedback only.");
  Serial.println("Experimental motion safety: DISABLED for commissioning.");
  Serial.println("Serial commands: status | mode open | mode hall | s 0..100 | f | r | stop | clear");
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
    disableMotorNow();
    delay(20);
    ESP.restart();
  }
}
