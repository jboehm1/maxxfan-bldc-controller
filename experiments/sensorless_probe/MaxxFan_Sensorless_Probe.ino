/*
  ============================================================================
  MaxxFan Sensorless Flux Observer Probe
  Branch : research/sensorless-foc
  Status : PASSIVE OBSERVER ONLY - NOT CLOSED-LOOP SENSORLESS CONTROL

  Board  : MKS ESP32 FOC Mega (motor 0)
  Motor  : StepperOnline 57BYA54-12-01
  Library: SimpleFOC 2.4.x only (no extra observer library required)

  PURPOSE
  ---------------------------------------------------------------------------
  The motor is still commutated by SimpleFOC velocity_openloop + foc_current.
  A separate flux observer watches the existing two phase-current channels and
  SimpleFOC's commanded alpha/beta voltage vector.  Its estimate NEVER feeds
  back into motor commutation in this probe.

  This makes the first sensorless experiment low-risk: compare commanded RPM
  with observer-estimated RPM/angle before attempting a closed-loop handover.

  NO HALL SENSOR IS INITIALISED OR READ.
  ============================================================================
*/

#include <Arduino.h>
#include <SimpleFOC.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

// MKS ESP32 FOC Mega motor-0 mapping verified against Makerbase examples.
static constexpr int PIN_PWM_U = 32;
static constexpr int PIN_PWM_V = 33;
static constexpr int PIN_PWM_W = 25;
static constexpr int PIN_ENABLE = 12;
static constexpr int PIN_CURRENT_A = 39;
static constexpr int PIN_CURRENT_B = 36;

static constexpr int MOTOR_POLE_PAIRS = 2;
static constexpr float SUPPLY_VOLTAGE_V = 12.0f;
static constexpr float SHUNT_OHM = 0.01f;
static constexpr float AMP_GAIN = 50.0f;

// 57BYA54-12-01 published starting parameters.
// These are model parameters and MUST be experimentally validated/tuned.
static constexpr float PHASE_RESISTANCE_OHM = 0.27f;
static constexpr float PHASE_INDUCTANCE_H = 0.00057f;
static constexpr float BACK_EMF_V_PER_KRPM = 2.2f;
static constexpr float KV_RPM_PER_V = 1000.0f / BACK_EMF_V_PER_KRPM; // ~454.5

// Conservative commissioning limits, matching the main project philosophy.
static constexpr float DEFAULT_CURRENT_A = 0.60f;
static constexpr float DEFAULT_VOLTAGE_LIMIT_V = 3.0f;
static constexpr float ALIGN_VOLTAGE_V = 0.60f;
static constexpr float MAX_COMMAND_RPM = 1500.0f;
static constexpr float COMMAND_RAMP_RPM_S = 300.0f;

BLDCMotor motor(MOTOR_POLE_PAIRS,
                PHASE_RESISTANCE_OHM,
                KV_RPM_PER_V,
                PHASE_INDUCTANCE_H);
BLDCDriver3PWM driver(PIN_PWM_U, PIN_PWM_V, PIN_PWM_W, PIN_ENABLE);
InlineCurrentSense currentSense(SHUNT_OHM, AMP_GAIN, PIN_CURRENT_A, PIN_CURRENT_B);

static inline float rpmToRad(float rpm) { return rpm * (_2PI / 60.0f); }
static inline float radToRpm(float rad_s) { return rad_s * (60.0f / _2PI); }
static inline float clampfLocal(float x, float lo, float hi) {
  if (!isfinite(x)) return lo;
  return x < lo ? lo : (x > hi ? hi : x);
}
static inline float wrapPi(float a) {
  while (a > _PI) a -= _2PI;
  while (a < -_PI) a += _2PI;
  return a;
}

class PassiveFluxObserver {
 public:
  float phaseResistance = PHASE_RESISTANCE_OHM;
  float phaseInductance = PHASE_INDUCTANCE_H;
  float kvRpmPerVolt = KV_RPM_PER_V;
  int polePairs = MOTOR_POLE_PAIRS;

  float fluxAlpha = 0.0f;
  float fluxBeta = 0.0f;
  float electricalAngle = 0.0f;
  float estimatedMechanicalRpm = 0.0f;
  float fluxMagnitude = 0.0f;
  float fluxLinkage = 0.0f;
  float iAlpha = 0.0f;
  float iBeta = 0.0f;
  float dtUs = 0.0f;
  uint32_t samples = 0;

  void begin() {
    recalcFluxLinkage();
    reset();
  }

  void reset() {
    fluxAlpha = fluxBeta = 0.0f;
    electricalAngle = 0.0f;
    estimatedMechanicalRpm = 0.0f;
    fluxMagnitude = 0.0f;
    iAlphaPrev = iBetaPrev = 0.0f;
    anglePrev = 0.0f;
    rpmFiltered = 0.0f;
    lastUs = micros();
    samples = 0;
  }

  void recalcFluxLinkage() {
    // Same motor-model relationship used by the official MXLEMMING observer.
    // KV convention is therefore a calibration parameter, not an assumption
    // that should be treated as exact from the datasheet alone.
    if (kvRpmPerVolt > 0.0f && polePairs > 0) {
      fluxLinkage = 60.0f / (_SQRT3 * _PI * kvRpmPerVolt * polePairs * 2.0f);
    } else {
      fluxLinkage = 0.0f;
    }
  }

  void update() {
    if (!currentSense.initialized || fluxLinkage <= 0.0f ||
        phaseResistance <= 0.0f || phaseInductance <= 0.0f) return;

    const uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    if (dt <= 0.0f || dt > 0.02f) {
      iAlphaPrev = iBetaPrev = 0.0f;
      return;
    }
    dtUs = dt * 1e6f;

    const PhaseCurrent_s phase = currentSense.getPhaseCurrents();
    const ABCurrent_s ab = currentSense.getABCurrents(phase);
    iAlpha = ab.alpha;
    iBeta = ab.beta;

    // RL flux integration:
    //   psi += (U - R*I) dt - L dI
    // Ualpha/Ubeta are the voltage vector requested by SimpleFOC.  This probe
    // intentionally does not add phase-voltage ADC hardware.
    const float dIAlpha = iAlpha - iAlphaPrev;
    const float dIBeta = iBeta - iBetaPrev;
    fluxAlpha += (motor.Ualpha - phaseResistance * iAlpha) * dt
                 - phaseInductance * dIAlpha;
    fluxBeta  += (motor.Ubeta  - phaseResistance * iBeta) * dt
                 - phaseInductance * dIBeta;

    // Bounded integrator, following the same physical constraint used by the
    // established MXLEMMING/MESC family of flux observers.
    fluxAlpha = clampfLocal(fluxAlpha, -fluxLinkage, fluxLinkage);
    fluxBeta  = clampfLocal(fluxBeta,  -fluxLinkage, fluxLinkage);

    fluxMagnitude = sqrtf(fluxAlpha * fluxAlpha + fluxBeta * fluxBeta);
    const float newAngle = atan2f(fluxBeta, fluxAlpha);
    const float dAngle = wrapPi(newAngle - anglePrev);
    anglePrev = newAngle;
    electricalAngle = newAngle;

    const float electricalRadS = dAngle / dt;
    const float mechanicalRpm = radToRpm(electricalRadS / (float)polePairs);

    // Light diagnostic LPF only. It does not participate in motor control.
    const float tau = 0.030f;
    const float alpha = dt / (tau + dt);
    rpmFiltered += alpha * (mechanicalRpm - rpmFiltered);
    estimatedMechanicalRpm = rpmFiltered;

    iAlphaPrev = iAlpha;
    iBetaPrev = iBeta;
    ++samples;
  }

 private:
  uint32_t lastUs = 0;
  float iAlphaPrev = 0.0f;
  float iBetaPrev = 0.0f;
  float anglePrev = 0.0f;
  float rpmFiltered = 0.0f;
};

PassiveFluxObserver observer;

float targetRpm = 0.0f;
float commandedRpm = 0.0f;
uint32_t lastControlUs = 0;
uint32_t lastPrintMs = 0;
char lineBuf[96];
size_t lineLen = 0;

void hardStop() {
  targetRpm = 0.0f;
  commandedRpm = 0.0f;
  motor.target = 0.0f;
  motor.disable();
  digitalWrite(PIN_ENABLE, LOW);
  observer.reset();
  Serial.println("STOP: driver disabled");
}

void printHelp() {
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  s <rpm>       signed commanded RPM, e.g. s 600 or s -600");
  Serial.println("  stop          immediate driver disable");
  Serial.println("  status        print current observer state");
  Serial.println("  resetobs      reset observer integrator");
  Serial.println("  kv <rpm/V>    set observer KV model and reset observer");
  Serial.println("  r <ohm>       set phase R model and reset observer");
  Serial.println("  l <mH>        set phase L model and reset observer");
  Serial.println("  help");
}

void printStatus() {
  const float fluxRatio = observer.fluxLinkage > 0.0f
    ? observer.fluxMagnitude / observer.fluxLinkage : 0.0f;
  Serial.printf("cmd=%.1f rpm  obs=%.1f rpm  elec=%.3f rad  flux=%.3f xPsi  "
                "Iab=%.3f/%.3f A  Uab=%.3f/%.3f V  dt=%.0f us  N=%lu\n",
                commandedRpm, observer.estimatedMechanicalRpm,
                observer.electricalAngle, fluxRatio,
                observer.iAlpha, observer.iBeta,
                motor.Ualpha, motor.Ubeta, observer.dtUs,
                (unsigned long)observer.samples);
}

void applyModelChange() {
  observer.recalcFluxLinkage();
  observer.reset();
  Serial.printf("Observer model: R=%.4f ohm L=%.4f mH KV=%.2f rpm/V psi=%.7f Wb\n",
                observer.phaseResistance, observer.phaseInductance * 1000.0f,
                observer.kvRpmPerVolt, observer.fluxLinkage);
}

void processCommand(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (!*line) return;

  if (!strcmp(line, "help") || !strcmp(line, "?")) {
    printHelp();
    return;
  }
  if (!strcmp(line, "status")) {
    printStatus();
    return;
  }
  if (!strcmp(line, "stop")) {
    hardStop();
    return;
  }
  if (!strcmp(line, "resetobs")) {
    observer.reset();
    Serial.println("Observer reset");
    return;
  }

  char* end = nullptr;
  if (line[0] == 's' && (line[1] == ' ' || line[1] == '\t')) {
    const float value = strtof(line + 2, &end);
    if (end == line + 2 || !isfinite(value)) {
      Serial.println("Invalid RPM");
      return;
    }
    targetRpm = clampfLocal(value, -MAX_COMMAND_RPM, MAX_COMMAND_RPM);
    if (!motor.enabled && fabsf(targetRpm) > 0.01f) {
      observer.reset();
      motor.enable();
    }
    Serial.printf("Target %.1f rpm\n", targetRpm);
    return;
  }
  if (!strncmp(line, "kv ", 3)) {
    const float v = strtof(line + 3, &end);
    if (end != line + 3 && isfinite(v) && v > 50.0f && v < 5000.0f) {
      observer.kvRpmPerVolt = v;
      applyModelChange();
    } else Serial.println("Invalid KV");
    return;
  }
  if (!strncmp(line, "r ", 2)) {
    const float v = strtof(line + 2, &end);
    if (end != line + 2 && isfinite(v) && v > 0.01f && v < 10.0f) {
      observer.phaseResistance = v;
      applyModelChange();
    } else Serial.println("Invalid R");
    return;
  }
  if (!strncmp(line, "l ", 2)) {
    const float mH = strtof(line + 2, &end);
    if (end != line + 2 && isfinite(mH) && mH > 0.01f && mH < 100.0f) {
      observer.phaseInductance = mH * 0.001f;
      applyModelChange();
    } else Serial.println("Invalid L");
    return;
  }
  Serial.println("Unknown command; type help");
}

void serviceSerial() {
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[lineLen] = '\0';
      processCommand(lineBuf);
      lineLen = 0;
    } else if (lineLen + 1 < sizeof(lineBuf)) {
      lineBuf[lineLen++] = c;
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("=== MaxxFan PASSIVE SENSORLESS OBSERVER PROBE ===");
  Serial.println("Hall: NOT USED | Observer: diagnostic only");

  pinMode(PIN_PWM_U, OUTPUT);
  pinMode(PIN_PWM_V, OUTPUT);
  pinMode(PIN_PWM_W, OUTPUT);
  digitalWrite(PIN_PWM_U, LOW);
  digitalWrite(PIN_PWM_V, LOW);
  digitalWrite(PIN_PWM_W, LOW);

  driver.pwm_frequency = 25000;
  driver.voltage_power_supply = SUPPLY_VOLTAGE_V;
  driver.voltage_limit = SUPPLY_VOLTAGE_V;
  if (!driver.init()) {
    Serial.println("FATAL: driver.init failed");
    return;
  }
  motor.linkDriver(&driver);

  motor.foc_modulation = FOCModulationType::SinePWM;
  motor.controller = MotionControlType::velocity_openloop;
  motor.torque_controller = TorqueControlType::foc_current;
  motor.current_limit = DEFAULT_CURRENT_A;
  motor.voltage_limit = DEFAULT_VOLTAGE_LIMIT_V;
  motor.velocity_limit = rpmToRad(MAX_COMMAND_RPM);
  motor.voltage_sensor_align = ALIGN_VOLTAGE_V;

  // Known commissioning current-loop seeds from the MaxxFan branch.
  motor.PID_current_q.P = 0.40f;
  motor.PID_current_q.I = 180.0f;
  motor.PID_current_q.D = 0.0f;
  motor.PID_current_d.P = 0.40f;
  motor.PID_current_d.I = 180.0f;
  motor.PID_current_d.D = 0.0f;
  motor.LPF_current_q.Tf = 0.002f;
  motor.LPF_current_d.Tf = 0.002f;

  if (!motor.init()) {
    Serial.println("FATAL: motor.init failed");
    return;
  }

  currentSense.linkDriver(&driver);
  if (!currentSense.init()) {
    Serial.println("FATAL: currentSense.init failed");
    motor.disable();
    return;
  }
  currentSense.skip_align = false;
  motor.linkCurrentSense(&currentSense);

  // No rotor sensor is linked. In velocity_openloop SimpleFOC only performs
  // current-sense alignment here; it does not require Hall/encoder alignment.
  if (!motor.initFOC()) {
    Serial.println("FATAL: initFOC/current-sense alignment failed (Hall unused)");
    motor.disable();
    return;
  }

  motor.disable();
  commandedRpm = targetRpm = 0.0f;
  lastControlUs = micros();
  observer.begin();
  applyModelChange();
  printHelp();
}

void loop() {
  serviceSerial();

  const uint32_t nowUs = micros();
  float dt = (nowUs - lastControlUs) * 1e-6f;
  lastControlUs = nowUs;
  if (dt <= 0.0f || dt > 0.02f) dt = 0.001f;

  const float maxStep = COMMAND_RAMP_RPM_S * dt;
  if (commandedRpm < targetRpm) commandedRpm = min(commandedRpm + maxStep, targetRpm);
  else if (commandedRpm > targetRpm) commandedRpm = max(commandedRpm - maxStep, targetRpm);

  if (motor.enabled) {
    motor.loopFOC();
    motor.move(rpmToRad(commandedRpm));
    observer.update();
  }

  // Probe output intentionally slow. The observer itself runs every motor loop.
  const uint32_t nowMs = millis();
  if (nowMs - lastPrintMs >= 200) {
    lastPrintMs = nowMs;
    if (motor.enabled) printStatus();
  }
}
