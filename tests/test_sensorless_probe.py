#!/usr/bin/env python3
from pathlib import Path
import math

ROOT = Path(__file__).parents[1]
src = (ROOT/'experiments/sensorless_probe/MaxxFan_Sensorless_Probe.ino').read_text()

required = [
    'PASSIVE OBSERVER ONLY',
    'PIN_CURRENT_A = 39',
    'PIN_CURRENT_B = 36',
    'PHASE_RESISTANCE_OHM = 0.27f',
    'PHASE_INDUCTANCE_H = 0.00057f',
    'BACK_EMF_V_PER_KRPM = 2.2f',
    'MotionControlType::velocity_openloop',
    'TorqueControlType::foc_current',
    'currentSense.getPhaseCurrents()',
    'currentSense.getABCurrents(phase)',
    'motor.Ualpha',
    'motor.Ubeta',
    'currentSense.skip_align = false',
    'Hall: NOT USED',
]
for token in required:
    assert token in src, token

# The probe must remain passive: no observer object is linked as a motor sensor.
assert 'motor.linkSensor(&observer)' not in src
assert 'HallSensor' not in src

kv = 1000.0/2.2
pp = 2
psi = 60.0/(math.sqrt(3)*math.pi*kv*pp*2.0)
assert 0.005 < psi < 0.01, psi

# Test wrap logic mathematically at +/-pi boundary.
def wrap_pi(a):
    while a > math.pi: a -= 2*math.pi
    while a < -math.pi: a += 2*math.pi
    return a
assert abs(wrap_pi((-math.pi+0.01) - (math.pi-0.01)) - 0.02) < 1e-9
assert abs(wrap_pi((math.pi-0.01) - (-math.pi+0.01)) + 0.02) < 1e-9

print('sensorless-probe tests: PASS')
