#!/usr/bin/env python3
import math
from pathlib import Path

HARD = 0.60
MIN_I = 0.22
MIN_RPM = 120.0
MAX_RPM = 1200.0
EXP = 2.0

def smootherstep(x):
    x = min(1.0, max(0.0, x))
    return x*x*x*(x*(x*6.0-15.0)+10.0)

def iq_for_rpm(rpm):
    rpm = abs(rpm)
    if rpm <= 0.01:
        return 0.0
    x = min(1.0, rpm / MAX_RPM)
    low = min(1.0, rpm / MIN_RPM)
    return min(HARD, MIN_I*smootherstep(low) + (HARD-MIN_I)*(x**EXP))

vals = [iq_for_rpm(r) for r in range(0, 1201, 10)]
assert vals[0] == 0.0
assert all(a <= b + 1e-9 for a,b in zip(vals, vals[1:])), "profile must be monotonic"
assert abs(iq_for_rpm(MAX_RPM)-HARD) < 1e-6
assert 0.20 <= iq_for_rpm(MIN_RPM) <= 0.24
assert iq_for_rpm(600) < HARD
assert iq_for_rpm(300) < iq_for_rpm(600) < iq_for_rpm(1200)

src = Path(__file__).parents[1] / 'firmware/MaxxFan_MKS_ESP32_FOC_Mega/MaxxFan_MKS_ESP32_FOC_Mega.ino'
s = src.read_text()
required = [
    '0.4.0-DYNAMIC-CURRENT-EXPERIMENTAL',
    'dynamicOpenCurrentForRpm',
    'dynamicOpenCurrentRequest',
    'openIqRequestA',
    'dynamic_current_enabled',
    'startup_boost_current',
    'currentLimitA remains the hard ceiling',
    'c.mode = OPEN_CURRENT_FOC',
    'SKIP_CURRENT_SENSE_ALIGNMENT = false',
]
for token in required:
    assert token in s, token
assert 'motor.current_limit = clampf(openIqRequestA, 0.0f, cfg.currentLimitA);' in s

# Source must preserve a current floor during continuous OPEN reversal, while
# STOP remains handled by the independent current fade-out path.
assert 'reversalPending && requestedPercent > 0.01f' in s
print('dynamic-current tests: PASS')
