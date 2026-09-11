# Changelog

## Historical baseline

### v0.3.7-TEST — known-good reference
- User-reported clean rotation on the actual MKS ESP32 FOC Mega without Hall connected in OPEN mode.
- OPEN uses `velocity_openloop + foc_current` with physical current feedback on GPIO39/GPIO36.
- Used as the hardware-reference baseline for future regression checks.

## Development line

The `develop` branch contains the newer GUI/S-curve/NVS/diagnostic work. It remains experimental until hardware requalification.
