# Hardware regression test plan

Before merging `develop` into `main`:

1. Compile on Arduino-ESP32 target used by the project.
2. Boot with Hall disconnected and verify OPEN_CURRENT_FOC.
3. Verify current-sense init/alignment.
4. Run fixed speed from Serial: no periodic tick.
5. Change speed from Serial: smooth transition.
6. Reverse from Serial at stable speed: no click/freeze.
7. Reverse from Serial during ramp: no freeze/overshoot.
8. Repeat 4-7 from GUI.
9. Compare GUI closed vs GUI open.
10. Verify Iq/Id values are plausible.
11. Check 600 / 900 / 1200 rpm at configured current/voltage limits.
12. Observe motor/driver temperature.
