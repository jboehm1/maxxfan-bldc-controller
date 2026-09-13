// Diagnostic GUI timing isolation only.
// Keep motor-control source byte-identical to test/fixed-current-baseline.
// AsyncTCP callbacks are confined to core 0 at low priority so the Arduino
// motor loop on core 1 is not pre-empted by HTTP request processing.
-DCONFIG_ASYNC_TCP_RUNNING_CORE=0
-DCONFIG_ASYNC_TCP_PRIORITY=3
