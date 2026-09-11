# Sensorless research sources

Primary sources used for this branch:

1. SimpleFOCDrivers — MXLEMMING Observer Sensor README  
   https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/MXLEMMING_observer/README.md

2. SimpleFOCDrivers — MXLEMMINGObserverSensor implementation  
   https://github.com/simplefoc/Arduino-FOC-drivers/blob/master/src/encoders/MXLEMMING_observer/MXLEMMINGObserverSensor.cpp

3. SimpleFOC documentation — Sensorless FOC example  
   https://docs.simplefoc.com/sensorless_foc_nucleo_example

4. SimpleFOCDrivers release history — v1.0.9 adds FluxObserverSensor / sensorless FOC support  
   https://github.com/simplefoc/Arduino-FOC-drivers/releases/tag/v1.0.9

5. SimpleFOC core — BLDCMotor / FOCMotor source for Ualpha/Ubeta, open-loop and current-sense behavior  
   https://github.com/simplefoc/Arduino-FOC/blob/master/src/BLDCMotor.cpp  
   https://github.com/simplefoc/Arduino-FOC/blob/master/src/common/base_classes/FOCMotor.cpp

6. Makerbase MKS ESP32FOC examples — motor-0 current sense `InlineCurrentSense(0.01, 50.0, 39, 36)`  
   https://github.com/makerbase-motor/MKS-ESP32FOC

7. StepperOnline 57BYA54-12-01 product/datasheet values  
   https://www.omc-stepperonline.com/12v-3000rpm-0-16nm-50w-5-90a-57x57x53-5mm-brushless-dc-motor-57bya54-12-01

## Licensing note

The passive probe in this repository is a project-local implementation of the standard RL flux-integration equations for diagnostic use. It does not copy the SimpleFOCDrivers observer source verbatim. The official MXLEMMING implementation itself carries attribution requirements inherited from the MESC work; consult its source/license before vendoring or modifying that implementation directly.
