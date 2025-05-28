# FluxGrip: demagnetization cycle optimization

This application is meant to be used to tune the demagnetization cycle of FluxGrip.

It consists of the following parts:

- _ForceMeasurementClient_: reads out measurements from _firmware_force_sensor_
- _StepperDriveClient_: controls the stepper driver (through _firmware_stepper_drive_) which moves the "arm" of the force measurement rig up/down
- _ConfigClient_: this clients allows to update the demagnetization cycle values on the FluxGrip (using Cyphal/CAN)

These 3 clients are used by _Optimizer_ to obtain the optimal demagnetization values.

## Installation

```shell
pip install -r requirements.txt
src/optimizer.py
```