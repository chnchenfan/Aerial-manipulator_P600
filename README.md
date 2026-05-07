# Aerial Manipulator P600 PX4 ESO Firmware

This repository is a PX4-based firmware branch for the P600 aerial manipulator platform. It adds an ESO-based nonlinear multicopter control stack to the Codev DP1000/P600 firmware while keeping the original Codev multicopter controllers available as a fallback.

The current implementation has generated a DP1000 firmware image and has completed successful P600 real-flight tests. The vehicle can fly with the ESO controller stack, including low-altitude flight and position-hold tests. Further parameter tuning is still needed to improve real-flight hovering and position-hold precision. Manipulator dynamics feed-forward compensation is not enabled yet.

## Project Structure

```text
.
+-- src/modules/eso_pos_control
|   +-- ESOPositionControl/
|   +-- ESOTakeoff/
|   +-- eso_pos_control_params.c
|   +-- ESOMulticopterPositionControl.cpp
|   +-- ESOMulticopterPositionControl.hpp
|
+-- src/modules/eso_att_control
|   +-- ESOAttitudeControl/
|   +-- ArmKinematics.hpp
|   +-- eso_att_control_params.c
|   +-- eso_att_control_main.cpp
|   +-- eso_att_control.hpp
|
+-- src/modules/eso_rate_control
|   +-- ESORateControl/
|   +-- eso_rate_control_params.c
|   +-- ESOMulticopterRateControl.cpp
|   +-- ESOMulticopterRateControl.hpp
|
+-- src/modules/eso_common
|   +-- ESOModelProfile.hpp
|
+-- src/modules/arm_joint_bridge
|   +-- ArmJointBridge.cpp
|   +-- ArmJointBridge.hpp
|
+-- msg
|   +-- arm_joint_states.msg
|   +-- eso_rate_aux.msg
|
+-- ROMFS/px4fmu_common/init.d/airframes
|   +-- 4066_codev_dp_1000_eso
|
+-- boards/codev/dp1000-v2/default.cmake
```

The ESO controller chain is split into position, attitude, and rate modules. The `arm_joint_bridge` module is reserved for receiving manipulator joint feedback and publishing `arm_joint_states`, but dynamics feed-forward compensation is intentionally disabled in the current flight-tested configuration.

## Workflow and Usage

Build the DP1000 firmware:

```bash
cd ~/Program/code/Codev-autopilot/Codev-autopilot
make codev_dp1000-v2_default
```

The generated firmware image is:

```text
build/codev_dp1000-v2_default/codev_dp1000-v2_default.px4
```

Flash this file through QGroundControl as a custom PX4 firmware image.

Use the ESO airframe:

```sh
param set SYS_AUTOSTART 4066
param save
reboot
```

Expected ESO controller status after reboot:

```sh
eso_rate_control status
eso_att_control status
eso_pos_control status
arm_joint_bridge status
mc_rate_control status
mc_att_control status
mc_pos_control status
```

For the ESO airframe, `eso_rate_control`, `eso_att_control`, and `eso_pos_control` should be running, while the stock `mc_*` controllers should not be running. The original Codev DP1000 airframe remains available through `SYS_AUTOSTART=4065`.

## Controller Notes

The added ESO stack includes:

- ESO-based position control for multicopter position/velocity tracking.
- ESO-based attitude control and geometric attitude reference handling.
- ESO-based rate control with observer and integral protections.
- Ground-contact and low-throttle protection for the rate observer to avoid ground-test windup.
- Optional manipulator-related inputs through `arm_joint_bridge`.

The current real-flight configuration keeps manipulator dynamics feed-forward disabled:

```sh
ESO_DYN_FF_EN = 0
ESO_TAUS_K = 0
```

This means the aircraft is currently flying with the ESO control stack, but without active manipulator dynamics compensation.

## Validation Status

The firmware has been tested on the P600 platform with a DP1000 flight controller. Completed checks include:

- Firmware build for `codev_dp1000-v2_default`.
- QGroundControl flashing of the generated `.px4` image.
- ESO airframe boot with `SYS_AUTOSTART=4066`.
- RC input, attitude direction, motor order, ESC status, and actuator output checks.
- Low-altitude real-flight validation.
- Position-hold validation using external motion-capture localization.

Current status:

- P600 real flight is successful with the ESO controller stack.
- Position-hold works but still needs parameter tuning for higher precision.
- Manipulator motion tests are limited by current stepper-motor hardware issues.
- Manipulator dynamics feed-forward compensation has not been enabled in real flight.

## Future Work

- Tune real-flight position-hold and altitude-hold parameters for higher precision.
- Repair and validate the manipulator stepper-motor hardware.
- Record repeatable datasets with the manipulator disabled, enabled but static, and moving.
- Verify `arm_joint_bridge` validity during real manipulator operation.
- Enable dynamics feed-forward gradually after the manipulator feedback path is stable.
- Compare flight logs before and after dynamics feed-forward compensation.

## Notes

- This repository is based on PX4/Codev firmware and targets the DP1000/P600 hardware configuration.
- The stock Codev multicopter controller path is preserved for fallback through the original airframe.
- Generated build artifacts should normally stay outside Git unless a release package is explicitly prepared.
- Flight data and detailed benchmark tables are intentionally not included in this README yet.
