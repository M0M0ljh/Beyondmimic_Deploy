# Beyondmimic_Deploy

[中文](README.zh.md)

> **Experimental low-level controller.** Use only on a suspended robot in a
> controlled area with an operator ready to select damping.  It is not a
> certified safety controller.

This C++17 project deploys two ONNX policies on a Unitree G1 through
`unitree_sdk2` DDS:

- **Loco**: joystick-commanded locomotion (`96` observations -> `29` actions).
- **RedRed**: a finite reference-motion tracking policy (`154` observations ->
  `29` actions).

The policy never writes DDS directly.  The state machine validates robot state,
runs the selected policy, and emits one 29-motor position/PD command.

```text
LowState + wireless remote
          ↓
UnitreeController (50 Hz control loop)
          ↓
StateMachine + SafetySupervisor
          ↓
ZeroTorque | StartupReady | Loco | RedRedCast | RedRed | Damping
          ↓
MotorCommand(q, dq, kp, kd, tau) × 29
          ↓
LowCmd DDS writer (500 Hz)
```

More implementation detail is in [FSM_ARCHITECTURE.md](FSM_ARCHITECTURE.md).

## Platform and dependencies

The executable must use an ONNX Runtime binary built for the target CPU:

| Use case | Target architecture | ONNX Runtime package |
| --- | --- | --- |
| Development / simulation | `x86_64` | `onnxruntime-linux-x64-1.22.0` |
| G1 PC2 real robot | `aarch64` | `onnxruntime-linux-aarch64-1.22.0` |

Install these dependencies for the same architecture as the executable:

- CMake 3.16 or newer and a C++17 compiler;
- `unitree_sdk2`;
- `yaml-cpp` and Eigen3;
- ONNX Runtime 1.22.0 CPU package.

Do not commit an architecture-specific ONNX Runtime distribution.  Installation
layout and the `ONNXRUNTIME_ROOT` override are documented in
[thirdparty/README.md](thirdparty/README.md).

## Build

On a G1 PC2 (native aarch64 build), after installing the aarch64 ONNX Runtime
package:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.22.0
cmake --build build -j
```

On an x86_64 development host, use the matching x64 package in the same way.
CMake chooses `thirdparty/onnxruntime-linux-<arch>-1.22.0` by default when that
directory exists.  The old `deploy_real.py` command is no longer part of this
project.

## Run on the robot

The robot must be in Unitree's low-level/debug-ready mode before launching the
controller.  Connect the configured network interface and run:

```bash
./build/Beyondmimic_Deploy eth0
```

Replace `eth0` with the NIC connected to the robot.  Startup begins in
`ZeroTorque`; select `StartupReady` and wait for its two-second pose transition
before selecting `Loco`.

## Controls

The Unitree wireless remote is decoded from `LowState.wireless_remote`.

| Action | Wireless remote | Keyboard (development only) |
| --- | --- | --- |
| Zero torque | `L1 + Start` | `0` |
| Startup pose | `Start` | `1` |
| Loco | `R1 + A` | `2` |
| RedRed cast / motion | `R1 + X` while in Loco | `4` while in Loco |
| Damping | `F1` or `Select` | `3` |
| Damping then exit | — | `Q` |
| Velocity command | sticks | `W/S` forward/back, `A/D` yaw |

After entering Loco, the wireless remote sticks continuously provide velocity
commands to the policy.  `R1` is used only in the `R1 + A` and `R1 + X` mode
switch combinations; use `F1` or `Select` to enter Damping.

Planar stick inputs use a `0.1` dead zone and are smoothed as
`0.95 × previous + 0.05 × current`; yaw is not smoothed.  Both parameters are
configured in `configs/loco_mode/g1.yaml`.

## Safety behavior and current limits

The controller discards LowState frames with an invalid CRC.  The state machine
enters `Damping` when LowState is unavailable or older than 100 ms,
state/command values are non-finite, the robot is inverted during an active
mode, or policy inference throws.  Damping sends zero position/torque with
`kd = 8`.

This first public version does **not** yet enforce joint limits, command slew
limits, or the `torque_limits` values stored in the RedRed YAML.  Do not treat
those YAML values as active protection.  Add and validate a command limiter
before unsuspended or disturbance-heavy use.

## Configuration and assets

- [configs/loco_mode/g1.yaml](configs/loco_mode/g1.yaml): Loco observation,
  joint map, PD gains, and velocity limits.
- [configs/redred/g1.yaml](configs/redred/g1.yaml): RedRed tensor contract,
  reference-motion settings, joint map, and PD gains.
- `model/g1/policy96.onnx` and `model/g1/redred.onnx`: policy assets.

Policy assets are ignored by default pending confirmation of their redistribution
rights.  See [model/README.md](model/README.md) before publishing them.

## Repository layout

```text
app/                 executable entry point
core/                state machine, state types, safety checks
hardware/unitree/    Unitree DDS and wireless remote adapter
inference/           validated ONNX Runtime wrapper
states/              basic, startup, Loco, and RedRed states
configs/             policy-specific YAML
thirdparty/          local dependency instructions (not runtime binaries)
```

## Motion demonstration

The animation below is cropped from the local `beyondmimic.mp4` demonstration,
starting at the first clean dance frame (about 4.0 s) and continuing through the
end of the routine.

<p align="center">
  <img src="assets/beyondmimic-dance.gif" alt="G1 RedRed dance demonstration" width="360">
</p>

## Copyright, attribution, and licensing

This repository is an independent C++ deployment implementation.  It includes
original work as well as material adapted from the projects below.  Third-party
code, models, and motion data do **not** automatically share one license.
Detailed notices are collected in [NOTICE.md](NOTICE.md).

- **Unitree RL Gym** — parts of the deployment workflow and compatibility
  configuration were adapted from
  [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym).
  That project is copyright © 2016–2023 HangZhou YuShu TECHNOLOGY CO., LTD.
  (Unitree Robotics) and is licensed under BSD 3-Clause.  Its full license is
  retained in [LICENSES/unitree_rl_gym-BSD-3-Clause.txt](LICENSES/unitree_rl_gym-BSD-3-Clause.txt).
- **RoboMimic Deploy** — the multi-policy deployment architecture and operating
  workflow were studied with
  [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy).
  Its repository does not currently declare a license.  This acknowledgement is
  not permission to copy or redistribute its source code, policy files, motion
  data, or robot assets.  Before publishing, remove/reimplement any such
  material or obtain written permission from its copyright holder.
- **BeyondMimic** — the motion-tracking training formulation was referenced from
  [HybridRobotics/whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking)
  and the BeyondMimic paper.  The upstream training code is MIT-licensed.  This
  deployment repository does not vendor that training code; if it is added in
  the future, its copyright and MIT license must accompany it.

The ONNX policies and motion assets are separate deliverables.  They remain
excluded until their provenance, training-data terms, and redistribution rights
are documented in [model/README.md](model/README.md).
