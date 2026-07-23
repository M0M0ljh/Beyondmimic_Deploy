# Beyondmimic_Deploy

C++17 deployment controller for Unitree G1 locomotion and reference-motion
tracking.  It runs ONNX policies through `unitree_sdk2` DDS and controls the
robot through a finite-state machine.

[中文](README.zh.md)

> **Experimental low-level controller.** Operate only in a controlled area with
> the robot suspended and an operator ready to select Damping.  This is not a
> certified safety controller.

## Features

- **Finite-state control** — `ZeroTorque`, `StartupReady`, `Loco`,
  `RedRedCast`, `RedRed`, and `Damping` make policy transitions explicit.
- **Two ONNX policies** — joystick-commanded Loco (`96` observations → `29`
  actions) and RedRed reference-motion tracking (`154` observations → `29`
  actions).
- **Unitree wireless remote** — on the physical robot, commands come directly
  from `LowState.wireless_remote`.  In MuJoCo, Unitree's simulator can emulate
  the same packet from an Xbox or Switch gamepad.
- **ARM64-ready build** — CMake selects the matching x64 or aarch64 ONNX
  Runtime package; aarch64 is the native target for G1 PC2.
- **Runtime checks** — invalid LowState CRC, stale state, non-finite values,
  inversion during active control, and inference errors switch the controller
  to `Damping`.

## Prerequisites

| Target | Architecture | ONNX Runtime package |
| --- | --- | --- |
| Development host | `x86_64` | `onnxruntime-linux-x64-1.22.0` |
| G1 PC2 real robot | `aarch64` | `onnxruntime-linux-aarch64-1.22.0` |

- CMake 3.16 or newer;
- a C++17 compiler;
- `unitree_sdk2`, `yaml-cpp`, and Eigen3;
- ONNX Runtime 1.22.0 CPU package for the target architecture.

### Installing ONNX Runtime

Download the package for the machine that will run the executable.  For example,
on G1 PC2:

```bash
mkdir -p thirdparty
cd thirdparty
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

On an x86_64 development host, replace `aarch64` with `x64`.  The extracted
directory is ignored by Git.  An installation outside this repository is also
supported through `ONNXRUNTIME_ROOT`; see [thirdparty/README.md](thirdparty/README.md).

## Building

Build on the target architecture.  With ONNX Runtime installed under
`thirdparty/`, CMake chooses it automatically:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

For an installation elsewhere, pass its directory explicitly:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.22.0
cmake --build build -j
```

The resulting executable is `build/Beyondmimic_Deploy`.

## Configuration

| File | Purpose |
| --- | --- |
| [configs/loco_mode/g1.yaml](configs/loco_mode/g1.yaml) | Loco policy path, observation settings, joint map, PD gains, velocity limits, and planar-command filter. |
| [configs/redred/g1.yaml](configs/redred/g1.yaml) | RedRed ONNX tensor contract, transition duration, reference-motion settings, joint map, and PD gains. |
| `model/g1/policy96.onnx` | Loco policy expected by the default configuration. |
| `model/g1/redred.onnx` | RedRed policy expected by the default configuration. |

The policy binaries are intentionally ignored.  Place locally validated models
at the paths above before running; see [model/README.md](model/README.md) for
the expected provenance and tensor-contract record.

## Deploy on MuJoCo Simulation

This controller uses the same Unitree SDK2 DDS `LowState` and `LowCmd` path in
simulation as on the real robot.  It works with the C++ simulator in
[unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco).

1. Install and build Unitree MuJoCo following its official instructions.
2. In `unitree_mujoco/simulate/config.yaml`, select the G1 scene and match the
   controller's simulation DDS settings:

   ```yaml
   robot: "g1"
   robot_scene: "scene_29dof.xml"
   domain_id: 1
   interface: "lo"
   use_joystick: 1
   joystick_type: "xbox"
   enable_elastic_band: 1
   ```

   `unitree_mujoco` uses the gamepad only to emulate Unitree's wireless remote
   packet.  The controller still consumes `LowState.wireless_remote`.
3. Start the simulator first:

   ```bash
   cd unitree_mujoco/simulate/build
   ./unitree_mujoco
   ```

4. In another terminal, start this controller with no interface argument:

   ```bash
   ./build/Beyondmimic_Deploy
   ```

   No argument selects DDS domain `1` and the loopback interface `lo`.

## Running on the Real Robot

1. Put the G1 in Unitree low-level/debug-ready mode and suspend the robot.
2. Copy the ONNX policy files to `model/g1/`.
3. Build the project on PC2 using the aarch64 ONNX Runtime package.
4. Start the controller with the network interface connected to the robot:

   ```bash
   ./build/Beyondmimic_Deploy eth0
   ```

   Replace `eth0` with the actual robot-facing interface.

## Controls

On the physical robot, the controller accepts Unitree's original wireless
remote from `LowState.wireless_remote`.  Under MuJoCo, `unitree_mujoco` can
emulate that packet with a gamepad.  Keyboard commands are provided for
development only.

| Action | Unitree wireless remote | Keyboard |
| --- | --- | --- |
| Zero torque | `L1 + Start` | `0` |
| Startup pose | `Start` | `1` |
| Enter Loco | `R1 + A` | `2` |
| Enter RedRed from Loco | `R1 + X` | `4` |
| Damping | `F1` or `Select` | `3` |
| Damping and exit | — | `Q` |
| Loco velocity | sticks | `W` / `S` forward/back, `A` / `D` yaw |

In Loco, stick velocity commands are continuously sent to the policy.  `R1`
is only part of the state-switch combinations.  Planar commands use a `0.1`
dead zone followed by `0.95 × previous + 0.05 × current` smoothing; yaw is
not smoothed.

## Operating Procedure

1. Start the executable.  The initial state is `ZeroTorque`.
2. Select `StartupReady` and wait for the two-second pose transition.
3. Keep the robot suspended, then select `Loco` with `R1 + A`.
4. Use the sticks to command velocity.  Select `R1 + X` in Loco to enter the
   RedRed transition and motion-tracking state.
5. Select `F1` or `Select` at any time to enter `Damping`.

## Safety Notes

`Damping` is selected for invalid LowState CRC, LowState older than 100 ms,
non-finite state or command values, inversion in an active mode, or ONNX
inference failure.  Its command is zero position/torque with `kd = 8`.

Joint limits, target slew limits, and enforcement of the RedRed YAML
`torque_limits` are not implemented in this version.  Do not treat those YAML
values as active protection.

## Project Structure

```text
app/                 executable entry point
common/              source-relative path helper
core/                state machine, control types, and safety supervisor
hardware/unitree/    Unitree DDS, CRC, and wireless remote adapter
inference/           ONNX Runtime wrapper and tensor validation
states/              basic, startup, Loco, and RedRed states
configs/             policy-specific YAML configuration
model/               locally supplied policy models (ignored by Git)
thirdparty/          local dependency instructions
```

For state transition and data-flow details, see
[FSM_ARCHITECTURE.md](FSM_ARCHITECTURE.md).

## Motion Demonstration

<p align="center">
  <img src="assets/beyondmimic-dance.gif" alt="G1 RedRed dance demonstration" width="360">
</p>

## Acknowledgments

- [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym) —
  deployment workflow and compatibility configuration reference; BSD 3-Clause
  text is retained in [LICENSES/](LICENSES/unitree_rl_gym-BSD-3-Clause.txt).
- [RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy) —
  multi-policy deployment architecture and operating-workflow reference.
- [BeyondMimic / whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking)
  — motion-tracking training formulation reference.
- [ONNX Runtime](https://onnxruntime.ai/) — policy inference runtime.

See [NOTICE.md](NOTICE.md) for third-party notices and
[model/README.md](model/README.md) for policy-asset requirements.
