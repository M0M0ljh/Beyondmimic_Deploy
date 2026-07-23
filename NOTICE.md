# Third-party notices and release checklist

This repository combines original deployment code with material informed by
third-party projects.  Each component keeps its own copyright and license.

## Unitree RL Gym

- Upstream: <https://github.com/unitreerobotics/unitree_rl_gym>
- Copyright: © 2016–2023 HangZhou YuShu TECHNOLOGY CO., LTD. (Unitree Robotics)
- License: BSD 3-Clause; full text:
  [LICENSES/unitree_rl_gym-BSD-3-Clause.txt](LICENSES/unitree_rl_gym-BSD-3-Clause.txt)
- Use in this repository: parts of the deployment workflow and compatibility
  configuration were adapted and extended for the C++ multi-policy controller.

Source and binary redistributions of adapted Unitree RL Gym material must retain
its copyright notice, BSD 3-Clause conditions, and disclaimer.

## RoboMimic Deploy

- Upstream: <https://github.com/ccrpRepo/RoboMimic_Deploy>
- Use in this repository: design reference for multi-policy deployment,
  state-machine workflow, and operating procedure.
- License status: no `LICENSE` or `LICENCE` file was present in the upstream
  repository when this notice was written.

Public availability is not a license.  If this repository contains copied or
modified RoboMimic Deploy source files, policy binaries, motion data, robot
assets, or documentation, do not publish those materials until their copyright
holder grants written permission or a license is added upstream.  Attribution by
itself is not sufficient.

## BeyondMimic motion-tracking framework

- Training code: <https://github.com/HybridRobotics/whole_body_tracking>
- Project page: <https://beyondmimic.github.io/>
- Upstream code license: MIT.
- Use in this repository: reference for the training formulation; the upstream
  training implementation is not vendored here.

If any BeyondMimic source is imported later, retain its copyright notice and
include the full MIT license with that source.  Cite the BeyondMimic paper in
research outputs that use its training formulation.

## Models, motion data, and binary dependencies

Policy models, reference motions, ONNX Runtime packages, and robot assets may
have licenses independent of the controller source.  See
[model/README.md](model/README.md) and [thirdparty/README.md](thirdparty/README.md).
Do not publish an asset until its origin, license, and redistribution rights are
known.

## Before a public release

1. Identify the copyright holder(s) for this repository's original code.
2. Audit every file copied or modified from an upstream project.
3. Remove/reimplement or obtain written permission for any RoboMimic Deploy
   material.
4. Record policy and motion-data provenance, license, and checksum.
5. Add a top-level project `LICENSE` only after steps 1–4 are complete; retain
   all third-party notices regardless of that choice.

