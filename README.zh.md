# Beyondmimic_Deploy

[English](README.md)

> **实验性低层控制程序。** 仅可在吊装、受控场地及操作员可立即切入阻尼的条件下使用；它不是经认证的安全控制器。

该 C++17 项目通过 `unitree_sdk2` DDS 在 Unitree G1 上部署两个 ONNX 策略：

- **Loco**：摇杆速度指令行走，`96` 维观测输出 `29` 维动作；
- **RedRed**：有限长度的参考动作跟踪，`154` 维观测输出 `29` 维动作。

策略不会直接写 DDS。状态机负责检查机器人状态、选择策略，并生成 29 个电机的位置/PD 命令。

```text
LowState + 原厂遥控器
          ↓
UnitreeController（50 Hz 控制循环）
          ↓
StateMachine + SafetySupervisor
          ↓
ZeroTorque | StartupReady | Loco | RedRedCast | RedRed | Damping
          ↓
MotorCommand(q, dq, kp, kd, tau) × 29
          ↓
LowCmd DDS 写线程（500 Hz）
```

实现细节见 [FSM_ARCHITECTURE.md](FSM_ARCHITECTURE.md)。

## 平台与依赖

ONNX Runtime 必须与运行可执行文件的 CPU 架构一致：

| 使用场景 | 目标架构 | ONNX Runtime 包 |
| --- | --- | --- |
| 开发 / 仿真 | `x86_64` | `onnxruntime-linux-x64-1.22.0` |
| G1 PC2 实机 | `aarch64` | `onnxruntime-linux-aarch64-1.22.0` |

同一架构下需安装：CMake ≥ 3.16、支持 C++17 的编译器、`unitree_sdk2`、`yaml-cpp`、Eigen3 和 ONNX Runtime 1.22.0 CPU 包。

不要提交特定架构的 ONNX Runtime 二进制；目录约定和 `ONNXRUNTIME_ROOT` 覆盖方式见 [thirdparty/README.md](thirdparty/README.md)。

## 编译

在 G1 PC2 上进行原生 aarch64 编译，安装 aarch64 ONNX Runtime 后执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.22.0
cmake --build build -j
```

x86_64 开发机使用匹配的 x64 包即可。若存在
`thirdparty/onnxruntime-linux-<arch>-1.22.0`，CMake 会自动选择它。旧版
`deploy_real.py` 已不属于当前项目。

## 实机运行

启动前需使机器人进入 Unitree 的低层/调试就绪状态，连接网线后运行：

```bash
./build/Beyondmimic_Deploy eth0
```

将 `eth0` 替换为机器人连接网卡。程序从 `ZeroTorque` 开始，必须先进入
`StartupReady`，等待两秒姿态插值完成后才能进入 `Loco`。

## 操作

程序从 `LowState.wireless_remote` 解析 Unitree 原厂遥控器：

| 操作 | 遥控器 | 键盘（仅开发） |
| --- | --- | --- |
| 零力矩 | `L1 + Start` | `0` |
| 启动姿态 | `Start` | `1` |
| Loco | `R1 + A` | `2` |
| RedRed 过渡 / 动作 | Loco 中按 `R1 + X` | Loco 中按 `4` |
| 阻尼 | `F1` 或 `Select` | `3` |
| 阻尼后退出 | — | `Q` |
| 速度指令 | 摇杆 | `W/S` 前后，`A/D` 偏航 |

进入 Loco 后，原厂遥控器摇杆会持续向策略提供速度指令。`R1` 只用于
`R1 + A`、`R1 + X` 的模式切换组合键；需要阻尼时请按 `F1` 或 `Select`。

平移摇杆使用 `0.1` 死区，并按 `0.95 × 上一帧 + 0.05 × 当前值` 平滑；偏航不做平滑。两个参数在 `configs/loco_mode/g1.yaml` 中配置。

## 安全行为与当前边界

CRC 不正确的 LowState 会被控制器丢弃。当 LowState 不可用或超过 100 ms、状态/命令存在非有限数、活动模式中机器人翻转、或 ONNX 推理异常时，状态机会进入 `Damping`。阻尼命令为零位置/零力矩、`kd = 8`。

此第一版**尚未**实施关节限位、目标角变化率限制，或 RedRed YAML 中 `torque_limits` 的实际限幅。请勿将这些 YAML 数值视作已经生效的保护；在无吊装或高扰动使用前，必须补充并验证命令限幅器。

## 配置与模型

- [configs/loco_mode/g1.yaml](configs/loco_mode/g1.yaml)：Loco 观测、关节映射、PD 参数和速度上限；
- [configs/redred/g1.yaml](configs/redred/g1.yaml)：RedRed 张量契约、参考动作、关节映射和 PD 参数；
- `model/g1/policy96.onnx`、`model/g1/redred.onnx`：策略模型。

模型默认被 `.gitignore` 忽略，等待确认再分发权利。发布前请阅读 [model/README.md](model/README.md)。

## 目录

```text
app/                 可执行程序入口
core/                状态机、状态类型和安全检查
hardware/unitree/    Unitree DDS 与原厂遥控器适配
inference/           带输入输出校验的 ONNX Runtime 包装
states/              基础、启动、Loco 与 RedRed 状态
configs/             策略专属 YAML
thirdparty/          本地依赖说明（不提交运行时二进制）
```

## 动作演示

下图从本地 `beyondmimic.mp4` 的首个干净舞蹈画面（约 `4.0 s`）开始裁剪，持续至整段动作结束；仅保留中央机器人视窗，不包含终端或控制面板。

<p align="center">
  <img src="assets/beyondmimic-dance.gif" alt="G1 RedRed 跳舞动作演示" width="360">
</p>

## 版权、致谢与许可证

本仓库是独立实现的 C++ 部署程序，包含自主编写的代码，也包含基于下列项目修改的内容。第三方代码、模型和动作数据**不**当然适用同一份许可证；完整说明见 [NOTICE.md](NOTICE.md)。

- **Unitree RL Gym**：部署流程和兼容配置的部分内容修改自 [unitreerobotics/unitree_rl_gym](https://github.com/unitreerobotics/unitree_rl_gym)。该项目版权归 HangZhou YuShu TECHNOLOGY CO., LTD.（Unitree Robotics）所有，© 2016–2023，采用 BSD 3-Clause 许可证。原始完整许可证已保留在 [LICENSES/unitree_rl_gym-BSD-3-Clause.txt](LICENSES/unitree_rl_gym-BSD-3-Clause.txt)。
- **RoboMimic Deploy**：多策略部署架构和操作流程参考了 [ccrpRepo/RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)。该上游仓库当前没有声明许可证。此处致谢**不构成**对其源代码、策略文件、动作数据或机器人资源的复制、修改或再发布授权；公开发布前，必须删除/自行重写任何此类内容，或取得权利人的书面许可。
- **BeyondMimic**：动作跟踪的训练方法参考了 [HybridRobotics/whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking) 与 BeyondMimic 论文；其上游训练代码采用 MIT 许可证。本部署仓库未包含该训练代码；若将来引入，必须同时保留其版权声明和 MIT 许可证。

ONNX 策略和动作资产属于独立交付物。在其来源、训练数据条款和再分发权利记录到 [model/README.md](model/README.md) 前，默认不纳入公开发布。

本仓库目前尚未选择项目级许可证。在版权持有人于根目录添加 `LICENSE` 前，仓库中自主贡献代码的全部权利均予保留。正式公开发布时，必须保留 Unitree 的 BSD 声明，并先处理 RoboMimic Deploy 内容的来源与授权问题，再为自主代码选择许可证。
