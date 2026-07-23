# Beyondmimic_Deploy

面向 Unitree G1 的 C++17 实机部署控制器。项目通过 `unitree_sdk2` DDS 运行
ONNX 策略，并由有限状态机管理行走和参考动作跟踪。

[English](README.md)

> **实验性低层控制程序。** 仅可在受控场地、机器人吊装且操作员可立即切入阻尼的条件下使用；它不是经认证的安全控制器。

## 功能特性

- **有限状态机控制**：`ZeroTorque`、`StartupReady`、`Loco`、`RedRedCast`、
  `RedRed`、`Damping` 明确管理策略和状态切换。
- **双 ONNX 策略**：Loco 使用摇杆速度指令（`96` 维观测 → `29` 维动作）；
  RedRed 执行参考动作跟踪（`154` 维观测 → `29` 维动作）。
- **Unitree 原厂遥控器**：实机直接读取 `LowState.wireless_remote`；MuJoCo 中可由
  Unitree 模拟器使用 Xbox 或 Switch 手柄模拟同一份遥控器数据包。
- **ARM64 构建适配**：CMake 自动选择 x64 或 aarch64 ONNX Runtime；G1 PC2 的
  原生目标为 aarch64。
- **运行时检查**：LowState CRC 错误、状态超时、非有限数、活动状态翻转和 ONNX
  推理异常都会切换到 `Damping`。

## 环境依赖

| 目标 | 架构 | ONNX Runtime 包 |
| --- | --- | --- |
| 开发机 | `x86_64` | `onnxruntime-linux-x64-1.22.0` |
| G1 PC2 实机 | `aarch64` | `onnxruntime-linux-aarch64-1.22.0` |

- CMake 3.16 或更高版本；
- 支持 C++17 的编译器；
- `unitree_sdk2`、`yaml-cpp`、Eigen3；
- 与目标架构匹配的 ONNX Runtime 1.22.0 CPU 包。

### 安装 ONNX Runtime

下载与实际运行机器架构一致的包。例如，在 G1 PC2 上执行：

```bash
mkdir -p thirdparty
cd thirdparty
wget https://github.com/microsoft/onnxruntime/releases/download/v1.22.0/onnxruntime-linux-aarch64-1.22.0.tgz
tar -xzf onnxruntime-linux-aarch64-1.22.0.tgz
```

x86_64 开发机将 `aarch64` 替换为 `x64`。解压目录已被 Git 忽略；也可通过
`ONNXRUNTIME_ROOT` 使用仓库外的安装路径，详见 [thirdparty/README.md](thirdparty/README.md)。

## 编译

若 ONNX Runtime 解压在 `thirdparty/` 下，CMake 会自动选择对应目录：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

若 ONNX Runtime 位于其他位置，显式指定其根目录：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT=/opt/onnxruntime-linux-aarch64-1.22.0
cmake --build build -j
```

生成的可执行文件为 `build/Beyondmimic_Deploy`。

## 配置

| 文件 | 用途 |
| --- | --- |
| [configs/loco_mode/g1.yaml](configs/loco_mode/g1.yaml) | Loco 的策略路径、观测设置、关节映射、PD 参数、速度上限和摇杆滤波。 |
| [configs/redred/g1.yaml](configs/redred/g1.yaml) | RedRed 的 ONNX 张量契约、过渡时长、参考动作设置、关节映射和 PD 参数。 |
| `model/g1/policy96.onnx` | 默认配置所需的 Loco 策略。 |
| `model/g1/redred.onnx` | 默认配置所需的 RedRed 策略。 |

策略二进制默认不提交。运行前请将已验证的模型放到上述路径；模型来源和张量契约记录见
[model/README.md](model/README.md)。

## 部署到 MuJoCo 仿真

本控制器在仿真和实机中都使用 Unitree SDK2 的 `LowState`／`LowCmd` DDS 通路，
可配合 [unitreerobotics/unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco)
中的 C++ 模拟器使用。

1. 按 Unitree MuJoCo 官方说明安装并编译模拟器。
2. 在 `unitree_mujoco/simulate/config.yaml` 中选择 G1 场景，并与控制器的仿真 DDS
   设置保持一致：

   ```yaml
   robot: "g1"
   robot_scene: "scene_29dof.xml"
   domain_id: 1
   interface: "lo"
   use_joystick: 1
   joystick_type: "xbox"
   enable_elastic_band: 1
   ```

   `unitree_mujoco` 中的手柄只负责模拟 Unitree 无线遥控器数据包；控制器仍从
   `LowState.wireless_remote` 读取输入。
3. 先启动模拟器：

   ```bash
   cd unitree_mujoco/simulate/build
   ./unitree_mujoco
   ```

4. 在另一终端不带网卡参数启动本控制器：

   ```bash
   ./build/Beyondmimic_Deploy
   ```

   不带参数时会使用 DDS 域 `1` 和回环网卡 `lo`。

## 实机运行

1. 让 G1 进入 Unitree 低层／调试就绪模式，并将机器人吊起。
2. 将 ONNX 策略文件复制到 `model/g1/`。
3. 在 PC2 上使用 aarch64 ONNX Runtime 编译本项目。
4. 使用连接机器人的网卡启动控制器：

   ```bash
   ./build/Beyondmimic_Deploy eth0
   ```

   将 `eth0` 替换为实际机器人网卡名称。

## 遥控器与键盘操作

实机上，控制器直接从 `LowState.wireless_remote` 读取 Unitree 原厂遥控器；MuJoCo
下可由 `unitree_mujoco` 使用手柄模拟该数据包。键盘仅用于开发调试。

| 操作 | Unitree 原厂遥控器 | 键盘 |
| --- | --- | --- |
| 零力矩 | `L1 + Start` | `0` |
| 启动姿态 | `Start` | `1` |
| 进入 Loco | `R1 + A` | `2` |
| 从 Loco 进入 RedRed | `R1 + X` | `4` |
| 阻尼 | `F1` 或 `Select` | `3` |
| 阻尼并退出 | — | `Q` |
| Loco 速度 | 摇杆 | `W` / `S` 前后，`A` / `D` 偏航 |

在 Loco 状态下，摇杆速度会持续传入策略。`R1` 只用于模式切换组合键。平移摇杆
使用 `0.1` 死区，并按 `0.95 × 上一帧 + 0.05 × 当前值` 平滑；偏航不平滑。

## 操作流程

1. 启动程序后，初始状态为 `ZeroTorque`。
2. 选择 `StartupReady`，等待两秒姿态过渡完成。
3. 保持机器人吊装，按 `R1 + A` 进入 `Loco`。
4. 使用摇杆输入速度；在 Loco 中按 `R1 + X` 进入 RedRed 过渡和动作跟踪状态。
5. 任意时刻按 `F1` 或 `Select` 都可进入 `Damping`。

## 安全说明

LowState CRC 无效、LowState 超过 100 ms、状态或命令出现非有限数、活动状态中翻转、
或 ONNX 推理失败时，控制器会进入 `Damping`。阻尼指令为零位置／零力矩，`kd = 8`。

当前版本尚未实现关节限位、目标角变化率限制，以及 RedRed YAML 中 `torque_limits` 的
实际限幅；请勿将这些数值视为已生效的保护。

## 目录结构

```text
app/                 可执行程序入口
common/              源码相对路径工具
core/                状态机、控制类型和安全监控
hardware/unitree/    Unitree DDS、CRC 和原厂遥控器适配
inference/           ONNX Runtime 包装与张量校验
states/              基础、启动、Loco 和 RedRed 状态
configs/             策略专属 YAML 配置
model/               本地策略模型（Git 忽略）
thirdparty/          本地依赖安装说明
```

状态转换和数据流细节见 [FSM_ARCHITECTURE.md](FSM_ARCHITECTURE.md)。

## 动作演示

<p align="center">
  <img src="assets/beyondmimic-dance.gif" alt="G1 RedRed 跳舞动作演示" width="360">
</p>

## 致谢

- [Unitree RL Gym](https://github.com/unitreerobotics/unitree_rl_gym)：部署流程和
  兼容配置参考；BSD 3-Clause 全文保留在 [LICENSES/](LICENSES/unitree_rl_gym-BSD-3-Clause.txt)。
- [RoboMimic_Deploy](https://github.com/ccrpRepo/RoboMimic_Deploy)：多策略部署架构和
  操作流程参考。
- [BeyondMimic / whole_body_tracking](https://github.com/HybridRobotics/whole_body_tracking)：
  动作跟踪训练方法参考。
- [ONNX Runtime](https://onnxruntime.ai/)：策略推理运行时。

第三方声明见 [NOTICE.md](NOTICE.md)，策略模型要求见
[model/README.md](model/README.md)。
