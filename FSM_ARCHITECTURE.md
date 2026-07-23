# G1 多模型状态机

## 目录分类

```text
deploy_real/
├── app/                         # 可执行程序入口
├── common/                      # 项目路径等无业务工具
├── core/                        # 状态机、公共状态与安全检查
├── hardware/unitree/            # Unitree DDS、遥控器、CRC
├── inference/                   # 通用张量和 ONNX Runtime
├── states/
│   ├── basic/                   # 零力矩、阻尼等无模型状态
│   ├── startup/                 # 启动预备状态
│   ├── policy/                  # 模型状态公共执行流程
│   ├── loco/                    # Loco 独有配置、观测和动作映射
│   ├── redred/                  # RedRed 动作数据、154 维观测和推理
│   └── StateFactory.*           # 状态注册和转移关系
├── configs/                     # 每个策略自己的 YAML
├── model/                       # 模型文件
├── tests/                       # 不连接机器人的离线测试
└── tools/                       # MuJoCo 等辅助验证脚本
```

## 公共边界

每个控制周期都遵守：

```text
UnitreeController
  -> RobotState + UserInput
  -> StateMachine
  -> ControlState::Update
  -> SafetySupervisor
  -> MotorCommand
  -> Unitree DDS
```

`RobotState` 提供原始关节状态、估计力矩、IMU 四元数、角速度和重力方向。`MotorCommand` 统一输出 29 个电机的位置、速度、PD 和力矩。硬件层不知道模型的观测维度，也不包含策略分支。

## 策略状态

模型状态继承 `PolicyState`。公共执行流程只有：

```text
BuildInputs -> OnnxRuntime::Run -> DecodeOutputs
```

- `BuildInputs()` 由每个策略实现。Loco 可以读取速度指令，动作模仿策略可以读取相位和历史，参考轨迹策略可以读取 motion 数据。
- `OnnxRuntime` 只处理具名 `TensorMap`，支持多输入、多输出和动态维度，不理解观测语义。
- `DecodeOutputs()` 由每个策略实现，负责动作缩放、关节映射和 PD 参数。
- `OnEnter()`/`OnExit()` 用于清空历史动作、相位、参考帧等策略内部状态。

因此，观测构造不是所有模型共用的。只有结构完全相同的一组模型才应在自己的策略目录中复用局部 Builder，不应把所有观测字段写进一个通用 YAML。

RedRed 通过 `RedRedCast` 从 Loco 切入。正向过渡时长由 `cast_duration` 配置；期间 Loco 以零速度指令持续控制腰腿电机 `0..14`，手臂电机 `15..28` 从实测位置插值到 YAML 中记录的 `motion_frame0_joint_angles`。进入 RedRed 时会记录机器人当前 heading 与参考动作初始 heading 的 yaw 偏移，并把整段参考轨迹相对旋转到机器人当前朝向，不依赖场地绝对方向。RedRed 使用 ONNX 内置参考生成器，通过 `time_step` 缓存下一帧关节和刚体参考，不依赖外部 NPZ。动作按 `loop: false` 完整执行一次，终端显示单行进度条，完成后按 `end_state: "loco"` 自动回到 Loco；反向过渡时长由独立的 `return_duration` 配置。`default_angles` 始终使用 ONNX metadata 对应的训练默认角。

## 状态图

```text
ZeroTorque <-> StartupReady <-> Loco -> RedRedCast -> RedRed
     ^                         ^                      |
     |                         +----------------------+
     |
  Damping <-------- fault/operator request from active states
```

- 禁止 `ZeroTorque -> Loco`，必须先完成两秒启动姿态插值。
- `RedRedCast` 只能从 Loco 进入；过渡中可退回 Loco 或进入阻尼。
- `Damping` 只允许回到 `ZeroTorque`。
- DDS 超过 100 ms 未更新、机器人翻转、状态/命令出现非有限值或推理异常时，统一进入 `Damping`。

## 增加策略

1. 在 `states/<strategy>/` 新建 `<Strategy>Config` 和 `<Strategy>State`。
2. 有模型的状态继承 `PolicyState`，实现 `BuildInputs()` 和 `DecodeOutputs()`；无模型状态直接继承 `ControlState`。
3. 在 `StateId` 中增加状态 ID。
4. 在 `StateFactory.cpp` 注册状态及允许的转移；需要新操作入口时再修改 `UnitreeController` 的输入映射。
5. 将 YAML 和模型分别放入 `configs/<strategy>/`、`model/<strategy>/`。
6. 增加离线测试，至少验证模型输入输出契约、动作有限性和状态转移。

## 操作

| 目标状态 | 键盘 | 遥控器 |
|---|---|---|
| `ZeroTorque` | `0` | `L1 + Start` |
| `StartupReady` | `1` | `Start` |
| `Loco` | `2` | `R1 + A` |
| `Damping` | `3` | `F1` 或 `Select` |
| `RedRedCast -> RedRed` | `4`（仅 Loco） | `R1 + X`（仅 Loco） |
| 阻尼后退出 | `Q` | - |

键盘 `W/S` 控制前后速度，`A/D` 控制偏航；遥控器使用左摇杆平移、右摇杆横向控制偏航。进入 Loco 后摇杆速度会持续传给策略；`R1` 仅用于模式切换组合键。平移摇杆采用 `0.1` 死区，并按 `0.95 × 上一帧 + 0.05 × 当前值` 平滑；偏航不平滑。LowState CRC 校验失败时整帧丢弃，超过 100 ms 未收到有效状态时进入阻尼。
