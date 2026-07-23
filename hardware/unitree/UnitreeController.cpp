#include "hardware/unitree/UnitreeController.h"

#include "common/ProjectPath.h"
#include "hardware/unitree/Crc.h"
#include "states/StateFactory.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <iostream>
#include <limits>
#include <thread>
#include <unistd.h>

namespace deploy_real
{
namespace
{

constexpr char kLowCmdTopic[] = "rt/lowcmd";
constexpr char kLowStateTopic[] = "rt/lowstate";

constexpr std::uint16_t ButtonMask(unsigned int bit)
{
    return static_cast<std::uint16_t>(1U << bit);
}

constexpr std::uint16_t kButtonR1 = ButtonMask(0);
constexpr std::uint16_t kButtonL1 = ButtonMask(1);
constexpr std::uint16_t kButtonStart = ButtonMask(2);
constexpr std::uint16_t kButtonSelect = ButtonMask(3);
constexpr std::uint16_t kButtonF1 = ButtonMask(6);
constexpr std::uint16_t kButtonA = ButtonMask(8);
constexpr std::uint16_t kButtonX = ButtonMask(10);

std::int64_t SteadyClockNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

float SafeAxis(float value)
{
    return std::isfinite(value) ? std::clamp(value, -1.0f, 1.0f) : 0.0f;
}

} // namespace

UnitreeController::UnitreeController(const std::string &network_interface)
    : loco_config_(LocoConfig::Load(ProjectPath("configs/loco_mode/g1.yaml"))),
      redred_config_(RedRedConfig::Load(ProjectPath("configs/redred/g1.yaml"))),
      state_machine_(CreateDefaultStateMachine(loco_config_, redred_config_))
{
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    lowcmd_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
            kLowCmdTopic));
    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
            kLowStateTopic));

    lowcmd_publisher_->InitChannel();
    lowstate_subscriber_->InitChannel(
        std::bind(&UnitreeController::LowStateMessageHandler, this,
                  std::placeholders::_1));

    while (!low_state_buffer_.GetDataPtr())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    low_cmd_write_thread_ = unitree::common::CreateRecurrentThreadEx(
        "low_cmd_write", UT_CPU_ID_NONE, 2000,
        &UnitreeController::LowCmdWriteHandler, this);
}

UnitreeController::~UnitreeController()
{
    low_cmd_write_thread_.reset();
    RestoreTerminal();
}

void UnitreeController::InitializeTerminal()
{
    if (terminal_initialized_ || !isatty(STDIN_FILENO))
    {
        return;
    }
    if (tcgetattr(STDIN_FILENO, &original_termios_) != 0)
    {
        return;
    }

    original_stdin_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (original_stdin_flags_ < 0)
    {
        return;
    }

    struct termios terminal = original_termios_;
    terminal.c_lflag &= ~(ICANON | ECHO);
    terminal.c_cc[VMIN] = 0;
    terminal.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &terminal) != 0)
    {
        return;
    }
    if (fcntl(STDIN_FILENO, F_SETFL, original_stdin_flags_ | O_NONBLOCK) != 0)
    {
        tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
        return;
    }
    terminal_initialized_ = true;
}

void UnitreeController::RestoreTerminal()
{
    if (!terminal_initialized_)
    {
        return;
    }
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios_);
    fcntl(STDIN_FILENO, F_SETFL, original_stdin_flags_);
    terminal_initialized_ = false;
}

RobotState UnitreeController::ReadRobotState()
{
    RobotState state;
    const auto low_state = low_state_buffer_.GetDataPtr();
    if (!low_state)
    {
        return state;
    }

    for (std::size_t motor_idx = 0; motor_idx < kNumMotors; ++motor_idx)
    {
        state.position[motor_idx] = low_state->motor_state()[motor_idx].q();
        state.velocity[motor_idx] = low_state->motor_state()[motor_idx].dq();
        state.estimated_torque[motor_idx] =
            low_state->motor_state()[motor_idx].tau_est();
    }
    for (std::size_t i = 0; i < state.base_quaternion.size(); ++i)
    {
        state.base_quaternion[i] = low_state->imu_state().quaternion()[i];
    }
    for (std::size_t i = 0; i < 3; ++i)
    {
        state.angular_velocity[i] = low_state->imu_state().gyroscope()[i];
    }

    Eigen::Quaternionf quaternion(
        state.base_quaternion[0], state.base_quaternion[1],
        state.base_quaternion[2], state.base_quaternion[3]);
    const float quaternion_norm = quaternion.norm();
    if (std::isfinite(quaternion_norm) && quaternion_norm > 1.0e-6f)
    {
        quaternion.normalize();
        state.base_quaternion = {
            quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
        const Eigen::Matrix3f rotation = quaternion.toRotationMatrix();
        for (std::size_t i = 0; i < 3; ++i)
        {
            state.gravity[i] = -rotation(2, i);
        }
    }
    else
    {
        state.gravity.fill(std::numeric_limits<float>::quiet_NaN());
    }

    const std::int64_t last_state_ns = last_low_state_ns_.load(std::memory_order_acquire);
    const std::int64_t age_ns =
        std::max<std::int64_t>(0, SteadyClockNowNs() - last_state_ns);
    state.age = std::chrono::nanoseconds(age_ns);
    state.valid = last_state_ns != 0;
    return state;
}

UserInput UnitreeController::ReadUserInput()
{
    UserInput input;
    if (const auto joystick = joystick_buffer_.GetDataPtr())
    {
        const bool joystick_is_finite =
            std::isfinite(joystick->lx) && std::isfinite(joystick->ly) &&
            std::isfinite(joystick->rx) && std::isfinite(joystick->ry);
        if (joystick_is_finite)
        {
            input.velocity_command = {
                SafeAxis(joystick->ly), SafeAxis(-joystick->lx), SafeAxis(-joystick->rx)};

            const std::uint16_t buttons = joystick->buttons.value;
            const std::uint16_t pressed = buttons & ~last_joystick_buttons_;
            const bool zero_requested =
                (buttons & (kButtonL1 | kButtonStart)) == (kButtonL1 | kButtonStart) &&
                (pressed & (kButtonL1 | kButtonStart));
            const bool loco_requested =
                (buttons & (kButtonR1 | kButtonA)) == (kButtonR1 | kButtonA) &&
                (pressed & (kButtonR1 | kButtonA));
            const bool redred_requested =
                (buttons & (kButtonR1 | kButtonX)) == (kButtonR1 | kButtonX) &&
                (pressed & (kButtonR1 | kButtonX));

            if (zero_requested)
            {
                input.requested_state = StateId::ZeroTorque;
            }
            else if (pressed & (kButtonF1 | kButtonSelect))
            {
                input.requested_state = StateId::Damping;
            }
            else if (loco_requested)
            {
                input.requested_state = StateId::Loco;
            }
            else if (redred_requested)
            {
                input.requested_state = StateId::RedRedCast;
            }
            else if (pressed & kButtonStart)
            {
                input.requested_state = StateId::StartupReady;
            }
            last_joystick_buttons_ = buttons;
        }
        else
        {
            // Do not retain a pressed-button edge after an invalid packet.
            last_joystick_buttons_ = 0;
        }
    }

    char key = 0;
    while (read(STDIN_FILENO, &key, 1) == 1)
    {
        switch (key)
        {
        case '0':
            input.requested_state = StateId::ZeroTorque;
            break;
        case '1':
            input.requested_state = StateId::StartupReady;
            break;
        case '2':
            input.requested_state = StateId::Loco;
            break;
        case '3':
            input.requested_state = StateId::Damping;
            break;
        case '4':
            input.requested_state = StateId::RedRedCast;
            break;
        case 'w':
        case 'W':
            input.velocity_command[0] = 1.0f;
            break;
        case 's':
        case 'S':
            input.velocity_command[0] = -1.0f;
            break;
        case 'a':
        case 'A':
            input.velocity_command[2] = 1.0f;
            break;
        case 'd':
        case 'D':
            input.velocity_command[2] = -1.0f;
            break;
        case 'q':
        case 'Q':
            input.requested_state = StateId::Damping;
            input.quit = true;
            break;
        default:
            break;
        }
    }
    return input;
}

void UnitreeController::PublishCommand(const MotorCommand &command)
{
    auto low_cmd = std::make_shared<unitree_hg::msg::dds_::LowCmd_>();
    for (auto &motor_cmd : low_cmd->motor_cmd())
    {
        motor_cmd.q() = 0.0f;
        motor_cmd.dq() = 0.0f;
        motor_cmd.kp() = 0.0f;
        motor_cmd.kd() = 0.0f;
        motor_cmd.tau() = 0.0f;
    }
    for (std::size_t motor_idx = 0; motor_idx < kNumMotors; ++motor_idx)
    {
        auto &motor_cmd = low_cmd->motor_cmd()[motor_idx];
        motor_cmd.q() = command.position[motor_idx];
        motor_cmd.dq() = command.velocity[motor_idx];
        motor_cmd.kp() = command.kp[motor_idx];
        motor_cmd.kd() = command.kd[motor_idx];
        motor_cmd.tau() = command.torque[motor_idx];
    }
    low_cmd_buffer_.SetDataPtr(low_cmd);
}

void UnitreeController::Run()
{
    InitializeTerminal();
    std::cout << "Controls: 0 zero torque, 1 startup ready, 2 loco, "
                 "3 damping, 4 RedRed cast, W/A/S/D velocity, Q damping and exit\n";

    const auto cycle_time = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<float>(loco_config_.control_dt));
    auto next_cycle = std::chrono::steady_clock::now();
    bool shutdown_requested = false;
    std::chrono::steady_clock::time_point shutdown_time{};

    while (true)
    {
        const auto now = std::chrono::steady_clock::now();
        const RobotState robot_state = ReadRobotState();
        UserInput user_input = ReadUserInput();
        if (user_input.quit && !shutdown_requested)
        {
            shutdown_requested = true;
            shutdown_time = now + std::chrono::seconds(1);
        }

        const StateContext context{
            robot_state, user_input, now, loco_config_.control_dt};
        MotorCommand command;
        state_machine_->Update(context, command);
        PublishCommand(command);

        if (shutdown_requested && now >= shutdown_time)
        {
            break;
        }

        next_cycle += cycle_time;
        if (next_cycle > std::chrono::steady_clock::now())
        {
            std::this_thread::sleep_until(next_cycle);
        }
        else
        {
            next_cycle = std::chrono::steady_clock::now();
        }
    }
}

void UnitreeController::LowStateMessageHandler(const void *message)
{
    const auto *low_state =
        static_cast<const unitree_hg::msg::dds_::LowState_ *>(message);
    const std::uint32_t received_crc = low_state->crc();
    const std::uint32_t computed_crc = Crc32Core(
        reinterpret_cast<const std::uint32_t *>(low_state),
        (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1);
    if (received_crc != computed_crc)
    {
        std::cerr << "[DDS] rejected LowState with invalid CRC\n";
        return;
    }

    low_state_buffer_.SetData(*low_state);

    JoystickData joystick{};
    const std::size_t copy_size =
        std::min(sizeof(joystick),
                 low_state->wireless_remote().size() * sizeof(std::uint8_t));
    std::memcpy(&joystick, low_state->wireless_remote().data(), copy_size);
    joystick_buffer_.SetData(joystick);
    last_low_state_ns_.store(SteadyClockNowNs(), std::memory_order_release);
}

void UnitreeController::LowCmdWriteHandler()
{
    const auto low_cmd = low_cmd_buffer_.GetDataPtr();
    const auto low_state = low_state_buffer_.GetDataPtr();
    if (!low_cmd || !low_state)
    {
        return;
    }

    low_cmd->mode_machine() = low_state->mode_machine();
    low_cmd->mode_pr() = 0;
    for (auto &motor_cmd : low_cmd->motor_cmd())
    {
        motor_cmd.mode() = 1;
    }
    low_cmd->crc() = Crc32Core(
        reinterpret_cast<const std::uint32_t *>(low_cmd.get()),
        (sizeof(unitree_hg::msg::dds_::LowCmd_) >> 2) - 1);
    lowcmd_publisher_->Write(*low_cmd);
}

} // namespace deploy_real
