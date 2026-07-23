#ifndef DEPLOY_REAL_HARDWARE_UNITREE_UNITREE_CONTROLLER_H
#define DEPLOY_REAL_HARDWARE_UNITREE_UNITREE_CONTROLLER_H

#include "core/StateMachine.h"
#include "hardware/unitree/DataBuffer.h"
#include "hardware/unitree/Joystick.h"
#include "states/loco/LocoConfig.h"
#include "states/redred/RedRedConfig.h"

#include <termios.h>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>

namespace deploy_real
{

class UnitreeController
{
public:
    UnitreeController(int dds_domain_id, const std::string &network_interface);
    ~UnitreeController();

    void Run();
    void LowStateMessageHandler(const void *message);

private:
    RobotState ReadRobotState();
    UserInput ReadUserInput();
    void PublishCommand(const MotorCommand &command);
    void LowCmdWriteHandler();
    void InitializeTerminal();
    void RestoreTerminal();

    LocoConfig loco_config_;
    RedRedConfig redred_config_;
    std::unique_ptr<StateMachine> state_machine_;

    unitree::common::ThreadPtr low_cmd_write_thread_;
    DataBuffer<unitree_hg::msg::dds_::LowCmd_> low_cmd_buffer_;
    DataBuffer<unitree_hg::msg::dds_::LowState_> low_state_buffer_;
    DataBuffer<JoystickData> joystick_buffer_;

    unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
        lowcmd_publisher_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
        lowstate_subscriber_;

    std::atomic<std::int64_t> last_low_state_ns_{0};
    std::uint16_t last_joystick_buttons_{0};
    struct termios original_termios_ {};
    int original_stdin_flags_{-1};
    bool terminal_initialized_{false};
};

} // namespace deploy_real

#endif
