#pragma once

#include <string>
#include <vector>

#include "aibomech_agrobot_hardware/protocol.hpp"
#include "aibomech_agrobot_hardware/serial_port.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace aibomech_agrobot_hardware
{

// ros2_control system for the AgroBot motor-controller board.
//
// Hardware parameters: serial_port, baud_rate, timeout_ms, max_missed_frames,
//                      command_rate_hz (default 50)
// Joint parameters:    channel, direction (+1/-1), offset
//   actuator_value = direction * joint_value + offset
class AgrobotSystemHardware : public hardware_interface::SystemInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointConfig
  {
    std::size_t channel = 0;
    double direction = 1.0;
    double offset = 0.0;
  };

  // Reads frames until one arrives or `timeout_ms` elapses.
  bool wait_for_state(int timeout_ms);
  bool poll_state();
  void report_status_changes(uint32_t status);

  SerialPort port_;
  std::string device_;
  int baud_rate_ = 115200;
  int timeout_ms_ = 100;
  int max_missed_frames_ = 10;
  double command_period_s_ = 0.02;

  std::size_t channels_ = 0;
  std::vector<JointConfig> joints_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  StateFrame last_frame_;
  bool have_frame_ = false;
  uint32_t last_status_ = 0;
  int missed_frames_ = 0;
  double silence_s_ = 0.0;
  rclcpp::Time last_command_time_{0, 0, RCL_STEADY_TIME};
  rclcpp::Clock clock_{RCL_STEADY_TIME};
};

}  // namespace aibomech_agrobot_hardware
