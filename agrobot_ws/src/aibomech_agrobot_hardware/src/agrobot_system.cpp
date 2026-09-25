// ros2_control SystemInterface for the real AgroBot: sends joint setpoints to the
// motor-controller board over serial and reads back positions, velocities and status.
// Protocol: protocol.hpp. Commissioning: docs/real_robot.md.

#include "aibomech_agrobot_hardware/agrobot_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace aibomech_agrobot_hardware
{

namespace
{
const rclcpp::Logger kLogger = rclcpp::get_logger("AgrobotSystemHardware");

std::string param(const std::unordered_map<std::string, std::string> & params,
                  const std::string & key, const std::string & fallback)
{
  const auto it = params.find(key);
  return it == params.end() ? fallback : it->second;
}
}  // namespace

AgrobotSystemHardware::CallbackReturn AgrobotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  try {
    device_ = param(info_.hardware_parameters, "serial_port", "/dev/ttyACM0");
    baud_rate_ = std::stoi(param(info_.hardware_parameters, "baud_rate", "115200"));
    timeout_ms_ = std::stoi(param(info_.hardware_parameters, "timeout_ms", "100"));
    max_missed_frames_ = std::stoi(param(info_.hardware_parameters, "max_missed_frames", "10"));
    command_period_s_ = 1.0 / std::stod(param(info_.hardware_parameters, "command_rate_hz", "50"));

    std::size_t max_channel = 0;
    for (const auto & joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(kLogger, "Joint '%s' needs exactly one position command interface.",
          joint.name.c_str());
        return CallbackReturn::ERROR;
      }
      JointConfig cfg;
      cfg.channel = std::stoul(param(joint.parameters, "channel", std::to_string(joints_.size())));
      cfg.direction = std::stod(param(joint.parameters, "direction", "1")) < 0.0 ? -1.0 : 1.0;
      cfg.offset = std::stod(param(joint.parameters, "offset", "0"));
      max_channel = std::max(max_channel, cfg.channel);
      joints_.push_back(cfg);
    }
    channels_ = std::stoul(param(info_.hardware_parameters, "channels",
        std::to_string(max_channel + 1)));
  } catch (const std::exception & e) {
    RCLCPP_FATAL(kLogger, "Invalid hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  const double nan = std::numeric_limits<double>::quiet_NaN();
  hw_positions_.assign(info_.joints.size(), nan);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  hw_commands_.assign(info_.joints.size(), nan);
  return CallbackReturn::SUCCESS;
}

AgrobotSystemHardware::CallbackReturn AgrobotSystemHardware::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    port_.open(device_, baud_rate_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(kLogger, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  // Boards that reset on connect (Arduino) need a moment before they talk.
  port_.write(encode_ping());
  if (!wait_for_state(3000)) {
    RCLCPP_ERROR(kLogger, "No state frame from the motor controller on %s. "
      "Is the firmware running and the baud rate %d?", device_.c_str(), baud_rate_);
    port_.close();
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(kLogger, "Connected to motor controller on %s (%zu channels).",
    device_.c_str(), channels_);
  return CallbackReturn::SUCCESS;
}

AgrobotSystemHardware::CallbackReturn AgrobotSystemHardware::on_cleanup(const rclcpp_lifecycle::State &)
{
  port_.write(encode_enable(false));
  port_.close();
  return CallbackReturn::SUCCESS;
}

AgrobotSystemHardware::CallbackReturn AgrobotSystemHardware::on_activate(const rclcpp_lifecycle::State &)
{
  if (!wait_for_state(timeout_ms_ * 5)) {
    RCLCPP_ERROR(kLogger, "Motor controller stopped responding.");
    return CallbackReturn::ERROR;
  }
  if (last_frame_.status & STATUS_ESTOP) {
    RCLCPP_ERROR(kLogger, "Emergency stop is active. Release it before activating the robot.");
    return CallbackReturn::ERROR;
  }
  // Start from where the robot is, so enabling the drives never makes it jump.
  hw_commands_ = hw_positions_;
  port_.write(encode_enable(true));
  missed_frames_ = 0;
  last_command_time_ = clock_.now();
  RCLCPP_INFO(kLogger, "Drives enabled.");
  return CallbackReturn::SUCCESS;
}

AgrobotSystemHardware::CallbackReturn AgrobotSystemHardware::on_deactivate(const rclcpp_lifecycle::State &)
{
  // The arm has no brakes, so switching torque off here would let joint_3 drop.
  // The board keeps holding: once commands stop, its watchdog freezes the
  // setpoint. Torque goes off in on_cleanup() or with the e-stop.
  RCLCPP_INFO(kLogger, "Deactivated, drives hold their position.");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AgrobotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> AgrobotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]);
  }
  return interfaces;
}

bool AgrobotSystemHardware::poll_state()
{
  std::vector<std::string> lines;
  port_.read_lines(lines);
  bool got = false;
  StateFrame frame;
  for (const auto & line : lines) {
    if (decode_state(line, channels_, frame)) {
      last_frame_ = frame;
      got = true;
    }
  }
  if (!got) {
    return false;
  }
  have_frame_ = true;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & cfg = joints_[i];
    hw_positions_[i] = (last_frame_.positions[cfg.channel] - cfg.offset) * cfg.direction;
    hw_velocities_[i] = last_frame_.velocities[cfg.channel] * cfg.direction;
  }
  report_status_changes(last_frame_.status);
  return true;
}

bool AgrobotSystemHardware::wait_for_state(int timeout_ms)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while (std::chrono::steady_clock::now() < deadline) {
    if (poll_state()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return false;
}

void AgrobotSystemHardware::report_status_changes(uint32_t status)
{
  const uint32_t rising = status & ~last_status_;
  const uint32_t falling = ~status & last_status_;
  if (rising & STATUS_ESTOP) {
    RCLCPP_ERROR(kLogger, "EMERGENCY STOP pressed - drives are off. After releasing it, "
      "re-activate the hardware (see docs/real_robot.md, 'Recovering from an e-stop').");
  }
  if (falling & STATUS_ESTOP) {
    RCLCPP_WARN(kLogger, "Emergency stop released. Drives stay off until the hardware is re-activated.");
  }
  if (rising & STATUS_WATCHDOG) {
    RCLCPP_WARN(kLogger, "Motor controller watchdog tripped: commands arrived too late.");
  }
  if (rising & STATUS_FAULT) {
    RCLCPP_ERROR(kLogger, "Actuator fault reported by the motor controller.");
  }
  last_status_ = status;
}

hardware_interface::return_type AgrobotSystemHardware::read(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (poll_state()) {
    missed_frames_ = 0;
    silence_s_ = 0.0;
    return hardware_interface::return_type::OK;
  }
  // The board streams at 50 Hz while the control loop runs faster, so a
  // missing frame in one cycle is normal. Count elapsed frame periods instead.
  silence_s_ += period.seconds();
  if (silence_s_ >= 0.02) {
    silence_s_ = 0.0;
    if (++missed_frames_ > max_missed_frames_) {
      RCLCPP_ERROR_THROTTLE(kLogger, clock_, 1000,
        "Lost communication with the motor controller (%d frames missed).", missed_frames_);
      return hardware_interface::return_type::ERROR;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AgrobotSystemHardware::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!have_frame_) {
    return hardware_interface::return_type::OK;
  }
  const auto now = clock_.now();
  if ((now - last_command_time_).seconds() < command_period_s_) {
    return hardware_interface::return_type::OK;
  }
  last_command_time_ = now;

  // Channels without a joint keep the position the board reports.
  std::vector<double> setpoints = last_frame_.positions;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    if (std::isfinite(hw_commands_[i])) {
      setpoints[joints_[i].channel] = hw_commands_[i] * joints_[i].direction + joints_[i].offset;
    }
  }
  if (!port_.write(encode_command(setpoints))) {
    RCLCPP_ERROR_THROTTLE(kLogger, clock_, 1000, "Serial write failed.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace aibomech_agrobot_hardware

PLUGINLIB_EXPORT_CLASS(aibomech_agrobot_hardware::AgrobotSystemHardware, hardware_interface::SystemInterface)
