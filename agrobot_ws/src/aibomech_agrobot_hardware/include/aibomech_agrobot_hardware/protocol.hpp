// Line protocol between the ROS 2 host and the AgroBot motor-controller board.
//
// Every frame is ASCII, NMEA style:  $<TYPE>,<field>,...*<XOR checksum, 2 hex>\n
// The checksum is the XOR of all bytes between '$' and '*'.
//
// Host -> board
//   $C,<p0>,...,<pN-1>*hh   position setpoints in joint units of the actuator (rad or m)
//   $E,1*hh / $E,0*hh       enable / disable the drives (torque on / off)
//   $P*hh                   ping, answered with a state frame
// Board -> host (50 Hz)
//   $S,<seq>,<status>,<p0>,...,<pN-1>,<v0>,...,<vN-1>*hh
//   status bits: see StatusBits
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace aibomech_agrobot_hardware
{

enum StatusBits : uint32_t
{
  STATUS_ENABLED = 1u << 0,
  STATUS_ESTOP = 1u << 1,          // e-stop chain open, drives powered down
  STATUS_WATCHDOG = 1u << 2,       // no command from host in time, holding position
  STATUS_FAULT = 1u << 3,          // actuator fault (overload, overheating, lost bus servo)
  STATUS_RAIL_NOT_HOMED = 1u << 4, // rail axis has not found its home switch yet
};

struct StateFrame
{
  uint32_t sequence = 0;
  uint32_t status = 0;
  std::vector<double> positions;
  std::vector<double> velocities;
};

uint8_t checksum(const std::string & payload);

// Wraps a payload ("C,0.1,0.2") into a complete frame including '\n'.
std::string make_frame(const std::string & payload);

std::string encode_command(const std::vector<double> & positions);
std::string encode_enable(bool enable);
std::string encode_ping();

// Parses one line without the trailing newline. Returns false for malformed
// frames, wrong checksums, other frame types or a channel count mismatch.
bool decode_state(const std::string & line, std::size_t channels, StateFrame & out);

}  // namespace aibomech_agrobot_hardware
