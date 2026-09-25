// Non-blocking POSIX serial port for the link to the motor-controller board.

#pragma once

#include <string>
#include <vector>

namespace aibomech_agrobot_hardware
{

// Minimal non-blocking POSIX serial port (8N1, raw mode).
class SerialPort
{
public:
  ~SerialPort();

  void open(const std::string & device, int baud_rate);  // throws std::runtime_error
  void close();
  bool is_open() const {return fd_ >= 0;}

  bool write(const std::string & data);

  // Appends complete lines received so far (without '\n') to `lines`.
  void read_lines(std::vector<std::string> & lines);

private:
  int fd_ = -1;
  std::string buffer_;
};

}  // namespace aibomech_agrobot_hardware
