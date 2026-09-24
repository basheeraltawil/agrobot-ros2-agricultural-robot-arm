#include "aibomech_agrobot_hardware/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace aibomech_agrobot_hardware
{

namespace
{
speed_t to_speed(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    case 1000000: return B1000000;
    default: throw std::runtime_error("unsupported baud rate " + std::to_string(baud_rate));
  }
}
}  // namespace

SerialPort::~SerialPort() {close();}

void SerialPort::open(const std::string & device, int baud_rate)
{
  close();
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    throw std::runtime_error("cannot open " + device + ": " + std::strerror(errno));
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    const std::string err = std::strerror(errno);
    close();
    throw std::runtime_error("tcgetattr failed on " + device + ": " + err);
  }
  cfmakeraw(&tty);
  const speed_t speed = to_speed(baud_rate);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    const std::string err = std::strerror(errno);
    close();
    throw std::runtime_error("tcsetattr failed on " + device + ": " + err);
  }
  tcflush(fd_, TCIOFLUSH);
  buffer_.clear();
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::string & data)
{
  if (fd_ < 0) {
    return false;
  }
  std::size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EINTR) {
        continue;
      }
      return false;
    }
    sent += static_cast<std::size_t>(n);
  }
  return true;
}

void SerialPort::read_lines(std::vector<std::string> & lines)
{
  if (fd_ < 0) {
    return;
  }
  char chunk[512];
  while (true) {
    const ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n <= 0) {
      break;
    }
    buffer_.append(chunk, static_cast<std::size_t>(n));
  }
  std::size_t start = 0;
  for (std::size_t i = 0; i < buffer_.size(); ++i) {
    if (buffer_[i] == '\n') {
      std::string line = buffer_.substr(start, i - start);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      lines.push_back(line);
      start = i + 1;
    }
  }
  buffer_.erase(0, start);
  if (buffer_.size() > 4096) {  // garbage without newlines, resynchronise
    buffer_.clear();
  }
}

}  // namespace aibomech_agrobot_hardware
