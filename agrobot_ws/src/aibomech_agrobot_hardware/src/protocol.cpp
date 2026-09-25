// Encoding and decoding of the host <-> board line protocol (see protocol.hpp).

#include "aibomech_agrobot_hardware/protocol.hpp"

#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace aibomech_agrobot_hardware
{

uint8_t checksum(const std::string & payload)
{
  uint8_t sum = 0;
  for (char c : payload) {
    sum ^= static_cast<uint8_t>(c);
  }
  return sum;
}

std::string make_frame(const std::string & payload)
{
  char tail[8];
  std::snprintf(tail, sizeof(tail), "*%02X\n", checksum(payload));
  return "$" + payload + tail;
}

std::string encode_command(const std::vector<double> & positions)
{
  std::string payload = "C";
  char buf[32];
  for (double p : positions) {
    std::snprintf(buf, sizeof(buf), ",%.4f", p);
    payload += buf;
  }
  return make_frame(payload);
}

std::string encode_enable(bool enable) {return make_frame(enable ? "E,1" : "E,0");}

std::string encode_ping() {return make_frame("P");}

bool decode_state(const std::string & line, std::size_t channels, StateFrame & out)
{
  const auto star = line.rfind('*');
  if (line.size() < 4 || line[0] != '$' || star == std::string::npos || star + 3 > line.size()) {
    return false;
  }
  const std::string payload = line.substr(1, star - 1);
  char * end = nullptr;
  const std::string hex = line.substr(star + 1, 2);
  const long received = std::strtol(hex.c_str(), &end, 16);
  if (end != hex.c_str() + 2 || received != checksum(payload)) {
    return false;
  }

  std::vector<std::string> fields;
  std::stringstream ss(payload);
  std::string field;
  while (std::getline(ss, field, ',')) {
    fields.push_back(field);
  }
  if (fields.size() != 3 + 2 * channels || fields[0] != "S") {
    return false;
  }

  StateFrame frame;
  try {
    frame.sequence = static_cast<uint32_t>(std::stoul(fields[1]));
    frame.status = static_cast<uint32_t>(std::stoul(fields[2]));
    for (std::size_t i = 0; i < channels; ++i) {
      frame.positions.push_back(std::stod(fields[3 + i]));
      frame.velocities.push_back(std::stod(fields[3 + channels + i]));
    }
  } catch (const std::exception &) {
    return false;
  }
  out = frame;
  return true;
}

}  // namespace aibomech_agrobot_hardware
