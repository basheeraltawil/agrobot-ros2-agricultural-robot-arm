#include <gtest/gtest.h>

#include "aibomech_agrobot_hardware/protocol.hpp"

using namespace aibomech_agrobot_hardware;

TEST(Protocol, CommandFrameHasValidChecksum)
{
  const std::string frame = encode_command({0.1, -0.25, 0.01});
  ASSERT_EQ(frame.front(), '$');
  ASSERT_EQ(frame.back(), '\n');
  const auto star = frame.rfind('*');
  const std::string payload = frame.substr(1, star - 1);
  EXPECT_EQ(payload, "C,0.1000,-0.2500,0.0100");
  EXPECT_EQ(std::stoi(frame.substr(star + 1, 2), nullptr, 16), checksum(payload));
}

TEST(Protocol, DecodesStateFrame)
{
  const std::string line = make_frame("S,42,5,0.1,0.2,-0.3,0.01,0.0,-0.02");
  StateFrame frame;
  ASSERT_TRUE(decode_state(line.substr(0, line.size() - 1), 3, frame));
  EXPECT_EQ(frame.sequence, 42u);
  EXPECT_EQ(frame.status, 5u);
  EXPECT_DOUBLE_EQ(frame.positions[2], -0.3);
  EXPECT_DOUBLE_EQ(frame.velocities[2], -0.02);
}

TEST(Protocol, RejectsCorruptedFrames)
{
  std::string line = make_frame("S,1,0,0.1,0.2,0.0,0.0");
  line.pop_back();
  StateFrame frame;
  EXPECT_TRUE(decode_state(line, 2, frame));
  EXPECT_FALSE(decode_state(line, 3, frame));              // channel count mismatch
  std::string flipped = line;
  flipped[6] = '9';
  EXPECT_FALSE(decode_state(flipped, 2, frame));           // checksum mismatch
  EXPECT_FALSE(decode_state("S,1,0,0.1,0.2,0.0,0.0", 2, frame));  // no framing
  EXPECT_FALSE(decode_state(make_frame("C,0.1,0.2"), 2, frame));
}
