#include <cstdio>
#include <cstdlib>

#include "communication/protocol.hpp"
#include "sensors/hwt101.hpp"

#define CHECK(condition)                                                \
  do                                                                    \
  {                                                                     \
    if (!(condition))                                                   \
    {                                                                   \
      std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); \
      std::exit(1);                                                     \
    }                                                                   \
  } while (false)

static void Near(float actual, float expected)
{
  CHECK(std::fabs(actual - expected) < 0.00001F);
}

static std::array<uint8_t, 11> ImuFrame(int16_t yaw)
{
  std::array<uint8_t, 11> frame{0x55, 0x53};
  frame[6] = static_cast<uint8_t>(yaw);
  frame[7] = static_cast<uint8_t>(static_cast<uint16_t>(yaw) >> 8);
  for (size_t i = 0; i < 10; ++i) frame[10] += frame[i];
  return frame;
}

static void TestImu()
{
  Ops9::Hwt101 imu;
  CHECK(!imu.Feed(0x42));
  const auto positive = ImuFrame(16384);
  for (size_t i = 0; i < 10; ++i) CHECK(!imu.Feed(positive[i]));
  CHECK(imu.Feed(positive[10]));
  Near(imu.Yaw(), Ops9::kPi / 2);
  auto corrupt = positive;
  corrupt[10] ^= 1;
  for (const auto byte : corrupt) CHECK(!imu.Feed(byte));
  CHECK(imu.ValidFrames() == 1);
  // A truncated packet followed immediately by a full packet must recover.
  imu.Feed(0x55);
  imu.Feed(0x53);
  imu.Feed(0x01);
  for (const auto byte : ImuFrame(-16384)) imu.Feed(byte);
  CHECK(imu.ValidFrames() == 2);
  Near(imu.Yaw(), -Ops9::kPi / 2);
  auto wrong_type = ImuFrame(0);
  wrong_type[1] = 0x52;
  --wrong_type[10];
  for (const auto byte : wrong_type) CHECK(!imu.Feed(byte));
  for (const auto byte : ImuFrame(-32768)) imu.Feed(byte);
  Near(imu.Yaw(), -Ops9::kPi);
  CHECK(imu.ValidFrames() == 3);
}

static void TestOdometry()
{
  const float circumference = 2 * Ops9::kPi * 0.029F;
  constexpr float expected_x[] = {-2, 1, 2, -1};
  constexpr float expected_y[] = {1, 2, -1, -2};
  for (uint8_t direction = 0; direction < 4; ++direction)
  {
    Ops9::Odometry odometry(0.029F, 1024);
    odometry.Integrate(1024, 2048, direction);
    Near(odometry.GetPose().x, expected_x[direction] * circumference);
    Near(odometry.GetPose().y, expected_y[direction] * circumference);
  }
  Ops9::Odometry rotated(0.029F, 1024);
  rotated.UpdateYaw(Ops9::kPi / 2);
  rotated.Integrate(0, 1024, 2);
  Near(rotated.GetPose().x, 0);
  Near(rotated.GetPose().y, circumference);
  rotated.Integrate(0, -1024, 2);
  Near(rotated.GetPose().y, 0);
  Ops9::Odometry wrap(0.029F, 1024);
  wrap.UpdateYaw(3.1F);
  wrap.UpdateYaw(-3.1F);
  Near(wrap.GetPose().yaw, 2 * Ops9::kPi - 3.1F);
  wrap.UpdateYaw(3.1F);
  Near(wrap.GetPose().yaw, 3.1F);
  // Pulse integration is independent of scheduling period.
  Ops9::Odometry batch(0.029F, 1024), split(0.029F, 1024);
  batch.Integrate(10, -20, 2);
  for (int i = 0; i < 10; ++i) split.Integrate(1, -2, 2);
  Near(batch.GetPose().x, split.GetPose().x);
  Near(batch.GetPose().y, split.GetPose().y);
}

static void TestProtocol()
{
  // Golden bytes calculated with the original project's CRC8 table.
  constexpr std::array<uint8_t, 14> golden{0x5C, 0,    0,    0x80, 0x3F, 0,    0,
                                           0,    0xC0, 0xDB, 0x0F, 0x49, 0x40, 0xC1};
  CHECK(Ops9::EncodePose({1, -2, Ops9::kPi}) == golden);
  CHECK(Ops9::EncodePose({})[13] == 0xC9);
  Ops9::CommandParser parser;
  uint8_t command = 0;
  CHECK(!parser.Feed(0x30, command));
  CHECK(!parser.Feed(0xC5, command));
  CHECK(parser.Feed(0x22, command) && command == 0x22);
  for (uint8_t direction = 0x30; direction <= 0x33; ++direction)
  {
    CHECK(!parser.Feed(0xC5, command));
    CHECK(parser.Feed(direction, command) && command == direction);
  }
  CHECK(!parser.Feed(0xC5, command));
  CHECK(!parser.Feed(0xC5, command));
  CHECK(parser.Feed(0x31, command) && command == 0x31);
  CHECK(!parser.Feed(0xC5, command));
  CHECK(!parser.Feed(0xFF, command));
  CHECK(!parser.Feed(0x32, command));
}

int main()
{
  TestImu();
  TestOdometry();
  TestProtocol();
  std::puts("PASS: IMU stream recovery, yaw unwrap, odometry and wire protocol");
}
