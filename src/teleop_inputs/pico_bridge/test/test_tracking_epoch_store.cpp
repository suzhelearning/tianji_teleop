#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include "pico_bridge/tracking_epoch_store.hpp"

namespace
{
class TrackingEpochStoreTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    root_ = std::filesystem::temp_directory_path() /
      ("pico_tracking_epoch_test_" + std::to_string(::getpid()));
    std::filesystem::remove_all(root_);
  }

  void TearDown() override
  {
    std::filesystem::remove_all(root_);
  }

  std::filesystem::path root_;
};
}  // namespace

TEST_F(TrackingEpochStoreTest, ReservesMonotonicEpochAcrossRestarts)
{
  const auto state = root_ / "nested" / "tracking_epoch";
  EXPECT_EQ(pico_bridge::reserve_tracking_epoch(state), 1U);
  EXPECT_EQ(pico_bridge::reserve_tracking_epoch(state), 2U);
  EXPECT_EQ(pico_bridge::reserve_tracking_epoch(state), 3U);
  ASSERT_TRUE(std::filesystem::exists(state));
  std::ifstream input(state);
  std::string persisted;
  input >> persisted;
  EXPECT_EQ(persisted, "3");
}

TEST_F(TrackingEpochStoreTest, RejectsMalformedStateInsteadOfResettingToOne)
{
  std::filesystem::create_directories(root_);
  const auto state = root_ / "tracking_epoch";
  {
    std::ofstream output(state);
    output << "not-a-number\n";
  }
  EXPECT_THROW(pico_bridge::reserve_tracking_epoch(state), std::runtime_error);
}

TEST_F(TrackingEpochStoreTest, RejectsTrailingGarbage)
{
  std::filesystem::create_directories(root_);
  const auto state = root_ / "tracking_epoch";
  {
    std::ofstream output(state);
    output << "5 trailing-garbage\n";
  }
  EXPECT_THROW(pico_bridge::reserve_tracking_epoch(state), std::runtime_error);
}

TEST_F(TrackingEpochStoreTest, ConcurrentProcessesReserveUniqueEpochs)
{
  constexpr int kProcessCount = 24;
  std::filesystem::create_directories(root_);
  const auto state = root_ / "tracking_epoch";
  int barrier[2];
  ASSERT_EQ(::pipe(barrier), 0);
  std::vector<pid_t> children;
  for (int index = 0; index < kProcessCount; ++index) {
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
      ::close(barrier[1]);
      char token = 0;
      if (::read(barrier[0], &token, 1) != 1) {
        _exit(10);
      }
      try {
        const auto epoch = pico_bridge::reserve_tracking_epoch(state);
        std::ofstream output(root_ / ("result_" + std::to_string(index)));
        output << epoch << '\n';
        output.flush();
        _exit(output ? 0 : 11);
      } catch (...) {
        _exit(12);
      }
    }
    children.push_back(child);
  }
  ::close(barrier[0]);
  for (int index = 0; index < kProcessCount; ++index) {
    ASSERT_EQ(::write(barrier[1], "x", 1), 1);
  }
  ::close(barrier[1]);

  for (const pid_t child : children) {
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status));
    EXPECT_EQ(WEXITSTATUS(status), 0);
  }

  std::set<std::uint64_t> epochs;
  for (int index = 0; index < kProcessCount; ++index) {
    std::ifstream input(root_ / ("result_" + std::to_string(index)));
    std::uint64_t epoch = 0;
    input >> epoch;
    ASSERT_TRUE(input);
    epochs.insert(epoch);
  }
  EXPECT_EQ(epochs.size(), static_cast<std::size_t>(kProcessCount));
  EXPECT_EQ(*epochs.begin(), 1U);
  EXPECT_EQ(*epochs.rbegin(), static_cast<std::uint64_t>(kProcessCount));
}
