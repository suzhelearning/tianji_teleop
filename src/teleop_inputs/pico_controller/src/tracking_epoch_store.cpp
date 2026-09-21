#include "pico_bridge/tracking_epoch_store.hpp"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/file.h>
#include <unistd.h>

namespace pico_bridge
{
namespace
{
class FileDescriptor
{
public:
  explicit FileDescriptor(int value) : value_(value) {}
  ~FileDescriptor()
  {
    if (value_ >= 0) {
      ::close(value_);
    }
  }
  FileDescriptor(const FileDescriptor &) = delete;
  FileDescriptor & operator=(const FileDescriptor &) = delete;
  int get() const {return value_;}

private:
  int value_;
};

std::runtime_error io_error(const std::string & prefix, const std::filesystem::path & path)
{
  return std::runtime_error(prefix + ":" + path.string() + ":" + std::strerror(errno));
}

std::uint64_t read_epoch(const std::filesystem::path & path)
{
  if (!std::filesystem::exists(path)) {
    return 0;
  }
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("tracking_epoch_state_read_failed:" + path.string());
  }
  std::string value;
  input >> value;
  if (value.empty() || value.find_first_not_of("0123456789") != std::string::npos) {
    throw std::runtime_error("tracking_epoch_state_invalid:" + path.string());
  }
  std::string trailing;
  if (input >> trailing) {
    throw std::runtime_error("tracking_epoch_state_invalid:" + path.string());
  }
  try {
    const auto parsed = std::stoull(value);
    if (parsed == 0) {
      throw std::runtime_error("tracking_epoch_state_invalid:" + path.string());
    }
    return static_cast<std::uint64_t>(parsed);
  } catch (const std::exception &) {
    throw std::runtime_error("tracking_epoch_state_invalid:" + path.string());
  }
}
}  // namespace

std::uint64_t reserve_tracking_epoch(const std::filesystem::path & state_file)
{
  if (state_file.empty()) {
    throw std::invalid_argument("tracking_epoch_state_file is empty");
  }
  const auto parent = state_file.parent_path();
  if (!parent.empty()) {
    std::error_code error;
    std::filesystem::create_directories(parent, error);
    if (error) {
      throw std::runtime_error("tracking_epoch_state_directory_failed:" + error.message());
    }
  }
  const auto lock_path = state_file.string() + ".lock";
  FileDescriptor lock_fd(::open(lock_path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC, 0600));
  if (lock_fd.get() < 0) {
    throw io_error("tracking_epoch_lock_open_failed", lock_path);
  }
  if (::flock(lock_fd.get(), LOCK_EX) != 0) {
    throw io_error("tracking_epoch_lock_failed", lock_path);
  }
  const auto previous = read_epoch(state_file);
  if (previous == std::numeric_limits<std::uint64_t>::max()) {
    throw std::runtime_error("tracking_epoch_exhausted");
  }
  const auto next = previous + 1;
  const auto temporary = state_file.string() + ".tmp." +
    std::to_string(static_cast<long long>(::getpid()));
  const std::string contents = std::to_string(next) + '\n';
  {
    FileDescriptor output_fd(
      ::open(temporary.c_str(), O_CREAT | O_WRONLY | O_TRUNC | O_CLOEXEC, 0600));
    if (output_fd.get() < 0) {
      throw io_error("tracking_epoch_state_write_failed", temporary);
    }
    std::size_t written = 0;
    while (written < contents.size()) {
      const auto count = ::write(
        output_fd.get(), contents.data() + written, contents.size() - written);
      if (count <= 0) {
        std::filesystem::remove(temporary);
        throw io_error("tracking_epoch_state_write_failed", temporary);
      }
      written += static_cast<std::size_t>(count);
    }
    if (::fsync(output_fd.get()) != 0) {
      std::filesystem::remove(temporary);
      throw io_error("tracking_epoch_state_sync_failed", temporary);
    }
  }
  std::error_code error;
  std::filesystem::rename(temporary, state_file, error);
  if (error) {
    std::filesystem::remove(temporary);
    throw std::runtime_error("tracking_epoch_state_commit_failed:" + error.message());
  }
  const auto sync_directory = parent.empty() ? std::filesystem::path(".") : parent;
  FileDescriptor directory_fd(
    ::open(sync_directory.c_str(), O_RDONLY | O_DIRECTORY | O_CLOEXEC));
  if (directory_fd.get() < 0 || ::fsync(directory_fd.get()) != 0) {
    throw io_error("tracking_epoch_state_directory_sync_failed", sync_directory);
  }
  return next;
}

}  // namespace pico_bridge
