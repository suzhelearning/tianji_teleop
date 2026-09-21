#pragma once

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace odin {
namespace sdk {

/**
 * @brief Transfer result structure
 */
struct TransferResult {
  bool success = false;
  std::string error_msg;
  size_t bytes_transferred = 0;

  TransferResult() = default;
  TransferResult(bool s, const std::string& msg, size_t bytes)
      : success(s), error_msg(msg), bytes_transferred(bytes) {}
};

/**
 * @brief Progress callback type
 * @param current  Current bytes transferred
 * @param total    Total bytes to transfer (0 if unknown)
 */
using FileTransferProgressCallback = std::function<void(size_t current, size_t total)>;

/**
 * @brief File info structure (for GetFileInfo)
 */
struct RemoteFileInfo {
  std::string name;
  size_t size = 0;
  std::string checksum;
  bool exists = false;
};

/**
 * @brief Abstract file transfer interface
 *
 * This interface abstracts file transfer operations, allowing different
 * implementations (HTTPS, UDP+custom protocol, etc.) to be used interchangeably.
 */
class IFileTransfer {
 public:
  virtual ~IFileTransfer() = default;

  // ========== Connection Management ==========

  /**
   * @brief Connect to remote target
   * @param target  Target address (IP:Port for UDP, URL for HTTPS)
   * @return true on success
   */
  virtual bool Connect(const std::string& target) = 0;

  /**
   * @brief Disconnect from remote target
   */
  virtual void Disconnect() = 0;

  /**
   * @brief Check if connected
   */
  virtual bool IsConnected() const = 0;

  // ========== File Transfer Operations ==========

  /**
   * @brief Download file from remote
   * @param remote_path  Remote file path or name
   * @param local_path   Local file path to save
   * @param cb           Progress callback (optional)
   * @return Transfer result
   */
  virtual TransferResult Download(const std::string& remote_path, const std::string& local_path,
                                  FileTransferProgressCallback cb = nullptr) = 0;

  /**
   * @brief Upload file to remote
   * @param local_path   Local file path to upload
   * @param remote_path  Remote file path or name
   * @param cb           Progress callback (optional)
   * @return Transfer result
   */
  virtual TransferResult Upload(const std::string& local_path, const std::string& remote_path,
                                FileTransferProgressCallback cb = nullptr) = 0;

  // ========== Optional Extensions (default implementations) ==========

  /**
   * @brief Check if resume (continue from last broken transfer) is supported
   */
  virtual bool SupportsResume() const { return false; }

  /**
   * @brief Resume download from offset
   */
  virtual TransferResult ResumeDownload(const std::string& remote_path,
                                        const std::string& local_path, size_t offset,
                                        FileTransferProgressCallback cb = nullptr) {
    (void)remote_path;
    (void)local_path;
    (void)offset;
    (void)cb;
    return TransferResult{false, "Resume not supported", 0};
  }

  /**
   * @brief Get remote file info (size, checksum, etc.)
   */
  virtual bool GetFileInfo(const std::string& remote_path, RemoteFileInfo& info) {
    (void)remote_path;
    (void)info;
    return false;
  }

  // ========== Metadata ==========

  /**
   * @brief Get transfer implementation name
   * @return Name string (e.g., "HTTPS", "UDP-OTA")
   */
  virtual const char* GetTransferName() const = 0;
};

/**
 * @brief Factory function type for creating file transfers
 */
using FileTransferFactory = std::function<std::unique_ptr<IFileTransfer>()>;

}  // namespace sdk
}  // namespace odin
