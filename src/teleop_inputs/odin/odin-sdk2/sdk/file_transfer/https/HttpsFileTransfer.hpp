#pragma once

#include "../IFileTransfer/IFileTransfer.hpp"
#include <string>

namespace odin {
namespace sdk {

/**
 * @brief HTTPS implementation of IFileTransfer
 *
 * Uses secure_transfer module (mongoose + OpenSSL) for HTTPS file transfer.
 */
class HttpsFileTransfer : public IFileTransfer {
 public:
  HttpsFileTransfer();
  ~HttpsFileTransfer() override;

  // Connection Management
  bool Connect(const std::string& target) override;
  void Disconnect() override;
  bool IsConnected() const override;

  // File Transfer Operations
  TransferResult Download(const std::string& remote_path, const std::string& local_path,
                          FileTransferProgressCallback cb = nullptr) override;

  TransferResult Upload(const std::string& local_path, const std::string& remote_path,
                        FileTransferProgressCallback cb = nullptr) override;

  // Metadata
  const char* GetTransferName() const override { return "HTTPS"; }

  // Configuration
  void SetTimeout(int timeout_ms);
  void SetVerifySignature(bool enable);

 private:
  std::string base_url_;
  bool connected_ = false;
  bool use_http_ = false; /* true when base_url_ is http:// (new API) */
  int timeout_ms_ = 30000;
  bool verify_signature_ = true;
};

}  // namespace sdk
}  // namespace odin
